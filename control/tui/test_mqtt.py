"""Pruebas de telemetria sin hardware ni dependencias MQTT obligatorias."""
import json
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from servos_tui import Control, Extras, Hardware, MQTTTraces, parse_args


class MQTTTests(unittest.TestCase):
    def setUp(self):
        with patch('sys.argv', ['servos_tui.py', '--simulate', '--channels', '0', '1', '--mqtt-host', 'localhost']):
            self.args = parse_args()
        self.hw = Hardware(self.args, [1500, 1505])
        self.ctrl = Control(self.hw, [1500, 1505], self.args)
        self.extras = Extras(self.args)
        self.client = Mock()
        self.traces = MQTTTraces(self.args, client=self.client)

    def messages(self, suffix):
        return [json.loads(c.args[1]) for c in self.client.publish.call_args_list
                if c.args[0].endswith('/' + suffix)]

    def connect(self):
        self.traces.on_connect(self.client, None, None, 0, None)

    def test_offline_drops_samples_and_reconnection_resumes(self):
        self.traces.sample(self.ctrl, self.extras)
        self.client.publish.assert_not_called()
        self.connect()
        self.traces.sample(self.ctrl, self.extras)
        self.traces.on_disconnect(self.client, None, None, 1, None)
        self.traces.sample(self.ctrl, self.extras)
        self.assertEqual(len(self.messages('telemetry')), 1)
        self.connect()
        self.traces.sample(self.ctrl, self.extras)
        self.assertEqual([m['seq'] for m in self.messages('telemetry')], [1, 2])

    def test_payload_units_and_sampled_events(self):
        self.connect()
        self.ctrl.arm(time.monotonic())
        self.ctrl.jog([0], .1, False, time.monotonic())
        self.ctrl.tick(time.monotonic())
        self.traces.sample(self.ctrl, self.extras)
        state = self.messages('telemetry')[0]
        self.assertTrue(state['simulate'])
        self.assertTrue(state['armed'])
        self.assertEqual(state['servos'][0]['command'], .1)
        self.assertGreater(state['servos'][0]['pwm_calculated_us'], 1500)
        self.assertEqual(state['servos'][1]['neutral_us'], 1505)
        self.assertEqual(state['schema'], 1)
        self.assertTrue(state['ts'].endswith('+00:00'))
        self.traces.sample(self.ctrl, self.extras)
        self.assertEqual(len(self.messages('events')), 1)
        self.ctrl.stop('Parada de prueba')
        self.traces.sample(self.ctrl, self.extras)
        self.assertFalse(self.messages('events')[-1]['armed'])
        self.assertEqual(len(self.messages('events')), 2)

    def test_stale_feedback_is_strict_json(self):
        self.hw.feedback[0] = SimpleNamespace(read=lambda now: (120, 30, math.inf))
        self.connect()
        self.traces.sample(self.ctrl, self.extras)
        servo = self.messages('telemetry')[0]['servos'][0]
        self.assertIsNone(servo['feedback_age_s'])
        self.assertIsNone(servo['rpm'])
        self.assertIsNone(servo['angle_deg'])
        self.assertFalse(servo['feedback_ok'])

    def test_retained_presence_and_clean_shutdown(self):
        self.assertEqual(json.loads(self.client.will_set.call_args.args[1])['state'], 'offline')
        self.assertTrue(self.client.will_set.call_args.kwargs['retain'])
        self.connect()
        self.assertEqual(self.messages('status')[-1]['state'], 'online')
        self.traces.close()
        self.assertEqual(self.messages('status')[-1]['state'], 'offline')
        self.client.disconnect.assert_called_once()
        self.client.loop_stop.assert_called_once()

    def test_refused_auth_does_not_mark_connected(self):
        self.traces.on_connect(self.client, None, None, 135, None)
        self.assertFalse(self.traces.connected.is_set())
        self.client.publish.assert_not_called()

    def test_password_from_environment(self):
        self.args.mqtt_username = 'robot'
        with patch.dict('os.environ', {'MQTT_PASSWORD': 'test-secret'}):
            MQTTTraces(self.args, client=self.client)
        self.client.username_pw_set.assert_called_once_with('robot', 'test-secret')

    def test_network_stall_does_not_hold_control_lock(self):
        self.connect()
        entered, release = threading.Event(), threading.Event()
        def slow_publish(*args, **kwargs):
            entered.set()
            release.wait(2)
        self.client.publish.side_effect = slow_publish
        worker = threading.Thread(target=self.traces.sample, args=(self.ctrl, self.extras))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            acquired = self.ctrl.lock.acquire(timeout=.2)
            self.assertTrue(acquired)
            if acquired:
                try:
                    self.ctrl.tick(time.monotonic())
                    self.ctrl.stop()
                finally:
                    self.ctrl.lock.release()
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_invalid_mqtt_arguments(self):
        for option, value in [('--mqtt-hz', 'nan'), ('--mqtt-hz', '0'),
                              ('--mqtt-port', '65536'), ('--mqtt-topic', 'bad/#'),
                              ('--mqtt-topic', '/')]:
            with self.subTest(option=option, value=value), patch('sys.argv', ['tui', option, value]):
                with patch('sys.stderr'), self.assertRaises(SystemExit):
                    parse_args()


if __name__ == '__main__':
    unittest.main()
