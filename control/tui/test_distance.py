"""Filtro, captura de ecos y telemetria sin conectar hardware."""
import json
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from servos_tui import Control, DistanceKalman, DistanceSensor, Extras, Hardware, MQTTTraces, parse_args


class KalmanTests(unittest.TestCase):
    def test_initial_value_and_update_match_sketch(self):
        f = DistanceKalman()
        self.assertEqual(f.update(50), (50, True))
        value, accepted = f.update(54)
        gain = 10.05 / 14.05
        self.assertAlmostEqual(value, 50 + gain * 4)
        self.assertAlmostEqual(f.p, 10.05 * (1 - gain))
        self.assertTrue(accepted)

    def test_isolated_outlier_does_not_pull_estimate(self):
        f = DistanceKalman()
        f.update(80)
        self.assertEqual(f.update(140), (80, False))
        self.assertEqual(f.rejections, 1)
        f.update(81)
        self.assertEqual(f.rejections, 0)
        self.assertLess(f.estimate, 82)

    def test_fifth_consecutive_jump_allows_adaptation(self):
        f = DistanceKalman()
        f.update(40)
        for _ in range(4):
            self.assertEqual(f.update(180), (40, False))
        value, accepted = f.update(180)
        self.assertTrue(accepted)
        self.assertGreater(value, 40)
        self.assertLess(value, 180)
        self.assertEqual(f.rejections, 0)

    def test_constant_distance_noise_is_reduced(self):
        f = DistanceKalman()
        for _ in range(100):
            f.update(98)
            f.update(102)
        self.assertLess(abs(f.estimate - 100), .3)

    def test_invalid_samples_do_not_corrupt_filter(self):
        f = DistanceKalman()
        f.update(100)
        for value in (0, 2, 400, 1000, math.inf, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                f.update(value)
        self.assertEqual(f.estimate, 100)
        self.assertEqual(f.p, 10)


class SensorTests(unittest.TestCase):
    def setUp(self):
        with patch('sys.argv', ['tui', '--simulate', '--channels', '0', '1', '--distance-backend', 'gpio']):
            self.args = parse_args()
        self.sensor = DistanceSensor(self.args)

    def test_echo_conversion_uses_nanosecond_ticks(self):
        gpio = Mock()
        gpio.gpio_read.return_value = 0
        self.sensor.lgpio, self.sensor.handle = gpio, 7
        def trigger(*args, **kwargs):
            tick = time.monotonic_ns() + 100_000
            self.sensor.edge(0, 14, 1, tick)
            self.sensor.edge(0, 14, 0, tick + 5_000_000)
        gpio.tx_pulse.side_effect = trigger
        self.assertAlmostEqual(self.sensor.measure(), 85.75)
        gpio.tx_pulse.assert_called_once_with(7, 13, 10, 10, pulse_cycles=1)
        self.assertFalse(self.sensor.waiting)

    def test_missing_echo_times_out_and_late_edges_are_ignored(self):
        gpio = Mock()
        gpio.gpio_read.return_value = 0
        self.sensor.lgpio, self.sensor.handle = gpio, 7
        with patch.object(self.sensor.echo_done, 'wait') as wait:
            self.assertIsNone(self.sensor.measure())
        wait.assert_called_once_with(.025)
        self.sensor.edge(0, 14, 1, time.monotonic_ns())
        self.assertIsNone(self.sensor.rise)

    def test_old_or_beyond_deadline_edges_do_not_complete_reading(self):
        self.sensor.waiting = True
        self.sensor.started_ns = 1_000_000_000
        for tick in (900_000_000, 1_030_000_000):
            self.sensor.edge(0, 14, 1, tick)
            self.sensor.edge(0, 14, 0, tick + 1000)
        self.assertIsNone(self.sensor.width_us)
        self.assertFalse(self.sensor.echo_done.is_set())

    def test_echo_stuck_high_does_not_trigger(self):
        gpio = Mock()
        gpio.gpio_read.return_value = 1
        self.sensor.lgpio, self.sensor.handle = gpio, 7
        self.assertIsNone(self.sensor.measure())
        gpio.tx_pulse.assert_not_called()

    def test_invalid_and_stale_readings_are_null_not_zero(self):
        now = time.monotonic()
        self.sensor.record(100, now)
        self.assertTrue(self.sensor.snapshot()['valid'])
        self.sensor.record(None, now + .1)
        state = self.sensor.snapshot()
        self.assertIsNone(state['raw_cm'])
        self.assertIsNone(state['kalman_cm'])
        self.assertFalse(state['valid'])
        self.sensor.record(80, now - 1)
        self.assertEqual(self.sensor.snapshot()['status'], 'LECTURA CADUCADA')

    def test_filter_reinitializes_after_signal_loss(self):
        now = time.monotonic()
        self.sensor.record(30, now - 1)
        self.sensor.record(200, now)
        self.assertEqual(self.sensor.snapshot()['kalman_cm'], 200)
        self.assertFalse(self.sensor.snapshot()['outlier_rejected'])

    def test_outlier_raw_and_filtered_are_both_exposed(self):
        now = time.monotonic()
        self.sensor.record(50, now)
        self.sensor.record(150, now + .01)
        state = self.sensor.snapshot()
        self.assertEqual(state['raw_cm'], 150)
        self.assertEqual(state['kalman_cm'], 50)
        self.assertTrue(state['outlier_rejected'])

    def test_simulation_thread_stops_without_gpio_import(self):
        with patch.dict('sys.modules', {'lgpio': None}):
            self.sensor.start()
            try:
                deadline = time.monotonic() + 1
                while not self.sensor.snapshot()['valid'] and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertTrue(self.sensor.snapshot()['valid'])
            finally:
                self.sensor.close()
        self.assertFalse(self.sensor.thread.is_alive())

    def test_sensor_initialization_failure_is_visible_and_releases_partial_claims(self):
        self.args.simulate = False
        gpio = Mock(BOTH_EDGES=3)
        gpio.gpiochip_open.return_value = 7
        gpio.gpio_claim_alert.side_effect = OSError('ECHO ocupado')
        with patch.dict('sys.modules', {'lgpio': gpio}):
            self.sensor.start()
            self.sensor.thread.join(1)
        self.assertFalse(self.sensor.thread.is_alive())
        self.assertIn('ECHO ocupado', self.sensor.snapshot()['status'])
        self.assertFalse(self.sensor.snapshot()['valid'])
        gpio.gpio_free.assert_called_once_with(7, 13)
        gpio.gpiochip_close.assert_called_once_with(7)

    def test_off_backend_never_starts_thread(self):
        self.args.distance_backend = 'off'
        sensor = DistanceSensor(self.args)
        sensor.start()
        sensor.close()
        self.assertIsNone(sensor.thread)
        self.assertFalse(sensor.snapshot()['enabled'])

    def test_gpio_conflicts_are_rejected(self):
        for pins in [('13', '13'), ('27', '14'), ('13', '10'), ('2', '14'), ('13', '28')]:
            with self.subTest(pins=pins), patch('sys.argv', ['tui', '--distance-backend', 'gpio', '--trig-pin', pins[0], '--echo-pin', pins[1]]):
                with patch('sys.stderr'), self.assertRaises(SystemExit):
                    parse_args()

    def test_mqtt_distance_and_no_echo_are_strict_json(self):
        hw = Hardware(self.args, [1500, 1500])
        ctrl, extras = Control(hw, [1500, 1500], self.args), Extras(self.args)
        client = Mock()
        traces = MQTTTraces(self.args, client=client, distance=self.sensor)
        traces.connected.set()
        self.sensor.record(123, time.monotonic())
        traces.sample(ctrl, extras)
        payload = json.loads(client.publish.call_args_list[0].args[1])
        self.assertEqual(payload['distance']['kalman_cm'], 123)
        self.sensor.record(None, time.monotonic())
        payload = traces.snapshot(ctrl, extras)
        self.assertIsNone(payload['distance']['kalman_cm'])
        traces.encode(payload)  # allow_nan=False en el codificador real.


if __name__ == '__main__':
    unittest.main()
