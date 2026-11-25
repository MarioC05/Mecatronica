"""Pruebas GY-521 sin bus I2C ni librerias de hardware."""
import json
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

from servos_tui import Control, DistanceSensor, Extras, Hardware, IMUSensor, MQTTTraces, parse_args


class IMUTests(unittest.TestCase):
    def setUp(self):
        with patch('sys.argv', ['tui', '--simulate', '--channels', '0', '1']):
            self.args = parse_args()
        self.bus = Mock()
        self.imu = IMUSensor(self.args, self.bus)

    def driver(self, factory):
        return SimpleNamespace(MPU6050=factory,
                               Range=SimpleNamespace(RANGE_2_G=0),
                               GyroRange=SimpleNamespace(RANGE_250_DPS=0),
                               Bandwidth=SimpleNamespace(BAND_21_HZ=4))

    def test_default_imu_on_ultrasonic_off_claims_no_gpio(self):
        self.assertEqual(self.args.imu_backend, 'i2c')
        self.assertEqual(self.args.distance_backend, 'off')
        sensor = DistanceSensor(self.args)
        with patch.object(sensor, 'open') as opened:
            sensor.start()
            sensor.close()
        opened.assert_not_called()
        self.assertIsNone(sensor.thread)
        self.assertFalse(sensor.snapshot()['enabled'])

    def test_open_falls_back_to_69_on_same_bus(self):
        sensor = SimpleNamespace()
        factory = Mock(side_effect=[OSError('No en 68'), sensor])
        with patch.dict('sys.modules', {'adafruit_mpu6050': self.driver(factory)}):
            self.imu.open()
        self.assertEqual(factory.call_args_list, [call(self.bus, address=0x68), call(self.bus, address=0x69)])
        self.assertEqual(self.imu.snapshot()['address'], '0x69')
        self.assertEqual(sensor.filter_bandwidth, 4)
        self.assertEqual(sensor.sample_rate_divisor, 19)
        self.imu.close()
        self.bus.deinit.assert_not_called()

    def test_explicit_address_and_missing_sensor(self):
        self.args.imu_address = 0x69
        factory = Mock(side_effect=OSError('Sin sensor'))
        with patch.dict('sys.modules', {'adafruit_mpu6050': self.driver(factory)}):
            with self.assertRaisesRegex(RuntimeError, '0x69'):
                self.imu.open()
        factory.assert_called_once_with(self.bus, address=0x69)

    def test_units_and_static_tilt(self):
        gravity = 9.80665
        self.imu.record((0, gravity / math.sqrt(2), gravity / math.sqrt(2)),
                        (math.pi, 0, -math.pi / 2), 30.5, time.monotonic())
        state = self.imu.snapshot()
        self.assertTrue(state['valid'])
        self.assertAlmostEqual(state['roll_deg'], 45)
        self.assertAlmostEqual(state['pitch_deg'], 0)
        self.assertEqual(state['gyro_deg_s'], {'x': 180, 'y': 0, 'z': -90})
        self.assertEqual(state['temperature_c'], 30.5)

    def test_pitch_sign_and_zero_gravity(self):
        self.imu.record((-1, 0, 1), (0, 0, 0), 25, time.monotonic())
        self.assertAlmostEqual(self.imu.snapshot()['pitch_deg'], 45)
        self.imu.record((0, 0, 0), (0, 0, 0), 25, time.monotonic())
        self.assertIsNone(self.imu.snapshot()['roll_deg'])
        self.assertIsNone(self.imu.snapshot()['pitch_deg'])

    def test_stale_values_are_null(self):
        self.imu.record((0, 0, 9.8), (0, 0, 0), 25, time.monotonic() - 1)
        state = self.imu.snapshot()
        self.assertFalse(state['valid'])
        self.assertIsNone(state['accel_m_s2'])
        self.assertIsNone(state['gyro_deg_s'])
        self.assertIsNone(state['temperature_c'])
        self.assertEqual(state['status'], 'LECTURA CADUCADA')

    def test_nonfinite_readings_rejected(self):
        with self.assertRaises(ValueError):
            self.imu.record((0, 0, math.nan), (0, 0, 0), 25, time.monotonic())
        self.assertFalse(self.imu.snapshot()['valid'])

    def test_simulation_needs_no_imu_library(self):
        with patch.dict('sys.modules', {'adafruit_mpu6050': None}):
            self.imu.start()
            try:
                self.assertTrue(self.wait_for(lambda: self.imu.snapshot()['valid']))
            finally:
                self.imu.close()
        self.assertFalse(self.imu.thread.is_alive())

    def wait_for(self, predicate):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(.005)
        return False

    def test_missing_dependency_is_visible_and_close_interrupts_retry(self):
        self.args.simulate = False
        with patch.dict('sys.modules', {'adafruit_mpu6050': None}):
            self.imu.start()
            try:
                self.assertTrue(self.wait_for(lambda: 'requirements-imu.txt' in self.imu.snapshot()['status']))
            finally:
                self.imu.close()
        self.assertFalse(self.imu.thread.is_alive())
        self.assertFalse(self.imu.snapshot()['valid'])

    def test_retries_after_read_error(self):
        self.args.simulate = False
        self.imu.RETRY = .005
        broken = Mock()
        type(broken).acceleration = property(lambda self: (_ for _ in ()).throw(OSError('I2C perdido')))
        good = SimpleNamespace(acceleration=(0, 0, 9.8), gyro=(0, 0, 0), temperature=26)
        self.imu.sensor = broken
        with patch.object(self.imu, 'open', side_effect=lambda: setattr(self.imu, 'sensor', good)) as opened:
            self.imu.start()
            try:
                self.assertTrue(self.wait_for(lambda: self.imu.snapshot()['valid']))
                opened.assert_called_once()
            finally:
                self.imu.close()

    def test_stalled_imu_read_does_not_lock_motor_control(self):
        self.args.simulate = False
        entered, release = threading.Event(), threading.Event()
        class SlowSensor:
            gyro, temperature = (0, 0, 0), 25
            @property
            def acceleration(self):
                entered.set()
                release.wait(2)
                return (0, 0, 9.8)
        self.imu.sensor = SlowSensor()
        self.args.simulate = True
        hw = Hardware(self.args, [1500, 1500])
        ctrl = Control(hw, [1500, 1500], self.args)
        self.args.simulate = False
        self.imu.start()
        try:
            self.assertTrue(entered.wait(1))
            now = time.monotonic()
            ctrl.last_ui = now
            ctrl.tick(now)
            self.assertEqual(hw.pulses, [1500, 1500])
        finally:
            release.set()
            self.imu.close()
        self.assertFalse(self.imu.thread.is_alive())

    def test_disabled_imu_never_starts(self):
        self.args.imu_backend = 'off'
        imu = IMUSensor(self.args)
        imu.start()
        imu.close()
        self.assertIsNone(imu.thread)
        self.assertFalse(imu.snapshot()['enabled'])

    def test_mqtt_exposes_imu_with_units(self):
        self.imu.record((0, 0, 9.8), (0, 0, 1), 25, time.monotonic())
        hw = Hardware(self.args, [1500, 1500])
        ctrl, extras = Control(hw, [1500, 1500], self.args), Extras(self.args)
        traces = MQTTTraces(self.args, client=Mock(), imu=self.imu)
        state = json.loads(traces.encode(traces.snapshot(ctrl, extras)))
        self.assertEqual(state['imu']['accel_m_s2']['z'], 9.8)
        self.assertAlmostEqual(state['imu']['gyro_deg_s']['z'], math.degrees(1))

    def test_invalid_address_and_pca_conflict(self):
        for options in (['--imu-address', '0x70'], ['--address', '0x68']):
            with patch('sys.argv', ['tui', *options]), patch('sys.stderr'):
                with self.assertRaises(SystemExit):
                    parse_args()


if __name__ == '__main__':
    unittest.main()
