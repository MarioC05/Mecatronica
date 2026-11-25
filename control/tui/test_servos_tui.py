import configparser
import contextlib
import io
from unittest.mock import patch
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from servos_tui import Control, Extras, Feedback, Hardware, slider, parse_args, save_config, speed_pulse


class FakeFeedback:
    age = 0
    def read(self, now):
        return 0, 0, self.age


class FakeHardware:
    def __init__(self):
        self.feedback = [FakeFeedback(), FakeFeedback()]
        self.writes = []
    def write(self, pulses):
        self.writes.append(list(pulses))


class Tests(unittest.TestCase):
    def control(self):
        hw = FakeHardware()
        ctrl = Control(hw, [1500, 1505], SimpleNamespace(limit=.5, deadman=.8))
        self.now = time.monotonic()
        ctrl.last_ui = self.now
        return ctrl, hw

    def test_saturation_and_neutral(self):
        self.assertEqual(speed_pulse(0, 1505), 1505)
        self.assertAlmostEqual(speed_pulse(1, 1550), 1550 * 1720 / 1500)
        self.assertAlmostEqual(speed_pulse(-1, 1450), 1450 * 1280 / 1500)

    def test_high_calibrated_neutral_does_not_clip_motion(self):
        self.assertEqual(speed_pulse(0, 1685), 1685)
        self.assertAlmostEqual(speed_pulse(.5, 1685), 1822.6083333333333)
        self.assertGreater(speed_pulse(.05, 1685), 1720)
        self.assertLess(speed_pulse(-.5, 1685), 1600)
        pulses = [speed_pulse(s / 100, 1685) for s in range(-50, 51)]
        self.assertEqual(pulses, sorted(pulses))

    def test_latched_mode_keeps_motion_but_watchdog_still_stops(self):
        c, h = self.control()
        c.args.deadman = 0
        c.arm(self.now)
        c.jog([0], .5, False, self.now)
        c.last_ui = self.now + 5
        c.tick(self.now + 5)
        self.assertTrue(c.armed)
        self.assertGreater(h.writes[-1][0], 1500)
        c.tick(self.now + 5.6)
        self.assertFalse(c.armed)
        self.assertEqual(h.writes[-1], [1500, 1505])

    def test_no_motion_without_arm(self):
        c, h = self.control()
        c.jog([0, 1], .5, True, self.now)
        c.tick(self.now)
        self.assertEqual(h.writes[-1], [1500, 1505])

    def test_mirror_limit_and_stop(self):
        c, h = self.control()
        c.arm(self.now)
        for _ in range(20):
            c.jog([0, 1], .05, True, self.now)
        c.tick(self.now)
        self.assertEqual(c.speeds, [.5, -.5])
        self.assertAlmostEqual(h.writes[-1][0], 1622.5)
        self.assertAlmostEqual(h.writes[-1][1], 1505 - 122.5 * 1505 / 1500)
        c.stop()
        c.tick(self.now)
        self.assertEqual(h.writes[-1], [1500, 1505])
        self.assertFalse(c.armed)

    def test_neutral_is_refreshed_even_without_new_command(self):
        c, h = self.control()
        c.tick(self.now)
        c.last_ui = self.now + .11
        c.tick(self.now + .11)
        self.assertEqual(h.writes, [[1500, 1505], [1500, 1505]])

    def test_trim_can_reach_wider_nominal_range(self):
        c, h = self.control()
        for _ in range(30): c.trim([0], -5)
        self.assertEqual(c.neutrals[0], 1350)
        for _ in range(100): c.trim([0], -5)
        self.assertEqual(c.neutrals[0], 1200)
        for _ in range(120): c.trim([0], 5)
        self.assertEqual(c.neutrals[0], 1800)

    def test_cannot_arm_while_motor_is_physically_moving(self):
        c, h = self.control()
        h.feedback[0].read = lambda now: (40, 94, 0)
        c.arm(self.now)
        self.assertFalse(c.armed)

    def test_deadman_stops_both(self):
        c, h = self.control()
        c.arm(self.now)
        c.jog([0, 1], .4, True, self.now)
        c.last_ui = self.now + .9
        c.tick(self.now + .9)
        self.assertEqual(h.writes[-1], [1500, 1505])
        self.assertFalse(c.armed)

    def test_frozen_ui_stops(self):
        c, h = self.control()
        c.arm(self.now)
        c.jog([0], .5, False, self.now)
        c.last_command = self.now + .6
        c.tick(self.now + .6)
        self.assertEqual(h.writes[-1], [1500, 1505])
        self.assertFalse(c.armed)

    def test_feedback_loss_stops_pair(self):
        c, h = self.control()
        c.arm(self.now)
        c.jog([0, 1], .4, True, self.now)
        h.feedback[1].age = .3
        c.tick(self.now)
        self.assertEqual(h.writes[-1], [1500, 1505])
        self.assertFalse(c.armed)

    def test_arm_requires_both_feedbacks(self):
        c, h = self.control()
        h.feedback[0].age = float('inf')
        c.arm(self.now)
        self.assertFalse(c.armed)

    def test_trim_disarms_and_preserves_other_neutral(self):
        c, h = self.control()
        c.arm(self.now)
        c.jog([0, 1], .4, False, self.now)
        c.trim([0], 5)
        c.tick(self.now)
        self.assertEqual(h.writes[-1], [1505, 1505])
        self.assertFalse(c.armed)

    def test_feedback_whole_cycles_and_wrap(self):
        f = Feedback()
        period = 1_100_000
        tick = 10_000_000
        f.edge(0, 27, 1, tick)
        self.assertIsNone(f.read(time.monotonic())[0])
        for k in range(220):
            angle = (350 + k) % 360
            duty = .027 + angle / 360 * .944
            f.edge(0, 27, 0, tick + round(period * duty))
            tick += period
            f.edge(0, 27, 1, tick)
        angle, rpm, age = f.read(time.monotonic())
        self.assertAlmostEqual(angle, 209, places=2)
        self.assertAlmostEqual(rpm, 1 / .0011 / 6, places=1)
        self.assertLess(age, .1)
        self.assertGreater(f.read(time.monotonic() + 1)[2], .9)

    def test_invalid_feedback_does_not_refresh(self):
        f = Feedback()
        f.edge(0, 27, 1, 1000)
        f.edge(0, 27, 0, 1100)
        f.edge(0, 27, 1, 1200)
        self.assertIsNone(f.read(time.monotonic())[0])

    def test_config_channel_mapping(self):
        with tempfile.TemporaryDirectory() as t:
            path = Path(t)/'servos.ini'
            save_config(path, [1495, 1510], [4, 6])
            cfg = configparser.ConfigParser()
            cfg.read(path)
            self.assertEqual(cfg.getfloat('neutral', 'channel_4'), 1495)
            self.assertEqual(cfg.getfloat('neutral', 'channel_6'), 1510)

    def test_i2c_attempts_both_channels_even_when_first_fails(self):
        attempts = []
        class Channel:
            def __init__(self, i): self.i = i
            @property
            def duty_cycle(self): return 0
            @duty_cycle.setter
            def duty_cycle(self, value):
                attempts.append(self.i)
                if self.i == 0: raise OSError('fallo simulado')
        h = Hardware.__new__(Hardware)
        h.simulate = False
        h.frequency = 50
        h.channels = [0, 1]
        h.pulses = [1500, 1500]
        h.pca = SimpleNamespace(channels=[Channel(0), Channel(1)])
        with self.assertRaises(RuntimeError): h.write([1500, 1500])
        self.assertEqual(attempts, [0, 1])

    def test_worker_failure_attempts_neutral(self):
        c, h = self.control()
        c.arm(self.now)
        c.jog([0, 1], .4, True, self.now)
        attempts = []
        def write(pulses):
            attempts.append(list(pulses))
            if len(attempts) == 1: raise OSError('fallo I2C')
        h.write = write
        c.run()
        self.assertIsNotNone(c.fault)
        self.assertEqual(attempts[-1], [1500, 1505])


class ExtrasTests(unittest.TestCase):
    def extras(self, simulate=True):
        return Extras(SimpleNamespace(simulate=simulate, led_backend='spi',
                                      led_count=12, led_order='GRB'))

    def test_color_brightness_clamp_and_switch(self):
        ex = self.extras()
        ex.led_change(0, -999)
        ex.led_change(1, 999)
        ex.led_change(3, 999)
        self.assertEqual(ex.snapshot()['rgb'][:2], [0, 255])
        self.assertEqual(ex.snapshot()['brightness'], 100)
        ex.led_change(toggle=True)
        self.assertTrue(ex.snapshot()['led_on'])
        ex.led_change(off=True)
        self.assertFalse(ex.snapshot()['led_on'])



    def test_mock_spi_receives_color_brightness_and_off(self):
        ex = self.extras(False)
        class Pixels:
            brightness = 0
            color = None
            calls = 0
            def fill(self, rgb): self.color = rgb
            def show(self): self.calls += 1
        ex.pixels = Pixels()
        status = ex.apply_leds([40, 80, 120], 25, True)
        self.assertEqual(ex.pixels.color, (10,20,30))
        self.assertEqual(ex.pixels.brightness, 0)  # El brillo se aplica a RGB, sin escalar dos veces.
        self.assertIn('BCM10', status)
        ex.apply_leds([40,80,120],25,False)
        self.assertEqual(ex.pixels.color, (0,0,0))
        self.assertEqual(ex.pixels.calls, 2)

    def test_rgbw_adds_zero_white(self):
        ex = self.extras(False)
        ex.args.led_order = 'GRBW'
        class Pixels:
            def fill(self, rgb): self.color = rgb
            def show(self): pass
        ex.pixels = Pixels()
        ex.apply_leds([10,20,30], 10, True)
        self.assertEqual(ex.pixels.color, (1,2,3,0))


    def test_worker_failure_is_reported_without_motor_coupling(self):
        ex = self.extras(False)
        ex.apply_leds = lambda *args: (_ for _ in ()).throw(OSError('SPI ausente'))
        ex.led_change(toggle=True)
        ex.start()
        deadline = time.monotonic() + 1
        while ex.applied != ex.generation and time.monotonic() < deadline: time.sleep(.01)
        ex.close()
        self.assertIn('SPI ausente', ex.snapshot()['led_status'])

    def test_simulated_peripherals_need_no_hardware_libraries(self):
        ex = self.extras()
        ex.start()
        ex.led_change(toggle=True)
        deadline = time.monotonic() + 1
        while ex.applied != ex.generation and time.monotonic() < deadline: time.sleep(.01)
        self.assertIn('SIMULADO', ex.snapshot()['led_status'])
        ex.close()
        self.assertFalse(ex.thread.is_alive())

    def test_default_limits_and_gpio_plan(self):
        with patch('sys.argv', ['servos_tui.py', '--simulate']):
            args = parse_args()
        self.assertEqual(args.limit, 1)
        self.assertEqual(args.deadman, 0)
        self.assertEqual(args.led_count, 12)
        self.assertEqual(args.led_backend, 'spi')
        self.assertEqual(args.pins, [27, 17, 5, 6])
        self.assertEqual(args.channels, [0, 1, 2, 3])

    def test_full_requested_speed_reaches_100_percent(self):
        h = FakeHardware()
        args = SimpleNamespace(limit=1, deadman=0, deadband_us=25)
        c = Control(h, [1680, 1685], args)
        now = time.monotonic()
        c.last_ui = now
        c.arm(now)
        for _ in range(30): c.jog([1], .05, False, now)
        c.tick(now)
        self.assertEqual(c.speeds, [0, 1])
        self.assertAlmostEqual(h.writes[-1][1], 1685 * 1720 / 1500)
        c.stop()
        c.tick(now)
        self.assertEqual(h.writes[-1], [1680, 1685])

    def test_gpio_conflict_is_rejected_before_opening_devices(self):
        with patch('sys.argv', ['servos_tui.py', '--pins', '10', '17']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): parse_args()

    def test_slider_endpoints(self):
        self.assertEqual(slider(0, 255), '['+'-'*16+']')
        self.assertEqual(slider(255,255), '['+'#'*16+']')


if __name__ == '__main__':
    unittest.main()
