"""Control de cuatro motores y migracion de los neutros por canal PCA."""
import configparser
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from servos_tui import Control, Extras, Hardware, MQTTTraces, load_neutrals, parse_args, save_config, servo_selection, tui
import threading


class FourServoTests(unittest.TestCase):
    def args(self, *options):
        with patch('sys.argv', ['tui', '--simulate', *options]):
            return parse_args()

    def control(self):
        args = self.args()
        neutrals = [1695, 1705, 1490, 1510]
        hw = Hardware(args, neutrals)
        ctrl = Control(hw, neutrals, args)
        now = time.monotonic()
        ctrl.last_ui = now
        return args, hw, ctrl, now

    def test_default_four_channels_and_free_feedback_pins(self):
        args = self.args()
        self.assertEqual(args.channels, [0, 1, 2, 3])
        self.assertEqual(args.pins, [27, 17, 5, 6])
        self.assertEqual(args.distance_backend, 'off')
        self.assertEqual(args.imu_backend, 'i2c')
        self.assertFalse(set(args.pins) & {2, 3, 10, args.trig_pin, args.echo_pin})

    def test_individual_selection_moves_only_chosen_servo(self):
        for number in range(1, 5):
            with self.subTest(number=number):
                args, hw, ctrl, now = self.control()
                selected, mirror = servo_selection(ord(str(number)), 4)
                ctrl.arm(now)
                ctrl.jog(selected, .2, mirror, now)
                ctrl.tick(now)
                expected = [0.0] * 4
                expected[number - 1] = .2
                self.assertEqual(ctrl.speeds, expected)
                for i in range(4):
                    self.assertEqual(hw.pulses[i] == ctrl.neutrals[i], i != number - 1)

    def test_knee_pair_mirror_and_global_stop(self):
        args, hw, ctrl, now = self.control()
        selected, mirror = servo_selection(ord('F'), 4)
        ctrl.arm(now)
        ctrl.jog(selected, .2, mirror, now)
        ctrl.tick(now)
        self.assertEqual(ctrl.speeds, [0, 0, .2, -.2])
        self.assertEqual(hw.pulses[:2], [1695, 1705])
        ctrl.stop()
        ctrl.tick(now)
        self.assertEqual(hw.pulses, ctrl.neutrals)
        self.assertEqual(ctrl.speeds, [0] * 4)
        self.assertFalse(ctrl.armed)

    def test_all_group_and_original_pair_are_distinct(self):
        self.assertEqual(servo_selection(ord('B'), 4), ([0, 1], False))
        self.assertEqual(servo_selection(ord('E'), 4), ([0, 1], True))
        self.assertEqual(servo_selection(ord('R'), 4), ([2, 3], False))
        args, hw, ctrl, now = self.control()
        selected, mirror = servo_selection(ord('T'), 4)
        ctrl.arm(now)
        ctrl.jog(selected, .05, mirror, now)
        ctrl.tick(now)
        self.assertEqual(ctrl.speeds, [.05] * 4)

    def test_arm_requires_new_knee_feedback_too(self):
        for index in (2, 3):
            args, hw, ctrl, now = self.control()
            hw.feedback[index] = SimpleNamespace(read=lambda now: (None, None, float('inf')))
            ctrl.arm(now)
            self.assertFalse(ctrl.armed)

    def test_new_knee_feedback_loss_stops_all_four(self):
        args, hw, ctrl, now = self.control()
        ctrl.arm(now)
        ctrl.jog([0, 1, 2, 3], .2, False, now)
        ctrl.tick(now)
        hw.feedback[3] = SimpleNamespace(read=lambda now: (None, None, .3))
        ctrl.tick(now)
        self.assertEqual(hw.pulses, ctrl.neutrals)
        self.assertFalse(ctrl.armed)

    def test_watchdog_and_deadman_stop_all_four(self):
        for reason in ('watchdog', 'deadman'):
            with self.subTest(reason=reason):
                args, hw, ctrl, now = self.control()
                args.deadman = .2 if reason == 'deadman' else 0
                ctrl.arm(now)
                ctrl.jog([0, 1, 2, 3], .2, False, now)
                ctrl.tick(now)
                ctrl.last_ui = now + 1 if reason == 'deadman' else now
                ctrl.tick(now + 1)
                self.assertEqual(hw.pulses, ctrl.neutrals)
                self.assertEqual(len(ctrl.speeds), 4)
                self.assertFalse(ctrl.armed)

    def test_trimming_knee_preserves_hip_calibration(self):
        args, hw, ctrl, now = self.control()
        ctrl.trim([2, 3], 5)
        ctrl.tick(now)
        self.assertEqual(ctrl.neutrals, [1695, 1705, 1495, 1515])
        self.assertEqual(hw.pulses, ctrl.neutrals)

    def test_trim_applies_when_arming_is_blocked_by_rpm(self):
        args, hw, ctrl, now = self.control()
        hw.feedback[2] = SimpleNamespace(read=lambda now: (0, 30, 0))
        ctrl.arm(now)
        self.assertFalse(ctrl.armed)
        ctrl.trim([2], -5)
        ctrl.tick(now)
        self.assertEqual(hw.pulses, [1695, 1705, 1485, 1510])
        self.assertEqual(ctrl.speeds, [0] * 4)
        self.assertFalse(ctrl.armed)

    def test_keyboard_can_trim_and_save_after_failed_arm(self):
        args, hw, ctrl, now = self.control()
        hw.feedback[2] = SimpleNamespace(read=lambda now: (0, 30, 0))
        keys = iter(map(ord, '3ajjLSq'))
        class Screen:
            def keypad(self, value): pass
            def timeout(self, value): pass
            def erase(self): pass
            def getmaxyx(self): return 24, 120
            def addnstr(self, *values): pass
            def refresh(self): pass
            def getch(self):
                ctrl.tick(time.monotonic())
                return next(keys)
        with tempfile.TemporaryDirectory() as td:
            args.config = Path(td) / 'servos.ini'
            with patch('servos_tui.curses.curs_set'), patch('servos_tui.curses.set_escdelay'):
                tui(Screen(), ctrl, args, threading.Event(), Extras(args))
            self.assertFalse(ctrl.armed)
            self.assertEqual(hw.pulses, [1695, 1705, 1485, 1510])
            self.assertEqual(load_neutrals(args), [1695, 1705, 1485, 1510])

    def test_ini_migration_preserves_existing_values_and_other_sections(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'servos.ini'
            path.write_text('[neutral]\nchannel_0=1695\nchannel_1=1705\nchannel_7=1520\n[other]\nkeep=yes\n')
            args = self.args('--config', str(path))
            self.assertEqual(load_neutrals(args), [1695, 1705, 1500, 1500])
            self.assertEqual(args.uncalibrated_channels, [2, 3])
            save_config(path, [1695, 1705, 1515, 1525], args.channels)
            self.assertEqual(load_neutrals(args), [1695, 1705, 1515, 1525])
            self.assertEqual(args.uncalibrated_channels, [])
            cfg = configparser.ConfigParser()
            cfg.read(path)
            self.assertEqual(cfg.getfloat('neutral', 'channel_7'), 1520)
            self.assertEqual(cfg.get('other', 'keep'), 'yes')

    def test_ini_values_follow_channel_numbers_not_positions(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'servos.ini'
            save_config(path, [1695, 1705, 1515, 1525], [0, 1, 2, 3])
            args = self.args('--config', str(path), '--channels', '3', '2', '1', '0')
            self.assertEqual(load_neutrals(args), [1525, 1515, 1705, 1695])

    def test_i2c_failure_still_attempts_every_channel(self):
        attempted = []
        class Channel:
            def __init__(self, number):
                self.number = number
            @property
            def duty_cycle(self):
                return 0
            @duty_cycle.setter
            def duty_cycle(self, value):
                attempted.append(self.number)
                if self.number == 2:
                    raise OSError('Fallo rodilla')
        hw = Hardware.__new__(Hardware)
        hw.channels, hw.pulses = [0, 1, 2, 3], [1500] * 4
        hw.simulate, hw.frequency = False, 50
        hw.pca = SimpleNamespace(channels=[Channel(i) for i in range(4)])
        with self.assertRaisesRegex(RuntimeError, 'CH2'):
            hw.write([1500] * 4)
        self.assertEqual(attempted, [0, 1, 2, 3])

    def test_mqtt_includes_four_channels_and_calibration_state(self):
        args, hw, ctrl, now = self.control()
        args.uncalibrated_channels = [2, 3]
        traces = MQTTTraces(args, client=Mock())
        state = json.loads(traces.encode(traces.snapshot(ctrl, Extras(args))))
        self.assertEqual([s['channel'] for s in state['servos']], [0, 1, 2, 3])
        self.assertEqual([s['gpio'] for s in state['servos']], [27, 17, 5, 6])
        self.assertEqual([s['neutral_configured'] for s in state['servos']], [True, True, False, False])

    def test_two_motor_mode_compatibility(self):
        for options in (['--channels', '0', '1'], ['--pins', '27', '17']):
            args = self.args(*options)
            hw = Hardware(args, [1695, 1705])
            ctrl = Control(hw, [1695, 1705], args)
            ctrl.tick(time.monotonic())
            self.assertEqual(args.channels, [0, 1])
            self.assertEqual(args.pins, [27, 17])
            self.assertEqual(len(hw.feedback), 2)
            self.assertEqual(hw.pulses, [1695, 1705])
            self.assertIsNone(servo_selection(ord('4'), 2))
            self.assertIsNone(servo_selection(ord('R'), 2))

    def test_invalid_lengths_and_pin_conflicts_rejected(self):
        cases = [ ['--channels', '0', '1', '2'],
                  ['--channels', '0', '1', '--pins', '27', '17', '5', '6'],
                  ['--neutral', '1695', '1705'],
                  ['--pins', '27', '17', '5', '5'],
                  ['--pins', '27', '17', '5', '10'],
                  ['--pins', '27', '17', '5', '2'],
                  ['--distance-backend', 'gpio', '--trig-pin', '5'] ]
        for options in cases:
            with self.subTest(options=options), patch('sys.stderr'):
                with self.assertRaises(SystemExit):
                    self.args(*options)

    def test_mismatched_hardware_arrays_fail_before_opening_gpio(self):
        args = self.args()
        with self.assertRaises(ValueError):
            Hardware(args, [1500, 1500])

    def test_tui_layout_and_keyboard_keep_four_rows_visible(self):
        args, hw, ctrl, now = self.control()
        keys = iter(map(ord, '4aw 3aw faw taw \t\tq'))
        frames = []
        class Screen:
            def keypad(self, value): pass
            def timeout(self, value): pass
            def erase(self): self.lines = {}
            def getmaxyx(self): return 24, 120
            def addnstr(self, row, col, value, count, attr): self.lines[row] = value[:count]
            def refresh(self): frames.append((dict(self.lines), list(ctrl.speeds)))
            def getch(self):
                ctrl.tick(time.monotonic())
                return next(keys)
        with patch('servos_tui.curses.curs_set'), patch('servos_tui.curses.set_escdelay'):
            tui(Screen(), ctrl, args, threading.Event(), Extras(args))
        self.assertTrue(any(speeds == [0, 0, 0, .05] for _, speeds in frames))
        self.assertTrue(any(speeds == [0, 0, .05, 0] for _, speeds in frames))
        self.assertTrue(any(speeds == [0, 0, .05, -.05] for _, speeds in frames))
        self.assertTrue(any(speeds == [.05] * 4 for _, speeds in frames))
        for lines, speeds in frames:
            for index in range(4):
                self.assertEqual(int(lines[5 + index].split()[0]), index + 1)
            self.assertTrue(lines[22].startswith('MQTT:'))
            self.assertNotIn('Amplia la terminal', ' '.join(lines.values()))


if __name__ == '__main__':
    unittest.main()
