"""Held-key XYZ recording: fake hardware/UI plus real-model IK checks."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/picking'))
import omx_pick_adjust as adjust
import omx_pick as app
import pick_robot as robot
from omx_jog_keyboard import HeldKeys, JogWindow
from omx_tool_direction import fixed_wrist_solver, pulse_to_q
from test_omx_picking import fixture, FakeBus


class FakeWindow:
    def __init__(self, events):
        self.events = iter(events)
        self.keys = SimpleNamespace(scale=1.)
        self.messages = []
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def activate(self): pass
    def poll(self): return next(self.events)
    def status(self, xyz, note=''): self.messages.append((list(xyz), note))
    def stop(self): pass


class XYRecordTest(unittest.TestCase):
    def run_cli(self, folder, events, reject_x=None, step=1):
        cfg, cal, target, saved = fixture()
        cfg['wrist_policy'] = 'fixed'
        cfg['classes']['milk']['grasp_reference_pulses'] = [2048]*5
        cfg['unfold'] = dict(elbow_pulse=2418, shoulder_pulse=1856)
        job = folder / 'config.json'
        job.write_text(json.dumps(cfg))
        before = job.read_bytes()
        samples = folder / 'samples.jsonl'
        bus = FakeBus()
        bus.registers[15, 132] = 2069
        camera = Mock(__enter__=Mock(), __exit__=Mock(return_value=False))
        window = FakeWindow(events)
        solved = []
        def factory(fk, wrist, seed):
            def solve(xyz):
                solved.append((list(xyz), wrist, list(seed)))
                if xyz[0] == reject_x:
                    raise ValueError('unreachable test point')
                return [1000+round(xyz[0]*10), 2000+round(xyz[1]*10),
                        2000+round(xyz[2]*10), 2900, wrist]
            return solve
        output = io.StringIO()
        profile = SimpleNamespace(name='arm_a', port=Path('/dev/fake'), camera='/dev/fake-camera')
        with patch.object(adjust, 'profile_for', return_value=profile), \
             patch.object(adjust, 'calibration_for', return_value=(cal, Path('unused'))), \
             patch.object(adjust, 'saved_for', return_value=saved), \
             patch.object(adjust, 'Detector'), patch.object(adjust, 'Camera', return_value=camera), \
             patch.object(adjust, 'capture_target', return_value=(target, None, 0)), \
             patch.object(adjust, 'show'), patch.object(adjust, 'DynamixelBus', return_value=bus), \
             patch.object(adjust, 'JogWindow', return_value=window), \
             patch('omx_tool_direction.fixed_wrist_solver', side_effect=factory), \
             patch.object(robot.time, 'sleep'), \
             patch.object(robot.time, 'monotonic', side_effect=iter(range(100000))), \
             patch('builtins.input', side_effect=AssertionError('No terminal input in key mode')), \
             contextlib.redirect_stdout(output):
            result = adjust.main(['--config', str(job), '--class', 'milk', '--xy-record',
                                  '--point-name', 'left_center', '--from-folded',
                                  '--samples-file', str(samples),
                                  *(['--step-mm', str(step)] if step is not None else [])])
        self.assertEqual(result, 0)
        self.assertEqual(job.read_bytes(), before)
        self.assertFalse(list(folder.glob('*.bak')))
        self.assertEqual(bus.read(16, 116, 4), cfg['gripper']['open_pulse'])
        self.assertTrue(all(bus.read(i, 64, 1) == 1 for i in range(11, 17)))
        self.assertNotIn('RETURN_HIGH', output.getvalue())
        self.assertNotIn('[GRIPPER] CLOSE', output.getvalue())
        rows = [json.loads(line) for line in samples.read_text().splitlines()]
        return rows, solved, bus, window

    def test_xy_hold_and_z_adjust_append_without_config_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            rows, solved, bus, _ = self.run_cli(Path(folder), [
                ('x+', 1), ('x+', 1), ('x+', 1), ('x+', 1),
                ('y-', 2), ('y-', 2), ('z+', 3), ('z+', 3), ('save', 4)])
            row = rows[0]
            self.assertEqual(row['original_goal_xyz_mm'], [190, 0, 35])
            self.assertEqual(row['aligned_goal_xyz_mm'], [190.25, -.125, 35.125])
            self.assertTrue(row['z_manually_adjusted'])
            self.assertEqual(row['z_delta_mm'], .125)
            self.assertEqual(row['uv'], [500, 400])
            self.assertIn('automatic_feedback', row)
            self.assertIn('aligned_feedback', row)
            self.assertTrue(all(xyz[2] == 35 for xyz, _, _ in solved[-4:-1]))
            self.assertTrue(all(wrist == 2069 for _, wrist, _ in solved))
            for i, address, _, value in bus.writes:
                if i == 15 and address == 116:
                    self.assertEqual(value, 2069)
            rows, _, _, _ = self.run_cli(Path(folder), [('save', 1)])
            self.assertEqual(len(rows), 2)
            self.assertFalse(rows[-1]['z_manually_adjusted'])

    def test_default_speed_is_ten_times_previous_for_xy_and_z(self):
        with tempfile.TemporaryDirectory() as folder:
            rows, _, _, _ = self.run_cli(Path(folder), [
                ('x+', 1), ('x+', 1), ('z+', 2), ('z+', 2), ('save', 3)], step=None)
            self.assertEqual(rows[0]['aligned_goal_xyz_mm'], [191.25, 0, 36.25])
            self.assertEqual(rows[0]['z_delta_mm'], 1.25)

    def test_release_during_ik_discards_unsent_goal_and_has_no_replay(self):
        with tempfile.TemporaryDirectory() as folder:
            rows, _, bus, _ = self.run_cli(Path(folder), [
                ('x+', 1), (None, 2), (None, 2), ('save', 3)])
            self.assertEqual(rows[0]['aligned_goal_xyz_mm'], [190, 0, 35])
            self.assertEqual(bus.read(11, 116, 4), 2900)

    def test_bad_ik_stays_at_previous_goal(self):
        with tempfile.TemporaryDirectory() as folder:
            rows, _, _, window = self.run_cli(Path(folder),
                [('x+', 1), ('save', 2)], reject_x=190.125)
            self.assertEqual(rows[0]['aligned_goal_xyz_mm'], [190, 0, 35])
            self.assertTrue(any('unreachable' in note for _, note in window.messages))

    def test_q_keeps_open_gripper_and_torque_without_saving(self):
        with tempfile.TemporaryDirectory() as folder:
            rows, _, _, _ = self.run_cli(Path(folder), [('quit', 1)])
            self.assertEqual(rows, [])

    def test_real_model_xyz_jog_keeps_down_direction_and_wrist(self):
        import numpy as np
        fk = app.legacy('02_omx_xyz_offline.py', 'xy_record_test_fk')
        xyz = [61.7564719, 204.2917092, 39.9951618]
        goal = [2853, 2161, 2103, 2904, 2069]
        for command in ['x+', 'y+', 'x-', 'y-', 'z+', 'z-']:
            xyz = adjust.jog_goal(xyz, command, .125)
            goal = fixed_wrist_solver(fk, 2069, goal)(xyz)
            pos, rot = fk.forward(pulse_to_q(goal))
            self.assertEqual(goal[4], 2069)
            self.assertLess(abs(float(pos[2])*1000 - xyz[2]), .5)
            self.assertLess(np.linalg.norm(pos*1000-xyz), 2)
            self.assertGreater(-float(rot[2, 0]), .999)


class KeyStateTest(unittest.TestCase):
    def test_hold_repeat_release_and_focus_stop(self):
        keys = HeldKeys()
        keys.press('w')
        first = keys.current()
        keys.press('w')  # Autorepeat is not queued.
        self.assertEqual(keys.current(), first)
        keys.release('w')
        self.assertIsNone(keys.current()[0])
        keys.press('f')
        self.assertEqual(keys.current()[0], 'z-')
        keys.stop()  # Focus-out / Space.
        keys.press('f')
        self.assertIsNone(keys.current()[0])
        keys.release('f')
        keys.press('f')
        self.assertEqual(keys.current()[0], 'z-')
        keys.press('return')
        self.assertEqual(keys.current()[0], 'save')

    def test_x11_repeat_release_press_pair_is_debounced(self):
        window = JogWindow(2.5)
        window.root = Mock()
        window.active = True
        event = SimpleNamespace(keysym='w')
        window._press(event)
        state = window.keys.current()
        window._release(event)
        token = window.root.after_idle.return_value
        window._press(event)
        window.root.after_cancel.assert_called_with(token)
        self.assertEqual(window.keys.current(), state)
        window._release(event)
        window.root.after_idle.call_args.args[0]()
        self.assertIsNone(window.keys.current()[0])

    def test_opposing_keys_do_not_move_and_speed_is_explicit(self):
        keys = HeldKeys()
        keys.press('w'); keys.press('s')
        self.assertIsNone(keys.current()[0])
        keys.release('s')
        self.assertEqual(keys.current()[0], 'x+')
        keys.press('1'); self.assertEqual(keys.scale, .5)
        keys.press('3'); self.assertEqual(keys.scale, 2.)
        keys.press('escape'); self.assertEqual(keys.current()[0], 'quit')


if __name__ == '__main__':
    unittest.main()
