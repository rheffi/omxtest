"""Offline regression checks. No camera, serial port, weights or SDK required."""

import contextlib
import copy
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/picking"))
from omx_pick import (add_wrist_report, legacy, main, reference_for, solver_for)
from omx_pick_setup import make_config
from pick_core import Detection, build_plan
from omx_tool_direction import direction_solver, heading, pulse_to_q, wrist_report
from omx_wrist import nearest_wrist

fk = legacy("02_omx_xyz_offline.py", "wrist_test_fk")


def raw(q):
    return np.rint(2048 + np.asarray(q) * 4096 / (2 * math.pi)).astype(int).tolist()


class WristDirectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        base = ROOT / "tests/fixtures"
        cls.cfg = json.loads((base / "omx_pick.json").read_text())
        cls.saved = {name: p["raw_pulses"] for name, p in
                     json.loads((base / "omx_saved_poses.json").read_text())["poses"].items()}
        cls.reference = cls.cfg["setup_measurements"]["pick_arm_pulses"]

    def assert_pose(self, values, target_mm, yaw):
        q = pulse_to_q(values)
        pos, rotation = fk.forward(q)
        self.assertLess(np.linalg.norm(pos * 1000 - target_mm), 2)
        self.assertLess(math.degrees(math.acos(np.clip(-rotation[2, 0], -1, 1))), 5)
        diff = (heading(fk, values) - yaw + math.pi) % (2 * math.pi) - math.pi
        self.assertLess(abs(math.degrees(diff)), 1)

    def test_plan_preserves_recorded_direction_and_saved_poses(self):
        xyz, _ = fk.forward(pulse_to_q(self.reference))
        x, y, _ = (xyz * 1000).tolist()
        cfg = copy.deepcopy(self.cfg)
        cfg["regions_uv"] = [[[0, 0], [99, 0], [99, 99], [0, 99]]]
        cal = dict(width=100, height=100, matrix_3x3=[[0, 0, x], [0, 0, y], [0, 0, 1]])
        target = Detection("jin_ramen", 0.99, (40, 40, 60, 60))
        for hover_only in (False, True):
            with self.subTest(hover_only=hover_only):
                plan = build_plan(cfg, cal, target, self.saved,
                                  solver_for(self.reference, self.saved["hover_high"]), hover_only=hover_only)
                add_wrist_report(plan, self.reference)
                stages = {s["name"]: s["target"] for s in plan["stages"]}
                yaw = heading(fk, self.reference)
                rule = cfg["classes"]["jin_ramen"]
                for name, dz in (("APPROACH", 90), ("HOVER", 50), ("PICK", 0)):
                    if name in stages:
                        self.assert_pose(stages[name], np.array([x, y, rule["pick_z_mm"] + dz]), yaw)
                        self.assertNotEqual(stages[name][4], 2048)
                self.assertEqual(stages["HIGH"], tuple(self.saved["hover_high"]))
                self.assertEqual(stages["RETURN_HIGH"], stages["HIGH"])
                if not hover_only:
                    self.assertEqual(stages["LIFT"], stages["HOVER"])
                    self.assertEqual(stages["DROP_RELEASE"], tuple(self.saved["drop_release"]))
                    # The controller interpolates joint angles, so check inside
                    # both vertical moves as well as their endpoints.
                    for a, b in (("APPROACH", "HOVER"), ("HOVER", "PICK")):
                        qa, qb = pulse_to_q(stages[a]), pulse_to_q(stages[b])
                        for t in np.linspace(0, 1, 21):
                            r = fk.forward(qa * (1-t) + qb * t)[1]
                            angle = math.atan2(r[1, 1], r[0, 1])
                            diff = (angle-yaw+math.pi) % (2*math.pi)-math.pi
                            self.assertLess(abs(math.degrees(diff)), 1)

    def test_world_direction_is_compensated_across_base_angles(self):
        reference = raw([0.7, -0.2, 0.8, math.pi/2 - 0.6, 0.45])
        yaw = heading(fk, reference)
        outputs = []
        solve = direction_solver(fk, reference, reference)
        for base in (0.2, 0.7, 1.2):
            # Independently generate a reachable point whose tool has this yaw.
            q = np.array([base, -0.2, 0.8, math.pi/2 - 0.6, base + math.pi/2 - yaw])
            target = fk.forward(q)[0] * 1000
            values = solve(target)
            self.assert_pose(values, target, yaw)
            outputs.append(values)
        self.assertGreater(outputs[-1][4] - outputs[0][4], 600)

    def test_wrap_boundary_uses_short_move_without_changing_base_branch(self):
        reference = raw([0.4, -0.2, 0.8, math.pi/2 - 0.6, 0])
        reference[4] = 10
        start = list(reference)
        start[4] = 4090
        target = fk.forward(pulse_to_q(reference))[0] * 1000
        output = direction_solver(fk, reference, start)(target)
        self.assertLess(abs(output[0] - reference[0]), 2)
        self.assertLess(abs(nearest_wrist(output[4], 4090) - 4090), 20)
        report = wrist_report(fk, [dict(name="HIGH", kind="arm", target=start),
                                  dict(name="DROP", kind="arm", target=reference)])
        self.assertEqual(report[-1]["id15_unwrapped_pulse"], 4106)
        self.assertEqual(nearest_wrist(4069, 26), -27)
        self.assertEqual(nearest_wrist(26, -27), 26)
        with self.assertRaises(ValueError):
            nearest_wrist(2048, 0)

    def test_motor_ramp_transmits_short_signed_wrist_goal(self):
        from pick_robot import MotionController
        class Bus:
            def __init__(self):
                self.positions = {11: 2000, 12: 2000, 13: 2000, 14: 2000, 15: 26}
                self.writes = []
            def read(self, motor_id, address, size):
                return self.positions[motor_id]
            def write(self, motor_id, address, size, value):
                self.writes.append((motor_id, address, size, value))
                self.positions[motor_id] = value
        bus = Bus()
        driver = MotionController(bus, dict(stage_seconds=0.1, stage_pause_seconds=0))
        with patch("pick_robot.time.sleep"):
            driver.arm("DROP_RELEASE", [2017, 2413, 2188, 2622, 4069])
        self.assertEqual(bus.positions[15], -27)
        self.assertTrue(all(-27 <= w[3] <= 26 for w in bus.writes if w[0] == 15))
        self.assertEqual(bus.positions[11], 2017)
        with patch("pick_robot.time.sleep"):
            driver.arm("RETURN", [2000, 2000, 2000, 2000, 26])
        self.assertEqual(bus.positions[15], 26)

    def test_preview_command_reads_existing_recordings_without_hardware(self):
        from omx_arm_config import ArmProfile
        from pick_robot import DynamixelBus
        profile = ArmProfile("single", Path(self.cfg["serial_port"]), "unused", 1920, 1080, 30,
                             Path("unused"), ROOT / "tests/fixtures/omx_xy_calibration.json",
                             ROOT / "tests/fixtures/omx_saved_poses.json")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "preview.json"
            with patch("omx_pick.profile_for", return_value=profile), \
                    patch.object(DynamixelBus, "__enter__", side_effect=AssertionError("hardware opened")), \
                    contextlib.redirect_stdout(io.StringIO()):
                status = main(["--config", str(ROOT / "tests/fixtures/omx_pick.json"),
                               "--wrist-preview", "--output", str(output)])
            self.assertEqual(status, 0)
            plan = json.loads(output.read_text())
            self.assertEqual(plan["mode"], "WRIST_PREVIEW_NO_CAMERA_NO_MOTOR")
            self.assertEqual(plan["grasp_direction"]["reference_pulses"], self.reference)

    def test_unreachable_and_invalid_inputs_fail(self):
        solve = direction_solver(fk, self.reference)
        with self.assertRaisesRegex(ValueError, "도달"):
            solve([2000, 2000, 2000])
        with self.assertRaises(ValueError):
            solve([float("nan"), 0, 0])
        for values in ([2048] * 4, [2048] * 4 + [True], [2048] * 4 + [4096]):
            with self.assertRaises(ValueError):
                direction_solver(fk, values)

    def test_old_offline_solver_still_supports_original_api(self):
        target = fk.forward(np.array([0.7, -0.2, 0.8, math.pi/2 - 0.6, 0]))[0]
        result = fk.solve(target, True)
        self.assertTrue(result.feasible)
        self.assertEqual(result.q[4], 0)

    def test_reference_migration_and_missing_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = SimpleNamespace(calibration=Path(directory) / "omx_xy_calibration.json", port="test-port",
                                      identity=lambda: {"arm": "single", "serial_port": "test-port"})
            cfg = dict(classes={"object": {}})
            with self.assertRaisesRegex(ValueError, "기록이 없습니다"):
                reference_for(cfg, "object", profile)
            cfg["setup_measurements"] = dict(pick_arm_pulses=self.reference)
            self.assertEqual(reference_for(cfg, "object", profile), self.reference)
            # A global reference from an old single-class setup cannot be used
            # for an unrelated second class.
            cfg["classes"]["other"] = {}
            with self.assertRaises(ValueError):
                reference_for(cfg, "other", profile)
            registry = dict(arm="single", serial_port="test-port", classes={
                "other": dict(pick=dict(arm_pulses=self.reference))})
            path = Path(directory) / "omx_pick_setup_samples.json"
            path.write_text(json.dumps(registry))
            self.assertEqual(reference_for(cfg, "other", profile), self.reference)
            registry["serial_port"] = "different-port"
            path.write_text(json.dumps(registry))
            with self.assertRaises(ValueError):
                reference_for(cfg, "other", profile)
            cfg["classes"]["other"]["grasp_reference_pulses"] = [2047] * 5
            self.assertEqual(reference_for(cfg, "other", profile), [2047] * 5)

    def test_preview_cannot_be_combined_with_live(self):
        from omx_pick import arguments
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            arguments(["--wrist-preview", "--live"])

    def test_setup_persists_direction_per_class(self):
        from omx_arm_config import ArmProfile
        calibration = json.loads((ROOT / "tests/fixtures/omx_xy_calibration.json").read_text())
        profile = ArmProfile("single", Path(calibration["serial_port"]), calibration["camera"],
                             calibration["width"], calibration["height"], 30,
                             Path("unused"), Path("unused"), Path("unused"))
        values = dict(pick_z_mm=1, open=2400, close=2100, current=100,
                      basket_type="temporary", mount="unspecified", note="",
                      setup_measurements=dict(pick_arm_pulses=self.reference))
        cfg = make_config(profile, calibration, Path("unused.pt"), "object", values)
        self.assertEqual(cfg["classes"]["object"]["grasp_reference_pulses"], self.reference)

    def test_click_hover_uses_high_direction_and_short_wrist_transmission(self):
        hover = legacy("06_omx_xy_hover.py", "wrist_test_click_hover")
        xyz, _ = fk.forward(pulse_to_q(self.reference))
        x, y, z = (xyz * 1000).tolist()
        cal = dict(width=100, height=100, model_z_reference_mm=z,
                   matrix_3x3=[[0, 0, x], [0, 0, y], [0, 0, 1]])
        plan = hover.make_plan(cal, (50, 50), np.array(self.reference))
        self.assert_pose(plan["hover_pulses"].tolist(), np.array([x, y, z+50]), heading(fk, self.reference))
        writes = []
        with patch.object(hover, "read_positions", return_value=np.array([2000]*4+[26])), \
                patch.object(hover, "sdk_write", side_effect=lambda *args: writes.append(args)), \
                patch.object(hover.time, "sleep"), contextlib.redirect_stdout(io.StringIO()):
            hover.send_stage(None, None, "TEST", {15:4069}, 0.1)
        self.assertEqual(writes[-1][5], -27)


if __name__ == "__main__":
    unittest.main()
