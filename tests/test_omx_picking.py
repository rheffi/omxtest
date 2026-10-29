"""Model/device-free regression tests. Run independently of legacy OpenCV tests."""

import argparse
import contextlib
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "picking"))

import pick_core as core
import pick_robot as robot
import pick_vision as vision
import omx_pick as app


def fixture():
    cfg = core.load_config(ROOT / "config" / "omx_pick.example.json")
    cfg.update(arm="arm_a", regions_uv=[[[0, 0], [999, 0], [999, 799], [0, 799]]],
               workspace_xyz_mm={"min": [-400, -400, -50], "max": [400, 400, 400]},
               gripper={"open_pulse": 1800, "close_pulse": 2200, "current_limit_raw": 50, "settle_s": 0.2})
    cfg["classes"] = {"milk": {"grasp_fraction": [0.5, 0.5], "xy_offset_mm": [0, 0], "pick_z_mm": 30,
                                  "hover_clearance_mm": 50, "approach_extra_mm": 40,
                                  "calibration_plane_verified": True}}
    cal = {"width": 1000, "height": 800, "pixel_hull": cfg["regions_uv"][0],
           "matrix_3x3": [[0.1, 0, 140], [0, 0.1, -40], [0, 0, 1]]}
    det = core.Detection("milk", 0.93, (400, 300, 600, 500))
    saved = {name: [2048] * 5 for name in ("hover_high", "drop_approach", "drop_release")}
    return cfg, cal, det, saved


class PlanningTest(unittest.TestCase):
    def setUp(self):
        self.cfg, self.cal, self.det, self.saved = fixture()

    def plan(self, **kwargs):
        return core.build_plan(self.cfg, self.cal, self.det, self.saved, lambda xyz: [2048] * 5, **kwargs)

    def test_model_free_demo_runs_complete_sequence(self):
        result = core.demo()
        self.assertTrue(result["plan"]["simulation"])
        self.assertEqual(result["plan"]["pick_model_xyz_mm"], (190, 0, 30))
        names = [event[0] for event in result["events"]]
        self.assertLess(names.index("CLOSE"), names.index("LIFT"))
        self.assertLess(names.index("DROP_RELEASE"), names.index("RELEASE"))
        self.assertEqual(names[-1], "RETURN_HIGH")

    def test_hover_has_no_gripper_or_contact(self):
        self.cfg["gripper"] = None
        self.saved.pop("drop_release")
        plan = self.plan(hover_only=True)
        self.assertTrue(all(s["kind"] == "arm" for s in plan["stages"]))
        self.assertNotIn("PICK", [s["name"] for s in plan["stages"]])

    def test_independent_shelf_regions_reject_gap_inside_calibration_hull(self):
        self.cfg["regions_uv"] = [[[0, 0], [300, 0], [300, 799], [0, 799]],
                                  [[700, 0], [999, 0], [999, 799], [700, 799]]]
        with self.assertRaisesRegex(ValueError, "작업영역 밖"):
            self.plan()
        with self.assertRaisesRegex(ValueError, "검출이 없습니다"):
            core.choose([self.det], self.cfg, "milk", 1000, 800)

    def test_polygon_edges_and_projection(self):
        self.assertTrue(core.inside((0, 400), self.cal["pixel_hull"]))
        self.assertEqual(core.to_xy(self.cal, (500, 400)), (190, 0))
        with self.assertRaises(ValueError):
            core.to_xy(self.cal, (-1, 0))
        self.cal["matrix_3x3"][2] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, "분모"):
            core.to_xy(self.cal, (500, 400))

    def test_unverified_height_null_pick_z_bad_gripper_and_bounds_fail(self):
        for mutation in ("z", "gripper", "nan"):
            with self.subTest(mutation=mutation):
                self.cfg, self.cal, self.det, self.saved = fixture()
                rule = self.cfg["classes"]["milk"]
                if mutation == "verified":
                    rule["calibration_plane_verified"] = False
                elif mutation == "z":
                    rule["pick_z_mm"] = None
                elif mutation == "gripper":
                    self.cfg["gripper"]["open_pulse"] = None
                elif mutation == "bounds":
                    self.cfg["workspace_xyz_mm"]["max"][0] = 100
                else:
                    rule["xy_offset_mm"][0] = float("nan")
                with self.assertRaises(ValueError):
                    self.plan()

    def test_failed_ik_aborts_before_execution(self):
        with self.assertRaisesRegex(ValueError, "unreachable"):
            core.build_plan(self.cfg, self.cal, self.det, self.saved,
                            Mock(side_effect=ValueError("unreachable")))

    def test_pick_through_return_needs_no_intermediate_input(self):
        driver = core.MockDriver()
        with patch("builtins.input", side_effect=AssertionError("Unexpected intermediate prompt")):
            core.execute(self.plan(), driver)
        names = [e[0] for e in driver.events]
        self.assertLess(names.index("CLOSE"), names.index("LIFT"))
        self.assertLess(names.index("LIFT"), names.index("RELEASE"))
        self.assertEqual(names[-1], "RETURN_HIGH")

    def test_simulated_plan_rejected_by_real_driver(self):
        driver = Mock(simulation=False)
        with self.assertRaisesRegex(ValueError, "모의 계획"):
            core.execute(core.demo()["plan"], driver)
        driver.arm.assert_not_called()

    def test_class_confidence_and_deterministic_selection(self):
        low = core.Detection("milk", 0.2, (10, 10, 30, 30))
        other = core.Detection("snack", 0.99, (10, 10, 30, 30))
        self.assertEqual(core.choose([low, other, self.det], self.cfg, "milk", 1000, 800), self.det)
        with self.assertRaises(ValueError):
            core.choose([self.det], self.cfg, "unknown", 1000, 800)

    def test_decode_rejects_nan_wrong_bbox_and_boolean_confidence(self):
        for box, confidence in (([0, 0, 20, 10], float("nan")), ([20, 0, 0, 10], 0.9),
                                ([0, 0, 1001, 20], 0.9), ([0, 0, 20, 10], True)):
            with self.subTest(box=box, confidence=confidence), self.assertRaises(ValueError):
                core.decode_detections({"width": 1000, "height": 800, "detections": [
                    {"class_name": "milk", "xyxy": box, "confidence": confidence}]})


class VisionTest(unittest.TestCase):
    def setUp(self):
        self.cfg, _, self.det, _ = fixture()
        self.frame = SimpleNamespace(shape=(800, 1000, 3))
        self.camera = SimpleNamespace(read=lambda: (self.frame, vision.time.monotonic()))

    def test_selects_target_from_one_frame_without_stability_or_age_gate(self):
        self.camera.read = lambda: (self.frame, 0)
        detector = Mock()
        detector.predict.return_value = [self.det]
        target, _, _ = vision.capture_target(self.camera, detector, self.cfg, "milk")
        self.assertEqual(target, self.det)
        detector.predict.assert_called_once_with(self.frame)

    def test_missing_target_reports_no_detection(self):
        detector = Mock()
        detector.predict.return_value = []
        with self.assertRaisesRegex(ValueError, "검출이 없습니다"):
            vision.capture_target(self.camera, detector, self.cfg, "milk")

    def test_nonexistent_weights_fail_before_yolo_import(self):
        with self.assertRaises(FileNotFoundError):
            vision.Detector(self.cfg, "does-not-exist.pt")


class FakeBus:
    def __init__(self):
        self.registers, self.writes = {}, []
        self.blocked = set()
        for i in (*core.ARM_IDS, 16):
            for address, value in {11: 5 if i == 16 else 4, 20: 0, 64: 0, 70: 0,
                                   132: 2048, 116: 2048, 31: 80, 146: 25, 144: 120,
                                   34: 60, 32: 140, 38: 100, 102: 100, 126: 20}.items():
                self.registers[i, address] = value

    def ping(self, i):
        return 1200 if i == 16 else core.MODELS[core.ARM_IDS.index(i)]

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read(self, i, address, size):
        return self.registers[i, address]

    def write(self, i, address, size, value):
        self.writes.append((i, address, size, value))
        self.registers[i, address] = value
        if address == 116 and i not in self.blocked:
            self.registers[i, 132] = value


class HardwarePolicyTest(unittest.TestCase):
    def test_missing_port_reports_disconnected_device_without_opening_sdk(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(RuntimeError,"연결된 OMX-F USB 직렬 장치가 없습니다"):
                with robot.DynamixelBus(Path(temp)/'serial/by-id/missing'):
                    self.fail('A missing port must not open')

    def setUp(self):
        self.cfg, _, _, self.saved = fixture()
        self.bus = FakeBus()
        self.driver = robot.MotionController(self.bus, self.cfg)

    def test_preflight_is_read_only_and_rejects_wrong_mode_or_current(self):
        self.driver.prepare(self.saved["hover_high"])
        self.assertEqual(self.bus.writes, [])
        self.bus.registers[16, 11] = 3
        with self.assertRaises(RuntimeError):
            self.driver.prepare(self.saved["hover_high"])
        self.bus.registers[16, 11] = 5
        self.cfg["gripper"]["current_limit_raw"] = 1751
        with self.assertRaises(ValueError):
            self.driver.prepare(self.saved["hover_high"])
        self.assertEqual(self.bus.writes, [])

    def test_preflight_accepts_current_start_pose(self):
        self.bus.registers[12, 132] = 1500
        self.driver.prepare(self.saved["hover_high"])
        self.assertEqual(self.bus.writes, [])

    def test_enable_aligns_goals_and_current_before_enabling_torque(self):
        self.driver.prepare(self.saved["hover_high"])
        self.driver.enable()
        torque_index = next(i for i, w in enumerate(self.bus.writes) if w[1] == 64)
        self.assertEqual(len([w for w in self.bus.writes[:torque_index] if w[1] == 116]), 6)
        self.assertIn((16, 102, 2, 50), self.bus.writes[:torque_index])
        self.assertTrue(all(w[1] not in (11, 20, 38) for w in self.bus.writes))

    def test_hover_enables_only_arm_joints(self):
        self.driver.prepare(self.saved["hover_high"], use_gripper=False)
        self.driver.enable()
        self.assertTrue(all(w[0] != 16 for w in self.bus.writes))

    def test_gripper_test_touches_only_id16(self):
        self.driver.prepare(gripper_only=True)
        self.driver.enable()
        self.assertTrue(all(w[0] == 16 for w in self.bus.writes))

    def test_unfold_sends_next_joint_despite_encoder_offset(self):
        self.bus.registers[13,132] = 2587
        self.bus.blocked.add(13)
        self.driver.prepare(self.saved['hover_high'],use_gripper=False,from_folded=True)
        self.driver.enable();self.bus.writes.clear()
        with patch.object(robot.time,'sleep'),contextlib.redirect_stdout(io.StringIO()):
            self.driver.joint('ELBOW_FIRST',{13:2559})
            self.driver.joint('SHOULDER_SECOND',{12:1616})
        self.assertEqual(self.bus.read(13,132,4),2587)
        self.assertEqual(self.bus.read(13,116,4),2559)
        self.assertEqual(self.bus.read(12,116,4),1616)
        self.assertLessEqual(sum(i==13 and a==116 for i,a,_,_ in self.bus.writes),30)

    def test_hold_attempts_other_motors_after_read_failure(self):
        self.driver.prepare(self.saved["hover_high"])
        self.driver.enable()
        read = self.bus.read
        self.bus.read = lambda i, a, s: (_ for _ in ()).throw(RuntimeError("disconnected")) if i == 12 else read(i, a, s)
        self.bus.writes.clear()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.driver.hold()
        self.assertEqual([w[0] for w in self.bus.writes], [11, 13, 14, 15, 16])



class EntryPointTest(unittest.TestCase):
    def test_full_camera_to_feedback_control_cycle_with_fake_hardware(self):
        cfg, cal, det, saved = fixture()
        bus = FakeBus()
        frame = SimpleNamespace(shape=(800, 1000, 3))
        fake_camera = Mock()
        fake_camera.__enter__ = Mock(return_value=fake_camera)
        fake_camera.__exit__ = Mock(return_value=False)
        with tempfile.TemporaryDirectory() as temp:
            logfile = Path(temp) / "live.jsonl"
            args = app.arguments(["--arm", "arm_a", "--class", "milk", "--live", "--output", str(logfile)])
            with patch.object(app, "calibration_for", return_value=(cal, Path("cal.json"))), \
                    patch.object(app, "saved_for", return_value=saved), \
                    patch.object(app, "solver_for", return_value=lambda _: [2048] * 5), \
                    patch.object(vision, "Detector"), patch.object(vision, "Camera", return_value=fake_camera), \
                    patch.object(vision, "capture_target", side_effect=lambda *a, **kw: (det, frame, robot.time.monotonic())) as capture, \
                    patch.object(robot, "DynamixelBus", return_value=bus), \
                    patch.object(robot.time, "sleep"), patch("builtins.input", side_effect=["MOVE"]) as prompt, \
                    contextlib.redirect_stdout(io.StringIO()):
                app.run_pipeline(args, SimpleNamespace(name="arm_a", port=Path("/dev/fake")), cfg)
            entries = [json.loads(line) for line in logfile.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(capture.call_count, 1)
            self.assertEqual(entries[-1]["event"], "completed")
            self.assertEqual(entries[-1]['data']['command_sequence'],'SENT')
            self.assertNotIn('encoder_sequence',entries[-1]['data'])
            done = [e["data"]["name"] for e in entries if e["event"] == "stage_done"]
            self.assertEqual(done[-1], "RETURN_HIGH")
            prompt.assert_called_once()
            self.assertNotIn("CHECK_GRASP", done)
            self.assertIn((16, 116, 4, cfg["gripper"]["close_pulse"]), bus.writes)
            self.assertIn((16, 116, 4, cfg["gripper"]["open_pulse"]), bus.writes)

    def test_cli_demo_needs_no_third_party_imports(self):
        proc = subprocess.run([sys.executable, "-I", "-S", "-B", "-X", "utf8", "-c",
                               "import sys,runpy; sys.path.insert(0,sys.argv[1]); "
                               "sys.argv=['omx_pick.py','--demo']; runpy.run_module('omx_pick',run_name='__main__')",
                               str(ROOT / "tools" / "picking")], capture_output=True, encoding="utf-8")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(json.loads(proc.stdout)["mode"], "DEMO_NO_MODEL_NO_HARDWARE")

    def test_unsafe_source_combinations_rejected_before_any_connection(self):
        for options in (["--demo", "--live"], ["--arm", "arm_a", "--live", "--image", "old.jpg"],
                        ["--arm", "arm_a", "--live", "--detections", "old.json"],
                        ["--arm", "arm_a", "--live", "--inspect"], ["--arm", "arm_a", "--gripper-test", "open"]):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    app.arguments(options)

    def test_output_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "result.json"
            app.write_output(path, {"first": True})
            with self.assertRaises(FileExistsError):
                app.write_output(path, {"second": True})
            self.assertEqual(core.read_json(path), {"first": True})

    def test_dryrun_has_no_camera_or_motor_connection(self):
        cfg, cal, det, saved = fixture()
        cal['check_error'] = {'max_mm': 6.9462096971082055, 'rms_mm': 5.542355875626934}
        cfg['max_check_error_mm'] = None
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "boxes.json"
            output = Path(temp) / "plan.json"
            path.write_text(json.dumps({"width": 1000, "height": 800, "detections": [
                {"class_name": det.label, "confidence": det.confidence, "xyxy": det.box}]}), encoding="utf-8")
            args = app.arguments(["--arm", "arm_a", "--class", "milk", "--detections", str(path),
                                  "--output", str(output)])
            with patch.object(app, "calibration_for", return_value=(cal, Path("cal.json"))), \
                    patch.object(app, "saved_for", return_value=saved), \
                    patch.object(app, "solver_for", return_value=lambda _: [2048] * 5), \
                    patch.object(vision, "Camera") as camera, patch.object(robot, "DynamixelBus") as bus, \
                    contextlib.redirect_stdout(io.StringIO()):
                app.run_pipeline(args, SimpleNamespace(name="arm_a", port=Path("/dev/fake")), cfg)
            camera.assert_not_called()
            bus.assert_not_called()
            report = json.loads(output.read_text())['plan']['calibration_check']
            self.assertEqual(report['check_error'], cal['check_error'])
            self.assertNotIn('max_check_error_mm', report)

    def test_measured_error_preserved_without_threshold_gate(self):
        cfg, cal, _, _ = fixture()
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'cal.json'
            profile = SimpleNamespace(name='arm_a',port=Path('/dev/fake'),camera='/dev/camera',
                width=1000,height=800,calibration=path,
                identity=lambda:dict(arm='arm_a',serial_port='/dev/fake'))
            cal.update(profile.identity(),camera=profile.camera,
                method='pixel_to_omx_model_xy_homography',status='check_points_recorded',
                robot_frame='link0',robot_point='end_effector_link',
                motor_zero_pulses_hypothesis=[2048]*5,motor_joint_signs_hypothesis=[1]*5,
                check_names=['a','b','c'],check_error={'max_mm':6.9462096971082055},plane_height_mm=20)
            path.write_text(json.dumps(cal))
            before = path.read_bytes()
            for limit in (None, 5, 7):
                cfg['max_check_error_mm'] = limit
                loaded, _ = app.calibration_for(profile,cfg['classes']['milk'],cfg)
                self.assertEqual(loaded['check_error'], cal['check_error'])
                self.assertEqual(path.read_bytes(), before)

    def test_disabled_error_cap_still_rejects_missing_or_invalid_measurement(self):
        cfg, cal, _, _ = fixture()
        cfg['max_check_error_mm'] = None
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'cal.json'
            profile = SimpleNamespace(name='arm_a',port=Path('/dev/fake'),camera='/dev/camera',
                width=1000,height=800,calibration=path,
                identity=lambda:dict(arm='arm_a',serial_port='/dev/fake'))
            cal.update(profile.identity(),camera=profile.camera,
                method='pixel_to_omx_model_xy_homography',status='check_points_recorded',
                robot_frame='link0',robot_point='end_effector_link',
                motor_zero_pulses_hypothesis=[2048]*5,motor_joint_signs_hypothesis=[1]*5,
                check_names=['a','b','c'],plane_height_mm=20)
            for value in (None, -1, float('nan'), float('inf'), '6.9'):
                with self.subTest(value=value):
                    cal['check_error'] = {'max_mm': value}
                    path.write_text(json.dumps(cal))
                    with self.assertRaises(ValueError):
                        app.calibration_for(profile,cfg['classes']['milk'],cfg)

    def test_calibration_allows_unmeasured_height_but_rejects_wrong_arm(self):
        cfg, cal, _, _ = fixture()
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "cal.json"
            profile = SimpleNamespace(name="arm_a", port=Path("/dev/fake"), camera="/dev/camera",
                                      width=1000, height=800, calibration=path,
                                      identity=lambda: {"arm": "arm_a", "serial_port": "/dev/fake"})
            cal.update(arm="arm_a", serial_port="/dev/fake", camera="/dev/camera",
                       method="pixel_to_omx_model_xy_homography", status="check_points_recorded",
                       robot_frame="link0", robot_point="end_effector_link",
                       motor_zero_pulses_hypothesis=[2048] * 5, motor_joint_signs_hypothesis=[1] * 5,
                       check_names=["a", "b", "c"], check_error={"max_mm": 1}, plane_height_mm=None)
            path.write_text(json.dumps(cal), encoding="utf-8")
            app.calibration_for(profile, cfg["classes"]["milk"], cfg)
            cal.update(plane_height_mm=20, arm="arm_b")
            path.write_text(json.dumps(cal), encoding="utf-8")
            with self.assertRaises(ValueError):
                app.calibration_for(profile, cfg["classes"]["milk"], cfg)


if __name__ == "__main__":
    unittest.main()
