"""Parking arrival/torque sequencing against fake motors; never opens hardware."""

import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/picking"))

from omx_pick import arguments, main, parking_for, run_park, run_pipeline
from pick_core import ARM_IDS, MODELS, Detection
from pick_robot import DynamixelBus, MotionController


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.01, seconds)


class Bus:
    def __init__(self, stalled=False, error=0, fail_off=None, gripper_stalled=False):
        self.positions = dict(zip((*ARM_IDS, 16), [2000] * 4 + [26, 2200]))
        self.torques = dict.fromkeys(self.positions, 1)
        self.writes = []
        self.stalled, self.error, self.fail_off = stalled, error, fail_off
        self.gripper_stalled = gripper_stalled

    def ping(self, motor_id):
        return dict(zip((*ARM_IDS, 16), (*MODELS, 1200)))[motor_id]

    def read(self, motor_id, address, size):
        return {132: self.positions[motor_id], 64: self.torques[motor_id],
                70: self.error, 11: 5 if motor_id == 16 else 4, 20: 0}[address]

    def write(self, motor_id, address, size, value):
        self.writes.append((motor_id, address, value))
        if address == 64:
            if motor_id == self.fail_off and value == 0:
                raise RuntimeError("simulated torque write failure")
            self.torques[motor_id] = value
        elif address == 116 and not self.stalled and not (motor_id == 16 and self.gripper_stalled):
            self.positions[motor_id] = value


def config():
    return dict(stage_seconds=1, stage_pause_seconds=0, gripper_seconds=0.1,
                gripper=dict(open_pulse=2400, close_pulse=2100, current_limit_raw=100, settle_s=0))


def route():
    return dict(route=[dict(name="park_approach", target=[2010] * 4 + [4069]),
                       dict(name="park_rest", target=[2100] * 4 + [26])],
                seconds=1.0, tolerance_pulses=20, timeout_seconds=2,
                settle_seconds=0.5, torque_off_scope="all")


class ParkingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch("pick_robot.time.monotonic", self.clock.monotonic))
        self.stack.enter_context(patch("pick_robot.time.sleep", self.clock.sleep))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def driver(self, bus, events=None):
        driver = MotionController(bus, config(),
                                  (lambda name, data: events.append((name, copy.deepcopy(data))))
                                  if events is not None else lambda *args: None)
        driver.ids, driver.touched = [*ARM_IDS, 16], True
        return driver

    def test_all_arrivals_precede_torque_off_and_wrist_takes_short_path(self):
        bus, events = Bus(), []
        self.driver(bus, events).park(route())
        self.assertEqual([data["name"] for name, data in events if name == "park_stage_done"],
                         ["park_approach", "park_rest"])
        self.assertEqual(sum(name == "park_arrival" for name, data in events), 4)
        self.assertEqual(events[-1][0], "park_torque_off")
        first_off = next(i for i, row in enumerate(bus.writes) if row[1:] == (64, 0))
        self.assertTrue(all(row[1] == 64 for row in bus.writes[first_off:]))
        self.assertEqual(bus.torques, dict.fromkeys((*ARM_IDS, 16), 0))
        self.assertEqual(bus.positions[16], 2100)
        self.assertLess(next(i for i, (name, data) in enumerate(events) if name == 'park_gripper_done'),
                        next(i for i, (name, data) in enumerate(events) if name == 'park_torque_off'))
        self.assertIn((16, 116, 2100), bus.writes[:first_off])
        self.assertTrue(all(-27 <= row[2] <= 26 for row in bus.writes if row[:2] == (15, 116)))
        self.assertGreaterEqual(self.clock.now, 3)  # Both ramps plus settling.

    def test_stall_error_and_interrupt_never_disable_torque(self):
        for kind in ("stall", "hardware_error", "interrupt"):
            with self.subTest(kind=kind):
                bus = Bus(stalled=kind == "stall", error=4 if kind == "hardware_error" else 0)
                driver = self.driver(bus)
                with contextlib.ExitStack() as stack:
                    if kind == "interrupt":
                        stack.enter_context(patch.object(driver, "wait_arm_at", side_effect=KeyboardInterrupt))
                    with self.assertRaises(KeyboardInterrupt if kind == "interrupt" else RuntimeError):
                        driver.park(route())
                self.assertFalse(any(row[1] == 64 for row in bus.writes))
                self.assertEqual(bus.torques[12], 1)
                # Every arm goal was held at its measured position.
                self.assertEqual({row[0] for row in bus.writes[-6:]}, {*ARM_IDS, 16})

    def test_partial_torque_failure_attempts_every_id_without_reenable(self):
        bus = Bus(fail_off=12)
        with self.assertRaisesRegex(RuntimeError, "일부 미확인"):
            self.driver(bus).park(route())
        off = [row for row in bus.writes if row[1] == 64]
        self.assertEqual(off, [(i, 64, 0) for i in (*ARM_IDS, 16)])
        self.assertEqual(bus.torques[12], 1)
        self.assertEqual(bus.torques[16], 0)

    def test_unstable_readings_do_not_count_as_arrival(self):
        bus = Bus()
        driver = self.driver(bus)
        original = bus.read
        count = 0

        def moving(motor_id, address, size):
            nonlocal count
            if address == 132 and motor_id == 11:
                count += 1
                return 2000 + (5 if count % 2 else -5)
            return original(motor_id, address, size)

        with patch.object(bus, "read", side_effect=moving), self.assertRaises(RuntimeError):
            driver.wait_arm_at([2000] * 4 + [26], route())
        self.assertFalse(bus.writes)

    def test_logged_high_offset_requires_explicit_transit_tolerance(self):
        target = [2033, 1860, 2187, 2433, 32]
        bus = Bus()
        bus.positions.update(dict(zip(ARM_IDS, [2034, 1893, 2193, 2475, 33])))
        driver = self.driver(bus)
        parking = route()
        with self.assertRaisesRegex(RuntimeError, "RETURN_HIGH.*기준=20.*14: 42"):
            driver.wait_arm_at(target, parking, stage="RETURN_HIGH")
        parking['transit_tolerance_pulses'] = 50
        driver.wait_arm_at(target, parking, stage="RETURN_HIGH")
        self.assertFalse(bus.writes)
        # Transit tolerance must never relax the final pose before torque OFF.
        with self.assertRaisesRegex(RuntimeError, "park_rest.*기준=20"):
            driver.wait_arm_at(target, parking, stage="park_rest", final=True)

    def test_route_uses_larger_transit_tolerance_but_final_error_prevents_torque_off(self):
        bus = Bus()
        driver = self.driver(bus)
        parking = route()
        parking['transit_tolerance_pulses'] = 50

        def ramp_with_tracking_error(targets, seconds):
            bus.positions.update(targets)
            bus.positions[14] += 42

        with patch.object(driver, 'ramp', side_effect=ramp_with_tracking_error), \
                self.assertRaisesRegex(RuntimeError, 'park_rest.*기준=20'):
            driver.park(parking)
        self.assertFalse(any(row[1] == 64 for row in bus.writes))

    def test_park_dry_run_only_needs_gripper_config_and_does_not_open_hardware(self):
        with patch("omx_pick.profile_for", return_value=SimpleNamespace(name="single", port="test")), \
                patch("omx_pick.parking_for", return_value=route()), \
                patch("omx_pick.read_json", return_value=config()), \
                patch("omx_pick.load_config", side_effect=AssertionError("pick config read")), \
                patch.object(DynamixelBus, "__enter__", side_effect=AssertionError("hardware opened")):
            self.assertEqual(main(["--park"]), 0)

    def test_missing_or_wrong_device_recordings_fail_before_pipeline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.json"
            profile = SimpleNamespace(name="single", port="test", poses=path,
                                      identity=lambda: dict(arm="single", serial_port="test"))
            with patch("omx_pick.profile_for", return_value=profile), \
                    patch("omx_pick.run_pipeline", side_effect=AssertionError("pipeline ran")), \
                    self.assertRaisesRegex(ValueError, "기록이 없습니다"):
                main(["--live", "--park-after"])
            registry = json.loads((ROOT / "tests/fixtures/omx_saved_poses.json").read_text())
            path.write_text(json.dumps(registry))
            with self.assertRaisesRegex(ValueError, "식별 정보"):
                parking_for(arguments(["--park"]), profile)

    def test_live_park_cancellation_does_not_write(self):
        bus = Bus()
        with patch.object(DynamixelBus, "__enter__", return_value=bus), \
                patch("omx_pick.read_json", return_value=config()), \
                patch.object(DynamixelBus, "__exit__", return_value=False), patch("builtins.input", return_value=""):
            run_park(arguments(["--park", "--live"]), SimpleNamespace(name="single", port="test"), route())
        self.assertFalse(bus.writes)

    def test_custom_route_loads_in_requested_order(self):
        with tempfile.TemporaryDirectory() as directory:
            registry = json.loads((ROOT / "tests/fixtures/omx_saved_poses.json").read_text())
            registry.update(arm="single", serial_port="test")
            path = Path(directory) / "poses.json"
            path.write_text(json.dumps(registry))
            profile = SimpleNamespace(poses=path, identity=lambda: dict(arm="single", serial_port="test"))
            args = arguments(["--park", "--park-pose", "drop_release", "--park-pose", "hover_high",
                              "--park-seconds", "5", "--park-transit-tolerance", "50"])
            result = parking_for(args, profile)
            self.assertEqual([p["name"] for p in result["route"]], ["drop_release", "hover_high"])
            self.assertEqual(result["route"][0]["target"], registry["poses"]["drop_release"]["raw_pulses"])
            self.assertEqual(result["seconds"], 5)
            self.assertEqual(result["transit_tolerance_pulses"], 50)
            self.assertEqual(result["tolerance_pulses"], 20)

    def test_live_park_closes_with_saved_current_then_turns_all_torque_off(self):
        bus = Bus()
        with patch.object(DynamixelBus, "__enter__", return_value=bus), \
                patch("omx_pick.read_json", return_value=config()), \
                patch.object(DynamixelBus, "__exit__", return_value=False), patch("builtins.input", return_value="PARK"):
            run_park(arguments(["--park", "--live"]), SimpleNamespace(name="single", port="test"), route())
        self.assertTrue(all(value == 0 for value in bus.torques.values()))
        self.assertEqual(bus.positions[16], 2100)
        self.assertIn((16, 102, 100), bus.writes)

    def test_gripper_stall_prevents_torque_release(self):
        bus = Bus(gripper_stalled=True)
        with self.assertRaisesRegex(RuntimeError, 'PARK_CLOSE.*16: 100'):
            self.driver(bus).park(route())
        self.assertFalse(any(row[1] == 64 for row in bus.writes))
        self.assertTrue(all(value == 1 for value in bus.torques.values()))

    def test_arm_displaced_by_closing_prevents_torque_release(self):
        bus = Bus()
        driver = self.driver(bus)
        close = driver.gripper

        def close_and_displace(*args):
            close(*args)
            bus.positions[14] += 40

        with patch.object(driver, 'gripper', side_effect=close_and_displace), \
                self.assertRaisesRegex(RuntimeError, 'PARK_REST_AFTER_CLOSE.*14: 40'):
            driver.park(route())
        self.assertEqual(bus.positions[16], 2100)
        self.assertFalse(any(row[1] == 64 for row in bus.writes))

    def test_invalid_gripper_settings_fail_before_hardware_opens(self):
        cfg = config()
        cfg['gripper']['close_pulse'] = None
        with patch('omx_pick.read_json', return_value=cfg), \
                patch.object(DynamixelBus, '__enter__', side_effect=AssertionError('hardware opened')), \
                self.assertRaises(ValueError):
            run_park(arguments(['--park', '--live']), SimpleNamespace(name='single', port='test'), route())

    def test_conflicting_options_and_bad_times_are_rejected(self):
        cases = [["--park", "--park-after"], ["--park", "--from-folded"],
                 ["--park", "--torque-scope", "arm"], ["--live", "--park-after", "--torque-scope", "gripper"],
                 ["--park", "--stage-seconds", "1"],
                 ["--park-after", "--torque-off"], ["--park-after", "--demo"],
                 ["--park-after", "--gripper-test", "open", "--live"],
                 ["--park-pose", "only_one"], ["--park", "--park-pose", "only_one"]]
        cases += [["--park", "--park-seconds", value] for value in ("nan", "inf", "0", "0.5", "61")]
        cases += [["--park", "--park-transit-tolerance", value] for value in ("0", "-1", "101", "50.5")]
        cases += [["--park-transit-tolerance", "50"]]
        for argv in cases:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                arguments(argv)

    def test_pipeline_parks_only_after_successful_cycle_and_high_arrival(self):
        for failure in (None, "cycle", "high", "hover"):
            with self.subTest(failure=failure), contextlib.ExitStack() as stack:
                args = arguments(["--live", "--park-after", "--class", "object"])
                args.hover = failure == "hover"
                profile = SimpleNamespace(name="single", port="test")
                cfg = dict(classes={"object": {}}, gripper=dict(
                    open_pulse=2400, close_pulse=2100, current_limit_raw=100, settle_s=0.3))
                target = Detection("object", 0.99, (0, 0, 1, 1))
                saved = dict(hover_high=[2000] * 4 + [26])
                plan = dict(stages=[], pick_model_xyz_mm=[1, 2, 3])
                stack.enter_context(patch("pick_vision.Detector"))
                stack.enter_context(patch("pick_vision.Camera"))
                stack.enter_context(patch("pick_vision.capture_target", return_value=(
                    target, SimpleNamespace(shape=(10, 10, 3)), None)))
                stack.enter_context(patch("omx_pick.calibration_for", return_value=(dict(width=10, height=10), "test")))
                stack.enter_context(patch("omx_pick.saved_for", return_value=saved))
                stack.enter_context(patch("omx_pick.reference_for", return_value=saved["hover_high"]))
                stack.enter_context(patch("omx_pick.solver_for"))
                stack.enter_context(patch("omx_pick.build_plan", return_value=plan))
                stack.enter_context(patch("omx_pick.add_wrist_report"))
                stack.enter_context(patch("omx_pick.print_wrist_report"))
                stack.enter_context(patch.object(DynamixelBus, "__enter__", return_value=Bus()))
                stack.enter_context(patch.object(DynamixelBus, "__exit__", return_value=False))
                stack.enter_context(patch("builtins.input", return_value="MOVE"))
                events = []
                prepare = stack.enter_context(patch.object(MotionController, 'prepare', autospec=True,
                                                           side_effect=MotionController.prepare))
                cycle = stack.enter_context(patch("omx_pick.execute", side_effect=lambda *a, **kw: events.append("cycle")))
                wait = stack.enter_context(patch.object(MotionController, "wait_arm_at", side_effect=lambda *a, **kw: events.append("high")))
                park = stack.enter_context(patch.object(MotionController, "park", side_effect=lambda *a: events.append("park")))
                if failure in ('cycle', 'high'):
                    (cycle if failure == "cycle" else wait).side_effect = RuntimeError("simulated failure")
                    with self.assertRaises(RuntimeError):
                        run_pipeline(args, profile, cfg, route())
                    park.assert_not_called()
                else:
                    run_pipeline(args, profile, cfg, route())
                    self.assertEqual(events, ["cycle", "high", "park"])
                    self.assertTrue(prepare.call_args.kwargs['use_gripper'])


if __name__ == "__main__":
    unittest.main()
