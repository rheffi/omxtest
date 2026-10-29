"""Shelf correction round trips through the real planner; no hardware."""
from copy import deepcopy
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
sys.path.insert(0, str(ROOT / "tools/picking"))
import pick_core as core
import omx_pick_adjust as adjust


def fixture():
    regions = [[[0, 0], [399, 0], [399, 799], [0, 799]],
               [[600, 0], [999, 0], [999, 799], [600, 799]]]
    cfg = dict(schema_version=1, confidence=.5, stage_seconds=2.5,
               arm="test", regions_source="calibration", regions_uv=regions,
               classes={"milk": dict(xy_offset_mm=[3, 4], pick_z_mm=30,
                                      hover_clearance_mm=50, approach_extra_mm=40,
                                      grasp_reference_pulses=[2048]*5)},
               gripper=dict(open_pulse=1800, close_pulse=2200,
                            current_limit_raw=50, settle_s=.2))
    cal = dict(width=1000, height=800, layout="two-shelves", regions_uv=regions,
               matrix_3x3=[[.1, 0, 140], [0, .1, -40], [0, 0, 1]])
    left = core.Detection("milk", .9, (100, 300, 300, 500))
    right = core.Detection("milk", .95, (700, 300, 900, 500))
    saved = {k: [2048]*5 for k in ("hover_high", "drop_approach", "drop_release")}
    return cfg, cal, left, right, saved


class ShelfAdjustmentTests(unittest.TestCase):
    def test_left_then_right_round_trip_and_repeat_without_accumulation(self):
        cfg, cal, left, right, saved = fixture()
        original = deepcopy(cfg)
        cfg = adjust.corrected_config(cfg, cal, left, [170, 10, 32], "left")
        self.assertEqual(cfg["shelf_adjustments"]["left"]["delta_xyz_mm"], [7, 6, 2])
        self.assertEqual(core.pick_point(cfg, cal, right), (223, 4, 30))
        cfg = adjust.corrected_config(cfg, cal, right, [218, 7, 29], "right")
        self.assertEqual(core.pick_point(cfg, cal, left), (170, 10, 32))
        self.assertEqual(core.pick_point(cfg, cal, right), (218, 7, 29))
        self.assertEqual(adjust.corrected_config(cfg, cal, right, [218, 7, 29], "right"), cfg)
        self.assertEqual(cfg["classes"], original["classes"])
        self.assertEqual(cfg["gripper"], original["gripper"])
        plan = core.build_plan(cfg, cal, right, saved, lambda xyz: [2048]*5)
        self.assertEqual(plan["pick_model_xyz_mm"], (218, 7, 29))
        release = next(s for s in plan["stages"] if s["name"] == "DROP_RELEASE")
        self.assertEqual(release["target"], tuple(saved["drop_release"]))
        low = adjust.approach_plan(cfg, cal, right, saved, lambda xyz: [2048]*5, 5, False)
        self.assertEqual(low["low_hover_xyz_mm"], (218, 7, 34))

    def test_shared_shelf_shift_preserves_other_product_height_and_offset(self):
        cfg, cal, left, _, _ = fixture()
        cfg["classes"]["other"] = dict(cfg["classes"]["milk"], xy_offset_mm=[-1, 2], pick_z_mm=50)
        result = adjust.corrected_config(cfg, cal, left, [170, 10, 32])
        other = core.Detection("other", .9, left.box)
        self.assertEqual(core.pick_point(result, cal, other), (166, 8, 52))
        self.assertNotIn("shelf_adjustments", cfg)

    def test_sides_follow_calibration_order_not_image_x(self):
        _, cal, left, right, _ = fixture()
        cal["regions_uv"].reverse()
        self.assertEqual(core.shelf_for(cal, right.point({})), "left")
        self.assertEqual(core.shelf_for(cal, left.point({})), "right")

    def test_wrong_side_is_not_saved(self):
        cfg, cal, left, _, _ = fixture()
        with self.assertRaisesRegex(ValueError, "right 선반"):
            adjust.corrected_config(cfg, cal, left, [170, 10, 32], "right")

    def test_selected_side_filters_higher_confidence_opposite_product(self):
        cfg, cal, left, right, _ = fixture()
        cfg["regions_uv"] = [cal["regions_uv"][0]]
        self.assertEqual(core.choose([right, left], cfg, "milk", 1000, 800), left)

    def test_legacy_map_without_shelf_values_keeps_old_behavior(self):
        cfg, cal, left, _, _ = fixture()
        cal.pop("layout")
        self.assertEqual(core.pick_point(cfg, cal, left), (163, 4, 30))
        result = adjust.corrected_config(cfg, cal, left, [170, 10, 32])
        self.assertNotIn("shelf_adjustments", result)
        self.assertEqual(result["classes"]["milk"]["xy_offset_mm"], [10, 10])
        self.assertEqual(result["classes"]["milk"]["pick_z_mm"], 32)

    def test_cli_save_and_cancel_preserve_other_shelf_and_base_settings(self):
        cfg, cal, left, _, saved = fixture()
        cfg["shelf_adjustments"] = {"right": {"delta_xyz_mm": [2, -3, 1]}}
        for answer in ("SAVE", ""):
            with self.subTest(answer=answer), tempfile.TemporaryDirectory() as tmp:
                job = Path(tmp)/"job.json"
                job.write_text(json.dumps(cfg))
                before = job.read_bytes()
                camera = Mock(__enter__=Mock(), __exit__=Mock(return_value=False))
                with contextlib.ExitStack() as stack:
                    stack.enter_context(patch.object(adjust, "profile_for", return_value=SimpleNamespace(name="test")))
                    stack.enter_context(patch.object(adjust, "calibration_for", return_value=(cal, Path("unused"))))
                    stack.enter_context(patch.object(adjust, "saved_for", return_value=saved))
                    stack.enter_context(patch.object(adjust, "Detector"))
                    stack.enter_context(patch.object(adjust, "Camera", return_value=camera))
                    capture = stack.enter_context(patch.object(adjust, "capture_target", return_value=(left, None, 0)))
                    stack.enter_context(patch.object(adjust, "show"))
                    stack.enter_context(patch("omx_pick.solver_for", return_value=lambda xyz: [2048]*5))
                    if hasattr(adjust, "solver_for"):
                        stack.enter_context(patch.object(adjust, "solver_for", return_value=lambda xyz: [2048]*5))
                    if hasattr(adjust, "add_wrist_report"):
                        stack.enter_context(patch.object(adjust, "add_wrist_report"))
                        stack.enter_context(patch.object(adjust, "print_wrist_report"))
                    stack.enter_context(patch.object(adjust, "move_and_release", return_value=True))
                    stack.enter_context(patch.object(adjust, "read_snapshot", return_value={"model_xyz_mm": [170, 10, 32]}))
                    bus = stack.enter_context(patch.object(adjust, "DynamixelBus"))
                    stack.enter_context(patch("builtins.input", side_effect=["MOVE", "", answer]))
                    stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                    self.assertEqual(adjust.main(["--config", str(job), "--class", "milk", "--shelf", "left"]), 0)
                bus.assert_not_called()
                self.assertEqual(capture.call_args.args[2]["regions_uv"], [cal["regions_uv"][0]])
                if answer == "SAVE":
                    result = json.loads(job.read_text())
                    self.assertEqual(result["classes"], cfg["classes"])
                    self.assertEqual(result["regions_uv"], cfg["regions_uv"])
                    self.assertEqual(result["shelf_adjustments"]["right"], cfg["shelf_adjustments"]["right"])
                    self.assertEqual(result["shelf_adjustments"]["left"]["delta_xyz_mm"], [7, 6, 2])
                    self.assertEqual(next(Path(tmp).glob("*.bak")).read_bytes(), before)
                else:
                    self.assertEqual(job.read_bytes(), before)
                    self.assertFalse(list(Path(tmp).glob("*.bak")))


if __name__ == "__main__":
    unittest.main()
