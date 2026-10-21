#!/usr/bin/env python3
"""YOLO 접근 → 낮은 호버 정지 → OFF → 수동 중앙 정렬 → 집기점 보정 저장."""
import argparse
from copy import deepcopy
from datetime import datetime
from pathlib import Path

from pick_core import ROOT, build_plan, get_rule, load_config, number, pulses, to_xy, vector
from omx_pick import (add_wrist_report, calibration_for, print_wrist_report,
                      profile_for, reference_for, require_job_arm, saved_for, solver_for)
from omx_pick_setup import atomic_record, read_snapshot
from pick_robot import DynamixelBus, MotionController, torque_off
from pick_vision import Camera, Detector, capture_target, show


def corrected_config(cfg, calibration, target, actual_xyz):
    updated = deepcopy(cfg)
    rule = get_rule(updated, target.label)
    xy = to_xy(calibration, target.point(rule))
    x, y, z = vector(actual_xyz, "실제 중심의 모델 XYZ", 3)
    # Absolute difference: repeating this operation must not accumulate offsets.
    rule["xy_offset_mm"] = [x - xy[0], y - xy[1]]
    rule["pick_z_mm"] = z
    return updated


def approach_plan(cfg, calibration, target, saved, solver, hover_mm, from_folded):
    number(hover_mm, "hover-mm", 0)
    plan = build_plan(cfg, calibration, target, saved, solver,
                      hover_only=True, from_folded=from_folded)
    plan["stages"] = [s for s in plan["stages"] if s["name"] not in ("RETREAT", "RETURN_HIGH")]
    x, y, z = plan["pick_model_xyz_mm"]
    plan["low_hover_xyz_mm"] = (x, y, z + hover_mm)
    plan["stages"].append(dict(name="LOW_HOVER", kind="arm",
                               target=pulses(solver(plan["low_hover_xyz_mm"]))))
    return plan


def move_and_release(profile, cfg, plan):
    """Leave the selected point for inspection, then release for hand alignment."""
    with DynamixelBus(profile.port) as bus:
        driver = MotionController(bus, cfg)
        driver.prepare(use_gripper=False)
        releasing = False
        try:
            driver.enable()
            for stage in plan["stages"]:
                if stage["kind"] == "joint":
                    driver.joint(stage["name"], stage["target"])
                else:
                    driver.arm(stage["name"], stage["target"])
            print("[LOW_HOVER_GOALS_SENT] 낮은 호버 목표 전송 종료. 자동 복귀 없이 토크를 유지합니다.")
            print("실제 그리퍼와 상품 중심의 편차를 확인하세요. 자동 도착 판정은 하지 않습니다.")
            if input("팔을 받치고 OFF 입력 → 수동 정렬 (다른 입력: 보정 취소·토크 유지): ").strip() != "OFF":
                driver.hold()
                return False
            releasing = True
            results = torque_off(bus)
            if not all(r["confirmed"] for r in results.values()):
                raise RuntimeError(f"토크 해제 미확인: {results}")
            print("[TORQUE_OFF] ID 11~16 해제. 이제 수동으로 중앙·실제 집기 높이에 맞추세요.")
            return True
        except BaseException:
            if not releasing:
                driver.hold()
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/omx_pick.json")
    parser.add_argument("--class", dest="label")
    parser.add_argument("--target-index", type=int, default=0)
    parser.add_argument("--from-folded", action="store_true", help="접힌 출발: ID13 → ID12 → HIGH")
    parser.add_argument("--hover-mm", type=float, default=5, help="집기 Z 위 정지 높이 mm (기본 5)")
    parser.add_argument("--stage-seconds", type=float, help="이번 실행의 일반 팔 이동 시간 (초)")
    args = parser.parse_args(argv)
    before = args.config.read_bytes()
    cfg = load_config(args.config)
    if args.stage_seconds is not None:
        if number(args.stage_seconds, "stage-seconds", 0) == 0:
            raise ValueError("stage-seconds는 양수여야 합니다")
        cfg = dict(cfg, stage_seconds=args.stage_seconds)
    profile = profile_for(None)
    require_job_arm(cfg, profile)
    names = list(cfg["classes"])
    label = args.label or (names[0] if len(names) == 1 else None)
    rule = get_rule(cfg, label)
    calibration, _ = calibration_for(profile, rule, cfg)
    if cfg.get("regions_source") == "calibration":
        cfg = dict(cfg, regions_uv=calibration["regions_uv"])
    print("상품 한 개를 시험 위치에 두세요. 촬영 후에는 상품을 움직이지 마세요.")
    detector = Detector(cfg)
    with Camera(profile) as camera:
        target, frame, _ = capture_target(camera, detector, cfg, label, args.target_index)
    print(f"선택 상품={target.label}, 영상 집기점={target.point(rule)}")
    print("화면의 박스와 십자가가 보정할 상품인지 확인하고 아무 키로 닫으세요.")
    show(frame, [target], target, cfg)
    saved = saved_for(profile, True)
    reference = reference_for(cfg, label, profile)
    plan = approach_plan(cfg, calibration, target, saved,
                         solver_for(reference, saved["hover_high"]), args.hover_mm, args.from_folded)
    add_wrist_report(plan, reference)
    print_wrist_report(plan)
    print(f"집기 XYZ mm={plan['pick_model_xyz_mm']}, 낮은 호버 XYZ mm={plan['low_hover_xyz_mm']}")
    print("순서: " + " → ".join(s["name"] for s in plan["stages"]))
    print("그리퍼를 자동 개폐하거나 상품을 들어 올리지 않습니다.")
    if input("위 위치로 이동하려면 MOVE (다른 입력: 취소): ").strip() != "MOVE":
        return 0
    if not move_and_release(profile, cfg, plan):
        return 0
    print("그리퍼를 평소 집기와 같은 수직 방향·벌림 방향으로 상품 중앙·실제 집기 높이에 맞추세요.")
    print(f"{args.hover_mm:g}mm 위 호버 자세를 그대로 기록하지 말고, 실제로 잡을 높이까지 수동으로 맞추세요.")
    print("상품은 움직이지 마세요. 기존 제어를 썼다면 종료한 뒤, 팔을 받친 상태로 읽으세요.")
    if input("중앙에 맞췄으면 Enter로 읽기 (Q: 취소): ").strip().upper() == "Q":
        return 0
    snapshot = read_snapshot(profile)
    # Preserve the file's other settings, including a manually chosen ROI.
    original = load_config(args.config)
    updated = corrected_config(original, calibration, target, snapshot["model_xyz_mm"])
    revised = get_rule(updated, label)
    print(f"XY 오프셋 mm: {rule.get('xy_offset_mm')} -> {revised['xy_offset_mm']}")
    print(f"집기 Z mm: {rule.get('pick_z_mm')} -> {revised['pick_z_mm']}")
    print("현재 시험 위치 주변용 보정입니다. 전체 선반 보정 행렬은 변경하지 않습니다.")
    if input("반영하려면 SAVE (Enter: 취소): ").strip() != "SAVE":
        return 0
    backup = args.config.with_name(args.config.name + "." + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".bak")
    with backup.open("xb") as stream:
        stream.write(before)
    atomic_record(args.config, updated, before)
    print(f"[SAVED] {args.config}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("[CANCELLED] 사용자 중단")
        raise SystemExit(130)
    except (OSError, RuntimeError, ValueError, KeyError, ImportError, EOFError) as exc:
        print(f"[FAIL] {exc}")
        raise SystemExit(2)
