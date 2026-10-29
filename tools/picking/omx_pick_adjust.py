#!/usr/bin/env python3
"""집기점 보정 또는 --xy-record: 열린 낮은 호버 → WASD/RF XYZ 조절 → 기록만 저장."""
import argparse
import hashlib
import json
import math
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path

from pick_core import (ROOT, build_plan, get_rule, load_config, number, pick_point,
                       pulses, shelf_for, to_xy, vector)
from omx_pick import (calibration_for, profile_for, reference_for, require_job_arm,
                      saved_for, select_grasp_plan)
from omx_pick_setup import atomic_record, read_snapshot
from pick_robot import DynamixelBus, MotionController, torque_off
from pick_vision import Camera, Detector, capture_target, show
from omx_jog_keyboard import JogWindow


def jog_goal(xyz, command, step_mm):
    """Robot link0 single-axis increments; XY input leaves Z unchanged."""
    moves = {"x+": (0, 1), "x-": (0, -1), "y+": (1, 1), "y-": (1, -1),
             "z+": (2, 1), "z-": (2, -1)}
    if command not in moves:
        raise ValueError("지원하지 않는 XYZ 조절 방향입니다")
    result = list(xyz)
    axis, sign = moves[command]
    result[axis] += sign * step_mm
    return result


def xy_snapshot(bus, fk):
    from omx_tool_direction import pulse_to_q
    from omx_wrist import canonical_wrist
    raw = [bus.read(i, 132, 4) for i in range(11, 16)]
    raw[4] = canonical_wrist(raw[4])
    position, rotation = fk.forward(pulse_to_q(raw))
    return dict(raw_pulses=raw, model_xyz_mm=(position * 1000).tolist(),
                model_down_error_deg=math.degrees(math.acos(max(-1., min(1., -float(rotation[2, 0]))))))


def jog_keyboard(driver, fk, wrist, xyz, goal, controls, speed):
    """One small IK target per tick; no buffered repeat or catch-up movement."""
    from omx_tool_direction import fixed_wrist_solver
    from omx_wrist import nearest_wrist
    automatic = None
    z_adjusted = False
    controls.activate()
    controls.status(xyz)
    next_send = 0.
    while True:
        command, revision = controls.poll()
        if command == 'quit':
            return None
        if command == 'save':
            if automatic is None:
                automatic = xy_snapshot(driver.bus, fk)
            return xyz, goal, automatic, z_adjusted
        if command is None or time.monotonic() < next_send:
            time.sleep(.01)
            continue
        if automatic is None:
            automatic = xy_snapshot(driver.bus, fk)
        candidate = jog_goal(xyz, command, speed * controls.keys.scale * .05)
        try:
            candidate_goal = list(pulses(fixed_wrist_solver(fk, wrist, goal)(candidate)))
        except ValueError as exc:
            controls.stop()
            controls.status(xyz, str(exc))
            continue
        # Key-up/focus loss while IK was running must discard this unsent goal.
        if controls.poll() != (command, revision):
            continue
        for index, motor_id in enumerate(range(11, 16)):
            value = candidate_goal[index]
            if motor_id == 15:
                value = nearest_wrist(value, driver.bus.read(15, 132, 4))
            driver.bus.write(motor_id, 116, 4, value)
        xyz, goal = candidate, candidate_goal
        z_adjusted = z_adjusted or command.startswith('z')
        controls.status(xyz)
        # Slow IK/serial reads never cause a later burst of queued movement.
        next_send = time.monotonic() + .05


def record_xy(args, cfg, calibration, target, profile, shelf):
    """Open low hover -> held-key XYZ jog -> append one sample, no config writes."""
    from omx_pick import build_pick_plan, legacy
    from omx_wrist import canonical_wrist
    if cfg.get("wrist_policy") != "fixed":
        raise ValueError("XY 기록 모드는 wrist_policy=fixed 설정을 사용합니다")
    saved = saved_for(profile, True)
    reference = reference_for(cfg, target.label, profile)
    fk = legacy("02_omx_xyz_offline.py", "xy_record_fk")
    args.samples_file.parent.mkdir(parents=True, exist_ok=True)
    speed = args.step_mm / args.jog_seconds
    with JogWindow(speed) as controls, args.samples_file.open("a", encoding="utf-8") as stream, DynamixelBus(profile.port) as bus:
        driver = MotionController(bus, cfg)
        driver.prepare(use_gripper=True, from_folded=args.from_folded)
        wrist = canonical_wrist(bus.read(15, 132, 4))
        plan = build_pick_plan(cfg, calibration, target, saved, reference, hover_only=True,
                              from_folded=args.from_folded, profile=profile,
                              fixed_wrist_pulse=wrist, hover_stop_mm=args.hover_mm)
        xyz = list(plan["low_hover_xyz_mm"])
        original = list(xyz)
        goal = list(plan["stages"][-1]["target"])
        try:
            print(f"[XY RECORD] {args.point_name}: XYZ={xyz}, ID15={wrist}; MOVE 입력 없이 이동합니다.", flush=True)
            driver.enable()
            for stage in plan["stages"]:
                getattr(driver, stage["kind"])(stage["name"], stage["target"])
            print("조절창에서 WASD: XY / R·F: Z. 누르는 동안 이동, 놓으면 추가 목표 전송 중단.")
            print("Enter: 저장·종료 / Q·Esc: 취소 / Space: 정지. 토크는 유지합니다.")
            result = jog_keyboard(driver, fk, wrist, xyz, goal, controls, speed)
            if result is None:
                print("[CANCELLED] 기록 없이 종료. 현재 목표·토크 유지.")
                return 0
            xyz, goal, automatic, z_adjusted = result
            actual = xy_snapshot(bus, fk)
            row = dict(recorded_at=datetime.now().astimezone().isoformat(),
                               point_name=args.point_name, class_name=target.label, shelf=shelf,
                               config_file=str(args.config.resolve()), serial_port=str(profile.port),
                               camera=str(profile.camera), image_size=[calibration["width"], calibration["height"]],
                               calibration_sha256=hashlib.sha256(json.dumps(calibration, sort_keys=True).encode()).hexdigest(),
                               uv=list(plan["uv"]), box_xyxy=list(target.box),
                               class_rule=get_rule(cfg, target.label),
                               shelf_adjustment=cfg.get("shelf_adjustments", {}).get(shelf),
                               pick_model_xyz_mm=list(plan["pick_model_xyz_mm"]),
                               hover_mm=args.hover_mm, fixed_wrist_pulse=wrist,
                               original_goal_xyz_mm=original, aligned_goal_xyz_mm=xyz,
                               original_goal_pulses=list(plan["stages"][-1]["target"]),
                               aligned_goal_pulses=goal, automatic_feedback=automatic,
                               aligned_feedback=actual, physical_alignment="operator_confirmed",
                               z_manually_adjusted=z_adjusted, z_delta_mm=xyz[2]-original[2])
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            print(f"[SAVED] {args.samples_file}: {args.point_name}. 설정 변경 없음, 토크 유지.")
            return 0
        except BaseException:
            driver.hold()
            raise


def corrected_config(cfg, calibration, target, actual_xyz, shelf=None):
    updated = deepcopy(cfg)
    rule = get_rule(updated, target.label)
    x, y, z = vector(actual_xyz, "실제 집기점의 모델 XYZ", 3)
    if shelf is not None or calibration.get("layout") == "two-shelves":
        actual_shelf = shelf_for(calibration, target.point(rule))
        if shelf is not None and shelf != actual_shelf:
            raise ValueError(f"선택한 상품이 {shelf} 선반에 없습니다")
        base = pick_point(cfg, calibration, target, apply_shelf=False)
        updated.setdefault("shelf_adjustments", {})[actual_shelf] = {
            "delta_xyz_mm": [value - origin for value, origin in zip((x, y, z), base)]}
        return updated
    xy = to_xy(calibration, target.point(rule))
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
            print("[TORQUE_OFF] ID 11~16 해제. 이제 수동으로 실제로 잘 잡히는 그리퍼 안쪽 위치·집기 높이에 맞추세요.")
            return True
        except BaseException:
            if not releasing:
                driver.hold()
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/omx_pick.json")
    parser.add_argument("--class", dest="label")
    parser.add_argument("--shelf", choices=("left", "right"), help="보정할 선반 (생략하면 검출 위치로 자동 선택)")
    parser.add_argument("--target-index", type=int, default=0)
    parser.add_argument("--from-folded", action="store_true", help="접힌 출발: ID13 → ID12 → HIGH")
    parser.add_argument("--hover-mm", type=float, default=5, help="집기 Z 위 정지 높이 mm (기본 5)")
    parser.add_argument("--stage-seconds", type=float, help="이번 실행의 일반 팔 이동 시간 (초)")
    parser.add_argument("--xy-record", action="store_true", help="수직·손목 고정 WASD/RF XYZ 조절 후 기록만 저장")
    parser.add_argument("--point-name", help="XY 기록 위치 이름, 예: left_front")
    parser.add_argument("--step-mm", type=float, default=10, help="조절 속도 분자 mm (step-mm / jog-seconds, 기본 10)")
    parser.add_argument("--jog-seconds", type=float, default=0.4, help="조절 속도 분모 초 (기본 0.4, 즉 기본 25mm/s)")
    parser.add_argument("--samples-file", type=Path, default=ROOT / "records/single/omx_xy_alignment_samples.jsonl")
    args = parser.parse_args(argv)
    if args.xy_record:
        if not args.point_name or not args.point_name.strip():
            parser.error("--xy-record에는 --point-name이 필요합니다")
        for name, value in (("step-mm", args.step_mm), ("jog-seconds", args.jog_seconds)):
            if number(value, name, 0) == 0:
                parser.error(f"{name}은 양수여야 합니다")
        number(args.hover_mm, "hover-mm", 0)
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
    if args.shelf:
        if calibration.get("layout") != "two-shelves" or len(calibration.get("regions_uv", [])) != 2:
            raise ValueError("--shelf에는 좌우 선반 캘리브레이션이 필요합니다")
        index = ("left", "right").index(args.shelf)
        cfg = dict(cfg, regions_uv=[calibration["regions_uv"][index]])
    print("상품 한 개를 시험 위치에 두세요. 촬영 후에는 상품을 움직이지 마세요.")
    detector = Detector(cfg)
    with Camera(profile) as camera:
        target, frame, _ = capture_target(camera, detector, cfg, label, args.target_index)
    print(f"선택 상품={target.label}, 영상 집기점={target.point(rule)}")
    shelf = shelf_for(calibration, target.point(rule)) if calibration.get("layout") == "two-shelves" else None
    if args.shelf and shelf != args.shelf:
        raise ValueError(f"선택한 상품이 {args.shelf} 선반에 없습니다")
    if shelf and not args.xy_record:
        print(f"보정 선반={shelf}; 이 선반의 상품에 공통으로 적용합니다.")
    if args.xy_record:
        print(f"기록 선반={shelf}; 화면을 닫으면 열린 그리퍼로 자동 이동합니다. 설정에는 반영하지 않습니다.")
    print("화면의 박스와 십자가가 보정할 상품인지 확인하고 아무 키로 닫으세요.")
    show(frame, [target], target, cfg)
    if args.xy_record:
        return record_xy(args, cfg, calibration, target, profile, shelf)
    saved = saved_for(profile, True)
    reference = reference_for(cfg, target.label, profile)
    plan = select_grasp_plan(reference, saved["hover_high"],
        lambda solver: approach_plan(cfg, calibration, target, saved, solver,
                                     args.hover_mm, args.from_folded),
        wrist_policy=cfg.get("wrist_policy", "recorded"))
    print(f"집기 XYZ mm={plan['pick_model_xyz_mm']}, 낮은 호버 XYZ mm={plan['low_hover_xyz_mm']}")
    print("순서: " + " → ".join(s["name"] for s in plan["stages"]))
    print("그리퍼를 자동 개폐하거나 상품을 들어 올리지 않습니다.")
    if input("위 위치로 이동하려면 MOVE (다른 입력: 취소): ").strip() != "MOVE":
        return 0
    if not move_and_release(profile, cfg, plan):
        return 0
    print("평소 집기 방향을 유지하며, 실제로 잘 잡히는 그리퍼 안쪽의 집기 위치와 높이로 맞추세요. 상품은 옮기지 마세요.")
    print(f"{args.hover_mm:g}mm 위 호버 자세를 그대로 기록하지 말고, 실제로 잡을 높이까지 수동으로 맞추세요.")
    print("상품은 움직이지 마세요. 기존 제어를 썼다면 종료한 뒤, 팔을 받친 상태로 읽으세요.")
    if input("실제 집기 위치에 맞췄으면 Enter로 읽기 (Q: 취소): ").strip().upper() == "Q":
        return 0
    snapshot = read_snapshot(profile)
    # Preserve the file's other settings, including a manually chosen ROI.
    original = load_config(args.config)
    updated = corrected_config(original, calibration, target, snapshot["model_xyz_mm"], shelf)
    revised = get_rule(updated, label)
    if shelf:
        print(f"{shelf} XYZ 추가 보정 mm: "
              f"{original.get('shelf_adjustments', {}).get(shelf, {}).get('delta_xyz_mm', [0, 0, 0])}"
              f" -> {updated['shelf_adjustments'][shelf]['delta_xyz_mm']}")
        print("선택한 선반의 추가 보정만 저장합니다. 상품별 값·반대 선반·캘리 행렬·드랍 자세는 유지합니다.")
    else:
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
