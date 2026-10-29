#!/usr/bin/env python3
"""Three-item field trial reusing the existing detector, planner and driver."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import sys

from omx_pick import (ROOT, legacy, profile_for, require_job_arm, calibration_for,
                      reference_for, build_pick_plan)
from pick_core import load_config, get_rule, execute
from pick_robot import DynamixelBus, MotionController
from pick_vision import Camera, Detector, capture_target
from omx_wrist import canonical_wrist

ITEMS = (("jin_ramen", 0), ("rice_chip", 1), ("shin_ramen", 2))


def main():
    parser = argparse.ArgumentParser(description="상품 3개를 지정 순서대로 슬롯0/1/2에 적재. --live는 즉시 실행")
    parser.add_argument("--config", type=Path, default=ROOT / "config/omx_pick_xy_trial.json")
    parser.add_argument("--classes", nargs=3, metavar=("SLOT0", "SLOT1", "SLOT2"),
                        default=[label for label, _ in ITEMS],
                        help="슬롯0/1/2 상품 클래스 (기본: jin_ramen rice_chip shin_ramen)")
    parser.add_argument("--from-folded", action="store_true", help="첫 상품 전에만 ID13→ID12→HIGH")
    parser.add_argument("--stage-seconds", type=float, default=6, help="팔 이동 구간당 시간; 기본 6초")
    parser.add_argument("--live", action="store_true", help="MOVE 입력 없이 3회 연속 실물 실행")
    args = parser.parse_args()
    if not math.isfinite(args.stage_seconds) or args.stage_seconds <= 0:
        parser.error("stage-seconds는 양수여야 합니다")
    cfg = load_config(args.config)
    cfg = dict(cfg, stage_seconds=args.stage_seconds, unfold_seconds=args.stage_seconds)
    profile = profile_for(None)
    require_job_arm(cfg, profile)
    recorder = legacy("07_omx_pose_recorder.py", "three_pose_recorder")
    poses = recorder.load_registry(profile)["poses"]
    jobs = []
    for slot, label in enumerate(args.classes):
        calibration, _ = calibration_for(profile, get_rule(cfg, label), cfg)
        job_cfg = dict(cfg, regions_uv=calibration["regions_uv"]) if cfg.get("regions_source") == "calibration" else cfg
        saved = {key: poses[name]["raw_pulses"] for key, name in {
            "hover_high": "hover_high", "drop_approach": f"slot{slot}_approach",
            "drop_release": f"slot{slot}_release"}.items()}
        jobs.append((label, slot, job_cfg, calibration, saved, reference_for(cfg, label, profile)))
    print(f"config={args.config.resolve()}; live={args.live}", flush=True)
    print(" / ".join(f"{label} → slot{slot}" for slot, label in enumerate(args.classes)), flush=True)
    detector = Detector(cfg)
    with ExitStack() as stack:
        camera = stack.enter_context(Camera(profile))
        driver = None
        fixed = None
        if args.live:
            bus = stack.enter_context(DynamixelBus(profile.port))
            driver = MotionController(bus, cfg)
            driver.prepare(use_gripper=True)
            if cfg.get("wrist_policy") == "fixed":
                fixed = canonical_wrist(bus.read(15, 132, 4))
        for index, (label, slot, job_cfg, calibration, saved, reference) in enumerate(jobs):
            try:
                target, frame, _ = capture_target(camera, detector, job_cfg, label)
                if frame.shape[:2] != (calibration["height"], calibration["width"]):
                    raise ValueError("카메라와 캘리 해상도가 다릅니다")
                plan = build_pick_plan(job_cfg, calibration, target, saved, reference,
                    from_folded=args.from_folded and index == 0, fixed_wrist_pulse=fixed,
                    profile=profile, drop_slot=slot)
                print(f"[{index+1}/3] {label} → slot{slot}; 집기 XYZ={plan['pick_model_xyz_mm']}", flush=True)
                if driver is not None and index == 0:
                    driver.enable()
            except BaseException:
                if driver is not None:
                    driver.hold()
                raise
            if driver is not None:
                execute(plan, driver)  # Existing executor holds on errors/Ctrl+C.
                print(f"[{index+1}/3 SENT] 집기·적재·HIGH 복귀 목표 전송 종료", flush=True)
            else:
                print("[PLAN ONLY] " + " → ".join(s["name"] for s in plan["stages"]))
    print("[END] 실물 집기·적재 결과는 직접 확인하세요. 실행 후 팔·그리퍼 토크 유지." if args.live
          else "[END] 모터 연결 없이 세 상품 경로 계산 종료.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] 중단. 팔·그리퍼 토크는 유지됩니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1)
