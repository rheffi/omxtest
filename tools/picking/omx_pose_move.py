#!/usr/bin/env python3
"""Move to named saved poses using the same motor frame as the pick runner."""
from __future__ import annotations

import argparse
import math
import sys

from omx_pick import legacy
from omx_arm_config import load_arm_profile
from pick_robot import DynamixelBus, MotionController, torque_off


def main():
    parser = argparse.ArgumentParser(description="저장 자세 바로 이동. MOVE 입력 없이 실행하며 종료 후 토크 유지")
    parser.add_argument("poses", nargs="*", metavar="POSE", help="저장 자세 이름. 여러 개면 입력 순서대로 이동")
    parser.add_argument("--seconds", type=float, default=6, help="자세당 이동 시간, 기본 6초")
    parser.add_argument("--from-folded", action="store_true", help="13번 펼침 → 12번 세움 → HIGH를 먼저 실행")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="저장 이름·값 조회, 모터 연결 없음")
    mode.add_argument("--torque-off", action="store_true", help="팔·그리퍼 ID11~16 토크 즉시 해제")
    args = parser.parse_args()
    if (args.list or args.torque_off) and (args.poses or args.from_folded):
        parser.error("조회·토크 해제와 자세 이동은 따로 실행하세요")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds는 양수여야 합니다")
    if not (args.poses or args.list or args.torque_off):
        parser.error("자세 이름 또는 --list / --torque-off를 지정하세요")
    profile = load_arm_profile(None)
    if args.torque_off:
        with DynamixelBus(profile.port) as bus:
            states = torque_off(bus)
            print(states)
            return 0 if all(row["confirmed"] for row in states.values()) else 1

    recorder = legacy("07_omx_pose_recorder.py", "pose_move_recorder")
    poses = recorder.load_registry(profile)["poses"]
    if args.list:
        for name, pose in poses.items():
            print(f"{name}: {pose['raw_pulses']}")
        return 0
    needed = list(args.poses)
    if args.from_folded:
        needed += ["unfold_elbow", "unfold_shoulder", "hover_high"]
    for name in needed:
        if name not in poses:
            parser.error(f"저장되지 않은 자세: {name}; --list로 조회하세요")
    cfg = {"stage_seconds": args.seconds, "unfold_seconds": args.seconds, "stage_pause_seconds": 0}
    print(f"poses={profile.poses}; 구간당 {args.seconds:g}초", flush=True)
    with DynamixelBus(profile.port) as bus:
        motion = MotionController(bus, cfg)
        try:
            motion.prepare(use_gripper=False)
            motion.enable()
            if args.from_folded:
                motion.joint("ELBOW_FIRST", {13: poses["unfold_elbow"]["raw_pulses"][2]})
                motion.joint("SHOULDER_SECOND", {12: poses["unfold_shoulder"]["raw_pulses"][1]})
                motion.arm("hover_high", poses["hover_high"]["raw_pulses"])
            for index, name in enumerate(args.poses):
                if args.from_folded and index == 0 and name == "hover_high":
                    continue
                motion.arm(name, poses[name]["raw_pulses"])
        except BaseException:
            motion.hold()
            raise
    print("[SENT] 목표 전송 종료. 실제 자세는 직접 확인하세요. 팔 토크 유지.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] 중단. 팔 토크는 유지됩니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1)
