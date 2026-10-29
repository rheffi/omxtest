#!/usr/bin/env python3
"""Supervised small ID 12 shoulder step while other OMX-F arm goals stay fixed.

--raw is offline. --execute reads motor state and, after two explicit prompts,
writes only ID 12 Goal Position. It never changes torque or motor settings.
"""

from __future__ import annotations

from omx_motor_frame import frame_packet, frame_metadata

import argparse
import importlib.util
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SHOULDER_ID = 12
SHOULDER_INDEX = 1


def load_hover():
    path = HERE / "06_omx_xy_hover.py"
    spec = importlib.util.spec_from_file_location("omx_hover_for_shoulder_check", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"OMX 호버 모듈을 읽지 못했습니다: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def inspect_candidate(hover, current, delta):
    current = np.asarray(current, dtype=int)
    if current.shape != (5,) or np.any(current < 0) or np.any(current > 4095):
        raise ValueError("관절값은 0..4095 펄스 5개여야 합니다")
    target = current.copy()
    target[SHOULDER_INDEX] += delta
    if not 0 <= target[SHOULDER_INDEX] <= 4095:
        raise ValueError("ID 12 목표값은 0..4095여야 합니다")
    fk = hover.load_module("02_omx_xyz_offline.py", "omx_fk_for_shoulder_check")
    points = []
    for candidate in (current, target):
        position, _ = fk.forward(hover.pulse_to_q(candidate))
        points.append(position * 1000)
    print(f"현재 팔 펄스={current.tolist()}")
    print(f"ID 12만 {current[SHOULDER_INDEX]} -> {target[SHOULDER_INDEX]} 펄스")
    print(f"모델 말단 현재 XYZ={np.round(points[0], 1).tolist()}mm")
    print(f"모델 말단 목표 XYZ={np.round(points[-1], 1).tolist()}mm")
    return target


def require_holding_state(hover, state):
    if state["torques"][SHOULDER_INDEX] != 1:
        raise RuntimeError("ID 12 토크가 ON이어야 움직입니다. 이 도구는 토크를 켜지 않습니다")
    current = np.asarray(state["positions"], dtype=int)
    return current


def execute_step(hover, delta):
    if input("팔을 받치고 실제 간격·주변을 확인했다면 SUPPORTED 입력 (취소는 Enter): ").strip() != "SUPPORTED":
        print("[CANCELLED] 포트를 열거나 모터 명령을 보내지 않았습니다")
        return 1
    from dynamixel_sdk import PacketHandler, PortHandler
    path = hover.check_port()
    port = PortHandler(str(path))
    packet = frame_packet(PacketHandler(2.0), path)
    goal_sent = False
    try:
        if not port.openPort() or not port.setBaudRate(1_000_000):
            raise RuntimeError("직렬 포트 또는 1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        state = hover.read_motor_state(packet, port)
        current = require_holding_state(hover, state)
        target = inspect_candidate(hover, current, delta)
        print("모델은 팔 몸체·그리퍼·작업대 충돌을 확인하지 못합니다. 실제 여유를 직접 보세요.")
        if input("이 한 번의 ID 12 이동을 실행하려면 STEP12 입력 (취소는 Enter): ").strip() != "STEP12":
            print("[CANCELLED] 모터 목표값·토크를 변경하지 않았습니다")
            return 1
        current = hover.read_positions(packet, port)
        target = inspect_candidate(hover, current, delta)
        goal_sent = True
        hover.sdk_write(packet, port, SHOULDER_ID, 116, 4,
                        int(target[SHOULDER_INDEX]), "small shoulder goal")
        print(f"[GOAL_SENT] ID 12={target[SHOULDER_INDEX]}")
        return 0
    except BaseException:
        if goal_sent:
            try:
                present = hover.sdk_read(packet, port, SHOULDER_ID, 132, 4, "present to stop")
                hover.sdk_write(packet, port, SHOULDER_ID, 116, 4, present, "stop shoulder at present")
                print(f"[STOP12] ID 12 잔여 목표를 현재값 {present}로 변경했습니다", flush=True)
            except Exception as exc:
                print(f"[ALERT] ID 12 목표 취소 확인 실패: {exc}", file=sys.stderr)
        raise
    finally:
        if port.is_open:
            port.closePort()
        if goal_sent:
            print("[HOLD] 팔 관절 토크는 ON일 수 있습니다. 팔을 받치고 상태를 확인하세요.", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="OMX-F ID 12만 작은 폭으로 점검; 자동 기립 없음")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--raw", nargs=5, type=int, metavar=("P11", "P12", "P13", "P14", "P15"),
                       help="기록된 펄스로 모델 경로만 계산; 장비 연결 없음")
    group.add_argument("--execute", action="store_true", help="두 번의 사용자 확인 후 ID 12만 한 번 이동")
    parser.add_argument("--delta", type=int, default=4,
                        help="ID 12 상대 이동량 pulse (기본 4)")
    return parser.parse_args()


def main():
    args = parse_args()
    hover = load_hover()
    if args.raw is not None:
        inspect_candidate(hover, args.raw, args.delta)
        print("[DRY_RUN] 직렬 포트를 열거나 모터를 움직이지 않았습니다")
        return 0
    return execute_step(hover, args.delta)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[CANCELLED] 사용자 중단. 토크 상태를 직접 확인하세요.", file=sys.stderr)
        raise SystemExit(1)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
