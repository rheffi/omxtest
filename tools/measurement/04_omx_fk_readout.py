#!/usr/bin/env python3
"""Read OMX-F joint positions and show an explicitly unvalidated URDF FK estimate.

Only Dynamixel PING and READ packets are sent. No torque, configuration, or
goal-position writes are made. Opening the serial port can reset OpenRB.

The default 2048-pulse zero and positive joint directions are hypotheses, not
an established mapping from this arm's encoders to the OMX-F URDF. The model
point is end_effector_link, not the measured midpoint between gripper fingers.
"""

from __future__ import annotations

from omx_motor_frame import frame_packet, frame_metadata

import argparse
import importlib.util
import json
import math
import stat
import subprocess
import sys
import time
from pathlib import Path
from omx_arm_config import load_arm_profile

import numpy as np


HERE = Path(__file__).resolve().parent
DEFAULT_PORT = Path(
    "/dev/serial/by-id/"
    "usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00"
)
MOTORS = (
    (11, "shoulder_pan", 1060),
    (12, "shoulder_lift", 1060),
    (13, "elbow_flex", 1060),
    (14, "wrist_flex", 1200),
    (15, "wrist_roll", 1200),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OMX-F 현재 모터값 읽기와 검증 전 FK 추정")
    parser.add_argument("--port", type=Path, default=None)
    parser.add_argument("--raw-json", action="store_true",
                        help="현재 관절 펄스 5개만 JSON으로 출력; PING/READ만 실행")
    parser.add_argument("--raw", nargs=5, type=int, metavar="PULSE",
                        help="직렬 연결 없이 다섯 관절의 기록된 펄스로 계산")
    parser.add_argument("--zero", nargs=5, type=int, default=[2048] * 5,
                        metavar="PULSE", help="관절별 URDF 0 rad에 해당하는 펄스 후보")
    parser.add_argument("--sign", nargs=5, type=int, default=[1] * 5,
                        metavar="DIR", help="관절별 펄스 증가 방향 후보: 1 또는 -1")
    parser.add_argument("--measured-gripper-xy-mm", nargs=2, type=float,
                        metavar=("X", "Y"),
                        help="자와 기준점으로 직접 측정한 그리퍼 중심 XY. 모델 말단점과 비교만 함")
    return parser.parse_args()


def checked_port(path: Path) -> Path:
    resolved = path.resolve(strict=True)
    if not stat.S_ISCHR(resolved.stat().st_mode) or not str(resolved).startswith("/dev/"):
        raise RuntimeError(f"/dev 아래의 직렬 문자 장치가 아닙니다: {resolved}")
    result = subprocess.run(["fuser", "-s", str(resolved)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            check=False)
    if result.returncode == 0:
        raise RuntimeError(f"다른 프로세스가 포트를 사용 중입니다: {resolved}")
    if result.returncode != 1:
        raise RuntimeError(f"포트 점유 확인 실패: fuser={result.returncode}")
    return resolved


def signed_32(value: int) -> int:
    return value - (1 << 32) if value & (1 << 31) else value


def read_positions(path: Path, *, verbose: bool = True) -> list[int]:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

    port = PortHandler(str(checked_port(path)))
    packet = frame_packet(PacketHandler(2.0), path)
    positions: list[int] = []
    try:
        if not port.openPort():
            raise RuntimeError(f"포트를 열지 못했습니다: {path}")
        if not port.setBaudRate(1_000_000):
            raise RuntimeError("기존 OMX-F 통신 속도 1000000 bps 설정 실패")
        time.sleep(1.5)
        for motor_id, name, expected_model in MOTORS:
            model, result, error = packet.ping(port, motor_id)
            if result != COMM_SUCCESS or error or model != expected_model:
                raise RuntimeError(f"ID {motor_id} {name}: 모델/통신 확인 실패 "
                                   f"(model={model}, result={result}, error={error})")
            torque, result, error = packet.read1ByteTxRx(port, motor_id, 64)
            if result != COMM_SUCCESS or error:
                raise RuntimeError(f"ID {motor_id} 토크 상태 READ 실패")
            hardware_error, result, error = packet.read1ByteTxRx(port, motor_id, 70)
            if result != COMM_SUCCESS or error or hardware_error:
                raise RuntimeError(f"ID {motor_id} 하드웨어 상태 확인 실패 "
                                   f"(error={hardware_error}, result={result})")
            raw, result, error = packet.read4ByteTxRx(port, motor_id, 132)
            if result != COMM_SUCCESS or error:
                raise RuntimeError(f"ID {motor_id} 현재 위치 READ 실패")
            position = signed_32(raw)
            positions.append(position)
            if verbose:
                print(f"ID {motor_id} {name:14s} present={position:6d} pulse, torque={torque}")
    finally:
        if port.is_open:
            port.closePort()
    return positions


def load_fk():
    path = HERE / "02_omx_xyz_offline.py"
    spec = importlib.util.spec_from_file_location("omx_xyz_offline", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"FK 모듈을 읽지 못했습니다: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.forward


def main() -> int:
    args = parse_args()
    if any(sign not in (-1, 1) for sign in args.sign):
        raise ValueError("--sign은 각 관절에 1 또는 -1만 입력하세요")
    if args.raw_json and args.raw is not None:
        raise ValueError("--raw-json과 --raw는 함께 사용할 수 없습니다")
    if args.raw is None:
        if not args.raw_json:
            print("PING/READ만 전송합니다. OpenRB 직렬 포트 열기로 컨트롤러가 재시작될 수 있습니다.")
        positions = read_positions(args.port or load_arm_profile(None).port, verbose=not args.raw_json)
    else:
        positions = args.raw
        print("OFFLINE: 입력 펄스만 사용하며 장비에 연결하지 않습니다.")

    if args.raw_json:
        print(json.dumps(positions))
        return 0

    angles = np.asarray([(p - z) * s * 2.0 * math.pi / 4096.0
                         for p, z, s in zip(positions, args.zero, args.sign)], dtype=float)
    point, _ = load_fk()(angles)
    print("\n[미검증 추정] 4096 펄스/회전, 입력한 영점·방향으로 계산")
    print(f"zero_pulses={args.zero} sign={args.sign}")
    for (_, name, _), angle in zip(MOTORS, angles):
        print(f"{name:14s} candidate_urdf={angle:+.4f} rad ({math.degrees(angle):+.1f} deg)")
    print("frame=link0, point=end_effector_link (실제 그리퍼 중심 아님)")
    print(f"candidate_xyz_mm={np.round(point * 1000.0, 1).tolist()}")
    if args.measured_gripper_xy_mm is not None:
        measured = np.asarray(args.measured_gripper_xy_mm)
        delta = point[:2] * 1000.0 - measured
        print(f"measured_gripper_xy_mm={measured.tolist()}")
        print(f"model_minus_gripper_xy_mm={np.round(delta, 1).tolist()}")
        print("이 차이에는 관절 영점·방향 오차와 그리퍼 중심 오프셋이 함께 들어 있습니다.")
    print("현재 출력만으로 실물 XY 정확도나 좌표 이동 가능 여부를 판정하지 마세요.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
