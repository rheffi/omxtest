#!/usr/bin/env python3
"""One small, supervised OMX-F wrist-roll motor step and return.

Default is READ ONLY. --execute commands ONLY ID 15 by +13 raw pulses and
returns to the measured start. No motor configuration or calibration writes.
The arm's other joint goals are unchanged.
Run with /home/choi-chun-hwan/venv/il/bin/python -B.
"""

from __future__ import annotations

import argparse
import stat
import subprocess
import time
from pathlib import Path
from omx_arm_config import load_arm_profile

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = Path(
    "/dev/serial/by-id/"
    "usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00"
)
BAUD = 1_000_000
MOTOR_IDS = {11: 1060, 12: 1060, 13: 1060, 14: 1200, 15: 1200, 16: 1200}
WRIST_ID = 15
STEP_PULSES = 13
TORQUE_ENABLE = 64
HARDWARE_ERROR = 70
GOAL_POSITION = 116
PRESENT_POSITION = 132
OPERATING_MODE = 11


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute", action="store_true",
        help="ID 15만 +13펄스 이동 후 시작 위치로 복귀합니다.",
    )
    return parser.parse_args()


def check_port() -> Path:
    global PORT
    PORT = load_arm_profile(None).port
    if not PORT.exists():
        raise RuntimeError(f"OMX-F 포트가 없습니다: {PORT}")
    path = PORT.resolve(strict=True)
    if not stat.S_ISCHR(path.stat().st_mode) or not str(path).startswith("/dev/"):
        raise RuntimeError(f"올바른 직렬 장치가 아닙니다: {path}")
    result = subprocess.run(
        ["fuser", "-s", str(path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 1:
        raise RuntimeError(f"포트 점유/확인 오류: fuser={result.returncode}")
    return path


def check_reply(packet: PacketHandler, result: int, error: int, label: str) -> None:
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{label}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{label}: status 0x{error:02X} {packet.getRxPacketError(error)}")


def read(packet: PacketHandler, port: PortHandler, motor_id: int,
         label: str, address: int, size: int) -> int:
    method = {1: packet.read1ByteTxRx, 4: packet.read4ByteTxRx}[size]
    value, result, error = method(port, motor_id, address)
    check_reply(packet, result, error, f"ID {motor_id} {label} READ")
    if size == 4 and value & 0x80000000:
        return value - 0x100000000
    return value


def write(packet: PacketHandler, port: PortHandler, label: str,
          address: int, size: int, value: int) -> None:
    method = {1: packet.write1ByteTxRx, 4: packet.write4ByteTxRx}[size]
    result, error = method(port, WRIST_ID, address, value)
    check_reply(packet, result, error, f"ID {WRIST_ID} {label} WRITE")


def inspect(packet: PacketHandler, port: PortHandler) -> int:
    """Verify identity, modes and errors; return ID 15 present position."""
    positions = {}
    for motor_id, expected_model in MOTOR_IDS.items():
        number, result, error = packet.ping(port, motor_id)
        check_reply(packet, result, error, f"ID {motor_id} PING")
        if number != expected_model:
            raise RuntimeError(f"ID {motor_id} model {number} != {expected_model}")
        torque = read(packet, port, motor_id, "torque", TORQUE_ENABLE, 1)
        hardware_error = read(packet, port, motor_id, "hardware error", HARDWARE_ERROR, 1)
        position = read(packet, port, motor_id, "position", PRESENT_POSITION, 4)
        positions[motor_id] = position
        print(f"ID {motor_id}: torque={torque}, hardware_error={hardware_error}, position={position}")
        if hardware_error != 0:
            raise RuntimeError(f"ID {motor_id}: 하드웨어 오류가 있습니다")
    mode = read(packet, port, WRIST_ID, "operating mode", OPERATING_MODE, 1)
    if mode != 4:
        raise RuntimeError(f"ID 15 모드 {mode}: 예상한 Extended Position(4)가 아닙니다")
    return positions[WRIST_ID]


def exercise(packet: PacketHandler, port: PortHandler, start: int) -> None:
    target = start + STEP_PULSES
    if not 0 <= start <= 4095 or not 0 <= target <= 4095:
        raise ValueError("시작·목표 펄스는 0..4095여야 합니다")
    write(packet, port, "align goal", GOAL_POSITION, 4, start)
    try:
        write(packet, port, "enable torque", TORQUE_ENABLE, 1, 1)
        write(packet, port, "step goal", GOAL_POSITION, 4, target)
        time.sleep(1)
        write(packet, port, "return goal", GOAL_POSITION, 4, start)
        time.sleep(1)
        print(f"[GOALS_SENT] ID 15: {start} -> {target} -> {start}")
    finally:
        write(packet, port, "disable torque", TORQUE_ENABLE, 1, 0)


def main() -> int:
    args = parse_args()
    path = check_port()
    print(f"port={PORT} -> {path}; execute={args.execute}")
    port = PortHandler(str(path))
    packet = PacketHandler(2.0)
    try:
        if not port.openPort():
            raise RuntimeError("직렬 포트를 열지 못했습니다")
        if not port.setBaudRate(BAUD):
            raise RuntimeError("1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        start = inspect(packet, port)
        if not args.execute:
            print("[READ ONLY] 이동 없음. 실제 시험은 --execute를 명시해야 합니다.")
            return 0
        exercise(packet, port, start)
        return 0
    finally:
        if port.is_open:
            port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
