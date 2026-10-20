#!/usr/bin/env python3
"""Read-only OMX-F motor communication preflight.

Sends Dynamixel Protocol 2.0 PING and READ instructions only. It does not
enable torque, configure motors, calibrate, or command motion. Opening a USB
serial port may reset the OpenRB controller; keep the robot workspace clear.
Run with the existing LeRobot environment:
    /home/choi-chun-hwan/venv/il/bin/python -B 01_omx_control_preflight.py
"""

from __future__ import annotations

import argparse
import stat
import subprocess
import time
from pathlib import Path
from omx_arm_config import load_arm_profile

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


DEFAULT_PORT = Path(
    "/dev/serial/by-id/"
    "usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00"
)
PROTOCOL_VERSION = 2.0
DEFAULT_BAUDRATE = 1_000_000

# Checked against the local OMX-F LeRobot motor map and ROBOTIS X-series manuals.
EXPECTED_MOTORS = {
    11: ("shoulder_pan", 1060, "XL430-W250"),
    12: ("shoulder_lift", 1060, "XL430-W250"),
    13: ("elbow_flex", 1060, "XL430-W250"),
    14: ("wrist_flex", 1200, "XL330-M288"),
    15: ("wrist_roll", 1200, "XL330-M288"),
    16: ("gripper", 1200, "XL330-M288"),
}
# Each tuple is (control-table address, bytes); all are read with READ packets.
READ_FIELDS = {
    "firmware": (6, 1),
    "operating_mode": (11, 1),
    "max_voltage_limit": (32, 2),
    "min_voltage_limit": (34, 2),
    "startup_configuration": (60, 1),
    "shutdown_mask": (63, 1),
    "torque_enabled": (64, 1),
    "hardware_error": (70, 1),
    "present_position": (132, 4),
    "input_voltage": (144, 2),
    "temperature": (146, 1),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OMX-F 모터 ID·모델·상태를 읽기만 합니다. 토크/설정/동작 명령은 없습니다."
    )
    parser.add_argument("--port", type=Path, default=None)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    return parser.parse_args()


def check_port(path: Path) -> Path:
    if not path.exists():
        raise RuntimeError(f"직렬 포트가 없습니다: {path}")
    resolved = path.resolve(strict=True)
    if not stat.S_ISCHR(resolved.stat().st_mode):
        raise RuntimeError(f"문자 장치가 아닙니다: {resolved}")
    if not str(resolved).startswith("/dev/"):
        raise RuntimeError(f"/dev/ 아래의 장치만 허용합니다: {resolved}")
    try:
        busy = subprocess.run(
            ["fuser", "-s", str(resolved)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("포트 점유를 확인할 fuser 명령이 없습니다.") from exc
    if busy.returncode == 0:
        raise RuntimeError(f"다른 프로세스가 직렬 포트를 사용 중입니다: {resolved}")
    if busy.returncode != 1:
        raise RuntimeError(f"포트 점유 확인에 실패했습니다: fuser={busy.returncode}")
    return resolved


def read_value(
    packet: PacketHandler, port: PortHandler, motor_id: int, address: int, size: int
) -> tuple[int | None, str | None]:
    reader = {
        1: packet.read1ByteTxRx,
        2: packet.read2ByteTxRx,
        4: packet.read4ByteTxRx,
    }[size]
    value, result, error = reader(port, motor_id, address)
    if result != COMM_SUCCESS:
        return None, packet.getTxRxResult(result)
    if error == 0x80:
        return value, "status alert 0x80"
    if error:
        return value, f"status error 0x{error:02X}: {packet.getRxPacketError(error)}"
    return value, None


def signed_32(value: int) -> int:
    return value - (1 << 32) if value & (1 << 31) else value


def inspect_motor(packet: PacketHandler, port: PortHandler, motor_id: int) -> bool:
    name, expected_number, expected_model = EXPECTED_MOTORS[motor_id]
    for attempt in range(3):
        number, result, error = packet.ping(port, motor_id)
        if result == COMM_SUCCESS:
            break
        if attempt < 2:
            time.sleep(0.2)
    if result != COMM_SUCCESS:
        print(f"[FAIL] ID {motor_id:2d} {name}: PING {packet.getTxRxResult(result)}")
        return False
    ping_issue = f"PING status alert/error 0x{error:02X}" if error else None
    if number != expected_number:
        print(
            f"[FAIL] ID {motor_id:2d} {name}: model={number}, "
            f"expected={expected_number} ({expected_model}); 상태 레지스터 읽기 생략"
        )
        return False

    values: dict[str, int | None] = {}
    issues: list[str] = [ping_issue] if ping_issue else []
    for field, (address, size) in READ_FIELDS.items():
        value, issue = read_value(packet, port, motor_id, address, size)
        values[field] = value
        if issue:
            detail = issue if issue == "status alert 0x80" else f"{field}: {issue}"
            if detail not in issues:
                issues.append(detail)

    position = values["present_position"]
    voltage = values["input_voltage"]
    status = "OK" if not issues and values["hardware_error"] == 0 else "CHECK"
    position_text = str(signed_32(position)) if position is not None else "?"
    voltage_text = f"{voltage / 10:.1f}V" if voltage is not None else "?"
    print(
        f"[{status}] ID {motor_id:2d} {name:14s} {expected_model:10s} "
        f"fw={values['firmware']} mode={values['operating_mode']} "
        f"torque={values['torque_enabled']} "
        f"position={position_text} pulse voltage={voltage_text}"
    )
    minimum = values["min_voltage_limit"]
    maximum = values["max_voltage_limit"]
    limits = (
        f"{minimum / 10:.1f}..{maximum / 10:.1f}V"
        if minimum is not None and maximum is not None else "?"
    )
    print(
        f"       temperature={values['temperature']} C "
        f"hardware_error={values['hardware_error']} "
        f"voltage_limits={limits} startup={values['startup_configuration']} "
        f"shutdown_mask={values['shutdown_mask']}"
    )
    for issue in issues:
        print(f"       status: {issue}")
    if values["hardware_error"] not in (None, 0):
        print(f"       모터 하드웨어 오류 비트: 0x{values['hardware_error']:02X}")
        if values["hardware_error"] & 0x01:
            print("       비트 0: 입력 전압 오류")
            if voltage is not None and minimum is not None and maximum is not None:
                within = minimum <= voltage <= maximum
                print(f"       현재 전압은 설정 한계 {'안' if within else '밖'}에 있습니다.")
    return not issues and values["hardware_error"] == 0


def main() -> int:
    args = parse_args()
    if args.baudrate != DEFAULT_BAUDRATE:
        raise SystemExit("첫 점검은 기존 OMX-F 설정인 1000000 bps만 허용합니다.")
    args.port = args.port or load_arm_profile(None).port
    port_path = check_port(args.port)
    print(f"port={args.port} -> {port_path}")
    print(f"protocol={PROTOCOL_VERSION}, baudrate={args.baudrate}")
    print("PING/READ만 실행합니다. 모터 설정, 토크, 목표 위치는 변경하지 않습니다.")

    port = PortHandler(str(port_path))
    packet = PacketHandler(PROTOCOL_VERSION)
    try:
        if not port.openPort():
            raise RuntimeError(f"직렬 포트를 열지 못했습니다: {port_path}")
        time.sleep(1.5)  # Allow the USB controller to settle after serial open.
        success = [inspect_motor(packet, port, motor_id) for motor_id in EXPECTED_MOTORS]
    finally:
        if port.is_open:
            port.closePort()

    if all(success):
        print("[PASS] 예상 모터 6개가 응답했고 읽은 하드웨어 오류는 0입니다.")
        return 0
    print("[INCOMPLETE] 통신·모델·상태에 확인이 필요한 항목이 있습니다.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
