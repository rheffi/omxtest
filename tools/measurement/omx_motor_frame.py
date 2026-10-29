"""Translate one reassembled ID14 between saved model pulses and hardware.

No SDK import, connection, torque change or EEPROM write. The packet adapter
only translates Goal/Present Position for the configured USB serial identity.
"""
from __future__ import annotations

import json
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config/single/omx_motor_frame.json"
TURN = 4096


def signed32(value):
    value = int(value)
    return value - (1 << 32) if value >= (1 << 31) else value


def usb_serial(path):
    path = Path(path)
    prefix = "usb-ROBOTIS_OpenRB-150_"
    if path.parent == Path("/dev/serial/by-id") and path.name.startswith(prefix):
        return path.name[len(prefix):].split("-if", 1)[0]
    # PortHandler often receives the resolved ttyACM node, not its by-id alias.
    device = Path("/sys/class/tty") / path.resolve().name / "device"
    for parent in device.resolve().parents:
        try:
            if "openrb" in (parent / "product").read_text().lower():
                return (parent / "serial").read_text().strip()
        except OSError:
            continue
    return None


class PositionFrame:
    def __init__(self, offset):
        self.offset = offset
        self.bias = None
        self.physical_present = None

    def observe(self, physical):
        physical = signed32(physical)
        if self.bias is None:
            # Choose the turn once per connection, using PRESENT, never GOAL.
            logical = (physical - self.offset) % TURN
            self.bias = physical - logical
        self.physical_present = physical
        return physical - self.bias

    def to_physical(self, logical):
        if self.bias is None:
            raise RuntimeError("ID14 현재 위치를 먼저 읽어야 합니다")
        return signed32(logical) + self.bias


class FramePacket:
    """SDK-compatible adapter; callers still decode signed 32-bit registers."""
    def __init__(self, packet, offset, serial):
        self.packet = packet
        self.frame = PositionFrame(offset)
        self.serial = serial

    def __getattr__(self, name):
        return getattr(self.packet, name)

    def _anchor(self, port):
        if self.frame.bias is None:
            value, result, error = self.packet.read4ByteTxRx(port, 14, 132)
            if result != 0 or error:
                raise RuntimeError(f"ID14 위치 기준 READ 실패: result={result}, status={error}")
            self.frame.observe(value)

    def read4ByteTxRx(self, port, motor_id, address):
        if motor_id == 14 and address == 116:
            self._anchor(port)
        value, result, error = self.packet.read4ByteTxRx(port, motor_id, address)
        if result == 0 and not error and motor_id == 14 and address in (116, 132):
            logical = (self.frame.observe(value) if address == 132
                       else signed32(value) - self.frame.bias)
            value = logical & 0xFFFFFFFF
        return value, result, error

    def write4ByteTxRx(self, port, motor_id, address, value):
        if motor_id == 14 and address == 116:
            self._anchor(port)
            value = self.frame.to_physical(value) & 0xFFFFFFFF
        return self.packet.write4ByteTxRx(port, motor_id, address, int(value))

    def metadata(self):
        return {"coordinate_frame": "original_model_pulses", "motor_id": 14,
                "usb_serial": self.serial, "physical_offset_pulses": self.frame.offset,
                "connection_bias_pulses": self.frame.bias,
                "physical_present_pulse": self.frame.physical_present}


def frame_metadata(packet):
    return packet.metadata() if isinstance(packet, FramePacket) else None


def frame_packet(packet, port_path, config_file=None):
    path = Path(config_file) if config_file is not None else DEFAULT_CONFIG
    if config_file is None and not path.exists():
        return packet
    data = json.loads(path.read_text(encoding="utf-8"))
    if (data.get("schema_version") != 1 or data.get("motor_id") != 14
            or type(data.get("physical_offset_pulses")) is not int
            or abs(data["physical_offset_pulses"]) >= TURN
            or not isinstance(data.get("usb_serial"), str) or not data["usb_serial"]):
        raise ValueError(f"ID14 위치 변환 설정 형식 오류: {path}")
    serial = usb_serial(port_path)
    if serial is None:
        raise RuntimeError("ID14 보정 대상 확인을 위한 OpenRB USB serial을 읽지 못했습니다")
    if serial != data["usb_serial"]:
        return packet  # Other arms, including laptop B, keep their original frame.
    return FramePacket(packet, data["physical_offset_pulses"], serial)
