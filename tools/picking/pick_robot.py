"""Linux Dynamixel transport and smooth single-arm goal transmission.

Imports no SDK until a real connection is explicitly requested. Operating modes
and EEPROM settings are never changed. Exceptions hold goals, keeping torque.
"""

from __future__ import annotations

import math
import stat
import subprocess
import sys
import time

from pick_core import ARM_IDS, MODELS, pulses, validate_gripper


class DynamixelBus:
    def __init__(self, path):
        self.path = path
        self.port = None

    def __enter__(self):
        if not sys.platform.startswith("linux"):
            raise RuntimeError("실물 모터 제어는 Ubuntu/Linux에서 실행하세요")
        try:
            resolved = self.path.resolve(strict=True)
        except FileNotFoundError:
            raise RuntimeError("연결된 OMX-F USB 직렬 장치가 없습니다. 로봇 USB를 다시 연결하세요.") from None
        if not str(resolved).startswith("/dev/") or not stat.S_ISCHR(resolved.stat().st_mode):
            raise ValueError(f"직렬 문자 장치가 아닙니다: {resolved}")
        check = subprocess.run(["fuser", "-s", str(resolved)], capture_output=True, check=False)
        if check.returncode != 1:
            raise RuntimeError(f"직렬 포트 사용 중 또는 점유 확인 실패: {resolved}")
        from dynamixel_sdk import PacketHandler, PortHandler
        self.packet = PacketHandler(2.0)
        self.port = PortHandler(str(resolved))
        try:
            if not self.port.openPort() or not self.port.setBaudRate(1_000_000):
                raise RuntimeError("직렬 포트/1000000 bps 연결 실패")
            time.sleep(1.5)
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *args):
        if self.port is not None and self.port.is_open:
            self.port.closePort()

    def check(self, result, error, label):
        from dynamixel_sdk import COMM_SUCCESS
        if result != COMM_SUCCESS or error:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(result)}, status={error}")

    def ping(self, motor_id):
        model, result, error = self.packet.ping(self.port, motor_id)
        self.check(result, error, f"ID {motor_id} PING")
        return model

    def read(self, motor_id, address, size):
        reader = {1: self.packet.read1ByteTxRx, 2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, result, error = reader(self.port, motor_id, address)
        self.check(result, error, f"ID {motor_id} READ {address}")
        if (address in (20, 116, 132) and value & (1 << 31)):
            return value - (1 << 32)
        if address == 126 and value & (1 << 15):
            return value - (1 << 16)
        return value

    def write(self, motor_id, address, size, value):
        writer = {1: self.packet.write1ByteTxRx, 2: self.packet.write2ByteTxRx,
                  4: self.packet.write4ByteTxRx}[size]
        result, error = writer(self.port, motor_id, address, int(value))
        self.check(result, error, f"ID {motor_id} WRITE {address}")


def inspect(bus):
    rows = []
    for motor_id in (*ARM_IDS, 16):
        rows.append({"id": motor_id, "model": bus.ping(motor_id),
                     "mode": bus.read(motor_id, 11, 1), "homing_offset": bus.read(motor_id, 20, 4),
                     "torque": bus.read(motor_id, 64, 1), "hardware_error": bus.read(motor_id, 70, 1),
                     "position": bus.read(motor_id, 132, 4), "temperature_c": bus.read(motor_id, 146, 1),
                     "voltage_v": bus.read(motor_id, 144, 2) / 10,
                     "current_limit_raw": bus.read(motor_id, 38, 2) if motor_id == 16 else None,
                     "goal_current_raw": bus.read(motor_id, 102, 2) if motor_id == 16 else None})
    return rows


class MotionController:
    simulation = False

    def __init__(self, bus, cfg, emit=lambda *args: None):
        self.bus, self.cfg, self.emit = bus, cfg, emit
        self.ids = []
        self.touched = False

    def prepare(self, high_pose=None, *, use_gripper=True, gripper_only=False, from_folded=False):
        self.ids = [16] if gripper_only else list(ARM_IDS) + ([16] if use_gripper else [])
        for motor_id in self.ids:
            expected = 1200 if motor_id == 16 else MODELS[ARM_IDS.index(motor_id)]
            if self.bus.ping(motor_id) != expected:
                raise RuntimeError(f"ID {motor_id}: 예상 모터 모델이 아닙니다")
            mode = self.bus.read(motor_id, 11, 1)
            if mode != (5 if motor_id == 16 else 4) or self.bus.read(motor_id, 20, 4) != 0:
                raise RuntimeError(f"ID {motor_id}: 팔 mode=4 / 그리퍼 mode=5 / offset=0이 필요합니다 (현재 mode={mode})")
            present = self.bus.read(motor_id, 132, 4)
            if not 0 <= present <= 4095:
                raise RuntimeError(f"ID {motor_id}: 현재 펄스가 0..4095 밖입니다")
        if 16 in self.ids:
            validate_gripper(self.cfg)

    def enable(self):
        self.touched = True
        for motor_id in self.ids:
            self.bus.write(motor_id, 116, 4, self.bus.read(motor_id, 132, 4))
        if 16 in self.ids:
            current = self.cfg["gripper"]["current_limit_raw"]
            self.bus.write(16, 102, 2, current)
        for motor_id in self.ids:
            self.bus.write(motor_id, 64, 1, 1)

    def ramp(self, targets, seconds=None):
        start = {i: self.bus.read(i, 132, 4) for i in targets}
        seconds = self.cfg["stage_seconds"] if seconds is None else seconds
        steps = max(1, math.ceil(seconds / 0.05))
        sent = start.copy()
        for motor_id, value in start.items():
            self.bus.write(motor_id, 116, 4, value)
        for step in range(1, steps + 1):
            began = time.monotonic()
            t = step / steps
            blend = t * t * (3 - 2 * t)
            for motor_id, target in targets.items():
                value = round(start[motor_id] + blend * (target - start[motor_id]))
                if value != sent[motor_id] or step == steps:
                    self.bus.write(motor_id, 116, 4, value)
                    sent[motor_id] = value
            time.sleep(max(0, seconds / steps - (time.monotonic() - began)))
        time.sleep(self.cfg.get("stage_pause_seconds", 0.5))

    def arm(self, name, target):
        targets = dict(zip(ARM_IDS, pulses(target)))
        print(f"[ARM] {name}", flush=True)
        self.ramp(targets)

    def joint(self, name, target):
        targets = {int(i): v for i, v in target.items()}
        if not targets or any(i not in ARM_IDS or i not in self.ids or type(v) is not int or not 0 <= v <= 4095
                              for i, v in targets.items()):
            raise ValueError("펼침 관절 목표가 유효하지 않습니다")
        print(f"[JOINT] {name}: {targets}", flush=True)
        self.ramp(targets, self.cfg.get("unfold_seconds", self.cfg["stage_seconds"]))

    def gripper(self, name, target):
        if 16 not in self.ids or not 0 <= target <= 4095:
            raise ValueError("그리퍼 실행 상태/목표 오류")
        print(f"[GRIPPER] {name}", flush=True)
        self.ramp({16: target}, self.cfg.get("gripper_seconds", self.cfg["stage_seconds"]))
        if name not in ("OPEN", "RELEASE"):
            time.sleep(self.cfg["gripper"]["settle_s"])

    def hold(self):
        if not self.touched:
            return
        failures = []
        for motor_id in self.ids:
            try:
                self.bus.write(motor_id, 116, 4, self.bus.read(motor_id, 132, 4))
            except Exception as exc:
                failures.append(f"ID {motor_id}: {exc}")
        print("[HOLD] 현재 위치 목표로 정지 시도. 토크를 유지합니다. 팔과 물체를 확인하세요.", flush=True)
        if failures:
            print("[HOLD_INCOMPLETE] " + "; ".join(failures), file=sys.stderr)


def torque_off(bus, scope="all"):
    """Only Torque Enable=0 writes. Attempt every requested ID and read back all."""
    ids = list(ARM_IDS) if scope == "arm" else [16] if scope == "gripper" else [*ARM_IDS, 16]
    results = {}
    for motor_id in ids:
        try:
            expected = 1200 if motor_id == 16 else MODELS[ARM_IDS.index(motor_id)]
            if bus.ping(motor_id) != expected:
                raise ValueError("예상 모터 모델 불일치")
            bus.write(motor_id, 64, 1, 0)
            value = bus.read(motor_id, 64, 1)
            results[motor_id] = {"torque": value, "confirmed": value == 0}
        except Exception as exc:
            results[motor_id] = {"confirmed": False, "error": str(exc)}
    return results
