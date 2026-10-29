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
from omx_wrist import canonical_wrist, nearest_wrist
from omx_motor_frame import frame_packet, frame_metadata


class DynamixelBus:
    def __init__(self, path, *, frame_file=None):
        self.frame_file = frame_file
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
        self.packet = frame_packet(PacketHandler(2.0), self.path, self.frame_file)
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
    metadata = frame_metadata(bus.packet) if hasattr(bus, "packet") else None
    if metadata:
        row = next(row for row in rows if row["id"] == 14)
        row["physical_position"] = metadata["physical_present_pulse"]
        row["motor_frame"] = metadata
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
            if motor_id == 15:
                canonical_wrist(present)
            elif not 0 <= present <= 4095:
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
        if 15 in targets:
            targets = dict(targets)
            targets[15] = nearest_wrist(targets[15], start[15])
            self.emit("wrist_goal", dict(present=start[15], goal=targets[15]))
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

    def wait_arm_at(self, target, parking, *, stage="PARK", final=False):
        """Require arrival and stable readings; elapsed ramp time is not arrival."""
        tolerance = (parking["tolerance_pulses"] if final else
                     parking.get("transit_tolerance_pulses", parking["tolerance_pulses"]))
        goals = dict(zip(ARM_IDS, pulses(target)))
        goals[15] = nearest_wrist(goals[15], self.bus.read(15, 132, 4))
        self._wait_positions(goals, parking, stage=stage, tolerance=tolerance, final=final)

    def _wait_positions(self, goals, parking, *, stage, tolerance, final):
        """Read back positions/torque/errors for the motors being verified."""
        deadline = time.monotonic() + parking["timeout_seconds"]
        stable_since = None
        previous = None
        while True:
            positions = {}
            for motor_id in goals:
                if self.bus.read(motor_id, 70, 1) != 0:
                    raise RuntimeError(f"ID {motor_id}: 하드웨어 오류; 복귀 중단·토크 자동 해제 안 함")
                if self.bus.read(motor_id, 64, 1) != 1:
                    raise RuntimeError(f"ID {motor_id}: 복귀 중 토크 ON 미확인")
                positions[motor_id] = self.bus.read(motor_id, 132, 4)
            now = time.monotonic()
            errors = {i: positions[i] - goals[i] for i in goals}
            max_error = max(abs(error) for error in errors.values())
            at_goal = max_error <= tolerance
            stable = previous is not None and all(abs(positions[i] - previous[i]) <= 2 for i in goals)
            if at_goal and stable:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= parking["settle_seconds"]:
                    self.emit("park_arrival", dict(stage=stage, goals=goals, positions=positions,
                                                   errors=errors, tolerance_pulses=tolerance, final=final))
                    print(f"[PARK ARRIVED] {stage}: 최대 오차={max_error}펄스 / 기준={tolerance}펄스; 정지 확인", flush=True)
                    return
            else:
                stable_since = None
            if now >= deadline:
                raise RuntimeError(f"{stage} 도착/정지 미확인: 기준={tolerance}펄스, "
                                   f"오차(현재-목표)={errors}, 목표={goals}, 현재={positions}; "
                                   "토크 자동 해제 안 함. 실제 간섭·받침을 확인하세요")
            previous = positions
            time.sleep(0.1)

    def park(self, parking):
        """Return, verify empty-gripper closure, then disable all torque."""
        try:
            grip = validate_gripper(self.cfg)
            if 16 not in self.ids:
                raise ValueError("복귀 후 닫힘을 위해 그리퍼 제어 준비가 필요합니다")
            # All six motors must be available before moving the parking route.
            for motor_id, expected in zip((*ARM_IDS, 16), (*MODELS, 1200)):
                if self.bus.ping(motor_id) != expected:
                    raise RuntimeError(f"ID {motor_id}: 예상 모터 모델이 아닙니다")
                if self.bus.read(motor_id, 70, 1) != 0:
                    raise RuntimeError(f"ID {motor_id}: 하드웨어 오류; 복귀 시작 안 함")
            for index, pose in enumerate(parking["route"]):
                self.emit("park_stage_start", pose)
                print(f"[PARK ARM] {pose['name']}", flush=True)
                self.ramp(dict(zip(ARM_IDS, pulses(pose["target"]))), parking["seconds"])
                self.wait_arm_at(pose["target"], parking, stage=pose["name"],
                                 final=index == len(parking["route"]) - 1)
                self.emit("park_stage_done", pose)
            closing = dict(name="PARK_CLOSE", target=grip["close_pulse"],
                           current_limit_raw=grip["current_limit_raw"])
            self.emit("park_gripper_start", closing)
            self.gripper("PARK_CLOSE", grip["close_pulse"])
            self._wait_positions({16: grip["close_pulse"]}, parking,
                                 stage="PARK_CLOSE", tolerance=20, final=True)
            self.emit("park_gripper_done", closing)
            # Closing near a support must not displace the arm before release.
            self.wait_arm_at(parking["route"][-1]["target"], parking,
                             stage="PARK_REST_AFTER_CLOSE", final=True)
        except BaseException:
            self.hold()
            raise
        # Do not hold or re-enable motors after a partial torque-off failure.
        # The operator must see the readback for every ID and support the arm.
        print("[PARK] 마지막 자세·그리퍼 닫힘·정지 확인. 팔·그리퍼 토크 해제.", flush=True)
        results = torque_off(self.bus)
        self.emit("park_torque_off", results)
        if not all(row["confirmed"] for row in results.values()):
            raise RuntimeError(f"토크 해제 일부 미확인. 팔을 받치고 상태를 확인하세요: {results}")

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
