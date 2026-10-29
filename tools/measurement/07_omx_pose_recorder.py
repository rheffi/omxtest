#!/usr/bin/env python3
"""Record and inspect named OMX-F arm poses. Saving only PINGs/READs motors."""

from __future__ import annotations

from omx_motor_frame import frame_packet, frame_metadata

import argparse
import json
import os
import re
import stat
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from omx_arm_config import (add_arm_argument, load_arm_profile,
                            require_explicit_arm, require_identity)
from omx_wrist import canonical_wrist

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
POSES_FILE = ROOT / "config" / "omx_saved_poses.json"
PORT = Path("/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00")
IDS = (11, 12, 13, 14, 15)
MODELS = (1060, 1060, 1060, 1200, 1200)
POSE_NAME = re.compile(r"[a-z][a-z0-9_]{0,31}\Z")


def validate_name(name: str) -> str:
    if not POSE_NAME.fullmatch(name):
        raise ValueError("자세 이름은 영문 소문자로 시작하고 영문 소문자·숫자·_만 사용하세요 (최대 32자)")
    return name


def empty_registry(profile=None) -> dict:
    data = {"schema_version": 1, "robot": "ROBOTIS OMX-AI follower OMX-F",
            "motor_ids": list(IDS), "poses": {}}
    if profile:
        data.update(profile.identity())
    return data


def load_registry(profile=None) -> dict:
    path = profile.poses if profile else POSES_FILE
    if not path.exists():
        return empty_registry(profile)
    data = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(data, dict) or data.get("schema_version") != 1
            or data.get("robot") != "ROBOTIS OMX-AI follower OMX-F"
            or data.get("motor_ids") != list(IDS)
            or not isinstance(data.get("poses"), dict)):
        raise ValueError(f"저장 자세 파일 형식이 예상과 다릅니다: {path}")
    require_identity(data, profile)
    for name, pose in data["poses"].items():
        validate_name(name)
        validate_pose(pose)
    return data


def validate_pose(pose: dict) -> None:
    if not isinstance(pose, dict):
        raise ValueError("저장 자세 항목이 객체가 아닙니다")
    raw = pose.get("raw_pulses")
    if (pose.get("motor_ids") != list(IDS)
            or pose.get("motor_models") != list(MODELS)
            or pose.get("operating_modes") != [4] * 5
            or pose.get("homing_offsets") != [0] * 5
            or not isinstance(raw, list) or len(raw) != 5
            or any(type(p) is not int or not 0 <= p <= 4095 for p in raw)):
        raise ValueError("저장 자세의 모터 정보나 펄스 형식이 잘못됐습니다")


def load_pose(name: str, profile=None) -> dict:
    validate_name(name)
    poses = load_registry(profile)["poses"]
    if name not in poses:
        option = f" --arm {profile.name}" if profile and profile.name != "single" else ""
        raise FileNotFoundError(f"'{name}' 자세가 없습니다. 07_omx_pose_recorder.py{option} --save {name}으로 먼저 기록하세요")
    return poses[name]


def atomic_save(data: dict, profile=None) -> None:
    path = profile.poses if profile else POSES_FILE
    require_identity(data, profile)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    try:
        temp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(temp, path)
    finally:
        if temp.exists():
            temp.unlink()


def sdk_reply(packet, result: int, error: int, label: str) -> None:
    from dynamixel_sdk import COMM_SUCCESS
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{label}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{label}: status 0x{error:02X} {packet.getRxPacketError(error)}")


def sdk_read(packet, port, motor_id: int, address: int, size: int, label: str) -> int:
    reader = {1: packet.read1ByteTxRx, 4: packet.read4ByteTxRx}[size]
    value, result, error = reader(port, motor_id, address)
    sdk_reply(packet, result, error, f"ID {motor_id} {label} READ")
    return value - 0x100000000 if size == 4 and value & 0x80000000 else value


def check_port(profile=None) -> Path:
    require_explicit_arm(profile)
    selected = profile.port if profile else load_arm_profile(None).port
    if not selected.exists():
        raise FileNotFoundError(f"OMX-F 직렬 포트가 없습니다: {selected}")
    path = selected.resolve(strict=True)
    if not stat.S_ISCHR(path.stat().st_mode) or not str(path).startswith("/dev/"):
        raise RuntimeError(f"올바른 직렬 장치가 아닙니다: {path}")
    busy = subprocess.run(["fuser", "-s", str(path)], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL, check=False)
    if busy.returncode != 1:
        raise RuntimeError(f"직렬 포트 점유/확인 오류: fuser={busy.returncode}")
    return path


def read_once(packet, port) -> dict:
    positions, goals, torques = [], [], []
    for motor_id, expected_model in zip(IDS, MODELS):
        model, result, error = packet.ping(port, motor_id)
        sdk_reply(packet, result, error, f"ID {motor_id} PING")
        if model != expected_model:
            raise RuntimeError(f"ID {motor_id}: 예상과 다른 모터 모델 {model}")
        mode = sdk_read(packet, port, motor_id, 11, 1, "mode")
        offset = sdk_read(packet, port, motor_id, 20, 4, "homing offset")
        torque = sdk_read(packet, port, motor_id, 64, 1, "torque")
        error_bits = sdk_read(packet, port, motor_id, 70, 1, "hardware error")
        goal = sdk_read(packet, port, motor_id, 116, 4, "goal")
        position = sdk_read(packet, port, motor_id, 132, 4, "present")
        if motor_id == 15:
            # Persist the equivalent single-turn pose; live transmission picks
            # the nearest signed turn from the actual measured wrist position.
            position = canonical_wrist(position)
        if mode != 4 or offset != 0 or error_bits != 0 or torque not in (0, 1):
            raise RuntimeError(f"ID {motor_id}: mode={mode}, offset={offset}, torque={torque}, error={error_bits}")
        if not 0 <= position <= 4095:
            raise RuntimeError(f"ID {motor_id}: 현재 펄스가 저장 범위 밖입니다: {position}")
        positions.append(position)
        goals.append(goal)
        torques.append(torque)
    state = {"positions": positions, "goals": goals, "torque": torques[0] if len(set(torques)) == 1 else torques}
    if frame_metadata(packet):
        state["motor_frame"] = frame_metadata(packet)
    return state


def snapshot(profile=None) -> dict:
    from dynamixel_sdk import PacketHandler, PortHandler
    path = check_port(profile)
    port = PortHandler(str(path))
    packet = frame_packet(PacketHandler(2.0), path)
    try:
        if not port.openPort() or not port.setBaudRate(1_000_000):
            raise RuntimeError("직렬 포트 또는 1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        return read_once(packet, port)
    finally:
        if port.is_open:
            port.closePort()


def save_pose(name: str, replace: bool, profile=None) -> int:
    validate_name(name)
    registry = load_registry(profile)
    exists = name in registry["poses"]
    if exists and not replace:
        raise FileExistsError(f"'{name}' 자세가 이미 있습니다. 교체하려면 --replace를 붙이세요")
    if not exists and replace:
        raise ValueError(f"'{name}' 자세가 없어 --replace할 수 없습니다")
    state = snapshot(profile)
    print(f"이름={name}, 현재 팔 펄스={state['positions']}, torque={state['torque']}")
    print("모터 PING/READ만 수행했습니다. 실제 자세와 주변 여유를 직접 확인하세요.")
    word = "REPLACE" if exists else "SAVE"
    if input(f"이 자세를 '{name}'으로 기록하려면 {word} 입력: ").strip() != word:
        print("[CANCELLED] 저장 자세 파일을 변경하지 않았습니다.")
        return 1
    if registry != load_registry(profile):
        raise RuntimeError("확인하는 동안 저장 자세 파일이 변경됐습니다. 다시 실행하세요")
    registry["poses"][name] = {
        "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "motor_ids": list(IDS), "motor_models": list(MODELS),
        "operating_modes": [4] * 5, "homing_offsets": [0] * 5,
        "raw_pulses": state["positions"],
    }
    if state.get("motor_frame"):
        registry["poses"][name]["motor_frame"] = state["motor_frame"]
    atomic_save(registry, profile)
    print(f"[SAVED] {profile.poses if profile else POSES_FILE}: {name}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OMX-F 팔 자세 기록·조회. 저장할 때만 모터 PING/READ")
    add_arm_argument(parser)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--save", metavar="NAME", help="현재 팔 자세를 지정한 이름으로 기록")
    group.add_argument("--show", metavar="NAME", help="저장 자세 표시 (장비 연결 불필요)")
    group.add_argument("--list", action="store_true", help="저장 자세 이름 표시 (장비 연결 불필요)")
    parser.add_argument("--replace", action="store_true", help="기존 이름을 명시적으로 교체")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = load_arm_profile(args.arm)
    if profile:
        profile.describe()
        print(f"poses={profile.poses}")
    if args.replace and not args.save:
        raise ValueError("--replace는 --save와 함께 사용하세요")
    if args.save:
        return save_pose(args.save, args.replace, profile)
    if args.show:
        print(json.dumps(load_pose(args.show, profile), ensure_ascii=False, indent=2))
    else:
        registry = load_registry(profile)
        for name in sorted(registry["poses"]):
            print(name)
        if not registry["poses"]:
            print("저장된 자세가 없습니다.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[CANCELLED] 자세 기록을 취소했습니다.", file=sys.stderr)
        raise SystemExit(1)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
