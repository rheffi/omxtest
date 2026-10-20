#!/usr/bin/env python3
"""Test an OMX-F camera hover or a saved fixed-dock hover.

Run with Python providing OpenCV/NumPy/SDK; --robot-python may override the
private motor child interpreter. Default is offline preview. MOVE starts automatic,
smooth joint-goal ramps through the requested stages. Arrival is not inferred.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import stat
import subprocess
import sys
import time
from pathlib import Path


import cv2
import numpy as np

from omx_arm_config import (add_arm_argument, load_arm_profile, require_camera,
                            require_explicit_arm, require_identity)


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
CALIBRATION = ROOT / "config" / "omx_xy_calibration.json"
LE_ROBOT_PYTHON = Path(sys.executable)
PORT = Path("/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00")
IDS = (11, 12, 13, 14, 15)
MODELS = (1060, 1060, 1060, 1200, 1200)
ZERO = 2048
PULSES_PER_REV = 4096
HOVER_ABOVE_REFERENCE_MM = 50.0
APPROACH_ABOVE_HOVER_MM = 40.0
UNFOLD_ELBOW_PULSES = 2559
UNFOLD_SHOULDER_PULSES = 1616
DEFAULT_STAGE_SECONDS = 4.0
GOAL_UPDATE_PERIOD_S = 0.05
STAGE_PAUSE_S = 0.5


def load_module(filename: str, name: str):
    spec = importlib.util.spec_from_file_location(name, HERE / filename)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"모듈을 읽지 못했습니다: {filename}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def pulse_to_q(pulses):
    return (np.asarray(pulses, dtype=float) - ZERO) * (2 * math.pi / PULSES_PER_REV)


def q_to_pulse(q):
    return np.rint(ZERO + np.asarray(q) * PULSES_PER_REV / (2 * math.pi)).astype(int)


def load_calibration(profile=None):
    path = profile.calibration if profile else CALIBRATION
    data = json.loads(path.read_text(encoding="utf-8"))
    require_identity(data, profile)
    if profile:
        require_camera(data, profile)
    if (data.get("method") != "pixel_to_omx_model_xy_homography"
            or data.get("robot_frame") != "link0"
            or data.get("robot_point") != "end_effector_link"
            or data.get("motor_zero_pulses_hypothesis") != [ZERO] * 5
            or data.get("motor_joint_signs_hypothesis") != [1] * 5):
        raise ValueError("OMX-F 카메라 보정·모터 변환 형식이 예상과 다릅니다")
    if any(type(data.get(key)) is not int or data[key] <= 0 for key in ("width", "height")):
        raise ValueError("보정된 영상 해상도는 양의 정수여야 합니다")
    matrix = np.asarray(data["matrix_3x3"], dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("보정 행렬 또는 영상 작업 영역 형식이 잘못됐습니다")
    return data


def choose_uv(args, data):
    if args.uv is not None:
        u, v = args.uv
    else:
        camera = load_module("05_omx_xy_calibration.py", "omx_xy_calibration")
        u, v, width, height = camera.click_pixel(data["camera"], data["width"], data["height"], 30)
        if (width, height) != (data["width"], data["height"]):
            raise ValueError("카메라 실제 해상도가 보정 기록과 다릅니다")
    if not np.isfinite([u, v]).all() or not (0 <= u < data["width"] and 0 <= v < data["height"]):
        raise ValueError("영상 좌표가 해상도 범위 밖입니다")
    return float(u), float(v)


def load_high_pose(name, profile=None):
    recorder = load_module("07_omx_pose_recorder.py", "omx_pose_recorder")
    saved = recorder.load_pose(name, profile)
    pulses = np.asarray(saved["raw_pulses"], dtype=int)
    if pulses.shape != (len(IDS),):
        raise ValueError(f"'{name}' 자세의 관절값 개수가 맞지 않습니다")
    return pulses


def make_drop_plan(start_pose, approach_pose, hover_pose, profile):
    if profile is None:
        raise ValueError("고정 하역 시험에 사용할 장비 정보가 없습니다")
    names = (start_pose, approach_pose, hover_pose)
    if len(set(names)) != 3:
        raise ValueError("높은 자세·하역 접근·하역 호버에는 서로 다른 자세 이름을 지정하세요")
    return {
        "mode": "saved_drop",
        "pose_names": names,
        "start_pulses": load_high_pose(start_pose, profile),
        "approach_pulses": load_high_pose(approach_pose, profile),
        "hover_pulses": load_high_pose(hover_pose, profile),
    }


def show_drop_plan(plan, stage_seconds, profile):
    profile.describe()
    print(f"poses={profile.poses}")
    for label, name, key in zip(
            ("HIGH", "DROP_APPROACH", "DROP_HOVER"), plan["pose_names"],
            ("start_pulses", "approach_pulses", "hover_pulses")):
        print(f"{label}: saved_pose={name}, pulses={plan[key].tolist()}")
    print(f"stage_seconds={stage_seconds:g}, stage_pause_seconds={STAGE_PAUSE_S:g}")
    print("stage_order=HIGH -> DROP_APPROACH -> DROP_HOVER")
    print("RETURN: DROP_APPROACH -> HIGH")
    print("고정 하역점의 저장 자세를 사용합니다. 카메라와 보정 파일은 열지 않습니다.")
    print("실제 출발 자세·경로·도착·핑키 정차 위치는 직접 확인하세요. 그리퍼는 제어하지 않습니다.")


def solve_pose(fk, target_mm, label):
    result = fk.solve(np.asarray(target_mm, dtype=float) / 1000, True)
    if not result.feasible or result.position_error_m > 0.002 or result.down_error_deg > 5:
        raise ValueError(f"{label}: 모델 IK 위치·아래 방향 제약을 충족하지 못했습니다")
    if np.asarray(result.q).shape != (len(IDS),) or not np.isfinite(result.q).all():
        raise ValueError(f"{label}: IK 관절 결과가 유효하지 않습니다")
    return q_to_pulse(result.q)


def make_plan(data, uv, start):
    fk = load_module("02_omx_xyz_offline.py", "omx_xyz_offline")
    u, v = [float(value) for value in uv]
    if (not np.isfinite([u, v]).all()
            or not (0 <= u < data["width"] and 0 <= v < data["height"])):
        raise ValueError("호버 목표의 UV가 보정 영역 밖입니다")
    matrix = np.asarray(data["matrix_3x3"], dtype=float)
    homogeneous = matrix @ np.array([u, v, 1.0])
    if not np.isfinite(homogeneous).all() or abs(homogeneous[2]) < 1e-9:
        raise ValueError("카메라-로봇 좌표 변환 결과가 유효하지 않습니다")
    xy = homogeneous[:2] / homogeneous[2]
    hover = np.r_[xy, float(data["model_z_reference_mm"]) + HOVER_ABOVE_REFERENCE_MM]
    approach = hover.copy()
    approach[2] += APPROACH_ABOVE_HOVER_MM
    if not np.isfinite(hover).all():
        raise ValueError("호버 좌표가 유효하지 않습니다")
    goal_approach = solve_pose(fk, approach, "상부 접근점")
    goal_hover = solve_pose(fk, hover, "호버점")
    plan = {"uv": uv, "hover_mm": hover, "approach_mm": approach,
            "approach_pulses": goal_approach, "hover_pulses": goal_hover,
            "start_pulses": start}
    if start is not None:
        start_position, _ = fk.forward(pulse_to_q(start))
        plan["start_mm"] = start_position * 1000
    return plan


def show_plan(data, plan, pose_name, from_folded=False, stage_seconds=DEFAULT_STAGE_SECONDS, profile=None):
    if profile:
        profile.describe()
        print(f"calibration={profile.calibration}; poses={profile.poses}")
    print(f"camera={data['camera']} {data['width']}x{data['height']}")
    print(f"clicked_uv={list(plan['uv'])}")
    print(f"model_hover_link0_mm={plan['hover_mm'].round(1).tolist()}")
    print(f"model_approach_link0_mm={plan['approach_mm'].round(1).tolist()}")
    print(f"candidate_hover_pulses={plan['hover_pulses'].tolist()}")
    if data.get("check_error"):
        print(f"check_point_xy_error_mm={data['check_error']['max_mm']}")
    print(f"stage_seconds={stage_seconds:g}, stage_pause_seconds={STAGE_PAUSE_S:g}")
    if plan["start_pulses"] is None:
        print(f"높은 자세 '{pose_name}' 미기록: 목표 계산만 표시합니다. 07_omx_pose_recorder.py로 먼저 기록하세요.")
    else:
        print(f"saved_high_pose={pose_name}, pulses={plan['start_pulses'].tolist()}")
        print(f"saved_start_model_mm={plan['start_mm'].round(1).tolist()}")
        print(f"candidate_approach_pulses={plan['approach_pulses'].tolist()}")
        if from_folded:
            print(f"unfold_waypoints: ID 13={UNFOLD_ELBOW_PULSES}, ID 12={UNFOLD_SHOULDER_PULSES}")
            print("stage_order=ID 13 -> ID 12 -> HIGH -> APPROACH -> HOVER")
        else:
            print("stage_order=HIGH -> APPROACH -> HOVER")
    print("이동 경로·실제 도착·상품 높이·그리퍼 중심·주변 충돌은 자동 확인하지 않습니다.")


def motor_command(operation, profile=None):
    if not LE_ROBOT_PYTHON.is_file():
        raise FileNotFoundError(f"모터용 Python이 없습니다: {LE_ROBOT_PYTHON}")
    command = [str(LE_ROBOT_PYTHON), "-u", "-B", str(Path(__file__).resolve()),
               "--_motor-op", operation]
    if profile and profile.name != "single":
        command += ["--arm", profile.name]
    return command


def child_command(uv, pose_name, from_folded=False, stage_seconds=DEFAULT_STAGE_SECONDS, profile=None):
    command = motor_command("live", profile) + ["--start-pose", pose_name,
               "--uv", str(uv[0]), str(uv[1]), "--stage-seconds", str(stage_seconds)]
    if from_folded:
        command += ["--from-folded", "--unfold-elbow", str(UNFOLD_ELBOW_PULSES),
                    "--unfold-shoulder", str(UNFOLD_SHOULDER_PULSES)]
    return command


def drop_child_command(start_pose, approach_pose, hover_pose, stage_seconds, profile):
    return motor_command("drop-test", profile) + [
        "--start-pose", start_pose, "--approach-pose", approach_pose,
        "--hover-pose", hover_pose, "--stage-seconds", str(stage_seconds),
    ]


def sdk_reply(packet, result, error, label):
    from dynamixel_sdk import COMM_SUCCESS
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{label}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{label}: status 0x{error:02X} {packet.getRxPacketError(error)}")


def sdk_read(packet, port, motor_id, address, size, label):
    reader = {1: packet.read1ByteTxRx, 4: packet.read4ByteTxRx}[size]
    value, result, error = reader(port, motor_id, address)
    sdk_reply(packet, result, error, f"ID {motor_id} {label} READ")
    return value - 0x100000000 if size == 4 and value & 0x80000000 else value


def sdk_write(packet, port, motor_id, address, size, value, label):
    writer = {1: packet.write1ByteTxRx, 4: packet.write4ByteTxRx}[size]
    result, error = writer(port, motor_id, address, int(value))
    sdk_reply(packet, result, error, f"ID {motor_id} {label} WRITE")


def check_port(profile=None):
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


def read_motor_state(packet, port):
    positions, torques, goals = [], [], []
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
        if mode != 4 or offset != 0 or error_bits != 0 or torque not in (0, 1):
            raise RuntimeError(f"ID {motor_id}: mode={mode}, offset={offset}, torque={torque}, error={error_bits}")
        positions.append(position)
        torques.append(torque)
        goals.append(goal)
    return {"positions": positions, "torques": torques, "goals": goals}


def read_positions(packet, port):
    return np.array([sdk_read(packet, port, i, 132, 4, "present") for i in IDS], dtype=int)


def send_stage(packet, port, label, targets, stage_seconds=DEFAULT_STAGE_SECONDS):
    """Ramp joint goals from measured positions; report errors without gating arrival."""
    present = read_positions(packet, port)
    indices = {motor_id: IDS.index(motor_id) for motor_id in targets}
    start = {motor_id: int(present[index]) for motor_id, index in indices.items()}
    steps = max(1, int(math.ceil(stage_seconds / GOAL_UPDATE_PERIOD_S)))
    period = stage_seconds / steps
    sent = start.copy()
    print(f"[{label}] present={present.tolist()}, goals={targets}, "
          f"ramp_seconds={stage_seconds:g}", flush=True)
    for motor_id, value in start.items():
        sdk_write(packet, port, motor_id, 116, 4, value, "ramp start")
    for step in range(1, steps + 1):
        tick_started = time.monotonic()
        fraction = step / steps
        # Smoothstep gives a gradual start and finish in joint goal space.
        blend = fraction * fraction * (3 - 2 * fraction)
        for motor_id, target in targets.items():
            value = int(round(start[motor_id] + (target - start[motor_id]) * blend))
            if value != sent[motor_id] or step == steps:
                sdk_write(packet, port, motor_id, 116, 4, value, "ramp goal")
                sent[motor_id] = value
        # Do not skip ahead when serial I/O is slow; the ramp simply takes longer.
        time.sleep(max(0.0, period - (time.monotonic() - tick_started)))
    time.sleep(STAGE_PAUSE_S)
    print(f"[{label}_GOALS_SENT] {targets}", flush=True)


def stop_at_present(packet, port):
    actual = read_positions(packet, port)
    for motor_id, value in zip(IDS, actual):
        sdk_write(packet, port, motor_id, 116, 4, value, "stop at present")
    print(f"[STOP] 잔여 이동 목표를 취소했습니다: {actual.tolist()}", flush=True)


def disable_arm_torque(packet, port):
    """Attempt OFF for every arm joint and verify each Torque Enable register."""
    print(f"[TORQUE_OFF] 팔 관절 {list(IDS)} 토크 해제", flush=True)
    for motor_id in IDS:
        try:
            sdk_write(packet, port, motor_id, 64, 1, 0, "disable torque")
        except Exception as exc:
            # A failure on one motor must not prevent OFF attempts on the others.
            print(f"[ALERT] ID {motor_id} 토크 해제 명령 오류: {exc}", flush=True)
    states = {}
    for motor_id in IDS:
        try:
            states[motor_id] = sdk_read(packet, port, motor_id, 64, 1, "torque after disable")
            print(f"ID {motor_id}: torque={states[motor_id]}", flush=True)
        except Exception as exc:
            states[motor_id] = None
            print(f"[ALERT] ID {motor_id} 토크 상태 확인 실패: {exc}", flush=True)
    if all(value == 0 for value in states.values()):
        print("[TORQUE_OFF_CONFIRMED] 팔 관절 ID 11~15 torque=0 확인. 수동으로 자세를 바꿀 수 있습니다.", flush=True)
        return True
    print(f"[TORQUE_OFF_INCOMPLETE] 해제되지 않았거나 상태를 모르는 관절이 있습니다: {states}", flush=True)
    return False


def internal_torque_off(profile=None):
    if profile:
        profile.describe()
    print("팔 관절 ID 11~15 토크를 해제합니다. 팔을 받친 상태를 유지하세요.", flush=True)
    if input("팔을 받쳤다면 OFF 입력 (취소는 Enter): ").strip().upper() != "OFF":
        print("[CANCELLED] 직렬 포트를 열거나 모터값을 변경하지 않았습니다.")
        return 0
    from dynamixel_sdk import PacketHandler, PortHandler
    port = PortHandler(str(check_port(profile)))
    packet = PacketHandler(2.0)
    try:
        if not port.openPort() or not port.setBaudRate(1_000_000):
            raise RuntimeError("직렬 포트 또는 1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        return 0 if disable_arm_torque(packet, port) else 2
    finally:
        if port.is_open:
            port.closePort()


def live_motion(packet, port, plan, *, from_folded=False, stage_seconds=DEFAULT_STAGE_SECONDS):
    state = read_motor_state(packet, port)
    current = np.asarray(state["positions"], dtype=int)
    high = np.asarray(plan["start_pulses"], dtype=int)
    approach = np.asarray(plan["approach_pulses"], dtype=int)
    hover = np.asarray(plan["hover_pulses"], dtype=int)
    saved_drop = plan.get("mode") == "saved_drop"
    approach_label = "DROP APPROACH" if saved_drop else "APPROACH"
    hover_label = "DROP HOVER" if saved_drop else "HOVER"
    print(f"현재 펄스={current.tolist()}, 팔 토크={state['torques']}", flush=True)
    print(f"높은 자세={high.tolist()}, 접근점={approach.tolist()}, 호버점={hover.tolist()}", flush=True)
    if from_folded:
        print(f"전송 순서: ID 13={UNFOLD_ELBOW_PULSES} -> ID 12={UNFOLD_SHOULDER_PULSES} -> "
              "HIGH -> APPROACH -> HOVER", flush=True)
    else:
        print(f"전송 순서: HIGH -> {approach_label} -> {hover_label}", flush=True)
    print(f"구간당 목표 보간 {stage_seconds:g}초 + 대기 {STAGE_PAUSE_S:g}초; "
          "MOVE 후 호버까지 자동 진행합니다.", flush=True)
    print("이 모드는 목표 전송 후 자동 도착·경로·충돌 검사를 하지 않습니다.", flush=True)
    if input("팔을 받치고 준비됐다면 MOVE 입력 (취소는 Enter): ").strip() != "MOVE":
        print("[CANCELLED] 모터 목표값·토크를 변경하지 않았습니다.")
        return 0

    torque_touched = any(state["torques"])
    try:
        # Opening the port or waiting for MOVE may have changed the pose.
        # Align each goal to the latest measured position before enabling torque.
        current = read_positions(packet, port)
        for motor_id, value in zip(IDS, current):
            sdk_write(packet, port, motor_id, 116, 4, value, "align goal")
        for motor_id, enabled in zip(IDS, state["torques"]):
            if not enabled:
                torque_touched = True
                sdk_write(packet, port, motor_id, 64, 1, 1, "enable torque")
        if from_folded:
            send_stage(packet, port, "ELBOW FIRST", {13: UNFOLD_ELBOW_PULSES}, stage_seconds)
            send_stage(packet, port, "SHOULDER SECOND", {12: UNFOLD_SHOULDER_PULSES}, stage_seconds)

        send_stage(packet, port, "TO HIGH", dict(zip(IDS, map(int, high))), stage_seconds)
        send_stage(packet, port, f"TO {approach_label}", dict(zip(IDS, map(int, approach))), stage_seconds)
        send_stage(packet, port, f"TO {hover_label}", dict(zip(IDS, map(int, hover))), stage_seconds)
        result_label = "DROP_HOVER" if saved_drop else "HOVER"
        print(f"[{result_label}_GOALS_SENT] 전체 목표 전송 종료. 실제 그리퍼 정렬을 확인하세요.", flush=True)

        visual = input("실제 그리퍼 중심 정렬: CENTERED / OFFSET / UNKNOWN 입력: ").strip().upper()
        if visual not in {"CENTERED", "OFFSET", "UNKNOWN"}:
            visual = "UNKNOWN"
        print(f"[VISUAL_RESULT] {visual} (사용자 육안 판정)")
        action = input("RETURN: 높은 자세 복귀 / OFF: 팔을 받치고 토크 해제 / Enter: 위치 유지: ").strip().upper()
        if action == "OFF":
            # Release owns its reporting; do not send hold goals after OFF begins.
            torque_touched = False
            return 0 if disable_arm_torque(packet, port) else 2
        if action != "RETURN":
            stop_at_present(packet, port)
            print("[HOLD AT HOVER] 현재 위치에서 팔 토크를 유지합니다.")
            return 0
        send_stage(packet, port, f"RETURN TO {approach_label}", dict(zip(IDS, map(int, approach))), stage_seconds)
        send_stage(packet, port, "RETURN TO HIGH", dict(zip(IDS, map(int, high))), stage_seconds)
        print("[RETURN_GOALS_SENT] 복귀 목표 전송 종료. 실제 높은 자세 도착은 별도 확인하세요.")
        if input("팔을 받치고 토크를 해제하려면 OFF 입력 (Enter: 토크 유지): ").strip().upper() == "OFF":
            torque_touched = False
            return 0 if disable_arm_torque(packet, port) else 2
        return 0
    except BaseException:
        if torque_touched:
            try:
                stop_at_present(packet, port)
            except Exception as exc:
                print(f"[ALERT] 잔여 Goal Position 취소 확인 실패: {exc}", file=sys.stderr)
        raise
    finally:
        if torque_touched:
            print("[HOLD] 팔 관절 토크가 ON일 수 있습니다. 팔을 지지·파킹한 뒤 상태를 확인하세요.", flush=True)


def internal_motor(uv, pose_name, from_folded=False, stage_seconds=DEFAULT_STAGE_SECONDS, profile=None):
    from dynamixel_sdk import PacketHandler, PortHandler
    data = load_calibration(profile)
    high = load_high_pose(pose_name, profile)
    if uv is None:
        raise ValueError("내부 실물 이동에 UV가 없습니다")
    if not Path(data["camera"]).exists():
        raise FileNotFoundError("상단 카메라가 클릭 뒤 분리됐습니다")
    plan = make_plan(data, tuple(uv), high)
    path = check_port(profile)
    port = PortHandler(str(path))
    packet = PacketHandler(2.0)
    try:
        if not port.openPort() or not port.setBaudRate(1_000_000):
            raise RuntimeError("직렬 포트 또는 1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        show_plan(data, plan, pose_name, from_folded, stage_seconds, profile)
        return live_motion(packet, port, plan, from_folded=from_folded, stage_seconds=stage_seconds)
    finally:
        if port.is_open:
            port.closePort()


def internal_drop_motor(start_pose, approach_pose, hover_pose, stage_seconds, profile):
    plan = make_drop_plan(start_pose, approach_pose, hover_pose, profile)
    from dynamixel_sdk import PacketHandler, PortHandler
    port = PortHandler(str(check_port(profile)))
    packet = PacketHandler(2.0)
    try:
        if not port.openPort() or not port.setBaudRate(1_000_000):
            raise RuntimeError("직렬 포트 또는 1000000 bps 설정에 실패했습니다")
        time.sleep(1.5)
        show_drop_plan(plan, stage_seconds, profile)
        return live_motion(packet, port, plan, stage_seconds=stage_seconds)
    finally:
        if port.is_open:
            port.closePort()


def parse_args():
    parser = argparse.ArgumentParser(description="카메라 클릭 또는 저장 자세로 OMX-F 저속 호버")
    add_arm_argument(parser)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--live", action="store_true", help="MOVE 확인 후 카메라 또는 저장 자세 호버를 실제 실행")
    mode.add_argument("--torque-off", action="store_true", help="카메라·보정 없이 팔 관절 ID 11~15 토크 해제")
    parser.add_argument("--from-folded", action="store_true",
                        help="ID 13 -> ID 12 -> 높은 자세 순서로 목표 전송")
    parser.add_argument("--start-pose", default="hover_high", metavar="NAME",
                        help="07 자세 기록 파일에서 읽을 높은 자세 이름 (기본: hover_high)")
    parser.add_argument("--drop-test", action="store_true",
                        help="카메라·보정 없이 저장 자세로 고정 핑키 하역점의 호버 시험")
    parser.add_argument("--approach-pose", metavar="NAME",
                        help="고정 하역 접근 자세 이름 (--drop-test 기본: drop_approach)")
    parser.add_argument("--hover-pose", metavar="NAME",
                        help="고정 하역 직전 호버 자세 이름 (--drop-test 기본: drop_hover)")
    parser.add_argument("--stage-seconds", type=float, default=DEFAULT_STAGE_SECONDS, metavar="SECONDS",
                        help="구간당 목표 보간 시간; 값이 클수록 느림 (기본: 4초)")
    parser.add_argument("--uv", nargs=2, type=float, metavar=("U", "V"),
                        help="계산 전용: 카메라 창 없이 저장된 영상 좌표로 미리보기")
    parser.add_argument("--_motor-op", choices=("live", "torque-off", "drop-test"), help=argparse.SUPPRESS)
    parser.add_argument("--robot-python", type=Path, default=None, help="모터용 Python; 기본은 현재 Python")
    parser.add_argument("--unfold-elbow", type=int, default=UNFOLD_ELBOW_PULSES, help="접힌 출발 ID13 중간 목표")
    parser.add_argument("--unfold-shoulder", type=int, default=UNFOLD_SHOULDER_PULSES, help="접힌 출발 ID12 중간 목표")
    args = parser.parse_args()
    if any(not 0 <= v <= 4095 for v in (args.unfold_elbow, args.unfold_shoulder)):
        parser.error("펼침 중간값은 0..4095 pulse여야 합니다")
    drop_mode = args.drop_test or args._motor_op == "drop-test"
    if drop_mode:
        if args.torque_off or args.from_folded or args.uv is not None:
            parser.error("고정 하역 시험에는 --torque-off, --from-folded, --uv를 함께 쓰지 마세요")
        if args._motor_op not in (None, "drop-test"):
            parser.error("고정 하역 시험의 내부 명령이 잘못됐습니다")
    elif args.approach_pose is not None or args.hover_pose is not None:
        parser.error("--approach-pose와 --hover-pose는 --drop-test에 사용하세요")
    args.approach_pose = args.approach_pose or "drop_approach"
    args.hover_pose = args.hover_pose or "drop_hover"
    if (args.torque_off or args._motor_op == "torque-off") and (args.from_folded or args.uv is not None):
        parser.error("--torque-off에는 --from-folded 또는 --uv를 함께 사용하지 마세요")
    if not math.isfinite(args.stage_seconds) or args.stage_seconds <= 0:
        parser.error("--stage-seconds는 유한한 양수여야 합니다")
    return args


def main():
    global LE_ROBOT_PYTHON, UNFOLD_ELBOW_PULSES, UNFOLD_SHOULDER_PULSES
    args = parse_args()
    if args.robot_python is not None:
        LE_ROBOT_PYTHON = args.robot_python
    UNFOLD_ELBOW_PULSES, UNFOLD_SHOULDER_PULSES = args.unfold_elbow, args.unfold_shoulder
    profile = load_arm_profile(args.arm)
    if args.live or args.torque_off or args._motor_op:
        require_explicit_arm(profile)
    if args._motor_op == "torque-off":
        return internal_torque_off(profile)
    if args._motor_op == "drop-test":
        return internal_drop_motor(args.start_pose, args.approach_pose, args.hover_pose,
                                   args.stage_seconds, profile)
    if args._motor_op:
        return internal_motor(args.uv, args.start_pose, args.from_folded, args.stage_seconds, profile)
    if args.torque_off:
        result = subprocess.run(motor_command("torque-off", profile), check=False)
        return result.returncode
    if args.live and args.uv is not None:
        raise ValueError("실물 이동은 저장된 --uv 대신 현재 카메라 클릭이 필요합니다")
    if profile:
        profile.describe()
    if args.drop_test:
        plan = make_drop_plan(args.start_pose, args.approach_pose, args.hover_pose, profile)
        show_drop_plan(plan, args.stage_seconds, profile)
        if not args.live:
            print("[DRY_RUN] 카메라·직렬 포트를 열거나 모터를 움직이지 않았습니다.")
            return 0
        command = drop_child_command(args.start_pose, args.approach_pose, args.hover_pose,
                                     args.stage_seconds, profile)
        return subprocess.run(command, check=False).returncode
    data = load_calibration(profile)
    try:
        high = load_high_pose(args.start_pose, profile)
    except FileNotFoundError:
        if args.live:
            raise
        high = None
    uv = choose_uv(args, data)
    plan = make_plan(data, uv, high)
    show_plan(data, plan, args.start_pose, args.from_folded, args.stage_seconds, profile)
    if not args.live:
        print("[DRY_RUN] 직렬 포트를 열거나 모터를 움직이지 않았습니다.")
        return 0
    result = subprocess.run(child_command(uv, args.start_pose, args.from_folded, args.stage_seconds, profile), check=False)
    return result.returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[CANCELLED] 사용자 중단. 모터가 이미 켜져 있다면 현재 토크 상태를 확인하세요.", file=sys.stderr)
        raise SystemExit(1)
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError, cv2.error) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
