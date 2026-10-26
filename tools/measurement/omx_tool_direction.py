"""Recorded grasp direction and continuous, single-turn IK. No hardware I/O."""

import math

import numpy as np
from omx_wrist import nearest_wrist


def pulse_to_q(values):
    if (not isinstance(values, (list, tuple, np.ndarray)) or len(values) != 5
            or any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer))
                   or not 0 <= v <= 4095 for v in values)):
        raise ValueError("집기 방향 기준은 0..4095 범위의 관절 정수 펄스 5개여야 합니다")
    return (np.asarray(values, dtype=float) - 2048) * (2 * math.pi / 4096)


def heading(fk, values):
    _, orientation = fk.forward(pulse_to_q(values))
    y = orientation[:2, 1]
    if np.linalg.norm(y) < 0.1:
        raise ValueError("집게 방향을 읽을 수 없습니다. 집게를 아래로 향하게 맞춰 집기 자세를 다시 기록하세요")
    return math.atan2(float(y[1]), float(y[0]))


def direction_solver(fk, reference_pulses, start_pulses=None):
    """Keep the recorded world heading; prefer the preceding planned pose."""
    yaw = heading(fk, reference_pulses)
    previous = pulse_to_q(reference_pulses if start_pulses is None else start_pulses)

    def solve(xyz_mm):
        nonlocal previous
        target = np.asarray(xyz_mm, dtype=float) / 1000
        result = fk.solve(target, True, tool_yaw_rad=yaw, reference_q=previous)
        if not result.feasible:
            raise ValueError(f"기록한 집게 방향으로 도달할 수 없는 모델 XYZ: {xyz_mm}")
        raw = np.rint(2048 + result.q * 4096 / (2 * math.pi)).astype(int)
        # +pi can round to 4096; its equivalent single-turn command is 0.
        raw[raw == 4096] = 0
        values = [int(v) for v in raw]
        quantized = pulse_to_q(values)
        large = np.flatnonzero(np.abs(quantized[:4] - previous[:4]) >= math.pi)
        if len(large):
            motor_id = 11 + int(large[0])
            raise ValueError(f"ID {motor_id} 관절이 180도 이상 회전하는 경로입니다. HIGH 또는 집기 방향을 다시 기록하세요")
        position, orientation = fk.forward(quantized)
        desired_y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        down_error = math.degrees(math.acos(float(np.clip(-orientation[2, 0], -1, 1))))
        yaw_error = math.degrees(math.acos(float(np.clip(orientation[:, 1] @ desired_y, -1, 1))))
        if np.linalg.norm(position - target) > 0.002 or down_error > 5 or yaw_error > 1:
            raise ValueError("정수 펄스 변환 후 위치·아래 방향·집게 방향 오차가 허용 범위를 벗어났습니다")
        previous = quantized
        return values

    return solve


def fixed_wrist_solver(fk, wrist_pulse, start_pulses):
    """Solve XYZ/downward approach with ID 15 constant, including tool offset."""
    previous = pulse_to_q(start_pulses)
    fixed = list(start_pulses)
    fixed[4] = wrist_pulse
    fixed_angle = pulse_to_q(fixed)[4]

    def solve(xyz_mm):
        nonlocal previous
        target = np.asarray(xyz_mm, dtype=float) / 1000
        result = fk.solve(target, True, reference_q=previous, fixed_wrist_rad=fixed_angle)
        if not result.feasible:
            raise ValueError(f"고정 손목으로 도달할 수 없는 모델 XYZ: {xyz_mm}")
        raw = np.rint(2048 + result.q * 4096 / (2 * math.pi)).astype(int)
        raw[raw == 4096] = 0
        values = [int(v) for v in raw]
        values[4] = wrist_pulse
        quantized = pulse_to_q(values)
        if np.any(np.abs(quantized[:4] - previous[:4]) >= math.pi):
            raise ValueError("고정 손목 경로의 팔 관절이 180도 이상 회전합니다")
        position, orientation = fk.forward(quantized)
        down_error = math.degrees(math.acos(float(np.clip(-orientation[2, 0], -1, 1))))
        if np.linalg.norm(position - target) > 0.002 or down_error > 5:
            raise ValueError("고정 손목 경로의 위치·아래 방향 오차가 허용 범위를 벗어났습니다")
        previous = quantized
        return values
    return solve


def height_only_pose(fk, reference_pulses, height_mm):
    """Change only model Z, retaining the original XY, tool pitch and wrist."""
    original = pulse_to_q(reference_pulses)
    target, _ = fk.forward(original)
    target[2] = float(height_mm) / 1000
    pitch = float(sum(original[1:4]))

    def residual(angles):
        position, _ = fk.forward(np.r_[angles, original[4]])
        return np.r_[position - target, .1 * (sum(angles[1:4]) - pitch)]

    angles = original[:4].copy()
    damping = 1e-8
    for _ in range(100):
        error = residual(angles)
        if np.linalg.norm(error) < 1e-10:
            break
        jacobian = np.column_stack([
            (residual(angles + np.eye(4)[i] * 1e-6) - error) / 1e-6
            for i in range(4)])
        step = np.linalg.solve(jacobian.T @ jacobian + damping * np.eye(4),
                               -jacobian.T @ error)
        candidate = angles + step
        if np.linalg.norm(residual(candidate)) < np.linalg.norm(error):
            angles = candidate
            damping = max(1e-12, damping / 3)
        else:
            damping = min(1e3, damping * 10)
    raw = np.rint(2048 + np.r_[angles, original[4]] * 4096 / (2 * math.pi)).astype(int)
    raw[4] = reference_pulses[4]
    values = [int(v) for v in raw]
    quantized = pulse_to_q(values)
    position, _ = fk.forward(quantized)
    if (np.linalg.norm(position - target) > .002
            or abs(sum(quantized[1:4]) - pitch) > math.radians(1)
            or np.any(np.abs(quantized[:4] - original[:4]) >= math.pi)):
        raise ValueError("기존 슬롯 XY·방향을 유지하는 놓기 높이를 계산할 수 없습니다")
    return values


def wrist_report(fk, stages):
    """Expose saved-pose transitions; never silently rewrite recorded poses."""
    report = []
    previous = None
    for stage in stages:
        if stage["kind"] != "arm":
            continue
        values = stage["target"]
        pulse_to_q(values)
        goal = int(values[4]) if previous is None else nearest_wrist(values[4], previous)
        delta = None if previous is None else goal - previous
        try:
            yaw_deg = math.degrees(heading(fk, values))
        except ValueError:
            yaw_deg = None  # A saved transport pose can point the tool sideways.
        report.append(dict(stage=stage["name"], id15_pulse=int(values[4]),
                           id15_unwrapped_pulse=goal,
                           delta_deg=None if delta is None else delta * 360 / 4096,
                           model_tool_heading_deg=yaw_deg))
        previous = goal
    return report
