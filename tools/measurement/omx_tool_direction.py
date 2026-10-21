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
