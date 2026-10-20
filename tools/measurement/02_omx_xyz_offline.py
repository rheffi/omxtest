#!/usr/bin/env python3
"""Offline OMX-F base-frame XYZ -> URDF joint-angle feasibility check.

No serial, ROS, camera, or robot-control imports. All geometry below is from
ROBOTIS omx_f.urdf (joint1..5 and end_effector_joint):
https://github.com/ROBOTIS-GIT/open_manipulator/blob/main/open_manipulator_description/urdf/omx_f/omx_f.urdf

Coordinates are meters in link0; results are URDF radians, NOT Dynamixel raw
positions or validated physical commands. The end_effector_link origin is a
model point, not a measured grasp point between the fingers.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5")
# URDF joint origin translations in parent-link coordinates, in meters.
JOINT_XYZ = np.array(
    [
        [-0.01125, 0.0, 0.034],
        [0.0, 0.0, 0.0635],
        [0.0415, 0.0, 0.11315],
        [0.162, 0.0, 0.0],
        [0.0287, 0.0, 0.0],
    ],
    dtype=float,
)
JOINT_AXES = ("z", "y", "y", "y", "x")
TOOL_XYZ = np.array([0.09193, -0.0016, 0.0], dtype=float)
# These are model limits; they are not verified mechanical/operational limits.
JOINT_LOWER = -2.0 * math.pi
JOINT_UPPER = 2.0 * math.pi
POSITION_TOLERANCE_M = 0.002
DOWN_TOLERANCE_DEG = 5.0


def rotation(axis: str, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def forward(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return end_effector_link position and rotation in link0."""
    if q.shape != (5,) or not np.all(np.isfinite(q)):
        raise ValueError("q must contain five finite URDF joint angles")
    p = np.zeros(3)
    r = np.eye(3)
    for origin, axis, angle in zip(JOINT_XYZ, JOINT_AXES, q):
        p = p + r @ origin
        r = r @ rotation(axis, float(angle))
    return p + r @ TOOL_XYZ, r


def residual(q4: np.ndarray, target: np.ndarray, down: bool) -> np.ndarray:
    q = np.r_[q4, 0.0]  # joint5 is tool-axis roll; leave it at zero offline.
    position, orientation = forward(q)
    errors = [*(position - target)]
    if down:
        # The URDF tool's +X axis must point downward for a top approach.
        # Weight 1 radian of direction error as 0.1 m of position error.
        errors.extend(0.1 * (orientation[:, 0] - np.array([0.0, 0.0, -1.0])))
    return np.asarray(errors)


def optimize(seed: np.ndarray, target: np.ndarray, down: bool) -> tuple[np.ndarray, float]:
    """Damped numerical least squares on joint1..4; no actuator commands."""
    q = seed.copy()
    damping = 0.01
    error = residual(q, target, down)
    cost = float(error @ error)
    for _ in range(180):
        jacobian = np.empty((len(error), 4))
        for col in range(4):
            shifted = q.copy()
            shifted[col] += 1e-5
            jacobian[:, col] = (residual(shifted, target, down) - error) / 1e-5
        step = np.linalg.solve(
            jacobian.T @ jacobian + damping * np.eye(4), -jacobian.T @ error
        )
        candidate = np.clip(q + step, JOINT_LOWER, JOINT_UPPER)
        candidate_error = residual(candidate, target, down)
        candidate_cost = float(candidate_error @ candidate_error)
        if candidate_cost < cost:
            improvement = cost - candidate_cost
            q, error, cost = candidate, candidate_error, candidate_cost
            damping = max(damping / 2.0, 1e-7)
            if improvement < 1e-15:
                break
        else:
            damping = min(damping * 5.0, 1e3)
            if np.linalg.norm(step) < 1e-9:
                break
    return q, cost


@dataclass
class Result:
    q: np.ndarray
    position: np.ndarray
    position_error_m: float
    down_error_deg: float
    feasible: bool


def solve(target: np.ndarray, down: bool) -> Result:
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("XYZ must contain three finite values in meters")
    # Deterministic seeds cover both elbow configurations and yaw directions.
    seeds = [
        np.array([yaw, shoulder, elbow, wrist], dtype=float)
        for yaw in (0.0, math.pi / 2, -math.pi / 2, math.pi)
        for shoulder, elbow, wrist in (
            (0.0, 0.0, 0.0),
            (-0.7, 1.4, 0.0),
            (0.7, -1.4, 0.0),
            (-1.0, 1.7, 0.9),
            (1.0, -1.7, 2.2),
        )
    ]
    candidates = []
    for seed in seeds:
        q4, _ = optimize(seed, target, down)
        q = np.r_[q4, 0.0]
        position, orientation = forward(q)
        position_error = float(np.linalg.norm(position - target))
        cosine = float(np.clip(orientation[:, 0] @ [0.0, 0.0, -1.0], -1.0, 1.0))
        down_error = math.degrees(math.acos(cosine))
        feasible = position_error <= POSITION_TOLERANCE_M and (
            not down or down_error <= DOWN_TOLERANCE_DEG
        )
        candidates.append(Result(q, position, position_error, down_error, feasible))
    # Among accurate solutions, prefer smaller joint travel from URDF zero.
    # Numerical errors at the picometer scale must not select a full turn.
    return min(candidates, key=lambda c: (
        not c.feasible,
        c.position_error_m > 0.0002,
        c.down_error_deg > 1.0 if down else False,
        float(np.linalg.norm(c.q)),
        c.position_error_m,
    ))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OMX-F 공식 URDF 기준 XYZ 도달성 계산. 장비 연결·동작 없음."
    )
    parser.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        required=True, help="link0 기준 목표 end_effector_link 위치 [m]")
    parser.add_argument("--approach", choices=("any", "down"), default="down",
                        help="down: 말단 링크 +X축을 아래로 향하게 제약")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target = np.asarray(args.xyz, dtype=float)
    result = solve(target, args.approach == "down")
    print("model=ROBOTIS omx_f.urdf; frame=link0; point=end_effector_link")
    print("robot_command=NONE; gripper_point_offset=UNMEASURED")
    print(f"target_m={target.tolist()}")
    print(f"predicted_m={np.round(result.position, 6).tolist()}")
    print(f"position_error_mm={result.position_error_m * 1000:.2f}")
    if args.approach == "down":
        print(f"tool_down_error_deg={result.down_error_deg:.2f}")
    for name, angle in zip(JOINT_NAMES, result.q):
        print(f"{name}={angle:.6f} rad ({math.degrees(angle):.2f} deg)")
    print("MODEL_FEASIBLE" if result.feasible else "MODEL_NOT_FEASIBLE")
    print("모터 펄스 변환·장비 가동 범위·충돌·실제 그리퍼 중심은 검증되지 않았습니다.")
    return 0 if result.feasible else 2


if __name__ == "__main__":
    raise SystemExit(main())
