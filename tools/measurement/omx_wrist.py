"""Nearest equivalent ID 15 goal for the already configured mode 4 motor.

Keep a small software window around the recorded 0..4095 turn. This is not a
mechanical clearance guarantee or a change to the motor's EEPROM limits.
"""

from numbers import Integral

WRIST_MIN = -4096
WRIST_MAX = 8191


def canonical_wrist(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or not WRIST_MIN <= value <= WRIST_MAX:
        raise ValueError(f"ID 15 현재 위치가 소프트웨어 범위 {WRIST_MIN}..{WRIST_MAX} 밖입니다")
    return int(value) % 4096


def nearest_wrist(target, current):
    """26 -> 4069 becomes 26 -> -27, never a 4043-pulse revolution."""
    canonical_wrist(current)
    target = canonical_wrist(target)
    delta = (target - int(current) + 2048) % 4096 - 2048
    if abs(delta) == 2048:
        raise ValueError("ID 15 목표가 정확히 180도 차이입니다. 집기·저장 자세 방향을 확인하세요")
    goal = int(current) + delta
    if not WRIST_MIN <= goal <= WRIST_MAX:
        raise ValueError("ID 15의 짧은 이동 목표가 소프트웨어 범위를 벗어납니다. 손목 누적 회전을 확인하세요")
    return goal
