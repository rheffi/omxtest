#!/usr/bin/env python3
"""A: 슬롯 1·2 네 자세를 Enter로 읽어 저장. 모터 목표·토크 쓰기 없음."""
from datetime import datetime
import argparse
import json
from pathlib import Path

from omx_arm_config import load_arm_profile
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[2]
NAMES = (
    ("slot1_approach", "슬롯 1: 물체를 든 채 주변을 피할 접근 자세"),
    ("slot1_release", "슬롯 1: 실제 원하는 놓기 자세"),
    ("slot2_approach", "슬롯 2: 물체를 든 채 주변을 피할 접근 자세"),
    ("slot2_release", "슬롯 2: 실제 원하는 놓기 자세"),
)


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    spec = importlib.util.spec_from_file_location("slot_recorder", Path(__file__).with_name("07_omx_pose_recorder.py"))
    recorder = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = recorder
    spec.loader.exec_module(recorder)
    profile = load_arm_profile(None)
    job = json.loads((ROOT/"config/omx_pick.json").read_text())
    registry = recorder.load_registry(profile)
    wrist = registry["poses"]["hover_high"]["raw_pulses"][4]
    print(f"poses={profile.poses}")
    print(f"손목 ID15 기준={wrist}, 정책={job.get('wrist_policy')}")
    print("팔을 받치고 토크를 해제한 뒤 직접 자세를 맞추세요. 손목은 기준 방향으로 유지하세요.")
    print("각 자세에서 Enter → 현재 엔코더를 읽어 즉시 저장. Q → 종료.")
    print("모터 목표·토크 명령·IK 계산은 하지 않습니다. 슬롯 0은 변경하지 않습니다.")
    backup = ROOT/"records/single/archive"/("slot_reteach_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    saved = []
    for name, description in NAMES:
        if input(f"\n{description}로 맞춘 뒤 Enter (Q: 종료): ").strip().upper() == "Q":
            break
        before = recorder.load_registry(profile)
        state = recorder.snapshot(profile)
        backup.mkdir(parents=True, exist_ok=True)
        with (backup/f"{name}.before.json").open("x") as stream:
            json.dump(before["poses"][name], stream, ensure_ascii=False, indent=2)
        if before != recorder.load_registry(profile):
            raise RuntimeError("저장 자세 파일이 다른 프로그램에서 변경됐습니다")
        before["poses"][name] = dict(
            recorded_at=datetime.now().astimezone().isoformat(timespec="seconds"),
            motor_ids=list(recorder.IDS), motor_models=list(recorder.MODELS),
            operating_modes=[4]*5, homing_offsets=[0]*5, raw_pulses=state["positions"])
        recorder.atomic_save(before, profile)
        print(f"[SAVED] {name}: {state['positions']}; ID15 실측={state['positions'][4]}, 기준={wrist}")
        saved.append(name)
    print("저장:", ", ".join(saved) or "없음")
    if saved:
        print("기존 자세 백업:", backup)
    print("집기·적재 시험은 시작하지 않았습니다.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n기록 종료")
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1)
