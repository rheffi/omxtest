#!/usr/bin/env python3
"""Choose a product and record pick/gripper states with PING/READ only."""
import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

from pick_core import (ROOT, ARM_IDS, MODELS, read_json, project_path, number,
                       integer, validate_gripper)

sys.path.insert(0, str(ROOT / "tools" / "measurement"))
from omx_arm_config import load_arm_profile, require_camera, require_identity
from omx_workspace import validate_regions
from omx_wrist import canonical_wrist


KNOWN_UNFOLD_SERIAL = "usb-ROBOTIS_OpenRB-150_1B8E02E050323952582E3120FF071F2E-if00"


def class_names(weights):
    path = weights.parent / "classes.txt"
    if path.exists():
        names = [line.strip() for line in path.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    else:
        from ultralytics import YOLO
        values = YOLO(str(weights)).names
        names = list(values.values()) if isinstance(values, dict) else list(values)
    if not names or len(set(names)) != len(names):
        raise ValueError("모델 클래스 목록이 비었거나 중복됐습니다")
    return names


def choose_class(weights, supplied=None):
    names = class_names(weights)
    if supplied is not None:
        if supplied not in names:
            raise ValueError("모델에 없는 클래스명입니다")
        return supplied
    print("시험할 상품을 번호로 고르세요:")
    for index, label in enumerate(names, 1):
        print(f"  {index}. {label}")
    value = input("상품 번호: ").strip()
    if not value.isdecimal() or not 1 <= int(value) <= len(names):
        raise ValueError("목록에 표시된 상품 번호를 입력하세요")
    return names[int(value) - 1]


def read_snapshot(profile):
    """Close the port before returning. No torque/goal/config writes."""
    from pick_robot import DynamixelBus, inspect
    from omx_pick import legacy
    import numpy as np
    with DynamixelBus(profile.port) as bus:
        rows = inspect(bus)
    for row in rows:
        motor_id = row["id"]
        expected_model = 1200 if motor_id == 16 else MODELS[ARM_IDS.index(motor_id)]
        if (row["model"] != expected_model or row["mode"] != (5 if motor_id == 16 else 4)
                or row["homing_offset"] != 0 or row["hardware_error"]):
            raise ValueError(f"ID {motor_id}: 모델·동작모드·영점·오류 상태를 확인하세요")
        if motor_id == 15:
            canonical_wrist(row["position"])
        else:
            integer(row["position"], f"ID {motor_id} position", 0, 4095)
    pulses = [canonical_wrist(row["position"]) if row["id"] == 15 else row["position"] for row in rows[:5]]
    q = (np.asarray(pulses) - 2048) * (2 * math.pi / 4096)
    xyz, orientation = legacy("02_omx_xyz_offline.py", "pick_setup_fk").forward(q)
    return {"recorded_at": datetime.now().astimezone().isoformat(), "motors": rows,
            "arm_pulses": pulses, "model_xyz_mm": (xyz * 1000).tolist(),
            "model_down_error_deg": math.degrees(math.acos(float(np.clip(orientation[:, 0] @ [0, 0, -1], -1, 1))))}


def atomic_record(path, data, before):
    """Persist partial recordings, checking for another writer before replace."""
    if (path.read_bytes() if path.exists() else None) != before:
        raise RuntimeError("다른 실행에서 측정 기록이 변경됐습니다. 덮어쓰지 않습니다")
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile("w", dir=path.parent, prefix=path.name+".", suffix=".tmp",
                                         delete=False, encoding="utf-8") as f:
            temp_name = f.name
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write("\n")
        if (path.read_bytes() if path.exists() else None) != before:
            raise RuntimeError("저장 직전 측정 기록이 변경됐습니다. 덮어쓰지 않습니다")
        os.replace(temp_name, path)
    finally:
        if temp_name and Path(temp_name).exists():
            Path(temp_name).unlink()


def capture_values(profile, calibration, label, from_folded, rerecord=False):
    path = profile.calibration.parent / "omx_pick_setup_samples.json"
    digest = hashlib.sha256(profile.calibration.read_bytes()).hexdigest()
    before = path.read_bytes() if path.exists() else None
    registry = json.loads(before) if before is not None else None
    if registry is not None:
        require_identity(registry, profile)
        if registry.get("schema_version") != 1 or not isinstance(registry.get("classes"), dict):
            raise ValueError("집기 측정 기록 형식이 올바르지 않습니다")
    if registry is None:
        registry = dict(profile.identity(), schema_version=1, calibration_sha256=digest, classes={})
    if rerecord:
        registry["classes"].pop(label, None)
    samples = registry["classes"].setdefault(label, {})

    def record(kind, instruction):
        nonlocal before
        if kind in samples:
            print(f"[재사용] {kind}: {samples[kind]['recorded_at']}")
            return samples[kind]
        print(instruction)
        print("손으로 맞추거나 기존 제어를 사용한 뒤, 다른 제어 프로그램은 종료하세요. 이 도구는 움직이지 않습니다.")
        if input("준비되면 Enter로 읽기·기록 (Q: 취소): ").strip().upper() == "Q":
            raise KeyboardInterrupt
        sample = read_snapshot(profile)
        if kind == "close":
            grip = sample["motors"][-1]
            current = integer(grip["goal_current_raw"], "읽은 그리퍼 Goal Current", 1, 1750)
            if current > grip["current_limit_raw"]:
                raise ValueError("현재 Goal Current가 모터 Current Limit보다 큽니다. 기존 제어 설정을 확인하세요")
            if grip["position"] == samples["open"]["motors"][-1]["position"]:
                raise ValueError("열림과 닫힘 상태가 같습니다. 실제로 닫은 뒤 다시 실행하세요. 앞 기록은 보존됩니다")
        samples[kind] = sample
        atomic_record(path, registry, before)
        before = path.read_bytes()
        print(f"[기록] {kind}: 모델 XYZ={sample['model_xyz_mm']}, 그리퍼={sample['motors'][-1]['position']}")
        return sample

    pick = record("pick", "[1/3] 선택한 상품을 실제로 잡을 높이·방향에 그리퍼를 맞추세요. 상품을 누르지 마세요.")
    opened = record("open", "[2/3] 물건 없이 그리퍼를 상품이 들어갈 만큼 벌리세요.")
    closed = record("close", "[3/3] 물건 없이 그리퍼를 사용할 닫힘 상태로 맞추세요.")
    gripper = closed["motors"][-1]
    current = integer(gripper["goal_current_raw"], "읽은 그리퍼 Goal Current", 1, 1750)
    if current > gripper["current_limit_raw"]:
        raise ValueError("현재 Goal Current가 모터 Current Limit보다 큽니다. 기존 제어 설정을 확인하세요")
    print(f"[전류 재사용] 닫힘 기록의 모터 16 Goal Current={current}. Current Limit를 새 목표로 복사하지 않습니다.")
    print("파지력 적합성은 아직 미확인입니다. 실제 집기 성공과 별도로 확인하세요.")
    values = dict(pick_z_mm=pick["model_xyz_mm"][2], open=opened["motors"][-1]["position"],
                  close=gripper["position"], current=current)
    unfold_source = None
    if from_folded:
        if profile.port.name == KNOWN_UNFOLD_SERIAL:
            values.update(elbow=2559, shoulder=1616)
            unfold_source = "previous_operator_verified_same_serial"
            print("[펼침 재사용] 이 OMX에서 이전에 확인한 ID 13=2559 → ID 12=1616")
        else:
            elbow = record("unfold_elbow", "접힌 상태에서 팔꿈치만 먼저 펼친 중간 자세를 맞추세요.")
            shoulder = record("unfold_shoulder", "이어서 어깨를 세운 중간 자세를 맞추세요.")
            values.update(elbow=elbow["arm_pulses"][2], shoulder=shoulder["arm_pulses"][1])
            unfold_source = "operator_recorded_intermediate_poses"
    target = input("적재 대상 1: 실제 핑키프로 / 2: 임시 바구니: ").strip()
    if target not in ("1", "2"):
        raise ValueError("적재 대상은 1 또는 2로 선택하세요. 측정 기록은 저장돼 있습니다")
    values.update(basket_type="pinkypro" if target == "1" else "temporary", mount="unspecified",
                  note=input("바구니 정차 위치 메모 (Enter: 생략): ").strip())
    values["setup_measurements"] = dict(samples_file=str(path), calibration_sha256=digest,
        current_source="ID16 Goal Current(102) read at close", current_force_verified=False,
        collision_verified=False,
        unfold_source=unfold_source, pick_arm_pulses=pick["arm_pulses"],
        pick_model_down_error_deg=pick["model_down_error_deg"])
    return values


def manual_values(args):
    """Legacy explicit-number mode, retained for existing integration callers."""
    values = {"pick_z_mm": float(input("집기 모델Z mm: ")),
              "open": int(input("빈 그리퍼 열림 pulse: ")),
              "close": int(input("빈 그리퍼 닫힘 pulse: ")),
              "current": int(input("시험에 사용할 Goal Current 원시값: "))}
    if args.from_folded:
        values.update(elbow=int(input("ID13 펼침 중간 pulse: ")),
                      shoulder=int(input("ID12 펼침 중간 pulse: ")))
    values.update(basket_type=input("적재 대상 pinkypro / temporary: ").strip(),
                  mount=input("장착 front / rear / unspecified: ").strip(),
                  note=input("정차 위치 메모: ").strip())
    return values


def make_config(profile, calibration, weights, label, values):
    require_identity(calibration, profile)
    require_camera(calibration, profile)
    regions = validate_regions(calibration.get("regions_uv"), profile.width, profile.height)
    cfg = read_json(ROOT / "config/omx_pick.example.json")
    rule = cfg["classes"].pop("REPLACE_WITH_EXACT_MODEL_CLASS_NAME")
    rule.update(pick_z_mm=number(values["pick_z_mm"], "pick_z_mm"))
    if "verified" in values:
        rule["calibration_plane_verified"] = values["verified"]
    cfg.update(arm=profile.name, weights=str(weights), confidence=0.5,
               regions_source="calibration", regions_uv=regions,
               classes={label: rule})
    if profile.name == "single":
        cfg.pop("arm", None)
        cfg["serial_port"] = str(profile.port)
    cfg["gripper"].update(open_pulse=values["open"], close_pulse=values["close"],
                          current_limit_raw=values["current"])
    validate_gripper(cfg)
    cfg["unfold"] = {"elbow_pulse": values.get("elbow"), "shoulder_pulse": values.get("shoulder")}
    if values.get("elbow") is not None:
        from pick_core import unfold_stages
        unfold_stages(cfg, True)
    cfg["basket"] = {"type": values["basket_type"], "shelf_to_basket_mm": 110,
                     "mount": values["mount"], "note": values["note"], "target_method": "saved_joint_poses"}
    if values.get("setup_measurements"):
        cfg["setup_measurements"] = values["setup_measurements"]
        rule["grasp_reference_pulses"] = list(values["setup_measurements"]["pick_arm_pulses"])
    return cfg


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", help=argparse.SUPPRESS)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--class", dest="label")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--from-folded", action="store_true", help="기록된 ID13→ID12 펼침 값을 사용")
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--rerecord", action="store_true", help="선택 상품의 집기·그리퍼 상태를 다시 읽어 기록")
    parser.add_argument("--manual", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    profile = load_arm_profile(args.arm)
    weights = project_path(args.weights).resolve()
    if not weights.is_file() or weights.suffix.lower() != ".pt":
        raise ValueError("로컬 .pt 경로를 확인하세요")
    path = args.config or ROOT / "config" / ("omx_pick.json" if profile.name == "single" else f"omx_pick.{profile.name}.laptop.json")
    if path.exists() and not args.replace:
        raise FileExistsError(f"설정이 있습니다: {path}; 갱신은 --replace")
    if args.replace and not path.exists():
        raise ValueError("교체할 설정이 없습니다. --replace를 빼세요")
    before = path.read_bytes() if path.exists() else None
    calibration = read_json(profile.calibration)
    require_identity(calibration, profile)
    require_camera(calibration, profile)
    if not calibration.get("regions_uv"):
        raise ValueError("먼저 fit --layout two-shelves로 보정·ROI를 저장하세요")
    print("숫자 입력 대신 실제 상태를 읽어 기록합니다. 모터 목표·토크·설정은 쓰지 않습니다.")
    print("직렬 포트를 열 때 OpenRB가 재시작될 수 있으므로 팔을 받치세요. 읽기 사이에는 포트를 닫습니다.")
    print("보정 확인 오차:", calibration.get("check_error"))
    if args.manual:
        label = args.label or input("모델의 정확한 클래스명: ").strip()
        if not label:
            raise ValueError("클래스명을 입력하세요")
        if (weights.parent / "classes.txt").exists():
            choose_class(weights, label)
    else:
        from omx_pick import saved_for
        saved_for(profile, False)
        print("[자세 재사용] 저장된 hover_high / drop_approach / drop_release")
        label = choose_class(weights, args.label)
    values = manual_values(args) if args.manual else capture_values(profile, calibration, label, args.from_folded, args.rerecord)
    if values["basket_type"] not in ("pinkypro", "temporary"):
        raise ValueError("실제 적재 대상 종류를 입력하세요")
    if values["mount"] not in ("front", "rear", "unspecified"):
        raise ValueError("장착 방향을 확인하세요")
    cfg = make_config(profile, calibration, weights, label, values)
    print(f"[설정 요약] 상품={label}, 기록한 집기 모델 Z={values['pick_z_mm']:.2f}mm")
    print(f"그리퍼 열림={values['open']}, 닫힘={values['close']}, 재사용 Goal Current={values['current']}")
    print("바구니:", cfg["basket"]["type"], "펼침:", cfg["unfold"])
    word = "REPLACE" if before is not None else "SAVE"
    if input(f"저장하려면 {word}: ").strip() != word:
        print("[CANCELLED] 저장하지 않았습니다")
        return 0
    if before is None:
        with path.open("x", encoding="utf-8") as f:
            f.write(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")
    else:
        if path.read_bytes() != before:
            raise RuntimeError("확인 중 설정이 변경됐습니다. 덮어쓰지 않습니다")
        # Preserve previous measured values before replacing the job file.
        from datetime import datetime
        backup = path.with_name(path.name + "." + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".bak")
        with backup.open("xb") as f:
            f.write(before)
        import tempfile, os
        temp_name = None
        try:
            with tempfile.NamedTemporaryFile("w", dir=path.parent, prefix=path.name+".", suffix=".tmp", delete=False, encoding="utf-8") as f:
                temp_name = f.name
                f.write(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")
            if path.read_bytes() != before:
                raise RuntimeError("저장 직전 설정이 변경됐습니다. 덮어쓰지 않습니다")
            os.replace(temp_name, path)
        finally:
            if temp_name and Path(temp_name).exists():
                Path(temp_name).unlink()
    print("[SAVED]", path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, RuntimeError, EOFError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2)
    except KeyboardInterrupt:
        print("[CANCELLED] 집기 설정 저장 전 중단. 이미 읽어 기록한 측정값은 다음 실행에서 이어 씁니다.", file=sys.stderr)
        raise SystemExit(130)
