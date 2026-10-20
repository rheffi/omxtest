#!/usr/bin/env python3
"""Portable entry point: demo, detection, dry-run, inspection and one live cycle."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

from pick_core import (ROOT, build_plan, choose, decode_detections, demo, execute,
                       get_rule, load_config, number, project_path, read_json,
                       validate_gripper, build_drop_plan)

MEASUREMENT = ROOT / "tools" / "measurement"
sys.path.insert(0, str(MEASUREMENT))


def legacy(filename, name):
    spec = importlib.util.spec_from_file_location(name, MEASUREMENT / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def profile_for(arm):
    from omx_arm_config import load_arm_profile
    return load_arm_profile(arm)


def require_job_arm(cfg, profile):
    if profile.name == "single":
        if cfg.get("serial_port") not in (None, str(profile.port)):
            raise ValueError("집기 설정의 장비가 현재 연결과 다릅니다. 설정 도구로 다시 기록하세요")
        return
    if cfg.get("arm") != profile.name:
        raise ValueError(f"집기 설정 arm을 {profile.name!r}으로 지정하세요. 팔별 집기 설정을 분리하세요")


def calibration_for(profile, rule, cfg):
    from omx_arm_config import require_camera, require_identity
    path = project_path(rule["calibration_file"]) if rule.get("calibration_file") else profile.calibration
    data = read_json(path)
    require_identity(data, profile)
    require_camera(data, profile)
    expected = {"method": "pixel_to_omx_model_xy_homography",
                "robot_frame": "link0", "robot_point": "end_effector_link",
                "motor_zero_pulses_hypothesis": [2048] * 5, "motor_joint_signs_hypothesis": [1] * 5}
    if any(data.get(key) != value for key, value in expected.items()):
        raise ValueError("지원하는 OMX 보정 형식/모터 영점·방향이 아닙니다")
    error = data.get("check_error")
    if error is not None:
        number(error.get("max_mm"), "check_error.max_mm", 0)
    return data, path


def calibration_report(data, cfg):
    return {"check_error": data.get("check_error"),
            "shelf_check_error": data.get("shelf_check_error")}


def saved_for(profile, hover_only):
    recorder = legacy("07_omx_pose_recorder.py", "pick_pose_recorder")
    names = ["hover_high"] if hover_only else ["hover_high", "drop_approach", "drop_release"]
    return {name: recorder.load_pose(name, profile)["raw_pulses"] for name in names}


def solver_for():
    import numpy as np
    fk = legacy("02_omx_xyz_offline.py", "pick_fk")

    def solve(xyz):
        result = fk.solve(np.asarray(xyz, dtype=float) / 1000, True)
        if not result.feasible or result.position_error_m > 0.002 or result.down_error_deg > 5:
            raise ValueError(f"도달 불가능한 모델 XYZ/아래 방향: {xyz}")
        raw = np.rint(2048 + np.asarray(result.q) * 4096 / (2 * math.pi)).astype(int)
        return [int(v) for v in raw]
    return solve


def print_json(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), flush=True)


def write_output(path, value):
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")


class Journal:
    def __init__(self, path=None):
        self.path = path
        self.stream = None

    def __enter__(self):
        if self.path is None:
            return self
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", encoding="utf-8", buffering=1)
        print(f"[LOG] {self.path}", flush=True)
        return self

    def __exit__(self, *args):
        if self.stream:
            self.stream.close()

    def emit(self, event, data):
        if self.stream is None:
            return
        record = {"at": datetime.now(timezone.utc).isoformat(), "event": event, "data": data}
        self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.stream.flush()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description="OMX YOLO 인식·집기·하역 (기본: 이동 없는 계획 출력)")
    parser.add_argument("--config", type=Path, default=(ROOT / "config/omx_pick.json" if (ROOT / "config/omx_pick.json").exists() else ROOT / "config/omx_pick.example.json"))
    parser.add_argument("--arm", choices=("single", "arm_a", "arm_b", "arm_test"), help=argparse.SUPPRESS)
    parser.add_argument("--class", dest="label", help="모델의 정확한 클래스 이름")
    parser.add_argument("--weights", help="로컬 .pt 파일; 프로젝트 기준 상대 경로 또는 절대 경로")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--demo", action="store_true", help="표준 라이브러리만으로 전체 모의 실행")
    source.add_argument("--image", type=Path, help="정지 이미지: 실물 이동 불가")
    source.add_argument("--detections", type=Path, help="검출 JSON: 실물 이동 불가")
    source.add_argument("--status", action="store_true", help="설정과 준비 상태 확인; 장비 연결 없음")
    source.add_argument("--torque-off", action="store_true", help="OFF 확인 후 토크만 해제; 카메라/집기설정 불필요")
    source.add_argument("--drop-test", action="store_true", help="카메라 없이 고정 드랍 경로 시험; 그리퍼 제어 없음")
    parser.add_argument("--torque-scope", choices=("all", "arm", "gripper"), default="all")
    parser.add_argument("--from-folded", action="store_true", help="설정의 ID13→ID12 중간값으로 펼친 후 HIGH")
    source.add_argument("--inspect", action="store_true", help="모터 PING/READ와 후보 FK 출력")
    source.add_argument("--gripper-test", choices=("open", "close"), help="그리퍼만 단독 시험; --live 필요")
    parser.add_argument("--detect-only", action="store_true", help="보정 없이 검출 목록만 출력")
    parser.add_argument("--hover", action="store_true", help="그리퍼/집기 없이 높은 호버와 복귀만")
    parser.add_argument("--live", action="store_true", help="현재 카메라 인식 + MOVE 확인 후 1회 실물 실행")
    parser.add_argument("--stage-seconds", type=float, help="이번 실행의 일반 팔 이동 시간만 변경 (초)")
    parser.add_argument("--target-index", type=int, default=0, help="허용된 같은 클래스 검출을 신뢰도 순으로 선택 (0부터)")
    parser.add_argument("--show", action="store_true", help="검출 정지 화면 표시; 키 입력으로 닫기")
    parser.add_argument("--output", type=Path, help="새 JSON 출력 또는 --live JSONL 기록 경로 (덮어쓰기 금지)")
    args = parser.parse_args(argv)
    if args.target_index < 0:
        parser.error("target-index는 0 이상입니다")
    if args.demo:
        if any((args.live, args.arm, args.label, args.weights, args.detect_only, args.show, args.from_folded)):
            parser.error("--demo에는 --hover와 --output만 함께 쓰세요")
        return args
    if args.torque_off or args.status or args.drop_test:
        if any((args.label, args.weights, args.detect_only, args.show, args.hover, args.gripper_test)):
            parser.error("상태/토크 해제/고정 드랍 시험에는 인식 옵션을 함께 쓰지 마세요")
        if args.torque_off or args.status:
            if args.live or args.from_folded:
                parser.error("상태/토크 해제에는 --live/--from-folded를 쓰지 마세요")
        return args
    if args.from_folded and (args.demo or args.inspect or args.gripper_test or args.detect_only):
        parser.error("--from-folded는 집기/호버/드랍 경로 시험에 사용하세요")
    if args.live and (args.image or args.detections or args.inspect or args.detect_only):
        parser.error("--live는 현재 카메라 또는 --gripper-test만 허용합니다")
    if args.inspect or args.gripper_test:
        if any((args.label, args.weights, args.detect_only, args.hover, args.show)):
            parser.error("장비 점검에는 인식/호버 옵션을 함께 쓰지 마세요")
        if args.gripper_test and not args.live:
            parser.error("--gripper-test에는 --live가 필요합니다")
    if args.detect_only and args.hover:
        parser.error("--detect-only와 --hover는 함께 쓸 수 없습니다")
    if args.detections and args.show:
        parser.error("검출 JSON에는 표시할 원본 영상이 없습니다")
    return args


def run_inspect(args, profile):
    from pick_robot import DynamixelBus, inspect
    with DynamixelBus(profile.port) as bus:
        rows = inspect(bus)
    # FK is informational; no motor writes or setup-config edits.
    import numpy as np
    fk = legacy("02_omx_xyz_offline.py", "pick_inspect_fk")
    q = (np.asarray([row["position"] for row in rows[:5]]) - 2048) * (2 * math.pi / 4096)
    xyz, orientation = fk.forward(q)
    down = math.degrees(math.acos(float(np.clip(orientation[:, 0] @ [0, 0, -1], -1, 1))))
    result = {"arm": profile.name, "motors": rows, "candidate_model_xyz_mm": (xyz * 1000).tolist(),
              "candidate_down_error_deg": down,
              "note": "현재 자세를 읽은 후보값. 실제 그리퍼 중심/집기 높이는 실측·검증 필요."}
    print_json(result)
    write_output(args.output, result)


def run_gripper_test(args, profile, cfg):
    from pick_robot import DynamixelBus, MotionController
    require_job_arm(cfg, profile)
    grip = validate_gripper(cfg)
    with Journal(args.output) as journal, DynamixelBus(profile.port) as bus:
        driver = MotionController(bus, cfg, journal.emit)
        driver.prepare(gripper_only=True)
        journal.emit("gripper_test", {"arm": profile.name, "action": args.gripper_test, "config": cfg})
        print(f"[GRIPPER ONLY] {profile.name}: {args.gripper_test}; current cap={grip['current_limit_raw']}")
        if input("그리퍼 주변을 확인하고 실행하려면 GRIPPER 입력: ").strip() != "GRIPPER":
            journal.emit("cancelled", {})
            return
        try:
            driver.prepare(gripper_only=True)
            driver.enable()
            driver.gripper("OPEN" if args.gripper_test == "open" else "CLOSE",
                           grip["open_pulse"] if args.gripper_test == "open" else grip["close_pulse"])
            journal.emit("completed", {"physical_success": "UNCONFIRMED"})
        except BaseException as exc:
            driver.hold()
            journal.emit("failed", {"error": str(exc)})
            raise
        finally:
            print("[HOLD] 그리퍼 토크 유지. 필요하면 기존 수동 조작 방법으로 해제하세요.")


def run_pipeline(args, profile, cfg):
    from pick_vision import Camera, Detector, show, capture_target
    with ExitStack() as stack:
        if not args.detect_only:
            require_job_arm(cfg, profile)
            rule = get_rule(cfg, args.label)
            calibration, calibration_path = calibration_for(profile, rule, cfg)
            measured = (calibration.get("check_error") or {}).get("max_mm")
            measured_text = f"{measured:.2f}mm" if measured is not None else "미기록"
            print(f"[CALIBRATION] 독립 확인 최대 오차={measured_text}", flush=True)
            if cfg.get("regions_source") == "calibration":
                if not calibration.get("regions_uv"):
                    raise ValueError("fit --layout two-shelves로 선반 ROI를 생성하세요")
                cfg = dict(cfg, regions_uv=calibration["regions_uv"])
        camera = detector = frame = None
        if args.detections:
            width, height, detections = decode_detections(read_json(args.detections))
        else:
            detector = Detector(cfg, args.weights)
            if args.image:
                import cv2
                frame = cv2.imread(str(args.image))
                if frame is None:
                    raise ValueError(f"이미지 읽기 실패: {args.image}")
                height, width = frame.shape[:2]
                detections = detector.predict(frame)
            else:
                camera = stack.enter_context(Camera(profile))
                if args.detect_only:
                    frame, _ = camera.read()
                    detections = detector.predict(frame)
                else:
                    target, frame, _ = capture_target(camera, detector, cfg, args.label, args.target_index)
                    detections = [target]
                height, width = frame.shape[:2]
        if args.detect_only:
            result = {"width": width, "height": height,
                      "model_classes": detector.model.names if detector else None,
                      "detections": [{"class_name": d.label, "class_id": d.class_id,
                                      "confidence": d.confidence, "xyxy": d.box} for d in detections]}
            print_json(result)
            write_output(args.output, result)
            if args.show:
                show(frame, detections)
            return
        require_job_arm(cfg, profile)
        target = (target if camera else choose(detections, cfg, args.label, width, height, args.target_index))
        rule = get_rule(cfg, target.label)
        if (width, height) != (calibration["width"], calibration["height"]):
            raise ValueError("입력 이미지/검출 JSON의 해상도가 선택한 팔의 보정과 다릅니다")
        saved = saved_for(profile, args.hover)
        print("[PLAN] IK 계산 중. 모터에는 연결하지 않습니다.", flush=True)
        plan = build_plan(cfg, calibration, target, saved, solver_for(), hover_only=args.hover, from_folded=args.from_folded)
        plan.update(arm=profile.name, serial_port=str(profile.port), calibration_file=str(calibration_path),
                    calibration_check=calibration_report(calibration, cfg))
        if args.live:
            print(f"[TARGET] {target.label}: XY={plan['pick_model_xyz_mm'][:2]}")
        else:
            print_json(plan)
        if args.show:
            show(frame, detections, target, cfg)
        if not args.live:
            write_output(args.output, {"mode": "DRY_RUN_NO_MOTOR_CONNECTION", "plan": plan})
            return
        from pick_robot import DynamixelBus, MotionController
        journal = stack.enter_context(Journal(args.output))
        bus = stack.enter_context(DynamixelBus(profile.port))
        driver = MotionController(bus, cfg, journal.emit)
        journal.emit("plan", {"plan": plan, "config": cfg})
        driver.prepare(saved["hover_high"], use_gripper=not args.hover, from_folded=args.from_folded)
        if input(f"{profile.name}: 주변과 경로를 확인하고 MOVE 입력: ").strip() != "MOVE":
            journal.emit("cancelled", {})
            print("[CANCELLED] 모터 목표·토크를 변경하지 않았습니다.")
            return
        try:
            driver.enable()
            execute(plan, driver, emit=journal.emit)
            journal.emit("completed", {"command_sequence": "SENT", "physical_success": "OPERATOR_VERIFY"})
            print("[DONE] 목표 전송 순서 종료. 실제 도착·집기·하역 결과를 확인하세요. 토크는 유지합니다.")
        except BaseException as exc:
            journal.emit("failed", {"error": str(exc)})
            raise


def run_status(args, profile):
    cfg = read_json(args.config)
    pending = []
    calibration_checks = {}
    try:
        load_config(args.config)
        require_job_arm(cfg, profile)
    except (ValueError, KeyError) as exc:
        pending.append(str(exc))
    for name in ("hover_high", "drop_approach", "drop_release"):
        try:
            recorder = legacy("07_omx_pose_recorder.py", "pick_status_poses")
            recorder.load_pose(name, profile)
        except (OSError, ValueError, KeyError) as exc:
            pending.append(str(exc))
    try:
        validate_gripper(cfg)
    except (ValueError, KeyError) as exc:
        pending.append(str(exc))
    classes = cfg.get("classes", {})
    if not classes:
        pending.append("집기 클래스 설정 필요")
    for label, rule in classes.items():
        try:
            cal, _ = calibration_for(profile, rule, cfg)
            calibration_checks[label] = calibration_report(cal, cfg)
            number(rule.get("pick_z_mm"), "pick_z_mm")
            regions = cal.get("regions_uv") if cfg.get("regions_source") == "calibration" else cfg.get("regions_uv")
            if not regions:
                pending.append(label + ": 선반별 ROI 필요")
            if cfg.get("regions_source") == "calibration" and not cal.get("regions_uv"):
                pending.append("선반 ROI: fit --layout two-shelves 필요")
        except (OSError, ValueError, KeyError) as exc:
            pending.append(label + ": " + str(exc))
    weights = cfg.get("weights")
    if not weights or not project_path(weights).is_file():
        pending.append("모델 경로 필요")
    print_json({"mode": "STATUS_NO_HARDWARE", "arm": profile.name, "config": str(args.config),
                "calibration_checks": calibration_checks,
                "pending": pending, "unfold": cfg.get("unfold"), "basket": cfg.get("basket"),
                "note": "장비/물리 성공은 별도 확인. 기존 수치 제한은 설정 파일에 표시됩니다."})


def run_torque_off(args, profile):
    from pick_robot import DynamixelBus, torque_off
    if input(f"팔·상품을 받친 뒤 {args.torque_scope} 토크 해제: OFF 입력: ").strip() != "OFF":
        return 0
    with DynamixelBus(profile.port) as bus:
        results = torque_off(bus, args.torque_scope)
    print_json(results)
    return 0 if all(r["confirmed"] for r in results.values()) else 2


def run_drop_test(args, profile, cfg):
    require_job_arm(cfg, profile)
    saved = saved_for(profile, False)
    plan = build_drop_plan(cfg, saved, args.from_folded)
    print_json(plan)
    if not args.live:
        write_output(args.output, {"mode": "DRY_RUN_NO_MOTOR_CONNECTION", "plan": plan})
        return
    from pick_robot import DynamixelBus, MotionController
    with Journal(args.output) as journal, DynamixelBus(profile.port) as bus:
        driver = MotionController(bus, cfg, journal.emit)
        driver.prepare(saved["hover_high"], use_gripper=False, from_folded=args.from_folded)
        if input("빈 그리퍼의 고정 드랍 경로 시험: MOVE 입력: ").strip() != "MOVE":
            journal.emit("cancelled", {})
            return
        try:
            journal.emit("plan", {"plan": plan, "config": cfg})
            driver.enable()
            execute(plan, driver, emit=journal.emit)
            journal.emit("completed", {"physical_success": "OPERATOR_VERIFY"})
        except BaseException as exc:
            journal.emit("failed", {"error": str(exc)})
            raise
        finally:
            print("[HOLD] 팔 토크 유지; 필요하면 --torque-off를 사용하세요.")


def main(argv=None):
    args = arguments(argv)
    if args.demo:
        result = demo(args.hover)
        print_json(result)
        write_output(args.output, result)
        return 0
    profile = profile_for(args.arm)
    if args.torque_off:
        return run_torque_off(args, profile)
    if args.status:
        run_status(args, profile)
        return 0
    if args.inspect:
        run_inspect(args, profile)
        return 0
    cfg = load_config(args.config)
    if args.stage_seconds is not None:
        if number(args.stage_seconds, "stage-seconds", 0) == 0:
            raise ValueError("stage-seconds는 양수여야 합니다")
        cfg = dict(cfg, stage_seconds=args.stage_seconds)
    if not (args.drop_test or args.gripper_test or args.detect_only) and not args.label:
        names = list(cfg["classes"])
        if len(names) != 1 or names[0] == "REPLACE_WITH_EXACT_MODEL_CLASS_NAME":
            raise ValueError("설정 도구로 시험할 상품을 기록하세요. 여러 종류 설정은 --class로 선택하세요")
        args.label = names[0]
    if args.drop_test:
        run_drop_test(args, profile, cfg)
    elif args.gripper_test:
        run_gripper_test(args, profile, cfg)
    else:
        run_pipeline(args, profile, cfg)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("[INTERRUPTED] 중단했습니다. 실제 팔·물체·토크 상태를 확인하세요.", file=sys.stderr)
        raise SystemExit(130)
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, ImportError, EOFError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        if isinstance(exc, ImportError):
            print("실행할 가상환경에서 python -m pip install -r requirements-pick.txt 를 실행하세요.", file=sys.stderr)
        raise SystemExit(2)
