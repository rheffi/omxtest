#!/usr/bin/env python3
"""OMX-F overhead camera 9-point calibration; no robot motion commands.

collect: click a visible cross, then manually align the gripper over it;
         read present joint positions (PING/READ only) and optionally save.
fit:     fit pixel-to-model-XY homography using nine or more train samples.
preview: transform a clicked pixel to XY and run offline URDF IK only.

The default pulse-to-URDF mapping is a hypothesis. A fitted mapping is useful
only after independent check points and a separately approved physical hover.
The camera, resolution, marker plane, gripper orientation, and motor mapping
must stay fixed. Opening the OpenRB serial port may reset the controller.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from omx_workspace import shelf_regions

import cv2
import numpy as np

from omx_arm_config import (add_arm_argument, load_arm_profile, require_camera,
                            require_explicit_arm, require_identity)


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-HFSD-231214-J_SNAP_U2-video-index0"
DEFAULT_READER_PYTHON = Path(sys.executable)
DEFAULT_SAMPLES = ROOT / "records" / "omx_xy_samples.csv"
DEFAULT_CALIBRATION = ROOT / "config" / "omx_xy_calibration.json"
CSV_FIELDS = (
    "timestamp", "purpose", "point_name", "camera", "width", "height",
    "plane_height_mm", "u", "v", "x_mm", "y_mm", "z_mm",
    "raw_pulses", "zero_pulses", "joint_signs", "joint_radians",
    "tool_axis_x", "tool_axis_y", "tool_axis_z",
)
ARM_CSV_FIELDS = CSV_FIELDS + ("arm", "serial_port")


def load_module(filename: str, module_name: str):
    path = HERE / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"모듈을 읽지 못했습니다: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def require_finite(values, name: str) -> None:
    if not np.isfinite(np.asarray(values, dtype=float)).all():
        raise ValueError(f"{name}에 유한한 숫자만 입력하세요")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 없이 OMX-F 상단 카메라 평면·좌우 선반 XY 보정")
    sub = parser.add_subparsers(dest="command", required=True)
    collect = sub.add_parser("collect", help="십자 클릭 후 현재 관절값을 READ하여 한 점 기록")
    add_arm_argument(collect)
    collect.add_argument("--point-name", required=True, help="예: left_top, center")
    collect.add_argument("--purpose", choices=("train", "check"), default="train")
    collect.add_argument("--plane-height-mm", type=float,
                         help="선택: 작업대에서 십자 윗면까지 실제 측정한 높이. 미측정이면 생략")
    collect.add_argument("--camera")
    collect.add_argument("--width", type=int)
    collect.add_argument("--height", type=int)
    collect.add_argument("--fps", type=int)
    collect.add_argument("--uv", nargs=2, type=float, metavar=("U", "V"),
                         help="이미 알고 있는 영상 좌표; 카메라 창을 열지 않음")
    collect.add_argument("--raw", nargs=5, type=int, metavar="PULSE",
                         help="이미 읽어 둔 관절 펄스; 직렬 포트를 열지 않음")
    collect.add_argument("--zero", nargs=5, type=int, default=[2048] * 5,
                         metavar="PULSE", help="URDF 0 rad의 관절별 펄스 후보")
    collect.add_argument("--sign", nargs=5, type=int, default=[1] * 5,
                         metavar="DIR", help="관절별 회전 방향 후보: 1 또는 -1")
    collect.add_argument("--reader-python", type=Path, default=DEFAULT_READER_PYTHON,
                         help="Dynamixel SDK가 설치된 모터 읽기 Python")
    collect.add_argument("--output", type=Path)
    fit = sub.add_parser("fit", help="train 점으로 변환식을 계산하고 check 점 오차 표시")
    add_arm_argument(fit)
    fit.add_argument("--layout", choices=("single-plane", "two-shelves"), default="single-plane",
                     help="two-shelves: 좌우 각 5점과 확인 각 2점, 선반 ROI 저장")
    fit.add_argument("--input", type=Path)
    fit.add_argument("--output", type=Path)
    fit.add_argument("--overwrite", action="store_true",
                     help="기존 보정 JSON을 명시적으로 다시 계산할 때만 사용")
    preview = sub.add_parser("preview", help="영상 좌표의 XY와 모델 IK만 출력; 장비 명령 없음")
    add_arm_argument(preview)
    preview.add_argument("--input", type=Path)
    preview.add_argument("--uv", nargs=2, type=float, metavar=("U", "V"))
    preview.add_argument("--camera")
    preview.add_argument("--hover-mm", required=True, type=float,
                         help="보정 때 맞춘 말단 높이보다 높일 거리; 출력 계산에만 사용")
    preview.add_argument("--approach", choices=("any", "down"), default="down")
    return parser.parse_args()


def configure_args(args, profile):
    """Use the selected device's artifact paths and capture dimensions."""
    args.profile = profile
    samples = profile.samples if profile else DEFAULT_SAMPLES
    calibration = profile.calibration if profile else DEFAULT_CALIBRATION
    paths = ({"output": samples} if args.command == "collect" else
             {"input": samples, "output": calibration} if args.command == "fit" else
             {"input": calibration})
    for key, expected in paths.items():
        supplied = getattr(args, key)
        if profile and supplied is not None and supplied.expanduser().resolve() != expected.resolve():
            raise ValueError(f"현재 장비의 {key} 기본 경로를 사용하세요: {expected}")
        setattr(args, key, supplied if supplied is not None else expected)
    if args.command != "fit":
        settings = {"camera": profile.camera if profile else DEFAULT_CAMERA}
        if args.command == "collect":
            settings.update(width=profile.width if profile else 640,
                            height=profile.height if profile else 480,
                            fps=profile.fps if profile else 30)
        for key, expected in settings.items():
            supplied = getattr(args, key)
            if profile and supplied is not None and supplied != expected:
                if profile.name == "single":
                    raise ValueError(f"현재 단일 구성의 {key}={expected}로 수집하세요")
                raise ValueError(f"{key} 설정은 config/omx_arm_profiles.json에서 변경하세요")
            setattr(args, key, supplied if supplied is not None else expected)
    if profile:
        profile.describe()
        for key in paths:
            print(f"{key}={getattr(args, key)}")


def click_pixel(camera: str, width: int, height: int, fps: int) -> tuple[float, float, int, int]:
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("카메라 폭·높이·fps는 양수여야 합니다")
    path = Path(camera)
    if not path.exists():
        raise FileNotFoundError(f"카메라가 없습니다: {camera}")
    cap = cv2.VideoCapture(str(path), cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 열지 못했습니다: {camera}")
    window = "OMX cross: click center, Enter confirm, R recapture, Q cancel"
    clicked: list[tuple[int, int]] = []
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        for _ in range(12):
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("상단 카메라 영상을 읽지 못했습니다")
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

        def on_mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked[:] = [(x, y)]

        cv2.setMouseCallback(window, on_mouse)
        print("고정된 화면에서 십자 중심 클릭 → Enter. R: 다시 촬영, Q/Esc: 취소")
        while True:
            view = frame.copy()
            if clicked:
                cv2.drawMarker(view, clicked[0], (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                raise RuntimeError("화면 좌표 수집을 취소했습니다")
            if key in (ord("r"), ord("R")):
                clicked.clear()
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError("상단 카메라 영상을 다시 읽지 못했습니다")
            if key in (10, 13) and clicked:
                return float(clicked[0][0]), float(clicked[0][1]), frame.shape[1], frame.shape[0]
    finally:
        cap.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def read_positions_via_helper(python: Path, profile=None) -> list[int]:
    if not python.is_file():
        raise FileNotFoundError(f"모터 읽기 Python이 없습니다: {python}")
    command = [str(python), "-B", str(HERE / "04_omx_fk_readout.py"), "--raw-json"]
    if profile:
        command += ["--port", str(profile.port)]
    result = subprocess.run(
        command,
        text=True, capture_output=True, timeout=20, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"모터 READ 실패: {result.stderr.strip() or result.stdout.strip()}")
    try:
        raw = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError("모터 READ 응답이 JSON 형식이 아닙니다") from exc
    if (not isinstance(raw, list) or len(raw) != 5
            or any(type(value) is not int for value in raw)):
        raise RuntimeError(f"모터 READ 응답 형식이 잘못됐습니다: {raw!r}")
    print(f"현재 관절 펄스: {raw} (PING/READ만 실행)")
    return raw


def append_row(path: Path, row: dict[str, str]) -> None:
    path = path.expanduser()
    fields = ARM_CSV_FIELDS if "arm" in row else CSV_FIELDS
    if path.is_file():
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != list(fields):
                raise ValueError(f"기존 CSV 열 형식이 다릅니다: {path}")
            existing = list(reader)
        if any(r["point_name"] == row["point_name"] and r["purpose"] == row["purpose"]
               for r in existing):
            raise ValueError(f"이미 기록된 점입니다: {row['purpose']}/{row['point_name']}")
        if existing:
            first = existing[0]
            keys = ("camera", "width", "height", "plane_height_mm", "zero_pulses", "joint_signs")
            if "arm" in row:
                keys += ("arm", "serial_port")
            for key in keys:
                if first[key] != row[key]:
                    raise ValueError(f"기존 수집 조건과 {key}이(가) 다릅니다. 다른 CSV를 사용하세요")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        if handle.tell() == 0:
            writer.writeheader()
        writer.writerow(row)


def collect(args: argparse.Namespace) -> int:
    if args.raw is None:
        require_explicit_arm(args.profile)
    if not args.point_name.strip() or any(ch in args.point_name for ch in "\r\n"):
        raise ValueError("point-name을 한 줄의 이름으로 입력하세요")
    if args.profile and args.profile.name == "single" and args.plane_height_mm is None:
        existing = read_rows(args.output) if args.output.exists() else []
        if existing:
            height = existing[0]["plane_height_mm"]
            args.plane_height_mm = None if height == "unknown" else float(height)
            print(f"동일 수집의 기준물 높이 재사용: {height} mm")
        else:
            value = input("선반 작업면→십자 윗면 실제 높이 mm (미측정이면 Enter): ").strip()
            args.plane_height_mm = float(value) if value else None
    if args.plane_height_mm is not None:
        require_finite([args.plane_height_mm], "plane-height-mm")
        if args.plane_height_mm < 0:
            raise ValueError("plane-height-mm은 0 이상이어야 합니다")
    if any(s not in (-1, 1) for s in args.sign):
        raise ValueError("--sign에는 1 또는 -1만 사용하세요")
    if args.uv is None:
        u, v, width, height = click_pixel(args.camera, args.width, args.height, args.fps)
    else:
        u, v = args.uv
        width, height = args.width, args.height
    require_finite([u, v], "영상 좌표")
    if width <= 0 or height <= 0 or not (0 <= u < width and 0 <= v < height):
        raise ValueError(f"영상 좌표가 {width}x{height} 범위 밖입니다: ({u}, {v})")
    if (width, height) != (args.width, args.height):
        raise ValueError("실제 카메라 해상도가 요청한 해상도와 다릅니다")
    print(f"고정된 십자 좌표: ({u:.1f}, {v:.1f}) / {width}x{height}")
    print("블록을 움직이지 말고 그리퍼 중심을 십자 바로 위에 맞추세요.")
    print("이전 점과 같은 그리퍼 방향·높이를 유지하세요. 모터 토크 설정은 변경하지 않습니다.")
    if input("맞췄으면 READ 입력 (취소는 Enter): ").strip() != "READ":
        print("취소했습니다. 파일을 쓰지 않았습니다.")
        return 1
    fk = load_module("02_omx_xyz_offline.py", "omx_xyz_offline")
    if args.raw is None:
        raw = read_positions_via_helper(args.reader_python, args.profile)
    else:
        raw = args.raw
        print("OFFLINE: --raw 입력값만 사용하며 모터를 읽지 않았습니다.")
    q = np.asarray([(p - z) * s * 2.0 * math.pi / 4096.0
                    for p, z, s in zip(raw, args.zero, args.sign)], dtype=float)
    point, rotation = fk.forward(q)
    print(f"[미검증 FK] end_effector_link XYZ = {np.round(point * 1000, 1).tolist()} mm")
    if input("십자와 그리퍼 중심 정렬이 맞으면 SAVE 입력: ").strip() != "SAVE":
        print("취소했습니다. 파일을 쓰지 않았습니다.")
        return 1
    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "purpose": args.purpose,
        "point_name": args.point_name.strip(),
        "camera": args.camera,
        "width": str(width), "height": str(height),
        "plane_height_mm": ("unknown" if args.plane_height_mm is None
                            else str(float(args.plane_height_mm))),
        "u": f"{u:.6f}", "v": f"{v:.6f}",
        "x_mm": f"{point[0] * 1000:.6f}",
        "y_mm": f"{point[1] * 1000:.6f}",
        "z_mm": f"{point[2] * 1000:.6f}",
        "raw_pulses": json.dumps(raw),
        "zero_pulses": json.dumps(args.zero),
        "joint_signs": json.dumps(args.sign),
        "joint_radians": json.dumps(q.tolist()),
        "tool_axis_x": f"{rotation[0, 0]:.8f}",
        "tool_axis_y": f"{rotation[1, 0]:.8f}",
        "tool_axis_z": f"{rotation[2, 0]:.8f}",
    }
    if args.profile:
        row.update(args.profile.identity())
    append_row(args.output, row)
    print(f"저장: {args.output} / {args.purpose}/{args.point_name}")
    return 0


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.expanduser().open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames not in (list(CSV_FIELDS), list(ARM_CSV_FIELDS)):
            raise ValueError(f"CSV 열 형식이 다릅니다: {path}")
        return list(reader)


def project(h: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(pixels.reshape(-1, 1, 2).astype(np.float64), h).reshape(-1, 2)


def errors_for(rows: list[dict[str, str]], h: np.ndarray) -> dict[str, float] | None:
    if not rows:
        return None
    pixels = np.asarray([[float(r["u"]), float(r["v"])] for r in rows], dtype=float)
    actual = np.asarray([[float(r["x_mm"]), float(r["y_mm"])] for r in rows], dtype=float)
    errors = np.linalg.norm(project(h, pixels) - actual, axis=1)
    return {"rms_mm": float(np.sqrt(np.mean(errors * errors))),
            "max_mm": float(errors.max())}


def fit(args: argparse.Namespace) -> int:
    rows = read_rows(args.input)
    for row in rows:
        require_identity(row, args.profile)
        if args.profile:
            require_camera(row, args.profile)
    train = [r for r in rows if r["purpose"] == "train"]
    check = [r for r in rows if r["purpose"] == "check"]
    if len(train) < 4:
        raise ValueError(f"Homography 계산에는 train 점이 최소 4개 필요합니다. 현재 {len(train)}개")
    if len({r["point_name"] for r in train}) != len(train):
        raise ValueError("train 점 이름이 중복됐습니다")
    if len({r["point_name"] for r in check}) != len(check):
        raise ValueError("check 점 이름이 중복됐습니다")
    keys = ("camera", "width", "height", "plane_height_mm", "zero_pulses", "joint_signs")
    if any(tuple(r[k] for k in keys) != tuple(train[0][k] for k in keys) for r in rows):
        raise ValueError("카메라·해상도·높이·모터 변환 설정이 다른 점이 섞였습니다")
    pixels = np.asarray([[float(r["u"]), float(r["v"])] for r in train], dtype=np.float64)
    robot_xy = np.asarray([[float(r["x_mm"]), float(r["y_mm"])] for r in train], dtype=np.float64)
    z_values = np.asarray([float(r["z_mm"]) for r in train], dtype=float)
    require_finite(np.r_[pixels.ravel(), robot_xy.ravel(), z_values], "보정 데이터")
    hull = cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)
    if len(hull) < 3 or cv2.contourArea(hull) == 0:
        raise ValueError("영상 보정점이 너무 좁거나 한 줄에 몰렸습니다")
    matrix, _ = cv2.findHomography(pixels, robot_xy, 0)
    if matrix is None or not np.isfinite(matrix).all():
        raise ValueError("Homography 계산에 실패했습니다")
    directions = np.asarray([[float(r[k]) for k in ("tool_axis_x", "tool_axis_y", "tool_axis_z")]
                             for r in train], dtype=float)
    norms = np.linalg.norm(directions, axis=1)
    if (norms < 0.9).any():
        raise ValueError("도구 방향 데이터가 유효하지 않습니다")
    cosines = np.clip((directions / norms[:, None]) @ (directions[0] / norms[0]), -1, 1)
    max_direction_difference_deg = float(np.degrees(np.arccos(cosines)).max())
    train_error = errors_for(train, matrix)
    check_error = errors_for(check, matrix)
    regions = None
    if getattr(args, "layout", "single-plane") == "two-shelves":
        regions = shelf_regions(rows, int(train[0]["width"]), int(train[0]["height"]))
    data = {
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "method": "pixel_to_omx_model_xy_homography",
        "status": ("check_points_recorded" if len(check) >= (3 if args.profile else 1)
                   else "insufficient_check_points" if check else "train_only_unvalidated"),
        "camera": train[0]["camera"],
        "width": int(train[0]["width"]), "height": int(train[0]["height"]),
        "plane_height_mm": (None if train[0]["plane_height_mm"] == "unknown"
                            else float(train[0]["plane_height_mm"])),
        "robot_frame": "link0", "robot_point": "end_effector_link",
        "motor_zero_pulses_hypothesis": json.loads(train[0]["zero_pulses"]),
        "motor_joint_signs_hypothesis": json.loads(train[0]["joint_signs"]),
        "train_names": [r["point_name"] for r in train],
        "check_names": [r["point_name"] for r in check],
        "matrix_3x3": matrix.tolist(),
        "pixel_hull": hull.tolist(),
        "model_z_reference_mm": float(np.median(z_values)),
        "model_z_spread_mm": float(np.ptp(z_values)),
        "max_tool_axis_difference_deg": max_direction_difference_deg,
        "train_error": train_error, "check_error": check_error,
        "source_csv": str(args.input.expanduser().resolve()),
        "robot_motion_verified": False,
    }
    if regions is not None:
        data.update(layout="two-shelves", regions_uv=regions,
                    shelf_check_error={side: errors_for([r for r in check if r["point_name"].startswith(side+"_")], matrix)
                                       for side in ("left", "right")})
        print("좌우 선반 ROI 두 개와 선반별 확인 오차를 저장합니다:", data["shelf_check_error"])
    if args.profile:
        data.update(args.profile.identity())
    output = args.output.expanduser()
    if output.exists() and not args.overwrite:
        raise FileExistsError(f"기존 보정 파일이 있습니다: {output}; 다시 계산하려면 --overwrite")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"보정 결과: {output}")
    print(f"train {len(train)}점: RMS {train_error['rms_mm']:.1f}mm, 최대 {train_error['max_mm']:.1f}mm")
    if check_error:
        print(f"check {len(check)}점: RMS {check_error['rms_mm']:.1f}mm, 최대 {check_error['max_mm']:.1f}mm")
    else:
        print("별도 check 점 없음: 실물 호버 전 독립 위치를 수집하세요")
    print(f"말단 모델 Z 변화 폭 {data['model_z_spread_mm']:.1f}mm, 도구 방향 최대 차이 {max_direction_difference_deg:.1f}도")
    print("모터 영점·방향과 실제 호버 정확도는 아직 검증되지 않았습니다")
    return 0


def preview(args: argparse.Namespace) -> int:
    require_finite([args.hover_mm], "hover-mm")
    if args.hover_mm < 0:
        raise ValueError("hover-mm은 0 이상이어야 합니다")
    data = json.loads(args.input.expanduser().read_text(encoding="utf-8"))
    require_identity(data, args.profile)
    if args.profile:
        require_camera(data, args.profile)
    if data.get("method") != "pixel_to_omx_model_xy_homography":
        raise ValueError("OMX XY 보정 파일이 아닙니다")
    if args.camera != data["camera"]:
        raise ValueError("보정에 사용한 카메라와 다릅니다")
    if args.uv is None:
        u, v, width, height = click_pixel(args.camera, data["width"], data["height"], 30)
        if (width, height) != (data["width"], data["height"]):
            raise ValueError("현재 영상 해상도가 보정 때와 다릅니다")
    else:
        u, v = args.uv
    require_finite([u, v], "영상 좌표")
    if not (0 <= u < data["width"] and 0 <= v < data["height"]):
        raise ValueError("영상 좌표가 보정된 해상도 밖입니다")
    matrix = np.asarray(data["matrix_3x3"], dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("보정 행렬이 유효하지 않습니다")
    x_mm, y_mm = project(matrix, np.asarray([[u, v]], dtype=float))[0]
    z_mm = float(data["model_z_reference_mm"]) + args.hover_mm
    require_finite([x_mm, y_mm, z_mm], "변환된 목표 좌표")
    fk = load_module("02_omx_xyz_offline.py", "omx_xyz_offline")
    target = np.asarray([x_mm, y_mm, z_mm], dtype=float) / 1000.0
    result = fk.solve(target, args.approach == "down")
    plane_text = ("미측정" if data["plane_height_mm"] is None
                  else f"{data['plane_height_mm']}mm")
    print(f"camera={data['camera']} {data['width']}x{data['height']}, plane_height={plane_text}")
    print(f"pixel_uv=({u:.1f}, {v:.1f})")
    print(f"model_target_link0_end_effector_mm={np.round(target * 1000, 1).tolist()}")
    print(f"model_ik_residual_mm={result.position_error_m * 1000:.1f}")
    if args.approach == "down":
        print(f"model_tool_down_error_deg={result.down_error_deg:.1f}")
    print("candidate_urdf_joint_radians=" + json.dumps(np.round(result.q, 4).tolist()))
    print("MODEL_FEASIBLE" if result.feasible else "MODEL_NOT_FEASIBLE")
    print("장비 명령 없음. 관절 제한·충돌·모터 명령 변환·실제 그리퍼 방향은 미검증입니다.")
    return 0 if result.feasible else 2


def main() -> int:
    args = parse_args()
    configure_args(args, load_arm_profile(args.arm))
    if args.command == "collect":
        return collect(args)
    if args.command == "fit":
        return fit(args)
    return preview(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
