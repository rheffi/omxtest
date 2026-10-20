#!/usr/bin/env python3
"""저장 사진과 현재 카메라 영상의 반투명 비교. 로봇 연결·보정값 변경 없음."""
import argparse
import csv
from datetime import datetime
from pathlib import Path

import cv2
from omx_arm_config import single_camera

ROOT = Path(__file__).resolve().parents[2]
WINDOW = "OMX camera overlay"


def reference_photos(output_dir):
    folders = (ROOT / "records/camera_layout", ROOT / "records/camera_layout_map", output_dir)
    return sorted({p for folder in folders for p in folder.rglob("*_webcam.jpg")}, key=lambda p: p.name)


def read_points(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return [(r["point_name"], float(r["u"]), float(r["v"]), int(r["width"]), int(r["height"]))
                for r in csv.DictReader(stream)]


def comparison_frame(live, reference, alpha, mode):
    if reference is None or mode == "live":
        return live.copy()
    # Display scaling only; never warp/align away the camera displacement.
    resized = (reference if reference.shape[:2] == live.shape[:2]
               else cv2.resize(reference, (live.shape[1], live.shape[0])))
    if mode == "reference":
        return resized.copy()
    return cv2.addWeighted(live, 1 - alpha, resized, alpha, 0)


def draw_points(view, points):
    height, width = view.shape[:2]
    for name, u, v, original_width, original_height in points:
        point = (round(u * width / original_width), round(v * height / original_height))
        cv2.drawMarker(view, point, (255, 0, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(view, name, (point[0] + 8, point[1] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 0, 255), 1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, help="겹쳐 볼 원본 사진; 생략하면 저장 사진 중 최신")
    parser.add_argument("--list", action="store_true", help="저장 사진 목록만 출력; 카메라 연결 없음")
    parser.add_argument("--camera", help="생략하면 상단 SNAP U2 자동 선택")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--samples", type=Path, default=ROOT / "records/single/omx_xy_samples.csv")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "records/camera_overlay")
    args = parser.parse_args(argv)
    photos = reference_photos(args.output_dir)
    if args.list:
        for photo in photos:
            print(photo)
        if not photos:
            print("저장된 참조 사진이 없습니다.")
        return 0
    reference_path = args.reference or (photos[-1] if photos else None)
    reference = cv2.imread(str(reference_path)) if reference_path else None
    if reference_path and reference is None:
        raise ValueError(f"참조 사진을 읽지 못했습니다: {reference_path}")
    points = read_points(args.samples)
    print(f"참조 사진: {reference_path or '없음'}")
    print(f"현재 수집점: {args.samples} ({len(points)}점)")
    print("사진 날짜와 배치가 이번 수집 때와 같은지 확인하세요. 점 좌표만으로 이전 영상을 복구하지는 못합니다.")
    print("Reference %: 사진 투명도 | Space: 현재/겹침 | R: 참조만/겹침 | P: 수집점 표시 | S: 현재 원본 저장 | Q/Esc: 종료")
    camera = args.camera or single_camera()
    capture = cv2.VideoCapture(camera, cv2.CAP_V4L2)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"카메라를 열지 못했습니다: {camera}. 다른 카메라 창을 닫으세요.")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(8):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("카메라 영상을 읽지 못했습니다")
        print(f"현재 영상: {frame.shape[1]}x{frame.shape[0]}")
        if reference is not None and reference.shape[:2] != frame.shape[:2]:
            print(f"참조 해상도 {reference.shape[1]}x{reference.shape[0]}를 현재 크기로 표시합니다. 화각이 다르면 정확한 비교가 아닙니다.")
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)
        cv2.createTrackbar("Reference %", WINDOW, 35, 100, lambda value: None)
        mode, show_points = "overlay", True
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("카메라 영상을 읽지 못했습니다")
            alpha = cv2.getTrackbarPos("Reference %", WINDOW) / 100
            view = comparison_frame(frame, reference, alpha, mode)
            if show_points:
                draw_points(view, points)
            cv2.rectangle(view, (0, 0), (view.shape[1], 67), (0, 0, 0), -1)
            ref_name = reference_path.name if reference_path else "NONE"
            cv2.putText(view, f"REF: {ref_name} | {mode} | reference={alpha:.0%}",
                        (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 1)
            cv2.putText(view, "Space: live/overlay | R: reference | P: points | S: save NOW | Q: quit",
                        (12, 53), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == 32:
                mode = "live" if mode != "live" else "overlay"
            elif key in (ord("r"), ord("R")):
                mode = "reference" if mode != "reference" else "overlay"
            elif key in (ord("p"), ord("P")):
                show_points = not show_points
            elif key in (ord("s"), ord("S")):
                args.output_dir.mkdir(parents=True, exist_ok=True)
                target = args.output_dir / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_webcam.jpg")
                if not cv2.imwrite(str(target), frame):
                    raise OSError(f"사진 저장 실패: {target}")
                print(f"현재 원본 저장: {target} (화면 참조 사진은 그대로 유지)")
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, cv2.error) as exc:
        print(f"[FAIL] {exc}")
        raise SystemExit(2)
