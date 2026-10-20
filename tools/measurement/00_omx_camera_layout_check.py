#!/usr/bin/env python3
"""Preview the external overhead camera without connecting to the robot.

No file is written unless --save-dir is supplied and S is pressed.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from omx_arm_config import single_camera

import cv2
import numpy as np


CAMERA = "/dev/v4l/by-id/usb-HFSD-231214-J_SNAP_U2-video-index0"
TILE_WIDTH = 640
TILE_HEIGHT = 480


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="외장 웹캠의 배치와 영상 상태를 확인합니다. 로봇에는 연결하지 않습니다."
    )
    parser.add_argument("--camera", default=None, help="외장 웹캠 장치 경로")
    parser.add_argument(
        "--role", choices=("unknown", "overhead", "arm"), default="overhead"
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="0이면 Q/Esc까지 실행; 양수면 지정한 초 후 종료",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        help="S를 눌렀을 때만 원본 사진과 장치 정보를 저장할 디렉터리",
    )
    return parser.parse_args()


def fourcc_text(value: float) -> str:
    code = int(value)
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)).strip()


@dataclass
class Camera:
    label: str
    path: Path
    role: str
    capture: cv2.VideoCapture | None = None
    frame: np.ndarray | None = None
    reads: int = 0
    failures: int = 0
    timestamps: deque[float] = field(default_factory=lambda: deque(maxlen=120))
    actual_width: int = 0
    actual_height: int = 0
    actual_fps: float = 0.0
    actual_fourcc: str = ""

    def open(self, width: int, height: int, fps: int) -> None:
        if not self.path.exists():
            raise FileNotFoundError(f"{self.label}: 카메라 장치가 없습니다: {self.path}")
        self.capture = cv2.VideoCapture(str(self.path), cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"{self.label}: 카메라를 열지 못했습니다: {self.path}")
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = float(self.capture.get(cv2.CAP_PROP_FPS))
        self.actual_fourcc = fourcc_text(self.capture.get(cv2.CAP_PROP_FOURCC))
        print(
            f"{self.label} [{self.role}] {self.path} -> {self.path.resolve()} | "
            f"reported={self.actual_width}x{self.actual_height} "
            f"{self.actual_fourcc} {self.actual_fps:.1f} fps"
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.read():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"{self.label}: 5초 동안 프레임을 읽지 못했습니다: {self.path}"
            )

    def read(self) -> bool:
        assert self.capture is not None
        ok, frame = self.capture.read()
        self.reads += 1
        if not ok or frame is None:
            self.failures += 1
            return False
        self.frame = frame
        self.timestamps.append(time.monotonic())
        return True

    @property
    def measured_fps(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / elapsed if elapsed > 0 else 0.0

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()


def tile(camera: Camera) -> np.ndarray:
    if camera.frame is None:
        view = np.zeros((TILE_HEIGHT, TILE_WIDTH, 3), dtype=np.uint8)
    else:
        view = cv2.resize(
            camera.frame, (TILE_WIDTH, TILE_HEIGHT), interpolation=cv2.INTER_AREA
        )
    for part in (1, 2, 3):
        x = TILE_WIDTH * part // 4
        y = TILE_HEIGHT * part // 4
        cv2.line(view, (x, 0), (x, TILE_HEIGHT), (120, 220, 120), 1)
        cv2.line(view, (0, y), (TILE_WIDTH, y), (120, 220, 120), 1)
    cv2.drawMarker(
        view, (TILE_WIDTH // 2, TILE_HEIGHT // 2),
        (0, 0, 255), cv2.MARKER_CROSS, 24, 2
    )
    header = np.zeros((68, TILE_WIDTH, 3), dtype=np.uint8)
    name = camera.path.name.replace("-video-index0", "")
    cv2.putText(
        header, f"{camera.label}: {name[:50]} [{camera.role}]",
        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
    )
    size = (
        f"{camera.frame.shape[1]}x{camera.frame.shape[0]}"
        if camera.frame is not None else "no frame"
    )
    cv2.putText(
        header,
        f"{size} | measured {camera.measured_fps:.1f} fps | failed {camera.failures}/{camera.reads}",
        (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
    )
    return np.vstack((header, view))


def save_snapshot(camera: Camera, directory: Path) -> None:
    if camera.frame is None:
        print("저장 실패: 카메라 프레임이 필요합니다.")
        return
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{stamp}_webcam.jpg"
    target = directory / filename
    if not cv2.imwrite(str(target), camera.frame):
        raise OSError(f"사진을 저장하지 못했습니다: {target}")
    info = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "camera": {
            "label": camera.label,
            "role": camera.role,
            "device": str(camera.path),
            "resolved_device": str(camera.path.resolve()),
            "image": filename,
            "width": int(camera.frame.shape[1]),
            "height": int(camera.frame.shape[0]),
            "reported_fps": camera.actual_fps,
            "measured_fps": round(camera.measured_fps, 2),
            "read_failures": camera.failures,
            "read_attempts": camera.reads,
        },
    }
    manifest = directory / f"{stamp}_devices.json"
    manifest.write_text(json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"원본 사진과 장치 정보 저장: {directory / stamp}_*")


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0 or args.duration_sec < 0:
        raise SystemExit("width, height, fps는 양수이고 duration-sec는 0 이상이어야 합니다.")
    camera = Camera("WEB", Path(args.camera or single_camera()), args.role)
    print("Q/Esc: 종료 | S: --save-dir를 지정한 경우에만 원본 사진 저장")
    print("확인: 좌우 선반 집기 영역, 팔/그리퍼 가림, 상품 크기와 초점, 조명·그림자·흔들림. 고정 바구니는 화면 밖이어도 됩니다.")
    start = time.monotonic()
    try:
        camera.open(args.width, args.height, args.fps)
        cv2.namedWindow("OMX camera layout check", cv2.WINDOW_NORMAL)
        while True:
            camera.read()
            cv2.imshow("OMX camera layout check", tile(camera))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):
                if args.save_dir is None:
                    print("저장하지 않았습니다: --save-dir를 지정하세요.")
                else:
                    save_snapshot(camera, args.save_dir)
            if args.duration_sec and time.monotonic() - start >= args.duration_sec:
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print(
            f"{camera.label}: reads={camera.reads}, failures={camera.failures}, "
            f"measured_fps={camera.measured_fps:.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
