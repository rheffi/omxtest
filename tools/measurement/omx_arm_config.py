"""Single-notebook device defaults; old named profiles remain readable."""

from __future__ import annotations

import json
import csv
import argparse
import re
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROFILES_FILE = ROOT / "config" / "omx_arm_profiles.json"
SERIAL_DIR = Path("/dev/serial/by-id")
TTY_DIR = Path("/sys/class/tty")
DEV_DIR = Path("/dev")
CAMERA_DIR = Path("/dev/v4l/by-id")
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-HFSD-231214-J_SNAP_U2-video-index0"


def single_camera():
    cameras = sorted(p for p in CAMERA_DIR.glob("*SNAP_U2*video-index0") if p.exists())
    if len(cameras) > 1:
        raise ValueError("상단 SNAP U2가 여러 대입니다. 노트북 한 대에 상단 카메라 한 대를 연결하세요")
    return str(cameras[0]) if cameras else DEFAULT_CAMERA


def connected_serial_ports():
    """Find actual OpenRB nodes, preferring persistent udev names."""
    ports = sorted(p for p in SERIAL_DIR.glob("usb-ROBOTIS_OpenRB-150_*") if p.exists())
    identified = {p.resolve() for p in ports}
    for tty in sorted([*TTY_DIR.glob("ttyACM*"), *TTY_DIR.glob("ttyUSB*")]):
        node = DEV_DIR / tty.name
        if not node.exists() or node.resolve() in identified:
            continue
        for parent in (tty / "device").resolve().parents:
            try:
                product = (parent / "product").read_text().strip()
            except OSError:
                continue
            if "openrb" in product.lower():
                ports.append(node)
                break
    return sorted(ports)


def single_profile():
    ports = connected_serial_ports()
    if len(ports) > 1:
        raise ValueError("OMX가 여러 대 연결됐습니다. 노트북 한 대에 OMX 한 대 구성인지 확인하세요")
    calibration = ROOT / "config/single/omx_xy_calibration.json"
    poses = ROOT / "config/single/omx_saved_poses.json"
    samples = ROOT / "records/single/omx_xy_samples.csv"
    saved_port = None
    for file in (calibration, poses):
        if file.exists():
            saved_port = json.loads(file.read_text()).get("serial_port")
            if saved_port:
                break
    if not saved_port and samples.exists():
        with samples.open(newline="") as f:
            saved_port = next(csv.DictReader(f), {}).get("serial_port")
    port = ports[0] if ports else Path(saved_port or "/dev/omx_follower")
    return ArmProfile("single", port, single_camera(), 1920, 1080, 30, samples, calibration, poses)


@dataclass(frozen=True)
class ArmProfile:
    name: str
    port: Path
    camera: str
    width: int
    height: int
    fps: int
    samples: Path
    calibration: Path
    poses: Path

    def identity(self) -> dict[str, str]:
        return {"arm": self.name, "serial_port": str(self.port)}

    def describe(self) -> None:
        label = "[OMX]" if self.name == "single" else f"[ARM] {self.name};"
        print(f"{label} port={self.port}", flush=True)


def add_arm_argument(parser) -> None:
    parser.add_argument("--arm", metavar="NAME",
                        help=argparse.SUPPRESS)


def load_arm_profile(name: str | None) -> ArmProfile | None:
    if name is None or name == "single":
        return single_profile()
    data = json.loads(PROFILES_FILE.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("팔 프로필 설정은 JSON 객체여야 합니다")
    arms = data.get("arms")
    camera = data.get("camera")
    if (data.get("schema_version") != 1 or not isinstance(arms, dict)
            or not isinstance(camera, dict) or not arms):
        raise ValueError("팔 프로필 설정 형식이 잘못됐습니다")
    ports = []
    for arm_name, arm in arms.items():
        if not re.fullmatch(r"[a-z][a-z0-9_]{0,31}", arm_name):
            raise ValueError("팔 이름은 영문 소문자·숫자·_만 사용하세요")
        value = arm.get("port") if isinstance(arm, dict) else None
        if not isinstance(value, str):
            raise ValueError(f"{arm_name}: 직렬 포트 설정이 없습니다")
        port = Path(value)
        if (str(port.parent) != "/dev/serial/by-id"
                or not port.name.startswith("usb-ROBOTIS_OpenRB-150_")):
            raise ValueError(f"{arm_name}: OpenRB의 고유 by-id 경로를 지정하세요")
        ports.append(port)
    if len(set(ports)) != len(ports):
        raise ValueError("여러 팔에 같은 직렬 포트가 지정됐습니다")
    if name not in arms:
        raise ValueError(f"알 수 없는 팔: {name}; 선택 가능: {', '.join(arms)}")
    device = camera.get("device")
    if not isinstance(device, str) or not device.startswith("/dev/v4l/by-id/"):
        raise ValueError("카메라의 고유 by-id 경로를 지정하세요")
    if any(type(camera.get(key)) is not int or camera[key] <= 0
           for key in ("width", "height", "fps")):
        raise ValueError("카메라 폭·높이·fps는 양의 정수여야 합니다")
    config_dir = ROOT / "config" / "arms" / name
    return ArmProfile(name, Path(arms[name]["port"]), device,
                      camera["width"], camera["height"], camera["fps"],
                      ROOT / "records" / "arms" / name / "omx_xy_samples.csv",
                      config_dir / "omx_xy_calibration.json",
                      config_dir / "omx_saved_poses.json")


def require_identity(data: dict, profile: ArmProfile | None) -> None:
    if profile is not None:
        if any(data.get(key) != value for key, value in profile.identity().items()):
            raise ValueError(f"기록의 장비 식별 정보가 현재 장비와 다릅니다: {profile.port}")
    elif "arm" in data or "serial_port" in data:
        raise ValueError("팔별 데이터에는 --arm을 지정하세요")


def require_camera(data: dict, profile: ArmProfile) -> None:
    if (data.get("camera"), int(data.get("width", 0)), int(data.get("height", 0))) != (
            profile.camera, profile.width, profile.height):
        raise ValueError(f"{profile.name}: 카메라/해상도가 현재 프로필과 다릅니다. 다시 보정하세요")


def require_explicit_arm(profile: ArmProfile | None) -> None:
    if profile is None and len(connected_serial_ports()) > 1:
        raise ValueError("OpenRB가 여러 대 연결됐습니다. --arm으로 시험할 팔을 지정하세요")
