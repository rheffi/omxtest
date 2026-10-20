"""Dependency-free detection selection, geometry and single-cycle pick planning."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "measurement"))

ARM_IDS = (11, 12, 13, 14, 15)
MODELS = (1060, 1060, 1060, 1200, 1200)


def number(value, name, low=None, high=None):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (low is not None and value < low) or (high is not None and value > high)):
        raise ValueError(f"{name}: 유효한 숫자가 필요합니다 ({low}..{high})")
    return value


def integer(value, name, low, high):
    if type(value) is not int:
        raise ValueError(f"{name}: 정수가 필요합니다")
    return number(value, name, low, high)


def vector(value, name, size, low=None, high=None):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name}: 값 {size}개가 필요합니다")
    return tuple(number(v, name, low, high) for v in value)


def pulses(value):
    values = vector(value, "joint pulses", 5, 0, 4095)
    if any(type(v) is not int for v in values):
        raise ValueError("관절 펄스는 정수여야 합니다")
    return values


def read_json(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON 객체가 필요합니다: {path}")
    return value


def project_path(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("파일 경로가 필요합니다")
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path


def load_config(path):
    cfg = read_json(path)
    if cfg.get("schema_version") != 1:
        raise ValueError("집기 설정 schema_version은 1이어야 합니다")
    number(cfg.get("confidence"), "confidence", 0, 1)
    if number(cfg.get("stage_seconds"), "stage_seconds", 0) == 0:
        raise ValueError("stage_seconds는 양수여야 합니다")
    for key in ("unfold_seconds", "gripper_seconds"):
        if key in cfg and number(cfg[key], key, 0) == 0:
            raise ValueError(f"{key}는 양수여야 합니다")
    if "stage_pause_seconds" in cfg:
        number(cfg["stage_pause_seconds"], "stage_pause_seconds", 0)
    if not isinstance(cfg.get("classes"), dict):
        raise ValueError("classes 객체가 필요합니다")
    return cfg


def polygon(value, name):
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError(f"{name}: 꼭짓점 3개 이상이 필요합니다")
    points = [vector(p, name, 2) for p in value]
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1]))
    if abs(area) < 1e-6:
        raise ValueError(f"{name}: 면적이 없는 다각형입니다")
    return points


def inside(point, points):
    """Ray casting, including the boundary; supports separate shelf polygons."""
    x, y = point
    result = False
    for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1]):
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) < 1e-7 and min(ax, bx) <= x <= max(ax, bx) and min(ay, by) <= y <= max(ay, by):
            return True
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            result = not result
    return result


def to_xy(calibration, uv):
    u, v = vector(uv, "UV", 2)
    width = integer(calibration.get("width"), "width", 1, 16384)
    height = integer(calibration.get("height"), "height", 1, 16384)
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError("목표 UV가 원본 영상 범위 밖입니다")
    matrix = calibration.get("matrix_3x3")
    if not isinstance(matrix, list) or len(matrix) != 3:
        raise ValueError("3x3 보정 행렬이 필요합니다")
    rows = [vector(row, "matrix_3x3", 3) for row in matrix]
    w = rows[2][0] * u + rows[2][1] * v + rows[2][2]
    if abs(w) < 1e-9:
        raise ValueError("좌표 변환 분모가 0입니다")
    return vector([(r[0] * u + r[1] * v + r[2]) / w for r in rows[:2]], "XY", 2)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple
    class_id: int = 0

    def point(self, rule):
        fx, fy = vector(rule.get("grasp_fraction", [0.5, 0.5]), "grasp_fraction", 2, 0, 1)
        x1, y1, x2, y2 = self.box
        return (x1 + (x2 - x1) * fx, y1 + (y2 - y1) * fy)

    def validate(self, width, height):
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("검출 클래스 이름이 필요합니다")
        number(self.confidence, "confidence", 0, 1)
        integer(self.class_id, "class_id", 0, 100000)
        x1, y1, x2, y2 = vector(self.box, "bbox xyxy", 4)
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("bbox가 원본 영상 범위 밖이거나 크기가 0입니다")


def decode_detections(data):
    width = integer(data.get("width"), "width", 1, 16384)
    height = integer(data.get("height"), "height", 1, 16384)
    items = data.get("detections")
    if not isinstance(items, list):
        raise ValueError("detections 배열이 필요합니다")
    detections = []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError("각 검출은 객체여야 합니다")
        det = Detection(item.get("class_name"), item.get("confidence"),
                        tuple(vector(item.get("xyxy"), "xyxy", 4)), item.get("class_id", 0))
        det.validate(width, height)
        detections.append(det)
    return width, height, detections


def get_rule(cfg, label):
    rule = cfg["classes"].get(label)
    if not isinstance(rule, dict):
        raise ValueError(f"classes에 모델의 정확한 클래스 이름을 등록하세요: {label}")
    return rule


def region_for(cfg, uv, width, height):
    regions = cfg.get("regions_uv")
    if not isinstance(regions, list) or not regions:
        raise ValueError("regions_uv에 실제 선반별 다각형을 설정하세요")
    matched = False
    for region in regions:
        points = polygon(region, "regions_uv")
        if any(not (0 <= u < width and 0 <= v < height) for u, v in points):
            raise ValueError("regions_uv가 원본 영상 해상도 밖입니다")
        matched |= inside(uv, points)
    if not matched:
        raise ValueError("목표가 실제 선반 작업영역 밖입니다")


def choose(detections, cfg, label, width, height, index=0):
    rule = get_rule(cfg, label)
    candidates = []
    for det in detections:
        det.validate(width, height)
        if det.label != label or det.confidence < cfg["confidence"]:
            continue
        try:
            region_for(cfg, det.point(rule), width, height)
        except ValueError as exc:
            # Malformed regions must still fail when no candidate can be selected.
            if "작업영역 밖" not in str(exc):
                raise
            continue
        candidates.append(det)
    candidates.sort(key=lambda d: (-d.confidence, d.box))
    if not candidates:
        raise ValueError(f"허용된 선반 안에 {label!r} 검출이 없습니다")
    if type(index) is not int or not 0 <= index < len(candidates):
        raise ValueError(f"target-index는 0..{len(candidates) - 1}이어야 합니다")
    return candidates[index]


@dataclass(frozen=True)
class Stage:
    name: str
    kind: str
    target: tuple | int | dict | None


def validate_gripper(cfg):
    grip = cfg.get("gripper")
    if not isinstance(grip, dict):
        raise ValueError("gripper 설정이 필요합니다")
    for key in ("open_pulse", "close_pulse"):
        integer(grip.get(key), f"gripper.{key}", 0, 4095)
    if grip["open_pulse"] == grip["close_pulse"]:
        raise ValueError("그리퍼 열림·닫힘 펄스가 같습니다")
    integer(grip.get("current_limit_raw"), "gripper.current_limit_raw", 1, 1750)
    number(grip.get("settle_s"), "gripper.settle_s", 0)
    return grip


def build_plan(cfg, calibration, detection, saved, solver, *, hover_only=False, from_folded=False):
    """solver(XYZ mm) returns five joint pulses. No motor or file operations."""
    detection.validate(calibration["width"], calibration["height"])
    rule = get_rule(cfg, detection.label)
    uv = detection.point(rule)
    region_for(cfg, uv, calibration["width"], calibration["height"])
    xy = to_xy(calibration, uv)
    offset = vector(rule.get("xy_offset_mm"), "xy_offset_mm", 2)
    pick = (xy[0] + offset[0], xy[1] + offset[1], number(rule.get("pick_z_mm"), "pick_z_mm"))
    clearance = number(rule.get("hover_clearance_mm"), "hover_clearance_mm", 0)
    extra = number(rule.get("approach_extra_mm"), "approach_extra_mm", 0)

    def solve(z):
        xyz = (*pick[:2], z)
        return pulses(solver(xyz))

    hover = solve(pick[2] + clearance)
    approach = solve(pick[2] + clearance + extra)
    high_pose = pulses(saved["hover_high"])
    stages = [Stage("HIGH", "arm", high_pose), Stage("APPROACH", "arm", approach),
              Stage("HOVER", "arm", hover)]
    if hover_only:
        stages += [Stage("RETREAT", "arm", approach), Stage("RETURN_HIGH", "arm", high_pose)]
    else:
        grip = validate_gripper(cfg)
        contact = solve(pick[2])
        drop_approach = pulses(saved["drop_approach"])
        drop_release = pulses(saved["drop_release"])
        stages.insert(1, Stage("OPEN", "gripper", grip["open_pulse"]))
        stages += [Stage("PICK", "arm", contact), Stage("CLOSE", "gripper", grip["close_pulse"]),
                   Stage("LIFT", "arm", hover),
                   Stage("RETREAT", "arm", approach), Stage("CARRY_HIGH", "arm", high_pose),
                   Stage("DROP_APPROACH", "arm", drop_approach), Stage("DROP_RELEASE", "arm", drop_release),
                   Stage("RELEASE", "gripper", grip["open_pulse"]),
                   Stage("DROP_RETREAT", "arm", drop_approach), Stage("RETURN_HIGH", "arm", high_pose)]
    stages = unfold_stages(cfg, from_folded) + stages
    return {"simulation": False, "class_name": detection.label, "confidence": detection.confidence,
            "box_xyxy": detection.box, "uv": uv, "pick_model_xyz_mm": pick,
            "hover_only": hover_only, "stages": [asdict(stage) for stage in stages]}


def unfold_stages(cfg, enabled):
    if not enabled:
        return []
    unfold = cfg.get("unfold") or {}
    elbow = integer(unfold.get("elbow_pulse"), "unfold.elbow_pulse", 0, 4095)
    shoulder = integer(unfold.get("shoulder_pulse"), "unfold.shoulder_pulse", 0, 4095)
    return [Stage("ELBOW_FIRST", "joint", {13: elbow}), Stage("SHOULDER_SECOND", "joint", {12: shoulder})]


def build_drop_plan(cfg, saved, from_folded=False):
    stages = unfold_stages(cfg, from_folded)
    for label, pose in (("HIGH", "hover_high"), ("DROP_APPROACH", "drop_approach"),
                        ("DROP_RELEASE", "drop_release"), ("DROP_RETREAT", "drop_approach"),
                        ("RETURN_HIGH", "hover_high")):
        stages.append(Stage(label, "arm", pulses(saved[pose])))
    return {"simulation": False, "mode": "FIXED_DROP_PATH_NO_GRIPPER", "stages": [asdict(s) for s in stages]}


def execute(plan, driver, emit=lambda *args: None):
    """Send stages without intermediate input; physical success is unconfirmed."""
    if plan.get("simulation") and not getattr(driver, "simulation", False):
        raise ValueError("모의 계획은 실제 모터로 실행할 수 없습니다")
    try:
        for stage in plan["stages"]:
            emit("stage_start", stage)
            if stage["kind"] == "arm":
                driver.arm(stage["name"], stage["target"])
            elif stage["kind"] == "joint":
                driver.joint(stage["name"], stage["target"])
            elif stage["kind"] == "gripper":
                driver.gripper(stage["name"], stage["target"])
            else:
                raise ValueError(f"알 수 없는 stage kind: {stage['kind']}")
            emit("stage_done", stage)
    except BaseException:
        driver.hold()
        raise


class MockDriver:
    simulation = True

    def __init__(self):
        self.events = []

    def arm(self, name, target):
        self.events.append((name, list(pulses(target))))

    def gripper(self, name, target):
        self.events.append((name, integer(target, "gripper", 0, 4095)))

    def hold(self):
        self.events.append(("HOLD", None))


def demo(hover_only=False):
    cfg = {"confidence": 0.6, "regions_uv": [[[0, 0], [999, 0], [999, 799], [0, 799]]],
           "classes": {"demo_object": {"grasp_fraction": [0.5, 0.5], "xy_offset_mm": [0, 0],
                       "pick_z_mm": 30, "hover_clearance_mm": 50, "approach_extra_mm": 40}},
           "gripper": {"open_pulse": 1800, "close_pulse": 2200, "current_limit_raw": 50, "settle_s": 1}}
    cal = {"width": 1000, "height": 800, "pixel_hull": cfg["regions_uv"][0],
           "matrix_3x3": [[0.1, 0, 140], [0, 0.1, -40], [0, 0, 1]]}
    det = choose([Detection("demo_object", 0.93, (400, 300, 600, 500))], cfg, "demo_object", 1000, 800)
    # Symbolic fake pulses: deliberately never sent to hardware and not an IK test.
    plan = build_plan(cfg, cal, det, {name: [2048] * 5 for name in
                      ("hover_high", "drop_approach", "drop_release")}, lambda xyz: [2048] * 5,
                      hover_only=hover_only)
    plan["simulation"] = True
    driver = MockDriver()
    execute(plan, driver)
    return {"mode": "DEMO_NO_MODEL_NO_HARDWARE", "plan": plan, "events": driver.events,
            "note": "가상 인식·가상 펄스. YOLO/IK/실물 성능 검증 결과가 아닙니다."}
