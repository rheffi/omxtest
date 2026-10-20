"""Image regions shared by calibration, clicked hover and picking. No hardware."""
import math


def inside(point, points):
    x, y = point
    result = False
    for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1]):
        cross = (x-ax)*(by-ay)-(y-ay)*(bx-ax)
        if abs(cross) < 1e-7 and min(ax,bx) <= x <= max(ax,bx) and min(ay,by) <= y <= max(ay,by):
            return True
        if (ay > y) != (by > y) and x < (bx-ax)*(y-ay)/(by-ay)+ax:
            result = not result
    return result


def validate_regions(regions, width, height):
    if not isinstance(regions, list) or not regions:
        raise ValueError("선반별 regions_uv가 필요합니다")
    for region in regions:
        if not isinstance(region, list) or len(region) < 3:
            raise ValueError("선반 영역은 꼭짓점 3개 이상이어야 합니다")
        for point in region:
            if (not isinstance(point, (list, tuple)) or len(point) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in point)
                    or not (0 <= point[0] < width and 0 <= point[1] < height)):
                raise ValueError("선반 영역 좌표가 원본 영상 범위 밖이거나 유효하지 않습니다")
        turns = [(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0])
                 for a,b,c in zip(region, region[1:]+region[:1], region[2:]+region[:2])]
        if not (all(t > 1e-7 for t in turns) or all(t < -1e-7 for t in turns)):
            raise ValueError("선반 모서리를 겹치지 않는 볼록 다각형의 둘레 순서로 기록하세요")
    return regions


def require_region(data, uv):
    """Legacy maps without regions retain their existing hull-only behavior."""
    if data.get("regions_uv") is None:
        return
    regions = validate_regions(data["regions_uv"], data["width"], data["height"])
    if not any(inside(uv, region) for region in regions):
        raise ValueError("목표가 실제 선반 작업영역 밖입니다 (선반 사이 공간 제외)")


def shelf_regions(rows, width, height):
    points = {r["point_name"]: r for r in rows if r["purpose"] == "train"}
    needed = [f"{s}_{p}" for s in ("left", "right")
              for p in ("corner_1", "corner_2", "corner_3", "corner_4", "center")]
    missing = [n for n in needed if n not in points]
    if missing:
        raise ValueError("좌우 선반 보정점 누락: " + ", ".join(missing))
    regions = []
    for side in ("left", "right"):
        corners = [[float(points[f"{side}_corner_{i}"][key]) for key in ("u", "v")]
                   for i in (1, 2, 3, 4)]
        if any(not math.isfinite(v) for point in corners for v in point):
            raise ValueError(f"{side}: 선반 모서리 좌표가 유효하지 않습니다")
        center = [sum(point[axis] for point in corners) / 4 for axis in (0, 1)]
        # Only ROI vertices are reordered. Keep every measured UV/XY pair intact.
        corners.sort(key=lambda point: math.atan2(point[1] - center[1], point[0] - center[0]))
        try:
            validate_regions([corners], width, height)
        except ValueError as exc:
            raise ValueError(f"{side}: 선반 모서리 4점을 확인하세요: {exc}") from exc
        regions.append(corners)
    return regions
