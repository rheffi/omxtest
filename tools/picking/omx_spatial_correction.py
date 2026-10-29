"""Piecewise-linear taught XY corrections and absolute contact Z, no device I/O."""
from collections import Counter


def weights(point, a, b, c):
    x, y = point
    den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
    if abs(den) < 1e-12:
        raise ValueError("위치별 보정 삼각형의 면적이 0입니다")
    wa = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / den
    wb = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / den
    return wa, wb, 1-wa-wb


def interpolate(mesh, point):
    """Return (extra X, extra Y, absolute Z), clamping corrections at the hull."""
    nodes = mesh['points']
    for node in nodes:
        if sum((p-q)**2 for p, q in zip(point, node['xy_mm'])) < 1e-12:
            return tuple(node['correction_xyz_mm'])
    for triangle in mesh['triangles']:
        selected = [nodes[i] for i in triangle]
        w = weights(point, *(node['xy_mm'] for node in selected))
        if min(w) >= -1e-9:
            return tuple(sum(t*node['correction_xyz_mm'][axis]
                             for t, node in zip(w, selected)) for axis in range(3))
    # Outside the sampled hull: use its closest boundary correction, not an
    # unbounded extension of a triangle plane. The requested base XY is retained.
    edges = Counter(tuple(sorted((a, b))) for a, b, c in mesh['triangles']
                    for a, b in ((a, b), (b, c), (c, a)))
    nearest = None
    for (i, j), count in edges.items():
        if count != 1:
            continue
        a, b = nodes[i]['xy_mm'], nodes[j]['xy_mm']
        d = [b[k]-a[k] for k in range(2)]
        length = sum(v*v for v in d)
        t = max(0., min(1., sum((point[k]-a[k])*d[k] for k in range(2))/length))
        distance = sum((point[k]-a[k]-t*d[k])**2 for k in range(2))
        if nearest is None or distance < nearest[0]:
            result = tuple((1-t)*nodes[i]['correction_xyz_mm'][k]
                           + t*nodes[j]['correction_xyz_mm'][k] for k in range(3))
            nearest = distance, result
    if nearest is None:
        raise ValueError("위치별 보정에 유효한 삼각형이 없습니다")
    return nearest[1]
