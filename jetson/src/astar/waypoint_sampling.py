import math
import statistics
from .path_geometry import clear_path

def _perp_dist(pt, a, b):
    (y0, x0), (y1, x1), (y2, x2) = pt, a, b
    num = abs((x1-x0)*(y2-y0) - (x2-x0)*(y1-y0))
    den = math.hypot(y2-y1, x2-x1) or 1.0
    return num/den

def _douglas_peucker(pts, eps):
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    idx, dmax = 0, 0.0
    for i in range(1, len(pts)-1):
        d = _perp_dist(pts[i], a, b)
        if d > dmax:
            idx, dmax = i, d
    if dmax <= eps:
        return [a, b]
    left = _douglas_peucker(pts[:idx+1], eps)
    right = _douglas_peucker(pts[idx:], eps)
    return left[:-1] + right

def sample_waypoints(path, mask):
    # 1) build raw H/V hops
    raw = [path[0]]
    y, x = path[0]
    for ty, tx in path[1:]:
        sx = tx-x, 1 if tx>x else -1
        for xx in range(x+sx, tx+sx, sx):
            if mask[y][xx]:
                raw.append((y, xx))
        x = tx
        sy = ty-y, 1 if ty>y else -1
        for yy in range(y+sy, ty+sy, sy):
            if mask[yy][x]:
                raw.append((yy, x))
        y = ty

    # 2) collapse straight runs to endpoints
    pts = [raw[0]]
    for i, (prev, curr, nxt) in enumerate(zip(raw, raw[1:], raw[2:])):
        if (curr[0] - prev[0], curr[1] - prev[1]) != (nxt[0] - curr[0], nxt[1] - curr[1]):
            if i % 1 == 0:
                pts.append(curr)
    pts.append(raw[-1])


    # 3) insert L-pivots at 90° corners
    full = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        if (a[0]==b[0] and b[1]!=c[1]) or (a[1]==b[1] and b[0]!=c[0]):
            for p in ((a[0], c[1]), (c[0], c[1])):
                if p != full[-1] and clear_path(full[-1], p, mask):
                    full.append(p)
        if b != full[-1]:
            full.append(b)
    if pts[-1] != full[-1]:
        full.append(pts[-1])

    # 4) adaptive ε based on raw-path length
    seg_dists = [math.hypot(y2-y1, x2-x1) for (y1,x1),(y2,x2) in zip(full, full[1:])]
    mean_seg = sum(seg_dists)/len(seg_dists) if seg_dists else 0.0
    raw_len = len(raw)
    eps = mean_seg * math.log(raw_len + 1)

    waypoints = _douglas_peucker(full, eps)

    if len(waypoints) >= 2:
        seg_lens = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(waypoints[:-1], waypoints[1:])
        ]

        avg = statistics.mean(seg_lens)
        std = statistics.stdev(seg_lens) if len(seg_lens) > 1 else 0.0
        threshold = avg + std

        new_wps = [waypoints[0]]
        for i in range(1, len(waypoints)):
            a, b = waypoints[i - 1], waypoints[i]
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            if dist > threshold:
                mid = ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)
                new_wps.append(mid)
            new_wps.append(b)

        waypoints = new_wps

    return waypoints