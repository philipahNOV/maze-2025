import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, obstacle_mask, angle_thresh=150, buffer=10):
    """
    Simplify path with:
      • Angle filtering to keep sharp turns.
      • Line‑of‑sight removal of redundant points.
      • Obstacle buffer via distance transform to avoid cutting too close.
    """

    # Build distance buffer map once
    free = obstacle_mask == 255
    dist_map = distance_transform_edt(free)

    def too_close(pt):
        x, y = int(pt[0]), int(pt[1])
        return dist_map[y, x] < buffer

    simplified = [path[0]]
    last = path[0]

    for i in range(1, len(path) - 1):
        curr = path[i]
        nxt = path[i + 1]

        angle = angle_between(last, curr, nxt)
        if angle < angle_thresh or too_close(curr):
            simplified.append(curr)
            last = curr
        else:
            # Only append curr if no direct LOS from last→nxt
            if not is_clear_path(obstacle_mask, last, nxt):
                simplified.append(curr)
                last = curr

    simplified.append(path[-1])
    return simplified
