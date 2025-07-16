import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, obstacle_mask, angle_thresh=150, buffer=10, max_lookahead=100, step=10):
    from scipy.ndimage import distance_transform_edt
    free = obstacle_mask == 255
    dist_map = distance_transform_edt(free)
    height, width = dist_map.shape

    def too_close(pt):
        x, y = int(pt[0]), int(pt[1])
        return not (0 <= x < width and 0 <= y < height) or dist_map[y, x] < buffer

    simplified = [path[0]]
    i = 0
    N = len(path)

    while i < N - 1:
        print(f"[PathFindingThread] Sampling waypoints: {i}/{N-1}")
        best = i + 1
        max_j = min(i + max_lookahead, N - 1)

        for j in range(i + step, max_j + 1, step):
            # Precheck: geometry + clearance only on 2 samples
            mid_idxs = [i + (j - i) // 3, i + 2 * (j - i) // 3]
            if any(
                too_close(path[k]) or angle_between(path[i], path[k], path[j]) < angle_thresh
                for k in mid_idxs
            ):
                continue

            # Do LOS check last (expensive)
            if is_clear_path(obstacle_mask, path[i], path[j]):
                best = j

        simplified.append(path[best])
        i = best

    return simplified


