import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, obstacle_mask, angle_thresh=150, buffer=10, max_lookahead=100, step=5):
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
            if not is_clear_path(obstacle_mask, path[i], path[j]):
                break

            # Sample only a few intermediate points
            check_idxs = range(i + 1, j, max((j - i) // 5, 1))
            valid = True
            for k in check_idxs:
                pt = path[k]
                if too_close(pt) or angle_between(path[i], pt, path[j]) < angle_thresh:
                    valid = False
                    break
            if valid:
                best = j

        simplified.append(path[best])
        i = best

    return simplified


