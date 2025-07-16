import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between, is_clear_path

def hybrid_simplify(path, obstacle_mask, angle_thresh=150, buffer=10):
    free = obstacle_mask == 255
    dist_map = distance_transform_edt(free)
    height, width = dist_map.shape

    def too_close(pt):
        x, y = int(pt[0]), int(pt[1])
        if not (0 <= x < width and 0 <= y < height):
            return True
        return dist_map[y, x] < buffer

    simplified = [path[0]]
    last = path[0]

    for curr, nxt in zip(path[1:-1], path[2:]):
        if angle_between(last, curr, nxt) < angle_thresh or too_close(curr):
            simplified.append(curr)
            last = curr
        elif not is_clear_path(obstacle_mask, last, nxt):
            simplified.append(curr)
            last = curr

    simplified.append(path[-1])
    return simplified
