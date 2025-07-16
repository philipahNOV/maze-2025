import numpy as np
from scipy.ndimage import distance_transform_edt
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, obstacle_mask, angle_thresh=150, buffer=10):
    free = obstacle_mask == 255
    dist_map = distance_transform_edt(free)
    height, width = dist_map.shape

    def too_close(pt):
        x, y = int(pt[0]), int(pt[1])
        if not (0 <= x < width and 0 <= y < height):
            return True
        return dist_map[y, x] < buffer

    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        # Try to find the furthest j we can skip to
        furthest = i + 1
        for j in range(i + 2, len(path)):
            if is_clear_path(obstacle_mask, path[i], path[j]):
                # Check all intermediate points for angle + wall proximity
                valid = True
                for k in range(i + 1, j):
                    if angle_between(path[i], path[k], path[j]) < angle_thresh or too_close(path[k]):
                        valid = False
                        break
                if valid:
                    furthest = j
            else:
                break  # no LOS anymore
        simplified.append(path[furthest])
        i = furthest

    return simplified

