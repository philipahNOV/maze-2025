import math
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, mask, target_count=20, angle_threshold=135):
    if not path or len(path) < 2:
        return path or []

    total_length = sum(
        math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        for p1, p2 in zip(path[:-1], path[1:])
    )
    spacing = total_length / target_count

    waypoints = [path[0]]
    last_wp_idx = 0
    accumulated = 0.0

    for i in range(1, len(path)):
        p1 = path[i - 1]
        p2 = path[i]
        step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        accumulated += step_dist

        if i < len(path) - 1:
            angle = angle_between(path[last_wp_idx], path[i], path[i + 1])
            if angle < angle_threshold and accumulated >= spacing / 2:
                if is_clear_path(mask, path[last_wp_idx], path[i]):
                    waypoints.append(path[i])
                    last_wp_idx = i
                    accumulated = 0.0
                    continue

        if accumulated >= spacing:
            best_idx = last_wp_idx + 1
            for j in range(i, last_wp_idx, -1):
                if is_clear_path(mask, path[last_wp_idx], path[j]):
                    best_idx = j
            waypoints.append(path[best_idx])
            last_wp_idx = best_idx
            accumulated = 0.0

    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints