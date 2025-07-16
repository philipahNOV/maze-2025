import math
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, mask, waypoint_spacing=120):
    DISTANCE_THRESHOLD = 10
    if not path or len(path) < 2:
        return path or []

    waypoints = [path[0]]
    last_wp_idx = 0
    accumulated = 0.0

    for i in range(1, len(path)):
        p1 = path[i - 1]
        p2 = path[i]
        step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        accumulated += step_dist

        if accumulated >= waypoint_spacing:
            best_idx = i
            for j in range(i, last_wp_idx, -1):
                if is_clear_path(mask, path[last_wp_idx], path[j]):
                    best_idx = j
                    break

            if best_idx != last_wp_idx:
                waypoints.append(path[best_idx])
                last_wp_idx = best_idx
                i = best_idx  # reset i here so we don’t skip forward too far
                accumulated = 0.0

    goal = path[-1]
    waypoints.append(goal)

    if len(waypoints) >= 2:
        prev = waypoints[-2]
        dx = goal[0] - prev[0]
        dy = goal[1] - prev[1]
        dist = math.hypot(dx, dy)

        threshold = max(DISTANCE_THRESHOLD, 0.2 * waypoint_spacing)
        if dist < threshold:
            waypoints.pop(-2)

    return waypoints