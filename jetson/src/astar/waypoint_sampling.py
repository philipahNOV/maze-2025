import math
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, mask, waypoint_spacing=120, angle_threshold=120):
    DISTANCE_THRESHOLD = 10
    if not path or len(path) < 2:
        return path or []

    total_length = sum(
        math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        for p1, p2 in zip(path[:-1], path[1:])
    )
    target_count = max(2, int(total_length / waypoint_spacing))
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
            if angle < angle_threshold and is_clear_path(mask, path[last_wp_idx], path[i]):
                # Only insert waypoint if next segment is also NOT straight
                angle_next = angle_between(path[i], path[i+1], path[i+2]) if i+2 < len(path) else 180
                if angle_next < angle_threshold:
                    waypoints.append(path[i])
                    last_wp_idx = i
                    accumulated = 0.0
                    continue


        if accumulated >= spacing:
            best_idx = last_wp_idx + 1
            for j in range(i, last_wp_idx, -1):
                if is_clear_path(mask, path[last_wp_idx], path[j]):
                    best_idx = j
                    break

            # Prefer longer segments, ignore if too short
            dist = math.hypot(
                path[best_idx][0] - path[last_wp_idx][0],
                path[best_idx][1] - path[last_wp_idx][1]
            )
            if dist >= spacing * 0.8:
                waypoints.append(path[best_idx])
                last_wp_idx = best_idx
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