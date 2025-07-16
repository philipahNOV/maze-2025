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

    for i in range(1, len(path) - 1):
        p1 = path[i - 1]
        p2 = path[i]
        step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        accumulated += step_dist

        dist_from_last = math.hypot(
            path[i][0] - path[last_wp_idx][0],
            path[i][1] - path[last_wp_idx][1]
        )

        # Distance-based insertion
        if dist_from_last >= spacing:
            best_idx = last_wp_idx + 1
            for j in range(i, last_wp_idx, -1):
                if is_clear_path(mask, path[last_wp_idx], path[j]):
                    best_idx = j
                    break

            dist_to_best = math.hypot(
                path[best_idx][0] - path[last_wp_idx][0],
                path[best_idx][1] - path[last_wp_idx][1]
            )

            if dist_to_best >= spacing:
                waypoints.append(path[best_idx])
                last_wp_idx = best_idx
                accumulated = 0.0
                continue

        # Angle-based insertion (with spacing and lookahead)
        angle = angle_between(path[last_wp_idx], path[i], path[i + 1])
        if angle < angle_threshold and dist_from_last >= spacing:
            if is_clear_path(mask, path[last_wp_idx], path[i]):
                if i + 2 < len(path):
                    angle_next = angle_between(path[i], path[i + 1], path[i + 2])
                    if angle_next < angle_threshold:
                        waypoints.append(path[i])
                        last_wp_idx = i
                        accumulated = 0.0
                        continue
                else:
                    waypoints.append(path[i])
                    last_wp_idx = i
                    accumulated = 0.0
                    continue

    # Always end at the goal
    goal = path[-1]
    waypoints.append(goal)

    # Remove second-to-last if too close to goal
    if len(waypoints) >= 2:
        prev = waypoints[-2]
        dx = goal[0] - prev[0]
        dy = goal[1] - prev[1]
        dist = math.hypot(dx, dy)

        threshold = max(DISTANCE_THRESHOLD, 0.2 * waypoint_spacing)
        if dist < threshold:
            waypoints.pop(-2)

    # In-place cleanup: remove redundant middle points if they are line-of-sight connected
    cleaned = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        if not is_clear_path(mask, cleaned[-1], waypoints[i + 1]):
            cleaned.append(waypoints[i])
    cleaned.append(waypoints[-1])

    # Overwrite waypoints with cleaned in-place
    waypoints[:] = cleaned

    return waypoints