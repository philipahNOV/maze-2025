import math
from .path_geometry import is_clear_path, angle_between

def sample_waypoints(path, mask, waypoint_spacing=120, angle_threshold=120):
    DISTANCE_THRESHOLD = 10
    if not path or len(path) < 2:
        return path or []

    waypoints = [path[0]]
    last_wp_idx = 0
    prev_direction = None

    for i in range(1, len(path)):
        prev = path[i - 1]
        curr = path[i]
        dx = curr[1] - prev[1]
        dy = curr[0] - prev[0]
        mag = math.hypot(dx, dy)

        if mag == 0:
            continue

        # Get unit direction vector
        direction = (round(dx / mag), round(dy / mag))

        # Check direction change
        if prev_direction is not None and direction != prev_direction:
            if is_clear_path(mask, waypoints[-1], prev):
                waypoints.append(prev)
                last_wp_idx = i - 1
                prev_direction = direction
                continue

        # Check if line of sight is broken (e.g. hallway curve)
        if not is_clear_path(mask, waypoints[-1], curr):
            if path[i - 1] != waypoints[-1]:
                waypoints.append(path[i - 1])
                last_wp_idx = i - 1

        prev_direction = direction

    # Always append the goal
    goal = path[-1]
    waypoints.append(goal)

    # Remove second-to-last waypoint if too close to goal
    if len(waypoints) >= 2:
        prev = waypoints[-2]
        dx = goal[0] - prev[0]
        dy = goal[1] - prev[1]
        dist = math.hypot(dx, dy)
        threshold = max(DISTANCE_THRESHOLD, 0.2 * waypoint_spacing)
        if dist < threshold:
            waypoints.pop(-2)

    return waypoints