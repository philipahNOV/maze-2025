import math
from .path_geometry import is_clear_path

def sample_waypoints(path, mask, waypoint_spacing=120):
    DISTANCE_THRESHOLD = 10

    if not path or len(path) < 2:
        return path or []

    waypoints = [path[0]]
    prev_direction = None

    for i in range(1, len(path)):
        dy = path[i][0] - path[i - 1][0]
        dx = path[i][1] - path[i - 1][1]

        mag = math.hypot(dx, dy)
        if mag == 0:
            continue
        direction = (round(dx / mag), round(dy / mag))

        if direction != prev_direction:
            if is_clear_path(mask, waypoints[-1], path[i - 1]):
                waypoints.append(path[i - 1])
            prev_direction = direction

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