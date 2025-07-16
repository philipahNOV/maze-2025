import math
from .path_geometry import is_clear_path

def rdp(points, epsilon):
    """
    Simplify a path using the Ramer-Douglas-Peucker algorithm.

    Args:
        points (list): List of (y, x) points in path.
        epsilon (float): Distance threshold for simplification.

    Returns:
        list: Simplified list of (y, x) points.
    """
    if len(points) < 3:
        return points

    def perpendicular_distance(pt, line_start, line_end):
        if line_start == line_end:
            return math.hypot(pt[0] - line_start[0], pt[1] - line_start[1])
        else:
            num = abs(
                (line_end[0] - line_start[0]) * (line_start[1] - pt[1]) -
                (line_start[0] - pt[0]) * (line_end[1] - line_start[1])
            )
            den = math.hypot(line_end[0] - line_start[0], line_end[1] - line_start[1])
            return num / den

    dmax = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        d = perpendicular_distance(points[i], points[0], points[-1])
        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        first_half = rdp(points[:index+1], epsilon)
        second_half = rdp(points[index:], epsilon)
        return first_half[:-1] + second_half
    else:
        return [points[0], points[-1]]


def sample_waypoints(path, mask, waypoint_spacing=120, angle_threshold=120):
    DISTANCE_THRESHOLD = 10

    if not path or len(path) < 2:
        return path or []

    # Step 1: Simplify path using RDP
    simplified_path = rdp(path, epsilon=5)

    # Step 2: Filter waypoints by visibility (optional)
    waypoints = [simplified_path[0]]
    for i in range(1, len(simplified_path)):
        if is_clear_path(mask, waypoints[-1], simplified_path[i]):
            waypoints.append(simplified_path[i])
        else:
            # Fall back to mid-point if path is blocked
            midpoint = simplified_path[i - 1]
            if midpoint != waypoints[-1]:
                waypoints.append(midpoint)
            waypoints.append(simplified_path[i])

    # Step 3: Always append goal
    goal = path[-1]
    waypoints.append(goal)

    # Step 4: Remove redundant point before goal if too close
    if len(waypoints) >= 2:
        prev = waypoints[-2]
        dx = goal[0] - prev[0]
        dy = goal[1] - prev[1]
        dist = math.hypot(dx, dy)

        threshold = max(DISTANCE_THRESHOLD, 0.2 * waypoint_spacing)
        if dist < threshold:
            waypoints.pop(-2)

    return waypoints
