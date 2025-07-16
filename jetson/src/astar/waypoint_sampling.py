import math
from .path_geometry import is_clear_path

def rdp(points, epsilon):
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

def interpolate_points(p1, p2, spacing):
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    if dist <= spacing:
        return []
    
    num_points = int(dist // spacing)
    dx = (p2[0] - p1[0]) / (num_points + 1)
    dy = (p2[1] - p1[1]) / (num_points + 1)

    return [(int(p1[0] + dx * i), int(p1[1] + dy * i)) for i in range(1, num_points + 1)]

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

    # Step 1: Simplify path
    simplified_path = rdp(path, epsilon=10)

    # Step 2: Build BRIO-style waypoints from RDP path
    waypoints = [simplified_path[0]]
    for i in range(1, len(simplified_path)):
        start = waypoints[-1]
        end = simplified_path[i]

        # Interpolate midpoints if long and straight
        mids = interpolate_points(start, end, spacing)
        for pt in mids:
            if is_clear_path(mask, waypoints[-1], pt):
                waypoints.append(pt)

        # Always try to add end point of segment
        if is_clear_path(mask, waypoints[-1], end):
            waypoints.append(end)

    # Step 3: Append the actual goal
    goal = path[-1]
    waypoints.append(goal)

    # Step 4: Remove second-to-last point if it's too close to goal
    if len(waypoints) >= 2:
        prev = waypoints[-2]
        dx = goal[0] - prev[0]
        dy = goal[1] - prev[1]
        dist = math.hypot(dx, dy)

        threshold = max(DISTANCE_THRESHOLD, 0.2 * waypoint_spacing)
        if dist < threshold:
            waypoints.pop(-2)

    return waypoints
