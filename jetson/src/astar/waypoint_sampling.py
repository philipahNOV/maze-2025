import math
from .path_geometry import angle_between, is_clear_path

def sample_waypoints(path, mask, waypoint_spacing=120):
    DISTANCE_THRESHOLD = 10
    if not path or len(path) < 2:
        return path or []

    waypoints = [path[0]]
    current_idx = 0
    n = len(path)

    while current_idx < n - 1:
        # Try to go directly to the goal
        for i in range(n - 1, current_idx, -1):
            if is_clear_path(mask, path[current_idx], path[i]):
                current_idx = i
                waypoints.append(path[i])
                break
        else:
            # No straight path — we must go around the obstacle
            # Try stepping in axis-aligned directions
            curr = path[current_idx]
            directions = [(1,0), (-1,0), (0,1), (0,-1)]  # Down, Up, Right, Left

            for dx, dy in directions:
                for step in range(1, 20):  # Try stepping out 20 pixels
                    next_pt = (curr[0] + dx * step, curr[1] + dy * step)
                    if 0 <= next_pt[0] < mask.shape[0] and 0 <= next_pt[1] < mask.shape[1]:
                        if is_clear_path(mask, curr, next_pt):
                            waypoints.append(next_pt)
                            path.insert(current_idx + 1, next_pt)  # Insert into path for future loops
                            break
                else:
                    continue
                break
            else:
                # Dead end
                break

    return waypoints