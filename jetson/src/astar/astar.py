import heapq
import numpy as np
import cv2

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manhattan
    #return np.hypot(a[0] - b[0], a[1] - b[1]) # euclidean, reduces node expansions


def compute_repulsion_cost(array, min_safe_dist = 8):
    dist_transform = cv2.distanceTransform((array * 255).astype(np.uint8), cv2.DIST_L2, 3)
    mask_safe = (dist_transform >= min_safe_dist).astype(np.uint8)
    max_dist = np.max(dist_transform)
    if max_dist == 0:
        return np.ones_like(dist_transform), mask_safe
    repulsion = (1.0 - (dist_transform / max_dist)) ** 3.0
    return repulsion, mask_safe

def astar(array, start, goal, repulsion_weight=5.0):
    neighbors = [(0, 1), (1, 0), (-1, 0), (0, -1)]
    rows, cols = array.shape
    repulsion_map, walkable_mask = compute_repulsion_cost(array)

    close_set = set()
    open_set = {start}
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = [(fscore[start], start)]

    while oheap:
        _, current = heapq.heappop(oheap)
        open_set.discard(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        close_set.add(current)

        for dx, dy in neighbors:
            neighbor = current[0] + dx, current[1] + dy
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if walkable_mask[neighbor[0], neighbor[1]] == 0:
                continue

            cost = 1.0 + repulsion_weight * repulsion_map[neighbor[0], neighbor[1]]
            tentative_g = gscore[current] + cost

            if neighbor in close_set and tentative_g >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g < gscore.get(neighbor, float('inf')) or neighbor not in open_set:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g
                fscore[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                open_set.add(neighbor)

    return False

def astar_downscaled(array, start, goal, repulsion_weight=5.0, scale=0.40):
    small_array = cv2.resize(array, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    small_array = (small_array > 0).astype(np.uint8)

    # Add walkable circle for launch pad (in downscaled space)
    launch_pad_center = (int(630 * scale), int(1030 * scale))  # (y, x) scaled
    launch_pad_radius = int(70 * scale)
    cv2.circle(small_array, (launch_pad_center[1], launch_pad_center[0]), launch_pad_radius, 1, -1)

    start_small = (int(start[0] * scale), int(start[1] * scale))
    goal_small = (int(goal[0] * scale), int(goal[1] * scale))

    path_small = astar(small_array, start_small, goal_small, repulsion_weight)

    if not path_small:
        print("A* failed in downscaled space.")
        return []

    path = [(int(y / scale), int(x / scale)) for y, x in path_small]
    return path
