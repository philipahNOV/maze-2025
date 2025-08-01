import heapq
import numpy as np
import cv2

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) # manhattan
    #return np.hypot(a[0] - b[0], a[1] - b[1]) # euclidean, reduces node expansions

def compute_repulsion_cost(array, min_safe_dist = 14):
    dist_transform = cv2.distanceTransform((array * 255).astype(np.uint8), cv2.DIST_L2, 3)
    mask_safe = (dist_transform >= min_safe_dist).astype(np.uint8)
    max_dist = np.max(dist_transform)
    if max_dist == 0:
        return np.ones_like(dist_transform), mask_safe
    repulsion = (1.0 - (dist_transform / max_dist)) ** 3.0
    return repulsion, mask_safe

def astar(array, start, goal, repulsion_weight=15.0):
    rows, cols = array.shape
    repulsion_map, walkable_mask = compute_repulsion_cost(array)

    neighbors = [(0, 1), (1, 0), (-1, 0), (0, -1)]

    open_heap = []
    heapq.heappush(open_heap, (0 + heuristic(start, goal), 0, start))

    came_from = {}
    g_score = np.full(array.shape, np.inf, dtype=np.float32)
    g_score[start] = 0
    in_open = set()
    in_open.add(start)
    closed = np.zeros(array.shape, dtype=bool)

    while open_heap:
        _, cost_so_far, current = heapq.heappop(open_heap)
        in_open.discard(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        closed[current] = True

        for dx, dy in neighbors: # could be better optimized
            nx, ny = current[0] + dx, current[1] + dy

            if not (0 <= nx < rows and 0 <= ny < cols):
                continue
            if walkable_mask[nx, ny] == 0 or closed[nx, ny]:
                continue

            neighbor = (nx, ny)
            move_cost = 1.0 + repulsion_weight * repulsion_map[nx, ny]
            tentative_g = cost_so_far + move_cost

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                fscore = tentative_g + heuristic(neighbor, goal)
                if neighbor not in in_open:
                    heapq.heappush(open_heap, (fscore, tentative_g, neighbor))
                    in_open.add(neighbor)

    return []

# default scale is 0.60, shows a good balanace, adjust based on needs
def astar_downscaled(array, start, goal, repulsion_weight=5.0, scale=1.0):
    small_array = cv2.resize(array, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    small_array = (small_array > 0).astype(np.uint8)

    start_small = (int(start[0] * scale), int(start[1] * scale))
    goal_small = (int(goal[0] * scale), int(goal[1] * scale))

    path_small = astar(small_array, start_small, goal_small, repulsion_weight)

    if not path_small:
        print("A* failed in downscaled space.")
        return []

    path = [(int(y / scale), int(x / scale)) for y, x in path_small]
    return path