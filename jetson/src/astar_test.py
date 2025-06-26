import heapq
import cv2
import numpy as np

def to_grid_coords(pos, scale):
    x, y = pos
    return (int(y * scale), int(x * scale))  # (row, col)

def to_pixel_coords(path, scale):
    return [(int(col / scale), int(row / scale)) for (row, col) in path]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def compute_repulsion_cost(grid):
    # grid: np.ndarray of 0=free, 1=obstacle
    # invert for distanceTransform: walls→0, free→255
    inv = ((1 - grid).astype(np.uint8)) * 255
    dist = cv2.distanceTransform(inv, cv2.DIST_L2, 3).astype(np.float32)
    max_d = dist.max() if dist.max() > 0 else 1.0
    repulse = 1.0 - (dist / max_d)
    return repulse

def astar(grid, start, goal, repulsion_weight=5.0):
    """
    grid: np.ndarray shape (H, W), with 0=free, 1=wall
    start, goal: (row, col) tuples
    """
    if not isinstance(grid, np.ndarray):
        grid = np.array(grid, dtype=np.uint8)

    H, W = grid.shape
    repulse = compute_repulsion_cost(grid)

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    gscore = {start: 0}

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W):
                continue
            if grid[nr, nc] != 0:
                continue

            move_cost = 1.0 + repulse[nr, nc] * repulsion_weight
            tentative = cost + move_cost
            neigh = (nr, nc)

            if tentative < gscore.get(neigh, float('inf')):
                gscore[neigh] = tentative
                fscore = tentative + heuristic(neigh, goal)
                heapq.heappush(open_set, (fscore, tentative, neigh))
                came_from[neigh] = current

    return None  # no path found