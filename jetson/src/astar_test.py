import heapq
import cv2
import numpy as np

def to_grid_coords(pos, scale):
    """
    Convert a pixel position (x, y) into grid coords (row=y, col=x) at the given scale.
    """
    x, y = pos
    return (int(y * scale), int(x * scale))  # (row, col)

def to_pixel_coords(path, scale):
    """
    Convert a list of grid coords (row, col) back into pixel positions (x, y).
    """
    return [(int(col / scale), int(row / scale)) for (row, col) in path]

def heuristic(a, b):
    """
    Manhattan distance between grid cells a=(r1,c1) and b=(r2,c2).
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def compute_repulsion_cost(grid):
    """
    Given a binary obstacle grid (0=free, 1=wall), compute a repulsion map:
    high cost near walls, low cost in open space.
    """
    # distanceTransform wants obstacles==0, free==nonzero, so invert:
    inv = (1 - grid).astype(np.uint8) * 255
    dist = cv2.distanceTransform(inv, cv2.DIST_L2, 3).astype(np.float32)
    max_d = dist.max() or 1.0
    # normalize [0,1] then invert so cells near walls -> cost near 1
    repulse = 1.0 - (dist / max_d)
    return repulse

def astar_with_repulsion(grid, start, goal, repulsion_weight=5.0):
    """
    A* on a binary np.ndarray grid:
      grid[r,c]==0 is free, 1 is obstacle.
    start/goal are (row, col) tuples.
    repulsion_weight scales extra cost for cells near walls.
    Returns a list of (row, col) from start to goal, or None if no path.
    """
    h, w = grid.shape
    repulse = compute_repulsion_cost(grid)

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    gscore = {start: 0}

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:  # 4-connected
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):  
                continue
            if grid[nr, nc] != 0:  
                continue

            # base move cost = 1, plus repulsion
            move_cost = 1.0 + repulse[nr, nc] * repulsion_weight
            tentative = cost + move_cost

            neighbor = (nr, nc)
            if tentative < gscore.get(neighbor, float('inf')):
                gscore[neighbor] = tentative
                fscore = tentative + heuristic(neighbor, goal)
                heapq.heappush(open_set, (fscore, tentative, neighbor))
                came_from[neighbor] = current

    return None  # no path