# File: astar_test.py

import heapq

def astar(start, goal, grid):
    """
    Perform A* search on a 2D occupancy grid.
    grid: either a NumPy 2D array or a Python list-of-lists where
          0 = free space, 1 = obstacle
    start, goal: (row, col) tuples
    Returns a list of (row, col) from start to goal inclusive, or [] if no path.
    """
    # Determine grid dimensions in a way that works for both lists and NumPy arrays
    try:
        H, W = grid.shape
    except AttributeError:
        H = len(grid)
        W = len(grid[0])

    def in_bounds(p):
        r, c = p
        return 0 <= r < H and 0 <= c < W

    def neighbors(p):
        r, c = p
        # 4‐connected grid; drop diagonals for simplicity
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = r + dr, c + dc
            if in_bounds((nr, nc)) and grid[nr][nc] == 0:
                yield (nr, nc)

    def heuristic(a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    # (f_score, g_score, node)
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    gscore = { start: 0 }

    while open_set:
        f, cost, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for nbr in neighbors(current):
            tentative_g = cost + 1  # uniform cost per move
            if tentative_g < gscore.get(nbr, float('inf')):
                gscore[nbr] = tentative_g
                fscore = tentative_g + heuristic(nbr, goal)
                heapq.heappush(open_set, (fscore, tentative_g, nbr))
                came_from[nbr] = current

    # no path found
    return []
