import heapq
import math


def astar(start, goal, grid):
    """
    Perform A* search on a 2D grid.
    grid: 2D list of 0 (free) or 1 (obstacle)
    start, goal: (x, y) tuples
    Returns list of (x, y) from start to goal inclusive, or [] if no path found.
    """
    H, W = len(grid), len(grid[0])

    def in_bounds(p):
        x, y = p
        return 0 <= x < W and 0 <= y < H

    def neighbors(p):
        x, y = p
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1), (1,1),(1,-1),(-1,1),(-1,-1)]:
            np_ = (x + dx, y + dy)
            if in_bounds(np_) and grid[np_[1]][np_[0]] == 0:
                yield np_

    def heuristic(a, b):
        # Euclidean distance
        return math.hypot(b[0] - a[0], b[1] - a[1])

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

        for nbr in neighbors(current):
            tentative = cost + heuristic(current, nbr)
            if tentative < gscore.get(nbr, float('inf')):
                gscore[nbr] = tentative
                fscore = tentative + heuristic(nbr, goal)
                heapq.heappush(open_set, (fscore, tentative, nbr))
                came_from[nbr] = current

    return []  # no path found