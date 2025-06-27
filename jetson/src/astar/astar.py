import heapq
import numpy as np
import cv2

def heuristic(a, b):
    #return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manhattan
    return np.linalg.norm(np.array(a)-np.array(b)) #Euclidean

def compute_repulsion_cost(array):
    dist_transform = cv2.distanceTransform((array * 255).astype(np.uint8), cv2.DIST_L2, 3)
    max_dist = np.max(dist_transform)
    if max_dist == 0:
        return np.ones_like(dist_transform)
    repulsion = (1.0 - (dist_transform / max_dist)) ** 3.0
    return repulsion

def astar(array, start, goal, repulsion_weight=5.0):
    neighbors = [(0, 1), (1, 0), (-1, 0), (0, -1)]
    rows, cols = array.shape
    repulsion_map = compute_repulsion_cost(array)

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = [(fscore[start], start)]

    while oheap:
        _, current = heapq.heappop(oheap)

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
            if array[neighbor[0], neighbor[1]] == 0:
                continue

            cost = 1.0 + repulsion_weight * repulsion_map[neighbor[0], neighbor[1]]
            tentative_g = gscore[current] + cost

            if neighbor in close_set and tentative_g >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g < gscore.get(neighbor, float('inf')) or neighbor not in [x[1] for x in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g
                fscore[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False
