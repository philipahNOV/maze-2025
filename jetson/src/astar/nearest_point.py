import cv2
import numpy as np

def find_nearest_walkable(mask, point, max_radius=10):
    y, x = point
    if mask[y, x] != 0:
        return point

    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    min_val = float('inf')
    nearest = point

    h, w = mask.shape
    for dy in range(-max_radius, max_radius + 1):
        for dx in range(-max_radius, max_radius + 1):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] > 0:
                if dist[ny, nx] < min_val:
                    min_val = dist[ny, nx]
                    nearest = (ny, nx)
    return nearest
