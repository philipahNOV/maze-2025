import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
from astar.astar import astar
from astar.brightness import get_dynamic_threshold, create_binary_mask
import math
import numpy as np

def dilate_mask(mask, iterations=2):
    kernel = np.ones((3, 3), np.uint8)
    return cv2.dilate(mask, kernel, iterations=iterations)

def sample_waypoints(path):
    if not path or len(path) < 2:
        return path or []

    total_length = sum(
        math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        for p1, p2 in zip(path[:-1], path[1:])
    )

    spacing = max(total_length, 100)

    waypoints = [path[0]]
    last_point = path[0]
    accumulated = 0.0

    for point in path[1:]:
        dy = point[0] - last_point[0]
        dx = point[1] - last_point[1]
        dist = math.hypot(dy, dx)
        accumulated += dist
        if accumulated >= spacing:
            waypoints.append(point)
            accumulated = 0.0
            last_point = point

    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints

def draw_path(image, path, waypoints, start, goal):
    out = image.copy()
    if len(out.shape) == 2 or out.shape[2] == 1:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    elif out.shape[2] == 4:
        out = cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)

    h, w = out.shape[:2]

    for y, x in path or []:
        if 0 <= y < h and 0 <= x < w:
            out[y, x] = (0, 0, 255)

    for y, x in waypoints or []:
        if 0 <= y < h and 0 <= x < w:
            cv2.circle(out, (x, y), 2, (0, 255, 255), -1)

    if start:
        cv2.circle(out, (start[1], start[0]), 5, (0, 255, 0), -1)
    if goal:
        cv2.circle(out, (goal[1], goal[0]), 5, (255, 0, 0), -1)

    return out

def main(tracker: tracking.BallTracker):

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")
    print("[INFO] Capturing maze for A* planning...")

    maze_frame = tracker.frame
    gray = get_dynamic_threshold(maze_frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = cv2.dilate(binary_mask, np.ones((3, 3), np.uint8), iterations=2)

    start = (55, 840)
    goal = (680, 790)

    cv2.circle(safe_mask, (start[0], start[1]), 15, 127, -1)  # start = (y, x)
    cv2.circle(safe_mask, (goal[0], goal[1]), 15, 200, -1)    # goal = (y, x)
    cv2.imshow("Safe Mask", safe_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    path = astar(safe_mask, start, goal, repulsion_weight=5.0)
    waypoints = sample_waypoints(path)
    path_array = [(x, y) for x, y in waypoints]


if __name__ == "__main__":
    main()