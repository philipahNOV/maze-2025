import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
from astar.astar import astar, astar_downscaled
from astar.brightness import get_dynamic_threshold, create_binary_mask, dilate_mask
import math
import numpy as np
from optuna_2 import pid_tuning_dual_axis

clicked_goal = None

def on_mouse_click(event, x, y, flags, param):
    global clicked_goal
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_goal = (y, x)
        print(f"[INFO] Clicked goal set to: {clicked_goal}")

def snap_to_nearest_walkable(mask, point, max_radius=10):
    y, x = point
    if mask[y, x] != 0:
        return point  # already walkable

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

def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("Waiting for tracking initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("Tracking started!")
    print("Finding shortest path...")

    while tracker.frame is None:
        time.sleep(0.1)

    maze_frame = tracker.get_stable_frame()
    while maze_frame is None:
        time.sleep(0.1)
        maze_frame = tracker.get_stable_frame()

    gray = get_dynamic_threshold(maze_frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = dilate_mask(binary_mask)

    start = (604, 950)
    # ball_pos = tracker.get_position()
    # while ball_pos is None:
    #     print("Waiting for ball position...")
    #     time.sleep(0.1)
    #     ball_pos = tracker.get_position()

    # ball_pos = smoother.update(ball_pos)
    # start_raw = (ball_pos[1], ball_pos[0])  # (y, x)
    # start = snap_to_nearest_walkable(safe_mask, start_raw)
    #goal = (990, 704)
    global clicked_goal
    clicked_goal = None

    cv2.namedWindow("Safe Mask")
    cv2.setMouseCallback("Safe Mask", on_mouse_click)

    preview_mask = safe_mask.copy()
    cv2.circle(preview_mask, (start[1], start[0]), 10, 127, -1)
    print("Click on a goal point...")

    while clicked_goal is None:
        cv2.imshow("Safe Mask", preview_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Goal selection cancelled.")
            return

    cv2.destroyWindow("Safe Mask")
    goal = clicked_goal

    path = astar_downscaled(safe_mask, start, goal, repulsion_weight=5.0, scale=0.55)
    waypoints = sample_waypoints(path)
    pid_tuning_dual_axis(controller, waypoints, n_trials=50, start=start)
    path_array = [(x, y) for y, x in waypoints]
    pathFollower = path_following.PathFollower(path_array, controller)

    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            
            frame = draw_path(frame, path, waypoints, start, goal)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            pathFollower.follow_path(ball_pos)

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(frame, path_array[i], 5, (0, 0, 255), -1)

            cv2.imshow("Ball & Marker Tracking", frame)

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                return

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("Tracker stopped.")

if __name__ == "__main__":
    main()