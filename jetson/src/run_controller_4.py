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


def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")
    print("[INFO] Capturing maze for A* planning...")
    while tracker.frame is None:
        time.sleep(0.1)

    maze_frame = tracker.frame.copy()
    gray = get_dynamic_threshold(maze_frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = cv2.dilate(binary_mask, np.ones((3, 3), np.uint8), iterations=2)

    start = (738, 699)
    goal = (830, 60)

    path = astar(safe_mask, start, goal, repulsion_weight=5.0)
    waypoints = sample_waypoints(path)
    path_array = [(x, y) for y, x in waypoints]

    # Pass path to your PathFollower
    pathFollower = path_following.PathFollower(path_array, controller)


    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            for x, y in pathFollower.path:
                cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)


            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.circle(frame, (770-150, 330-150), 5, (0, 0, 255), -1)
            cv2.circle(frame, (770+150, 330+150), 5, (0, 0, 255), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Ball & Marker Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            # Use path following instead of static control
            pathFollower.follow_path(ball_pos)

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                return

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()