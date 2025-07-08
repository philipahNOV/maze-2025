import time
import cv2
import math
import base64
import numpy as np
import position_controller
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
from astar.astar import astar_downscaled
from astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
from astar.nearest_point import find_nearest_walkable
from astar.waypoint_sampling import sample_waypoints
from astar.draw_path import draw_path

clicked_goal = None

def on_mouse_click(event, x, y, flags, param):
    global clicked_goal
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_goal = (y, x)
        print(f"[INFO] Clicked goal set to: {clicked_goal}")

def send_frame_to_pi(mqtt_client: MQTTClientJetson, frame):
    scale = 0.5
    height, width = frame.shape[:2]
    new_size = (int(width * scale), int(height * scale))
    resized = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
    _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    mqtt_client.client.publish("pi/camera", jpg_as_text)

def main(tracker, controller: position_controller.Controller, mqtt_client: MQTTClientJetson):
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

    safe_mask = (safe_mask > 0).astype(np.uint8) * 255

    start = (630, 1030)  # y, x
    cv2.circle(safe_mask, (start[1], start[0]), 70, 255, -1)

    ball_pos = tracker.get_position()
    while ball_pos is None:
        print("Waiting for ball position...")
        time.sleep(0.1)
        ball_pos = tracker.get_position()

    ball_pos = smoother.update(ball_pos)
    start_raw = (ball_pos[1], ball_pos[0])  # (y, x)
    start = find_nearest_walkable(safe_mask, start_raw)

    global clicked_goal
    cv2.namedWindow("Safe Mask")
    cv2.setMouseCallback("Safe Mask", on_mouse_click)
    cv2.circle(safe_mask, (start[1], start[0]), 10, 127, -1)

    print("Click on a goal point...")
    while clicked_goal is None:
        cv2.imshow("Safe Mask", safe_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Goal selection cancelled.")
            return

    cv2.destroyWindow("Safe Mask")
    goal = clicked_goal

    path = astar_downscaled(safe_mask, start, goal, repulsion_weight=5.0, scale=0.60)
    waypoints = sample_waypoints(path, safe_mask)
    path_array = [(x, y) for y, x in waypoints]
    pathFollower = path_following.PathFollower(path_array, controller)

    controller.horizontal()
    time.sleep(1)

    last_sent_frame_time = time.time()
    frame_send_hz = 5
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    frame_warned = False
    x1, y1, x2, y2 = 390, 10, 1120, 720

    try:
        while True:
            loop_start = time.time()

            frame = tracker.frame
            if frame is None:
                if not frame_warned:
                    print("[WARN] Tracker returned empty frame. Waiting...")
                    frame_warned = True
                time.sleep(0.015)
                continue

            ball_pos = tracker.get_position()
            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cv2.circle(frame, ball_pos, 8, (255, 165, 0), -1)
            else:
                controller.arduinoThread.send_target_positions(0, 0)
                ball_pos = smoother.update(ball_pos)

            frame = draw_path(frame, path, waypoints, start, goal)

            if not ball_pos:
                print("No ball found")
                continue

            for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 200, 0), -1)
                elif i == pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(frame, path_array[i], 5, (0, 0, 255), -1)

            cropped_frame = frame[y1:y2, x1:x2]
            cropped_frame = cv2.rotate(cropped_frame, cv2.ROTATE_180)

            if time.time() > last_sent_frame_time + 1 / frame_send_hz:
                send_frame_to_pi(mqtt_client, cropped_frame)
                last_sent_frame_time = time.time()

            cv2.imshow("Ball tracking", cropped_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_target_positions(0, 0)
                cv2.destroyAllWindows()
                break

            sleep_time = LOOP_DT - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("Tracker stopped.")

if __name__ == "__main__":
    main()