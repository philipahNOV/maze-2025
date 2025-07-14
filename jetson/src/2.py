import time
import cv2
import numpy as np
import position_controller
import low_pass_filter as low_pass_filter
import path_following
import path_following_lookahead
import uitility_threads
from camera.tracker_service import TrackerService
from logger import LoggingThread
from pathlib import Path
import yaml
import pos2
from arduino_connection import ArduinoConnection

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

def initialize_component(component, name, retries=5, delay=2):
    for attempt in range(retries):
        try:
            comp_instance = component()
            print(f"{name} initialized on attempt {attempt + 1}")
            return comp_instance
        except Exception as e:
            print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
            time.sleep(delay)
    raise Exception(f"Failed to initialize {name} after {retries} attempts")

def load_config():
    project_root = Path(__file__).resolve().parents[2]
    config_path = project_root / "config.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}")

    with config_path.open('r') as file:
        config = yaml.safe_load(file)
    return config

def draw_waypoints(frame, pathFollower):
    if frame is None:
        return
    for i in range(pathFollower.length):
        if i < pathFollower.next_waypoint:
            color = (0, 200, 0)
        elif i == pathFollower.next_waypoint:
            color = (0, 255, 255)
        else:
            color = (0, 0, 255)
        cv2.circle(frame, pathFollower.path[i], 5, color, -1)
    return frame

def get_dynamic_path(tracker, smoother):
    print("[INFO] Capturing maze frame...")
    maze_frame = tracker.get_stable_frame()
    while maze_frame is None:
        time.sleep(0.1)
        maze_frame = tracker.get_stable_frame()

    gray = get_dynamic_threshold(maze_frame)
    binary_mask = create_binary_mask(gray)
    safe_mask = dilate_mask(binary_mask)
    safe_mask = (safe_mask > 0).astype(np.uint8) * 255

    cv2.circle(safe_mask, (998, 588), 70, 255, -1)

    ball_pos = tracker.get_ball_position()
    while ball_pos is None:
        print("Waiting for ball position...")
        time.sleep(0.1)
        ball_pos = tracker.get_ball_position()

    ball_pos = smoother.update(ball_pos)
    start_raw = (ball_pos[1], ball_pos[0])  # (y, x)
    start = find_nearest_walkable(safe_mask, start_raw)

    global clicked_goal
    clicked_goal = None
    cv2.namedWindow("Safe Mask")
    cv2.setMouseCallback("Safe Mask", on_mouse_click)
    cv2.circle(safe_mask, (start[1], start[0]), 10, 127, -1)

    print("[INFO] Click on a goal point...")
    while clicked_goal is None:
        cv2.imshow("Safe Mask", safe_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[INFO] Goal selection cancelled.")
            return None
    cv2.destroyWindow("Safe Mask")
    goal = clicked_goal

    print(f"[INFO] Calculating path from {start} to {goal}...")
    path = astar_downscaled(safe_mask, start, goal, repulsion_weight=5.0, scale=1.0)
    waypoints = sample_waypoints(path, safe_mask)
    path_array = [(x, y) for y, x in waypoints]
    print(waypoints)
    print(path_array)
    return path_array, path, start, goal, waypoints

def main():
    try:
        arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
        time.sleep(5)
    except Exception as e:
        print(e)
        exit(1)

    config = load_config()
    tracking_config = config["tracking"]
    tracker = TrackerService(
        model_path=tracking_config["model_path"],
        tracking_config=tracking_config
    )
    tracker.camera.init_camera()
    tracker.start_tracker()

    smoother = low_pass_filter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.is_initialized:
        time.sleep(0.1)
    print("[INFO] Tracking initialized.")

    path_data = get_dynamic_path(tracker, smoother)
    if path_data is None:
        return

    path_array, full_path, start, goal, sampled_waypoints = path_data

    controller = pos2.Controller(
        arduinoThread=arduino_thread,
        tracker=tracker,
        path_following=True,
        lookahead=False,
        config=config
    )

    if controller.lookahead:
        print("[INFO] Using lookahead path following.")
        pathFollower = path_following_lookahead.PathFollower(path_array, controller, config)
    else:
        print("[INFO] Using standard path following.")
        pathFollower = path_following.PathFollower(path_array, controller, config)

    controller.horizontal()
    logger = LoggingThread(path_array, config)
    logger.start()
    controller.logger = logger

    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    blinker = None
    ball_not_found_timer = None
    ball_not_found_limit = 30

    try:
        while True:
            loop_start = time.time()

            frame = tracker.get_stable_frame()
            if frame is None:
                time.sleep(0.015)
                continue

            ball_pos = tracker.get_ball_position()
            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                if blinker:
                    blinker.stop()
                    blinker = None
                    ball_not_found_timer = None
            else:
                ball_pos = smoother.update(ball_pos)
                if blinker is None:
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = uitility_threads.BlinkRed(controller.arduinoThread)
                    blinker.start()
                    ball_not_found_timer = time.time()

            if ball_not_found_timer and time.time() - ball_not_found_timer > ball_not_found_limit:
                print("[WARNING] Ball not found for too long, exiting.")
                controller.arduinoThread.send_speed(0, 0)
                break

            frame = draw_path(frame, full_path, sampled_waypoints, start, goal)
            frame = draw_waypoints(frame, pathFollower)
            if ball_pos:
                cv2.circle(frame, ball_pos, 10, (0, 255, 0), -1)

            cv2.imshow("Ball tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            sleep_time = LOOP_DT - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    except Exception as e:
        import traceback
        print(f"[ERROR] Control loop crashed: {e}")
        traceback.print_exc()
    finally:
        print("[INFO] Exiting control loop.")
        if blinker:
            blinker.stop()
        controller.arduinoThread.send_speed(0, 0)
        tracker.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()