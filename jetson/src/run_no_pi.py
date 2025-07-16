import time
import position_controller
import jetson.src.low_pass_filter as low_pass_filter
import path_following
import path_following_lookahead
import utility_threads
from mqtt_client import MQTTClientJetson
from image_controller import ImageController
from image_controller import ImageSenderThread
from camera.tracker_service import TrackerService
from logger import LoggingThread
from pathlib import Path
import yaml
import pos2
from arduino_connection import ArduinoConnection
import cv2


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
    project_root = Path(__file__).resolve().parents[2]  # up from src → jetson → maze-2025
    config_path = project_root / "config.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}")

    with config_path.open('r') as file:
        config = yaml.safe_load(file)
    return config

def draw_waypoints(frame, pathFollower):
        """Draw path waypoints on the current frame with color coding."""
        if frame is None:
            return

        for i in range(pathFollower.length):
            if i < pathFollower.next_waypoint:
                color = (0, 200, 0)  # Green for past waypoints
            elif i == pathFollower.next_waypoint:
                color = (0, 255, 255)  # Yellow for current target
            else:
                color = (0, 0, 255)  # Red for future waypoints
            cv2.circle(frame, pathFollower.path[i], 5, color, -1)
        return frame


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
    controller = pos2.Controller(
        arduinoThread=arduino_thread,
        tracker=tracker,
        path_following=True,
        lookahead=False,
        config=config
    )

    path_array = [
            (949, 701), (943, 589), (938, 478), (939, 363), (1027, 316),
            (1090, 245), (1069, 150), (974, 178), (896, 234), (825, 234), (825, 154),
            (736, 154), (734, 263), (772, 349), (825, 430), (843, 540),
            (850, 690), (770, 690), (747, 581), (640, 561), (560, 640),
            (420, 629), (420, 538), (530, 479), (650, 425), (650, 321),
            (583, 243), (552, 143), (552, 49), (687, 49), (763, 49)
        ]

    smoother = low_pass_filter.SmoothedTracker(alpha=0.5)

    ball_not_found_timer = None
    ball_not_found_limit = 30  # seconds

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.is_initialized:
        time.sleep(0.1)
    print("[INFO] Tracking initialized.")

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

    # Set control loop parameters
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    blinker = None

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
                if blinker is not None:
                    blinker.stop()
                    blinker = None
                    ball_not_found_timer = None
            else:
                ball_pos = smoother.update(ball_pos)
                if blinker is None:
                    # Ball not found, start blinking red LED
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = utility_threads.BlinkRed(controller.arduinoThread)
                    blinker.start()
                    ball_not_found_timer = time.time()

            if ball_not_found_timer is not None:
                elapsed_time = time.time() - ball_not_found_timer
                if elapsed_time > ball_not_found_limit:
                    print("[WARNING] Ball not found for too long, returning to main menu.")
                    controller.arduinoThread.send_speed(0, 0)
                    ball_not_found_timer = None
                    break

            frame = draw_waypoints(frame, pathFollower)
            cv2.circle(frame, ball_pos, 10, (0, 255, 0), -1) if ball_pos else None
            cv2.imshow("Ball tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Keep loop running at the target frequency
            loop_duration = time.time() - loop_start
            sleep_time = LOOP_DT - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    except Exception as e:
        import traceback
        print(f"[ERROR] Control loop crashed: {e}")
        traceback.print_exc()
    finally:
        print("[INFO] Control thread exited.")
        if blinker is not None:
            blinker.stop()
            blinker = None
        controller.arduinoThread.send_speed(0, 0)

if __name__ == "__main__":
    main()
