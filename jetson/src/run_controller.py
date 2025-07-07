import YOLO_tracking.hsv3 as tracking
import time
import cv2
import position_controller
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
import queue
from image_controller import ImageController
import light_controller
import threading

frame_queue = queue.Queue(maxsize=1)

def main(tracker: tracking.BallTracker,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson):
    """
    Main control loop for vision-based ball-on-plate control.

    Responsibilities:
    - Wait for tracker to initialize
    - Follow a predefined waypoint path
    - Track and smooth ball position
    - Control platform orientation using controller
    - Update and send visual feedback to Pi via MQTT

    Args:
        tracker (BallTracker): YOLO-based ball tracking module
        controller (Controller): Ball-on-plate position controller
        mqtt_client (MQTTClientJetson): MQTT client for Jetson to Pi communication
    """

    smoother = lowPassFilter.SmoothedTracker(alpha=0.4)
    image_controller = ImageController()

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)
    print("[INFO] Tracking initialized.")

    # Define the path to follow (list of pixel coordinates)
    path_array = [
        (949, 701), (943, 589), (938, 478), (939, 363), (1027, 316),
        (1090, 245), (1069, 150), (974, 178), (896, 234), (825, 234), (825, 154),
        (736, 154), (734, 263), (772, 349), (825, 430), (843, 540),
        (850, 690), (770, 690), (747, 581), (640, 561), (560, 640),
        (420, 629), (420, 538), (530, 479), (650, 425), (650, 321),
        (583, 243), (552, 143), (552, 49), (687, 49), (763, 49)
    ]

    pathFollower = path_following.PathFollower(path_array, controller)

    # Level the platform before starting path following
    controller.horizontal()
    time.sleep(1)

    # Set loop rate
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ

    try:
        while True:
            loop_start = time.time()

            # Get current frame from tracker
            frame = tracker.frame
            if frame is None:
                time.sleep(0.015)
                continue

            # Set frame for image overlay operations
            image_controller.frame = frame.copy()

            # Track and smooth the ball position
            ball_pos = tracker.get_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
            else:
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                controller.arduinoThread.send_speed(0, 0)
                ball_pos = smoother.update(ball_pos)  # smooth None to hold last known


            # Display cropped frame on local display
            cv2.imshow("Ball tracking", cropped_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Stop control externally
            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_speed(0, 0)
                cv2.destroyAllWindows()
                break

            # Maintain real-time control loop timing
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
        cv2.destroyAllWindows()
        print("[INFO] Control thread exited.")

if __name__ == "__main__":
    main()
