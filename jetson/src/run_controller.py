import YOLO_tracking.hsv3 as tracking
import time
import cv2
import position_controller
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
import queue
import base64
from image_controller import ImageController

frame_queue = queue.Queue(maxsize=1)

def main(tracker: tracking.BallTracker, controller: position_controller.Controller, mqtt_client: MQTTClientJetson):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.4)
    image_controller = ImageController()

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking initialized.")

    path_array = [
        (949, 701), (943, 589), (938, 478), (939, 363), (1027, 316),
        (1090, 245), (1069, 150), (974, 178), (896, 234), (825, 234), (825, 154),
        (736, 154), (734, 263), (772, 349), (825, 430), (843, 540),
        (850, 690), (770, 690), (747, 581), (640, 561), (560, 640),
        (420, 629), (420, 538), (530, 479), (650, 425), (650, 321),
        (583, 243), (552, 143), (552, 49), (687, 49), (763, 49)
    ]
    pathFollower = path_following.PathFollower(path_array, controller)
    controller.horizontal()
    time.sleep(1)

    
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ

    try:
        while True:
            loop_start = time.time()

            frame = tracker.frame
            if frame is None:
                time.sleep(0.015)
                continue

            image_controller.frame = frame.copy()

            ball_pos = tracker.get_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
            else:
                controller.arduinoThread.send_target_positions(0, 0)
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                ball_pos = smoother.update(ball_pos)

            cv2.imshow("Ball tracking", cropped_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_target_positions(0, 0)
                cv2.destroyAllWindows()
                break

            # === Maintain 60 Hz ===
            loop_duration = time.time() - loop_start
            sleep_time = LOOP_DT - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    except Exception as e:
        print(f"[ERROR] Control loop crashed: {e}")
    finally:
        #tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Control thread exited.")

if __name__ == "__main__":
    main()