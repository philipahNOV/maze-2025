import time
import cv2
import numpy as np
import queue

import position_controller
import lowPassFilter
import path_following
import path_following_lookahead
import uitility_threads
from mqtt_client import MQTTClientJetson
from image_controller import ImageController

from camera.tracker_service import TrackerService

frame_queue = queue.Queue(maxsize=1)

def main(tracker: TrackerService,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson):
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)
    image_controller = ImageController()

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.is_initialized:
        time.sleep(0.1)
    print("[INFO] Tracking initialized.")

    # Define and densify path
    path_array = [
        (949, 701), (943, 589), (938, 478), (939, 363), (1027, 316),
        (1090, 245), (1069, 150), (974, 178), (896, 234), (825, 234), (825, 154),
        (736, 154), (734, 263), (772, 349), (825, 430), (843, 540),
        (850, 690), (770, 690), (747, 581), (640, 561), (560, 640),
        (420, 629), (420, 538), (530, 479), (650, 425), (650, 321),
        (583, 243), (552, 143), (552, 49), (687, 49), (763, 49)
    ]

    def densify_path(path, factor=6):
        new_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            new_path.append(tuple(p1))
            for j in range(1, factor):
                interp = p1 + (p2 - p1) * (j / factor)
                new_path.append(tuple(interp.astype(int)))
        new_path.append(path[-1])
        return new_path

    path_array = densify_path(path_array, factor=3)
    pathFollower = path_following.PathFollower(path_array, controller)
    pathFollowerLookahead = path_following_lookahead.PathFollower(path_array, controller)

    controller.horizontal()
    time.sleep(1)

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

            image_controller.frame = frame.copy()
            ball_pos = tracker.get_ball_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollowerLookahead.follow_path(ball_pos)
                cropped_frame = image_controller.update(ball_pos, pathFollowerLookahead, mqtt_client)
                if blinker is not None:
                    blinker.stop()
                    #blinker.join()
                    blinker = None
            else:
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                ball_pos = smoother.update(ball_pos)
                if blinker is None:
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = uitility_threads.BlinkRed(controller.arduinoThread)
                    blinker.start()

            # Display the visual overlay
            cv2.imshow("Ball tracking", cropped_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_speed(0, 0)
                cv2.destroyAllWindows()
                break

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
