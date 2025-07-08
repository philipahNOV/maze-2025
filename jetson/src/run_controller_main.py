import time
import cv2
import numpy as np
import queue

import position_controller
import lowPassFilter
import path_following
import path_following_lookahead
import light_controller
from mqtt_client import MQTTClientJetson
from image_controller import ImageController

from camera.tracker_service import TrackerService

def main(tracker: TrackerService,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson,
         path_array=None,
         stop_event=None):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)
    image_controller = ImageController()

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.is_initialized:
        time.sleep(0.1)
    print("[INFO] Tracking initialized.")

    if controller.lookahead:
        print("[INFO] Using lookahead path following.")
        pathFollower = path_following_lookahead.PathFollower(path_array, controller)
    else:
        print("[INFO] Using standard path following.")
        pathFollower = path_following.PathFollower(path_array, controller)

    controller.horizontal()
    time.sleep(1)
    wiggle_thread = light_controller.EscapeElevatorThread(controller.arduinoThread)
    wiggle_thread.start()
    time.sleep(6)

    # Set control loop parameters
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    blinker = None

    try:
        while not stop_event.is_set():
            loop_start = time.time()

            frame = tracker.get_stable_frame()
            if frame is None:
                time.sleep(0.015)
                continue

            image_controller.frame = frame.copy()
            ball_pos = tracker.get_ball_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                if blinker is not None:
                    blinker.stop()
                    blinker = None
            else:
                cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                ball_pos = smoother.update(ball_pos)
                if blinker is None:
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = light_controller.BlinkRed(controller.arduinoThread)
                    blinker.start()

            # Display the visual overlay
            #cv2.imshow("Ball tracking", cropped_frame)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break

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
        #cv2.destroyAllWindows()
        print("[INFO] Control thread exited.")
        if blinker is not None:
            blinker.stop()
            blinker = None
        controller.arduinoThread.send_speed(0, 0)

if __name__ == "__main__":
    main()
