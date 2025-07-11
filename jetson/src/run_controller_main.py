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
from image_controller import ImageSenderThread
from camera.tracker_service import TrackerService

def main(tracker: TrackerService,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson,
         path_array=None,
         image_controller:ImageController = None,
         stop_event=None,
         config=None):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

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

    image_thread = ImageSenderThread(image_controller, mqtt_client, tracker, path_array, pathFollower, stop_event=stop_event)
    image_controller.set_new_path(path_array)
    image_thread.start()

    controller.horizontal()
    escape_thread = uitility_threads.EscapeElevatorThread(controller.arduinoThread)
    escape_thread.start()
    time.sleep(escape_thread.duration)
    controller.horizontal()

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
                #pathFollower.follow_path(ball_pos)
                #cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                if blinker is not None:
                    blinker.stop()
                    blinker = None
                    ball_not_found_timer = None
            else:
                #cropped_frame = image_controller.update(ball_pos, pathFollower, mqtt_client)
                ball_pos = smoother.update(ball_pos)
                if blinker is None:
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = uitility_threads.BlinkRed(controller.arduinoThread)
                    blinker.start()
                    ball_not_found_timer = time.time()

            if ball_not_found_timer is not None:
                elapsed_time = time.time() - ball_not_found_timer
                if elapsed_time > ball_not_found_limit:
                    print("[WARNING] Ball not found for too long, returning to main menu.")
                    controller.arduinoThread.send_speed(0, 0)
                    ball_not_found_timer = None
                    mqtt_client.client.publish("pi/info", "timeout")
                    break

            cv2.imshow("Ball tracking", frame)
            print(tracker.get_orientation())
            if cv2.waitKey(1) & 0xFF == ord('q'):
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
        #cv2.destroyAllWindows()
        print("[INFO] Control thread exited.")
        if blinker is not None:
            blinker.stop()
            blinker = None
        controller.arduinoThread.send_speed(0, 0)
        if image_thread.is_alive():
            image_thread.stop()
            image_thread.join()

if __name__ == "__main__":
    main()
