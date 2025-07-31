import time
from mqtt.mqtt_client import MQTTClientJetson
import control.position_controller as position_controller
import utils.low_pass_filter
import control.paths.path_following as path_following
import control.paths.path_following_lookahead as path_following_lookahead
import utils.utility_threads as utility_threads
from control.image_controller import ImageController
from control.image_controller import ImageSenderThread
from tracking.tracker_service import TrackerService
from logger import LoggingThread

def main(tracker: TrackerService,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson,
         path_array=None,
         image_controller:ImageController = None,
         stop_event=None,
         config=None):

    smoother = utils.low_pass_filter.SmoothedTracker(alpha=0.5)

    ball_not_found_timer = None
    ball_not_found_limit = 60  # seconds

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
    escape_thread = utility_threads.EscapeElevatorThread(controller.arduinoThread, controller)
    escape_thread.start()
    time.sleep(escape_thread.duration)
    controller.horizontal()

    # logger = None
    # logger = LoggingThread(path_array)
    # logger.start()
    # controller.logger = logger

    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    blinker = None

    try:
        while not stop_event.is_set():
            loop_start = time.time()

            frame = tracker.get_stable_frame()
            if frame is None:
                time.sleep(0.005)  # Reduced from 0.015 to avoid blocking YOLO
                continue

            image_controller.frame = frame#.copy()
            ball_pos = tracker.get_ball_position()
            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                # logger.set_waypoint(pathFollower.get_current_waypoint())

                if blinker is not None:
                    blinker.stop()
                    # if logger is not None:
                    #     logger.reset_ball_lost()
                    if blinker.triggered:
                        escape_thread = utility_threads.EscapeElevatorThread(controller.arduinoThread, controller)
                        escape_thread.start()
                        time.sleep(escape_thread.duration)
                        controller.horizontal()
                        if controller.lookahead:
                            print("[INFO] Using lookahead path following.")
                            pathFollower = path_following_lookahead.PathFollower(path_array, controller, config)
                            image_thread.path_follower = pathFollower
                        else:
                            print("[INFO] Using standard path following.")
                            pathFollower = path_following.PathFollower(path_array, controller, config)
                            image_thread.path_follower = pathFollower
                    blinker = None
                    ball_not_found_timer = None
            else:
                ball_pos = smoother.update(ball_pos)

                if blinker is None:
                    controller.arduinoThread.send_speed(0, 0)
                    blinker = utility_threads.BlinkRed(controller.arduinoThread, config, controller)
                    blinker.start()
                    ball_not_found_timer = time.time()
                    
                    # if logger is not None:
                    #     logger.mark_ball_lost()

            if ball_not_found_timer is not None:
                elapsed_time = time.time() - ball_not_found_timer
                if elapsed_time > ball_not_found_limit:
                    print("[WARNING] Ball not found for too long, returning to main menu.")
                    controller.arduinoThread.send_speed(0, 0)
                    ball_not_found_timer = None
                    mqtt_client.client.publish("pi/info", "timeout")
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
        print("[INFO] Control thread exited.")
        if blinker is not None:
            blinker.stop()
            blinker = None
        controller.arduinoThread.send_speed(0, 0)
        if image_thread.is_alive():
            image_thread.stop()
            image_thread.join()
        # if logger is not None:
        #     logger.stop()
        #     logger = None

if __name__ == "__main__":
    main()