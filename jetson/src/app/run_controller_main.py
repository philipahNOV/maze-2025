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

def main(tracker: TrackerService,
         controller: position_controller.Controller,
         mqtt_client: MQTTClientJetson,
         path_array=None,
         image_controller:ImageController = None,
         stop_event=None,
         config=None,
         playervsai=False):

    alpha = config["controller"].get("position_smoothing_alpha", 0.1)
    smoother = utils.low_pass_filter.SmoothedTracker(alpha=alpha)

    ball_not_found_timer = None
    ball_not_found_limit = 60  # seconds

    while not tracker.is_initialized:
        time.sleep(0.1)

    if controller.lookahead:
        pathFollower = path_following_lookahead.PathFollower(path_array, controller, config)
    else:
        pathFollower = path_following.PathFollower(path_array, controller, config)
    controller.path_follower = pathFollower

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

    TARGET_HZ = 50
    LOOP_DT = 1.0 / TARGET_HZ
    blinker = None

    try:
        while not stop_event.is_set():
            loop_start = time.time()

            if tracker.get_stable_frame() is None:
                time.sleep(0.005)
                continue

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
                        controller.arduinoThread.send_speed(0, 0)
                        controller.e_x_int = 0
                        controller.e_y_int = 0
                        controller.prevPos = None
                        controller.prevVel = None
                        controller.vel = None
                        controller.pos = None
                        time.sleep(1.0)
                        escape_thread = utility_threads.EscapeElevatorThread(controller.arduinoThread, controller)
                        escape_thread.start()
                        time.sleep(escape_thread.duration)
                        controller.horizontal()
                        if controller.lookahead:
                            pathFollower = path_following_lookahead.PathFollower(path_array, controller, config)
                            image_thread.path_follower = pathFollower
                        else:
                            pathFollower = path_following.PathFollower(path_array, controller, config)
                            image_thread.path_follower = pathFollower
                        controller.path_follower = pathFollower
                        smoother.smoothed_pos = None
                    blinker = None
                    ball_not_found_timer = None
            else:
                ball_pos = smoother.update(ball_pos)
                controller.arduinoThread.send_speed(0, 0)
                if blinker is None and not playervsai:
                    blinker = utility_threads.BlinkRed(controller.arduinoThread, config, controller)
                    blinker.start()
                    ball_not_found_timer = time.time()

                    # if logger is not None:
                    #     logger.mark_ball_lost()

            if ball_not_found_timer is not None:
                elapsed_time = time.time() - ball_not_found_timer
                if elapsed_time > ball_not_found_limit:
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
        if blinker is not None:
            blinker.stop()
            blinker = None
        controller.arduinoThread.send_speed(0, 0)
        controller.arduinoThread.send_color(255, 255, 255)
        for _ in range(5):
            controller.arduinoThread.send_elevator(-1)
            time.sleep(0.05)
        if image_thread.is_alive():
            image_thread.stop()
            image_thread.join()
        # if logger is not None:
        #     logger.stop()
        #     logger = None

if __name__ == "__main__":
    main()