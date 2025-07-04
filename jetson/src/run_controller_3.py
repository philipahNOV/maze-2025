import testing.yolov1.hsv4 as tracking
import time
import cv2
import positionController_2
import arduino_connection_test
import lowPassFilter
import path_following
import path_following_mpc
from mqtt_client import MQTTClientJetson
import queue

frame_queue = queue.Queue(maxsize=1)

def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.3)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    # path_array = [
    # (738, 699),
    # (990, 704),
    # (1090, 627),
    # (1082, 480),
    # (890, 460),
    # (842, 369),
    # (900, 325),
    # (1030, 333),
    # (1080, 265),
    # (1080, 155),
    # (945, 150),
    # (940, 226),
    # (877, 219),
    # (877, 170),
    # (800, 165),
    # (780, 216),
    # (660, 212),
    # (663, 304),
    # (744, 316),
    # (750, 385),
    # (560, 460),
    # (580, 550),
    # (620, 660),
    # (493, 673),
    # (424, 560),
    # (422, 429),
    # (479, 371),
    # (422, 250),
    # (430, 132),
    # (558, 140),
    # (606, 57),
    # (830, 60)
    # ] 
    path_array = [
        (949, 701), (943, 589), (938, 478), (939, 363), (1027, 316),
        (1090, 245), (1069, 150), (974, 178), (896, 234), (825, 234), (825, 154),
        (736, 154), (734, 263), (772, 349), (825, 430), (843, 540),
        (850, 690), (770, 690), (747, 581), (640, 561), (560, 640),
        (420, 629), (420, 538), (530, 479), (650, 425), (650, 321),
        (583, 243), (552, 143), (552, 49), (687, 49), (763, 49)
    ]
    pathFollower = path_following.PathFollower(path_array, controller)
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)

    last_frame_time = time.time()
    fps = 0.0
    frame_warned = False
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                if not frame_warned:
                    print("[WARN] Tracker returned empty frame. Waiting...")
                    frame_warned = True
                time.sleep(0.015)
                continue

            now = time.time()
            dt = now - last_frame_time
            if dt > 0.00001:
                fps = 1.0 / dt
            last_frame_time = now
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            ball_pos = tracker.get_position()

            #if not ball_pos:
            #    print("No ball found (run_controller)")
            #    continue

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cv2.circle(frame, ball_pos, 8, (255, 165, 0), -1)
                cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
            else:
                controller.arduinoThread.send_target_positions(0, 0)

            for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(frame, path_array[i], 5, (0, 0, 255), -1)

            #cv2.imshow("Ball & Marker Tracking", frame)
            if not frame_queue.full():
                print(frame.size)
                frame_queue.put_nowait(frame.copy())

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_target_positions(0, 0)
                cv2.destroyAllWindows()
                break

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