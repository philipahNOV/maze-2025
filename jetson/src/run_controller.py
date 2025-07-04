import YOLO_tracking.hsv3 as tracking
import time
import cv2
import positionController_2
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
import queue
import base64

frame_queue = queue.Queue(maxsize=1)

def send_frame_to_pi(mqtt_client: MQTTClientJetson, frame):
        scale = 0.5  # 50% of original size
        height, width = frame.shape[:2]
        new_size = (int(width * scale), int(height * scale))

        resized = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
        _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        mqtt_client.client.publish("pi/camera", jpg_as_text)


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

    last_sent_frame_time = time.time()
    frame_send_hz = 5
    TARGET_HZ = 60
    LOOP_DT = 1.0 / TARGET_HZ
    frame_warned = False
    x1, y1, x2, y2 = 390, 10, 1120, 720
    try:
        while True:
            loop_start = time.time()

            frame = tracker.frame
            if frame is None:
                if not frame_warned:
                    print("[WARN] Tracker returned empty frame. Waiting...")
                    frame_warned = True
                time.sleep(0.015)
                continue

            ball_pos = tracker.get_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                cv2.circle(frame, ball_pos, 8, (255, 165, 0), -1)
                cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
            else:
                controller.arduinoThread.send_target_positions(0, 0)
                ball_pos = smoother.update(ball_pos)

            for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(frame, path_array[i], 5, (0, 0, 255), -1)

            cropped_frame = frame[y1:y2, x1:x2]
            cropped_frame = cv2.rotate(cropped_frame, cv2.ROTATE_180)

            if time.time() > last_sent_frame_time + 1/frame_send_hz:
                send_frame_to_pi(mqtt_client, cropped_frame)
                last_sent_frame_time = time.time()

            cv2.imshow("Ball & Marker Tracking", cropped_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            #if not frame_queue.full():
                #frame_queue.put_nowait(frame.copy())

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                controller.arduinoThread.send_target_positions(0, 0)
                cv2.destroyAllWindows()
                break

            # === Maintain 60 Hz ===
            loop_duration = time.time() - loop_start
            sleep_time = LOOP_DT - loop_duration
            print(sleep_time)
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