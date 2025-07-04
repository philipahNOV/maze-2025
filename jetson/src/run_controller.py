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

def send_frame_to_pi(mqtt_client: MQTTClientJetson, frame):
        scale = 0.5  # 50% of original size
        height, width = frame.shape[:2]
        new_size = (int(width * scale), int(height * scale))

        resized = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
        _, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        mqtt_client.client.publish("pi/camera", jpg_as_text)


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

            image_controller.frame = frame.copy()

            ball_pos = tracker.get_position()

            if ball_pos is not None:
                ball_pos = smoother.update(ball_pos)
                pathFollower.follow_path(ball_pos)
                image_controller.draw_ball(ball_pos)
            else:
                controller.arduinoThread.send_target_positions(0, 0)
                ball_pos = smoother.update(ball_pos)

            image_controller.draw_waypoints(pathFollower)

            cropped_frame = image_controller.get_fixed_frame()

            if time.time() > last_sent_frame_time + 1/frame_send_hz:
                send_frame_to_pi(mqtt_client, cropped_frame)
                last_sent_frame_time = time.time()

            cv2.imshow("Ball tracking", cropped_frame)
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