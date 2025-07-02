import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import arduino_connection_test
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson
import numpy as np

def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    reached = False
     # Define horizontal lines (adjust based on resolution)
    line_top_y = 600     # upper threshold line
    line_bottom_y = 100   # lower threshold line
    crossed_state = "waiting_for_entry"
    t_entry = None
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            ori = tracker.get_orientation()

            if ball_pos is not None:
                ball_x, ball_y = ball_pos

                if crossed_state == "waiting_for_entry" and ball_y <= line_top_y:
                    t_entry = time.time()
                    print(f"[INFO] Ball crossed top line at y={ball_y}, time={t_entry:.3f}")
                    crossed_state = "waiting_for_exit"

                elif crossed_state == "waiting_for_exit" and ball_y >= line_bottom_y:
                    t_exit = time.time()
                    time_elapsed = t_exit - t_entry if t_entry else None
                    if time_elapsed is not None:
                        print(f"[INFO] Ball crossed bottom line at y={ball_y}, time={t_exit:.3f}")
                        print(f"[RESULT] Time between lines: {time_elapsed:.3f} seconds")
                    crossed_state = "done"  # or reset to "waiting_for_entry" if repeatable


            if abs(ori[1]+np.deg2rad(1.5)) < 0.002 and abs(ori[0]) < 0.002 and not reached:
                controller.arduinoThread.send_target_positions(0, 0)
                reached = True
            
            if not reached:
                print(f"{abs(ori[1]+np.deg2rad(1.5))}, {abs(ori[0])}")
                controller.axisControl((-np.deg2rad(1.5), 0))

            print(ori)
            # Draw guide lines
            cv2.line(frame, (0, line_top_y), (frame.shape[1], line_top_y), (0, 255, 0), 2)
            cv2.line(frame, (0, line_bottom_y), (frame.shape[1], line_bottom_y), (0, 0, 255), 2)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            cv2.circle(frame, ball_pos, 8, (255, 165, 0), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
            cv2.imshow("Ball & Marker Tracking", frame)

            time.sleep(0.015)

            if mqtt_client.stop_control:
                mqtt_client.stop_control = False
                return

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()