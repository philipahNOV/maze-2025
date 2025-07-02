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
    line_top_y = 100     # upper threshold line
    line_bottom_y = 600    # lower threshold line
    state = "waiting_above"
    t_entry = None
    cooldown_time = 0.5   # seconds to ignore new events after logging
    last_event_time = 0
    prev_ball_y = None
    vertical_motion_thresh = 2  # pixels/frame to avoid noise
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue
            
            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            ori = tracker.get_orientation()

            current_time = time.time()
            if ball_pos is not None and isinstance(ball_pos, tuple):
                _, ball_y = ball_pos

                if state == "waiting_above" and ball_y < line_top_y:
                    # Ball is above the top line, ready to start
                    state = "ready_to_start"

                elif state == "ready_to_start":
                    if prev_ball_y is not None:
                        dy = ball_y - prev_ball_y
                        if ball_y >= line_top_y and ball_y < line_bottom_y and dy > vertical_motion_thresh:
                            t_entry = current_time
                            print(f"[INFO] Entry detected at y={ball_y:.1f}, time={t_entry:.3f}")
                            state = "timing"

                elif state == "timing" and ball_y >= line_bottom_y:
                    # Ball exited the region
                    t_exit = current_time
                    time_elapsed = t_exit - t_entry
                    exit_ori = tracker.get_orientation()
                    ori_x_deg = exit_ori[0]
                    ori_y_deg = exit_ori[1]
                    print(f"[INFO] Exit detected at y={ball_y:.1f}, time={t_exit:.3f}")
                    print(f"[INFO] Orientation at exit: X={ori_x_deg:.2f}°, Y={ori_y_deg:.2f}°")
                    print(f"[RESULT] Time between lines: {time_elapsed:.3f} seconds")
                    last_event_time = current_time
                    state = "cooldown"

                elif state == "cooldown":
                    if current_time - last_event_time > cooldown_time and ball_y < line_top_y:
                        # Reset only after ball goes back above top line
                        state = "waiting_above"


            if abs(ori[1]-np.deg2rad(1.5)) < 0.001 and abs(ori[0]) < 0.001 and not reached:
                controller.arduinoThread.send_target_positions(0, 0)
                reached = True
            
            if not reached:
                print(f"{abs(ori[1]-np.deg2rad(1.5))}, {abs(ori[0])}")
                controller.axisControl((np.deg2rad(1.5), 0))
                time.sleep(0.015)

            prev_ball_y = tracker.get_position()[1]

            #print(ori)
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