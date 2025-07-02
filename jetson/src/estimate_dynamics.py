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
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            ori = tracker.get_orientation()

            if abs(ori[1]-np.deg2rad(1.5)) > 0.0015 and abs(ori[0]) > 0.0015 and not reached:
                controller.axisControl((np.deg2rad(1.5), 0))
                reached = True

            #print(ori)

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