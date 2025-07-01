import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import arduino_connection_test
import lowPassFilter
import path_following
from mqtt_client import MQTTClientJetson

def main(tracker: tracking.BallTracker, controller: positionController_2.Controller, mqtt_client: MQTTClientJetson):

    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    path_array = [
    (914, 705), (939, 623), (945, 532), (887, 485), (845, 547),
    (850, 638), (814, 701), (767, 641), (732, 572), (643, 563),
    (583, 609), (499, 630), (432, 600), (470, 536), (527, 483),
    (589, 436), (627, 367), (600, 294), (563, 227), (549, 143),
    (556, 59), (647, 52), (739, 52), (827, 59), (827, 83)
    ]
    pathFollower = path_following.PathFollower(path_array, controller)
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)

    def plot_waypoints(frame, pathFollower: path_following.PathFollower):  # Draw waypoints on frame
        for n in range(pathFollower.length):
            cv2.circle(frame, pathFollower.path[n], 5, (0, 0, 255), -1)

    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            #plot_waypoints(frame, pathFollower)  # Draw path on frame

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            # Use path following instead of static control
            pathFollower.follow_path(ball_pos)

            cv2.circle(frame, ball_pos, 8, (255, 165, 0), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
            for i in range(pathFollower.length):
                if i < pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 200, 0), -1)
                    continue
                elif i == pathFollower.next_waypoint:
                    cv2.circle(frame, path_array[i], 5, (0, 255, 255), -1)
                else:
                    cv2.circle(frame, path_array[i], 5, (0, 0, 255), -1)

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