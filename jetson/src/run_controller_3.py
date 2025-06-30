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
    (738, 699),
    (990, 704),
    (1090, 627),
    (1082, 480),
    (890, 460),
    (842, 369),
    (900, 325),
    (1030, 333),
    (1080, 265),
    (1080, 155),
    (945, 150),
    (940, 226),
    (877, 219),
    (877, 170),
    (800, 165),
    (780, 216),
    (660, 212),
    (663, 304),
    (744, 316),
    (750, 385),
    (560, 460),
    (580, 550),
    (620, 660),
    (493, 673),
    (424, 560),
    (422, 429),
    (479, 371),
    (422, 250),
    (430, 132),
    (558, 140),
    (606, 57),
    (830, 60)
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

            plot_waypoints(frame, pathFollower)  # Draw path on frame

            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            # Use path following instead of static control
            pathFollower.follow_path(ball_pos)

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            for i in range(pathFollower.next_waypoint):
                if i == pathFollower.prev_waypoint:
                    cv2.circle(frame, path_array[i], 5, (255, 102, 255), -1)
                    continue
                cv2.circle(frame, ball_pos, 5, (255, 215, 0), -1)

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