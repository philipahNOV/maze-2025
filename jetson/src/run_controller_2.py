import testing.yolov1.hsv3 as tracking
import time
import cv2
import positionController_2
import arduino_connection_test
import lowPassFilter
import path_following

def main():

    def initialize_component(component, name, retries=5, delay=2):
        for attempt in range(retries):
            try:
                comp_instance = component()
                print(f"{name} initialized on attempt {attempt + 1}")
                return comp_instance
            except Exception as e:
                print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
                time.sleep(delay)
        raise Exception(f"Failed to initialize {name} after {retries} attempts")

    try:
        arduino_thread = initialize_component(arduino_connection_test.ArduinoConnection, "ArduinoConnection")
        time.sleep(10)
    except Exception as e:
        print(e)
        exit(1)

    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    print("[INFO] Waiting for YOLO initialization...")
    while not tracker.initialized:
        time.sleep(0.1)

    print("[INFO] Tracking started. Press 'q' to quit.")

    controller = positionController_2.Controller(arduino_thread, tracker)

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

            cv2.circle(frame, ball_pos, 8, (0, 255, 0), -1)
            cv2.circle(frame, (770-150, 330-150), 5, (0, 0, 255), -1)
            cv2.circle(frame, (770+150, 330+150), 5, (0, 0, 255), -1)
            cv2.putText(frame, "Ball", (ball_pos[0]+10, ball_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Ball & Marker Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if not ball_pos:
                print("No ball found (run_controller)")
                continue

            # Use path following instead of static control
            pathFollower.follow_path(ball_pos)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Tracker stopped.")

if __name__ == "__main__":
    main()