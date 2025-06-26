import time
import cv2
import numpy as np
from astar_test import astar
import arduino_connection_test
import testing.yolov1.hsv3 as tracking
import lowPassFilter
import positionController_2
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
        arduino_thread = initialize_component(
            arduino_connection_test.ArduinoConnection, "ArduinoConnection"
        )
        time.sleep(10)
    except Exception as e:
        print(e)
        exit(1)

    def plot_waypoints(frame, pathFollower: path_following.PathFollower):
        for pt in pathFollower.path:
            cv2.circle(frame, pt, 5, (0, 0, 255), -1)

    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    # wait for YOLO
    while not tracker.initialized:
        time.sleep(0.1)

    controller = positionController_2.Controller(arduino_thread, tracker)

    # 2) define start/goal
    start = (738, 699)
    goal  = (830,  60)

    # 3) sample one frame, threshold it, and build occupancy grid
    #    (binary==0 → wall, 255→free)
    frame0 = None
    while frame0 is None:
        frame0 = tracker.frame
        time.sleep(0.05)
    gray0   = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)
    _, binary0 = cv2.threshold(gray0, 100, 255, cv2.THRESH_BINARY)
    H, W = binary0.shape
    # grid[y][x] = 1 for obstacle, 0 for free
    grid_list = [[1 if binary0[y, x] == 0 else 0 for x in range(W)] for y in range(H)]
    grid = np.array(grid_list, dtype=np.uint8)  # ← changed here

    # 4) compute path
    path = astar(start, goal, grid)
    if not path:
        raise RuntimeError("A* failed to find a path")
    print(f"A* found path with {len(path)} waypoints")

    # 5) split & print sections
    n = len(path)
    sections = [path[i*n//4:(i+1)*n//4] for i in range(4)]
    for i, sec in enumerate(sections, start=1):
        print(f" Section {i}: {len(sec)} points, from {sec[0]} to {sec[-1]}")

    # 6) controller & follower
    follower = path_following.PathFollower(path, controller)
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)

    # 7) display loop
    win     = "Maze View"
    bin_win = "Binary View"
    cv2.namedWindow(win,     cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win,     960, 540)
    cv2.namedWindow(bin_win,  cv2.WINDOW_NORMAL)
    cv2.resizeWindow(bin_win, 960, 540)

    print("[INFO] Press 'q' to quit.")
    try:
        while True:
            frame = tracker.frame
            if frame is None:
                continue

            # live binary view
            gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            cv2.imshow(bin_win, binary)

            # draw planned waypoints on color frame
            plot_waypoints(frame, follower)

            # draw and label ball
            ball_pos = tracker.get_position()
            ball_pos = smoother.update(ball_pos)
            if ball_pos:
                cv2.circle(frame, ball_pos, 8, (0,255,0), -1)
                cv2.putText(
                    frame, "Ball",
                    (ball_pos[0]+10, ball_pos[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2
                )

            cv2.imshow(win, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if ball_pos:
                follower.follow_path(ball_pos)

    finally:
        tracker.stop()
        cv2.destroyAllWindows()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
