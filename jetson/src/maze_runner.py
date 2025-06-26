import time
import cv2
import numpy as np
from astar_test import astar
import arduino_connection_test
import testing.yolov1.hsv3 as tracking
import lowPassFilter
import positionController_2
import path_following

def get_board(img):
    """
    Applies image processing techniques to the input image and returns a masked image.

    Parameters:
    img (numpy.ndarray): The input image.

    Returns:
    numpy.ndarray: The masked image.
    """
    width = int(img.shape[1])
    height = int(img.shape[0])
    dim = (width, height)

    # Resize the image
    resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    img_gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)

    # Apply median blur
    img_blurred = cv2.medianBlur(img_gray, 5)

    # Initial bounds
    lower_bound_init = 0
    upper_bound_init = 60

    # Create the mask based on the initial bounds
    mask = np.ones_like(img_blurred) * 255
    mask[(img_blurred >= lower_bound_init) & (img_blurred <= upper_bound_init)] = 0

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    return mask_bgr

def get_mm_per_pixel(img):
    """
    Returns the size of the board in the input image.

    Returns:
    list: The size of the board [mm per pixel in x, mm per pixel in y].
    """
    if img is None:
        return [0, 0]
    board_size = 280  # mm
    img_size_x = img.shape[0]  # height in pixels
    img_size_y = img.shape[1]  # width in pixels
    mm_per_pixel_x = board_size / img_size_x
    mm_per_pixel_y = board_size / img_size_y
    return [mm_per_pixel_x, mm_per_pixel_y]

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

    # --- Arduino setup ---
    try:
        arduino_thread = initialize_component(
            arduino_connection_test.ArduinoConnection, "ArduinoConnection"
        )
        time.sleep(10)
    except Exception as e:
        print(e)
        exit(1)

    # helper to draw the planned path
    def plot_waypoints(frame, pathFollower: path_following.PathFollower):
        for pt in pathFollower.path:
            cv2.circle(frame, pt, 5, (0, 0, 255), -1)

    # --- Tracker & smoother ---
    tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
    tracker.start()
    smoother = lowPassFilter.SmoothedTracker(alpha=0.5)

    # wait for YOLO to spin up
    while not tracker.initialized:
        time.sleep(0.1)

    controller = positionController_2.Controller(arduino_thread, tracker)

    # 2) define start/goal
    start = (738, 699)
    goal  = (830,  60)

    # 3) grab one frame, mask it, compute mm/pixel
    frame0 = None
    while frame0 is None:
        frame0 = tracker.frame
        time.sleep(0.05)

    # apply get_board to produce a binary-style mask
    board_mask_bgr = get_board(frame0)
    board_mask_gray = cv2.cvtColor(board_mask_bgr, cv2.COLOR_BGR2GRAY)
    # walls are the zeroed regions (== 0)
    grid = (board_mask_gray == 0).astype(np.uint8)

    # compute physical scale (mm per pixel)
    mm_per_pixel = get_mm_per_pixel(frame0)
    print(f"Scale: {mm_per_pixel[0]:.3f} mm/px (x), {mm_per_pixel[1]:.3f} mm/px (y)")

    # 4) compute path via A*
    path = astar(start, goal, grid)
    if not path:
        raise RuntimeError("A* failed to find a path")
    print(f"A* found path with {len(path)} waypoints")

    # 5) split & print into 4 sections
    n = len(path)
    for i in range(4):
        sec = path[i * n//4 : (i+1) * n//4]
        print(f" Section {i+1}: {len(sec)} points, from {sec[0]} to {sec[-1]}")

    # 6) controller & follower
    follower = path_following.PathFollower(path, controller)
    time.sleep(1)
    controller.horizontal()
    time.sleep(2)

    # 7) display loop
    win     = "Maze View"
    bin_win = "Board Mask"
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

            # live board mask view
            mask_live = get_board(frame)
            cv2.imshow(bin_win, mask_live)

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
