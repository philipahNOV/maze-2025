import sys
import cv2
import numpy as np
from astar_test import astar

# ZED SDK imports
import pyzed.sl as sl

def get_board(img):
    """
    Applies image processing techniques to the input image and returns a single-channel mask.
    Walls are black (0), free space white (255).
    """
    h, w = img.shape[:2]
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, 5)

    lb, ub = 0, 60
    mask = np.full_like(blurred, 255)
    mask[(blurred >= lb) & (blurred <= ub)] = 0
    return mask

def draw_path(frame, path, color=(0,0,255), thickness=2):
    """Overlay the A* path on the BGR frame."""
    for i in range(len(path)-1):
        cv2.line(frame, path[i], path[i+1], color, thickness)
    for p in path:
        cv2.circle(frame, p, 4, color, -1)

def main():
    # --- 1) Create and open ZED camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # we only need RGB
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"ZED Open failed: {status}")
        sys.exit(1)

    # Prepare containers
    left_image = sl.Mat()

    # --- 2) Grab first frame for path planning
    print("[INFO] Capturing initial frame...")
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            frame0 = left_image.get_data()  # numpy array BGR
            break

    h, w = frame0.shape[:2]

    # --- 3) Define start & goal (pixel coords)
    # Adjust these to your scene or compute interactively.
    start = (50, h - 50)
    goal  = (w - 50, 50)

    # --- 4) Build occupancy grid & run A*
    mask0 = get_board(frame0)
    grid  = (mask0 == 0).astype(np.uint8)

    path = astar(start, goal, grid)
    if not path:
        print("A* failed to find a path")
        zed.close()
        sys.exit(1)
    print(f"A* found path with {len(path)} waypoints")

    # --- 5) Live display loop ---
    window_name = "ZED Live Path Planner"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)
    print("[INFO] Press 'q' to quit.")

    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(left_image, sl.VIEW.LEFT)
        frame = left_image.get_data()

        # overlay path
        draw_path(frame, path)

        # optional: show mask side-by-side
        mask_color = cv2.cvtColor(mask0, cv2.COLOR_GRAY2BGR)
        combined   = np.hstack((frame, mask_color))

        cv2.imshow(window_name, combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # cleanup
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()
