import cv2

def draw_path(image, waypoints, start, goal):
    out = image.copy()

    # VERY important for handling different image formats
    if len(out.shape) == 2 or out.shape[2] == 1:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    elif out.shape[2] == 4:
        out = cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)

    h, w = out.shape[:2]

    for x, y in waypoints or []:
        if 0 <= y < h and 0 <= x < w:
            cv2.circle(out, (x, y), 2, (0, 255, 0), -1)

    if start:
        cv2.circle(out, (start[1], start[0]), 5, (0, 255, 0), -1)
    if goal:
        cv2.circle(out, (goal[1], goal[0]), 5, (255, 0, 0), -1)

    return out