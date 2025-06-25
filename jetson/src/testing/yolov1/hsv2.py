import numpy as np
import cv2
import pyzed.sl as sl

# BallDetector locks onto largest circular blob within HSV range
class BallDetector:
    DEFAULT_HSV_BALL = (
        (44, 90),   # hue min, max
        (40, 255),  # sat min, max
        (0, 255)    # val min, max
    )

    def __init__(self, hsv_params=None):
        self.hsv_params = hsv_params or BallDetector.DEFAULT_HSV_BALL

    def process_frame(self, frame: np.ndarray):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mn = np.array([self.hsv_params[0][0], self.hsv_params[1][0], self.hsv_params[2][0]])
        mx = np.array([self.hsv_params[0][1], self.hsv_params[1][1], self.hsv_params[2][1]])
        mask = cv2.inRange(hsv, mn, mx)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, mask
        cnt = max(contours, key=cv2.contourArea)
        (x, y), r = cv2.minEnclosingCircle(cnt)
        return (int(x), int(y)), int(r), mask

    def draw(self, frame, center, radius):
        if center and radius:
            cv2.circle(frame, center, radius, (0,0,255), 2)

# ZED camera setup

def init_zed_camera():
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD720
    params.camera_fps = 30
    params.coordinate_units = sl.UNIT.MILLIMETER
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera.")
        exit(1)
    return zed

# Grab left image from ZED

def grab_zed_frame(zed):
    mat = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None
    zed.retrieve_image(mat, sl.VIEW.LEFT)
    return mat.get_data()

# Main application

def main():
    zed = init_zed_camera()
    detector = BallDetector()

    while True:
        frame = grab_zed_frame(zed)
        if frame is None:
            break
        center, radius, mask = detector.process_frame(frame)
        detector.draw(frame, center, radius)

        cv2.imshow('Mask', mask)
        cv2.imshow('Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
