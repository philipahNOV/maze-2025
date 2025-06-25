import numpy as np
import cv2
import pyzed.sl as sl

# BallDetector locks onto largest circular blob within HSV range
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

# Grab left image from ZED, return BGR

def grab_zed_frame(zed):
    mat = sl.Mat()
    # retrieve U8 RGBA image
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None
    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C4)
    rgba = mat.get_data()
    # convert RGBA to BGR
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    return bgr

class BallDetector:
    """
    Detector that locks onto the largest circular blob (green ball) via HSV masking.
    """
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
        best = None
        best_score = 0
        # select most circular large blob
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # ignore small blobs
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.6:
                continue
            ((x, y), r) = cv2.minEnclosingCircle(cnt)
            score = area  # could weight by circularity
            if score > best_score:
                best_score = score
                best = ((int(x), int(y)), int(r))
        if best is None:
            return None, None
        return best

    def draw(self, frame: np.ndarray, center, radius: int):
        if center and radius:
            cv2.circle(frame, center, radius, (0,0,255), 2)

# Main application(self, frame: np.ndarray, center, radius: int):
        if center and radius:
            cv2.circle(frame, center, radius, (0,0,255), 2)

# Main application

def main():
    zed = init_zed_camera()
    detector = BallDetector()

    while True:
        frame = grab_zed_frame(zed)
        if frame is None:
            break
        center, radius = detector.process_frame(frame)
        detector.draw(frame, center, radius)
        cv2.imshow('Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
