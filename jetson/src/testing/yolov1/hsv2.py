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
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
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

    def process_frame(self, frame):
        # 1. equalize V
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        v_eq = clahe.apply(v)
        hsv_eq = cv2.merge((h, s, v_eq))

        # 2. HSV mask
        mn = np.array([44, 40, 0])
        mx = np.array([90, 255, 255])
        mask_hsv = cv2.inRange(hsv_eq, mn, mx)

        # 3. Lab mask on 'a' channel
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        a = lab[:,:,1]
        mask_lab = cv2.inRange(a, 0, 150)

        # 4. combined mask + cleanup
        mask = cv2.bitwise_and(mask_hsv, mask_lab)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 5. find largest circular blob as before
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        best = (None,0,0)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area<500: continue
            peri = cv2.arcLength(cnt,True)
            circ = 4*np.pi*area/(peri*peri) if peri else 0
            if circ<0.6: continue
            (x,y),r = cv2.minEnclosingCircle(cnt)
            if area>best[2]:
                best = ((int(x),int(y)),int(r),area)
        center,radius,_ = best

        return center, radius


    def draw(self, frame: np.ndarray, center, radius: int):
        if center and radius:
            cv2.circle(frame, center, radius, (0,0,255), 2)

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
