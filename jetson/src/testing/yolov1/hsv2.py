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

    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None

    # 1) grab 4-channel RGBA
    zed.retrieve_image(
        mat,
        sl.VIEW.LEFT,
        sl.MEM.CPU,
        sl.MAT_TYPE.U8_C4
    )
    rgba = mat.get_data()  # H×W×4, [R, G, B, A]

    # 2) convert *from RGBA into BGR* (note the BGR ordering)
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

    return bgr


class BallDetector:
    DEFAULT_HSV_BALL = (
        (44, 90),     # H min, max
        (40, 255),    # S min, max
        (0, 255)      # V min, max
    )

    def __init__(self, hsv_params=None):
        self.hsv_params = hsv_params or BallDetector.DEFAULT_HSV_BALL

    def process_frame(self, frame):
        # 1. equalize V
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        v_eq = clahe.apply(v)
        hsv_eq = cv2.merge((h,s,v_eq))

        # 2. HSV mask
        (h_min,h_max),(s_min,s_max),(v_min,v_max) = self.hsv_params
        mn = np.array([h_min, s_min, v_min])
        mx = np.array([h_max, s_max, 255])
        mask_hsv = cv2.inRange(hsv_eq, mn, mx)

        # 3. Lab mask on 'a' channel
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        a   = lab[:,:,1]
        # tweak these to taste
        mask_lab = cv2.inRange(a,  40, 190)

        # 4. combine + clean
        mask = cv2.bitwise_and(mask_hsv, mask_lab)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 5. find largest circular blob
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = (None, 0, 0)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300: continue
            peri = cv2.arcLength(cnt, True)
            circ = 4*np.pi*area/(peri*peri) if peri else 0
            if circ < 0.5: continue
            (x,y),r = cv2.minEnclosingCircle(cnt)
            if area > best[2]:
                best = ((int(x),int(y)), int(r), area)

        center, radius, _ = best
        return center, radius

    def draw(self, frame, center, radius):
        if center and radius:
            cv2.circle(frame, center, radius, (0,0,255), 2)

def main():
    zed      = init_zed_camera()
    detector = BallDetector()
    tracker  = cv2.TrackerKCF_create()
    initialized = False

    while True:
        frame = grab_zed_frame(zed)
        if frame is None:
            break

        # 1) If tracker active, try that first
        if initialized:
            ok, box = tracker.update(frame)
            if ok:
                x,y,w,h = map(int, box)
                center = (x + w//2, y + h//2)
                radius = w//2
            else:
                initialized = False
                center, radius = detector.process_frame(frame)
        else:
            # 2) fallback: run detection
            center, radius = detector.process_frame(frame)
            if center:
                # init tracker on first successful detect
                x,y = center
                tracker.init(frame, (x-radius, y-radius, radius*2, radius*2))
                initialized = True

        # draw the result
        detector.draw(frame, center, radius)
        cv2.imshow('Tracking w/ MOSSE', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
