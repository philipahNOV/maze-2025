import cv2
import pyzed.sl as sl
import time

def init_zed_camera():
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD720
    params.camera_fps = 30
    params.coordinate_units = sl.UNIT.MILLIMETER
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Unable to open ZED camera")
    return zed

def grab_frame(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            bgr = image.get_data()  # ZED returns BGR directly (720, 1280, 3)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return rgb, bgr
        return None

def main():
    zed = init_zed_camera()
    win_orig = "ZED Original"
    win_bin  = "ZED Binary"

    cv2.namedWindow(win_orig, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_bin,  cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_orig,  960, 540)
    cv2.resizeWindow(win_bin,   480, 270)

    print("Press 'q' in any window to quit")
    try:
        while True:
            frame = grab_frame(zed)
            if frame is None:
                time.sleep(0.01)
                continue

            # grayscale + binary threshold
            gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)

            # show both
            cv2.imshow(win_orig, frame)
            cv2.imshow(win_bin,  binary)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        zed.close()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
