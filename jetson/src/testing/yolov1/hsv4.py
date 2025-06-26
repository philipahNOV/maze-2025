import cv2
import numpy as np
from ultralytics import YOLO
import pyzed.sl as sl
import threading
import time
from zed_main import ZEDCamera

class BallTracker:
    def __init__(self, model_path="best.pt"):
        self.zed = ZEDCamera()
        self.zed = None
        self.frame = None
        self.latest_frame = None
        self.lock = threading.Lock()

        self.tracked_objects = {
            "ball": {"position": None},
            "marker_1": {"position": None},
            "marker_2": {"position": None},
            "marker_3": {"position": None},
            "marker_4": {"position": None},
        }

        self.HSV_RANGES = {
            "ball": (np.array([35, 80, 80]), np.array([85, 255, 255])), # green
            # "ball": (np.array([0, 100, 50]), np.array([10, 255, 180])), # Red
            "marker": (np.array([0, 100, 100]), np.array([10, 255, 255])),
        }

        self.INIT_BALL_REGION = ((390, 10), (1120, 720))

        self.model = YOLO(model_path)
        self.WINDOW_SIZE = 50
        self.running = False
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def producer_loop(self):
        while self.running:
            frame = self.zed_cam.grab_frame()   # single BGR image
            if frame is not None:
                with self.lock:
                    self.latest_bgr_frame = frame
            time.sleep(0.001)

    def get_center_of_mass(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    def hsv_tracking(self, frame, prev_pos, hsv_lower, hsv_upper):
        h, w = frame.shape[:2]
        if prev_pos is None:
            return None
        x, y = prev_pos
        x_min = max(0, x - self.WINDOW_SIZE)
        x_max = min(w, x + self.WINDOW_SIZE)
        y_min = max(0, y - self.WINDOW_SIZE)
        y_max = min(h, y + self.WINDOW_SIZE)
        roi = frame[y_min:y_max, x_min:x_max]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        local_center = self.get_center_of_mass(mask)
        if local_center:
            cx, cy = local_center
            return (x_min + cx, y_min + cy)
        return None

    def consumer_loop(self):
        while self.running:
            with self.lock:
                bgr = None if self.latest_bgr_frame is None else self.latest_bgr_frame.copy()
            if bgr is None:
                time.sleep(0.01)
                continue

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


            if not self.initialized:
                results = self.model.predict(source=rgb, conf=0.6)[0]
                for box in results.boxes:
                    cls = int(box.cls[0])
                    label = self.model.names[cls]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    if label == "ball":
                        if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and self.INIT_BALL_REGION[0][1] <= cy <= self.INIT_BALL_REGION[1][1]:
                            self.ball_confirm_counter += 1
                            self.tracked_objects["ball"]["position"] = (cx, cy)
                            if self.ball_confirm_counter >= self.ball_confirm_threshold:
                                self.initialized = True



                    elif label.startswith("marker"):
                        self.tracked_objects[label]["position"] = (cx, cy)
            else:
                # use HSV tracking after ball has been initialized
                for label in self.tracked_objects:
                    hsv_lower, hsv_upper = self.HSV_RANGES["ball" if "ball" in label else "marker"]
                    prev_pos = self.tracked_objects[label]["position"]
                    new_pos = self.hsv_tracking(bgr, prev_pos, hsv_lower, hsv_upper)
                    if new_pos:
                        self.tracked_objects[label]["position"] = new_pos

            self.frame = bgr
            time.sleep(0.005)


    def start(self):
        # 1) Open ZED
        try:
            self.zed = self.zed_cam.init_camera()
        except RuntimeError as e:
            raise RuntimeError(f"Cannot open ZED: {e}")

        self.running = True

        # 2) Launch threads
        self.producer_thread = threading.Thread(target=self.producer_loop, daemon=True)
        self.consumer_thread = threading.Thread(target=self.consumer_loop, daemon=True)
        self.producer_thread.start()
        self.consumer_thread.start()


    def stop(self):
        self.running = False
        if self.zed:
            self.zed.close()

    def get_position(self):
        return self.tracked_objects["ball"]["position"]
