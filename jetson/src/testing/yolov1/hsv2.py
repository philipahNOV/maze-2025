import cv2
import numpy as np
from ultralytics import YOLO
import pyzed.sl as sl
import threading

class BallTracker:
    def __init__(self, model_path="best.pt"):
        self.zed = None
        self.frame = None
        self.tracked_objects = {
            "ball": {"position": None},
            "marker_1": {"position": None},
            "marker_2": {"position": None},
            "marker_3": {"position": None},
            "marker_4": {"position": None},
        }
        self.HSV_RANGES = {
            "ball": (np.array([35, 80, 80]), np.array([85, 255, 255])),
            "marker": (np.array([0, 100, 100]), np.array([10, 255, 255])),
        }
        self.model = YOLO(model_path)
        self.WINDOW_SIZE = 50
        self.running = False
        self.initialized = False

    def init_camera(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.MILLIMETER
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("ZED camera failed to open")

    def grab_frame(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            rgba = image.get_data()
            rgb = cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return bgr
        return None

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

    def _tracking_loop(self):
        while self.running:
            frame = self.grab_frame()
            if frame is None:
                continue

            if not self.initialized:
                results = self.model.predict(source=frame, conf=0.5)[0]
                for box in results.boxes:
                    cls = int(box.cls[0])
                    label = self.model.names[cls]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    if label == "ball":
                        self.tracked_objects["ball"]["position"] = (cx, cy)
                    elif label.startswith("marker"):
                        self.tracked_objects[label]["position"] = (cx, cy)
                self.initialized = True
                continue

            for label in self.tracked_objects:
                hsv_lower, hsv_upper = self.HSV_RANGES["ball" if "ball" in label else "marker"]
                prev_pos = self.tracked_objects[label]["position"]
                new_pos = self.hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper)
                if new_pos:
                    self.tracked_objects[label]["position"] = new_pos

            self.frame = frame

    def start(self):
        self.init_camera()
        self.grab_frame()
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.running = False
        if self.zed:
            self.zed.close()

    def get_position(self):
        return self.tracked_objects["ball"]["position"]
