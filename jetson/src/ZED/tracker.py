import cv2
import numpy as np
import threading
from ultralytics import YOLO
from .camera import ZEDCamera

class BallTracker:
    def __init__(self, model_path="best.pt"):
        self.camera = ZEDCamera()
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

    def _tracking_loop(self):
        while self.running:
            frame = self.camera.grab_frame()
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
                if prev_pos:
                    roi_pos = self.hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper)
                    if roi_pos:
                        self.tracked_objects[label]["position"] = roi_pos

            self.frame = frame

    def hsv_tracking(self, frame, prev_pos, hsv_lower, hsv_upper):
        h, w = frame.shape[:2]
        x, y = prev_pos
        x_min = max(0, x - self.WINDOW_SIZE)
        x_max = min(w, x + self.WINDOW_SIZE)
        y_min = max(0, y - self.WINDOW_SIZE)
        y_max = min(h, y + self.WINDOW_SIZE)
        roi = frame[y_min:y_max, x_min:x_max]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (x_min + cx, y_min + cy)

    def start(self):
        self.camera.init_camera()
        self.camera.grab_frame()
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.camera.close()

    def get_position(self):
        return self.tracked_objects["ball"]["position"]
    
    def get_orientation(self):
        return self.camera.get_orientation()

