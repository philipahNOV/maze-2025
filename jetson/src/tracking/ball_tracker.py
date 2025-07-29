import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
from tracking.vision_utils import get_center_of_mass


def iou(boxA, boxB):
    ax1, ay1, ax2, ay2 = boxA
    bx1, by1, bx2, by2 = boxB

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    boxA_area = (ax2 - ax1) * (ay2 - ay1)
    boxB_area = (bx2 - bx1) * (by2 - by1)

    return inter_area / float(boxA_area + boxB_area - inter_area + 1e-5)


class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        self.iou_threshold = tracking_config.get("box_iou_threshold", 0.4)

        self.ball_position = None
        self.locked_box = None
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1

        self.running = False
        self.lock = threading.Lock()

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def producer_loop(self):
        while self.running:
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                with self.lock:
                    self.latest_rgb_frame = rgb
                    self.latest_bgr_frame = bgr
            time.sleep(0.001)

    def consumer_loop(self):
        while self.running:
            with self.lock:
                rgb = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

            if rgb is None or bgr is None:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            results = self.model.predict(rgb)

            best_com = None
            best_box = None
            best_conf = 0

            for box in results.boxes:
                label = self.model.get_label(box.cls[0])
                conf = float(box.conf[0])
                if label != "ball" or conf < 0.6:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # If already locked, compare with locked box
                if self.locked_box and self.initialized:
                    if iou((x1, y1, x2, y2), self.locked_box) < self.iou_threshold:
                        continue  # skip unrelated detection

                # Get center of mass inside YOLO box
                roi = gray[y1:y2, x1:x2]
                _, thresh = cv2.threshold(roi, 30, 255, cv2.THRESH_BINARY)
                local_com = get_center_of_mass(thresh)

                if local_com:
                    cx, cy = local_com
                    global_com = (x1 + cx, y1 + cy)
                    if conf > best_conf:
                        best_com = global_com
                        best_box = (x1, y1, x2, y2)
                        best_conf = conf

            if best_com:
                self.ball_position = best_com
                self.locked_box = best_box

                if not self.initialized:
                    self.ball_confirm_counter += 1
                    if self.ball_confirm_counter >= self.ball_confirm_threshold:
                        self.initialized = True

            time.sleep(0.005)

    def start(self):
        self.running = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def get_position(self):
        return self.ball_position

    def retrack(self):
        self.initialized = False
        self.ball_confirm_counter = 0
        self.locked_box = None
        print("[BallTracker] Retracking initiated.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None