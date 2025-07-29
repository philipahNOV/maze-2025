import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
from tracking.vision_utils import get_center_of_mass


class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        self.smoothing_alpha = tracking_config.get("smoothing_alpha", 0.5)
        self.yolo_interval = tracking_config.get("yolo_frame_interval", 5)

        self.ball_position = None
        self.locked_box = None  # Stores (x1, y1, x2, y2) of tracked ball
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1

        self.frame_counter = 0

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
            self.frame_counter += 1

            run_yolo = self.frame_counter % self.yolo_interval == 0 or not self.initialized

            if run_yolo:
                results = self.model.predict(rgb)
                best_box = None
                best_conf = 0
                best_com = None

                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    conf = float(box.conf[0])
                    if label != "ball" or conf < 0.6:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                    # If we're locking, check similarity to last box
                    if self.locked_box and self.initialized:
                        lx1, ly1, lx2, ly2 = self.locked_box
                        dist = np.hypot((x1 + x2) / 2 - (lx1 + lx2) / 2, (y1 + y2) / 2 - (ly1 + ly2) / 2)
                        size_diff = abs((x2 - x1) * (y2 - y1) - (lx2 - lx1) * (ly2 - ly1))

                        if dist > 40 or size_diff > 500:  # adjust thresholds
                            continue  # not same object

                    # Get COM in cropped grayscale box
                    roi = gray[y1:y2, x1:x2]
                    _, thresh = cv2.threshold(roi, 30, 255, cv2.THRESH_BINARY)
                    local_com = get_center_of_mass(thresh)

                    if local_com:
                        com_x, com_y = local_com
                        global_com = (x1 + com_x, y1 + com_y)
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
            # Else: hold last known ball_position (no change)
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