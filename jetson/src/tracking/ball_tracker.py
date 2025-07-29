import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel


class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        self.smoothing_alpha = tracking_config.get("smoothing_alpha", 0.5)

        self.ball_position = None
        self.prev_position = None
        self.prev_gray_frame = None

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

            if not self.initialized:
                results = self.model.predict(rgb)
                best_conf = 0
                new_pos = None
                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    conf = float(box.conf[0])
                    if label == "ball" and conf > best_conf and conf > 0.6:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and \
                           self.INIT_BALL_REGION[0][1] <= cy <= self.INIT_BALL_REGION[1][1]:
                            new_pos = (cx, cy)
                            best_conf = conf

                if new_pos:
                    self.ball_position = new_pos
                    self.prev_position = np.array([[new_pos]], dtype=np.float32)
                    self.prev_gray_frame = gray
                    self.ball_confirm_counter += 1
                    if self.ball_confirm_counter >= self.ball_confirm_threshold:
                        self.initialized = True
            else:
                if self.prev_gray_frame is not None and self.prev_position is not None:
                    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                        self.prev_gray_frame, gray,
                        self.prev_position, None,
                        winSize=(15, 15),
                        maxLevel=2,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
                    )
                    if status[0][0] == 1:
                        self.ball_position = tuple(map(int, next_pts[0]))
                        self.prev_position = next_pts
                        self.prev_gray_frame = gray

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
        self.prev_position = None
        self.prev_gray_frame = None
        print("[BallTracker] Retracking initiated.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None