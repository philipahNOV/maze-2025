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
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1

        self.running = False
        self.lock = threading.Lock()

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], dtype=np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], dtype=np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman_initialized = False

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

            if rgb is None:
                time.sleep(0.01)
                continue

            if not self.initialized:
                results = self.model.predict(rgb)
                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    if label == "ball":
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and \
                           self.INIT_BALL_REGION[0][1] <= cy <= self.INIT_BALL_REGION[1][1]:
                            self.ball_confirm_counter += 1
                            self.ball_position = (cx, cy)
                            if self.ball_confirm_counter >= self.ball_confirm_threshold:
                                self.initialized = True
            else:
                results = self.model.predict(rgb)
                new_pos = None
                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    if label == "ball":
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        new_pos = (cx, cy)
                        break

                if new_pos:
                    measurement = np.array([[np.float32(new_pos[0])], [np.float32(new_pos[1])]])
                    if not self.kalman_initialized:
                        self.kalman.statePre[:2] = measurement
                        self.kalman.statePre[2:] = 0
                        self.kalman_initialized = True

                    prediction = self.kalman.predict()
                    corrected = self.kalman.correct(measurement)
                    self.ball_position = (int(corrected[0]), int(corrected[1]))
                else:
                    prediction = self.kalman.predict()
                    self.ball_position = (int(prediction[0]), int(prediction[1]))

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
        self.kalman_initialized = False
        print("[BallTracker] Retracking initiated.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None