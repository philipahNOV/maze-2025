import threading
import time
import numpy as np
from vision_utils import hsv_tracking, global_hsv_search
from model_loader import YOLOModel

class BallTracker:
    def __init__(self, camera, model_path="best.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.ball_position = None
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1

        self.hsv_fail_counter = 0
        self.hsv_fail_threshold = 5
        self.yolo_cooldown = 0
        self.yolo_cooldown_period = 15

        self.running = False
        self.lock = threading.Lock()

        self.WINDOW_SIZE = 80
        self.INIT_BALL_REGION = ((390, 10), (1120, 720))
        self.HSV_RANGE = (np.array([35, 80, 80]), np.array([85, 255, 255])) # Green

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None
        self.yolo_result = None

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
                new_pos = hsv_tracking(bgr, self.ball_position, *self.HSV_RANGE, self.WINDOW_SIZE)
                if new_pos:
                    self.ball_position = new_pos
                    self.hsv_fail_counter = 0
                else:
                    self.hsv_fail_counter += 1
                    if self.hsv_fail_counter >= self.hsv_fail_threshold and self.yolo_cooldown == 0:
                        global_pos = global_hsv_search(bgr, *self.HSV_RANGE)
                        if global_pos:
                            results = self.model.predict(rgb)
                            for box in results.boxes:
                                label = self.model.get_label(box.cls[0])
                                if label == "ball":
                                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                    self.ball_position = (cx, cy)
                                    self.hsv_fail_counter = 0
                                    self.yolo_cooldown = self.yolo_cooldown_period
                                    break
                        else:
                            self.ball_position = None
            if self.yolo_cooldown > 0:
                self.yolo_cooldown -= 1

            time.sleep(0.005)

    def start(self):
        self.running = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None