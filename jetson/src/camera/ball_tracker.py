import threading
import time
import numpy as np
from camera.vision_utils import hsv_tracking, global_hsv_search
from camera.model_loader import YOLOModel

def is_in_region(point: tuple[int, int], region: tuple[tuple[int, int], tuple[int, int]]) -> bool:
    (x1, y1), (x2, y2) = region
    cx, cy = point
    return x1 <= cx <= x2 and y1 <= cy <= y2


def get_box_center(box) -> tuple[int, int]:
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    return (x1 + x2) // 2, (y1 + y2) // 2


class BallTracker:
    def __init__(self, camera, model_path="v8-291.pt"):
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
        self.HSV_RANGE = (np.array([35, 80, 80]), np.array([85, 255, 255]))  # works for light green ball, should or potentially could be less hard coded

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
                        cx, cy = get_box_center(box)
                        if is_in_region((cx, cy), self.INIT_BALL_REGION):
                            self.ball_confirm_counter += 1
                            self.ball_position = (cx, cy)
                            if self.ball_confirm_counter >= self.ball_confirm_threshold:
                                self.initialized = True
            else:
                new_pos = hsv_tracking(
                    bgr=bgr,
                    position=self.ball_position,
                    hsv_min=self.HSV_RANGE[0],
                    hsv_max=self.HSV_RANGE[1],
                    window_size=self.WINDOW_SIZE
                )

                if new_pos:
                    self.ball_position = new_pos
                    self.hsv_fail_counter = 0
                else:
                    self.ball_position = None
                    self.hsv_fail_counter += 1

                    if self.hsv_fail_counter >= self.hsv_fail_threshold and self.yolo_cooldown == 0:
                        global_pos = global_hsv_search(
                            bgr=bgr,
                            hsv_min=self.HSV_RANGE[0],
                            hsv_max=self.HSV_RANGE[1]
                        )
                        if global_pos:
                            results = self.model.predict(rgb)
                            for box in results.boxes:
                                label = self.model.get_label(box.cls[0])
                                if label == "ball":
                                    cx, cy = get_box_center(box)
                                    self.ball_position = (cx, cy)
                                    self.hsv_fail_counter = 0
                                    self.yolo_cooldown = self.yolo_cooldown_period
                                    break

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

    def retrack(self):
        self.initialized = False
        self.ball_confirm_counter = 0
        print("[BallTracker] Retracking initiated.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None