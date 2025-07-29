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

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def box_center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) // 2, (y1 + y2) // 2)

class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.onnx"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        self.iou_threshold = tracking_config.get("box_iou_threshold", 0.4)
        self.max_distance_threshold = 200
        self.ball_position = None
        self.locked_box = None
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1
        self.last_yolo_result = None
        self.last_yolo_time = 0
        self.yolo_result_lock = threading.Lock()

        self.max_lost_frames = 30
        self.frame_counter = 0

        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=False)
        self.background_learning_rate = 0.01

        full_config = tracking_config.get("full_config", {})
        camera_config = full_config.get("camera", {})
        self.elevator_center = (camera_config.get("elevator_center_x", 998),
                                camera_config.get("elevator_center_y", 588))
        self.elevator_radius = camera_config.get("elevator_radius", 60)

        self.resized_input = True
        self.yolo_input_size = (640, 360)

        self.running = False
        self.lock = threading.Lock()
        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def is_in_elevator_area(self, point):
        return distance(point, self.elevator_center) <= self.elevator_radius

    def producer_loop(self):
        while self.running:
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                with self.lock:
                    self.latest_rgb_frame = rgb
                    self.latest_bgr_frame = bgr
            time.sleep(0.001)

    def yolo_loop(self):
        while self.running:
            with self.lock:
                rgb = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
            if rgb is None:
                time.sleep(0.01)
                continue

            if self.resized_input:
                original_size = (rgb.shape[1], rgb.shape[0])
                resized_rgb = cv2.resize(rgb, self.yolo_input_size)
                scale_x = original_size[0] / self.yolo_input_size[0]
                scale_y = original_size[1] / self.yolo_input_size[1]
                results = self.model.predict(resized_rgb)
            else:
                results = self.model.predict(rgb)
                scale_x, scale_y = 1.0, 1.0

            best_detection = None
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

            for box in results.boxes:
                label = self.model.get_label(box.cls[0])
                conf = float(box.conf[0])
                if label != "ball" or conf < 0.4:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x1 = int(x1 * scale_x)
                x2 = int(x2 * scale_x)
                y1 = int(y1 * scale_y)
                y2 = int(y2 * scale_y)

                roi = gray[y1:y2, x1:x2]
                if roi.size == 0 or cv2.mean(roi)[0] < 50:
                    continue

                _, thresh = cv2.threshold(roi, 30, 255, cv2.THRESH_BINARY)
                com = get_center_of_mass(thresh)
                if not com:
                    continue
                cx, cy = x1 + com[0], y1 + com[1]

                best_detection = (cx, cy)
                break

            with self.yolo_result_lock:
                if best_detection:
                    self.last_yolo_result = best_detection
                    self.last_yolo_time = time.time()

            time.sleep(0.5)  # Run every 500ms

    def consumer_loop(self):
        while self.running:
            start_time = time.perf_counter()
            with self.lock:
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None
            if bgr is None:
                time.sleep(0.001)
                continue

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            self.background_subtractor.apply(gray, learningRate=self.background_learning_rate)

            current_position = None
            # Try to use latest YOLO result
            with self.yolo_result_lock:
                if self.last_yolo_result and (time.time() - self.last_yolo_time < 1.0):
                    current_position = self.last_yolo_result

            if current_position:
                self.ball_position = current_position
                self.lost_frames_counter = 0
            else:
                self.lost_frames_counter += 1
                if self.lost_frames_counter > self.max_lost_frames:
                    self.retrack()

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0, (1 / 60.0) - elapsed)
            time.sleep(sleep_time)

    def start(self):
        self.running = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.yolo_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def get_position(self):
        return self.ball_position

    def retrack(self):
        self.initialized = False
        self.ball_confirm_counter = 0
        self.locked_box = None
        self.ball_position = None
        self.lost_frames_counter = 0
        self.last_yolo_result = None
        self.frame_counter = 0
        print("[BallTracker] Retracking initiated.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None