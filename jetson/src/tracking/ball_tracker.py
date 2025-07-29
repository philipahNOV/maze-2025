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
        self.lost_frames_counter = 0
        self.last_yolo_result = None
        self.last_yolo_time = 0
        self.yolo_result_lock = threading.Lock()

        self.max_lost_frames = 30  # Allow 30 frames (0.5 seconds at 60fps) before reset
        self.frame_counter = 0
        
        # Fast tracking fallback
        self.fast_tracker = None
        self.tracking_window_size = 80

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

    def init_fast_tracker(self, frame, position):
        """Initialize fast tracker for fallback tracking"""
        try:
            self.fast_tracker = cv2.TrackerCSRT_create()
            x, y = position
            half = self.tracking_window_size // 2
            bbox = (x - half, y - half, self.tracking_window_size, self.tracking_window_size)
            h, w = frame.shape[:2]
            x, y, w_box, h_box = bbox
            x = max(0, min(x, w - w_box))
            y = max(0, min(y, h - h_box))
            w_box = min(w_box, w - x)
            h_box = min(h_box, h - y)
            bbox = (x, y, w_box, h_box)
            if self.fast_tracker.init(frame, bbox):
                return True
        except Exception as e:
            print(f"[BallTracker] Failed to init fast tracker: {e}")
        self.fast_tracker = None
        return False

    def update_fast_tracker(self, frame):
        """Update fast tracker for fallback tracking"""
        if self.fast_tracker is None:
            return None
        try:
            success, bbox = self.fast_tracker.update(frame)
            if success:
                x, y, w, h = bbox
                center = (int(x + w / 2), int(y + h / 2))
                # Validate tracking result
                if self.ball_position:
                    distance_moved = distance(center, self.ball_position)
                    if distance_moved < 150:  # Reasonable movement
                        return center
                else:
                    return center
        except Exception as e:
            print(f"[BallTracker] Fast tracker update failed: {e}")
        self.fast_tracker = None
        return None

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
                    # Initialize/reinitialize fast tracker with new YOLO detection
                    self.init_fast_tracker(gray, best_detection)

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
            
            # First, try to use latest fresh YOLO result
            with self.yolo_result_lock:
                if self.last_yolo_result and (time.time() - self.last_yolo_time < 1.0):
                    if not self.is_in_elevator_area(self.last_yolo_result):
                        current_position = self.last_yolo_result

            # If no fresh YOLO result, use fast tracker as fallback
            if current_position is None and self.fast_tracker and self.ball_position:
                tracker_pos = self.update_fast_tracker(gray)
                if tracker_pos and not self.is_in_elevator_area(tracker_pos):
                    current_position = tracker_pos

            # Update ball position and lost frames counter
            if current_position:
                self.ball_position = current_position
                self.lost_frames_counter = 0
                self.initialized = True
            else:
                self.lost_frames_counter += 1
                
                # Keep last known position for brief periods
                if self.lost_frames_counter <= 15 and self.ball_position:
                    # Don't update ball_position, keep using last known position
                    pass
                
                # Print debug info occasionally
                if self.lost_frames_counter % 30 == 0:
                    print(f"[BallTracker] Lost ball for {self.lost_frames_counter} frames")
                
                # Reset if lost for too long
                if self.lost_frames_counter > self.max_lost_frames:
                    print(f"[BallTracker] Lost ball for {self.lost_frames_counter} frames, resetting...")
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
        self.fast_tracker = None  # Reset fast tracker
        print("[BallTracker] Retracking initiated - resetting all tracking state.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None