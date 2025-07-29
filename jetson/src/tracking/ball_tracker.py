import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
from tracking.vision_utils import get_center_of_mass

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

class NanoTracker:
    """Lightweight tracking implementation based on template matching"""
    def __init__(self, template_size=80):
        self.template = None
        self.last_position = None
        self.template_size = template_size
        self.search_radius = 100
        self.confidence_threshold = 0.3
        
    def init(self, frame, bbox):
        """Initialize tracker with bounding box (x, y, w, h)"""
        x, y, w, h = [int(v) for v in bbox]
        h_frame, w_frame = frame.shape[:2]
        
        # Ensure bbox is within frame bounds
        x = max(0, min(x, w_frame - w))
        y = max(0, min(y, h_frame - h))
        w = min(w, w_frame - x)
        h = min(h, h_frame - y)
        
        if w > 0 and h > 0:
            self.template = frame[y:y+h, x:x+w].copy()
            self.last_position = (x + w//2, y + h//2)
            return True
        return False
    
    def update(self, frame):
        """Update tracker and return (success, bbox)"""
        if self.template is None or self.last_position is None:
            return False, None
            
        x_center, y_center = self.last_position
        h_frame, w_frame = frame.shape[:2]
        
        # Define search region
        search_x1 = max(0, x_center - self.search_radius)
        search_y1 = max(0, y_center - self.search_radius)
        search_x2 = min(w_frame, x_center + self.search_radius)
        search_y2 = min(h_frame, y_center + self.search_radius)
        
        search_region = frame[search_y1:search_y2, search_x1:search_x2]
        if search_region.size == 0:
            return False, None
            
        # Template matching
        try:
            result = cv2.matchTemplate(search_region, self.template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            
            if max_val > self.confidence_threshold:
                # Convert local coordinates to global
                match_x = search_x1 + max_loc[0]
                match_y = search_y1 + max_loc[1]
                
                # Update position to center of template
                h_template, w_template = self.template.shape[:2]
                new_center = (match_x + w_template//2, match_y + h_template//2)
                
                # Validate reasonable movement
                if distance(new_center, self.last_position) < 150:
                    self.last_position = new_center
                    bbox = (match_x, match_y, w_template, h_template)
                    return True, bbox
                    
        except Exception as e:
            print(f"[NanoTracker] Template matching failed: {e}")
            
        return False, None

class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.onnx"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        
        # Core tracking state
        self.ball_position = None
        self.initialized = False
        self.lost_frames_counter = 0
        self.max_lost_frames = 60  # 1 second at 60fps
        
        # YOLO detection state
        self.last_yolo_result = None
        self.last_yolo_time = 0
        self.yolo_result_lock = threading.Lock()
        
        # NanoTracker for fast tracking
        self.nano_tracker = NanoTracker(template_size=60)
        
        # Elevator area detection
        full_config = tracking_config.get("full_config", {})
        camera_config = full_config.get("camera", {})
        self.elevator_center = (camera_config.get("elevator_center_x", 998),
                                camera_config.get("elevator_center_y", 588))
        self.elevator_radius = camera_config.get("elevator_radius", 60)

        # YOLO optimization
        self.resized_input = True
        self.yolo_input_size = (640, 360)

        # Threading
        self.running = False
        self.lock = threading.Lock()
        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def is_in_elevator_area(self, point):
        return distance(point, self.elevator_center) <= self.elevator_radius

    def init_nano_tracker(self, frame, position):
        """Initialize NanoTracker with ball position"""
        x, y = position
        size = 60  # Template size
        half = size // 2
        bbox = (x - half, y - half, size, size)
        
        if self.nano_tracker.init(frame, bbox):
            print(f"[BallTracker] NanoTracker initialized at {position}")
            return True
        return False

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

            # Run YOLO detection
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
            best_score = 0
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

            for box in results.boxes:
                label = self.model.get_label(box.cls[0])
                conf = float(box.conf[0])
                if label != "ball" or conf < 0.3:  # Lower threshold for more detections
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                if self.resized_input:
                    x1 = int(x1 * scale_x)
                    x2 = int(x2 * scale_x)
                    y1 = int(y1 * scale_y)
                    y2 = int(y2 * scale_y)

                # Get center of mass for more accurate position
                roi = gray[y1:y2, x1:x2]
                if roi.size == 0 or cv2.mean(roi)[0] < 40:  # Lower intensity threshold
                    continue

                _, thresh = cv2.threshold(roi, 25, 255, cv2.THRESH_BINARY)  # Lower threshold
                com = get_center_of_mass(thresh)
                if not com:
                    continue
                    
                cx, cy = x1 + com[0], y1 + com[1]

                # Skip detections in elevator area (ball is in the hole)
                if self.is_in_elevator_area((cx, cy)):
                    continue

                # Score based on confidence and distance from last known position
                score = conf
                if self.ball_position:
                    dist = distance((cx, cy), self.ball_position)
                    if dist < 250:  # More tolerant distance
                        score += (250 - dist) / 250
                
                if score > best_score:
                    best_score = score
                    best_detection = (cx, cy)

            # Update YOLO result and initialize NanoTracker
            with self.yolo_result_lock:
                if best_detection:
                    self.last_yolo_result = best_detection
                    self.last_yolo_time = time.time()
                    # Initialize NanoTracker with new detection
                    self.init_nano_tracker(gray, best_detection)

            time.sleep(0.2)  # Run every 200ms for faster updates

    def consumer_loop(self):
        while self.running:
            start_time = time.perf_counter()
            
            with self.lock:
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None
            if bgr is None:
                time.sleep(0.001)
                continue

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            current_position = None
            
            # First priority: Fresh YOLO result
            with self.yolo_result_lock:
                if self.last_yolo_result and (time.time() - self.last_yolo_time < 0.8):
                    current_position = self.last_yolo_result
            
            # Second priority: NanoTracker fallback
            if current_position is None:
                success, bbox = self.nano_tracker.update(gray)
                if success and bbox:
                    x, y, w, h = bbox
                    tracker_pos = (x + w//2, y + h//2)
                    # Validate position is not in elevator area
                    if not self.is_in_elevator_area(tracker_pos):
                        current_position = tracker_pos

            # Update tracking state
            if current_position:
                self.ball_position = current_position
                self.lost_frames_counter = 0
                self.initialized = True
            else:
                self.lost_frames_counter += 1
                
                # Keep last known position for brief periods
                if self.lost_frames_counter <= 20 and self.ball_position:
                    # Use last known position during brief tracking loss
                    pass
                else:
                    # Clear position after extended loss
                    self.ball_position = None
                
                # Debug output for tracking loss
                if self.lost_frames_counter % 60 == 0:  # Every second
                    print(f"[BallTracker] Lost tracking for {self.lost_frames_counter} frames")
                
                # Reset after extended loss
                if self.lost_frames_counter > self.max_lost_frames:
                    self.retrack()

            # Maintain 60fps timing
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
        """Reset all tracking state"""
        self.initialized = False
        self.ball_position = None
        self.lost_frames_counter = 0
        self.last_yolo_result = None
        self.nano_tracker = NanoTracker(template_size=60)  # Reset NanoTracker
        print("[BallTracker] Retracking initiated - all state reset.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None