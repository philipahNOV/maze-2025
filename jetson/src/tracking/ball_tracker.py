import threading
import time
import numpy as np
from tracking.vision_utils import (
    histogram_matching_tracking, 
    template_matching_tracking, 
    optical_flow_tracking,
    adaptive_ball_detection,
    extract_ball_template,
    calculate_histogram,
    create_circular_mask
)
from tracking.model_loader import YOLOModel

class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        # Remove HSV-specific config, add new tracking config
        self.tracking_fail_threshold = tracking_config.get("tracking_fail_threshold", 5)
        self.yolo_cooldown_period = tracking_config.get("yolo_cooldown_period", 10)
        self.search_radius = tracking_config.get("search_radius", 100)
        self.template_update_interval = tracking_config.get("template_update_interval", 30)  # frames
        
        self.INIT_BALL_REGION = ((tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]), (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"]))
        self.smoothing_alpha = tracking_config.get("smoothing_alpha", 0.7)

        self.ball_position = None
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 3  # Require more confirmations for stability

        # New tracking state
        self.tracking_fail_counter = 0
        self.yolo_cooldown = 0
        self.running = False
        self.lock = threading.Lock()

        # Multi-modal tracking components
        self.ball_template = None
        self.reference_histograms = None
        self.prev_frame = None
        self.template_age = 0
        
        # Tracking method priorities
        self.tracking_methods = [
            'optical_flow',      # Fastest, best for small movements
            'histogram',         # Good for color-based tracking without HSV dependency
            'template',          # Robust for shape matching
            'adaptive_detection' # Fallback using HoughCircles
        ]

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

    def update_ball_template_and_histogram(self, frame, position):
        """Update the ball template and reference histograms"""
        if position is None:
            return
            
        x, y = position
        
        # Create a bounding box around the ball
        ball_radius = 25  # Approximate ball radius in pixels
        x1 = max(0, x - ball_radius)
        y1 = max(0, y - ball_radius)
        x2 = min(frame.shape[1], x + ball_radius)
        y2 = min(frame.shape[0], y + ball_radius)
        
        # Extract template
        self.ball_template, _ = extract_ball_template(frame, (x1, y1, x2, y2))
        
        # Extract ball region and create circular mask
        ball_roi = frame[y1:y2, x1:x2]
        if ball_roi.shape[0] > 0 and ball_roi.shape[1] > 0:
            center = (ball_roi.shape[1]//2, ball_roi.shape[0]//2)
            mask = create_circular_mask(ball_roi.shape, center, ball_radius//2)
            self.reference_histograms = calculate_histogram(ball_roi, mask)
        
        self.template_age = 0
        print(f"[BallTracker] Updated ball template and histogram at position {position}")

    def track_with_multiple_methods(self, frame):
        """Try multiple tracking methods in order of preference"""
        for method in self.tracking_methods:
            new_pos = None
            
            if method == 'optical_flow' and self.prev_frame is not None:
                new_pos = optical_flow_tracking(self.prev_frame, frame, self.ball_position)
                
            elif method == 'histogram' and self.reference_histograms is not None:
                new_pos = histogram_matching_tracking(frame, self.ball_position, 
                                                    self.reference_histograms, self.search_radius)
                
            elif method == 'template' and self.ball_template is not None:
                new_pos = template_matching_tracking(frame, self.ball_template, 
                                                   self.ball_position, self.search_radius)
                
            elif method == 'adaptive_detection':
                new_pos = adaptive_ball_detection(frame, self.ball_position, self.search_radius)
            
            if new_pos is not None:
                print(f"[BallTracker] Tracking successful with method: {method}")
                return new_pos, method
                
        return None, None

    def consumer_loop(self):
        while self.running:
            with self.lock:
                rgb = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

            if rgb is None or bgr is None:
                time.sleep(0.01)
                continue

            if not self.initialized:
                # Use YOLO for initial detection
                results = self.model.predict(rgb)
                if results.boxes is not None:
                    for box in results.boxes:
                        label = self.model.get_label(box.cls[0])
                        if label == "ball":
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                            if self.INIT_BALL_REGION[0][0] <= cx <= self.INIT_BALL_REGION[1][0] and \
                               self.INIT_BALL_REGION[0][1] <= cy <= self.INIT_BALL_REGION[1][1]:
                                self.ball_confirm_counter += 1
                                self.ball_position = (cx, cy)
                                print(f"[BallTracker] Ball detected at {cx}, {cy} (confirmation {self.ball_confirm_counter}/{self.ball_confirm_threshold})")
                                
                                if self.ball_confirm_counter >= self.ball_confirm_threshold:
                                    self.initialized = True
                                    # Initialize tracking components
                                    self.update_ball_template_and_histogram(bgr, self.ball_position)
                                    print(f"[BallTracker] Ball tracking initialized at position {self.ball_position}")
                                break
            else:
                # Use multi-modal tracking
                new_pos, successful_method = self.track_with_multiple_methods(bgr)

                if new_pos:
                    # Apply smoothing
                    if self.ball_position:
                        alpha = self.smoothing_alpha
                        x = int(alpha * new_pos[0] + (1 - alpha) * self.ball_position[0])
                        y = int(alpha * new_pos[1] + (1 - alpha) * self.ball_position[1])
                        self.ball_position = (x, y)
                    else:
                        self.ball_position = new_pos
                    
                    self.tracking_fail_counter = 0
                    self.template_age += 1
                    
                    # Periodically update template to adapt to lighting changes
                    if self.template_age >= self.template_update_interval:
                        self.update_ball_template_and_histogram(bgr, self.ball_position)
                        
                else:
                    # Tracking failed
                    self.ball_position = None
                    self.tracking_fail_counter += 1
                    print(f"[BallTracker] Tracking failed (count: {self.tracking_fail_counter}/{self.tracking_fail_threshold})")

                    # Try YOLO recovery if tracking fails for too long
                    if self.tracking_fail_counter >= self.tracking_fail_threshold and self.yolo_cooldown == 0:
                        print("[BallTracker] Attempting YOLO recovery...")
                        results = self.model.predict(rgb)
                        if results.boxes is not None:
                            for box in results.boxes:
                                label = self.model.get_label(box.cls[0])
                                if label == "ball":
                                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                    self.ball_position = (cx, cy)
                                    self.tracking_fail_counter = 0
                                    self.yolo_cooldown = self.yolo_cooldown_period
                                    
                                    # Update tracking components with new position
                                    self.update_ball_template_and_histogram(bgr, self.ball_position)
                                    print(f"[BallTracker] YOLO recovery successful at {cx}, {cy}")
                                    break

            # Store current frame for optical flow
            self.prev_frame = bgr.copy()
            
            if self.yolo_cooldown > 0:
                self.yolo_cooldown -= 1

            time.sleep(0.005)

    def start(self):
        self.running = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()
        print("[BallTracker] Multi-modal ball tracker started (no HSV dependency)")

    def stop(self):
        self.running = False

    def get_position(self):
        return self.ball_position

    def retrack(self):
        """Reset tracking and force re-initialization"""
        self.initialized = False
        self.ball_confirm_counter = 0
        self.tracking_fail_counter = 0
        self.ball_template = None
        self.reference_histograms = None
        self.prev_frame = None
        self.template_age = 0
        self.ball_position = None
        print("[BallTracker] Retracking initiated - all tracking data reset")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None
    
    def get_tracking_info(self):
        """Return current tracking status for debugging"""
        return {
            'initialized': self.initialized,
            'ball_position': self.ball_position,
            'tracking_fail_counter': self.tracking_fail_counter,
            'template_age': self.template_age,
            'has_template': self.ball_template is not None,
            'has_histograms': self.reference_histograms is not None,
            'yolo_cooldown': self.yolo_cooldown
        }