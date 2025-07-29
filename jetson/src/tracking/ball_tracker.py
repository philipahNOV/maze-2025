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


def distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def box_center(box):
    """Get center point of a bounding box"""
    x1, y1, x2, y2 = box
    return ((x1 + x2) // 2, (y1 + y2) // 2)


class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.pt"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        self.iou_threshold = tracking_config.get("box_iou_threshold", 0.4)
        self.max_distance_threshold = 200  # Maximum pixel distance for tracking continuity
        self.position_weight = 0.7  # Weight for position-based tracking vs IoU
        
        self.ball_position = None
        self.locked_box = None
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 1
        self.lost_frames_counter = 0
        self.max_lost_frames = 10  # Allow some frames without detection
        
        # Elevator area detection (to avoid false positives in elevator hole)
        # Get camera config from global config structure
        full_config = tracking_config.get("full_config", {})
        camera_config = full_config.get("camera", {})
        self.elevator_center = (camera_config.get("elevator_center_x", 998), 
                               camera_config.get("elevator_center_y", 588))
        self.elevator_radius = camera_config.get("elevator_radius", 60)
        
        print(f"[BallTracker] Elevator area: center={self.elevator_center}, radius={self.elevator_radius}")

        self.running = False
        self.lock = threading.Lock()

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def is_in_elevator_area(self, point):
        """Check if a point is within the elevator area"""
        dist = distance(point, self.elevator_center)
        return dist <= self.elevator_radius

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

            # Collect all valid ball detections
            valid_detections = []
            
            for box in results.boxes:
                label = self.model.get_label(box.cls[0])
                conf = float(box.conf[0])
                if label != "ball" or conf < 0.6:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get center of mass inside YOLO box
                roi = gray[y1:y2, x1:x2]
                _, thresh = cv2.threshold(roi, 30, 255, cv2.THRESH_BINARY)
                local_com = get_center_of_mass(thresh)

                if local_com:
                    cx, cy = local_com
                    global_com = (x1 + cx, y1 + cy)
                    box_center_point = box_center((x1, y1, x2, y2))
                    
                    valid_detections.append({
                        'position': global_com,
                        'box': (x1, y1, x2, y2),
                        'box_center': box_center_point,
                        'confidence': conf,
                        'in_elevator': self.is_in_elevator_area(global_com)
                    })

            # Choose best detection based on tracking state
            best_detection = None
            
            if not self.initialized:
                # During initialization, prefer detections in the init region
                for det in valid_detections:
                    x, y = det['position']
                    if (self.INIT_BALL_REGION[0][0] <= x <= self.INIT_BALL_REGION[1][0] and 
                        self.INIT_BALL_REGION[0][1] <= y <= self.INIT_BALL_REGION[1][1]):
                        if best_detection is None or det['confidence'] > best_detection['confidence']:
                            best_detection = det
                            
                # If we have a valid detection in init region
                if best_detection:
                    self.ball_position = best_detection['position']
                    self.locked_box = best_detection['box']
                    self.ball_confirm_counter += 1
                    if self.ball_confirm_counter >= self.ball_confirm_threshold:
                        self.initialized = True
                        print(f"[BallTracker] Initialized at position {self.ball_position}")
                        
            else:
                # Already initialized - use smart tracking logic
                if self.ball_position and valid_detections:
                    # Score each detection based on multiple factors
                    scored_detections = []
                    
                    for det in valid_detections:
                        score = 0
                        
                        # 1. Distance from last known position (closer is better)
                        pos_distance = distance(det['position'], self.ball_position)
                        if pos_distance <= self.max_distance_threshold:
                            score += (self.max_distance_threshold - pos_distance) / self.max_distance_threshold * 0.4
                        
                        # 2. Confidence score
                        score += det['confidence'] * 0.3
                        
                        # 3. IoU with locked box (if available)
                        if self.locked_box:
                            iou_score = iou(det['box'], self.locked_box)
                            score += iou_score * 0.2
                        
                        # 4. Penalty for being in elevator area (unless that's where we last saw it)
                        if det['in_elevator'] and not self.is_in_elevator_area(self.ball_position):
                            score -= 0.3  # Penalize elevator detections when ball was outside
                            # print(f"[BallTracker] Penalizing elevator detection at {det['position']}")
                        
                        # 5. Bonus for being outside elevator when we expect movement
                        if not det['in_elevator'] and self.is_in_elevator_area(self.ball_position):
                            score += 0.2  # Bonus for ball moving out of elevator
                            # print(f"[BallTracker] Bonus for ball moving out of elevator to {det['position']}")
                            
                        scored_detections.append((score, det))
                    
                    # Choose detection with highest score
                    if scored_detections:
                        scored_detections.sort(key=lambda x: x[0], reverse=True)
                        best_score, best_detection = scored_detections[0]
                        
                        # Only accept if score is reasonable
                        if best_score > 0.3:
                            self.ball_position = best_detection['position']
                            self.locked_box = best_detection['box']
                            self.lost_frames_counter = 0
                        else:
                            self.lost_frames_counter += 1
                    else:
                        self.lost_frames_counter += 1
                else:
                    self.lost_frames_counter += 1

                # Reset tracking if lost for too long
                if self.lost_frames_counter > self.max_lost_frames:
                    print(f"[BallTracker] Lost ball for {self.lost_frames_counter} frames, resetting...")
                    self.ball_position = None
                    self.locked_box = None
                    self.lost_frames_counter = 0
                    # Don't reset initialized flag - keep trying to reacquire

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
        self.ball_position = None
        self.lost_frames_counter = 0
        print("[BallTracker] Retracking initiated - resetting all tracking state.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None