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
    def __init__(self, camera, tracking_config, model_path="v8-291.onnx"):
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
        
        # Fast tracking parameters
        self.use_fast_tracking = True
        self.yolo_every_n_frames = 5  # Run YOLO every 5 frames, use fast tracking in between
        self.frame_counter = 0
        self.fast_tracker = None  # OpenCV tracker for fast tracking
        self.last_yolo_position = None
        self.tracking_window_size = 100  # Size of region around ball for fast tracking
        
        # Background subtraction for motion detection
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=False)
        self.background_learning_rate = 0.01
        
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

    def init_fast_tracker(self, frame, position):
        """Initialize OpenCV tracker for fast tracking"""
        try:
            # Use CSRT tracker - good balance of speed and accuracy
            self.fast_tracker = cv2.TrackerCSRT_create()
            
            # Create bounding box around the ball
            x, y = position
            half_size = self.tracking_window_size // 2
            bbox = (x - half_size, y - half_size, self.tracking_window_size, self.tracking_window_size)
            
            # Make sure bbox is within frame bounds
            h, w = frame.shape[:2]
            x, y, w_box, h_box = bbox
            x = max(0, min(x, w - w_box))
            y = max(0, min(y, h - h_box))
            w_box = min(w_box, w - x)
            h_box = min(h_box, h - y)
            bbox = (x, y, w_box, h_box)
            
            success = self.fast_tracker.init(frame, bbox)
            if success:
                self.last_yolo_position = position
                print(f"[BallTracker] Fast tracker initialized at {position}")
            return success
        except Exception as e:
            print(f"[BallTracker] Failed to init fast tracker: {e}")
            self.fast_tracker = None
            return False

    def update_fast_tracker(self, frame):
        """Update fast tracker and return new position"""
        if self.fast_tracker is None:
            return None
            
        try:
            success, bbox = self.fast_tracker.update(frame)
            if success:
                x, y, w, h = bbox
                center_x = int(x + w/2)
                center_y = int(y + h/2)
                return (center_x, center_y)
            else:
                return None
        except Exception as e:
            print(f"[BallTracker] Fast tracker update failed: {e}")
            self.fast_tracker = None
            return None

    def find_ball_in_region(self, gray_frame, center, search_radius=50):
        """Fast ball detection in a small region using computer vision"""
        x, y = center
        h, w = gray_frame.shape
        
        # Define search region
        x1 = max(0, x - search_radius)
        y1 = max(0, y - search_radius)
        x2 = min(w, x + search_radius)
        y2 = min(h, y + search_radius)
        
        roi = gray_frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None
            
        # Apply Gaussian blur and threshold
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (likely the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Check if contour is reasonable size for a ball
            area = cv2.contourArea(largest_contour)
            if 20 < area < 2000:  # Reasonable ball size
                # Get centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + x1
                    cy = int(M["m01"] / M["m00"]) + y1
                    return (cx, cy)
        
        return None

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
                time.sleep(0.001)  # Much faster sleep
                continue

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            self.frame_counter += 1
            
            # Update background model for motion detection
            self.background_subtractor.apply(gray, learningRate=self.background_learning_rate)
            
            current_position = None
            use_yolo_this_frame = False
            
            if not self.initialized:
                # During initialization, always use YOLO
                use_yolo_this_frame = True
            elif self.frame_counter % self.yolo_every_n_frames == 0:
                # Periodic YOLO check
                use_yolo_this_frame = True
            elif self.fast_tracker is not None and self.ball_position is not None:
                # Try fast tracking first
                fast_pos = self.update_fast_tracker(gray)
                if fast_pos is not None:
                    # Verify fast tracking result with local search
                    verified_pos = self.find_ball_in_region(gray, fast_pos, search_radius=30)
                    if verified_pos is not None:
                        current_position = verified_pos
                    else:
                        # Fast tracker failed verification, fallback to YOLO
                        use_yolo_this_frame = True
                        self.fast_tracker = None
                else:
                    # Fast tracker lost target, fallback to YOLO
                    use_yolo_this_frame = True
                    self.fast_tracker = None
            else:
                # No fast tracker available, use YOLO
                use_yolo_this_frame = True

            # YOLO-based detection (slower but more accurate)
            if use_yolo_this_frame:
                results = self.model.predict(rgb)
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
                        
                        valid_detections.append({
                            'position': global_com,
                            'box': (x1, y1, x2, y2),
                            'confidence': conf,
                            'in_elevator': self.is_in_elevator_area(global_com)
                        })

                # Process YOLO detections
                best_detection = None
                
                if not self.initialized:
                    # During initialization, prefer detections in the init region
                    for det in valid_detections:
                        x, y = det['position']
                        if (self.INIT_BALL_REGION[0][0] <= x <= self.INIT_BALL_REGION[1][0] and 
                            self.INIT_BALL_REGION[0][1] <= y <= self.INIT_BALL_REGION[1][1]):
                            if best_detection is None or det['confidence'] > best_detection['confidence']:
                                best_detection = det
                                
                    if best_detection:
                        current_position = best_detection['position']
                        self.locked_box = best_detection['box']
                        self.ball_confirm_counter += 1
                        if self.ball_confirm_counter >= self.ball_confirm_threshold:
                            self.initialized = True
                            # Initialize fast tracker
                            self.init_fast_tracker(gray, current_position)
                            print(f"[BallTracker] Initialized at position {current_position}")
                            
                else:
                    # Already initialized - use smart tracking logic
                    if self.ball_position and valid_detections:
                        scored_detections = []
                        
                        for det in valid_detections:
                            score = 0
                            
                            # Distance from last known position
                            pos_distance = distance(det['position'], self.ball_position)
                            if pos_distance <= self.max_distance_threshold:
                                score += (self.max_distance_threshold - pos_distance) / self.max_distance_threshold * 0.4
                            
                            # Confidence score
                            score += det['confidence'] * 0.3
                            
                            # IoU with locked box
                            if self.locked_box:
                                iou_score = iou(det['box'], self.locked_box)
                                score += iou_score * 0.2
                            
                            # Elevator area logic
                            if det['in_elevator'] and not self.is_in_elevator_area(self.ball_position):
                                score -= 0.3
                            elif not det['in_elevator'] and self.is_in_elevator_area(self.ball_position):
                                score += 0.2
                                
                            scored_detections.append((score, det))
                        
                        if scored_detections:
                            scored_detections.sort(key=lambda x: x[0], reverse=True)
                            best_score, best_detection = scored_detections[0]
                            
                            if best_score > 0.3:
                                current_position = best_detection['position']
                                self.locked_box = best_detection['box']
                                
                                # Reinitialize fast tracker with new YOLO position
                                self.init_fast_tracker(gray, current_position)
                                self.lost_frames_counter = 0
                            else:
                                self.lost_frames_counter += 1
                    else:
                        self.lost_frames_counter += 1

            # Update ball position if we found one
            if current_position is not None:
                self.ball_position = current_position
                self.lost_frames_counter = 0
            else:
                self.lost_frames_counter += 1

            # Reset tracking if lost for too long
            if self.lost_frames_counter > self.max_lost_frames:
                print(f"[BallTracker] Lost ball for {self.lost_frames_counter} frames, resetting...")
                self.ball_position = None
                self.locked_box = None
                self.fast_tracker = None
                self.lost_frames_counter = 0

            # Much faster sleep for real-time tracking
            time.sleep(0.001)

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
        self.fast_tracker = None
        self.last_yolo_position = None
        self.frame_counter = 0
        print("[BallTracker] Retracking initiated - resetting all tracking state including fast tracker.")

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None