import threading
import time
import numpy as np
import cv2
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
from tracking.model_loader import YOLOModel
from tracking.vision_utils import get_center_of_mass

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

@dataclass
class BallTrack:
    """Track for a single ball with position history and prediction"""
    positions: deque = field(default_factory=lambda: deque(maxlen=100))  # (x, y, frame_number)
    prediction: Tuple[float, float] = (0, 0)  # Predicted next position
    last_frame: int = 0
    start_frame: int = 0
    track_id: int = 0
    confidence_history: deque = field(default_factory=lambda: deque(maxlen=10))
    disappeared_count: int = 0
    
    def update(self, position: Tuple[float, float], frame_number: int, confidence: float = 1.0):
        """Update track with new position"""
        self.positions.append((position[0], position[1], frame_number))
        self.last_frame = frame_number
        self.confidence_history.append(confidence)
        self.disappeared_count = 0
        
        # Update prediction based on velocity
        if len(self.positions) >= 2:
            curr_pos = self.positions[-1]
            prev_pos = self.positions[-2]
            dt = curr_pos[2] - prev_pos[2]
            if dt > 0:
                vx = (curr_pos[0] - prev_pos[0]) / dt
                vy = (curr_pos[1] - prev_pos[1]) / dt
                self.prediction = (curr_pos[0] + vx, curr_pos[1] + vy)
            else:
                self.prediction = (curr_pos[0], curr_pos[1])
        else:
            self.prediction = position
    
    def get_stability_score(self) -> float:
        """Calculate stability score based on position consistency"""
        if len(self.positions) < 3:
            return 0.0
            
        # Calculate velocity variations
        velocities = []
        for i in range(1, len(self.positions)):
            curr = self.positions[i]
            prev = self.positions[i-1]
            dt = curr[2] - prev[2]
            if dt > 0:
                vx = (curr[0] - prev[0]) / dt
                vy = (curr[1] - prev[1]) / dt
                velocities.append((vx, vy))
        
        if not velocities:
            return 0.0
            
        # Lower variance = higher stability
        var_x = np.var([v[0] for v in velocities])
        var_y = np.var([v[1] for v in velocities])
        stability = 1.0 / (var_x + var_y + 1e-6)
        
        # Weight by track length and confidence
        length_weight = np.log(len(self.positions) + 1)
        avg_confidence = np.mean(list(self.confidence_history)) if self.confidence_history else 0.5
        
        return stability * length_weight * avg_confidence
    
    def get_current_position(self) -> Optional[Tuple[float, float]]:
        """Get current position or prediction"""
        if self.positions:
            return (self.positions[-1][0], self.positions[-1][1])
        return None

class BallTracker:
    def __init__(self, camera, tracking_config, model_path="v8-291.onnx"):
        self.camera = camera
        self.model = YOLOModel(model_path)

        # Region for initial ball detection
        self.INIT_BALL_REGION = (
            (tracking_config["init_ball_region"]["x_min"], tracking_config["init_ball_region"]["y_min"]),
            (tracking_config["init_ball_region"]["x_max"], tracking_config["init_ball_region"]["y_max"])
        )
        
        # Track management - inspired by volleyball.py
        self.tracks: Dict[int, BallTrack] = {}
        self.next_track_id = 0
        self.max_disappeared = 30  # Frames before deleting track
        self.max_match_distance = 150  # Pixels for track-detection matching
        self.frame_counter = 0
        
        # Current state
        self.main_ball_id: Optional[int] = None
        self.ball_position: Optional[Tuple[float, float]] = None
        self.initialized = False
        
        # YOLO settings
        self.yolo_every_n_frames = 5
        self.resized_input = True
        self.yolo_input_size = (640, 640)
        
        # Fast tracking fallback
        self.fast_tracker = None
        self.tracking_window_size = 100
        
        # Elevator area detection
        self.elevator_center = (998, 588)
        self.elevator_radius = 60

        # Threading
        self.running = False
        self.lock = threading.Lock()
        self.latest_rgb_frame = None
        self.latest_bgr_frame = None
        self.latest_gray_frame = None  # Add this missing attribute

    def is_in_elevator_area(self, point):
        """Check if point is in elevator area"""
        return distance(point, self.elevator_center) <= self.elevator_radius
    
    def box_to_position(self, box_data):
        """Convert YOLO detection to position and confidence"""
        x1, y1, x2, y2 = box_data['box']
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return (center_x, center_y), box_data['confidence']
    
    def update_tracks(self, detections, frame_number):
        """Update tracks with new detections using volleyball.py approach"""
        # Prioritize init region detections during startup
        detections = self._prioritize_init_detections(detections)
        
        # Remove old tracks that haven't been updated
        tracks_to_remove = []
        for track_id, track in self.tracks.items():
            track.disappeared_count += 1
            if track.disappeared_count > self.max_disappeared:
                tracks_to_remove.append(track_id)
                
        for track_id in tracks_to_remove:
            print(f"[BallTracker] Removing stale track {track_id}")
            del self.tracks[track_id]
            if self.main_ball_id == track_id:
                self.main_ball_id = None
        
        if not detections:
            return
            
        # Create distance matrix between tracks and detections
        active_tracks = list(self.tracks.items())
        if not active_tracks:
            # No existing tracks, create new ones
            # During initialization, only create tracks for balls in init region
            for det in detections:
                position, confidence = self.box_to_position(det)
                if not self.is_in_elevator_area(position):
                    if not self.initialized:
                        # Only create track if in init region during startup
                        if self._is_in_init_region(position):
                            self._create_track(position, frame_number, confidence)
                            break  # Only create one track during init
                    else:
                        # After initialization, create tracks anywhere
                        self._create_track(position, frame_number, confidence)
            return
            
        # Calculate distances
        distance_matrix = np.full((len(active_tracks), len(detections)), np.inf)
        for i, (track_id, track) in enumerate(active_tracks):
            track_pos = track.get_current_position()
            if track_pos:
                for j, det in enumerate(detections):
                    det_pos, _ = self.box_to_position(det)
                    if not self.is_in_elevator_area(det_pos):
                        distance_matrix[i, j] = distance(track_pos, det_pos)
        
        # Greedy matching - assign detections to closest tracks
        used_detections = set()
        for _ in range(min(len(active_tracks), len(detections))):
            if np.all(np.isinf(distance_matrix)):
                break
                
            min_dist = np.min(distance_matrix)
            if min_dist > self.max_match_distance:
                break
                
            i, j = np.unravel_index(np.argmin(distance_matrix), distance_matrix.shape)
            track_id, track = active_tracks[i]
            detection = detections[j]
            
            position, confidence = self.box_to_position(detection)
            track.update(position, frame_number, confidence)
            used_detections.add(j)
            
            # Reinitialize fast tracker if this is the main ball
            if track_id == self.main_ball_id:
                self._init_fast_tracker_from_position(position)
            
            # Mark this assignment as used
            distance_matrix[i, :] = np.inf
            distance_matrix[:, j] = np.inf
        
        # Create new tracks for unmatched detections
        for j, det in enumerate(detections):
            if j not in used_detections:
                position, confidence = self.box_to_position(det)
                if not self.is_in_elevator_area(position):
                    self._create_track(position, frame_number, confidence)
    
    def _create_track(self, position, frame_number, confidence):
        """Create new track"""
        track = BallTrack()
        track.track_id = self.next_track_id
        track.start_frame = frame_number
        track.update(position, frame_number, confidence)
        
        self.tracks[self.next_track_id] = track
        print(f"[BallTracker] Created track {self.next_track_id} at {position}")
        self.next_track_id += 1
        
        # If no main ball, make this the main ball
        if self.main_ball_id is None:
            self.main_ball_id = track.track_id
            self.ball_position = position
            self.initialized = True
    
    def _is_in_init_region(self, position):
        """Check if position is in the initialization region"""
        x, y = position
        return (self.INIT_BALL_REGION[0][0] <= x <= self.INIT_BALL_REGION[1][0] and 
                self.INIT_BALL_REGION[0][1] <= y <= self.INIT_BALL_REGION[1][1])
    
    def _prioritize_init_detections(self, detections):
        """Prioritize detections in init region during startup"""
        if self.initialized:
            return detections
            
        # During initialization, prioritize detections in init region
        init_detections = []
        other_detections = []
        
        for det in detections:
            position, _ = self.box_to_position(det)
            if self._is_in_init_region(position):
                init_detections.append(det)
            else:
                other_detections.append(det)
        
        # Return init region detections first, then others
        return init_detections + other_detections
    
    def _select_main_ball(self):
        """Select the best track as main ball using stability scoring"""
        if not self.tracks:
            self.main_ball_id = None
            self.ball_position = None
            return
            
        best_track_id = None
        best_score = -1
        
        for track_id, track in self.tracks.items():
            score = track.get_stability_score()
            if score > best_score:
                best_score = score
                best_track_id = track_id
        
        if best_track_id != self.main_ball_id:
            self.main_ball_id = best_track_id
            print(f"[BallTracker] Switched to track {best_track_id} as main ball")
        
        if self.main_ball_id and self.main_ball_id in self.tracks:
            self.ball_position = self.tracks[self.main_ball_id].get_current_position()
        else:
            self.ball_position = None

    def _init_fast_tracker_from_position(self, position):
        """Initialize fast tracker at given position"""
        if not hasattr(self, 'latest_gray_frame') or self.latest_gray_frame is None:
            return False
            
        try:
            self.fast_tracker = cv2.TrackerKCF_create()
            x, y = position
            half = self.tracking_window_size // 2
            bbox = (x - half, y - half, self.tracking_window_size, self.tracking_window_size)
            
            h, w = self.latest_gray_frame.shape[:2]
            x, y, w_box, h_box = bbox
            x = max(0, min(x, w - w_box))
            y = max(0, min(y, h - h_box))
            w_box = min(w_box, w - x)
            h_box = min(h_box, h - y)
            bbox = (x, y, w_box, h_box)
            
            if self.fast_tracker.init(self.latest_gray_frame, bbox):
                return True
        except Exception as e:
            print(f"[BallTracker] Failed to init fast tracker: {e}")
        
        self.fast_tracker = None
        return False

    def update_fast_tracker(self, frame):
        """Update fast tracker and return position"""
        if self.fast_tracker is None:
            return None
        try:
            success, bbox = self.fast_tracker.update(frame)
            if success:
                x, y, w, h = bbox
                return (int(x + w / 2), int(y + h / 2))
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
                    self.latest_gray_frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            time.sleep(0.001)

    def consumer_loop(self):
        while self.running:
            with self.lock:
                rgb = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None
                gray = self.latest_gray_frame.copy() if self.latest_gray_frame is not None else None

            if rgb is None or bgr is None or gray is None:
                time.sleep(0.001)
                continue

            self.frame_counter += 1
            use_yolo = self.frame_counter % self.yolo_every_n_frames == 0
            current_position = None

            # Try fast tracker first if we have a main ball
            if not use_yolo and self.fast_tracker and self.main_ball_id:
                fast_pos = self.update_fast_tracker(gray)
                if fast_pos and not self.is_in_elevator_area(fast_pos):
                    # Update main track with fast tracker result
                    if self.main_ball_id in self.tracks:
                        self.tracks[self.main_ball_id].update(fast_pos, self.frame_counter, 0.8)
                        current_position = fast_pos
                else:
                    use_yolo = True

            # Run YOLO detection
            if use_yolo:
                detections = self._run_yolo_detection(rgb, gray)
                self.update_tracks(detections, self.frame_counter)
                self._select_main_ball()
                
                if self.main_ball_id and self.main_ball_id in self.tracks:
                    current_position = self.tracks[self.main_ball_id].get_current_position()

            # Update ball position
            if current_position:
                self.ball_position = current_position
            elif self.main_ball_id and self.main_ball_id in self.tracks:
                # Use prediction if available
                track = self.tracks[self.main_ball_id]
                if track.disappeared_count < 10:  # Use prediction for brief periods
                    self.ball_position = track.prediction

            time.sleep(0.001)

    def _run_yolo_detection(self, rgb, gray):
        """Run YOLO detection and return valid detections"""
        detections = []
        
        try:
            if self.resized_input:
                original_size = (rgb.shape[1], rgb.shape[0])
                resized_rgb = cv2.resize(rgb, self.yolo_input_size)
                scale_x = original_size[0] / self.yolo_input_size[0]
                scale_y = original_size[1] / self.yolo_input_size[1]
                results = self.model.predict(resized_rgb)
            else:
                results = self.model.predict(rgb)
                scale_x = scale_y = 1.0

            for box in results.boxes:
                label = self.model.get_label(box.cls[0])
                conf = float(box.conf[0])
                # Lower confidence threshold during initialization
                min_conf = 0.3 if not self.initialized else 0.5
                if label != "ball" or conf < min_conf:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                if self.resized_input:
                    x1 = int(x1 * scale_x)
                    x2 = int(x2 * scale_x)
                    y1 = int(y1 * scale_y)
                    y2 = int(y2 * scale_y)

                # Get center of mass for better accuracy
                roi = gray[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                    
                mean_intensity = cv2.mean(roi)[0]
                # Lower intensity threshold during initialization
                min_intensity = 30 if not self.initialized else 50
                if mean_intensity < min_intensity:
                    continue

                _, thresh = cv2.threshold(roi, 30, 255, cv2.THRESH_BINARY)
                local_com = get_center_of_mass(thresh)
                if not local_com:
                    continue

                cx, cy = local_com
                global_com = (x1 + cx, y1 + cy)
                
                detections.append({
                    'box': (x1, y1, x2, y2),
                    'position': global_com,
                    'confidence': conf
                })
                
        except Exception as e:
            print(f"[BallTracker] YOLO detection failed: {e}")
            
        return detections

    def start(self):
        self.running = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def get_position(self):
        """Get current ball position"""
        return self.ball_position

    def retrack(self):
        """Reset tracking state - inspired by volleyball.py cleanup"""
        print("[BallTracker] Retracking initiated - clearing all tracks")
        self.tracks.clear()
        self.main_ball_id = None
        self.ball_position = None
        self.initialized = False
        self.fast_tracker = None
        self.frame_counter = 0

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None
    
    def get_track_info(self):
        """Get information about all active tracks"""
        info = {
            'main_ball_id': self.main_ball_id,
            'total_tracks': len(self.tracks),
            'tracks': {}
        }
        
        for track_id, track in self.tracks.items():
            info['tracks'][track_id] = {
                'position': track.get_current_position(),
                'prediction': track.prediction,
                'stability_score': track.get_stability_score(),
                'length': len(track.positions),
                'disappeared_count': track.disappeared_count
            }
            
        return info