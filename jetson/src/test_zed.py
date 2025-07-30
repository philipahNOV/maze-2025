import os
import cv2
import pyzed.sl as sl
import threading
import sys
import time
import pygame
import numpy as np
from scipy.spatial.distance import cdist

from control.joystick_controller import JoystickController
from arduino_connection import ArduinoConnection
from tracking.model_loader import YOLOModel

class Ball3DTracker:
    def __init__(self, roi_size=100, depth_threshold=50, min_points=10, max_velocity=500):
        """
        3D point cloud ball tracker
        
        Args:
            roi_size: Size of 3D ROI cube around last position (mm)
            depth_threshold: Max depth difference for clustering (mm) 
            min_points: Minimum points needed for valid detection
            max_velocity: Maximum expected velocity (mm/frame) for outlier rejection
        """
        self.roi_size = roi_size
        self.depth_threshold = depth_threshold
        self.min_points = min_points
        self.max_velocity = max_velocity
        
        self.last_3d_position = None
        self.last_2d_position = None
        self.position_history = []
        self.velocity_history = []
        self.confidence = 0.0
        self.tracking = False
        
        # For motion detection
        self.prev_point_cloud = None
        self.motion_threshold = 20  # mm movement to be considered motion
        
    def initialize_tracking(self, point_cloud, x, y):
        """Initialize tracking from YOLO detection at 2D pixel position"""
        pos_3d = get_3d_position(point_cloud, x, y)
        if pos_3d is not None:
            self.last_3d_position = np.array(pos_3d)
            self.last_2d_position = (x, y)
            self.position_history = [self.last_3d_position.copy()]
            self.velocity_history = []
            self.confidence = 1.0
            self.tracking = True
            print(f"[3DTracker] Initialized at 3D position: {pos_3d}")
            return True
        return False
    
    def extract_roi_points(self, point_cloud):
        """Extract 3D points within ROI around last known position"""
        if self.last_3d_position is None:
            return None, None
            
        # Get point cloud as numpy array
        point_cloud_np = point_cloud.get_data()
        if point_cloud_np is None:
            return None, None
            
        height, width = point_cloud_np.shape[:2]
        
        # Reshape to (H*W, 4) and filter valid points
        points = point_cloud_np.reshape(-1, 4)
        
        # More robust filtering for valid points
        valid_mask = (
            ~np.isnan(points[:, 0]) & 
            ~np.isnan(points[:, 1]) & 
            ~np.isnan(points[:, 2]) &
            (points[:, 2] > 100) &  # Minimum 100mm depth
            (points[:, 2] < 5000)   # Maximum 5m depth
        )
        
        valid_points = points[valid_mask]
        
        if len(valid_points) < self.min_points:
            return None, None
            
        # Define ROI boundaries - make it larger to be more forgiving
        roi_size = self.roi_size * 1.5  # 50% larger ROI
        roi_min = self.last_3d_position - roi_size/2
        roi_max = self.last_3d_position + roi_size/2
        
        # Filter points within ROI
        roi_mask = (
            (valid_points[:, 0] >= roi_min[0]) & (valid_points[:, 0] <= roi_max[0]) &
            (valid_points[:, 1] >= roi_min[1]) & (valid_points[:, 1] <= roi_max[1]) &
            (valid_points[:, 2] >= roi_min[2]) & (valid_points[:, 2] <= roi_max[2])
        )
        
        roi_points = valid_points[roi_mask]
        
        if len(roi_points) < self.min_points:
            print(f"[3DTracker] Only {len(roi_points)} points in ROI, need {self.min_points}")
            return None, None
        
        # Get corresponding 2D pixel coordinates
        valid_indices = np.where(valid_mask)[0]
        roi_indices = valid_indices[roi_mask]
        
        pixel_coords = np.column_stack([
            roi_indices % width,  # x coordinates
            roi_indices // width   # y coordinates  
        ])
        
        return roi_points[:, :3], pixel_coords  # Return XYZ only
    
    def detect_motion_blobs(self, current_points, pixel_coords):
        """Detect moving blobs in the current point cloud ROI"""
        if current_points is None or len(current_points) < self.min_points:
            return None, None
            
        # For first frame, just return center of mass
        if self.prev_point_cloud is None:
            self.prev_point_cloud = current_points.copy()
            center_3d = np.mean(current_points, axis=0)
            center_2d = np.mean(pixel_coords, axis=0).astype(int)
            return center_3d, tuple(center_2d)
        
        # Instead of motion detection, find the cluster closest to the expected ball position
        # This is more reliable than motion detection
        
        # If we have velocity history, predict where the ball should be
        if len(self.position_history) >= 2:
            predicted_pos = self.get_predicted_position()
        else:
            predicted_pos = self.last_3d_position
        
        # Find the densest cluster of points near the predicted position
        # Use a smaller search radius for clustering
        cluster_radius = 50  # 50mm radius for clustering
        
        # Calculate distances from all points to predicted position
        distances = np.linalg.norm(current_points - predicted_pos, axis=1)
        
        # Find points within cluster radius
        cluster_mask = distances < cluster_radius
        cluster_points = current_points[cluster_mask]
        cluster_pixels = pixel_coords[cluster_mask]
        
        if len(cluster_points) >= max(3, self.min_points // 3):  # Need at least 3 points
            # Use the cluster
            center_3d = np.mean(cluster_points, axis=0)
            center_2d = np.mean(cluster_pixels, axis=0).astype(int)
            print(f"[3DTracker] Using cluster: {len(cluster_points)} points, dist from pred: {np.linalg.norm(center_3d - predicted_pos):.1f}mm")
        else:
            # Fallback: use the closest point to prediction
            closest_idx = np.argmin(distances)
            center_3d = current_points[closest_idx]
            center_2d = pixel_coords[closest_idx]
            print(f"[3DTracker] Using closest point, dist: {distances[closest_idx]:.1f}mm")
        
        # Update previous point cloud for next frame
        self.prev_point_cloud = current_points.copy()
        
        return center_3d, tuple(center_2d)
    
    def update(self, point_cloud):
        """Update tracker with new point cloud"""
        if not self.tracking:
            return None, None
            
        # Extract ROI points
        roi_points, pixel_coords = self.extract_roi_points(point_cloud)
        
        if roi_points is None or len(roi_points) < self.min_points:
            # Much more aggressive confidence decay when no points found
            self.confidence = max(0.0, self.confidence - 0.1)
            print(f"[3DTracker] Insufficient ROI points: {len(roi_points) if roi_points is not None else 0}/{self.min_points}, conf: {self.confidence:.2f}")
            if self.confidence < 0.3:  # Reset faster
                self.tracking = False
                print("[3DTracker] Lost tracking - insufficient points in ROI")
            return self.last_2d_position, self.last_3d_position
        
        # Detect motion blobs
        new_3d_pos, new_2d_pos = self.detect_motion_blobs(roi_points, pixel_coords)
        
        if new_3d_pos is None:
            self.confidence = max(0.0, self.confidence - 0.1)
            print(f"[3DTracker] No motion detected, conf: {self.confidence:.2f}")
            return self.last_2d_position, self.last_3d_position
        
        # Check if the new position makes sense
        if len(self.position_history) > 0:
            movement = np.linalg.norm(new_3d_pos - self.last_3d_position)
            
            # If movement is too large, reject
            if movement > self.max_velocity:
                print(f"[3DTracker] Rejecting large movement: {movement:.1f}mm/frame (max: {self.max_velocity})")
                self.confidence = max(0.0, self.confidence - 0.15)
                if self.confidence < 0.3:
                    self.tracking = False
                    print("[3DTracker] Lost tracking - too much movement")
                return self.last_2d_position, self.last_3d_position
            
            # Track velocity for prediction
            self.velocity_history.append(movement)
            if len(self.velocity_history) > 10:
                self.velocity_history.pop(0)
        
        # Update position - this is the key fix!
        self.last_3d_position = new_3d_pos
        self.last_2d_position = new_2d_pos
        self.position_history.append(new_3d_pos.copy())
        if len(self.position_history) > 20:
            self.position_history.pop(0)
        
        # Strong confidence boost for successful tracking
        self.confidence = min(1.0, self.confidence + 0.2)
        
        print(f"[3DTracker] Updated position: {new_3d_pos[0]:.1f},{new_3d_pos[1]:.1f},{new_3d_pos[2]:.1f}mm, conf: {self.confidence:.2f}")
        
        return new_2d_pos, new_3d_pos
    
    def get_predicted_position(self):
        """Get predicted next position based on velocity"""
        if len(self.position_history) < 2:
            return self.last_3d_position
            
        # Simple linear prediction
        velocity = self.position_history[-1] - self.position_history[-2]
        predicted = self.last_3d_position + velocity
        return predicted
    
    def reset(self):
        """Reset tracker"""
        self.tracking = False
        self.last_3d_position = None
        self.last_2d_position = None
        self.position_history = []
        self.velocity_history = []
        self.confidence = 0.0
        self.prev_point_cloud = None

class BallTrackingJoystickController(JoystickController):
    def __init__(self, arduino):
        super().__init__(arduino)
        
    def start(self):
        pygame.init()
        pygame.joystick.init()

        try:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        except pygame.error:
            print("[XboxController] No joystick found.")
            return

        self.running = True
        interval = 1.0 / self.update_rate_hz
        print("[XboxController] Started polling at {:.0f} Hz".format(self.update_rate_hz))

        try:
            while self.running:
                loop_start = time.time()

                pygame.event.pump()
                axis_x = -joystick.get_axis(1)  # left stick horizontal
                axis_y = -joystick.get_axis(0)  # left stick vertical

                vel_x = self.scaled_output(axis_x)
                vel_y = self.scaled_output(axis_y)

                self.arduino.send_speed(vel_x, vel_y)

                # A button for elevator
                button = joystick.get_button(0)
                if button and not self.prev_button_state:
                    self.elevator_state *= -1
                    self.arduino.send_elevator(self.elevator_state)
                self.prev_button_state = button

                time.sleep(max(0, interval - (time.time() - loop_start)))

        except Exception as e:
            print(f"[XboxController] Exception: {e}")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()
            print("[XboxController] Clean exit, resources released.")

class LetZedTakePics():
    pass
    

def init_zed_camera():
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD720
    params.camera_fps = 60
    params.coordinate_units = sl.UNIT.MILLIMETER
    params.depth_mode = sl.DEPTH_MODE.ULTRA  # Enable depth sensing
    params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); exit(1)
    return zed

def grab_zed_frame(zed):
    mat = sl.Mat()
    point_cloud = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None, None, None
    
    # Grab 3-channel BGR image with proper format
    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
    bgr = mat.get_data()
    
    # Fix potential format issues - ensure proper BGR format
    if bgr is not None:
        bgr = np.ascontiguousarray(bgr.copy())  # Ensure contiguous memory layout
        if len(bgr.shape) == 3 and bgr.shape[2] == 4:  # If BGRA, convert to BGR
            bgr = bgr[:, :, :3]
    
    # Grab point cloud for 3D coordinates
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, sl.MAT_TYPE.F32_C4)
    
    # Make an RGB copy
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if bgr is not None else None
    return rgb, bgr, point_cloud

def get_3d_position(point_cloud, x, y):
    """Get 3D coordinates from point cloud at given 2D pixel position"""
    try:
        # Get the 3D point at the specified pixel
        err, point_3d = point_cloud.get_value(x, y)
        if err == sl.ERROR_CODE.SUCCESS:
            # point_3d is a 4-element array [x, y, z, confidence]
            if not np.isnan(point_3d[0]) and not np.isnan(point_3d[1]) and not np.isnan(point_3d[2]):
                return point_3d[:3]  # Return [x, y, z] in millimeters
    except:
        pass
    return None

def detect_and_track_ball_3d_hybrid(rgb, point_cloud, model, tracker):
    """
    Hybrid detection: YOLO for initialization, 3D point cloud for tracking
    """
    # If not tracking, try YOLO detection for initialization
    if not tracker.tracking:
        results = model.predict(rgb)
        for box in results.boxes:
            label = model.get_label(box.cls[0])
            if label == "ball":
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Try to initialize 3D tracking
                if tracker.initialize_tracking(point_cloud, cx, cy):
                    return {
                        'bbox': (x1, y1, x2, y2),
                        'center_2d': (cx, cy),
                        'position_3d': tracker.last_3d_position,
                        'confidence': float(box.conf[0]),
                        'tracking_mode': 'YOLO_INIT'
                    }
        return None
    
    # Use 3D tracking
    pos_2d, pos_3d = tracker.update(point_cloud)
    
    if pos_2d is None or pos_3d is None:
        return None
    
    # Estimate bounding box from 2D position (rough approximation)
    bbox_size = 30  # pixels
    x1 = max(0, pos_2d[0] - bbox_size//2)
    y1 = max(0, pos_2d[1] - bbox_size//2)
    x2 = min(rgb.shape[1], pos_2d[0] + bbox_size//2)
    y2 = min(rgb.shape[0], pos_2d[1] + bbox_size//2)
    
    return {
        'bbox': (x1, y1, x2, y2),
        'center_2d': pos_2d,
        'position_3d': pos_3d,
        'confidence': tracker.confidence,
        'tracking_mode': '3D_TRACKING'
    }

def main():
    # Initialize ZED camera first
    zed = init_zed_camera()
    
    # Initialize YOLO model
    print("Loading YOLO model...")
    model = YOLOModel("tracking/v8-291.onnx")
    print("Model loaded successfully")
    
    # Initialize 3D ball tracker with better parameters
    ball_tracker = Ball3DTracker(
        roi_size=120,          # Smaller, more focused ROI
        depth_threshold=30,    # Tighter depth clustering
        min_points=5,          # Even fewer points needed
        max_velocity=150       # More reasonable max velocity
    )
    print("3D ball tracker initialized")

    # Initialize Arduino connection
    print("Initializing Arduino connection...")
    try:
        arduino = ArduinoConnection()
        time.sleep(2)  # Give Arduino time to initialize
        print("Arduino connected successfully")
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        print("Continuing without joystick control...")
        arduino = None

    # Initialize joystick controller if Arduino is available
    joystick_controller = None
    joystick_thread = None
    if arduino:
        try:
            joystick_controller = BallTrackingJoystickController(arduino)
            joystick_thread = threading.Thread(target=joystick_controller.start, daemon=True)
            joystick_thread.start()
            print("Joystick controller started - use joystick to tilt maze, A button for elevator")
        except Exception as e:
            print(f"Failed to initialize joystick: {e}")
            print("Continuing without joystick control...")
            joystick_controller = None

    # Create a small, resizable window
    win_name = "ZED 3D Ball Tracking"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    print("Starting 3D ball tracking with joystick control. Press 'q' to quit, 'r' to reset tracker.")
    print("Controls:")
    print("  Q - Quit")
    print("  R - Reset 3D tracker")
    if joystick_controller:
        print("  Left stick - Tilt maze")
        print("  A button - Toggle elevator")
    
    try:
        while True:
            # Grab frame and point cloud
            rgb, bgr, point_cloud = grab_zed_frame(zed)
            if rgb is None:
                continue
                
            # Use hybrid detection/tracking
            ball_detection = detect_and_track_ball_3d_hybrid(rgb, point_cloud, model, ball_tracker)
            
            # Draw detection on the image
            display_frame = bgr.copy()
            if ball_detection:
                x1, y1, x2, y2 = ball_detection['bbox']
                cx, cy = ball_detection['center_2d']
                pos_3d = ball_detection['position_3d']
                confidence = ball_detection['confidence']
                mode = ball_detection['tracking_mode']
                
                # Color code by tracking mode
                color = (0, 255, 0) if mode == 'YOLO_INIT' else (255, 0, 0)  # Green for YOLO, Blue for 3D
                
                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                
                # Draw center point
                cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Display 3D position and tracking info
                text = f"3D: ({pos_3d[0]:.1f}, {pos_3d[1]:.1f}, {pos_3d[2]:.1f})mm"
                mode_text = f"Mode: {mode} | Conf: {confidence:.2f}"
                
                cv2.putText(display_frame, text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, mode_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Print to console
                avg_velocity = np.mean(ball_tracker.velocity_history) if ball_tracker.velocity_history else 0
                print(f"Ball: {pos_3d[0]:.1f},{pos_3d[1]:.1f},{pos_3d[2]:.1f}mm | {mode} | Conf:{confidence:.2f} | Vel:{avg_velocity:.1f}mm/f")
            
            # Display tracker status
            status_color = (0, 255, 0) if ball_tracker.tracking else (0, 0, 255)
            status_text = f"3D Tracker: {'ACTIVE' if ball_tracker.tracking else 'SEARCHING'} | Conf: {ball_tracker.confidence:.2f}"
            cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # Display joystick status
            if joystick_controller:
                joy_text = "Joystick: Active (Left stick tilts maze, A for elevator)"
                cv2.putText(display_frame, joy_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Show the frame
            cv2.imshow(win_name, display_frame)
            
            # Check for quit or reset
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                ball_tracker.reset()
                print("[3DTracker] Tracker reset - searching for ball...")
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up joystick controller
        if joystick_controller:
            joystick_controller.stop()
            if joystick_thread and joystick_thread.is_alive():
                joystick_thread.join(timeout=2.0)
        
        # Stop motors and clean up Arduino
        if arduino:
            arduino.send_speed(0, 0)
            time.sleep(0.1)
        
        cv2.destroyAllWindows()
        zed.close()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()