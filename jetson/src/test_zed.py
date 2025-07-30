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

class SimpleBall3DTracker:
    def __init__(self, roi_size=100, ball_radius=15):
        """
        Simple 3D ball tracker - just tracks the closest ball-sized cluster
        
        Args:
            roi_size: Size of search area around last position (mm)
            ball_radius: Expected ball radius for clustering (mm)
        """
        self.roi_size = roi_size
        self.ball_radius = ball_radius
        
        self.last_3d_position = None
        self.last_2d_position = None
        self.tracking = False
        
    def initialize_tracking(self, point_cloud, x, y):
        """Initialize tracking from YOLO detection at 2D pixel position"""
        pos_3d = get_3d_position(point_cloud, x, y)
        if pos_3d is not None:
            self.last_3d_position = np.array(pos_3d)
            self.last_2d_position = (x, y)
            self.tracking = True
            print(f"[SimpleBallTracker] Initialized at 3D position: {pos_3d[0]:.1f},{pos_3d[1]:.1f},{pos_3d[2]:.1f}mm")
            return True
        else:
            print(f"[SimpleBallTracker] Failed to get 3D position at pixel ({x}, {y})")
        return False
    
    def update(self, point_cloud):
        """Update tracker - just find closest valid point to last position"""
        if not self.tracking:
            return None, None
        
        # Get point cloud data
        point_cloud_np = point_cloud.get_data()
        if point_cloud_np is None:
            return self.last_2d_position, self.last_3d_position
            
        height, width = point_cloud_np.shape[:2]
        points = point_cloud_np.reshape(-1, 4)
        
        # Filter valid points - be more lenient
        valid_mask = (
            ~np.isnan(points[:, 0]) & 
            ~np.isnan(points[:, 1]) & 
            ~np.isnan(points[:, 2]) &
            (points[:, 2] > 50) &   # More lenient depth range
            (points[:, 2] < 4000)   
        )
        
        valid_points = points[valid_mask]
        if len(valid_points) < 5:
            print(f"[SimpleBallTracker] Only {len(valid_points)} valid points found")
            return self.last_2d_position, self.last_3d_position
        
        # Find points within search radius of last position
        search_center = self.last_3d_position
        distances_to_last = np.linalg.norm(valid_points[:, :3] - search_center, axis=1)
        
        # Start with smaller radius and expand if needed
        search_radius = self.roi_size
        for attempt in range(3):  # Try 3 different radii
            nearby_mask = distances_to_last < search_radius
            nearby_points = valid_points[nearby_mask]
            
            if len(nearby_points) >= 3:
                break
            search_radius *= 1.5  # Expand search
            print(f"[SimpleBallTracker] Expanding search radius to {search_radius:.1f}mm")
        
        if len(nearby_points) < 3:
            print(f"[SimpleBallTracker] No points found within {search_radius:.1f}mm of last position")
            return self.last_2d_position, self.last_3d_position
        
        # Simple approach: just use the closest point to the last position
        closest_idx = np.argmin(distances_to_last[distances_to_last < search_radius])
        nearby_indices = np.where(distances_to_last < search_radius)[0]
        best_point_idx = nearby_indices[closest_idx]
        
        new_3d_position = valid_points[best_point_idx, :3]
        
        # Update position
        self.last_3d_position = new_3d_position
        
        # Convert back to 2D pixel coordinates
        valid_indices = np.where(valid_mask)[0]
        original_idx = valid_indices[best_point_idx]
        
        pixel_x = int(original_idx % width)
        pixel_y = int(original_idx // width)
        self.last_2d_position = (pixel_x, pixel_y)
        
        movement = np.linalg.norm(new_3d_position - search_center)
        print(f"[SimpleBallTracker] Updated: {new_3d_position[0]:.1f},{new_3d_position[1]:.1f},{new_3d_position[2]:.1f}mm (moved {movement:.1f}mm)")
        
        return self.last_2d_position, self.last_3d_position
    
    def reset(self):
        """Reset tracker"""
        self.tracking = False
        self.last_3d_position = None
        self.last_2d_position = None

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

def simple_detect_and_track_ball_3d(rgb, point_cloud, model, tracker):
    """
    Simplified detection: YOLO for initialization, simple 3D tracking
    """
    # If not tracking, try YOLO detection for initialization
    if not tracker.tracking:
        results = model.predict(rgb)
        ball_detections = []
        
        for box in results.boxes:
            label = model.get_label(box.cls[0])
            if label == "ball":
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                confidence = float(box.conf[0])
                ball_detections.append((cx, cy, confidence, (x1, y1, x2, y2)))
        
        if ball_detections:
            # Use the highest confidence detection
            ball_detections.sort(key=lambda x: x[2], reverse=True)
            cx, cy, yolo_conf, bbox = ball_detections[0]
            print(f"[YOLO] Found ball at ({cx}, {cy}) with confidence {yolo_conf:.3f}")
            
            # Try to initialize 3D tracking
            if tracker.initialize_tracking(point_cloud, cx, cy):
                return {
                    'bbox': bbox,
                    'center_2d': (cx, cy),
                    'position_3d': tracker.last_3d_position,
                    'confidence': 1.0,
                    'tracking_mode': 'YOLO_INIT'
                }
            else:
                print(f"[YOLO] Failed to initialize 3D tracking at pixel ({cx}, {cy})")
        else:
            print("[YOLO] No ball detected")
        return None
    
    # Use simple 3D tracking - always succeeds if initialized
    pos_2d, pos_3d = tracker.update(point_cloud)
    
    if pos_2d is None or pos_3d is None:
        # This should rarely happen with the simple tracker
        print("[3DTracker] Update failed - no position returned")
        return None
    
    # Estimate bounding box from 2D position
    bbox_size = 30
    x1 = max(0, pos_2d[0] - bbox_size//2)
    y1 = max(0, pos_2d[1] - bbox_size//2)
    x2 = min(rgb.shape[1], pos_2d[0] + bbox_size//2)
    y2 = min(rgb.shape[0], pos_2d[1] + bbox_size//2)
    
    return {
        'bbox': (x1, y1, x2, y2),
        'center_2d': pos_2d,
        'position_3d': pos_3d,
        'confidence': 1.0,  # Always confident - there's only one ball!
        'tracking_mode': '3D_TRACKING'
    }

def main():
    # Initialize ZED camera first
    zed = init_zed_camera()
    
    # Initialize YOLO model
    print("Loading YOLO model...")
    model = YOLOModel("tracking/v8-291.onnx")
    print("Model loaded successfully")
    
    # Initialize simple 3D ball tracker
    ball_tracker = SimpleBall3DTracker(
        roi_size=150,          # 150mm search radius
        ball_radius=15         # 15mm ball radius
    )
    print("Simple 3D ball tracker initialized")

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
                
            # Use simple detection/tracking
            ball_detection = simple_detect_and_track_ball_3d(rgb, point_cloud, model, ball_tracker)
            
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
                
                # Print to console - simplified
                print(f"Ball: {pos_3d[0]:.1f},{pos_3d[1]:.1f},{pos_3d[2]:.1f}mm | {mode}")
            
            # Display tracker status
            status_color = (0, 255, 0) if ball_tracker.tracking else (0, 0, 255)
            status_text = f"Simple Tracker: {'ACTIVE' if ball_tracker.tracking else 'SEARCHING'}"
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