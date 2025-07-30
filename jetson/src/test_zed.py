import os
import cv2
import pyzed.sl as sl
import threading
import sys
import time
import pygame
import numpy as np

from control.joystick_controller import JoystickController
from arduino_connection import ArduinoConnection
from tracking.model_loader import YOLOModel

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
    
    # Grab 3-channel BGR image
    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
    bgr = mat.get_data()
    
    # Grab point cloud for 3D coordinates
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, sl.MAT_TYPE.F32_C4)
    
    # Make an RGB copy
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
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

def detect_and_track_ball_3d(rgb, point_cloud, model):
    """Detect ball using YOLO and get its 3D position"""
    results = model.predict(rgb)
    ball_3d_positions = []
    
    for box in results.boxes:
        label = model.get_label(box.cls[0])
        if label == "ball":
            # Get 2D bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            # Get 3D position at center of bounding box
            pos_3d = get_3d_position(point_cloud, cx, cy)
            if pos_3d is not None:
                ball_3d_positions.append({
                    'bbox': (x1, y1, x2, y2),
                    'center_2d': (cx, cy),
                    'position_3d': pos_3d,
                    'confidence': float(box.conf[0])
                })
    
    return ball_3d_positions

def main():
    # Initialize ZED camera first
    zed = init_zed_camera()
    
    # Initialize YOLO model
    print("Loading YOLO model...")
    model = YOLOModel("tracking/v8-291.onnx")
    print("Model loaded successfully")

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

    print("Starting 3D ball tracking with joystick control. Press 'q' to quit.")
    print("Controls:")
    print("  Q - Quit")
    if joystick_controller:
        print("  Left stick - Tilt maze")
        print("  A button - Toggle elevator")
    
    try:
        while True:
            # Grab frame and point cloud
            rgb, bgr, point_cloud = grab_zed_frame(zed)
            if rgb is None:
                continue
                
            # Detect balls and get 3D positions
            ball_detections = detect_and_track_ball_3d(rgb, point_cloud, model)
            
            # Draw detections on the image
            display_frame = bgr.copy()
            for detection in ball_detections:
                x1, y1, x2, y2 = detection['bbox']
                cx, cy = detection['center_2d']
                pos_3d = detection['position_3d']
                confidence = detection['confidence']
                
                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(display_frame, (cx, cy), 5, (0, 0, 255), -1)
                
                # Display 3D position and confidence
                text = f"3D: ({pos_3d[0]:.1f}, {pos_3d[1]:.1f}, {pos_3d[2]:.1f})mm"
                conf_text = f"Conf: {confidence:.2f}"
                
                cv2.putText(display_frame, text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Print to console
                print(f"Ball detected at 3D position: X={pos_3d[0]:.1f}mm, Y={pos_3d[1]:.1f}mm, Z={pos_3d[2]:.1f}mm (Conf: {confidence:.2f})")
            
            # Display joystick status
            if joystick_controller:
                status_text = "Joystick: Active (Left stick tilts maze, A for elevator)"
                cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Show the frame
            cv2.imshow(win_name, display_frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
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