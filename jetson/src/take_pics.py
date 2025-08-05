import os
import cv2
import pyzed.sl as sl
import threading
import sys
import time
import pygame

# Add parent directory to path to import joystick controller
from control.joystick_controller import JoystickController
from arduino_connection import ArduinoConnection

class PictureTakingJoystickController(JoystickController):
    def __init__(self, arduino, save_callback):
        super().__init__(arduino)
        self.save_callback = save_callback
        self.prev_right_trigger = 0.0
        self.trigger_threshold = 0.5

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

                # Right trigger for taking pictures
                right_trigger = joystick.get_axis(5)  # Right trigger axis
                if right_trigger > self.trigger_threshold and self.prev_right_trigger <= self.trigger_threshold:
                    # Trigger pressed - take picture
                    if self.save_callback:
                        self.save_callback()
                self.prev_right_trigger = right_trigger

                time.sleep(max(0, interval - (time.time() - loop_start)))

        except Exception as e:
            print(f"[XboxController] Exception: {e}")

        finally:
            self.arduino.send_speed(0, 0)
            pygame.quit()
            print("[XboxController] Clean exit, resources released.")

def init_zed_camera():
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD720
    params.camera_fps = 60
    params.coordinate_units = sl.UNIT.MILLIMETER
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); exit(1)
    return zed

def grab_zed_frame(zed):
    mat = sl.Mat()
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None, None
    # Updated ZED SDK syntax - remove MAT_TYPE parameter
    zed.retrieve_image(mat, sl.VIEW.LEFT)
    bgr = mat.get_data()
    # 2) make an RGB copy if you need it elsewhere
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb, bgr

def main():
    # Initialize ZED camera first
    zed = init_zed_camera()
    save_dir = "img_finetune"
    os.makedirs(save_dir, exist_ok=True)
    img_count = 1
    
    # Shared variables for picture taking
    current_frame = None
    frame_lock = threading.Lock()

    def save_picture():
        """Callback function to save the current frame"""
        nonlocal img_count, current_frame
        with frame_lock:
            if current_frame is not None:
                fn = os.path.join(save_dir, f"img_{img_count+1}.jpg")
                cv2.imwrite(fn, current_frame)
                print(f"Saved {fn}")
                img_count += 1
            else:
                print("No frame available to save")

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
            joystick_controller = PictureTakingJoystickController(arduino, save_picture)
            joystick_thread = threading.Thread(target=joystick_controller.start, daemon=True)
            joystick_thread.start()
            print("Joystick controller started - use joystick to tilt maze, A button for elevator, RT to take pics")
        except Exception as e:
            print(f"Failed to initialize joystick: {e}")
            print("Continuing without joystick control...")
            joystick_controller = None

    # create a small, resizable window
    win_name = "ZED View"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    print("Controls:")
    print("  Q - Quit")
    if joystick_controller:
        print("  Left stick - Tilt maze")
        print("  A button - Toggle elevator")
        print("  Right trigger - Take picture")
    else:
        print("  SPACE - Save image (fallback)")

    try: 
        while True:
            rgb, bgr = grab_zed_frame(zed)
            if bgr is None:
                print("Frame grab failed"); break

            # Update current frame for picture taking
            with frame_lock:
                current_frame = bgr.copy()

            # Display the BGR image directly:
            cv2.imshow(win_name, bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and not joystick_controller:  # SPACE: fallback if no joystick
                save_picture()
            elif key == ord('q'):  # Quit
                break
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
        
        zed.close()
        cv2.destroyAllWindows()
        print("Cleanup complete")
        

if __name__ == "__main__":
    main()