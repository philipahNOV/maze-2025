import os
import cv2
import pyzed.sl as sl
import threading
import sys
import time
import pygame

from control.joystick_controller import JoystickController
from arduino_connection import ArduinoConnection

# Global depth map for mouse callback
depth_map = None

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
                axis_x = -joystick.get_axis(1)
                axis_y = -joystick.get_axis(0)

                vel_x = self.scaled_output(axis_x)
                vel_y = self.scaled_output(axis_y)

                self.arduino.send_speed(vel_x, vel_y)

                button = joystick.get_button(0)
                if button and not self.prev_button_state:
                    self.elevator_state *= -1
                    self.arduino.send_elevator(self.elevator_state)
                self.prev_button_state = button

                right_trigger = joystick.get_axis(5)
                if right_trigger > self.trigger_threshold and self.prev_right_trigger <= self.trigger_threshold:
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
        print("ZED failed to open")
        exit(1)
    return zed

def grab_zed_frame(zed):
    global depth_map
    mat = sl.Mat()
    depth_map = sl.Mat()

    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        return None, None

    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
    bgr = mat.get_data()
    zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb, bgr

def on_mouse(event, x, y, flags, param):
    global depth_map
    if event == cv2.EVENT_MOUSEMOVE and depth_map is not None:
        value = depth_map.get_value(x, y)
        if value and len(value) == 3:
            z_depth = value[1]
            if np.isfinite(z_depth) and z_depth > 0:
                print(f"Depth at ({x}, {y}): {z_depth:.2f} mm")
            else:
                print(f"Depth at ({x}, {y}): INVALID")

def main():
    zed = init_zed_camera()
    save_dir = "captures7"
    os.makedirs(save_dir, exist_ok=True)
    img_count = 26

    current_frame = None
    frame_lock = threading.Lock()

    def save_picture():
        nonlocal img_count, current_frame
        with frame_lock:
            if current_frame is not None:
                fn = os.path.join(save_dir, f"img_{img_count+1}.jpg")
                cv2.imwrite(fn, current_frame)
                print(f"Saved {fn}")
                img_count += 1
            else:
                print("No frame available to save")

    print("Initializing Arduino connection...")
    try:
        arduino = ArduinoConnection()
        time.sleep(2)
        print("Arduino connected successfully")
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        print("Continuing without joystick control...")
        arduino = None

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

    win_name = "ZED View"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)
    cv2.setMouseCallback(win_name, on_mouse)

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
                print("Frame grab failed")
                break

            with frame_lock:
                current_frame = bgr.copy()

            cv2.imshow(win_name, bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and not joystick_controller:
                save_picture()
            elif key == ord('q'):
                break
    finally:
        if joystick_controller:
            joystick_controller.stop()
            if joystick_thread and joystick_thread.is_alive():
                joystick_thread.join(timeout=2.0)

        if arduino:
            arduino.send_speed(0, 0)
            time.sleep(0.1)

        zed.close()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
