import os
import cv2
import pyzed.sl as sl
import threading
import sys
import time

# Add parent directory to path to import joystick controller
from control.joystick_controller import JoystickController
from arduino_connection import ArduinoConnection

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
    # 1) grab 3-channel BGR
    zed.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, sl.MAT_TYPE.U8_C3)
    bgr = mat.get_data()
    # 2) make an RGB copy if you need it elsewhere
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb, bgr

def main():
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
            joystick_controller = JoystickController(arduino)
            joystick_thread = threading.Thread(target=joystick_controller.start, daemon=True)
            joystick_thread.start()
            print("Joystick controller started - use joystick to tilt maze, A button for elevator")
        except Exception as e:
            print(f"Failed to initialize joystick: {e}")
            print("Continuing without joystick control...")
            joystick_controller = None
    
    # Initialize ZED camera
    zed = init_zed_camera()
    save_dir = "captures2"
    os.makedirs(save_dir, exist_ok=True)
    img_count = 26

    # create a small, resizable window
    win_name = "ZED View"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    print("Controls:")
    print("  SPACE - Save image")
    print("  Q - Quit")
    if joystick_controller:
        print("  Left stick - Tilt maze")
        print("  A button - Toggle elevator")

    try: 
        while True:
            rgb, bgr = grab_zed_frame(zed)
            if bgr is None:
                print("Frame grab failed"); break

            # Display the BGR image directly:
            cv2.imshow(win_name, bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # SPACE: save BGR so colors stay correct
                fn = os.path.join(save_dir, f"img_{img_count+1}.jpg")
                cv2.imwrite(fn, bgr)
                print("Saved", fn)
                img_count += 1
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