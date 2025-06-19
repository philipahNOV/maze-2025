from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic

from manual_part.manuel_main import elManuel

import time
import cv2

def initialize_component(component, name, retries=5, delay=2):
    for attempt in range(retries):
        try:
            comp_instance = component()
            print(f"{name} initialized on attempt {attempt + 1}")
            return comp_instance
        except Exception as e:
            print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
            time.sleep(delay)
    raise Exception(f"Failed to initialize {name} after {retries} attempts")

try:
    camera_thread = initialize_component(CameraThread, "CameraThread")
    camera_thread.start()
except Exception as e:
    print(e)
    exit(1)

try:
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
except Exception as e:
    print(e)
    exit(1)


def horizontal(tol = 0.2):
    x_offset = 0.017  # Offset for x-axis orientation
    y_offset = -0.002  # Offset for y-axis orientation
    min_velocity = 22 # Minimum velocity for motors
    kp = 700 # Proportional gain for the control loop
    deadline = time.time() + 300  # 60 seconds deadline
    arduino_thread.send_target_positions(120, 120, 120, 120)  # Stop motors initially

    while time.time() < deadline:
        print(camera_thread.orientation)
        theta_x = camera_thread.orientation[1] + x_offset
        theta_y = camera_thread.orientation[0] + y_offset
        if theta_x is None or theta_y is None:
            print("Orientation data not available yet.")
            continue
        if abs(theta_x) < tol and abs(theta_y) < tol:
            print("Orientation is within tolerance, stopping motors.")
            arduino_thread.send_target_positions(120, 120, 120, 120)
            return
        if abs(theta_x) < tol:
            dir_x = 2
        elif theta_x > 0:
            dir_x = 3
        elif theta_x < 0:
            dir_x = 1
        if abs(theta_y) < tol:
            dir_y = 2
        elif theta_y > 0:
            dir_y = 3
        elif theta_y < 0:
            dir_y = 1

        vel_x = max(int(kp * abs(theta_x)), min_velocity)
        vel_y = max(int(kp * abs(theta_y)), min_velocity)
        print(f"Orientation: {theta_x}, {theta_y} | Velocities: {vel_x}, {vel_y}")
        arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)
    print("Deadline reached, stopping motors.")

time.sleep(10)  # Allow time for Arduino connection to stabilize
horizontal(0.0025)