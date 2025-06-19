from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import testing.ball_recognition
import numpy as np

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

def axisControl(ref):
    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 700 # Proportional gain for the control loop
    tol = 0.001

    theta_x = camera_thread.orientation[1] + x_offset
    theta_y = camera_thread.orientation[0] + y_offset

    if theta_x is None or theta_y is None:
        print("Orientation data not available yet.")
        return
    
    e_x = ref[0] - theta_x
    e_y = ref[1] - theta_y
    if abs(e_x) < tol:
        dir_x = 2
    elif e_x > 0:
        dir_x = 3
    elif e_x < 0:
        dir_x = 1
    if abs(e_y) < tol:
        dir_y = 2
    elif e_y > 0:
        dir_y = 3
    elif e_y < 0:
        dir_y = 1

    vel_x = max(int(kp * abs(e_x)), min_velocity)
    vel_y = max(int(kp * abs(e_y)), min_velocity)
    dir_y = 2
    arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)

def posControl(center, prev_center, ref=(200, 200), tol=10):
    if prev_center is not None:
        if abs(np.linalg.norm(np.array(center) - np.array(prev_center))) > 300:
            print("Large jump detected, resetting position control.")
            return

    e_x = ref[0] - center[0]
    e_y = ref[1] - center[1]
    kp = 0.001  # Proportional gain for position control

    theta_x = -kp * e_x
    theta_y = -kp * e_y
    print(f"theta_x: {theta_x}, theta_y: {theta_y}")
    axisControl((theta_x, theta_y))

def horizontal(tol = 0.2):
    """
    Gradually levels the platform by adjusting actuator angles based on camera orientation.

    This function reads orientation data (theta_x, theta_y) from the camera and sends
    corresponding direction and velocity commands to the Arduino via the arduino_thread.
    It applies a proportional control strategy (P-controller) to minimize tilt in both
    axes, driving the platform toward a horizontal position.

    Parameters:
        tol (float): Acceptable angular deviation from horizontal in radians (default: 0.2).

    Behavior:
    - Adds optional offsets to compensate for sensor calibration error.
    - Sends stop command initially and when within tolerance.
    - Computes direction based on sign of tilt.
    - Computes velocity with a proportional gain (kp), respecting a minimum speed threshold.
    - Terminates either when the platform is level within tolerance or a time deadline is reached.
    """

    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 700 # Proportional gain for the control loop
    deadline = time.time() + 20  # 20 seconds deadline
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
        arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)
    print("Deadline reached, stopping motors.")

time.sleep(10)  # Allow time for Arduino connection to stabilize
horizontal(0.001)

frame = camera_thread.latest_frame
center = None
prev_center = None
limit = time.time() + 100
while time.time() < limit:
    frame = camera_thread.latest_frame
    if frame is None:
        continue
    #frame = cv2.resize(frame, (320, 240))  # Resize to a standard size if needed
    center, radius, masked_frame = testing.ball_recognition.detect_red_ball_frame(frame, center)
    if center is None:
        print("No ball detected, skipping frame.")
        continue
    center = (center[1], center[0])  # Convert to (x, y) format for consistency
    cv2.circle(frame, center, 15, (0, 255, 0), 4)
    print(f"Center: {center}")
    if limit - time.time() < 90:
        posControl(center, prev_center)
    prev_center = center

    cv2.imshow("Test Image", frame)
    cv2.imshow("Masked Frame", masked_frame)
    cv2.waitKey(1)  # This is necessary for the window to update

cv2.destroyAllWindows()

