from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import testing.ball_recognition
import numpy as np
import threading

from manual_part.manuel_main import elManuel

import time
import cv2

ref_lock = threading.Lock()
ref_theta = None

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

def axisControlMultithread():
    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 5000  # Proportional gain for the control loop
    tol = 0
    arduino_thread.send_target_positions(120, 120, 120, 120)

    while True:
        orientation = camera_thread.get_orientation()
        theta_x = orientation[1] + x_offset
        theta_y = orientation[0] + y_offset

        if theta_x is None or theta_y is None:
            print("Orientation data not available yet.")
            time.sleep(0.05)
            continue

        with ref_lock:
            if ref_theta is None:
                time.sleep(0.05)
                continue
            ref_theta_x, ref_theta_y = ref_theta
        e_x = theta_x - ref_theta_x
        e_y = theta_y - ref_theta_y

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

        vel_x = min(max(int(kp * abs(e_x)), min_velocity), 255)
        vel_y = min(max(int(kp * abs(e_y)), min_velocity), 255)
        dir_y = 2
        print(f"e_x: {e_x}, theta_x: {theta_x}, ref: {ref_theta_x}, dir_x: {dir_x}, vel_x: {vel_x}")
        arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)

def axisControl(ref):
    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 5000  # Proportional gain for the control loop
    tol = 0.001

    orientation = camera_thread.get_orientation()
    theta_x = orientation[1] + x_offset
    theta_y = orientation[0] + y_offset

    if theta_x is None or theta_y is None:
        print("Orientation data not available yet.")
        return
    
    e_x = theta_x - ref[0]
    e_y = theta_y - ref[1]
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

    vel_x = min(max(int(kp * abs(e_x)), min_velocity), 255)
    vel_y = min(max(int(kp * abs(e_y)), min_velocity), 255)
    dir_y = 2
    #print(f"e_x: {e_x}, theta_x: {theta_x}, dir_x: {dir_x}, vel_x: {vel_x}")
    arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)

def posControl(center, prev_center, e_prev, t_prev, edot_prev, ref=(200, 200), tol=10):
    kp = 0.0001  # Proportional gain for position control
    kd = 0.00015  # Derivative gain for position control
    deadzone_pos_tol = 30
    deadzone_vel_tol = 3
    deadzone_tilt = 0.0
    pos_tol = 10
    vel_tol = 1

    if prev_center is not None:
        if abs(np.linalg.norm(np.array(center) - np.array(prev_center))) > 300:
            print("Large jump detected, resetting position control.")
            return None, None, None
        
    e_x = ref[0] - center[0]
    e_y = ref[1] - center[1]
        
    edot_x = 0
    edot_y = 0
    alpha = 0.5
    if e_prev is not None and t_prev is not None:
        dt = time.time() - t_prev
        if dt > 0.0001:  # Avoid division by zero
            edot_x = (e_x - e_prev[0]) / dt
            edot_y = (e_y - e_prev[1]) / dt
            edot_x = alpha * edot_x + (1 - alpha) * edot_prev[0]
            edot_y = alpha * edot_y + (1 - alpha) * edot_prev[1]

    if abs(e_x) < pos_tol and abs(edot_x) < vel_tol:
        # Ball is at target → STOP
        theta_x = 0
        return None, None, None
    elif abs(e_x) < deadzone_pos_tol and abs(edot_x) < deadzone_vel_tol:
        # Ball is close, but needs help moving → ESCAPE DEAD ZONE
        theta_x = np.sign(e_x) * deadzone_tilt
    else:
        # Ball is far → USE CONTROL
        theta_x = (kp * e_x  + kd * edot_x)

    if abs(e_y) < pos_tol and abs(edot_y) < vel_tol:
        # Ball is at target → STOP
        theta_y = 0
    elif abs(e_y) < deadzone_pos_tol and abs(edot_x) < deadzone_vel_tol:
        # Ball is close, but needs help moving → ESCAPE DEAD ZONE
        theta_y = -np.sign(e_y) * deadzone_tilt
    else:
        # Ball is far → USE CONTROL
        theta_y = -(kp * e_y  + kd * edot_y)
        
    if abs(e_x) < pos_tol and abs(edot_x) < vel_tol and abs(e_y) < tol and abs(edot_y) < vel_tol:
    	print("Target reached!")
    	return None, None, None

    print(f"e_x: {e_x}, theta_x: {theta_x}, theta_y: {theta_y}, edot_x: {edot_x}, edot_y: {edot_y}")
    axisControl((theta_x, theta_y))
    return (e_x, e_y), time.time(), (edot_x, edot_y)

def posControlMultithread(center, prev_center, e_prev, t_prev, edot_prev, ref=(200, 200), tol=1):
    global ref_theta

    kp = 0.0004  # Proportional gain for position control
    kd = 0.0004  # Derivative gain for position control

    if prev_center is not None:
        if abs(np.linalg.norm(np.array(center) - np.array(prev_center))) > 300:
            print("Large jump detected, resetting position control.")
            return None, None
        
    e_x = ref[0] - center[0]
    e_y = ref[1] - center[1]
        
    edot_x = 0
    edot_y = 0
    alpha = 0.5
    if e_prev is not None and t_prev is not None:
        dt = time.time() - t_prev
        if dt > 0.0001:  # Avoid division by zero
            edot_x = (e_x - e_prev[0]) / dt
            edot_y = (e_y - e_prev[1]) / dt
            edot_x = alpha * edot_x + (1 - alpha) * edot_prev[0]
            edot_y = alpha * edot_y + (1 - alpha) * edot_prev[1]

    with ref_lock:
        ref_theta = (-(kp * e_x  + kd * edot_x), -(kp * e_y  + kd * edot_y))

    #print(f"e_x: {e_x}, ref_tehta_x: {ref_theta[0]}, ref_theta_y: {ref_theta[1]}, edot_x: {edot_x}, edot_y: {edot_y}")
    return (e_x, e_y), time.time(), (edot_x, edot_y)

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
        orientation = camera_thread.get_orientation()
        theta_x = orientation[1] + x_offset
        theta_y = orientation[0] + y_offset
        print(orientation)

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
horizontal(0.0015)

#axis_thread = threading.Thread(target=axisControlMultithread, daemon=True)
#axis_thread.start()

frame = camera_thread.latest_frame
center = None
prev_center = None
limit = time.time() + 100
e_prev = None
t_prev = None
edot_prev = (0, 0)
while time.time() < limit:
    frame = camera_thread.latest_frame
    if frame is None:
        continue
    #frame = cv2.resize(frame, (320, 240))  # Resize to a standard size if needed
    center, radius, masked_frame = testing.ball_recognition.detect_red_ball_frame(frame, center)
    if center is None:
        print("No ball detected, skipping frame.")
        continue
    cv2.circle(frame, center, 10, (0, 255, 0), 4)
    center = (center[1], center[0])  # Convert to (x, y) format for consistency
    print(f"Center: {center}")
    if limit - time.time() < 95:
        e_prev, t_prev, edot_prev = posControl(center, prev_center, e_prev, t_prev, edot_prev)
    prev_center = center

    cv2.imshow("Test Image", frame)
    cv2.imshow("Masked Frame", masked_frame)
    cv2.waitKey(1)  # This is necessary for the window to update

cv2.destroyAllWindows()

