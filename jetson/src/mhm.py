import numpy as np
import threading
import time
import cv2

from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import testing.ball_recognition
import testing.yolov1.hsv2 as tracking
from manual_part.manuel_main import elManuel

ref_lock = threading.Lock()
ref_theta = None
e_prev = None
t_prev = None
edot_prev = (0, 0)
prev_center = None

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
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
except Exception as e:
    print(e)
    exit(1)

def axisControlMultithread():
    x_offset = 0
    y_offset = 0
    min_velocity = 22
    kp = 5000
    tol = 0
    arduino_thread.send_target_positions(120, 120, 120, 120)

    while True:
        orientation = camera_thread.get_orientation()
        theta_x = orientation[1] + x_offset
        theta_y = orientation[0] + y_offset

        if theta_x is None or theta_y is None:
            time.sleep(0.05)
            continue

        with ref_lock:
            if ref_theta is None:
                time.sleep(0.05)
                continue
            ref_theta_x, ref_theta_y = ref_theta

        e_x = theta_x - ref_theta_x
        e_y = theta_y - ref_theta_y

        dir_x = 2 if abs(e_x) < tol else (3 if e_x > 0 else 1)
        dir_y = 2 if abs(e_y) < tol else (3 if e_y > 0 else 1)

        vel_x = min(max(int(kp * abs(e_x)), min_velocity), 255)
        vel_y = min(max(int(kp * abs(e_y)), min_velocity), 255)
        dir_y = 2  # Force Y to hold position?

        arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)

def posControlMultithread(center, prev_center, e_prev, t_prev, edot_prev, ref=(200, 200), tol=1):
    global ref_theta
    if prev_center is not None:
        if np.linalg.norm(np.array(center) - np.array(prev_center)) > 300:
            print("Large jump detected")
            return None, None

    e_x = ref[0] - center[0]
    e_y = ref[1] - center[1]

    edot_x = edot_y = 0
    alpha = 0.5
    if e_prev is not None and t_prev is not None:
        dt = time.time() - t_prev
        if dt > 0.0001:
            edot_x = (e_x - e_prev[0]) / dt
            edot_y = (e_y - e_prev[1]) / dt
            edot_x = alpha * edot_x + (1 - alpha) * edot_prev[0]
            edot_y = alpha * edot_y + (1 - alpha) * edot_prev[1]

    with ref_lock:
        ref_theta = (-(0.0004 * e_x + 0.0004 * edot_x), -(0.0004 * e_y + 0.0004 * edot_y))

    return (e_x, e_y), time.time(), (edot_x, edot_y)

if __name__ == "__main__":
    tracker = tracking.BallTracker("testing/yolov1/best.pt")
    tracker.start()

    axis_thread = threading.Thread(target=axisControlMultithread, daemon=True)
    axis_thread.start()

    limit = time.time() + 100

    try:
        while time.time() < limit:
            pos = tracker.get_position()
            if pos:
                print(f"Ball position: x={pos[0]}, y={pos[1]}")
                center = pos[::-1] 
                e_prev, t_prev, edot_prev = posControlMultithread(
                    center, prev_center, e_prev, t_prev, edot_prev, ref=(120, 180)
                )
                prev_center = center
            else:
                print("Ball not found.")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Stopping...")

    tracker.stop()
    cv2.destroyAllWindows()
