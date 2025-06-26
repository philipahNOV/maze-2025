from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection_test import ArduinoConnection
from zed_main import ZEDCamera
import run_controller_3
import positionController_2
import testing.yolov1.hsv3 as tracking

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
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
    time.sleep(10)
except Exception as e:
    print(e)
    exit(1)

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

while mqtt_client.jetson_state != "0.0":
    print("Waiting for handshake from Pi...")
    time.sleep(1)

print("Connected!")

zed_cam = ZEDCamera()
try:
    zed = zed_cam.init_camera()
    print("ZED camera opened.")
except RuntimeError as e:
    print(e)
    exit(1)

tracker    = None
controller = None

while True:
    frame = zed_cam.grab_frame()
    # if frame is not None:
    #     cv2.imshow("ZED Live", frame)
    #     cv2.waitKey(1)

    cmd = mqtt_client.command
    if not cmd:
        time.sleep(0.1)
        continue

    if cmd == "Idle":
        arduino_thread.send_target_positions(0, 0, "Idle")

    elif cmd == "Elevator":
        arduino_thread.send_target_positions(0, 0, "Get_ball")

    elif cmd == "Horizontal":
        if controller:
            controller.horizontal()

    elif cmd == "Control":
        if tracker is None:
            tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
            tracker.start()
            controller = positionController_2.Controller(arduino_thread, tracker)
            print("Tracker + Controller initialized.")
        run_controller_3.main(tracker, controller, mqtt_client)

    mqtt_client.command = None
    time.sleep(0.1)
