from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import run_controller_2

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
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

while mqtt_client.jetson_state != "0.0":
    print("Waiting for handshake from Pi...")
    time.sleep(1)

print("Connected!")

if mqtt_client.jetson_state == "1.0":
    run_controller_2.main()