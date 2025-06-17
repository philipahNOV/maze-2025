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
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
except Exception as e:
    print(e)
    exit(1)

try:
    camera_thread = initialize_component(CameraThread, "CameraThread")
    camera_thread.start()
except Exception as e:
    print(e)
    exit(1)

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)


try:
    automatics = Automatic(mqtt_client, camera_thread, arduino_thread)
    print("Automatic initialized")
except Exception as e:
    print(f"Failed to initialize Automatic: {e}")
    exit(1)

try:
    math_client = State1_3(mqtt_client, camera_thread, arduino_thread)
    print("Math_main initialized")
except Exception as e:
    print(f"Failed to initialize Math_main: {e}")
    exit(1)

try:
    ai_client = State2_3(mqtt_client, camera_thread, arduino_thread)
    print("Ai_main initialized")
except Exception as e:
    print(f"Failed to initialize Ai_main: {e}")
    exit(1)



while mqtt_client.jetson_state != "0.0":
    print("Waiting for handshake from Pi...")
    time.sleep(1)

print("All Threads Initialized")

#--------------
#TESTING AREA
#The state is hardcoded to "0.0" to not rely on the Pi for testing purposes
print("TEST")
img = camera_thread.latest_frame
if img is not None:
    cv2.imshow("Test Image", img)
    cv2.waitKey(0)

#--------------

while mqtt_client.jetson_state != "-1":
    if mqtt_client.jetson_state == "1.0":
        automatics.state_1_0()
    
    if mqtt_client.jetson_state == "2.0":
        elManuel(mqtt_client, camera_thread, arduino_thread)

try:
    camera_thread.stop()
    print("CameraThread stopped")
except Exception as e:
    print(f"Failed to stop CameraThread: {e}")

try:
    arduino_thread.stop()
    print("ArduinoConnection stopped")
except Exception as e:
    print(f"Failed to stop ArduinoConnection: {e}")

try:
    mqtt_client.stop()
    print("MQTTClientJetson stopped")
except Exception as e:
    print(f"Failed to stop MQTTClientJetson: {e}")

print("All Threads Stopped")
print("Exiting...")
