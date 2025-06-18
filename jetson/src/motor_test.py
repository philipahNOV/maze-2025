from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import time

from manual_part.manuel_main import elManuel

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

time.sleep(6)  # Allow time for Arduino to initialize
print("Arduino initialized")
arduino_thread.send_target_positions(1, 3, 100, 100)
time.sleep(10)
arduino_thread.send_target_positions(3, 1, 100, 100)