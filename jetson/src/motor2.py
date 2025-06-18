from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import time
import math

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

time.sleep(5)  # Allow time for Arduino connection to stabilize
arduino_thread.send_target_positions(120, 120, 120, 120)
time.sleep(1)
arduino_thread.send_target_positions(777, 777, 777, 777)

# Circular motion loop
center_x = 120      # Offset for channel 0
center_y = 120      # Offset for channel 1
radius = 100        # Radius of the circular motion
steps = 200         # Number of steps in the circle
delay = 0.02        # Delay between each step

for i in range(steps):
    angle = 2 * math.pi * i / steps
    x = center_x + int(radius * math.cos(angle))
    y = center_y + int(radius * math.sin(angle))
    
    # Send circular motion on channels 0 and 1, fixed values on 2 and 3
    arduino_thread.send_target_positions(x, y, 201, 799)
    time.sleep(delay)

# Return to neutral position
arduino_thread.send_target_positions(120, 120, 120, 120)
