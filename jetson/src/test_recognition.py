from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import testing.ball_recognition

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
    print("Camera thread started")
except Exception as e:
    print(e)
    exit(1)

time.sleep(1.5)


frame = camera_thread.latest_frame
n = 200
center = None
while(n>0):
    frame = camera_thread.latest_frame
    if frame is None:
        time.sleep(1 / 15)
        continue
    center, radius = testing.ball_recognition.detect_red_ball_frame(frame, center)
    cv2.circle(frame, center, radius, (0, 255, 0), 4)
    cv2.imshow("Test Image", frame)
    n -= 1
    time.sleep(1 / 15)
cv2.destroyAllWindows()