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

#--------------
#TESTING AREA
#The state is hardcoded to "0.0" to not rely on the Pi for testing purposes
time.sleep(3)
print("TEST")
print("Camera thread started")
img = camera_thread.latest_frame
n = 100
while(n>0):
    img = camera_thread.latest_frame
    if img is not None:
        cv2.imshow("Test Image", img)
        cv2.waitKey(1)
    n -= 1
cv2.destroyAllWindows()

img = None
if img is not None:
    print("Image received from camera thread")
    cv2.imwrite("test_image.jpg", img)  # Save the image for verification
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No image received from camera thread.")
#--------------