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

time.sleep(3)
print("TEST")
print("Camera thread started")
img = camera_thread.latest_frame
n = 1
while(n>0):
    img = camera_thread.latest_frame
    if img is not None:
        cv2.imshow("Test Image", img)
        cv2.waitKey(1)
    n -= 1
cv2.destroyAllWindows()


# Define video codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # or 'MJPG', 'MP4V', etc.
fps = 30  # frames per second
frame_size = (640, 480)  # must match your actual frame size
out = cv2.VideoWriter('output.avi', fourcc, fps, frame_size)
n = 150
while n > 0:
    frame = camera_thread.latest_frame
    if frame is not None:
        resized_frame = cv2.resize(frame, frame_size)  # Ensure it matches frame_size
        out.write(resized_frame)  # Write frame to video
        cv2.imshow("Recording...", resized_frame)

        if cv2.waitKey(1) == 27:  # ESC key to exit early
            break
    time.sleep(1 / fps)
    n -= 1