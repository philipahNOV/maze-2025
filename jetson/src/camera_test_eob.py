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

print("All Threads Initialized")

#--------------
#TESTING AREA
#The state is hardcoded to "0.0" to not rely on the Pi for testing purposes
print("TEST")
img = camera_thread.latest_frame
camthread = CameraThread()

try:
    while True:
        last_frame = camthread.get_latest_frame()
        if last_frame is not None:
            cv2.imshow("Live Frame", last_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.01)  # Add a small delay to prevent high CPU usage

except KeyboardInterrupt:
    pass
finally:
    # Stop the camera thread and close OpenCV windows
    camthread.stop()
    cv2.destroyAllWindows()

#--------------

try:
    camera_thread.stop()
    print("CameraThread stopped")
except Exception as e:
    print(f"Failed to stop CameraThread: {e}")

print("All Threads Stopped")
print("Exiting...")
