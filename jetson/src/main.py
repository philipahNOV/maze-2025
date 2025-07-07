from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import rl2
import pos2
from camera.tracker_service import TrackerService
import queue
import threading
import time


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
    time.sleep(5)
except Exception as e:
    print(e)
    exit(1)

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

tracker_service = TrackerService(model_path="camera/v8-291.pt")
tracker_service.camera.init_camera()

print("Waiting for handshake from Pi...")
while not mqtt_client.handshake_complete:
    time.sleep(1)

print("Connected!")

tracker_service.start_tracker()
while tracker_service.tracker is None:
    time.sleep(0.1)

tracker = tracker_service.tracker
ballPos = tracker.get_position()
while ballPos is None:
    ballPos = tracker.get_position()
    time.sleep(0.1)

mqtt_client.client.publish("pi/info", "ball_found")