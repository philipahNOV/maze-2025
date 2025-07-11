from mqtt_client_2 import MQTTClientJetson
from arduino_connection import ArduinoConnection
import pos2
from camera.tracker_service import TrackerService
import time
from fsm2 import HMIController
import yaml
from pathlib import Path


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

def load_config():
    # Get path to project root from the current file (assuming this file is in jetson/src/)
    project_root = Path(__file__).resolve().parents[2]  # up from src → jetson → maze-2025
    config_path = project_root / "config.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}")

    with config_path.open('r') as file:
        config = yaml.safe_load(file)
    return config


try:
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
    time.sleep(5)
except Exception as e:
    print(e)
    exit(1)

config = load_config()
print("Config loaded successfully.")

tracker_service = TrackerService(model_path="camera/v8-291.pt")
tracker_service.camera.init_camera()
controller = pos2.Controller(
    arduinoThread=arduino_thread,
    tracker=tracker_service,
    path_following=True,
    lookahead=False,
    config=config
)
controller.horizontal()

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

fsm = HMIController(tracker_service, arduino_thread, mqtt_client, config=config)
mqtt_client.fsm = fsm

print("Waiting for handshake from Pi...")
while not mqtt_client.handshake_complete:
    time.sleep(1)

print("Connected!")

#tracker_service.start_tracker()
#while tracker_service.tracker is None:
#    time.sleep(0.1)
#
#tracker = tracker_service.tracker
#ballPos = tracker_service.get_ball_position()
#while ballPos is None:
#    ballPos = tracker_service.get_ball_position()
#    time.sleep(0.1)
#
#print("Ball found at:", ballPos)
#
#mqtt_client.client.publish("pi/info", "ball_found")