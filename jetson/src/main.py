from mqtt.mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import control.position_controller as position_controller
from tracking.tracker_service import TrackerService
import time
from app.finite_state_machine import HMIController
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
tracking_config = config["tracking"]
path_finding_config = config["path_finding"]
print("Config loaded successfully.")

tracker_service = TrackerService(
    model_path=tracking_config["model_path"],
    tracking_config=tracking_config,
)

tracker_service.camera.init_camera()
controller = position_controller.Controller(
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