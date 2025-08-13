from mqtt.mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import control.position_controller as position_controller
from tracking.tracker_service import TrackerService
import time
from app.finite_state_machine import HMIController
import yaml
from pathlib import Path
import os

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
    time.sleep(2)
except Exception as e:
    exit(1)

def load_temp_offsets(controller):
    offset_file = "offsets.txt"
    if os.path.exists(offset_file):
        with open(offset_file, "r") as f:
            line = f.read().strip()
            x_offset, y_offset = map(float, line.split(","))
    else:
        x_offset, y_offset = (controller.x_offset, controller.y_offset)
        with open(offset_file, "w") as f:
            f.write(f"{x_offset},{y_offset}")
    controller.x_offset = x_offset
    controller.y_offset = y_offset

config = load_config()
tracking_config = config["tracking"]
path_finding_config = config["path_finding"]

tracker_service = TrackerService(
    model_path=tracking_config["model_path"],
    tracking_config=tracking_config,
)

controller = position_controller.Controller(
    arduinoThread=arduino_thread,
    tracker=tracker_service,
    path_following=True,
    lookahead=False,
    config=config
)
load_temp_offsets(controller)
controller.horizontal()

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    exit(1)

fsm = HMIController(tracker_service, arduino_thread, mqtt_client, config=config)
mqtt_client.fsm = fsm

while not mqtt_client.handshake_complete:
    time.sleep(0.1)