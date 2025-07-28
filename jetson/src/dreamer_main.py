import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'jetson', 'src'))

from mqtt.mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import app.run_controller_main as run_controller
import control.position_controller as position_controller
from tracking.tracker_service import TrackerService
from dreamerv3_controller import DreamerV3Controller
import queue
import threading
import time
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


print("Starting DreamerV3 Maze Learning System...")

# Initialize Arduino connection
try:
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
    time.sleep(5)
except Exception as e:
    print(e)
    exit(1)

# Initialize MQTT client (keeping for compatibility)
try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

print("Waiting for handshake from Pi...")
while not mqtt_client.handshake_complete:
    time.sleep(1)

# Initialize tracker service
tracker_service = TrackerService(model_path="tracking/v8-291.pt")
tracker_service.camera.init_camera()

print("Camera and tracking initialized")

# Create models directory if it doesn't exist
os.makedirs("models", exist_ok=True)

# Initialize DreamerV3 controller
dreamer_controller = DreamerV3Controller(arduino_thread, tracker_service)

print("DreamerV3 controller initialized")

# Start the learning process
try:
    print("Starting autonomous maze exploration and learning...")
    dreamer_controller.start()
    
    # Keep the main thread alive
    while True:
        time.sleep(1)
        if not dreamer_controller.running:
            break
            
except KeyboardInterrupt:
    print("\nShutting down...")
    dreamer_controller.stop()
    
except Exception as e:
    print(f"Error in main loop: {e}")
    dreamer_controller.stop()

print("DreamerV3 system shut down complete")
