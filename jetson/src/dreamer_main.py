import sys
import os
import yaml
import time
from mqtt.mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
from tracking.tracker_service import TrackerService
from dreamer_controller import DreamerMazeController
import utils.utility_threads as utility_threads


def load_config():
    """Load configuration from config.yaml"""
    # Look for config.yaml in the workspace root
    current_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_dir, '..', '..', 'config.yaml')
    if not os.path.exists(config_path):
        # Fallback to same directory as script
        config_path = os.path.join(current_dir, 'config.yaml')
    
    print(f"Loading config from: {config_path}")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


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


print("Starting Dreamer Maze Learning System...")

# Load configuration
config = load_config()
goal_pos = (config['game']['goal_position']['x'], config['game']['goal_position']['y'])
print(f"Target goal position: {goal_pos}")

# Initialize Arduino connection
try:
    arduino_connection = initialize_component(ArduinoConnection, "ArduinoConnection")
    time.sleep(5)
except Exception as e:
    print(f"Arduino initialization failed: {e}")
    exit(1)

# Initialize tracker service
print("Initializing camera and tracking...")
tracker_service = TrackerService(model_path="tracking/v8-291.pt")
tracker_service.camera.init_camera()

# Wait for tracker initialization like in PID controller
print("Waiting for YOLO initialization...")
while not tracker_service.is_initialized:
    time.sleep(0.1)
print("Camera and tracking initialized")

# Create models directory
os.makedirs("models", exist_ok=True)

# Initialize Dreamer controller
print("Initializing Dreamer controller...")
dreamer_controller = DreamerMazeController(arduino_connection, tracker_service, goal_position=goal_pos, config=config)
print("Dreamer controller initialized")

# Start tracker service like in PID controller
print("Starting tracker service...")
tracker_service.start_tracker()

# Initial platform leveling like PID controller
print("Performing initial platform leveling...")
dreamer_controller.horizontal()

# Run escape elevator sequence like PID controller
print("Running escape elevator sequence...")
escape_thread = utility_threads.EscapeElevatorThread(arduino_connection, dreamer_controller)
escape_thread.start()
time.sleep(escape_thread.duration)
dreamer_controller.horizontal()

# Start autonomous learning
try:
    print("Starting autonomous maze learning with Dreamer...")
    print("Press Ctrl+C to stop")
    
    dreamer_controller.start()
    
    # Keep main thread alive
    while True:
        time.sleep(1)
        if not dreamer_controller.running:
            break
            
except KeyboardInterrupt:
    print("\nShutdown requested...")
    
except Exception as e:
    print(f"Error in main loop: {e}")

finally:
    print("Stopping Dreamer controller...")
    dreamer_controller.stop()
    
    print("Stopping tracker service...")
    tracker_service.stop_tracker()
    
    print("Dreamer system shutdown complete")
