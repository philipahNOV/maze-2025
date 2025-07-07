from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
import run_controller
import position_controller
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

print("Waiting for handshake from Pi...")
while not mqtt_client.handshake_complete:
    time.sleep(1)

print("Connected to Pi!")

tracker_service = TrackerService(model_path="YOLO_tracking/best.pt")
tracker_service.camera.init_camera()

controller = position_controller.Controller(arduino_thread, tracker_service)

color = "white"

while True:
    try:
        command = mqtt_client.command_queue.get_nowait()
    except queue.Empty:
        time.sleep(0.01)
        continue

    if command.startswith("PID:"):
        params = command.split(":")[1].split(",")
        params.pop(0)
        for i in range(len(params)):
            if params[i] != "pass":
                params[i] = float(params[i])
        controller.set_pid_parameters(params)

    elif command == "Elevator":
        arduino_thread.send_get_ball()

    elif command == "Idle":
        if color == "white":
            arduino_thread.send_color(255, 0, 0)
            color = "red"
        elif color == "red":
            arduino_thread.send_color(255, 255, 255)
            color = "white"

    elif command == "Get_pid":
        pid_str = (
            "PID:" + str(controller.x_offset) + "," + str(controller.y_offset) + "," + str(controller.kp_x)
            + "," + str(controller.kp_y) + "," + str(controller.kd_x) + "," + str(controller.kd_y)
            + "," + str(controller.ki_x) + "," + str(controller.ki_y) + "," + str(controller.kf_min) + "," + str(controller.kf_max)
        )
        mqtt_client.client.publish("pi/command", pid_str)

    elif command == "Control":
        mqtt_client.stop_control = False

        if not tracker_service.started:
            tracker_service.start_tracker()

        # Start control loop thread if not running
        if not hasattr(mqtt_client, "control_thread") or not mqtt_client.control_thread.is_alive():
            mqtt_client.control_thread = threading.Thread(
                target=run_controller.main,
                args=(tracker_service, controller, mqtt_client),
                daemon=True
            )
            mqtt_client.control_thread.start()
        else:
            print("[INFO] Control loop already running")

    elif command == "Horizontal":
        controller.horizontal()

    elif command.startswith("Motor_"):
        dir = command.split("_")[1]
        if dir == "stop":
            arduino_thread.send_speed(0, 0)
            continue
        speed = int(command.split("_")[2])
        if dir == "up":
            arduino_thread.send_speed(speed, 0)
        if dir == "down":
            arduino_thread.send_speed(-speed, 0)
        if dir == "left":
            arduino_thread.send_speed(0, speed)
        if dir == "right":
            arduino_thread.send_speed(0, -speed)

    time.sleep(0.015)