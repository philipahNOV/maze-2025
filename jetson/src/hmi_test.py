from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection_test import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import run_controller_3
import positionController_2
import testing.yolov1.hsv3 as tracking
import queue
import threading

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
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
    time.sleep(10)
except Exception as e:
    print(e)
    exit(1)

try:
    mqtt_client = initialize_component(MQTTClientJetson, "MQTTClientJetson")
except Exception as e:
    print(e)
    exit(1)

while mqtt_client.jetson_state != "0.0":
    print("Waiting for handshake from Pi...")
    time.sleep(1)

print("Connected!")

tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
tracker.start()
controller = positionController_2.Controller(arduino_thread, tracker)

while True:
    #command = mqtt_client.command

    #if not command:
    #    time.sleep(0.1)
    #    continue

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
        mqtt_client.command = None

    elif command == "Elevator":
        arduino_thread.send_target_positions(0, 0, "Get_ball")
        mqtt_client.command = None
    elif command == "Idle":
        arduino_thread.send_target_positions(0, 0, "Idle")
        mqtt_client.command = None
    elif command == "Get_pid":
        pid_str = (
            "PID:" + str(controller.x_offset) + "," + str(controller.y_offset) + "," + str(controller.kp_x)
            + "," + str(controller.kp_y) + "," + str(controller.kd_x) + "," + str(controller.kd_y)
            + "," + str(controller.ki_x) + "," + str(controller.ki_y)
        )
        
    elif command == "Control":
        #run_controller_3.main(tracker, controller, mqtt_client)
        #mqtt_client.command = None
        if not hasattr(mqtt_client, "control_thread") or not mqtt_client.control_thread.is_alive():
            mqtt_client.stop_control = False
            mqtt_client.control_thread = threading.Thread(
                target=run_controller_3.main,
                args=(tracker, controller, mqtt_client),
                daemon=True
            )
            mqtt_client.control_thread.start()
        else:
            print("[INFO] Control loop already running")
        mqtt_client.command = None
    elif command == "Horizontal":
        controller.horizontal()
        mqtt_client.command = None
    elif command.startswith("Motor_"):
        dir = command.split("_")[1]
        if dir == "stop":
            arduino_thread.send_target_positions(0, 0)
            mqtt_client.command = None
            continue
        speed = int(command.split("_")[2])
        if dir == "up":
            arduino_thread.send_target_positions(speed, 0)
        if dir == "down":
            arduino_thread.send_target_positions(-speed, 0)
        if dir == "left":
            arduino_thread.send_target_positions(0, speed)
        if dir == "right":
            arduino_thread.send_target_positions(0, -speed)

    time.sleep(0.01)
