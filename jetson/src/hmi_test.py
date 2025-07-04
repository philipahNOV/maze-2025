from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection_test import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import run_controller_3
import positionController_2
import run_controller_3_LOS
import testing.yolov1.hsv3 as tracking
import queue
import threading
import maze_solver
import estimate_dynamics

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

def get_frame():
    # === Display frame from control thread, if any ===
    if hasattr(mqtt_client, "control_thread") and mqtt_client.control_thread.is_alive():
        try:
            frame = run_controller_3.frame_queue.get_nowait()

            # Check if frame has meaningful content
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if cv2.countNonZero(gray) > 1000 and frame.std() > 5:
                print(f"[MAIN] Got frame with mean: {frame.mean():.2f}")
                return frame
        except queue.Empty:
            pass
    else:
        try:
            cv2.destroyWindow("Ball & Marker Tracking")
        except:
            pass
    return None

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

def get_frame(mqtt_client, frame_queue):
    # === Display frame from control thread, if any ===
    if hasattr(mqtt_client, "control_thread") and mqtt_client.control_thread.is_alive():
        try:
            frame = frame_queue.get_nowait()

            # Check if frame has meaningful content
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if cv2.countNonZero(gray) > 1000 and frame.std() > 5:
                print(f"[MAIN] Got frame with mean: {frame.mean():.2f}")
                return frame
            else:
                print("[DEBUG] Frame rejected: too black or flat")
        except queue.Empty:
            print("[DEBUG] Frame queue empty")
    else:
        try:
            cv2.destroyWindow("Ball & Marker Tracking")
        except:
            pass
    return None

print("Waiting for handshake from Pi...")
while not mqtt_client.handshake_complete:
    time.sleep(1)

print("Connected to Pi!")

tracker = tracking.BallTracker(model_path="testing/yolov1/best.pt")
tracker.start()
controller = positionController_2.Controller(arduino_thread, tracker)

while True:

    frame = get_frame(mqtt_client, run_controller_3.frame_queue)
    if frame is not None:
        cv2.imshow("Ball & Marker Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

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
            + "," + str(controller.ki_x) + "," + str(controller.ki_y) + "," + str(controller.kf_min) + "," + str(controller.kf_max)
        )
        mqtt_client.client.publish("pi/command", pid_str)
        
    elif command == "Control":
        #run_controller_3.main(tracker, controller, mqtt_client)
        #mqtt_client.command = None
        mqtt_client.stop_control = False
        if not hasattr(mqtt_client, "control_thread") or not mqtt_client.control_thread.is_alive():
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
    elif command == "Maze":
        maze_solver.main(tracker)
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

    time.sleep(0.015)
