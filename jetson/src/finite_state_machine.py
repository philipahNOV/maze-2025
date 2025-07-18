from enum import Enum, auto
import time
from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
from camera.tracker_service import TrackerService
import utility_threads
from image_controller import ImageController
from image_controller import ImageSenderThread
import position_controller
from utility_functions import is_in_elevator, remove_withing_elevator
import run_controller_main
import threading
import os
import sys
from typing import Dict, Any


class SystemState(Enum):
    BOOTING = auto()
    MAIN_SCREEN = auto()
    INFO_SCREEN = auto()
    NAVIGATION = auto()
    LOCATING = auto()
    AUTO_PATH = auto()
    CUSTOM_PATH = auto()
    CONTROLLING = auto()

class HMIController:
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection, mqtt_client: MQTTClientJetson, config: Dict[str, Any]):
        self.state = SystemState.BOOTING
        self.tracking_service = tracking_service
        self.arduino_thread = arduino_thread
        self.mqtt_client = mqtt_client
        self.loop_control = False
        self.ball_finder = None
        self.path = None
        self.path_lookahead = None
        self.image_controller = ImageController(config)
        self.image_thread = None
        self.controller = position_controller.Controller(arduino_thread, tracking_service, config=config)
        self.controller_thread = None
        self.stop_controller_event = threading.Event()
        self.custom_goal = None
        self.path_thread = None
        self.disco_mode = 0
        self.disco_thread = None
        self.loop_path = False
        self.config = config

    def restart_program(self):
        print("Restart requested...")
        self.stop_threads()
        python = sys.executable
        script = os.path.abspath(sys.argv[0])
        print(f"Launching new process: {python} {script}")
        #subprocess.Popen([python, script], start_new_session=True)
        #sys.exit(0)
        os.execv(python, [python, script])

    def stop_threads(self):
        try:
            self.mqtt_client.stop()
        except Exception as e:
            print(f"Error stopping MQTT client: {e}")
        if self.image_thread is not None:
            self.image_thread.stop()
            self.image_thread.join()
            self.image_thread = None
        if self.controller_thread is not None and self.controller_thread.is_alive():
            self.stop_controller_event.set()
            self.controller_thread.join()
            self.controller_thread = None
        if self.path_thread is not None and self.path_thread.is_alive():
            self.path_thread.stop()
            self.path_thread = None
        if self.ball_finder is not None:
            self.ball_finder.stop()
            self.ball_finder = None
        if self.disco_thread is not None:
            self.disco_thread.stop()
            self.disco_thread.join()
            self.disco_thread = None
        self.tracking_service.stop_tracker()
        self.tracking_service.camera.close()

    def stop_program(self):
        print("Stopping program...")
        self.stop_threads()
        sys.exit(0)

    def on_ball_found(self):
        if is_in_elevator(self.config, self.tracking_service.get_ball_position()):
            self.mqtt_client.client.publish("pi/info", "ball_found")
        else:
            self.ball_finder = utility_threads.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
            self.ball_finder.start_ball_check()

    def on_path_found(self, path, path_lookahead):
        self.path = path
        self.path_lookahead = path_lookahead
        if self.path is not None:
            self.mqtt_client.client.publish("pi/info", "path_found")
        else:
            self.mqtt_client.client.publish("pi/info", "path_not_found")
            if self.image_thread is None:
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
            return
        self.path = remove_withing_elevator(self.config, self.path, radius=self.config['camera'].get('elevator_radius', 60))
        self.path_lookahead = remove_withing_elevator(self.config, self.path_lookahead, radius=self.config['camera'].get('elevator_radius', 60)+20)
        self.image_controller.set_new_path(self.path)
        if self.image_thread is None:
            self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
            self.image_thread.start()
    
    def stop_controller(self):
        if self.controller_thread is not None and self.controller_thread.is_alive():
            self.stop_controller_event.set()
            self.controller_thread.join()
            self.controller_thread = None
        self.arduino_thread.send_speed(0, 0)
        threading.Thread(target=self.controller.horizontal, daemon=True).start()

    def start_path_finding(self, custom_goal=None):
        if custom_goal is not None:
            goal = (custom_goal[1], custom_goal[0])
        else:
            goal = (49, 763)

        self.path_thread = utility_threads.PathFindingThread(
            tracking_service=self.tracking_service,
            goal=goal,
            on_path_found=self.on_path_found,
            config=self.config,
        )
        self.path_thread.start()

    def on_command(self, cmd):
        print(f"[FSM] State: {self.state.name} | Command received: {cmd}")
        
        # --- BOOTING STATE ---
        if self.state == SystemState.BOOTING:
            if cmd == "booted":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                if self.controller.elevator_state is not None:
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.2)
                    self.arduino_thread.send_elevator(0)
                    #self.controller.elevator_state = "up"
        
        # --- MAIN_SCREEN STATE ---
        elif self.state == SystemState.MAIN_SCREEN:
            if cmd == "Info":
                self.state = SystemState.INFO_SCREEN
                print("[FSM] Transitioned to INFO_SCREEN")

            elif cmd == "Navigate":
                if self.disco_thread is not None:
                    self.disco_thread.stop()
                    self.disco_thread.join()
                    self.disco_thread = None
                self.state = SystemState.NAVIGATION
                print("[FSM] Transitioned to NAVIGATION")

            elif cmd == "Disco":
                if self.disco_thread is not None:
                    self.disco_thread.toggle_mode()
            elif cmd.startswith("Locate"):
                # Transition to LOCATING and start ball tracking
                self.state = SystemState.LOCATING
                if self.controller.elevator_state is not None:
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.2)
                    self.arduino_thread.send_elevator(0)
                    #self.controller.elevator_state = "up"
                if self.disco_thread is not None:
                    self.disco_thread.stop()
                    self.disco_thread.join()
                    self.disco_thread = None
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.start_tracker()
                self.ball_finder = utility_threads.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()
        
        # --- INFO_SCREEN STATE ---
        elif self.state == SystemState.INFO_SCREEN:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")

        # --- NAVIGATION STATE ---
        elif self.state == SystemState.NAVIGATION:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")

            elif cmd.startswith("Locate"):
                if "loop" in cmd:
                    self.loop_control = True
                elif "dont_loop" in cmd:
                    self.loop_control = False

                if cmd.endswith("safe"):
                    self.controller.lookahead = False
                elif cmd.endswith("speed"):
                    self.controller.lookahead = True

                # Transition to LOCATING and start ball tracking
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.start_tracker()
                self.ball_finder = utility_threads.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()

            elif cmd == "Elevator":
                pass

            elif cmd == "Horizontal":
                threading.Thread(target=self.controller.horizontal, daemon=True).start()

        # --- LOCATING STATE ---
        elif self.state == SystemState.LOCATING:
            if cmd == "AutoPath":
                # Transition to AUTO_PATH and start pathfinding
                self.state = SystemState.AUTO_PATH
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to AUTO_PATH")
                self.start_path_finding()

            elif cmd == "CustomPath":
                # Transition to CUSTOM_PATH and set up image thread
                self.state = SystemState.CUSTOM_PATH
                print("[FSM] Transitioned to CUSTOM_PATH")
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()

            elif cmd == "Back":
                # Cleanup ball finder and tracking before going back
                self.tracking_service.stop_tracker()
                if self.ball_finder:
                    self.ball_finder.stop()
                    self.ball_finder = None
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")
            elif cmd == "BallFound":
                # Transition to CUSTOM_PATH and set up image thread
                self.state = SystemState.CUSTOM_PATH
                print("[FSM] Transitioned to CUSTOM_PATH")
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()

        # --- AUTO_PATH STATE ---
        elif self.state == SystemState.AUTO_PATH:
            if cmd == "Back":
                # Stop controller, tracker, threads, and reset path
                self.stop_controller()
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                if self.path_thread is not None:
                    self.path_thread.stop()
                    self.path_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to NAVIGATION")

            elif cmd.startswith("Locate"):
                # Go back to locating mode
                self.stop_controller()
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.tracking_service.start_tracker()
                self.ball_finder = utility_threads.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()
                if self.controller_thread is not None and self.controller_thread.is_alive():
                    self.controller_thread.join()
                    self.controller_thread = None

            elif cmd == "Start":
                # Start the control loop if path is ready
                if self.path is None:
                    print("[FSM] No path found, cannot start.")
                else:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None

                    self.stop_controller_event.clear()
                    if self.controller_thread is None or not self.controller_thread.is_alive():
                        self.controller_thread = threading.Thread(
                            target=run_controller_main.main,
                            args=(self.tracking_service, self.controller, self.mqtt_client, self.path, self.image_controller, self.stop_controller_event, self.config),
                            daemon=True
                        )
                        self.controller_thread.start()
                    else:
                        print("[INFO] Control loop already running")
                    self.state = SystemState.CONTROLLING
                    print("[FSM] Transitioned to CONTROLLING")

            elif cmd == "timeout":
                # Handle inactivity timeout
                print("[FSM] Timeout command received in AUTO_PATH")
                self.stop_controller()
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to MAIN_SCREEN")

        # --- CUSTOM_PATH STATE ---
        elif self.state == SystemState.CUSTOM_PATH:
            if cmd == "Back":
                self.stop_controller()
                self.tracking_service.stop_tracker()
                if self.path_thread is not None:
                    self.path_thread.stop()
                    self.path_thread = None
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to MAIN_SCREEN")

            elif cmd.startswith("Goal_set:"):
                # Parse and set custom goal coordinates
                coords = cmd.split(":")[1]
                x, y = map(int, coords.split(","))
                self.custom_goal = (x, y)

            elif cmd == "CalculatePath":
                # Initiate custom pathfinding to goal
                if self.custom_goal is None:
                    print("[FSM] No custom goal set, cannot calculate path.")
                else:
                    self.start_path_finding(custom_goal=self.custom_goal)

            elif cmd.startswith("Start"):
                # Start controller thread with custom path
                if self.path is None:
                    print("[FSM] No path found, cannot start.")
                else:
                    if cmd.endswith("Safe"):
                        self.controller.lookahead = False
                        path = self.path
                    elif cmd.endswith("Speed"):
                        self.controller.lookahead = True
                        path = self.path_lookahead

                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None

                    self.stop_controller_event.clear()
                    if self.controller_thread is None or not self.controller_thread.is_alive():
                        self.controller_thread = threading.Thread(
                            target=run_controller_main.main,
                            args=(self.tracking_service, self.controller, self.mqtt_client, path, self.image_controller, self.stop_controller_event, self.config),
                            daemon=True
                        )
                        self.controller_thread.start()
                    else:
                        print("[INFO] Control loop already running")
                    self.state = SystemState.CONTROLLING
                    print("[FSM] Transitioned to CONTROLLING")

            elif cmd == "timeout":
                # Timeout handling for custom path mode
                print("[FSM] Timeout command received in AUTO_PATH")
                self.stop_controller()
                self.state = SystemState.MAIN_SCREEN
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")

        # --- CONTROLLING STATE ---
        elif self.state == SystemState.CONTROLLING:
            if cmd == "Back":
                # Stop controller and clean up threads and resources
                self.stop_controller()
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")

            elif cmd == "timeout":
                print("[FSM] Timeout command received in CONTROLLING")
                self.stop_controller()
                self.state = SystemState.MAIN_SCREEN
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")

            elif cmd.startswith("Loop"):
                if cmd.endswith("On"):
                    self.controller.looping = True
                elif cmd.endswith("Off"):
                    self.controller.looping = False
        if cmd == "Restart":
            self.restart_program()
        elif cmd == "Exit":
            self.stop_program()