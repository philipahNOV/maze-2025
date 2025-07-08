from enum import Enum, auto
from mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
from camera.tracker_service import TrackerService
import light_controller
from image_controller import ImageController
from image_controller import ImageSenderThread
import pos2
from astar.astar import astar_downscaled
from astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
from astar.nearest_point import find_nearest_walkable
from astar.waypoint_sampling import sample_waypoints
from astar.draw_path import draw_path
import numpy as np
import run_controller_main
import threading


class SystemState(Enum):
    BOOTING = auto()
    MAIN_SCREEN = auto()
    INFO_SCREEN = auto()
    NAVIGATION = auto()
    LOCATING = auto()
    AUTO_PATH = auto()
    CUSTOM_PATH = auto()

class HMIController:
    def __init__(self, tracking_service: TrackerService, arduino_thread: ArduinoConnection, mqtt_client: MQTTClientJetson):
        self.state = SystemState.BOOTING
        self.tracking_service = tracking_service
        self.arduino_thread = arduino_thread
        self.mqtt_client = mqtt_client
        self.loop_control = False
        self.ball_finder = None
        self.path = None
        self.image_controller = ImageController()
        self.image_thread = None
        self.controller = pos2.Controller(arduino_thread, tracking_service)
        self.controller_thread = None
        self.stop_controller_event = threading.Event()

    def densify_path(self, path, factor=6):
        new_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            new_path.append(tuple(p1))
            for j in range(1, factor):
                interp = p1 + (p2 - p1) * (j / factor)
                new_path.append(tuple(interp.astype(int)))
        new_path.append(path[-1])
        return new_path

    def on_ball_found(self):
        self.mqtt_client.client.publish("pi/info", "ball_found")

    def on_path_found(self, path):
        print("TEST 1")
        self.path = path
        if self.controller.lookahead:
            self.path = self.densify_path(self.path, factor=3)
        self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
        self.image_thread.start()
    
    def stop_controller(self):
        if self.controller_thread is not None and self.controller_thread.is_alive():
            self.stop_controller_event.set()
            self.controller_thread.join()
            self.controller_thread = None

    def start_path_finding(self):
        frame = self.tracking_service.get_stable_frame()
        gray = get_dynamic_threshold(frame)
        binary_mask = create_binary_mask(gray)
        safe_mask = dilate_mask(binary_mask)
        #self.image_controller.frame = safe_mask
        #cv2.circle(self.image_controller.frame, (1030, 630), 70, 255, -1)
        #self.image_controller.crop_and_rotate_frame()
        #self.image_controller.send_frame_to_pi(self.mqtt_client)

        goal = (49, 763)
        path_thread = light_controller.PathFindingThread(
            tracking_service=self.tracking_service,
            goal=goal,
            on_path_found=self.on_path_found
        )
        path_thread.start()

    def on_command(self, cmd):
        print(f"[FSM] State: {self.state.name} | Command received: {cmd}")
        if self.state == SystemState.BOOTING:
            if cmd == "booted":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")
        elif self.state == SystemState.MAIN_SCREEN:
            if cmd == "Info":
                self.state = SystemState.INFO_SCREEN
                print("[FSM] Transitioned to INFO_SCREEN")

            elif cmd == "Navigate":
                self.state = SystemState.NAVIGATION
                print("[FSM] Transitioned to NAVIGATION")

        elif self.state == SystemState.INFO_SCREEN:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")

        elif self.state == SystemState.NAVIGATION:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")
            if cmd.startswith("Locate"):
                if "loop" in cmd:
                    self.loop_control = True
                elif "dont_loop" in cmd:
                    self.loop_control = False
                if cmd.endswith("safe"):
                    self.controller.lookahead = False
                elif cmd.endswith("speed"):
                    self.controller.lookahead = True
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.start_tracker()
                self.ball_finder = light_controller.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()

        elif self.state == SystemState.LOCATING:
            if cmd == "AutoPath":
                self.state = SystemState.AUTO_PATH
                print("[FSM] Transitioned to AUTO_PATH")
                self.start_path_finding()
            elif cmd == "CustomPath":
                self.state = SystemState.CUSTOM_PATH
                print("[FSM] Transitioned to CUSTOM_PATH")
            elif cmd == "Back":
                self.tracking_service.stop_tracker()
                if self.ball_finder:
                    self.ball_finder.stop()
                    self.ball_finder = None
                self.state = SystemState.NAVIGATION
                print("[FSM] Transitioned to NAVIGATION")
        
        elif self.state == SystemState.AUTO_PATH:
            if cmd == "Back":
                self.stop_controller()
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                print("[FSM] Transitioned to NAVIGATION")
            if cmd.startswith("Locate"):
                self.stop_controller()
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.stop_tracker()
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                self.path = None
                self.tracking_service.start_tracker()
                self.ball_finder = light_controller.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()
                if self.controller_thread is not None and self.controller_thread.is_alive():
                    self.controller_thread.join()
                    self.controller_thread = None
            if cmd == "Start":
                if self.path is None:
                    print("[FSM] No path found, cannot start.")
                else:
                    self.image_thread.stop()
                    self.image_thread.join()

                    # Start control loop thread if not running
                    self.stop_controller_event.clear()
                    if self.controller_thread is None or not self.controller_thread.is_alive():
                        self.controller_thread = threading.Thread(
                            target=run_controller_main.main,
                            args=(self.tracking_service, self.controller, self.mqtt_client, self.path, self.stop_controller_event),
                            daemon=True
                        )
                        self.controller_thread.start()
                    else:
                        print("[INFO] Control loop already running")

        elif self.state == SystemState.CUSTOM_PATH:
            if cmd == "Back":
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                print("[FSM] Transitioned to NAVIGATION")

        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED