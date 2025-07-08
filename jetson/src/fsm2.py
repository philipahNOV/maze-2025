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
import cv2


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
        self.safe_control = False
        self.speed_control = False
        self.loop_control = False
        self.ball_finder = None
        self.path = None
        self.image_controller = ImageController()
        self.image_thread = None

    def on_ball_found(self):
        self.mqtt_client.client.publish("pi/info", "ball_found")

    def on_path_found(self, path):
        self.path = path
        self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
        self.image_thread.start()

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
                    self.safe_control = True
                    self.speed_control = False
                elif cmd.endswith("speed"):
                    self.safe_control = False
                    self.speed_control = True
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
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                self.image_thread.stop()
                self.image_thread.join()
                self.path = None
                self.image_thread = None
                print("[FSM] Transitioned to NAVIGATION")
            if cmd.startswith("Locate"):
                self.state = SystemState.LOCATING
                print("[FSM] Transitioned to LOCATING")
                self.tracking_service.stop_tracker()
                self.image_thread.stop()
                self.image_thread.join()
                self.path = None
                self.image_thread = None
                self.tracking_service.start_tracker()
                self.ball_finder = light_controller.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
                self.ball_finder.start_ball_check()

        elif self.state == SystemState.CUSTOM_PATH:
            if cmd == "Back":
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                print("[FSM] Transitioned to NAVIGATION")

        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED