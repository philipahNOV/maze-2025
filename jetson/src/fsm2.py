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

    def on_ball_found(self):
        self.mqtt_client.client.publish("pi/info", "ball_found")

    def on_path_found(self, path):
        self.path = path
        image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
        image_thread.start()

    def path_finding_auto(self):
        gray = get_dynamic_threshold(self.tracking_service.get_stable_frame())
        binary_mask = create_binary_mask(gray)
        safe_mask = dilate_mask(binary_mask)

        ball_pos = self.tracking_service.get_ball_position()
        ball_pos = (ball_pos[1], ball_pos[0])  # Convert to (y, x) format for processing
        start = find_nearest_walkable(safe_mask, ball_pos)

        goal = (49, 763)

        path = astar_downscaled(safe_mask, start, goal, repulsion_weight=5.0, scale=0.60)
        waypoints = sample_waypoints(path, safe_mask)
        self.path = [(x, y) for y, x in waypoints]

    def start_path_finding(self):
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
                else:
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
                print("[FSM] Transitioned to NAVIGATION")

        elif self.state == SystemState.CUSTOM_PATH:
            if cmd == "Back":
                self.state = SystemState.NAVIGATION
                self.tracking_service.stop_tracker()
                print("[FSM] Transitioned to NAVIGATION")

        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED