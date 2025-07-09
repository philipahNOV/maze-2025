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
        self.custom_goal = None
        self.path_thread = None

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
        if self.is_in_elevator(self.tracking_service.get_ball_position()):
            self.mqtt_client.client.publish("pi/info", "ball_found")
        else:
            self.ball_finder = light_controller.LookForBall(
                    self.tracking_service, on_ball_found=self.on_ball_found
                )
            self.ball_finder.start_ball_check()

    def is_in_elevator(self, ball_pos, center_x: int = 1030, center_y: int = 630, radius: int = 60):
        if ball_pos is None:
            return False
        x, y = ball_pos
        dx = x - center_x
        dy = y - center_y
        distance_squared = dx * dx + dy * dy
        return distance_squared <= radius * radius

    def on_path_found(self, path):
        self.path = path
        if self.path is not None:
            self.mqtt_client.client.publish("pi/info", "path_found")
        if self.controller.lookahead:
            self.path = self.densify_path(self.path, factor=3)
        self.remove_withing_elevator(self.path)
        self.image_controller.set_new_path(self.path)
        if self.image_thread is None:
            self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
            self.image_thread.start()
        
        #self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
        #self.image_thread.start()
    
    def stop_controller(self):
        if self.controller_thread is not None and self.controller_thread.is_alive():
            self.stop_controller_event.set()
            self.controller_thread.join()
            self.controller_thread = None

    def start_path_finding(self, custom_goal=None):
        frame = self.tracking_service.get_stable_frame()
        gray = get_dynamic_threshold(frame)
        binary_mask = create_binary_mask(gray)
        safe_mask = dilate_mask(binary_mask)
        #self.image_controller.frame = safe_mask
        #cv2.circle(self.image_controller.frame, (1030, 630), 70, 255, -1)
        #self.image_controller.crop_and_rotate_frame()
        #self.image_controller.send_frame_to_pi(self.mqtt_client)

        if custom_goal is not None:
            goal = (custom_goal[1], custom_goal[0])
        else:
            goal = (49, 763)

        self.path_thread = light_controller.PathFindingThread(
            tracking_service=self.tracking_service,
            goal=goal,
            on_path_found=self.on_path_found
        )
        self.path_thread.start()

    def remove_withing_elevator(self, path, center_x: int = 1030, center_y: int = 630, radius: int = 60):
        within_indexes = []
        if path is None:
            return
        for i in range(len(path)):
            x, y = path[i]
            dx = x - center_x
            dy = y - center_y
            distance_squared = dx * dx + dy * dy
            if distance_squared <= radius * radius:
                within_indexes.append(i)
        for i in reversed(within_indexes):
            path.pop(i)

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
            if cmd == "Elevator":
                self.arduino_thread.send_get_ball()
            if cmd == "Horizontal":
                self.controller.horizontal()

        elif self.state == SystemState.LOCATING:
            if cmd == "AutoPath":
                self.state = SystemState.AUTO_PATH
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to AUTO_PATH")
                self.start_path_finding()
            elif cmd == "CustomPath":
                self.state = SystemState.CUSTOM_PATH
                print("[FSM] Transitioned to CUSTOM_PATH")
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
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
                self.image_controller.set_new_path(self.path)
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
                self.image_controller.set_new_path(self.path)
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
                    self.image_thread = None

                    self.stop_controller_event.clear()
                    if self.controller_thread is None or not self.controller_thread.is_alive():
                        self.controller_thread = threading.Thread(
                            target=run_controller_main.main,
                            args=(self.tracking_service, self.controller, self.mqtt_client, self.path, self.image_controller, self.stop_controller_event),
                            daemon=True
                        )
                        self.controller_thread.start()
                    else:
                        print("[INFO] Control loop already running")
            if cmd == "timeout":
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
                print("[FSM] Transitioned to MAIN_SCREEN")

        elif self.state == SystemState.CUSTOM_PATH:
            if cmd == "Back":
                self.tracking_service.stop_tracker()
                if self.path_thread is not None:
                    self.path_thread.stop()
                    self.path_thread = None
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.state = SystemState.NAVIGATION
                self.path = None
                self.image_controller.set_new_path(self.path)
                print("[FSM] Transitioned to NAVIGATION")
            if cmd.startswith("Goal_set:"):
                coords = cmd.split(":")[1]
                x, y = map(int, coords.split(","))
                self.custom_goal = (x, y)
            if cmd == "CalculatePath":
                if self.custom_goal is None:
                    print("[FSM] No custom goal set, cannot calculate path.")
                else:
                    self.start_path_finding(custom_goal=self.custom_goal)
            if cmd == "Start":
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
                            args=(self.tracking_service, self.controller, self.mqtt_client, self.path, self.image_controller, self.stop_controller_event),
                            daemon=True
                        )
                        self.controller_thread.start()
                    else:
                        print("[INFO] Control loop already running")
        elif cmd == "Emergency_Stop":
            self.model.stop_all()
            self.state = SystemState.STOPPED