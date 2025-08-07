import cmd
from enum import Enum, auto
import time
from mqtt.mqtt_client import MQTTClientJetson
from arduino_connection import ArduinoConnection
from tracking.tracker_service import TrackerService
import utils.utility_threads as utility_threads
from control.image_controller import ImageController
from control.image_controller import ImageSenderThread
from control.joystick_controller import JoystickController
import control.position_controller as position_controller
from utils.utility_functions import is_in_elevator, remove_withing_elevator, determine_maze, is_within_goal
import app.run_controller_main as run_controller_main
import threading
import os
import sys
from typing import Dict, Any
from utils.leaderboard_utils import add_score
from utils.leaderboard_utils import read_leaderboard
from utils.leaderboard_utils import clear_leaderboard
from datetime import datetime
import numpy as np
import subprocess


class SystemState(Enum):
    BOOTING = auto()
    MAIN_SCREEN = auto()
    INFO_SCREEN = auto()
    NAVIGATION = auto()
    LOCATING = auto()
    AUTO_PATH = auto()
    CUSTOM_PATH = auto()
    CONTROLLING = auto()
    HUMAN_CONTROLLER = auto()
    PRACTICE = auto()
    PLAYALONE = auto()
    PLAYALONE_START = auto()
    PLAYVSAI = auto()
    PLAYVSAI_PID = auto()
    PLAYVSAI_HUMAN = auto()
    LEADERBOARD = auto()
    PLAYALONE_VICTORY = auto()
    PLAYALONE_FAILED = auto()
    ADMIN_TOOLS = auto()

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
        self.playvsai_goal = None
        self.current_player_name = "Unknown"
        self.path_thread = None
        self.disco_mode = 0
        self.disco_thread = None
        self.loop_path = False
        self.maze_version = None
        self.config = config

        self.alive_thread = utility_threads.ImAliveThread(self.mqtt_client)
        self.alive_thread.start()

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
        self.tracking_service.stop_tracker()
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
        if self.alive_thread is not None:
            self.alive_thread.stop()
            self.alive_thread = None
        if self.disco_thread is not None:
            self.disco_thread.stop()
            self.disco_thread.join()
            self.disco_thread = None
        self.tracking_service.camera.close()

    def stop_program(self):
        print("Stopping program...")
        self.stop_threads()
        sys.exit(0)

    def on_ball_found(self):
        print(self.tracking_service.get_ball_position())
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

    def _start_joystick_control(self, playalone_wait=False): # could be moved to sep file
        if hasattr(self, 'joystick_controller'):
            self.joystick_controller.stop()
            if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                self.joystick_thread.join()

        self.joystick_controller = JoystickController(self.arduino_thread, self.mqtt_client, playalone_wait=playalone_wait)
        self.joystick_thread = threading.Thread(target=self.joystick_controller.start, daemon=True)
        self.joystick_thread.start()

    def _ball_crossed_goal(self, position):
        if (self.state in [SystemState.PLAYVSAI_PID, SystemState.PLAYVSAI_HUMAN] and 
            hasattr(self, 'playvsai_goal') and self.playvsai_goal is not None):
            goal = self.playvsai_goal
            radius = 30
        else:
            goal_config = self.config.get("game", {})
            goal_pos = goal_config.get("goal_position", {"x": 49, "y": 763})
            goal = (goal_pos["x"], goal_pos["y"])
            radius = goal_config.get("goal_radius", 30)
        
        x, y = position
        goal_x, goal_y = goal
        
        distance = ((x - goal_x) ** 2 + (y - goal_y) ** 2) ** 0.5
        return distance <= radius
    
    def determine_rank(self, data, duration):
        rank = 1
        if data.strip():
                lines = data.strip().split('\n')
                for line in lines:
                    if line.strip():
                        parts = line.split(',')
                        print(parts)
                        if len(parts) == 4:
                            name, time_str, date_str, maze_str = parts
                            try:
                                time_value = float(time_str)
                                if time_value < duration:
                                    rank += 1
                            except ValueError:
                                continue
        return rank
        

    def run_playalone_game(self):
        print("[PLAYALONE] Starting tracking thread...")
        game_config = self.config.get("game", {})
        #maze_id = game_config.get("maze_id", 1)
        player_name = self.current_player_name
        ball_lost_timeout = game_config.get("ball_lost_timeout", 3)
        
        self.tracking_service.start_tracker()
        self.mqtt_client.client.publish("pi/tracking_status", "tracking_started")
        time.sleep(1)
        self.maze_version = determine_maze(self.tracking_service)
        if self.maze_version is not None and self.maze_version == "Hard":
            maze_id = 1
        else:
            maze_id = 2

        print(f"[PLAYALONE] Playing on Maze {self.maze_version} with timeout {ball_lost_timeout}s")
        print(f"[PLAYALONE] Player name: {player_name}")

        game_running = True
        start_time = None
        last_valid_pos_time = time.time()
        game_timer_started = False
        ball_previously_detected = False

        while game_running and self.state == SystemState.PLAYALONE_START:
            if hasattr(self, 'playalone_game_stop_requested') and self.playalone_game_stop_requested:
                print("[PLAYALONE] Game stop requested, exiting game loop")
                break
                
            ball_pos = self.tracking_service.get_ball_position()

            if ball_pos is None:
                if start_time and (time.time() - last_valid_pos_time > ball_lost_timeout):
                    print(f"[PLAYALONE] Game failed: ball lost > {ball_lost_timeout} seconds.")
                    self.mqtt_client.client.publish("pi/command", "playalone_fail")
                    break
                elif ball_previously_detected:
                    self.mqtt_client.client.publish("pi/tracking_status", "ball_lost")
                    self.joystick_controller.ball_in_elevator = False
                    ball_previously_detected = False
            elif not game_timer_started:
                last_valid_pos_time = time.time()
                
                if not ball_previously_detected and is_in_elevator(self.config, ball_pos):
                    self.mqtt_client.client.publish("pi/tracking_status", "ball_detected")
                    self.joystick_controller.ball_in_elevator = True
                    ball_previously_detected = True

            else:
                last_valid_pos_time = time.time()
                
                if not ball_previously_detected:
                    self.mqtt_client.client.publish("pi/tracking_status", "ball_detected")
                    self.joystick_controller.ball_in_elevator = True
                    ball_previously_detected = True

                if game_timer_started and start_time is None:
                    print("[PLAYALONE] Ball seen and game started — starting timer.")
                    start_time = time.time()

                if is_within_goal(self.maze_version, ball_pos) and start_time is not None:
                    duration = time.time() - start_time
                    print(f"[PLAYALONE] Goal reached in {duration:.2f} sec")
                    add_score(player_name, duration, maze_id, self.mqtt_client)

                    leaderboard_data = read_leaderboard(maze_id)
                    csv_data = []
                    for entry in leaderboard_data:
                        csv_data.append(f"{entry['name']},{entry['time']:.2f},{entry['date']},{entry['maze_id']}")
                    
                    csv_string = '\n'.join(csv_data)
                    rank = self.determine_rank(csv_string, duration)

                    self.mqtt_client.client.publish("pi/command", f"playalone_success:{duration:.2f}:{rank}")
                    break

            if hasattr(self, 'playalone_timer_start_requested') and self.playalone_timer_start_requested:
                game_timer_started = True
                start_time = time.time()
                self.playalone_timer_start_requested = False
                print(f"[PLAYALONE] Game start requested, timer started immediately at {start_time}")

            time.sleep(0.1)

        print("[PLAYALONE] Game loop ended, stopping tracker")
        self.tracking_service.stop_tracker()

    def start_playalone_timer(self):
        self.playalone_timer_start_requested = True
        print("[PLAYALONE] Timer start requested!")

    def run_playvsai_pid_turn(self):
        print("[PLAYVSAI] Starting PID turn...")
        game_config = self.config.get("game", {})
        ball_lost_timeout = game_config.get("ball_lost_timeout", 3)
        self.mqtt_client.client.publish("pi/command", "playvsai_pid_started")
        
        if hasattr(self, 'playvsai_goal') and self.playvsai_goal is not None:
            self.start_path_finding(custom_goal=self.playvsai_goal)
        else:
            self.start_path_finding()
        
        path_found = False
        timeout_counter = 0
        while not path_found and timeout_counter < 100:  # 10 second timeout
            if self.path is not None:
                path_found = True
                break
            time.sleep(0.1)
            timeout_counter += 1
        
        if not path_found:
            print("[PLAYVSAI] PID failed: No path found")
            self.mqtt_client.client.publish("pi/command", "playvsai_pid_fail:no_path")
            self.state = SystemState.PLAYVSAI
            return
        
        if self.image_thread is not None:
            self.image_thread.stop()
            self.image_thread.join()
            self.image_thread = None

        self.stop_controller_event.clear()
        if self.controller_thread is None or not self.controller_thread.is_alive():
            self.controller.lookahead = True
            self.controller_thread = threading.Thread(
                target=run_controller_main.main,
                args=(self.tracking_service, self.controller, self.mqtt_client, self.path, self.image_controller, self.stop_controller_event, self.config, True),
                daemon=True
            )
            self.controller_thread.start()
        
        game_running = True
        start_time = time.time()
        last_valid_pos_time = time.time()
        
        while game_running and self.state == SystemState.PLAYVSAI_PID:
            if hasattr(self, 'playvsai_stop_requested') and self.playvsai_stop_requested:
                print("[PLAYVSAI] PID turn stop requested")
                break
                
            ball_pos = self.tracking_service.get_ball_position()
            
            if ball_pos is None:
                if time.time() - last_valid_pos_time > ball_lost_timeout:
                    print(f"[PLAYVSAI] PID failed: ball lost > {ball_lost_timeout} seconds.")
                    self.mqtt_client.client.publish("pi/command", "playvsai_pid_fail:ball_lost")
                    break
            else:
                last_valid_pos_time = time.time()
                
                if is_within_goal(self.maze_version, ball_pos, self.playvsai_goal):
                    duration = time.time() - start_time
                    print(f"[PLAYVSAI] PID succeeded in {duration:.2f} sec")
                    self.mqtt_client.client.publish("pi/command", f"playvsai_pid_success:{duration:.2f}")
                    break
            
            time.sleep(0.1)
        
        self.stop_controller()
        print("[PLAYVSAI] PID turn ended")
        
        if self.state == SystemState.PLAYVSAI_PID:
            self.state = SystemState.PLAYVSAI_HUMAN
            threading.Thread(target=self.run_playvsai_human_turn, daemon=True).start()

    def run_playvsai_human_turn(self):
        print("[PLAYVSAI] Starting human turn...")
        game_config = self.config.get("game", {})
        ball_lost_timeout = game_config.get("ball_lost_timeout", 3)

        for _ in range(5):
            self.arduino_thread.send_elevator(1)
            time.sleep(0.05)
        
        self.mqtt_client.client.publish("pi/command", "playvsai_human_started")
        self.image_controller.set_new_path(None)
        self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
        self.image_thread.start()  
        self._start_joystick_control()
        
        game_running = True
        start_time = None
        last_valid_pos_time = time.time()
        game_timer_started = False
        ball_previously_detected = False
        
        while game_running and self.state == SystemState.PLAYVSAI_HUMAN:
            if hasattr(self, 'playvsai_stop_requested') and self.playvsai_stop_requested:
                print("[PLAYVSAI] Human turn stop requested")
                break
                
            ball_pos = self.tracking_service.get_ball_position()
            
            if ball_pos is None:
                if start_time and (time.time() - last_valid_pos_time > ball_lost_timeout):
                    print(f"[PLAYVSAI] Human failed: ball lost > {ball_lost_timeout} seconds.")
                    self.mqtt_client.client.publish("pi/command", "playvsai_human_fail")
                    break
                elif ball_previously_detected:
                    ball_previously_detected = False
            else:
                last_valid_pos_time = time.time()
                
                if not ball_previously_detected:
                    ball_previously_detected = True

                if game_timer_started and start_time is None:
                    print("[PLAYVSAI] Human ball seen and game started — starting timer.")
                    start_time = time.time()

                if is_within_goal(self.maze_version, ball_pos, self.playvsai_goal) and start_time is not None:
                    duration = time.time() - start_time
                    print(f"[PLAYVSAI] Human succeeded in {duration:.2f} sec")
                    self.mqtt_client.client.publish("pi/command", f"playvsai_human_success:{duration:.2f}")
                    break

            if hasattr(self, 'playvsai_human_timer_start_requested') and self.playvsai_human_timer_start_requested:
                game_timer_started = True
                self.playvsai_human_timer_start_requested = False
                print("[PLAYVSAI] Human game start requested, timer will begin when ball is detected")

            time.sleep(0.1)
        
        if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
        if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
            self.joystick_thread.join()
        
        self.tracking_service.stop_tracker()
        
        if self.image_thread is not None:
            self.image_thread.stop()
            self.image_thread.join()
            self.image_thread = None
        if self.path_thread is not None:
            self.path_thread.stop()
            self.path_thread = None
        self.path = None
        print("[PLAYVSAI] Human turn ended")
        
        if self.state == SystemState.PLAYVSAI_HUMAN:
            self.playvsai_goal = None

    def start_playvsai_human_timer(self):
        self.playvsai_human_timer_start_requested = True
        print("[PLAYVSAI] Human timer start requested!")

    def start_path_finding(self, custom_goal=None):
        if custom_goal is not None:
            goal = (custom_goal[1], custom_goal[0])
        else:
            game_config = self.config.get("game", {})
            goal_pos = game_config.get("goal_position", {"x": 49, "y": 763})
            goal = (goal_pos["y"], goal_pos["x"])

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
                    for _ in range(5):
                        self.arduino_thread.send_elevator(-1)
                        time.sleep(0.05)
                    #self.arduino_thread.send_elevator(-1)
        
        # --- MAIN_SCREEN STATE ---
        elif self.state == SystemState.MAIN_SCREEN:
            if cmd == "Info":
                self.state = SystemState.INFO_SCREEN
                print("[FSM] Transitioned to INFO_SCREEN")
            
            elif cmd == "Human":
                print("[FSM] Transitioned to HUMAN_CONTROLLER")

                if self.disco_thread:
                    self.disco_thread.stop()
                    self.disco_thread.join()
                    self.disco_thread = None

                self.state = SystemState.HUMAN_CONTROLLER
                self.mqtt_client.client.publish("pi/command", "show_human_screen")

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
                    for _ in range(5):
                        self.arduino_thread.send_elevator(1)
                        time.sleep(0.05)
                    #self.arduino_thread.send_elevator(1)
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
            elif cmd == "AdminTools":
                self.state = SystemState.ADMIN_TOOLS
                print("[FSM] Transitioned to ADMIN_TOOLS")

        elif self.state == SystemState.ADMIN_TOOLS:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")
            elif cmd == "ClearEasyLeaderboard":
                clear_leaderboard(2)
                print("[FSM] Easy leaderboard cleared")
            elif cmd == "ClearHardLeaderboard":
                clear_leaderboard(1)
                print("[FSM] Hard leaderboard cleared")   
            elif cmd.startswith("SetOffsets"):
                offsets_str = cmd.split(":")[1].split(",")
                try:
                    offset_x = float(offsets_str[0])
                    self.controller.x_offset = offset_x
                    print(f"[FSM] X offset set to {offset_x}")
                except:
                    self.controller.x_offset = self.config["controller"]["arduino"].get("x_offset", 0.002)
                try:
                    offset_y = float(offsets_str[1])
                    self.controller.y_offset = offset_y
                    print(f"[FSM] Y offset set to {offset_y}")
                except:
                    self.controller.y_offset = self.config["controller"]["arduino"].get("y_offset", 0.001)
            elif cmd == "Reboot":
                self.stop_threads()
                subprocess.run(['sudo', '/usr/sbin/reboot'], check=True)
            elif cmd == "Shutdown":
                self.stop_threads()
                subprocess.run(['sudo', '/usr/sbin/poweroff'], check=True)


        # --- INFO_SCREEN STATE ---
        elif self.state == SystemState.INFO_SCREEN:
            if cmd == "Back":
                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")

        # --- HUMAN STATE ---
        elif self.state == SystemState.HUMAN_CONTROLLER:
            if cmd == "Back":
                print("[FSM] Exiting HUMAN_CONTROLLER...")
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                self.arduino_thread.send_speed(0, 0)

                self.state = SystemState.MAIN_SCREEN
                print("[FSM] Transitioned to MAIN_SCREEN")

                self.disco_thread = utility_threads.DiscoThread(
                    self.arduino_thread, self.config['general'].get('idle_light_time', 15)
                )
                self.disco_thread.start()

            elif cmd == "Practice":
                print("[FSM] Entering PRACTICE mode")
                self.mqtt_client.client.publish("pi/command", "display_practice_message")
                self.state = SystemState.PRACTICE
                self._start_joystick_control()

            elif cmd == "PlayVsAI":
                print("[FSM] Entering PLAYVSAI mode")
                self.mqtt_client.clear_image_buffer()
                
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                
                self.path = None
                self.path_lookahead = None
                self.playvsai_goal = None
                self.image_controller.set_new_path(None)
                
                self.tracking_service.stop_tracker()
                time.sleep(0.2)
                self.tracking_service.start_tracker()
                
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()                
                self.state = SystemState.PLAYVSAI
                self.mqtt_client.client.publish("pi/command", "show_playvsai_screen")

                for _ in range(5):
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.05)

            elif cmd == "PlayAlone":
                print("[FSM] Entering PLAYALONE mode")
                self.playalone_timer_start_requested = False
                self.state = SystemState.PLAYALONE
                self.mqtt_client.client.publish("pi/command", "show_playalone_screen")

            elif cmd == "Leaderboard":
                print("[FSM] Entering LEADERBOARD mode")
                self.state = SystemState.LEADERBOARD
                self.mqtt_client.client.publish("pi/command", "show_leaderboard_screen")
                
                from utils.leaderboard_utils import send_leaderboard_data
                send_leaderboard_data(self.mqtt_client, 1)
                send_leaderboard_data(self.mqtt_client, 2)
        
        elif self.state == SystemState.PRACTICE:
            if cmd == "Back":
                print("[FSM] Exiting PRACTICE mode...")

                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                    del self.joystick_controller
                
                if hasattr(self, 'joystick_thread'):
                    if self.joystick_thread.is_alive():
                        self.joystick_thread.join()
                    del self.joystick_thread

                self.arduino_thread.send_speed(0, 0)
                self.state = SystemState.HUMAN_CONTROLLER
                print("[FSM] Returned to HUMAN_CONTROLLER")

        elif self.state == SystemState.PLAYALONE:
            if cmd == "Back":
                self.playalone_timer_start_requested = False
                self.state = SystemState.HUMAN_CONTROLLER
            
            elif cmd == "StartGame":
                self.state = SystemState.PLAYALONE_START
                print("[PLAYALONE] Entering play alone start screen")
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = False
                
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                
                self.path = None
                self.image_controller.set_new_path(None)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
                self.arduino_thread.send_speed(0, 0)
                self._start_joystick_control(playalone_wait=True)
                threading.Thread(target=self.run_playalone_game, daemon=True).start()

                for _ in range(5):
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.05)

        elif self.state == SystemState.PLAYALONE_START:
            if cmd == "Back":
                self.state = SystemState.HUMAN_CONTROLLER
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = True
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                    del self.joystick_controller
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                    del self.joystick_thread
                
                self.tracking_service.stop_tracker()
                
                if self.path_thread is not None and self.path_thread.is_alive():
                    self.path_thread.stop()
                    self.path_thread = None

                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.path = None
                self.image_controller.set_new_path(self.path)
            
            elif cmd == "StartPlayAloneGame":
                print("[PLAYALONE] Start game button clicked - activating timer")
                self.start_playalone_timer()
            
            elif cmd == "RestartPlayAlone":
                self.mqtt_client.clear_image_buffer()
                self.tracking_service.stop_tracker()
                
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                
                self.path = None
                self.path_lookahead = None
                self.image_controller.set_new_path(None)
                self.custom_goal = None
                
                time.sleep(0.5)
                
                print("[PLAYALONE] Restarting tracking service...")
                self.tracking_service.start_tracker()
                self.mqtt_client.client.publish("pi/tracking_status", "tracking_started")
                self.arduino_thread.send_speed(0,0)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
                
                print("[PLAYALONE] Image system completely reset with tracking active")

            elif cmd == "PlayAloneVictory":
                self.state = SystemState.PLAYALONE_VICTORY
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = True
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                    del self.joystick_controller
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                    del self.joystick_thread
                
                self.tracking_service.stop_tracker()
                
                if self.path_thread is not None and self.path_thread.is_alive():
                    self.path_thread.stop()
                    self.path_thread = None

                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.path = None
                self.image_controller.set_new_path(self.path)

            elif cmd == "PlayAloneFailed":
                print("[FSM] Entering PLAYALONE_FAILED mode")
                self.state = SystemState.PLAYALONE_FAILED
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = True
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                    del self.joystick_controller
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                    del self.joystick_thread
                
                self.tracking_service.stop_tracker()
                
                if self.path_thread is not None and self.path_thread.is_alive():
                    self.path_thread.stop()
                    self.path_thread = None

                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.path = None
                self.image_controller.set_new_path(self.path)

            elif cmd == "Leaderboard":
                    print("[FSM] Entering LEADERBOARD mode")
                    self.state = SystemState.LEADERBOARD
                    self.mqtt_client.client.publish("pi/command", "show_leaderboard_screen")
                    
                    from utils.leaderboard_utils import send_leaderboard_data
                    send_leaderboard_data(self.mqtt_client, 1)
                    send_leaderboard_data(self.mqtt_client, 2)

        elif self.state == SystemState.PLAYALONE_VICTORY:
            if cmd == "Back":
                self.state = SystemState.HUMAN_CONTROLLER
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = True
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                    del self.joystick_controller
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                    del self.joystick_thread
                
                self.tracking_service.stop_tracker()
                
                if self.path_thread is not None and self.path_thread.is_alive():
                    self.path_thread.stop()
                    self.path_thread = None

                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                    self.custom_goal = None
                
                self.path = None
                self.image_controller.set_new_path(self.path)

            elif cmd == "Leaderboard":
                print("[FSM] Entering LEADERBOARD mode")
                self.state = SystemState.LEADERBOARD
                self.mqtt_client.client.publish("pi/command", "show_leaderboard_screen")
                
                from utils.leaderboard_utils import send_leaderboard_data
                send_leaderboard_data(self.mqtt_client, 1)
                send_leaderboard_data(self.mqtt_client, 2)
        
        elif self.state == SystemState.PLAYALONE_FAILED:
            if cmd == "Back":
                self.state = SystemState.HUMAN_CONTROLLER
            if cmd == "Retry":
                self.state = SystemState.PLAYALONE_START
                print("[PLAYALONE] Entering play alone start screen")
                self.playalone_timer_start_requested = False
                self.playalone_game_stop_requested = False
                
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                
                self.path = None
                self.image_controller.set_new_path(None)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
                self.arduino_thread.send_speed(0, 0)
                self._start_joystick_control(playalone_wait=True)
                threading.Thread(target=self.run_playalone_game, daemon=True).start()

                for _ in range(5):
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.05)

        elif self.state == SystemState.LEADERBOARD:
            if cmd == "Back":
                self.state = SystemState.HUMAN_CONTROLLER
            elif cmd.startswith("SendLeaderboard:"):
                maze_id = int(cmd.split(":")[1])
                print(f"[LEADERBOARD] Sending leaderboard data for maze {maze_id}")
                from utils.leaderboard_utils import send_leaderboard_data
                send_leaderboard_data(self.mqtt_client, maze_id)

        # --- PLAYVSAI STATES ---
        elif self.state == SystemState.PLAYVSAI:
            if cmd == "Back":
                print("[FSM] Exiting PLAYVSAI mode...")
                self.tracking_service.stop_tracker()

                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                
                if self.path_thread is not None and self.path_thread.is_alive():
                    self.path_thread.stop()
                    self.path_thread = None
                
                self.mqtt_client.clear_image_buffer()
                self.playvsai_goal = None
                self.path = None
                self.path_lookahead = None
                self.image_controller.set_new_path(None)
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                
                self.arduino_thread.send_speed(0, 0)
                self.state = SystemState.HUMAN_CONTROLLER
                self.mqtt_client.client.publish("pi/command", "show_human_screen")
                print("[FSM] Transitioned to HUMAN_CONTROLLER")
            
            elif cmd.startswith("Goal_set:"):
                coords = cmd.split(":")[1]
                x, y = map(int, coords.split(","))
                self.playvsai_goal = (x, y)
                print(f"[PLAYVSAI] Goal set to ({x}, {y})")
            
            elif cmd == "StartBattle":
                print("[PLAYVSAI] Starting battle - PID goes first")
                self.state = SystemState.PLAYVSAI_PID
                self.playvsai_stop_requested = False
                self.playvsai_human_timer_start_requested = False
                
                threading.Thread(target=self.run_playvsai_pid_turn, daemon=True).start()

        elif self.state == SystemState.PLAYVSAI_PID:
            if cmd == "Back":
                self.playvsai_goal = None
                self.state = SystemState.HUMAN_CONTROLLER
                self.playvsai_stop_requested = True
                self.stop_controller()
                self.tracking_service.stop_tracker()
                self.mqtt_client.client.publish("pi/command", "show_human_screen")

        elif self.state == SystemState.PLAYVSAI_HUMAN:
            if cmd == "Back":
                self.playvsai_goal = None
                self.state = SystemState.HUMAN_CONTROLLER
                self.playvsai_stop_requested = True
                
                if hasattr(self, 'joystick_controller'):
                    self.joystick_controller.stop()
                if hasattr(self, 'joystick_thread') and self.joystick_thread.is_alive():
                    self.joystick_thread.join()
                
                self.tracking_service.stop_tracker()
                
                if self.image_thread is not None:
                    self.image_thread.stop()
                    self.image_thread.join()
                    self.image_thread = None
                if self.path_thread is not None:
                    self.path_thread.stop()
                    self.path_thread = None
                self.path = None
                self.mqtt_client.client.publish("pi/command", "show_human_screen")
            
            elif cmd == "StartHumanTurn":
                print("[PLAYVSAI] Human start button clicked - activating timer")
                self.start_playvsai_human_timer()

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
                self.state = SystemState.MAIN_SCREEN
                self.disco_thread = utility_threads.DiscoThread(self.arduino_thread, self.config['general'].get('idle_light_time', 15))
                self.disco_thread.start()
                print("[FSM] Transitioned to MAIN_SCREEN")
            elif cmd == "BallFound":
                self.state = SystemState.CUSTOM_PATH
                print("[FSM] Transitioned to CUSTOM_PATH")
                self.path = None
                self.image_controller.set_new_path(self.path)
                self.image_thread = ImageSenderThread(self.image_controller, self.mqtt_client, self.tracking_service, self.path)
                self.image_thread.start()
                self.maze_version = determine_maze(self.tracking_service)
                print(f"[FSM] Detected maze version: {self.maze_version}")

        # --- AUTO_PATH STATE ---
        elif self.state == SystemState.AUTO_PATH:
            if cmd == "Back":
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
                coords = cmd.split(":")[1]
                x, y = map(int, coords.split(","))
                self.custom_goal = (x, y)

            elif cmd == "CalculatePath":
                if self.custom_goal is None:
                    print("[FSM] No custom goal set, cannot calculate path.")
                else:
                    self.start_path_finding(custom_goal=self.custom_goal)

            elif cmd.startswith("Start"):
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
                self.stop_controller()
                self.arduino_thread.send_color(255, 255, 255)
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
