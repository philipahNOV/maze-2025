import time
import threading
from control.astar.astar import astar
from control.astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
from control.astar.nearest_point import find_nearest_walkable
from control.astar.waypoint_sampling import sample_waypoints
from control.astar.waypoint_sampling_la import sample_waypoints_la
from control.astar.path_memory import PathMemory
import colorsys
import cv2
import random


class BlinkRed(threading.Thread):
    def __init__(self, arduino_thread, config, controller):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self._stop_event = threading.Event()
        self.trigger_delay = config["general"].get("get_ball_delay", 7)  # seconds before elevator triggers
        self.start_time = None
        self.triggered = False
        self.controller = controller

    def run(self):
        red = (255, 0, 0)
        white = (255, 255, 255)
        self.start_time = time.time()
        self.arduino_thread.send_color(*red)
        time.sleep(2.0)
        self.arduino_thread.send_color(*white)
        
        while not self._stop_event.is_set():
            if not self.triggered and (time.time() - self.start_time) > self.trigger_delay:
                for _ in range(5):
                    self.triggered = True
                    self.arduino_thread.send_elevator(1)
                    time.sleep(0.05)
                #self.arduino_thread.send_elevator(1)
                self.arduino_thread.send_color(*white)
                self.triggered = True
                #self.controller.elevator_state = "up"

            time.sleep(0.2)

    def stop(self):
        self._stop_event.set()


class LookForBall:
    def __init__(self, tracking_service, on_ball_found=None):
        self.tracking_service = tracking_service
        self.on_ball_found = on_ball_found
        self._stop_event = threading.Event()

    def start_ball_check(self):
        thread = threading.Thread(target=self._check_loop, daemon=True)
        thread.start()

    def _check_loop(self):
        while not self._stop_event.is_set():
            pos = self.tracking_service.get_ball_position()
            print(pos)
            if pos is not None:
                if self.on_ball_found:
                    self.on_ball_found()
                break
            time.sleep(0.1)

    def stop(self):
        self._stop_event.set()

class PathFindingThread(threading.Thread):
    def __init__(self, tracking_service, goal, on_path_found, config):
        super().__init__(daemon=True)
        self.tracking_service = tracking_service
        self.config = config
        self.goal = goal
        self.on_path_found = on_path_found
        self.repulsion_weight = self.config['path_finding'].get('repulsion_weight', 5)
        self.path_cache = PathMemory(config)
        self._stop_event = threading.Event()

    def run(self):
        print("[PathFindingThread] Started path finding...")

        if self._stop_event.is_set():
            return

        frame = self.tracking_service.get_stable_frame()
        if frame is None or self._stop_event.is_set():
            return

        gray = get_dynamic_threshold(frame)
        binary_mask = create_binary_mask(gray)
        safe_mask = dilate_mask(binary_mask)

        if self._stop_event.is_set():
            return

        #cv2.circle(safe_mask, (998, 588), 70, 255, -1)
        # Define the starting point (rightmost edge of the rectangle)
        start_point = (998, 588)

        # Define the width and height of the rectangle
        width = 120     # How far to stretch to the left (along -x)
        height = 55     # Height of the rectangle

        # Define the top-left and bottom-right corners
        top_left = (start_point[0] - width, start_point[1] - height // 2)
        bottom_right = (start_point[0] + 40, start_point[1] + height // 2)

        # Draw a white filled rectangle on safe_mask
        cv2.rectangle(safe_mask, top_left, bottom_right, 255, -1)

        cv2.imshow("Safe Mask", safe_mask)
        cv2.waitKey(0)         # Wait until a key is pressed
        cv2.destroyAllWindows()  # Close the window

        ball_pos = self.tracking_service.get_ball_position()
        if ball_pos is None or self._stop_event.is_set():
            self.on_path_found(None, None)
            return

        ball_pos = (ball_pos[1], ball_pos[0])

        start = find_nearest_walkable(safe_mask, ball_pos)

        if self._stop_event.is_set():
            return

        cached = self.path_cache.get_cached_path(start, self.goal)

        if cached:
            path = cached
        else:
            path = astar(safe_mask, start, self.goal, repulsion_weight=self.repulsion_weight)
            if self._stop_event.is_set():
                return
            if path:
                self.path_cache.cache_path(start, self.goal, path)
            else:
                self.on_path_found(None, None)
                return

        if self._stop_event.is_set():
            return

        waypoints = sample_waypoints(path, safe_mask)
        #waypoints = astar.waypoint_sampling_2.sample_waypoints(path, safe_mask)
        waypoints_lookahead = sample_waypoints_la(path, safe_mask, waypoint_spacing=120, angle_threshold=135)

        if self._stop_event.is_set():
            return

        final_path = [(x, y) for y, x in waypoints]
        final_path_lookahead = [(x, y) for y, x in waypoints_lookahead]
        self.on_path_found(final_path, final_path_lookahead)

    # function to stop the thread if we fsm receives the "back" command from states auto path or custom path
    def stop(self):
        self._stop_event.set()

class EscapeElevatorThread(threading.Thread):
    def __init__(self, arduino_thread, controller):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self.duration = 1.2
        self.y_duration = 0.1
        self.opposite_duration = 0.2
        self.speed = 255
        self._stop_event = threading.Event()
        self.start_time = time.time()
        self.controller = controller

    def run(self):
        while time.time() - self.start_time < self.duration:
            elapsed = time.time() - self.start_time
            if elapsed < self.opposite_duration:
                self.arduino_thread.send_speed(0, self.speed)
            elif elapsed >= self.y_duration + self.opposite_duration:
                self.arduino_thread.send_speed(25, -self.speed)
            else:
                self.arduino_thread.send_speed(0, -self.speed)
            time.sleep(0.1)
        time.sleep(0.3)
        if self.controller.elevator_state is not None:
            for _ in range(5):
                    self.arduino_thread.send_elevator(-1)
                    time.sleep(0.05)
            #self.arduino_thread.send_elevator(-1)
        
        self.arduino_thread.send_speed(0, 0)

class DiscoThread(threading.Thread):
    def __init__(self, arduino_thread, idle_time=15):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self._stop_event = threading.Event()
        self.mode = 0
        self.last_mode_zero_time = time.time()
        self.idle_time = idle_time

    def run(self):
        hue = random.randrange(0, 100) / 100.0

        while not self._stop_event.is_set():
            if self.mode == 1:
                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                r, g, b = int(r * 255), int(g * 255), int(b * 255)
                self.arduino_thread.send_color(r, g, b)

                hue += 0.005
                if hue > 1.0:
                    hue = 0.0

                time.sleep(0.1)

            else:
                if time.time() - self.last_mode_zero_time >= self.idle_time:
                    self.mode = 1
                time.sleep(0.2)

    def toggle_mode(self):
        self.mode = (self.mode + 1) % 2
        if self.mode == 0:
            self.last_mode_zero_time = time.time()
            self.arduino_thread.send_color(255, 255, 255)

    def off(self):
        self.mode = 0
        self.last_mode_zero_time = time.time()
        self.arduino_thread.send_color(255, 255, 255)

    def stop(self):
        self.mode = 0
        self.arduino_thread.send_color(255, 255, 255)
        self._stop_event.set()