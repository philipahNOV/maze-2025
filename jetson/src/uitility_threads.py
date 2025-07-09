import time
import threading
from astar.astar import astar_downscaled
from astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
from astar.nearest_point import find_nearest_walkable
from astar.waypoint_sampling import sample_waypoints
from astar.path_memory import PathMemory
import colorsys
import cv2


class BlinkRed(threading.Thread):
    def __init__(self, arduino_thread):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self._stop_event = threading.Event()

    def run(self):
        red = (255, 0, 0)
        white = (255, 255, 255)
        while not self._stop_event.is_set():
            self.arduino_thread.send_color(*red)
            time.sleep(0.2)
            self.arduino_thread.send_color(*white)
            time.sleep(4)

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
        print("[LookForBall] Started checking...")
        while not self._stop_event.is_set():
            pos = self.tracking_service.get_ball_position()
            if pos is not None:
                print(f"[LookForBall] Ball found at {pos}")
                if self.on_ball_found:
                    self.on_ball_found()
                break
            time.sleep(0.1)

    def stop(self):
        self._stop_event.set()

class PathFindingThread(threading.Thread):
    def __init__(self, tracking_service, goal, on_path_found, repulsion_weight=5.0, scale=1.0):
        super().__init__(daemon=True)
        self.tracking_service = tracking_service
        self.goal = goal
        self.on_path_found = on_path_found
        self.repulsion_weight = repulsion_weight
        self.scale = scale
        self.path_cache = PathMemory(max_paths=10, tolerance=15, cache_file="astar/path_cache.json")
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

        cv2.circle(safe_mask, (1030, 630), 70, 255, -1)

        ball_pos = self.tracking_service.get_ball_position()
        if ball_pos is None or self._stop_event.is_set():
            print("[PathFindingThread] Ball position is None or stop requested. Aborting.")
            return

        ball_pos = (ball_pos[1], ball_pos[0])
        start = find_nearest_walkable(safe_mask, ball_pos)

        if self._stop_event.is_set():
            return

        cached = self.path_cache.get_cached_path(start, self.goal)
        print(f"[PathFindingThread] Start point: {start}, Goal: {self.goal}")

        if cached:
            path = cached
            print("[PathFindingThread] Using cached path.")
        else:
            path = astar_downscaled(safe_mask, start, self.goal,
                                    repulsion_weight=self.repulsion_weight, scale=self.scale)
            if self._stop_event.is_set():
                return
            if path:
                self.path_cache.cache_path(start, self.goal, path)
            else:
                print("[PathMemory] Pathfinding failed. Not caching empty path.")
                return  # exit early if pathfinding failed

        if self._stop_event.is_set():
            return

        print(f"[PathFindingThread] Path length: {len(path)}")
        waypoints = sample_waypoints(path, safe_mask)

        if self._stop_event.is_set():
            return

        print(f"[PathFindingThread] Sampled {len(waypoints)} waypoints.")
        final_path = [(x, y) for y, x in waypoints]
        print(f"[PathFindingThread] Path found with {len(final_path)} points.")

        self.on_path_found(final_path)

    # function to stop the thread if we fsm receives the "back" command from states auto path or custom path
    def stop(self):
        print("[PathFindingThread] Stopping path finding thread.")
        self._stop_event.set()

class EscapeElevatorThread(threading.Thread):
    def __init__(self, arduino_thread):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self.duration = 2.5
        self.y_duration = 0.5
        self.speed = 150  # absolute motor speed
        self._stop_event = threading.Event()
        self.start_time = time.time()

    def run(self):
        print("[EscapeElevatorThread] Starting escape...")
        while time.time() - self.start_time < self.duration:
            elapsed = time.time() - self.start_time
            if elapsed >= self.y_duration:
                self.arduino_thread.send_speed(25, -self.speed)
            else:
                self.arduino_thread.send_speed(0, self.speed)
            time.sleep(0.1)
        self.arduino_thread.send_speed(0, 0)

class DiscoThread(threading.Thread):
    def __init__(self, arduino_thread, mode):
        super().__init__(daemon=True)
        self.arduino_thread = arduino_thread
        self._stop_event = threading.Event()
        self.mode = mode

    def run(self):
        print("[DiscoThread] Starting disco...")
        if self.mode == 1:
            hue = 0.0
            while not self._stop_event.is_set():
                # Convert hue to RGB (colorsys returns floats 0–1)
                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                r, g, b = int(r * 255), int(g * 255), int(b * 255)

                # Send color to Arduino
                self.arduino_thread.send_color(r, g, b)

                # Increment hue and wrap around
                hue += 0.005
                if hue > 1.0:
                    hue = 0.0

                time.sleep(0.1)
        elif self.mode == 2:
            hue = 0.0
            while not self._stop_event.is_set():
                # Convert hue to RGB (colorsys returns floats 0–1)
                r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                r, g, b = int(r * 255), int(g * 255), int(b * 255)

                # Send color to Arduino
                self.arduino_thread.send_color(r, g, b)

                # Increment hue and wrap around
                hue += 0.005
                if hue > 1.0:
                    hue = 0.0

                time.sleep(0.01)
        elif self.mode == 3:
            brightness = 0.0
            direction = 1
            while not self._stop_event.is_set():
                value = int(brightness * 255)
                self.arduino_thread.send_color(value, value, value)

                brightness += 0.01 * direction
                if brightness >= 1.0:
                    brightness = 1.0
                    direction = -1
                elif brightness <= 0.0:
                    brightness = 0.0
                    direction = 1

                time.sleep(0.01)

        elif self.mode == 4:
            import random
            while not self._stop_event.is_set():
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self.arduino_thread.send_color(r, g, b)
                time.sleep(0.05)  # flash fast

        elif self.mode == 5:
            brightness = 0.0
            direction = 1
            while not self._stop_event.is_set():
                value = int(brightness * 255)
                self.arduino_thread.send_color(value, 0, 0)

                brightness += 0.01 * direction
                if brightness >= 1.0:
                    brightness = 1.0
                    direction = -1
                elif brightness <= 0.0:
                    brightness = 0.0
                    direction = 1

                time.sleep(0.01)
            
        elif self.mode == 6:
            while not self._stop_event.is_set():
                self.arduino_thread.send_color(255, 255, 255)
                self.arduino_thread.send_color(255, 0, 0, 5)
                time.sleep(0.01)

    def stop(self):
        print("[DiscoThread] Stopping disco thread.")
        self.arduino_thread.send_color(255, 255, 255)
        self._stop_event.set()