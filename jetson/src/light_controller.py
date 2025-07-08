import time
import threading
from astar.astar import astar_downscaled
from astar.board_masking import get_dynamic_threshold, create_binary_mask, dilate_mask
from astar.nearest_point import find_nearest_walkable
from astar.waypoint_sampling import sample_waypoints
from astar.path_memory import PathMemory
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

    def run(self):
        print("[PathFindingThread] Started path finding...")
        frame = self.tracking_service.get_stable_frame()
        gray = get_dynamic_threshold(frame)
        binary_mask = create_binary_mask(gray)
        safe_mask = dilate_mask(binary_mask)
        cv2.circle(safe_mask, (1030, 630), 70, 255, -1)

        ball_pos = self.tracking_service.get_ball_position()
        if ball_pos is None:
            print("[PathFindingThread] Ball position is None. Aborting.")
            return

        ball_pos = (ball_pos[1], ball_pos[0])  # Convert to (y, x)
        #ball_pos = (701, 941)
        start = find_nearest_walkable(safe_mask, ball_pos)
        cached = self.path_cache.get_cached_path(start, self.goal)

        if cached:
            path = cached
        else:
            path = astar_downscaled(safe_mask, start, self.goal, repulsion_weight=self.repulsion_weight, scale=self.scale)
            if path: 
                self.path_cache.cache_path(start, self.goal, path)
            else:
                print("[PathMemory] Pathfinding failed. Not caching empty path.")

        waypoints = sample_waypoints(path, safe_mask)

        final_path = [(x, y) for y, x in waypoints]
        print(f"[PathFindingThread] Path found with {len(final_path)} points.")

        self.on_path_found(final_path)


