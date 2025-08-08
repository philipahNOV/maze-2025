import threading
import time
import numpy as np
from collections import deque
from tracking.model_loader import YOLOModel

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="v8-512.engine"):
        self.model = YOLOModel(model_path)
        self.camera = camera
        self.tracking_config = tracking_config or {}
        self.running = False
        self.initialized = False
        self.frame_queue = deque(maxlen=3)
        self.latest_bgr_frame = None
        self.ball_position = None
        self.timing_print_counter = 0
        self._bbox_buffer = np.zeros((4, 2), dtype=np.float32)
        camera_config = self.tracking_config.get('camera', {})
        
        top_left = camera_config.get('top_left', [460, 5])
        top_right = camera_config.get('top_right', [1100, 5])
        bottom_left = camera_config.get('bottom_left', [460, 720])
        bottom_right = camera_config.get('bottom_right', [1100, 720])
        
        self.maze_x_min = min(top_left[0], bottom_left[0])
        self.maze_x_max = max(top_right[0], bottom_right[0])
        self.maze_y_min = min(top_left[1], top_right[1])
        self.maze_y_max = max(bottom_left[1], bottom_right[1])

    def _is_point_in_maze(self, x, y):
        return (self.maze_x_min <= x <= self.maze_x_max and 
                self.maze_y_min <= y <= self.maze_y_max)

    def producer_loop(self):
        TARGET_FPS = 60
        while self.running:
            start = time.time()
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                self.frame_queue.append((rgb, bgr))
            loop_duration = time.time() - start
            sleep_time = max(0, (1 / TARGET_FPS) - loop_duration)
            time.sleep(sleep_time)

    def consumer_loop(self):
        TARGET_FPS = 30
        MAX_BOXES = 1
        while self.running:
            loop_start = time.time()
            if not self.frame_queue:
                time.sleep(0.001)
                continue

            while len(self.frame_queue) > 1:
                self.frame_queue.popleft()

            rgb, bgr = self.frame_queue.popleft()
            self.latest_bgr_frame = bgr
            inference_start = time.time()
            results = self.model.predict(rgb)
            inference_time = (time.time() - inference_start) * 1000

            h, w = rgb.shape[:2]
            self.ball_position = None
            post_start = time.time()

            ball_boxes = [
                box for box in results.boxes
                if self.model.get_label(box.cls[0]) == "ball"
            ][:MAX_BOXES]

            for box in ball_boxes:
                x_center, y_center, width, height = box.xywh[0]
                cx, cy = int(x_center), int(y_center)
                
                if self._is_point_in_maze(cx, cy):
                    self.ball_position = (cx, cy)

                    x_center_norm = x_center / w
                    y_center_norm = y_center / h
                    width_norm = width / w
                    height_norm = height / h

                    x_min = x_center_norm - width_norm / 2
                    x_max = x_center_norm + width_norm / 2
                    y_min = y_center_norm - height_norm / 2
                    y_max = y_center_norm + height_norm / 2

                    self._bbox_buffer[0, 0] = x_min
                    self._bbox_buffer[0, 1] = y_min
                    self._bbox_buffer[1, 0] = x_max
                    self._bbox_buffer[1, 1] = y_min
                    self._bbox_buffer[2, 0] = x_max
                    self._bbox_buffer[2, 1] = y_max
                    self._bbox_buffer[3, 0] = x_min
                    self._bbox_buffer[3, 1] = y_max
                    
                    break

            post_time = (time.time() - post_start) * 1000
            total_loop_time = (time.time() - loop_start) * 1000

            self.timing_print_counter += 1
            if self.timing_print_counter >= 30:
                print(f"[TIMING] Inference: {inference_time:.2f}ms | Postproc: {post_time:.2f}ms | Total: {total_loop_time:.2f}ms")
                self.timing_print_counter = 0

            loop_duration = time.time() - loop_start
            sleep_time = max(0, (1 / TARGET_FPS) - loop_duration)
            time.sleep(sleep_time)

    def start(self):
        self.running = True
        self.initialized = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.initialized = False
        if hasattr(self.model, 'shutdown'):
            self.model.shutdown()

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        return self.latest_bgr_frame if self.latest_bgr_frame is not None else None

    def retrack(self):
        self.ball_position = None