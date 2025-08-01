import threading
import time
import numpy as np
import cv2
from collections import deque
from tracking.model_loader import YOLOModel
import pyzed.sl as sl
from control.astar.board_masking import create_binary_mask

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="new-v8-fp16.engine"):
        self.model = YOLOModel(model_path)
        self.camera = camera
        self.tracking_config = tracking_config or {}
        self.running = False
        self.initialized = False
        self.frame_queue = deque(maxlen=3)
        self.latest_bgr_frame = None
        self.ball_position = None
        self.objects = sl.Objects()
        self.object_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
        self.zed_od_initialized = False
        self.timing_print_counter = 0
        self._normalization_array = None
        self._bbox_buffer = np.zeros((4, 2), dtype=np.float32)
        self._board_mask = None
        self._mask_frame_counter = 0

    def _is_ball_in_hole(self, image, cx, cy, ball_width, ball_height):
        if self._board_mask is None or self._mask_frame_counter % 10 == 0:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self._board_mask = create_binary_mask(gray)
        
        self._mask_frame_counter += 1
        
        h, w = image.shape[:2]
        
        ball_radius = max(ball_width, ball_height) / 2
        padding = ball_radius * 0.8  # Check 80% of ball area
        
        x_min = max(0, int(cx - padding))
        x_max = min(w, int(cx + padding))
        y_min = max(0, int(cy - padding))
        y_max = min(h, int(cy + padding))
        
        # Extract the ball area from the mask
        ball_mask_region = self._board_mask[y_min:y_max, x_min:x_max]
        
        if ball_mask_region.size == 0:
            return False
        
        # Check if the ball area is mostly in black regions (holes)
        # In the mask: white (255) = valid board area, black (0) = holes
        black_pixels = np.sum(ball_mask_region == 0)
        total_pixels = ball_mask_region.size
        
        # If more than 80% of the ball area is in black regions, it's in a hole
        black_ratio = black_pixels / total_pixels
        return black_ratio > 0.8

    def init_zed_object_detection(self):
        if not self.zed_od_initialized and hasattr(self.camera, 'zed'):
            tracking_params = sl.PositionalTrackingParameters()
            self.camera.zed.enable_positional_tracking(tracking_params)
            obj_param = sl.ObjectDetectionParameters()
            obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
            obj_param.enable_tracking = True
            obj_param.enable_segmentation = False
            self.camera.zed.enable_object_detection(obj_param)
            self.zed_od_initialized = True

    def producer_loop(self):
        while self.running:
            start = time.time()
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                self.frame_queue.append((rgb, bgr))
            TARGET_FPS = 60
            loop_duration = time.time() - start
            sleep_time = max(0, (1 / TARGET_FPS) - loop_duration)
            time.sleep(sleep_time)

    def consumer_loop(self):
        while self.running:
            start = time.time()
            if not self.frame_queue:
                time.sleep(0.001)
                continue

            rgb, bgr = self.frame_queue.popleft()
            self.latest_bgr_frame = bgr
            results = self.model.predict(rgb)

            custom_boxes = []
            for box in results.boxes:
                if self.model.get_label(box.cls[0]) != "ball":
                    continue

                confidence = float(box.conf[0])
                if confidence < 0.55:
                    self.ball_position = None
                    continue

                h, w = rgb.shape[:2]
                x_center, y_center, width, height = box.xywh[0]
                cx, cy = int(x_center), int(y_center)
                
                # Check if ball is fully in a black area of the board mask (indicating a hole)
                if self._is_ball_in_hole(bgr, cx, cy, width, height):
                    self.ball_position = None
                    continue
                
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

                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = self._bbox_buffer.copy()  # copy for ZED
                obj.label = int(box.cls[0])
                obj.probability = float(box.conf[0])
                obj.is_grounded = False
                custom_boxes.append(obj)

            self.frame_counter = 0
            if custom_boxes and self.zed_od_initialized and self.frame_counter % 3 == 0:
                self.camera.zed.ingest_custom_box_objects(custom_boxes)
                self.camera.zed.retrieve_custom_objects(self.objects, self.object_runtime_params)
            self.frame_counter += 1

            TARGET_FPS = 60
            loop_duration = time.time() - start
            sleep_time = max(0, (1 / TARGET_FPS) - loop_duration)
            time.sleep(sleep_time)

    def start(self):
        self.running = True
        self.init_zed_object_detection()
        self.initialized = True
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.initialized = False
        if self.zed_od_initialized and hasattr(self.camera, 'zed'):
            self.camera.zed.disable_object_detection()
            self.camera.zed.disable_positional_tracking()
            self.zed_od_initialized = False

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        return self.latest_bgr_frame if self.latest_bgr_frame is not None else None

    def get_tracked_objects(self):
        return self.objects.object_list

    def retrack(self):
        self.ball_position = None