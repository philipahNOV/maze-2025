import threading
import time
import numpy as np
import cv2
from collections import deque
from tracking.model_loader import YOLOModel
import pyzed.sl as sl

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="new-v8.engine"):
        self.model = YOLOModel(model_path)
        self.camera = camera
        self.tracking_config = tracking_config or {}
        self.running = False
        self.initialized = False
        self.frame_queue = deque(maxlen=1)
        self.latest_bgr_frame = None
        self.ball_position = None
        self.objects = sl.Objects()
        self.object_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
        self.zed_od_initialized = False

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
            print("[BallTracker] ZED object detection initialized")

    def producer_loop(self):
        while self.running:
            start = time.time()
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                self.frame_queue.append((rgb, bgr))
            elapsed = time.time() - start
            if elapsed < 1 / 60:
                time.sleep(1 / 60 - elapsed)

    def consumer_loop(self):
        while self.running:
            loop_start = time.time()
            if not self.frame_queue:
                time.sleep(0.001)
                continue

            rgb, bgr = self.frame_queue.popleft()
            self.latest_bgr_frame = bgr

            inference_start = time.time()
            results = self.model.predict(rgb)
            inference_time = (time.time() - inference_start) * 1000

            custom_boxes = []
            post_start = time.time()
            for box in results.boxes:
                if self.model.get_label(box.cls[0]) != "ball":
                    continue

                h, w = rgb.shape[:2]
                xywh = box.xywh[0].cpu().numpy() / np.array([w, h, w, h])
                x_center, y_center, width, height = xywh
                cx, cy = int(x_center * w), int(y_center * h)
                self.ball_position = (cx, cy)

                x_min = x_center - width / 2
                x_max = x_center + width / 2
                y_min = y_center - height / 2
                y_max = y_center + height / 2
                bounding_box_2d = np.array([
                    [x_min, y_min],
                    [x_max, y_min],
                    [x_max, y_max],
                    [x_min, y_max]
                ], dtype=np.float32)

                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = bounding_box_2d
                obj.label = int(box.cls[0])
                obj.probability = float(box.conf[0])
                obj.is_grounded = False
                custom_boxes.append(obj)

            post_time = (time.time() - post_start) * 1000

            ingest_start = time.time()
            if custom_boxes and self.zed_od_initialized:
                self.camera.zed.ingest_custom_box_objects(custom_boxes)
                self.camera.zed.retrieve_custom_objects(self.objects, self.object_runtime_params)
            ingest_time = (time.time() - ingest_start) * 1000

            total_loop_time = (time.time() - loop_start) * 1000
            print(f"[TIMING] Inference: {inference_time:.2f}ms | Postproc: {post_time:.2f}ms | ZED: {ingest_time:.2f}ms | Total: {total_loop_time:.2f}ms")

            elapsed = time.time() - loop_start
            if elapsed < 1 / 60:
                time.sleep(1 / 60 - elapsed)

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
