import threading
import time
import numpy as np
import cv2
from collections import deque
from tracking.model_loader import YOLOModel
import pyzed.sl as sl

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
        self._normalization_array = None
        self._bbox_buffer = np.zeros((4, 2), dtype=np.float32)

        self.hole_boxes = []
        self.hole_mask_initialized = False

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

    def initialize_static_hole_mask(self):
        if not self.camera:
            return

        rgb, _ = self.camera.grab_frame()
        if rgb is None:
            return

        results = self.model.predict(rgb)
        self.hole_boxes.clear()

        for box in results.boxes:
            if self.model.get_label(box.cls[0]) != "ball":
                continue
            self.hole_boxes.append(box.xyxy[0])  # store raw (x1, y1, x2, y2)

        self.hole_mask_initialized = True
        print(f"[BallTracker] Initialized hole mask with {len(self.hole_boxes)} regions")

    def producer_loop(self):
        while self.running:
            start = time.time()
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                self.frame_queue.append((rgb, bgr))
            time.sleep(max(0, (1 / 60) - (time.time() - start)))

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

                if self._is_in_hole(box.xyxy[0]):
                    continue  # filtered out

                h, w = rgb.shape[:2]
                x_center, y_center, width, height = box.xywh[0]
                cx, cy = int(x_center), int(y_center)
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
                obj.bounding_box_2d = self._bbox_buffer.copy()
                obj.label = int(box.cls[0])
                obj.probability = float(box.conf[0])
                obj.is_grounded = False
                custom_boxes.append(obj)

            self.frame_counter = 0
            if custom_boxes and self.zed_od_initialized and self.frame_counter % 3 == 0:
                self.camera.zed.ingest_custom_box_objects(custom_boxes)
                self.camera.zed.retrieve_custom_objects(self.objects, self.object_runtime_params)
            self.frame_counter += 1

            time.sleep(max(0, (1 / 60) - (time.time() - start)))

    def _is_in_hole(self, box_xyxy):
        bx1, by1, bx2, by2 = box_xyxy
        for hx1, hy1, hx2, hy2 in self.hole_boxes:
            iou = self._compute_iou((bx1, by1, bx2, by2), (hx1, hy1, hx2, hy2))
            if iou > 0.5:  # threshold can be tuned
                return True
        return False

    def _compute_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

        iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6)
        return iou

    def start(self):
        self.running = True
        self.init_zed_object_detection()
        self.initialized = True
        self.initialize_static_hole_mask()
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