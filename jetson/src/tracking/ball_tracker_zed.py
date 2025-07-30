import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
from tracking.camera_manager import CameraManager
import pyzed.sl as sl

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="v8-291.onnx"):
        self.model = YOLOModel(model_path)
        self.camera = camera or CameraManager()
        self.tracking_config = tracking_config or {}
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_detections = []
        self.ball_position = None

        self.objects = sl.Objects()
        self.object_runtime_params = sl.ObjectDetectionRuntimeParameters()

    def camera_loop(self):
        while self.running:
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None:
                with self.lock:
                    self.latest_frame = rgb.copy()
            time.sleep(0.005)

    def detection_loop(self):
        while self.running:
            with self.lock:
                frame = self.latest_frame.copy() if self.latest_frame is not None else None

            if frame is not None:
                results = self.model.predict(frame)
                custom_boxes = []
                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    if label == "ball":
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        self.ball_position = (cx, cy)

                        obj = sl.CustomBoxObjectData()
                        obj.bounding_box_2d = np.array([
                            [x1, y1], [x2, y1], [x2, y2], [x1, y2]
                        ], dtype=np.float32)
                        obj.label = 0  # assuming ball class
                        obj.probability = float(box.conf[0])
                        obj.unique_object_id = sl.generate_unique_id()
                        obj.is_grounded = True
                        custom_boxes.append(obj)

                if custom_boxes:
                    self.camera.zed.ingest_custom_box_objects(custom_boxes)
                    self.camera.zed.retrieve_objects(self.objects, self.object_runtime_params)

            time.sleep(0.01)

    def start(self):
        self.running = True
        self.camera.init_camera()
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.detection_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.camera.close()

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def get_tracked_objects(self):
        return self.objects.object_list

    def retrack(self):
        self.ball_position = None
