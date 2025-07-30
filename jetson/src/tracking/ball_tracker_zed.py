import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
import pyzed.sl as sl

class BallTracker:
    def __init__(self, model_path="v8-291.onnx"):
        self.model = YOLOModel(model_path)
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_detections = []
        self.ball_position = None

        # ZED initialization
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED: {status}")

        self.image_zed = sl.Mat()
        self.zed_runtime_params = sl.RuntimeParameters()

        # Positional tracking and object detection
        tracking_params = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(tracking_params)

        object_params = sl.ObjectDetectionParameters()
        object_params.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        object_params.enable_tracking = True
        self.zed.enable_object_detection(object_params)

        self.objects = sl.Objects()
        self.object_runtime_params = sl.ObjectDetectionRuntimeParameters()

    def zed_loop(self):
        while self.running:
            if self.zed.grab(self.zed_runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
                frame = self.image_zed.get_data()
                if frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                with self.lock:
                    self.latest_frame = frame.copy()
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
                    self.zed.ingest_custom_box_objects(custom_boxes)
                    self.zed.retrieve_objects(self.objects, self.object_runtime_params)

            time.sleep(0.01)

    def start(self):
        self.running = True
        threading.Thread(target=self.zed_loop, daemon=True).start()
        threading.Thread(target=self.detection_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
        self.zed.close()

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def get_tracked_objects(self):
        return self.objects.object_list