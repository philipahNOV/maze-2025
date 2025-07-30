import threading
import time
import numpy as np
import cv2
from tracking.model_loader import YOLOModel
import pyzed.sl as sl

def xywh2abcd(xywh, im_shape):
    """Convert YOLO xywh format to ZED bounding box corners"""
    output = np.zeros((4, 2))

    # Center / Width / Height -> BBox corners coordinates
    x_min = (xywh[0] - 0.5*xywh[2])
    x_max = (xywh[0] + 0.5*xywh[2])
    y_min = (xywh[1] - 0.5*xywh[3])
    y_max = (xywh[1] + 0.5*xywh[3])

    # A ------ B
    # | Object |
    # D ------ C
    output[0][0] = x_min
    output[0][1] = y_min
    output[1][0] = x_max
    output[1][1] = y_min
    output[2][0] = x_max
    output[2][1] = y_max
    output[3][0] = x_min
    output[3][1] = y_max
    return output

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="tracking/v8-291.onnx"):
        self.model = YOLOModel(model_path)
        self.camera = camera
        self.tracking_config = tracking_config or {}
        self.running = False
        self.initialized = False  # Add initialized property for TrackerService
        self.lock = threading.Lock()
        self.latest_rgb_frame = None
        self.latest_bgr_frame = None
        self.latest_detections = []
        self.ball_position = None

        self.objects = sl.Objects()
        self.object_runtime_params = sl.CustomObjectDetectionRuntimeParameters()  # Use Custom version
        
        # Initialize ZED object detection when camera is available
        self.zed_od_initialized = False

    def init_zed_object_detection(self):
        """Initialize ZED object detection - call this after camera is initialized"""
        if not self.zed_od_initialized and hasattr(self.camera, 'zed'):
            # Enable positional tracking first (required for object detection)
            tracking_params = sl.PositionalTrackingParameters()
            self.camera.zed.enable_positional_tracking(tracking_params)
            
            # Enable custom object detection
            obj_param = sl.ObjectDetectionParameters()
            obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
            obj_param.enable_tracking = True
            obj_param.enable_segmentation = False
            self.camera.zed.enable_object_detection(obj_param)
            
            self.zed_od_initialized = True
            print("[BallTracker] ZED object detection initialized")

    def producer_loop(self):
        while self.running:
            rgb, bgr = self.camera.grab_frame()
            if rgb is not None and bgr is not None:
                with self.lock:
                    self.latest_rgb_frame = rgb
                    self.latest_bgr_frame = bgr
            time.sleep(0.001)

    def consumer_loop(self):
        while self.running:
            with self.lock:
                rgb = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
                bgr = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

            if rgb is None or bgr is None:
                time.sleep(0.01)
                continue

            if rgb is not None:
                results = self.model.predict(rgb)
                custom_boxes = []
                for box in results.boxes:
                    label = self.model.get_label(box.cls[0])
                    if label == "ball":
                        # Get YOLO detection in xywh format (normalized)
                        xywh = box.xywh[0].cpu().numpy()  # [x_center, y_center, width, height]
                        
                        # Convert to pixel coordinates
                        h, w = rgb.shape[:2]
                        x_center = xywh[0] / w
                        y_center = xywh[1] / h  
                        width = xywh[2] / w
                        height = xywh[3] / h
                        
                        # Store 2D position for compatibility
                        cx, cy = int(xywh[0]), int(xywh[1])
                        self.ball_position = (cx, cy)

                        # Create ZED custom object using proper format from detector.py
                        obj = sl.CustomBoxObjectData()
                        obj.bounding_box_2d = xywh2abcd([x_center, y_center, width, height], rgb.shape)
                        obj.label = int(box.cls[0])  # Use actual class ID
                        obj.probability = float(box.conf[0])
                        obj.is_grounded = False  # Ball is not grounded
                        custom_boxes.append(obj)

                if custom_boxes and self.zed_od_initialized:
                    self.camera.zed.ingest_custom_box_objects(custom_boxes)
                    self.camera.zed.retrieve_custom_objects(self.objects, self.object_runtime_params)

            time.sleep(0.01)

    def start(self):
        self.running = True
        self.camera.init_camera()
        # Initialize ZED object detection after camera is ready
        self.init_zed_object_detection()
        self.initialized = True  # Set initialized flag for TrackerService
        threading.Thread(target=self.producer_loop, daemon=True).start()
        threading.Thread(target=self.consumer_loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.initialized = False  # Reset initialized flag
        if self.zed_od_initialized and hasattr(self.camera, 'zed'):
            self.camera.zed.disable_object_detection()
            self.camera.zed.disable_positional_tracking()
        self.camera.close()

    def get_position(self):
        return self.ball_position

    def get_frame(self):
        with self.lock:
            return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

    def get_tracked_objects(self):
        return self.objects.object_list

    def retrack(self):
        self.ball_position = None
