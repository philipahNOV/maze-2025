import threading
import time
import numpy as np
import cv2
from queue import Queue
from tracking.model_loader import YOLOModel
import pyzed.sl as sl

def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))
    x_min = (xywh[0] - 0.5 * xywh[2])
    x_max = (xywh[0] + 0.5 * xywh[2])
    y_min = (xywh[1] - 0.5 * xywh[3])
    y_max = (xywh[1] + 0.5 * xywh[3])
    output[0] = [x_min, y_min]
    output[1] = [x_max, y_min]
    output[2] = [x_max, y_max]
    output[3] = [x_min, y_max]
    return output

class BallTracker:
    def __init__(self, camera=None, tracking_config=None, model_path="v8-291.engine"):
        self.model = YOLOModel(model_path)
        self.camera = camera
        self.tracking_config = tracking_config or {}
        self.running = False
        self.initialized = False
        self.frame_queue = Queue(maxsize=1)
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
                if not self.frame_queue.full():
                    self.frame_queue.put_nowait((rgb, bgr))
            elapsed = time.time() - start
            sleep_time = max(0, (1/60) - elapsed)
            time.sleep(sleep_time)

    def consumer_loop(self):
        while self.running:
            loop_start = time.time()
            if self.frame_queue.empty():
                time.sleep(0.001)
                continue

            rgb, bgr = self.frame_queue.get()
            self.latest_bgr_frame = bgr

            inference_start = time.time()
            results = self.model.predict(rgb)
            inference_time = (time.time() - inference_start) * 1000

            custom_boxes = []
            post_start = time.time()
            for box in results.boxes:
                if self.model.get_label(box.cls[0]) != "ball":
                    continue

                xywh = box.xywh[0].cpu().numpy()
                h, w = rgb.shape[:2]
                x_center = xywh[0] / w
                y_center = xywh[1] / h
                width = xywh[2] / w
                height = xywh[3] / h
                cx, cy = int(xywh[0]), int(xywh[1])
                self.ball_position = (cx, cy)

                obj = sl.CustomBoxObjectData()
                obj.bounding_box_2d = xywh2abcd([x_center, y_center, width, height], rgb.shape)
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

            sleep_time = max(0, (1/60) - (time.time() - loop_start))
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
        return self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

    def get_tracked_objects(self):
        return self.objects.object_list

    def retrack(self):
        self.ball_position = None
