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
        self.timing_print_counter = 0
        self._normalization_array = None
        self._bbox_buffer = np.zeros((4, 2), dtype=np.float32)

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

    def find_ball_cv_fast(self, gray_frame, last_position, search_radius=60):
        """Ultra-fast ball detection using pure computer vision around last known position"""
        if last_position is None:
            return None
            
        x, y = last_position
        h, w = gray_frame.shape
        
        # Define search region around last position
        x1 = max(0, x - search_radius)
        y1 = max(0, y - search_radius)
        x2 = min(w, x + search_radius)
        y2 = min(h, y + search_radius)
        
        roi = gray_frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        
        # Fast ball detection using thresholding and contours
        # Gray ball detection - look for darker circular objects
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        
        # Adaptive threshold to handle varying lighting
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY, 11, 2)
        
        # Invert so ball becomes white
        thresh = cv2.bitwise_not(thresh)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            best_contour = None
            best_score = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by reasonable ball size
                if 15 < area < 800:  # Adjust based on your ball size
                    # Check circularity
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * np.pi * area / (perimeter * perimeter)
                        
                        # Score based on area and circularity
                        score = area * circularity
                        if score > best_score and circularity > 0.4:  # Reasonably circular
                            best_score = score
                            best_contour = contour
            
            if best_contour is not None:
                # Get centroid
                M = cv2.moments(best_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + x1
                    cy = int(M["m01"] / M["m00"]) + y1
                    return (cx, cy)
        
        return None

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
            
            # Convert BGR to grayscale for CV detection
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            
            # Run YOLO detection
            results = self.model.predict(rgb)
            
            yolo_ball_pos = None
            custom_boxes = []
            
            for box in results.boxes:
                if self.model.get_label(box.cls[0]) != "ball":
                    continue

                h, w = rgb.shape[:2]
                x_center, y_center, width, height = box.xywh[0]
                cx, cy = int(x_center), int(y_center)
                yolo_ball_pos = (cx, cy)
                
                # Validate YOLO detection with CV to avoid tracking holes
                cv_validation = self.find_ball_cv_fast(gray, yolo_ball_pos, search_radius=30)
                
                if cv_validation is not None:
                    # CV confirms ball detection - use YOLO position but validated
                    self.ball_position = yolo_ball_pos
                    print(f"[BallTracker] YOLO detection validated by CV at {yolo_ball_pos}")
                else:
                    # CV couldn't validate - might be a hole, use fallback CV detection
                    fallback_pos = self.find_ball_cv_fast(gray, self.ball_position, search_radius=80)
                    if fallback_pos is not None:
                        self.ball_position = fallback_pos
                        print(f"[BallTracker] YOLO failed CV validation, using CV fallback at {fallback_pos}")
                    else:
                        print(f"[BallTracker] YOLO detection at {yolo_ball_pos} failed CV validation - likely a hole")
                        # Don't update ball_position, keep previous valid position
                        continue
                
                # Continue with ZED object detection setup
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
                obj.bounding_box_2d = self._bbox_buffer.copy()  # lil copy for ZED
                obj.label = int(box.cls[0])
                obj.probability = float(box.conf[0])
                obj.is_grounded = False
                custom_boxes.append(obj)

            # If YOLO didn't find anything, try pure CV detection around last known position
            if yolo_ball_pos is None and self.ball_position is not None:
                cv_pos = self.find_ball_cv_fast(gray, self.ball_position, search_radius=100)
                if cv_pos is not None:
                    self.ball_position = cv_pos
                    print(f"[BallTracker] Pure CV detection at {cv_pos}")

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