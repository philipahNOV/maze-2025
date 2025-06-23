import cv2
import numpy as np
from ultralytics import YOLO
import pyzed.sl as sl
import threading
import time

class BallTracker:
    def __init__(self, model_path="best.pt"):
        self.zed = None
        self.frame = None
        self.latest_frame = None
        self.lock = threading.Lock()

        self.tracked_objects = {
            "ball": {"position": None},
            "marker_1": {"position": None},
            "marker_2": {"position": None},
            "marker_3": {"position": None},
            "marker_4": {"position": None},
        }

        self.HSV_RANGES = {
            # "ball": (np.array([0, 100, 50]), np.array([10, 255, 180])), # green
            "ball": (np.array([0, 100, 50]), np.array([10, 255, 180])), # Red
            "marker": (np.array([0, 100, 100]), np.array([10, 255, 255])),
        }

        self.model = YOLO(model_path)
        self.WINDOW_SIZE = 50
        self.running = False
        self.initialized = False
        self.ball_confirm_counter = 0
        self.ball_confirm_threshold = 3

        self.latest_rgb_frame = None
        self.latest_bgr_frame = None

    def init_camera(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.coordinate_units = sl.UNIT.MILLIMETER
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("ZED camera failed to open")

    def grab_frame(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            bgr = image.get_data()  # ZED returns BGR directly (720, 1280, 3)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return rgb, bgr
        return None

    def producer_loop(self):
        while self.running:
            frames = self.grab_frame()
            if frames is not None:
                rgb_frame, bgr_frame = frames
                with self.lock:
                    self.latest_rgb_frame = rgb_frame
                    self.latest_bgr_frame = bgr_frame
            time.sleep(0.001)

    def get_center_of_mass(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    def hsv_tracking(self, frame, prev_pos, hsv_lower, hsv_upper):
        h, w = frame.shape[:2]
        if prev_pos is None:
            return None
        x, y = prev_pos
        x_min = max(0, x - self.WINDOW_SIZE)
        x_max = min(w, x + self.WINDOW_SIZE)
        y_min = max(0, y - self.WINDOW_SIZE)
        y_max = min(h, y + self.WINDOW_SIZE)
        roi = frame[y_min:y_max, x_min:x_max]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        local_center = self.get_center_of_mass(mask)
        if local_center:
            cx, cy = local_center
            return (x_min + cx, y_min + cy)
        return None

    def consumer_loop(self):
        while self.running:
            with self.lock:
                rgb_frame = self.latest_rgb_frame.copy() if self.latest_rgb_frame is not None else None
                bgr_frame = self.latest_bgr_frame.copy() if self.latest_bgr_frame is not None else None

            if rgb_frame is None or bgr_frame is None:
                time.sleep(0.01)
                continue

            if not self.initialized:
                results = self.model.predict(source=rgb_frame, conf=0.5)[0]
                for box in results.boxes:
                    cls = int(box.cls[0])
                    label = self.model.names[cls]
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # if label == "ball":
                    #     # (1130, 30)
                    #     if abs(cx - 1130) < 100 and abs(cy - 30) < 100:
                    #         self.ball_confirm_counter += 1
                    #         self.tracked_objects["ball"]["position"] = (cx, cy)
                    #         if self.ball_confirm_counter >= self.ball_confirm_threshold:
                    #             self.initialized = True



                    if label == "ball":
                        if abs(cx - 1130) > 60 or abs(cy - 30) > 60:
                            continue  # Skip detections too far from target region

                        roi = bgr_frame[y1:y2, x1:x2]
                        if roi.size == 0:
                            continue  # skip invalid ROI

                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(hsv, *self.HSV_RANGES["ball"])
                        nonzero_count = cv2.countNonZero(mask)

                        if nonzero_count > 50:
                            self.tracked_objects["ball"]["position"] = (cx, cy)
                            self.ball_confirm_counter += 1
                            print(f"[INFO] Ball HSV mean: {cv2.mean(hsv)}, non-zero: {nonzero_count}, position: ({cx}, {cy})")

                            if self.ball_confirm_counter >= self.ball_confirm_threshold:
                                self.initialized = True  # switch to HSV tracking


                    elif label.startswith("marker"):
                        self.tracked_objects[label]["position"] = (cx, cy)

            else:
                # use HSV tracking after ball has been initialized
                for label in self.tracked_objects:
                    hsv_lower, hsv_upper = self.HSV_RANGES["ball" if "ball" in label else "marker"]
                    prev_pos = self.tracked_objects[label]["position"]
                    new_pos = self.hsv_tracking(bgr_frame, prev_pos, hsv_lower, hsv_upper)
                    if new_pos:
                        self.tracked_objects[label]["position"] = new_pos

            self.frame = bgr_frame
            time.sleep(0.005)


    def start(self):
        self.init_camera()
        self.grab_frame()
        self.running = True

        self.producer_thread = threading.Thread(target=self.producer_loop)
        self.producer_thread.daemon = True
        self.producer_thread.start()

        self.consumer_thread = threading.Thread(target=self.consumer_loop)
        self.consumer_thread.daemon = True
        self.consumer_thread.start()


    def stop(self):
        self.running = False
        if self.zed:
            self.zed.close()

    def get_position(self):
        return self.tracked_objects["ball"]["position"]

    def get_orientation(self):
        sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            return None

        imu_data = sensors_data.get_imu_data()
        zed_imu_pose = sl.Transform()
        imu_orientation = imu_data.get_pose(zed_imu_pose).get_orientation().get()
        ox, oy, oz, ow = [round(v, 3) for v in imu_orientation]
        dir1 = ox + ow
        dir2 = oy - oz

        import math
        sinr_cosp = 2 * (ow * ox + oy * oz)
        cosr_cosp = 1 - 2 * (ox * ox + oy * oy)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (ow * oy - oz * ox)
        pitch = math.degrees(math.copysign(math.pi / 2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))

        return [-dir2, dir1]
