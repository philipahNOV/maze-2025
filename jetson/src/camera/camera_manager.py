import cv2
import numpy as np
import pyzed.sl as sl

class CameraManager:
    def __init__(self):
        self.zed = sl.Camera()
        self.initialized = False

    def init_camera(self):
        if self.initialized:
            print("[CameraManager] Camera already initialized. Skipping open().")
            return

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("ZED camera failed to open.")
        
        print("[CameraManager] Camera initialized.")
        self.initialized = True

    def grab_frame(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            bgr = image.get_data()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return rgb, bgr
        return None, None

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

    def close(self):
        if self.initialized:
            self.zed.close()
            self.initialized = False
            print("[CameraManager] Camera closed.")
