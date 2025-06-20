import pyzed.sl as sl
import cv2

class ZEDCamera:
    def init_camera(self):
        zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 for 30 FPS
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("ZED camera failed to open.")
            exit(1)

        return zed

    def grab_frame(self):
        image = sl.Mat()
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            rgba = image.get_data()
            rgb = cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return bgr
        return None

    
    def get_orientation(self):
        sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            imu_data = sensors_data.get_imu_data()
            zed_imu_pose = sl.Transform()
            imu_orientation = imu_data.get_pose(zed_imu_pose).get_orientation().get()
            ox, oy, oz, ow = [round(v, 3) for v in imu_orientation]
            dir1 = ox + ow
            dir2 = oy - oz
            return [-dir2, dir1]
        else:
            return None



    def close(self):
        self.zed.close()