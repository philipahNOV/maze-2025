import pyzed.sl as sl
import cv2

class ZEDCamera:
    def init_camera(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 for 30 FPS
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("ZED camera failed to open.")
            raise RuntimeError("ZED camera failed to open.")

        return self.zed

    def grab_frame(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            rgba = image.get_data()
            rgb = cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return bgr
        return None

    def close(self):
        self.zed.close()