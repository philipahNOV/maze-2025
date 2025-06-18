import sys
import cv2
import pyzed.sl as sl
import threading
import time
import numpy as np


class CameraThread(threading.Thread):
    """
    A class representing a camera thread.

    This class extends the `threading.Thread` class and provides functionality
    for capturing frames from a camera using the ZED SDK.

    Attributes:
        zed (sl.Camera): The ZED camera object.
        image_size (sl.Resolution): The size of the captured image.
        image_zed (sl.Mat): The ZED image matrix.
        latest_frame (numpy.ndarray): The latest captured frame.
        running (bool): Flag indicating if the camera thread is running.
    """

    def __init__(self):
        threading.Thread.__init__(self)
        self.zed = sl.Camera()

        # Set configuration parameters
        input_type = sl.InputType()
        if len(sys.argv) >= 2:
            input_type.set_from_svo_file(sys.argv[1])
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.VGA
        init.camera_fps = 100
        init.coordinate_units = sl.UNIT.MILLIMETER
        init.depth_mode = sl.DEPTH_MODE.NONE

        # Open the camera
        
        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            raise Exception("Failed to open camera")

        self.image_size = self.zed.get_camera_information().camera_configuration.resolution
        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.latest_frame = None
        self.running = True
        self.orientation = None
        self.orientation_deg = None
        self.ang_vel = None
        print("CameraThread initialized")
    
    def find_board_corners(self, img):
        """
        Finds the corners of a board in an image.

        Args:
            img (numpy.ndarray): The input image.

        Returns:
            tuple: A tuple containing the coordinates of the top-left corner of the board,
                   the width and height of the board, and the largest square contour found.

        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_square = None
        max_area = 0
        for contour in contours:
            epsilon = 0.1 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            area = cv2.contourArea(approx)
            if area > max_area:
                max_area = area
                largest_square = approx

        if largest_square is not None:
            x, y, w, h = cv2.boundingRect(largest_square)
            return (x, y, w, h), largest_square

        print("No image found")
        return (0, 0, gray.shape[1], gray.shape[0]), None

    def accumulate_contours(self, num_frames=5):
        """
        Accumulates contours from multiple frames.

        Args:
            num_frames (int): The number of frames to process. Defaults to 5.

        Returns:
            list: A list of tuples containing the coordinates of the bounding rectangle (x, y, w, h) and the contour.

        """
        contours_list = []
        runtime = sl.RuntimeParameters()
        for _ in range(num_frames):
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                img = self.image_zed.get_data()
                (x, y, w, h), contour = self.find_board_corners(img)
                if contour is not None:
                    contours_list.append((x, y, w, h, contour))
        return contours_list

    def run(self):
        """
        Runs the camera thread.

        This method continuously grabs frames from the camera and processes them.
        It retrieves the left image from the ZED camera, crops it, converts it to BGR format,
        and extracts orientation and angular velocity data from the IMU sensor.

        Returns:
            None
        """
        print("CameraThread started")
        shaped = False
        x, y, w, h = (200, 0, self.image_size.width - 300, self.image_size.height)
        #x, y, w, h = (0, 0, self.image_size.width, self.image_size.height)
        runtime = sl.RuntimeParameters()
        c = 0
        sensors_data = sl.SensorsData()
        while self.running:
            c += 1
            start = time.time()
            err = self.zed.grab(runtime)
            duration = time.time() - start
            print(f"Grab duration: {duration:.3f} seconds")
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                img = self.image_zed.get_data()
                # I have removed the board auto detection part because i do not have the time to fully test it. 
                # This feature will be added back in the future, hopefully.
                """if not shaped:
                    contours_list = self.accumulate_contours(num_frames=5)
                    if contours_list:
                        contours_list.sort(key=lambda x: x[2] * x[3], reverse=True)
                        for x, y, w, h, contour in contours_list:
                            if abs(w - h) / max(w, h) <= 0.10 and w > 100 and h > 100:
                                shaped = True
                                break
                    if not shaped:
                        continue
                """
                cropped_img = img[y:y+h, x:x+w]
                self.latest_frame = cv2.cvtColor(cropped_img, cv2.COLOR_BGRA2BGR)
                self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE) # Retrieve only frame synchronized data
                imu_data = sensors_data.get_imu_data()
            
                zed_imu_pose = sl.Transform()
                ox = round(imu_data.get_pose(zed_imu_pose).get_orientation().get()[0], 3)
                oy = round(imu_data.get_pose(zed_imu_pose).get_orientation().get()[1], 3)
                oz = round(imu_data.get_pose(zed_imu_pose).get_orientation().get()[2], 3)
                ow = round(imu_data.get_pose(zed_imu_pose).get_orientation().get()[3], 3)
                
                dir1 = ox + ow
                dir1_deg = dir1*180/np.pi
                dir2 = oy - oz
                dir2_deg = dir2*180/np.pi
                self.orientation = [-dir2, dir1]
                self.orientation_deg = [-dir2_deg, dir1_deg]
                self.ang_vel = imu_data.get_angular_velocity()
            else:
                print("Failed to grab frame:", repr(err))

    def stop(self):
        """
        Stops the camera thread.

        This method sets the `running` flag to False and closes the camera.
        """
        self.zed.close()
        self.running = False
        self.join()
        print("CameraThread stopped")
       

    def get_latest_frame(self):
        """
        Returns the latest captured frame.

        Returns:
            numpy.ndarray: The latest captured frame.
        """
        return self.latest_frame
