# data_handler.py
import base64
import numpy as np
import cv2

class DataHandler:
    def __init__(self, check_screen_func):
        self.check_screen_func = check_screen_func
    def handle_camera_data(self, data):
        if self.check_screen_func() == "Screen3":
            image_data = base64.b64decode(data)
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)

    def handle_other_data(self, data):
        # Process other types of data
        print("Handling other data types")

    def close_resources(self):
        cv2.destroyAllWindows()
