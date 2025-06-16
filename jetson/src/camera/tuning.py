import sys
import cv2
import numpy as np
import pyzed.sl as sl
import threading

class CameraThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.zed = sl.Camera()
        input_type = sl.InputType()
        if len(sys.argv) >= 2:
            input_type.set_from_svo_file(sys.argv[1])
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.VGA
        init.camera_fps = 100
        init.coordinate_units = sl.UNIT.MILLIMETER
        init.depth_mode = sl.DEPTH_MODE.NONE
        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            self.zed.close()
            exit(1)
        self.image_size = self.zed.get_camera_information().camera_configuration.resolution
        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.latest_frame = None
        self.running = True
        print("CameraThread initialized")
    

    def find_board_corners(self, img):
               
        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)
        
        
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_img = img.copy()
        
        largest_square = None
        max_area = 0
        for contour in contours:
            epsilon = 0.1 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            debug_img = img.copy()
                       
            if len(approx) == 4:  # Check if the contour has 4 vertices
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    largest_square = approx
        
        if largest_square is not None:
            x, y, w, h = cv2.boundingRect(largest_square)
            return x, y, w, h
        
        print("No image found")
        return (0, 0, gray.shape[1], gray.shape[0])

    def run(self):
        print("CameraThread started")
        shaped = False
        runtime = sl.RuntimeParameters()
        c = 0
        sensors_data = sl.SensorsData()
        x, y, w, h = (0, 0, self.image_size.width, self.image_size.height)
        while self.running:
            c += 1
            err = self.zed.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                img = self.image_zed.get_data()
                if not shaped:
                    x, y, w, h = self.find_board_corners(img)
                    shaped = True
            
                   
                
                if x is not None and y is not None and w is not None and h is not None:
                    cropped_img = img[y:y+h, x:x+w]
                    self.latest_frame = cv2.cvtColor(cropped_img, cv2.COLOR_BGRA2BGR)
                else:
                    self.latest_frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                
                self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE) # Retrieve only frame synchronized data
                imu_data = sensors_data.get_imu_data()
            else:
                print("Failed to grab frame:", repr(err))

    def stop(self):
        self.running = False
        self.zed.close()
        print("CameraThread stopped")

    def get_latest_frame(self):
        return self.latest_frame

def nothing(x):
    pass

def create_sliders():
    cv2.namedWindow('Sliders')
    cv2.createTrackbar('LowerH1', 'Sliders', 0, 180, nothing)
    cv2.createTrackbar('UpperH1', 'Sliders', 10, 180, nothing)
    cv2.createTrackbar('LowerH2', 'Sliders', 160, 180, nothing)
    cv2.createTrackbar('UpperH2', 'Sliders', 180, 180, nothing)
    cv2.createTrackbar('LowerS', 'Sliders', 69, 255, nothing)
    cv2.createTrackbar('UpperS', 'Sliders', 255, 255, nothing)
    cv2.createTrackbar('LowerV', 'Sliders', 70, 255, nothing)
    cv2.createTrackbar('UpperV', 'Sliders', 255, 255, nothing)

def get_hsv_ranges():
    lower_h1 = cv2.getTrackbarPos('LowerH1', 'Sliders')
    upper_h1 = cv2.getTrackbarPos('UpperH1', 'Sliders')
    lower_h2 = cv2.getTrackbarPos('LowerH2', 'Sliders')
    upper_h2 = cv2.getTrackbarPos('UpperH2', 'Sliders')
    lower_s = cv2.getTrackbarPos('LowerS', 'Sliders')
    upper_s = cv2.getTrackbarPos('UpperS', 'Sliders')
    lower_v = cv2.getTrackbarPos('LowerV', 'Sliders')
    upper_v = cv2.getTrackbarPos('UpperV', 'Sliders')
    lower_red1 = np.array([lower_h1, lower_s, lower_v])
    upper_red1 = np.array([upper_h1, upper_s, upper_v])
    lower_red2 = np.array([lower_h2, lower_s, lower_v])
    upper_red2 = np.array([upper_h2, upper_s, upper_v])
    return lower_red1, upper_red1, lower_red2, upper_red2

def find_ball_contours(mask, board, search_area=None):
    if search_area is not None:
        x_min, y_min, x_max, y_max = search_area
        mask = mask[y_min:y_max, x_min:x_max]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_centers = []
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y)) if search_area is None else (int(x) + x_min, int(y) + y_min)
        radius = int(radius)
        if 5 <= radius <= 20:
            if np.all(board[center[1], center[0]] == [255, 255, 255]):
                valid_centers.append((center, radius))
    return valid_centers

def get_red_ball(img, board, old_ball):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red1, upper_red1, lower_red2, upper_red2 = get_hsv_ranges()
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    search_area = None
    if old_ball is not None:
        search_radius = 50
        x_min = max(0, old_ball[0] - search_radius)
        y_min = max(0, old_ball[1] - search_radius)
        x_max = min(img.shape[1], old_ball[0] + search_radius)
        y_max = min(img.shape[0], old_ball[1] + search_radius)
        search_area = (x_min, y_min, x_max, y_max)
    valid_centers = find_ball_contours(mask, board, search_area)
    return valid_centers[0][0] if valid_centers else None

camera_thread = CameraThread()
camera_thread.start()
create_sliders()

while True:
    frame = camera_thread.get_latest_frame()
    if frame is not None:
        board = np.ones_like(frame) * 255  # Assuming a white board for testing
        red_ball_position = get_red_ball(frame, board, None)
        if red_ball_position:
            cv2.circle(frame, red_ball_position, 10, (0, 255, 0), 2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1, upper_red1, lower_red2, upper_red2 = get_hsv_ranges()
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        combined_frame = np.hstack((frame, masked_frame))
        cv2.imshow("Frame and Masked Frame", combined_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera_thread.stop()
cv2.destroyAllWindows()
