import cv2
import numpy as np
from ultralytics import YOLO
import pyzed.sl as sl
import math

def init_zed_camera():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 for 30 FPS
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED camera failed to open.")
        exit(1)

    return zed

def grab_zed_frame(zed):
    image = sl.Mat()
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        bgr = image.get_data()  # ZED returns BGR directly (720, 1280, 3)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return rgb, bgr
    return None, None


# Model will be loaded inside main()

# HSV ranges
HSV_RANGES = {
    "ball": (np.array([35, 80, 80]), np.array([85, 255, 255])),      # Green
    # "ball": (np.array([2, 207, 65]), np.array([8, 255, 85])),         # Red (shiny + non-shiny combined)
    # "ball": (np.array([0, 0, 90]), np.array([180, 60, 255])),       # Grey
    "marker": (np.array([0, 100, 100]), np.array([10, 255, 255])),  # Red
}

tracked_objects = {
    "ball": {"position": None},
    "marker_1": {"position": None},
    "marker_2": {"position": None},
    "marker_3": {"position": None},
    "marker_4": {"position": None},
}

WINDOW_SIZE = 80  # px radius around previous location

def get_center_of_mass(mask):
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

import math

# def get_orientation(zed):
#     sensors_data = sl.SensorsData()
#     if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
#         return None

#     imu_data = sensors_data.get_imu_data()
#     zed_imu_pose = sl.Transform()
#     imu_orientation = imu_data.get_pose(zed_imu_pose).get_orientation().get()
#     ox, oy, oz, ow = [round(v, 3) for v in imu_orientation]
#     dir1 = ox + ow
#     dir2 = oy - oz

#     # Euler angles (roll and pitch only)
#     import math
#     sinr_cosp = 2 * (ow * ox + oy * oz)
#     cosr_cosp = 1 - 2 * (ox * ox + oy * oy)
#     roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

#     sinp = 2 * (ow * oy - oz * ox)
#     if abs(sinp) >= 1:
#         pitch = math.degrees(math.copysign(math.pi / 2, sinp))
#     else:
#         pitch = math.degrees(math.asin(sinp))

#     print(f"Roll: {round(roll, 2)}°, Pitch: {round(pitch, 2)}°")
#     print(f"Orientation: dir1={dir1}, dir2={dir2}")
#     return [-dir2, dir1]



def hsv_tracking(frame, prev_pos, hsv_lower, hsv_upper):
    h, w = frame.shape[:2]
    if prev_pos is None:
        return None

    x, y = prev_pos
    x_min = max(0, x - WINDOW_SIZE)
    x_max = min(w, x + WINDOW_SIZE)
    y_min = max(0, y - WINDOW_SIZE)
    y_max = min(h, y + WINDOW_SIZE)

    roi = frame[y_min:y_max, x_min:x_max]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)

    local_center = get_center_of_mass(mask)
    if local_center:
        cx, cy = local_center
        return (x_min + cx, y_min + cy)
    return None

def draw_tracking(frame, label, position, color=(0,255,0)):
    if position:
        cv2.circle(frame, position, 8, color, -1)
        cv2.putText(frame, label, (position[0]+10, position[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

def get_position():
    pos = tracked_objects["ball"]["position"]
    if pos:
        print("Ball position: x={}, y={}".format(pos[0], pos[1]))
    
    else:
        print("Ball nto detected.")


def main():
    # Load YOLO model inside main to avoid slow import time
    model = YOLO("best.pt")
    #cap = cv2.VideoCapture(0)
    zed = init_zed_camera()

    initialized = False

    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         print("Failed to grab frame from camera.")
    #         break

    while True:
        rgb_frame, bgr_frame = grab_zed_frame(zed)
        if rgb_frame is None:
            print("Failed to grab frame from ZED camera.")
            break

        if not initialized:
            results = model.predict(source=rgb_frame, conf=0.6)[0]
            for box in results.boxes:
                cls = int(box.cls[0])
                label = model.names[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                if label == "ball":
                    if 390 <= cx <= 1120 and 10 <= cy <= 720:  # restrict to bounding box
                        roi = bgr_frame[y1:y2, x1:x2]
                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(hsv, *HSV_RANGES["ball"])
                        if cv2.countNonZero(mask) > 100:
                            tracked_objects["ball"]["position"] = (cx, cy)

                elif label.startswith("marker"):
                    tracked_objects[label]["position"] = (cx, cy)


            initialized = True
            continue

        for label in tracked_objects:
            hsv_lower, hsv_upper = HSV_RANGES["ball" if "ball" in label else "marker"]
            prev_pos = tracked_objects[label]["position"]
            new_pos = hsv_tracking(bgr_frame, prev_pos, hsv_lower, hsv_upper)

            if new_pos:
                tracked_objects[label]["position"] = new_pos

            draw_tracking(bgr_frame, label, tracked_objects[label]["position"])

        get_position()

        # get_orientation(zed)
        

        cv2.imshow("Tracking", bgr_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

    zed.close()
    #cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
