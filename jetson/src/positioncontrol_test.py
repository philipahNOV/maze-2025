from mqtt_client import MQTTClientJetson
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from arduino_connection import ArduinoConnection
from camera.cam_loop import CameraThread
from automatic import Automatic
import testing.ball_recognition
import numpy as np

from manual_part.manuel_main import elManuel

import time
import cv2

def initialize_component(component, name, retries=5, delay=2):
    for attempt in range(retries):
        try:
            comp_instance = component()
            print(f"{name} initialized on attempt {attempt + 1}")
            return comp_instance
        except Exception as e:
            print(f"Failed to initialize {name} on attempt {attempt + 1}: {e}")
            time.sleep(delay)
    raise Exception(f"Failed to initialize {name} after {retries} attempts")

try:
    camera_thread = initialize_component(CameraThread, "CameraThread")
    camera_thread.start()
except Exception as e:
    print(e)
    exit(1)

try:
    arduino_thread = initialize_component(ArduinoConnection, "ArduinoConnection")
except Exception as e:
    print(e)
    exit(1)

def axisControl(ref):
    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 3000  # Proportional gain for the control loop
    tol = 0.001

    theta_x = camera_thread.orientation[1] + x_offset
    theta_y = camera_thread.orientation[0] + y_offset

    if theta_x is None or theta_y is None:
        print("Orientation data not available yet.")
        return
    
    e_x = ref[0] - theta_x
    e_y = ref[1] - theta_y
    if abs(e_x) < tol:
        dir_x = 2
    elif e_x > 0:
        dir_x = 3
    elif e_x < 0:
        dir_x = 1
    if abs(e_y) < tol:
        dir_y = 2
    elif e_y > 0:
        dir_y = 3
    elif e_y < 0:
        dir_y = 1

    vel_x = max(int(kp * abs(e_x)), min_velocity)
    vel_y = max(int(kp * abs(e_y)), min_velocity)
    dir_y = 2
    print(f"e_x: {e_x}, theta_x: {theta_x}, dir_x: {dir_x}, vel_x: {vel_x}")
    arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)

def posControl(center, prev_center, e_prev, t_prev, edot_prev, ref=(200, 200), tol=1):
    kp = 0.00025  # Proportional gain for position control
    kd = 0.000  # Derivative gain for position control

    if prev_center is not None:
        if abs(np.linalg.norm(np.array(center) - np.array(prev_center))) > 300:
            print("Large jump detected, resetting position control.")
            return None, None
        
    e_x = ref[0] - center[0]
    e_y = ref[1] - center[1]
        
    edot_x = 0
    edot_y = 0
    alpha = 0.5
    if e_prev is not None and t_prev is not None:
        dt = time.time() - t_prev
        if dt > 0.0001:  # Avoid division by zero
            edot_x = (e_x - e_prev[0]) / dt
            edot_y = (e_y - e_prev[1]) / dt
            edot_x = alpha * edot_x + (1 - alpha) * edot_prev[0]
            edot_y = alpha * edot_y + (1 - alpha) * edot_prev[1]

    theta_x = -(kp * e_x  + kd * edot_x)
    theta_y = -(kp * e_y  + kd * edot_y)
    print(f"e_x: {e_x}, theta_x: {theta_x}, theta_y: {theta_y}, edot_x: {edot_x}, edot_y: {edot_y}")
    axisControl((theta_x, theta_y))
    return (e_x, e_y), time.time(), (edot_x, edot_y)

def horizontal(tol = 0.2):
    """
    Gradually levels the platform by adjusting actuator angles based on camera orientation.

    This function reads orientation data (theta_x, theta_y) from the camera and sends
    corresponding direction and velocity commands to the Arduino via the arduino_thread.
    It applies a proportional control strategy (P-controller) to minimize tilt in both
    axes, driving the platform toward a horizontal position.

    Parameters:
        tol (float): Acceptable angular deviation from horizontal in radians (default: 0.2).

    Behavior:
    - Adds optional offsets to compensate for sensor calibration error.
    - Sends stop command initially and when within tolerance.
    - Computes direction based on sign of tilt.
    - Computes velocity with a proportional gain (kp), respecting a minimum speed threshold.
    - Terminates either when the platform is level within tolerance or a time deadline is reached.
    """

    x_offset = 0  # Offset for x-axis orientation (tested -0.008)
    y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
    min_velocity = 22 # Minimum velocity for motors
    kp = 700 # Proportional gain for the control loop
    deadline = time.time() + 20  # 20 seconds deadline
    arduino_thread.send_target_positions(120, 120, 120, 120)  # Stop motors initially

    while time.time() < deadline:
        print(camera_thread.orientation)
        theta_x = camera_thread.orientation[1] + x_offset
        theta_y = camera_thread.orientation[0] + y_offset
        if theta_x is None or theta_y is None:
            print("Orientation data not available yet.")
            continue
        if abs(theta_x) < tol and abs(theta_y) < tol:
            print("Orientation is within tolerance, stopping motors.")
            arduino_thread.send_target_positions(120, 120, 120, 120)
            return
        if abs(theta_x) < tol:
            dir_x = 2
        elif theta_x > 0:
            dir_x = 3
        elif theta_x < 0:
            dir_x = 1
        if abs(theta_y) < tol:
            dir_y = 2
        elif theta_y > 0:
            dir_y = 3
        elif theta_y < 0:
            dir_y = 1

        vel_x = max(int(kp * abs(theta_x)), min_velocity)
        vel_y = max(int(kp * abs(theta_y)), min_velocity)
        arduino_thread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)
    print("Deadline reached, stopping motors.")

time.sleep(10)  # Allow time for Arduino connection to stabilize
horizontal(0.0015)

frame = camera_thread.latest_frame
center = None
prev_center = None
limit = time.time() + 100
e_prev = None
t_prev = None
edot_prev = (0, 0)
while time.time() < limit:
    frame = camera_thread.latest_frame
    if frame is None:
        continue
    #frame = cv2.resize(frame, (320, 240))  # Resize to a standard size if needed
    center, radius, masked_frame = testing.ball_recognition.detect_red_ball_frame(frame, center)
    if center is None:
        print("No ball detected, skipping frame.")
        continue
    cv2.circle(frame, center, 10, (0, 255, 0), 4)
    center = (center[1], center[0])  # Convert to (x, y) format for consistency
    print(f"Center: {center}")
    if limit - time.time() < 95:
        e_prev, t_prev, edot_prev = posControl(center, prev_center, e_prev, t_prev, edot_prev)
    prev_center = center

    cv2.imshow("Test Image", frame)
    cv2.imshow("Masked Frame", masked_frame)
    cv2.waitKey(1)  # This is necessary for the window to update

cv2.destroyAllWindows()


Center: (369, 182)
e_x: -169, theta_x: 0.04225, theta_y: -0.0045000000000000005, edot_x: -75.96584793669518, edot_y: 58.59157780341971
e_x: -0.046749999999999965, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 140
TEST
Center: (233, 284)
e_x: -33, theta_x: 0.00825, theta_y: 0.021, edot_x: 360.6076374575705, edot_y: -269.6471321677288
e_x: -0.08074999999999996, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 242
TEST
Center: (370, 189)
e_x: -170, theta_x: 0.0425, theta_y: -0.00275, edot_x: -180.55385020787025, edot_y: 115.40620434666315
e_x: -0.046499999999999965, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 139
TEST
Center: (353, 210)
e_x: -153, theta_x: 0.03825, theta_y: 0.0025, edot_x: -45.55603378655258, edot_y: 2.459648193035484
e_x: -0.05174999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 155
TEST
Center: (369, 195)
e_x: -169, theta_x: 0.04225, theta_y: -0.00125, edot_x: -62.7918404097108, edot_y: 38.74278364317509
e_x: -0.04874999999999997, theta_x: 0.09099999999999997, dir_x: 1, vel_x: 146
TEST
Center: (370, 194)
e_x: -170, theta_x: 0.0425, theta_y: -0.0015, edot_x: -34.06378777016062, edot_y: 22.039259386892766
e_x: -0.046499999999999965, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 139
TEST
Center: (368, 191)
e_x: -168, theta_x: 0.042, theta_y: -0.0022500000000000003, edot_x: -10.795594832054796, edot_y: 20.374078272984654
e_x: -0.047999999999999966, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 143
TEST
Center: (361, 181)
e_x: -161, theta_x: 0.04025, theta_y: -0.00475, edot_x: 15.223426900616584, edot_y: 39.645931017412295
e_x: -0.04874999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 146
No ball detected, skipping frame.
No ball detected, skipping frame.
TEST
Center: (278, 181)
e_x: -78, theta_x: 0.0195, theta_y: -0.00475, edot_x: 132.0633191115018, edot_y: 19.822965508706147
e_x: -0.06949999999999996, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 208
TEST
Center: (203, 243)
e_x: -3, theta_x: 0.00075, theta_y: 0.010750000000000001, edot_x: 283.2975743132913, edot_y: -169.69500677854697
e_x: -0.08824999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 264
TEST
Center: (180, 181)
e_x: 20, theta_x: -0.005, theta_y: -0.00475, edot_x: 223.24161212310995, edot_y: 135.09837260728244
e_x: -0.09399999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 281
TEST
Center: (152, 185)
e_x: 48, theta_x: -0.012, theta_y: -0.00375, edot_x: 199.07098038032674, edot_y: 55.0563042581024
e_x: -0.10099999999999996, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 302
TEST
Center: (133, 185)
e_x: 67, theta_x: -0.01675, theta_y: -0.00375, edot_x: 155.3546426881356, edot_y: 27.5281521290512
e_x: -0.10574999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 317
TEST
Center: (129, 186)
e_x: 71, theta_x: -0.017750000000000002, theta_y: -0.0035, edot_x: 88.31474440333531, edot_y: 11.104720299708724
e_x: -0.10774999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 323
TEST
Center: (125, 186)
e_x: 75, theta_x: -0.01875, theta_y: -0.0035, edot_x: 62.79644520385628, edot_y: 5.552360149854362
e_x: -0.10774999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (124, 185)
e_x: 76, theta_x: -0.019, theta_y: -0.00375, edot_x: 33.99529878102301, edot_y: 5.373256254022047
e_x: -0.10899999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 326
TEST
Center: (124, 186)
e_x: 76, theta_x: -0.019, theta_y: -0.0035, edot_x: 16.997649390511505, edot_y: 0.031218803236639836
e_x: -0.10899999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 326
TEST
Center: (124, 185)
e_x: 76, theta_x: -0.019, theta_y: -0.00375, edot_x: 8.498824695255752, edot_y: 2.6430498186707636
e_x: -0.10799999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (124, 186)
e_x: 76, theta_x: -0.019, theta_y: -0.0035, edot_x: 4.249412347627876, edot_y: -1.1481795366347693
e_x: -0.10799999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (124, 186)
e_x: 76, theta_x: -0.019, theta_y: -0.0035, edot_x: 2.124706173813938, edot_y: -0.5740897683173847
e_x: -0.10899999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 326
TEST
Center: (123, 186)
e_x: 77, theta_x: -0.01925, theta_y: -0.0035, edot_x: 3.66321377669899, edot_y: -0.28704488415869234
e_x: -0.10824999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 324
TEST
Center: (124, 185)
e_x: 76, theta_x: -0.019, theta_y: -0.00375, edot_x: -0.9401122591806554, edot_y: 2.6281967054508044
e_x: -0.10799999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (125, 186)
e_x: 75, theta_x: -0.01875, theta_y: -0.0035, edot_x: -3.279587211710573, edot_y: -1.4954327293948428
e_x: -0.10774999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (123, 186)
e_x: 77, theta_x: -0.01925, theta_y: -0.0035, edot_x: 4.241047374774677, edot_y: -0.7477163646974214
e_x: -0.10824999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 324
TEST
Center: (123, 186)
e_x: 77, theta_x: -0.01925, theta_y: -0.0035, edot_x: 2.1205236873873385, edot_y: -0.3738581823487107
e_x: -0.10824999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 324
TEST
Center: (124, 187)
e_x: 76, theta_x: -0.019, theta_y: -0.0032500000000000003, edot_x: -1.7466181946388142, edot_y: -2.9938091295068388
e_x: -0.10799999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 323
TEST
Center: (122, 186)
e_x: 78, theta_x: -0.0195, theta_y: -0.0035, edot_x: 4.050039429128468, edot_y: 0.9647696984705179
e_x: -0.10849999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 325
TEST
Center: (114, 186)
e_x: 86, theta_x: -0.021500000000000002, theta_y: -0.0035, edot_x: 22.225439201691856, edot_y: 0.48238484923525893
e_x: -0.11049999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 331
TEST
Center: (101, 186)
e_x: 99, theta_x: -0.02475, theta_y: -0.0035, edot_x: 41.93411857681894, edot_y: 0.24119242461762946
e_x: -0.11374999999999996, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 341
TEST
Center: (89, 187)
e_x: 111, theta_x: -0.02775, theta_y: -0.0032500000000000003, edot_x: 60.98777109783029, edot_y: -3.21446310514292
e_x: -0.11674999999999996, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 350
TEST
Center: (83, 186)
e_x: 117, theta_x: -0.02925, theta_y: -0.0035, edot_x: 46.46417600639996, edot_y: 1.0544835236760097
e_x: -0.11824999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 354
TEST
Center: (74, 187)
e_x: 126, theta_x: -0.0315, theta_y: -0.0032500000000000003, edot_x: 53.264624473069375, edot_y: -2.8097067348141502
e_x: -0.12049999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 361
TEST
Center: (76, 186)
e_x: 124, theta_x: -0.031, theta_y: -0.0035, edot_x: 19.965745174364, edot_y: 1.9284301636782697
e_x: -0.11999999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 359
TEST
Center: (75, 185)
e_x: 125, theta_x: -0.03125, theta_y: -0.00375, edot_x: 12.47834429472487, edot_y: 3.459686789382005
e_x: -0.12024999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 360
TEST
Center: (74, 186)
e_x: 126, theta_x: -0.0315, theta_y: -0.0035, edot_x: 8.889594515963083, edot_y: -0.9205789739096455
e_x: -0.12049999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 361
TEST
Center: (74, 183)
e_x: 126, theta_x: -0.0315, theta_y: -0.00425, edot_x: 4.444797257981541, edot_y: 7.848421943925828
e_x: -0.12149999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 364
TEST
Center: (78, 182)
e_x: 122, theta_x: -0.0305, theta_y: -0.0045000000000000005, edot_x: -8.26619329082758, edot_y: 6.546358951917502
e_x: -0.11949999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 358
TEST
Center: (69, 192)
e_x: 131, theta_x: -0.03275, theta_y: -0.002, edot_x: 22.704456427180133, edot_y: -26.546323938034497
e_x: -0.12174999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 365
TEST
Center: (73, 187)
e_x: 127, theta_x: -0.03175, theta_y: -0.0032500000000000003, edot_x: 1.850696276235828, edot_y: -1.396247047324449
e_x: -0.12174999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 365
TEST
Center: (73, 186)
e_x: 127, theta_x: -0.03175, theta_y: -0.0035, edot_x: 0.925348138117914, edot_y: 1.7664089365902278
e_x: -0.12174999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 365
TEST
Center: (72, 187)
e_x: 128, theta_x: -0.032, theta_y: -0.0032500000000000003, edot_x: 3.290121919437541, edot_y: -1.94424338208347
e_x: -0.12199999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 365
TEST
Center: (69, 188)
e_x: 131, theta_x: -0.03275, theta_y: -0.003, edot_x: 9.971894953927071, edot_y: -3.7477330224445016
e_x: -0.12174999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 365
TEST
Center: (57, 193)
e_x: 143, theta_x: -0.035750000000000004, theta_y: -0.00175, edot_x: 38.690657544250215, edot_y: -15.917495705925031
e_x: -0.12474999999999997, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 374
TEST
Center: (53, 187)
e_x: 147, theta_x: -0.03675, theta_y: -0.0032500000000000003, edot_x: 29.4664256515591, edot_y: 7.222897466188474
e_x: -0.12674999999999997, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 380
TEST
Center: (45, 187)
e_x: 155, theta_x: -0.03875, theta_y: -0.0032500000000000003, edot_x: 42.67730869759514, edot_y: 3.611448733094237
e_x: -0.12874999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 386
TEST
Center: (41, 188)
e_x: 159, theta_x: -0.03975, theta_y: -0.003, edot_x: 31.861837418589516, edot_y: -0.8250714009008684
e_x: -0.12974999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 389
TEST
Center: (38, 188)
e_x: 162, theta_x: -0.0405, theta_y: -0.003, edot_x: 24.352438006709054, edot_y: -0.4125357004504342
e_x: -0.12949999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 388
TEST
Center: (37, 190)
e_x: 163, theta_x: -0.04075, theta_y: -0.0025, edot_x: 15.248051012541524, edot_y: -6.349931868599212
e_x: -0.12974999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 389
TEST
Center: (34, 188)
e_x: 166, theta_x: -0.0415, theta_y: -0.003, edot_x: 16.44794696489629, edot_y: 2.7076483714507447
e_x: -0.13149999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 394
TEST
Center: (28, 188)
e_x: 172, theta_x: -0.043000000000000003, theta_y: -0.003, edot_x: 25.109867093730358, edot_y: 1.3538241857253723
e_x: -0.13199999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 395
TEST
Center: (21, 188)
e_x: 179, theta_x: -0.04475, theta_y: -0.003, edot_x: 35.34818813926674, edot_y: 0.6769120928626862
e_x: -0.13374999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 401
TEST
Center: (17, 186)
e_x: 183, theta_x: -0.04575, theta_y: -0.0035, edot_x: 28.750854629294068, edot_y: 5.876836326261691
e_x: -0.13574999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 407
TEST
Center: (12, 186)
e_x: 188, theta_x: -0.047, theta_y: -0.0035, edot_x: 29.90116915737534, edot_y: 2.9384181631308457
e_x: -0.13599999999999995, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 407
TEST
Center: (13, 187)
e_x: 187, theta_x: -0.04675, theta_y: -0.0032500000000000003, edot_x: 12.128926796616344, edot_y: -1.3524487005059032
e_x: -0.13574999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 407
TEST
Center: (13, 186)
e_x: 187, theta_x: -0.04675, theta_y: -0.0035, edot_x: 6.064463398308172, edot_y: 2.243286788886405
e_x: -0.13674999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 410
TEST
Center: (10, 187)
e_x: 190, theta_x: -0.0475, theta_y: -0.0032500000000000003, edot_x: 11.957194248520828, edot_y: -1.853344122012379
e_x: -0.13649999999999995, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 409
TEST
Center: (13, 186)
e_x: 187, theta_x: -0.04675, theta_y: -0.0035, edot_x: -2.8555585736448723, edot_y: 2.0180465049622396
e_x: -0.13574999999999998, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 407
TEST
Center: (12, 183)
e_x: 188, theta_x: -0.047, theta_y: -0.00425, edot_x: 1.5020141476748132, edot_y: 9.798403555972868
e_x: -0.13599999999999995, theta_x: 0.08899999999999997, dir_x: 1, vel_x: 407
TEST
Center: (11, 186)
e_x: 189, theta_x: -0.04725, theta_y: -0.0035, edot_x: 3.564480717377216, edot_y: -3.541219152632996
e_x: -0.13724999999999998, theta_x: 0.08999999999999997, dir_x: 1, vel_x: 411
TEST
