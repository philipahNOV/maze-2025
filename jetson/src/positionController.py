import time
import numpy as np
import testing.yolov1.hsv3 as tracking
import arduino_connection

class Controller:

    """
    Handles position ad orientation control for a ball-on-plate system.

    Uses computer vision tracking and IMU to get the ball's position and orientation,
    applies PD control logic to compute desired platform tilt angles,
    and sends commands to the Arduino to actuate motors accordingly.
    """

    def __init__(self, arduinoThread: arduino_connection.ArduinoConnection, tracker: tracking.BallTracker):
        self.arduinoThread = arduinoThread
        self.prevPos = None
        self.pos = None
        self.prevVel = None
        self.vel = None
        self.ref = None
        self.prevError = None
        self.prevTime = None
        self.prevVelError = None
        self.ori = None
        self.tracker = tracker

        #ARDUINO PARAMETERS
        self.x_offset = 0  # Offset for x-axis orientation (tested -0.008)
        self.y_offset = 0  # Offset for y-axis orientation (tested -0.0015)
        self.min_velocity = 22 # Minimum velocity for motors

        #TUNING PARAMETERS
        #Pos control
        self.kp_x = 0.0001
        self.kd_x = 0.000
        self.kp_y = 0.00007
        self.kd_y = 0.00008
        self.deadzone_pos_tol = 30
        self.deadzone_vel_tol = 5
        self.deadzone_tilt = 0
        self.pos_tol = 10
        self.vel_tol = 1

        #Axis control
        self.kp_theta = 5000  # Proportional gain for the control loop

    def set_ball_pos(self, pos):
        self.pos = pos

    def posControl(self, ref):
        
        """
        Calculates the control angles (theta_x, theta_y) needed to move the ball toward the reference position.
        Applies PD control with deadzone handling, and updates previous error and time for derivative calculation.
        """

        self.pos = self.tracker.get_position()
        #self.pos = (pos[1], pos[0])


        if not self.pos: 
            print("No ball detected")
            return

        self.ref = ref

        if self.prevPos is not None:
            if abs(np.linalg.norm(np.array(self.pos) - np.array(self.prevPos))) > 300:
                print("Large jump detected, resetting position control.")
                return
            
        e_x = ref[0] - self.pos[0]
        e_y = ref[1] - self.pos[1]
            
        edot_x = 0
        edot_y = 0
        alpha = 0.5
        if self.prevError is not None and self.prevTime is not None:
            dt = time.time() - self.prevTime
            if dt > 0.0001:  # Avoid division by zero
                edot_x = (e_x - self.prevError[0]) / dt
                edot_y = (e_y - self.prevError[1]) / dt
                edot_x = alpha * edot_x + (1 - alpha) * self.prevVelError[0]
                edot_y = alpha * edot_y + (1 - alpha) * self.prevVelError[1]

        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol:
            # Ball is at target → STOP
            theta_x = 0
        elif abs(e_x) < self.deadzone_pos_tol and abs(edot_x) < self.deadzone_vel_tol:
            # Ball is close, but needs help moving → ESCAPE DEAD ZONE
            theta_x = np.sign(e_x) * self.deadzone_tilt
        else:
            # Ball is far → USE CONTROL
            theta_x = (self.kp_x * e_x  + self.kd_x * edot_x)

        if abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            # Ball is at target → STOP
            theta_y = 0
        elif abs(e_y) < self.deadzone_pos_tol and abs(edot_x) < self.deadzone_vel_tol:
            # Ball is close, but needs help moving → ESCAPE DEAD ZONE
            theta_y = -np.sign(e_y) * self.deadzone_tilt
        else:
            # Ball is far → USE CONTROL
            theta_y = (self.kp_y * e_y  + self.kd_y * edot_y)
            
        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol and abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            print("Target reached!")
            return
        
        #print(f"e_x: {e_x}, theta_x: {theta_x}, theta_y: {theta_y}, edot_x: {edot_x}, edot_y: {edot_y}")
        pos = self.tracker.get_position()
        print(f"\nBall position: {pos}\n")

        self.prevError = (e_x, e_y)
        self.prevVelError = (edot_x, edot_y)
        self.prevTime = time.time()

        self.axisControl((theta_y, theta_x))
        #time.sleep(0.05)

    def axisControl(self, ref):

        """
        Sends directional commands to the Arduino to tilt the platform toward the target angles.
        Uses proportional control based on orientation error to determine motor direction and velocity.
        """

        tol = 0.001

        self.ori = self.tracker.get_orientation()
        if not self.ori: return

        theta_x = self.ori[1] + self.x_offset
        theta_y = self.ori[0] + self.y_offset

        if theta_x is None or theta_y is None:
            print("Orientation data not available yet.")
            return
        
        e_x = theta_x - ref[0]
        e_y = theta_y - ref[1]
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

        vel_x = min(max(int(self.kp_theta * abs(e_x)), self.min_velocity), 255)
        vel_y = min(max(int(self.kp_theta * abs(e_y)), self.min_velocity), 255)
        dir_y = 2
        #print(f"e_x: {e_x}, theta_x: {theta_x}, dir_x: {dir_x}, vel_x: {vel_x}")
        self.arduinoThread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
        time.sleep(0.05)

    def horizontal(self, tol = 0.0015, timeLimit = 20):
        
        """
        Levels the platform using P-control based on camera orientation data.

        Continuously adjusts motor directions and speeds to minimize tilt (theta_x, theta_y),
        stopping when the platform is within a specified angular tolerance or a time limit is reached.

        Args:
            tol (float): Angular tolerance in radians (default: 0.0015).
            timeLimit (float): Maximum duration in seconds to attempt leveling (default: 20).
        """
        
        kp = 700 # Proportional gain for the control loop
        deadline = time.time() + timeLimit
        self.arduinoThread.send_target_positions(120, 120, 120, 120)  # Stop motors initially

        while time.time() < deadline:
            orientation = self.tracker.get_orientation()
            if not orientation: continue

            theta_x = orientation[1] + self.x_offset
            theta_y = orientation[0] + self.y_offset
            #print(orientation)

            if abs(theta_x) < tol and abs(theta_y) < tol:
                print("Orientation is within tolerance, stopping motors.")
                self.arduinoThread.send_target_positions(120, 120, 120, 120)
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

            print(f"{theta_x}, {theta_y}, {dir_x}, {dir_y}")
            vel_x = max(int(kp * abs(theta_x)), self.min_velocity)
            vel_y = max(int(kp * abs(theta_y)), self.min_velocity)
            self.arduinoThread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
            time.sleep(0.05)
        print("Deadline reached, stopping motors.")
        
