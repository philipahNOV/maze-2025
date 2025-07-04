import time
import numpy as np
import YOLO_tracking.hsv3 as tracking
import arduino_connection_test

class Controller:

    """
    Handles position and orientation control for a ball-on-plate system.

    Uses computer vision tracking and IMU to get the ball's position and orientation,
    applies PD control logic to compute desired platform tilt angles,
    and sends commands to the Arduino to actuate motors accordingly.
    """

    def __init__(self, arduinoThread: arduino_connection_test.ArduinoConnection, tracker: tracking.BallTracker, path_following=True):
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
        self.e_x_int = 0
        self.e_y_int = 0
        self.stuck = False

        self.use_feedforward = False
        self.use_feedforward_model = True
        self.feedforward_vector = (0, 0)

        self.path_following = path_following

        self.prev_vel_x = 0
        self.prev_vel_y = 0

        self.prev_command_time = time.time()

        #ARDUINO PARAMETERS
        self.x_offset = 0.002 # Offset for x-axis orientation (BEST SO FAR: -0.01)
        self.y_offset = 0.001 # Offset for y-axis orientation (tested -0.0015)
        self.min_velocity = 22 # Minimum velocity for motors
        self.min_vel_diff = 10
        self.vel_max = 100

        #TUNING PARAMETERS
        #Pos control

        #Best so far (setpoint):
        #self.kp_x = 0.00008
        #self.kd_x = 0.00005
        #self.kp_y = 0.00008
        #self.kd_y = 0.00005
        #self.ki_y = 0.0006
        #self.ki_x = 0.0006

        #Best so far (pathfollowing)
        self.kp_x = 0.00003
        self.kd_x = 0.00013
        self.kp_y = 0.00003
        self.kd_y = 0.00013
        self.ki_y = 0.0002
        self.ki_x = 0.0002
        self.kf_min = 0.02
        self.kf_max = 0.005
        self.deadzone_pos_tol = 30
        self.deadzone_vel_tol = 10
        self.deadzone_tilt = np.deg2rad(0)
        self.pos_tol = 40
        self.vel_tol = 20

        #Axis control
        self.kp_theta = 6500  # Proportional gain for the control loop
        self.max_angle = 1.8 #Max angle in deg
        self.command_delay = 0.0001

    def set_pid_parameters(self, params):
        param_names = ["x_offset", "y_offset", "kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "kf_min", "kf_max"]
        for i, name in enumerate(param_names):
            if params[i] != "pass":
                setattr(self, name, params[i])

    def set_ball_pos(self, pos):
        self.pos = pos

    def significant_motor_change(self, vel_x, vel_y):
        if abs(vel_x-self.prev_vel_x) < self.min_vel_diff and abs(vel_y-self.prev_vel_y) < self.min_vel_diff:
            return False
        return True
    
    def saturate_angles(self, theta_x, theta_y):
        rad = np.deg2rad(self.max_angle)
        theta_x = max(min(theta_x, rad), -rad)
        theta_y = max(min(theta_y, rad), -rad)
        return (theta_x, theta_y)

    def posControl(self, ref):
        
        """
        Calculates the control angles (theta_x, theta_y) needed to move the ball toward the reference position.
        Applies PD control with deadzone handling, and updates previous error and time for derivative calculation.
        """

        self.pos = self.tracker.get_position()
        #print(f"Target: {ref}, Ball: {self.pos}")
        #self.pos = (pos[1], pos[0])


        if not self.pos: 
            print("No ball detected")
            return

        self.ref = ref

        if self.prevPos is not None:
            if abs(np.linalg.norm(np.array(self.pos) - np.array(self.prevPos))) > 300:
                #print("Large jump detected, resetting position control.")
                self.prevTime = time.time()
                return
            
        e_x = ref[0] - self.pos[0]
        e_y = ref[1] - self.pos[1]
            
        edot_x = 0
        edot_y = 0
        alpha = 0.85
        if self.prevError is not None and self.prevTime is not None:
            dt = time.time() - self.prevTime
            if dt > 0.0001:  # Avoid division by zero
                edot_x = (e_x - self.prevError[0]) / dt
                edot_y = (e_y - self.prevError[1]) / dt
                edot_x = alpha * edot_x + (1 - alpha) * self.prevVelError[0]
                edot_y = alpha * edot_y + (1 - alpha) * self.prevVelError[1]
                edot_x = max(min(edot_x, 300), -300)
                edot_y = max(min(edot_y, 300), -300)
        else:
            dt = 0

        self.e_x_int += e_x * dt
        self.e_y_int += e_y * dt

        ff_x = 0
        ff_y = 0
        if self.use_feedforward:
            # Compute distance between ball and next target (used as scaling factor)
            dx = self.ref[0] - self.pos[0]
            dy = self.ref[1] - self.pos[1]
            distance = np.linalg.norm((dx, dy))

            # Normalize distance to a 0–1 range (assume 0–500 pixels is typical range)
            distance_norm = min(distance / 500, 1.0)

            # Interpolate gain
            kf_dynamic = self.kf_min + (self.kf_max - self.kf_min) * distance_norm

            # Apply dynamic gain to unit direction vector
            ff_x = kf_dynamic * self.feedforward_vector[0]
            ff_y = kf_dynamic * self.feedforward_vector[1]

        elif self.use_feedforward_model:
            # Compute distance between ball and next target (used as scaling factor)
            dx = self.ref[0] - self.pos[0]
            dy = self.ref[1] - self.pos[1]
            distance = np.linalg.norm((dx, dy))
            direction = (dx / distance, dy / distance) if distance > 1e-6 else (0, 0)

            # Estimate time to reach waypoint (tuned value or based on velocity)
            t_estimate = 5  # seconds, tune this based on performance

            
            # Desired acceleration magnitude
            a_mag = 2 * distance / (t_estimate ** 2)  # s = 0.5*a*t^2 → a = 2s/t²

            # Compute acceleration vector
            a_x = a_mag * direction[0]
            a_y = a_mag * direction[1]

            # Estimate tilt angles from model
            a_model = 5.5 / 0.0122
            ff_x = a_x / a_model
            ff_y = a_y / a_model

        if abs(edot_x) > 30 or abs(e_x) > 60: self.e_x_int = 0
        if abs(edot_y) > 30 or abs(e_y) > 60: self.e_y_int = 0

        if self.e_x_int > 0 or self.e_y_int > 0: pass#print(f"Integral term active: {edot_x}, {edot_y}")
        else: pass#print("Integral term NOT active")


        if abs(e_x) < self.pos_tol: #at target → STOP
            theta_x = 0
        elif abs(e_x) < self.deadzone_pos_tol and abs(edot_x) < self.deadzone_vel_tol:
            # Ball is close, but needs help moving → ESCAPE DEAD ZONE
            theta_x = np.sign(e_x) * self.deadzone_tilt
            #print("Escaping")
        else:
            # Ball is far → USE CONTROL
            theta_x = (self.kp_x * e_x  + self.kd_x * edot_x + self.ki_x * self.e_x_int + ff_x)

        if abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            # Ball is at target → STOP
            theta_y = 0
        elif abs(e_y) < self.deadzone_pos_tol and abs(edot_y) < self.deadzone_vel_tol:
            # Ball is close, but needs help moving → ESCAPE DEAD ZONE
            theta_y = -np.sign(e_y) * self.deadzone_tilt
            #print("Escaping")
        else:
            # Ball is far → USE CONTROL
            theta_y = (self.kp_y * e_y  + self.kd_y * edot_y + self.ki_y * self.e_y_int + ff_y)
            
        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol and abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            #print("Target reached!")
            self.e_x_int = 0
            self.e_y_int = 0
            if not self.path_following:
                self.arduinoThread.send_target_positions(0, 0)
                time.sleep(self.command_delay)
            self.prevTime = time.time()
            #return
        
        #print(f"e_x: {e_x}, theta_x: {theta_x}, theta_y: {theta_y}, edot_x: {edot_x}, edot_y: {edot_y}")
        pos = self.tracker.get_position()
        #print(f"\nBall position: {pos}\n")

        self.prevError = (e_x, e_y)
        self.prevVelError = (edot_x, edot_y)
        self.prevTime = time.time()
        self.ori = self.tracker.get_orientation()
        #if self.ori:
        #    print(f"({self.ori[1]}, {self.ori[0]})")

        dist = np.linalg.norm(np.array(pos) - np.array(ref))
        if np.sqrt(edot_x**2+edot_y**2) < self.vel_tol and dist > self.pos_tol:
            self.stuck = True
            #print("STUCK")
        else:
            #print(np.sqrt(edot_x**2+edot_y**2))
            self.stuck = False

        if self.stuck:
            theta_x += np.sign(e_x) * np.deg2rad(0.8) * np.sin(time.time() * 10)  # 20 Hz oscillation
            theta_y += np.sign(e_y) * np.deg2rad(0.8) * np.sin(time.time() * 10)  # 20 Hz oscillation
            #self.axisControl((theta_y, theta_x))
            #return
        #self.axisControl((np.deg2rad(1.5), 0))
        self.axisControl(self.saturate_angles(theta_y, theta_x))

    def axisControl(self, ref):

        """
        Sends directional commands to the Arduino to tilt the platform toward the target angles.
        Uses proportional control based on orientation error to determine motor direction and velocity.
        """

        if time.time() < self.prev_command_time + self.command_delay:
            return

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
            vel_x = 0
        else:
            vel_x = min(max(int(self.kp_theta * abs(e_x)), self.min_velocity), self.vel_max)
            vel_x *= - np.sign(e_x)

        if abs(e_y) < tol:
            vel_y = 0
        else:
            vel_y = min(max(int(self.kp_theta * abs(e_y)), self.min_velocity), self.vel_max)
            vel_y = - np.sign(e_y) * vel_y

        if self.stuck:
            self.arduinoThread.send_target_positions(vel_x, vel_y)
            self.prev_command_time = time.time()
        if self.significant_motor_change(vel_x, vel_y):
            self.arduinoThread.send_target_positions(vel_x, vel_y)
            self.prev_command_time = time.time()
        
        self.prev_vel_x = vel_x
        self.prev_vel_y = vel_y


    def horizontal(self, tol = 0.0015, timeLimit = 20):
        
        """
        Levels the platform using P-control based on camera orientation data.

        Continuously adjusts motor directions and speeds to minimize tilt (theta_x, theta_y),
        stopping when the platform is within a specified angular tolerance or a time limit is reached.

        Args:
            tol (float): Angular tolerance in radians (default: 0.0015).
            timeLimit (float): Maximum duration in seconds to attempt leveling (default: 20).
        """

        print("Stabilizing horizontally...")
        
        kp = 700 # Proportional gain for the control loop
        deadline = time.time() + timeLimit
        self.arduinoThread.send_target_positions(0, 0)  # Stop motors initially

        while time.time() < deadline:
            orientation = self.tracker.get_orientation()
            if not orientation: continue

            theta_x = orientation[1] + self.x_offset
            theta_y = orientation[0] + self.y_offset
            #print(orientation)

            if abs(theta_x) < tol and abs(theta_y) < tol:
                print("Orientation is within tolerance, stopping motors.")
                self.arduinoThread.send_target_positions(0, 0)
                return
            if abs(theta_x) < tol:
                vel_x = 0
            else:
                vel_x = min(max(int(kp * abs(theta_x)), self.min_velocity), 255)
                vel_x *= - np.sign(theta_x)

            if abs(theta_y) < tol:
                vel_y = 0
            else:
                vel_y = min(max(int(kp * abs(theta_y)), self.min_velocity), 255)
                vel_y *= - np.sign(theta_y)

            #print(f"{theta_x}, {theta_y}")
            self.arduinoThread.send_target_positions(vel_x, vel_y)
            time.sleep(self.command_delay)
        print("Deadline reached, stopping motors.")
        
