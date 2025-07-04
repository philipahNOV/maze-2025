import time
import numpy as np
import YOLO_tracking.hsv3 as tracking
import arduino_connection_test

class Controller:
    """
    Position and orientation controller for a ball-on-plate system.

    Attributes:
        arduinoThread (ArduinoConnection): Arduino communication interface.
        tracker (BallTracker): Vision-based tracker for ball position and orientation.
        Various control parameters for PD, feedforward, saturation, and deadzones.

    Methods:
        set_pid_parameters(params): Update controller gains and offsets.
        set_ball_pos(pos): Set current ball position.
        posControl(ref): Run position control loop toward reference.
        axisControl(ref): Run orientation control to match tilt angles.
        horizontal(tol, timeLimit): Level platform horizontally within tolerance.
    """

    def __init__(self, arduinoThread: arduino_connection_test.ArduinoConnection, tracker: tracking.BallTracker, path_following=True):
        # Interfaces
        self.arduinoThread = arduinoThread
        self.tracker = tracker

        # State tracking
        self.prevPos = None
        self.pos = None
        self.prevVel = None
        self.vel = None
        self.ref = None
        self.prevError = None
        self.prevTime = None
        self.prevVelError = None
        self.ori = None
        self.e_x_int = 0
        self.e_y_int = 0
        self.stuck = False

        # Feedforward control flags and values
        self.use_feedforward = False
        self.use_feedforward_model = True
        self.feedforward_vector = (0, 0)

        self.path_following = path_following

        # Motor command memory for change detection
        self.prev_vel_x = 0
        self.prev_vel_y = 0
        self.prev_command_time = time.time()

        # Arduino calibration and motor limits
        self.x_offset = 0.002
        self.y_offset = 0.001
        self.min_velocity = 22
        self.min_vel_diff = 10
        self.vel_max = 100

        # PID controller tuning parameters
        self.kp_x = 0.00003
        self.kd_x = 0.00013
        self.kp_y = 0.00003
        self.kd_y = 0.00013
        self.ki_y = 0.0002
        self.ki_x = 0.0002
        self.kf_min = 0.02
        self.kf_max = 0.005

        # Deadzone thresholds and tolerances
        self.deadzone_pos_tol = 30
        self.deadzone_vel_tol = 10
        self.deadzone_tilt = np.deg2rad(0)
        self.pos_tol = 40
        self.vel_tol = 20

        # Axis (orientation) control parameters
        self.kp_theta = 6500
        self.max_angle = 1.8
        self.command_delay = 0.0001

    def set_pid_parameters(self, params):
        """Update PID and feedforward parameters from a list."""
        param_names = ["x_offset", "y_offset", "kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "kf_min", "kf_max"]
        for i, name in enumerate(param_names):
            if params[i] != "pass":
                setattr(self, name, params[i])

    def set_ball_pos(self, pos):
        """Set the current ball position."""
        self.pos = pos

    def significant_motor_change(self, vel_x, vel_y):
        """Return True if motor speed changed significantly since last command."""
        return abs(vel_x - self.prev_vel_x) >= self.min_vel_diff or abs(vel_y - self.prev_vel_y) >= self.min_vel_diff

    def saturate_angles(self, theta_x, theta_y):
        """Clamp angles to max allowable range."""
        rad = np.deg2rad(self.max_angle)
        return (max(min(theta_x, rad), -rad), max(min(theta_y, rad), -rad))

    def posControl(self, ref):
        """
        Execute position control loop to move ball toward a target reference.

        Applies PD + optional feedforward control and updates state. Sends tilt commands.

        Args:
            ref (tuple): Target x, y position for the ball.
        """
        self.pos = self.tracker.get_position()
        if not self.pos:
            print("No ball detected")
            return

        self.ref = ref

        # Reject sudden position jumps (likely tracking error)
        if self.prevPos and np.linalg.norm(np.array(self.pos) - np.array(self.prevPos)) > 300:
            self.prevTime = time.time()
            return

        # Position error
        e_x = ref[0] - self.pos[0]
        e_y = ref[1] - self.pos[1]
        edot_x = edot_y = 0
        alpha = 0.85

        # Derivative calculation with smoothing
        if self.prevError and self.prevTime:
            dt = time.time() - self.prevTime
            if dt > 0.0001:
                edot_x = alpha * (e_x - self.prevError[0]) / dt + (1 - alpha) * self.prevVelError[0]
                edot_y = alpha * (e_y - self.prevError[1]) / dt + (1 - alpha) * self.prevVelError[1]
                edot_x = np.clip(edot_x, -300, 300)
                edot_y = np.clip(edot_y, -300, 300)
        else:
            dt = 0

        self.e_x_int += e_x * dt
        self.e_y_int += e_y * dt

        # Feedforward estimate (dynamic or model-based)
        ff_x = ff_y = 0
        if self.use_feedforward:
            distance = np.linalg.norm([e_x, e_y])
            kf_dynamic = self.kf_min + (self.kf_max - self.kf_min) * min(distance / 500, 1.0)
            ff_x = kf_dynamic * self.feedforward_vector[0]
            ff_y = kf_dynamic * self.feedforward_vector[1]

        elif self.use_feedforward_model:
            distance = np.linalg.norm([e_x, e_y])
            direction = (e_x / distance, e_y / distance) if distance > 1e-6 else (0, 0)
            a_mag = 2 * distance / (5 ** 2)
            a_model = 5.5 / 0.0122
            ff_x = (a_mag * direction[0]) / a_model
            ff_y = (a_mag * direction[1]) / a_model

        # Reset integral windup if error too large
        if abs(edot_x) > 30 or abs(e_x) > 60: self.e_x_int = 0
        if abs(edot_y) > 30 or abs(e_y) > 60: self.e_y_int = 0

        # Compute control output for each axis
        theta_x = 0 if abs(e_x) < self.pos_tol else self.kp_x * e_x + self.kd_x * edot_x + self.ki_x * self.e_x_int + ff_x
        theta_y = 0 if abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol else self.kp_y * e_y + self.kd_y * edot_y + self.ki_y * self.e_y_int + ff_y

        # Stop motors if target reached
        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol and abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            self.e_x_int = self.e_y_int = 0
            if not self.path_following:
                self.arduinoThread.send_target_positions(0, 0)
                time.sleep(self.command_delay)
            self.prevTime = time.time()

        pos = self.tracker.get_position()
        self.prevError = (e_x, e_y)
        self.prevVelError = (edot_x, edot_y)
        self.prevTime = time.time()
        self.ori = self.tracker.get_orientation()

        # Detect stuck condition
        dist = np.linalg.norm(np.array(pos) - np.array(ref))
        speed = np.sqrt(edot_x ** 2 + edot_y ** 2)
        self.stuck = speed < self.vel_tol and dist > self.pos_tol

        # Oscillate to escape local minima if stuck
        if self.stuck:
            theta_x += np.sign(e_x) * np.deg2rad(0.8) * np.sin(time.time() * 10)
            theta_y += np.sign(e_y) * np.deg2rad(0.8) * np.sin(time.time() * 10)

        self.axisControl(self.saturate_angles(theta_y, theta_x))
