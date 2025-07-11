import time
import numpy as np
from camera.tracker_service import TrackerService
import arduino_connection


class Controller:
    """
    Handles position and orientation control for a ball-on-plate system.
    """

    def __init__(self, arduinoThread: arduino_connection.ArduinoConnection,
                 tracker: TrackerService,
                 path_following=True,
                 lookahead=False):
        self.arduinoThread = arduinoThread
        self.tracker = tracker

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

        self.use_feedforward = False
        self.use_feedforward_model = True
        self.feedforward_vector = (0, 0)

        self.looping = False

        self.path_following = path_following
        self.prev_vel_x = 0
        self.prev_vel_y = 0
        self.prev_command_time = time.time()

        # === ARDUINO PARAMETERS ===
        self.x_offset = 0.002
        self.y_offset = 0.001
        self.min_velocity = 22
        self.min_vel_diff = 10
        self.vel_max = 100

        # === PID TUNING PARAMETERS ===
        self.lookahead = lookahead
        if self.lookahead:
            self.kp_x = 0.000015
            self.kd_x = 0.0002
            self.kp_y = 0.000015
            self.kd_y = 0.0002
            self.ki_y = 0.0
            self.ki_x = 0.0
            self.kf_min = 0.02
            self.kf_max = 0.005
            self.deadzone_pos_tol = 30
            self.deadzone_vel_tol = 10
            self.deadzone_tilt = np.deg2rad(0)
            self.pos_tol = 40
            self.vel_tol = 20
        else:
            self.kp_x = 0.00003
            self.kd_x = 0.000145
            self.kp_y = 0.00003
            self.kd_y = 0.000145
            self.ki_y = 0.0002
            self.ki_x = 0.0002
            self.kf_min = 0.02
            self.kf_max = 0.005
            self.deadzone_pos_tol = 30
            self.deadzone_vel_tol = 10
            self.deadzone_tilt = np.deg2rad(0)
            self.pos_tol = 40
            self.vel_tol = 20

        # === AXIS CONTROL ===
        self.kp_theta = 6500
        self.max_angle = 1.8  # degrees
        self.command_delay = 0.0001

    def set_pid_parameters(self, params):
        param_names = ["x_offset", "y_offset", "kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "kf_min", "kf_max"]
        for i, name in enumerate(param_names):
            if params[i] != "pass":
                setattr(self, name, params[i])

    def set_ball_pos(self, pos):
        self.pos = pos

    def significant_motor_change(self, vel_x, vel_y):
        return not (abs(vel_x - self.prev_vel_x) < self.min_vel_diff and
                    abs(vel_y - self.prev_vel_y) < self.min_vel_diff)

    def saturate_angles(self, theta_x, theta_y):
        rad = np.deg2rad(self.max_angle)
        return max(min(theta_x, rad), -rad), max(min(theta_y, rad), -rad)

    def posControl(self, ref):
        self.pos = self.tracker.get_ball_position()  # ✅ updated API
        if not self.pos:
            print("No ball detected")
            return

        self.ref = ref

        if self.prevPos and np.linalg.norm(np.array(self.pos) - np.array(self.prevPos)) > 300:
            self.prevTime = time.time()
            return

        e_x = ref[0] - self.pos[0]
        e_y = ref[1] - self.pos[1]

        edot_x = edot_y = 0
        alpha = 0.9

        if self.prevError and self.prevTime:
            dt = time.time() - self.prevTime
            if dt > 0.0001:
                edot_x = (e_x - self.prevError[0]) / dt
                edot_y = (e_y - self.prevError[1]) / dt
                edot_x = alpha * edot_x + (1 - alpha) * self.prevVelError[0]
                edot_y = alpha * edot_y + (1 - alpha) * self.prevVelError[1]
                edot_x = np.clip(edot_x, -300, 300)
                edot_y = np.clip(edot_y, -300, 300)
        else:
            dt = 0

        self.e_x_int += e_x * dt
        self.e_y_int += e_y * dt

        ff_x = ff_y = 0
        if self.use_feedforward_model:
            dx = self.ref[0] - self.pos[0]
            dy = self.ref[1] - self.pos[1]
            distance = np.linalg.norm((dx, dy))
            direction = (dx / distance, dy / distance) if distance > 1e-6 else (0, 0)
            if self.lookahead:
                t_estimate = 6
            else:
                t_estimate = 5.3
            a_mag = 2 * distance / (t_estimate ** 2)
            a_x = a_mag * direction[0]
            a_y = a_mag * direction[1]
            a_model = 5.5 / 0.0122
            ff_x = a_x / a_model
            ff_y = a_y / a_model

        if abs(edot_x) > 30 or abs(e_x) > 60: self.e_x_int = 0
        if abs(edot_y) > 30 or abs(e_y) > 60: self.e_y_int = 0

        if abs(e_x) < self.pos_tol:
            theta_x = 0
        elif abs(e_x) < self.deadzone_pos_tol and abs(edot_x) < self.deadzone_vel_tol:
            theta_x = np.sign(e_x) * self.deadzone_tilt
        else:
            theta_x = self.kp_x * e_x + self.kd_x * edot_x + self.ki_x * self.e_x_int + ff_x

        if abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            theta_y = 0
        elif abs(e_y) < self.deadzone_pos_tol and abs(edot_y) < self.deadzone_vel_tol:
            theta_y = -np.sign(e_y) * self.deadzone_tilt
        else:
            theta_y = self.kp_y * e_y + self.kd_y * edot_y + self.ki_y * self.e_y_int + ff_y

        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol and abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            self.e_x_int = 0
            self.e_y_int = 0
            if not self.path_following:
                self.arduinoThread.send_speed(0, 0)
                time.sleep(self.command_delay)
            self.prevTime = time.time()

        self.prevError = (e_x, e_y)
        self.prevVelError = (edot_x, edot_y)
        self.prevTime = time.time()

        pos = self.pos
        self.ori = self.tracker.get_orientation()
        dist = np.linalg.norm(np.array(pos) - np.array(ref))
        vel_mag = np.sqrt(edot_x ** 2 + edot_y ** 2)

        if vel_mag < self.vel_tol and dist > self.pos_tol:
            self.stuck = True
        else:
            self.stuck = False

        if self.stuck:
            theta_x += np.sign(e_x) * np.deg2rad(0.8) * np.sin(time.time() * 10)
            theta_y += np.sign(e_y) * np.deg2rad(0.8) * np.sin(time.time() * 10)

        self.axisControl(self.saturate_angles(theta_y, theta_x))

    def axisControl(self, ref):
        if time.time() < self.prev_command_time + self.command_delay:
            return

        tol = 0.001
        self.ori = self.tracker.get_orientation()
        if not self.ori:
            return

        theta_x = self.ori[1] + self.x_offset
        theta_y = self.ori[0] + self.y_offset

        e_x = theta_x - ref[0]
        e_y = theta_y - ref[1]

        vel_x = 0 if abs(e_x) < tol else -np.sign(e_x) * min(max(int(self.kp_theta * abs(e_x)), self.min_velocity), self.vel_max)
        vel_y = 0 if abs(e_y) < tol else -np.sign(e_y) * min(max(int(self.kp_theta * abs(e_y)), self.min_velocity), self.vel_max)

        if self.stuck or self.significant_motor_change(vel_x, vel_y):
            self.arduinoThread.send_speed(vel_x, vel_y)
            self.prev_command_time = time.time()

        self.prev_vel_x = vel_x
        self.prev_vel_y = vel_y

    def horizontal(self, tol=0.0015, timeLimit=20):
        print("Stabilizing horizontally...")
        kp = 700
        deadline = time.time() + timeLimit
        self.arduinoThread.send_speed(0, 0)

        while time.time() < deadline:
            orientation = self.tracker.get_orientation()
            if not orientation:
                continue

            theta_x = orientation[1] + self.x_offset
            theta_y = orientation[0] + self.y_offset

            if abs(theta_x) < tol and abs(theta_y) < tol:
                print("Orientation is within tolerance, stopping motors.")
                self.arduinoThread.send_speed(0, 0)
                return

            vel_x = 0 if abs(theta_x) < tol else -np.sign(theta_x) * min(max(int(kp * abs(theta_x)), self.min_velocity), 255)
            vel_y = 0 if abs(theta_y) < tol else -np.sign(theta_y) * min(max(int(kp * abs(theta_y)), self.min_velocity), 255)

            self.arduinoThread.send_speed(vel_x, vel_y)
            time.sleep(self.command_delay)

        print("Deadline reached, stopping motors.")
