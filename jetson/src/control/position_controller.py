import time
import numpy as np
from tracking.tracker_service import TrackerService
import arduino_connection
from logger import LoggingThread


class Controller:
    def __init__(self, arduinoThread: arduino_connection.ArduinoConnection,
                 tracker: TrackerService,
                 path_following=True,
                 lookahead=False,
                 config=None):
        self.arduinoThread = arduinoThread
        self.tracker = tracker
        self.config = config
        self.logger = None
        self.elevator_state = "down"  # Set to None if not using elevator, "down" otherwise

        # === INITIALIZATION ===
        self.prevPos = None
        self.pos = None
        self.prevVel = None
        self.vel = None
        self.ref = None
        self.prevError = None
        self.prevTime = None
        self.prevVelError = None
        self.ori = None
        self.looping = False
        self.stuck = False
        self.ball_velocity = None
        self.prev_waypoint = None
        self.stuck_timer_x = None
        self.stuck_timer_y = None
        self.stuck_time_threshold = config["controller"].get("stuck_time_threshold", 0.8)  # seconds
        self.stuck_x_active = False
        self.stuck_y_active = False
        self.unstuck_timer_x = None
        self.unstuck_timer_y = None
        self.stuck_unstuck_hold_time = config["controller"].get("stuck_unstuck_hold_time", 0.25)  # seconds

        self.prev_vel_x = 0
        self.prev_vel_y = 0
        self.e_x_int = 0
        self.e_y_int = 0
        self.feedforward_vector = (0, 0)
        self.prev_command = (0, 0)

        self.prev_command_time = time.time()

        # === ARDUINO PARAMETERS ===
        self.x_offset = config["controller"]["arduino"].get("x_offset", 0.002)
        self.y_offset = config["controller"]["arduino"].get("y_offset", 0.001)
        self.min_velocity = config["controller"]["arduino"].get("minimum_velocity", 22)
        self.min_vel_diff = config["controller"]["arduino"].get("minimum_velocity_difference", 10)
        self.vel_max = config["controller"]["arduino"].get("maximum_velocity", 100)

        # === PID TUNING PARAMETERS ===
        self.lookahead = lookahead
        self.integration_pos_threshold = config["controller"].get("integration_pos_threshold", 60)  # px
        self.integration_vel_threshold = config["controller"].get("integration_vel_threshold", 30)  # px/s
        if self.lookahead:
            self.kp_x = config["controller"]["position_controller_lookahead"].get("kp_x", 0.000015)
            self.kd_x = config["controller"]["position_controller_lookahead"].get("kd_x", 0.0002)
            self.kp_y = config["controller"]["position_controller_lookahead"].get("kp_y", 0.000015)
            self.kd_y = config["controller"]["position_controller_lookahead"].get("kd_y", 0.0002)
            self.ki_y = config["controller"]["position_controller_lookahead"].get("ki_y", 0.0)
            self.ki_x = config["controller"]["position_controller_lookahead"].get("ki_x", 0.0)
            self.pos_tol = config["controller"]["position_controller_lookahead"].get("position_tolerance", 40)
            self.vel_tol = config["controller"]["position_controller_lookahead"].get("velocity_tolerance", 20)
        else:
            self.kp_x = config["controller"]["position_controller_normal"].get("kp_x", 0.00003)
            self.kd_x = config["controller"]["position_controller_normal"].get("kd_x", 0.000145)
            self.kp_y = config["controller"]["position_controller_normal"].get("kp_y", 0.00003)
            self.kd_y = config["controller"]["position_controller_normal"].get("kd_y", 0.000145)
            self.ki_y = config["controller"]["position_controller_normal"].get("ki_y", 0.0002)
            self.ki_x = config["controller"]["position_controller_normal"].get("ki_x", 0.0002)
            self.pos_tol = config["controller"]["position_controller_normal"].get("position_tolerance", 40)
            self.vel_tol = config["controller"]["position_controller_normal"].get("velocity_tolerance", 20)

        # === AXIS CONTROL ===
        self.kp_theta = config["controller"]["angular_controller"].get("kp", 6500)
        self.max_angle = config["controller"]["angular_controller"].get("max_angle", 1.8)
        self.command_delay = config["controller"]["angular_controller"].get("command_delay", 0.0001)
        self.angle_tolerance = config["controller"]["angular_controller"].get("angle_tolerance", 0.001)  # rad

        # ====================

        self.ff_t_lookahead = config["controller"]["position_controller_lookahead"].get("feedforward_t", 6)
        self.ff_t_normal = config["controller"]["position_controller_normal"].get("feedforward_t", 5.3)
        self.stuck_wiggle_amplitude = config["controller"].get("stuck_wiggle_amplitude", 0.8)  # degrees
        self.stuck_wiggle_frequency = config["controller"].get("stuck_wiggle_frequency", 10)  # Hz
        self.stuck_vel_threshold = config["controller"].get("stuck_velocity_threshold", 20)  # px/s
        self.stuck_upper_pos_threshold = config["controller"].get("stuck_upper_position_threshold", 90)  # px

        self.velocity_smoothing_alpha = config["controller"].get("velocity_smoothing_alpha", 0.9)

    def set_pid_parameters(self, params):
        param_names = ["x_offset", "y_offset", "kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "kf_min", "kf_max"]
        for i, name in enumerate(param_names):
            if params[i] != "pass":
                setattr(self, name, params[i])

    def set_ball_pos(self, pos):
        self.pos = pos

    def significant_motor_change_x(self, vel_x):
        return not (abs(vel_x - self.prev_command[0]) < self.min_vel_diff)

    def significant_motor_change_y(self, vel_y):
        return not (abs(vel_y - self.prev_command[1]) < self.min_vel_diff)

    def saturate_angles(self, theta_x, theta_y):
        rad = np.deg2rad(self.max_angle)
        return max(min(theta_x, rad), -rad), max(min(theta_y, rad), -rad)

    def posControl(self, ref):
        self.pos = self.tracker.get_ball_position()
        if not self.pos:
            if self.logger is not None:
                self.logger.update_state(None, self.ori, None, (0, 0))
            return

        self.ref = ref

        if self.prevPos and np.linalg.norm(np.array(self.pos) - np.array(self.prevPos)) > 300:
            self.prevTime = time.time()
            return

        e_x = ref[0] - self.pos[0]
        e_y = ref[1] - self.pos[1]

        edot_x = edot_y = 0
        alpha = self.velocity_smoothing_alpha
        if self.prevError and self.prevTime:
            dt = time.time() - self.prevTime
            if dt > 0.0001:
                edot_x = (e_x - self.prevError[0]) / dt
                edot_y = (e_y - self.prevError[1]) / dt
                edot_x = alpha * edot_x + (1 - alpha) * self.prevVelError[0]
                edot_y = alpha * edot_y + (1 - alpha) * self.prevVelError[1]
                edot_x = np.clip(edot_x, -300, 300)
                edot_y = np.clip(edot_y, -300, 300)
                self.ball_velocity = (edot_x, edot_y)
        else:
            dt = 0

        #--- Integral action ---
        self.e_x_int += e_x * dt
        self.e_y_int += e_y * dt
        if abs(edot_x) > self.integration_vel_threshold or abs(e_x) > self.integration_pos_threshold: self.e_x_int = 0 # Reset integral if large error or velocity
        if abs(edot_y) > self.integration_vel_threshold or abs(e_y) > self.integration_pos_threshold: self.e_y_int = 0 # Reset integral if large error or velocity

        #--- Feedforward ---
        ff_x = ff_y = 0
        dx = self.ref[0] - self.pos[0]
        dy = self.ref[1] - self.pos[1]
        distance = np.linalg.norm((dx, dy))
        direction = (dx / distance, dy / distance) if distance > 1e-6 else (0, 0)
        if self.lookahead:
            t_estimate = self.ff_t_lookahead
        else:
            t_estimate = self.ff_t_normal
        a_mag = 2 * distance / (t_estimate ** 2)
        a_x = a_mag * direction[0]
        a_y = a_mag * direction[1]
        a_model = 5.5 / 0.0122
        ff_x = a_x / a_model
        ff_y = a_y / a_model

        #--- Update logger if entered a new waypoint ---
        if np.linalg.norm(np.array((e_x, e_y))) < self.pos_tol and self.logger is not None:
            self.logger.set_waypoint(self.ref)

        #--- PID Control ---
        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol:
            theta_x = 0
        else:
            theta_x = self.kp_x * e_x + self.kd_x * edot_x + self.ki_x * self.e_x_int + ff_x

        if abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            theta_y = 0
        else:
            theta_y = self.kp_y * e_y + self.kd_y * edot_y + self.ki_y * self.e_y_int + ff_y

        if abs(e_x) < self.pos_tol and abs(edot_x) < self.vel_tol and abs(e_y) < self.pos_tol and abs(edot_y) < self.vel_tol:
            self.e_x_int = 0
            self.e_y_int = 0
            self.prevTime = time.time()

        self.prevError = (e_x, e_y)
        self.prevVelError = (edot_x, edot_y)
        self.prevTime = time.time()

        #--- Stuck prevention ---
        pos = self.pos
        self.ori = self.tracker.get_orientation()
        dist = np.linalg.norm(np.array(pos) - np.array(ref))
        #vel_mag = np.sqrt(edot_x ** 2 + edot_y ** 2)

        vel_x, vel_y = (edot_x, edot_y)
        dist_x = abs(e_x)
        dist_y = abs(e_y)

        #stuck_x = abs(vel_x) < self.stuck_vel_threshold and dist > self.pos_tol and dist < self.stuck_upper_pos_threshold
        #stuck_y = abs(vel_y) < self.stuck_vel_threshold and dist > self.pos_tol and dist < self.stuck_upper_pos_threshold
        stuck_x = False
        stuck_y = False
        if abs(vel_x) < self.stuck_vel_threshold and abs(e_x) > self.pos_tol:
            if self.stuck_timer_x is None:
                self.stuck_timer_x = time.time()
            elif time.time() - self.stuck_timer_x > self.stuck_time_threshold:
                self.stuck_x_active = True
                self.unstuck_timer_x = None
        else:
            self.stuck_timer_x = None
            if self.stuck_x_active:
                if self.unstuck_timer_x is None:
                    self.unstuck_timer_x = time.time()
                elif time.time() - self.unstuck_timer_x > self.stuck_unstuck_hold_time:
                    self.stuck_x_active = False
            else:
                self.unstuck_timer_x = None
        if abs(vel_y) < self.stuck_vel_threshold and abs(e_y) > self.pos_tol:
            if self.stuck_timer_y is None:
                self.stuck_timer_y = time.time()
            elif time.time() - self.stuck_timer_y > self.stuck_time_threshold:
                self.stuck_y_active = True
                self.unstuck_timer_y = None
        else:
            self.stuck_timer_y = None
            if self.stuck_y_active:
                if self.unstuck_timer_y is None:
                    self.unstuck_timer_y = time.time()
                elif time.time() - self.unstuck_timer_y > self.stuck_unstuck_hold_time:
                    self.stuck_y_active = False
            else:
                self.unstuck_timer_y = None

        #print(dist, self.stuck_x_active, self.stuck_y_active)
        if self.stuck_x_active or self.stuck_y_active:
            theta_x += np.sign(e_x) * np.deg2rad(self.stuck_wiggle_amplitude) * np.sin(time.time() * self.stuck_wiggle_frequency)
            theta_y += np.sign(e_y) * np.deg2rad(self.stuck_wiggle_amplitude) * np.sin(time.time() * self.stuck_wiggle_frequency)

        #if self.stuck_y_active:
        #    theta_y += np.sign(e_y) * np.deg2rad(self.stuck_wiggle_amplitude) * np.sin(time.time() * self.stuck_wiggle_frequency)

        # Send angles to axis control
        self.axisControl(self.saturate_angles(theta_y, theta_x))

    def axisControl(self, ref):
        if time.time() < self.prev_command_time + self.command_delay:
            return

        tol = self.angle_tolerance
        self.ori = self.tracker.get_orientation()
        if not self.ori:
            if self.logger is not None:
                self.logger.update_state(self.pos, None, self.ball_velocity, (0, 0))
            return

        theta_x = self.ori[1] + self.x_offset
        theta_y = self.ori[0] + self.y_offset

        e_x = theta_x - ref[0]
        e_y = theta_y - ref[1]

        vel_x = 0 if abs(e_x) < tol else -np.sign(e_x) * min(self.kp_theta * abs(e_x), self.vel_max)
        vel_y = 0 if abs(e_y) < tol else -np.sign(e_y) * min(self.kp_theta * abs(e_y), self.vel_max)
        if self.logger is not None:
            self.logger.update_state(self.pos, self.ori, self.ball_velocity, (vel_x, vel_y))

        if not self.significant_motor_change_x(vel_x):
            new_vel_x = self.prev_command[0]
        else:
            new_vel_x = np.sign(e_x) * min(max(abs(vel_x), self.min_velocity), self.vel_max)
        if not self.significant_motor_change_y(vel_y):
            new_vel_y = self.prev_command[1]
        else:
            new_vel_y = np.sign(e_y) * min(max(abs(vel_y), self.min_velocity), self.vel_max)

        if self.stuck_x_active or self.stuck_y_active or self.significant_motor_change_x(vel_x) or self.significant_motor_change_y(vel_y):
            vel_x = new_vel_x
            vel_y = new_vel_y
            self.arduinoThread.send_speed(vel_x, vel_y)
            self.prev_command = (vel_x, vel_y)
            self.prev_command_time = time.time()

        self.prev_vel_x = vel_x
        self.prev_vel_y = vel_y

    def horizontal(self):
        kp = self.config["controller"]["horizontal_controller"].get("kp", 700)
        tol = self.config["controller"]["horizontal_controller"].get("tolerance", 0.0015)
        timeLimit = self.config["controller"]["horizontal_controller"].get("time_limit", 300)
        deadline = time.time() + timeLimit
        self.arduinoThread.send_speed(0, 0)

        while time.time() < deadline:
            orientation = self.tracker.get_orientation()
            if not orientation:
                continue

            theta_x = orientation[1] + self.x_offset
            theta_y = orientation[0] + self.y_offset

            if abs(theta_x) < tol and abs(theta_y) < tol:
                self.arduinoThread.send_speed(0, 0)
                return

            vel_x = 0 if abs(theta_x) < tol else -np.sign(theta_x) * min(max(int(kp * abs(theta_x)), self.min_velocity), 255)
            vel_y = 0 if abs(theta_y) < tol else -np.sign(theta_y) * min(max(int(kp * abs(theta_y)), self.min_velocity), 255)
            #print(orientation)
            #print(f"Sending speeds: vel_x={vel_x}, vel_y={vel_y}")
            self.arduinoThread.send_speed(vel_x, vel_y)
            time.sleep(0.02)

    def get_last_command(self):
        return (self.prev_vel_x, self.prev_vel_y)