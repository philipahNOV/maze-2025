import time

class controller:
    def __init__(self, arduionThread, cameraThread):
        self.arduinoThread = arduionThread
        self.cameraThread = cameraThread
        self.prevPos = None
        self.pos = None
        self.prevVel = None
        self.vel = None

    def horizontal(self, tol = 0.0015, timeLimit = 20):
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
        deadline = time.time() + timeLimit
        self.arduinoThread.send_target_positions(120, 120, 120, 120)  # Stop motors initially

        while time.time() < deadline:
            orientation = self.cameraThread.get_orientation()
            theta_x = orientation[1] + x_offset
            theta_y = orientation[0] + y_offset
            print(orientation)

            if theta_x is None or theta_y is None:
                print("Orientation data not available yet.")
                continue
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

            vel_x = max(int(kp * abs(theta_x)), min_velocity)
            vel_y = max(int(kp * abs(theta_y)), min_velocity)
            self.arduinoThread.send_target_positions(dir_x, dir_y, vel_x, vel_y)
            time.sleep(0.05)
        print("Deadline reached, stopping motors.")
        