import time
import base64
import cv2
import numpy as np
from state_base import StateBase
from camera.ball import get_red_ball, ball_tracking
from camera.board import get_board
import signal
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class State1_3(StateBase):
    def __init__(self, mqtt_client, camera_thread, arduino_thread):
        self.mqtt_client = mqtt_client
        self.camera_thread = camera_thread
        self.arduino_thread = arduino_thread
        self.min_gain = 20
        self.k_p = 10e-1
        self.k_i = 0 #8e-5
        self.k_d = 10e-1
        
        self.setpointOrigo = (230.0, 285.0)
        self.x_target = self.setpointOrigo[0]
        self.y_target = self.setpointOrigo[1]
    
        self.max_pos_act = 18e-3 #mm
        self.max_vel_act = 0
        self.max_vel_act_level = 0
        
        self.L_beam = 15e-2

        self.current_target_index = 0
        self.x_integral = 0
        self.y_integral = 0
        self.prev_x_error = 0
        self.prev_y_error = 0
        self.prev_time = time.time()
        self.leveled = False

        self.prev_x = 0
        self.prev_y = 0
        self.img = self.camera_thread.latest_frame
        self.mqtt_client.client.publish("jetson/path", "None", 0)


    def go_to_level(self):
        accepted_value = np.radians(0.4)
        stable_time_required = .5  # Required time in seconds to stay within the accepted range
        stable_start_time = time.time()
        max_attempt_duration = 12  # Maximum time in seconds to attempt leveling
        start_time = time.time()
        target_angle = 0
        self.min_angle_error = np.radians(0.2)
        x_offset = np.radians(0)
        y_offset = np.radians(0)
        self.arduino_thread.send_target_positions(200, 205, 201, 799)

        while time.time() - start_time < max_attempt_duration:
            orientation = self.camera_thread.orientation
            if orientation is None:
                continue

            x_orientation = orientation[0] + x_offset
            y_orientation = orientation[1] + y_offset 

            x_error = target_angle - x_orientation
            y_error = target_angle - y_orientation

            if abs(x_orientation) <= accepted_value and abs(y_orientation) <= accepted_value:
                if time.time() - stable_start_time >= stable_time_required:
                    print("Successfully leveled")
                    return
            else:
                stable_start_time = time.time()
                
            if abs(x_error) < self.min_angle_error:
                dirMotor1 = 120 #Stå stille
            elif x_error < 0:
                dirMotor1 = 124
            else:
                dirMotor1 = 123

            if abs(y_error) < self.min_angle_error:
                dirMotor2 = 120
            elif y_error < 0:
                dirMotor2 = 124
            else:
                dirMotor2 = 123
            self.arduino_thread.send_target_positions(dirMotor1, dirMotor2, 120, 120)
            time.sleep(0.2)
            print(f"Angles: {x_error}, {y_error}")
        print("Failed to level within the maximum duration")

    def update_target(self, ball_position, context, frames_required=5):
        distance_to_target = np.hypot(self.x_target - ball_position[0], self.y_target - ball_position[1])
        if distance_to_target < 30: 
           self.frames_close_to_target += 1
        else:
           self.frames_close_to_target = 0

        if self.frames_close_to_target >= frames_required:
           self.current_target_index = (self.current_target_index + 1) % len(context.path)
           self.x_target, self.y_target = context.path[self.current_target_index]
           print(f"New target: {self.x_target}, {self.y_target}")
           self.frames_close_to_target = 0


    def run(self, context):
        c = 0
        def handle_exit(sig, frame):
            print("Script interrupted, sending stop command.")
            self.arduino_thread.send_target_positions(2, 2, 0, 0)
            plt.close('all')
            sys.exit(0)
    
        


        def update(c):
            if not self.leveled:
                self.go_to_level()
                self.arduino_thread.send_target_positions(120, 120, 0, 0) #Do not move command
                time.sleep(0.1)
                self.arduino_thread.send_target_positions(777, 777, 777, 777) #Save midpoints
                time.sleep(1)
                print("Leveled")
                self.leveled = True

            self.img = self.camera_thread.get_latest_frame()
            current_time = time.time()

            ball_position = get_red_ball(self.img, context.board)
            context.ball_no_ball(ball_position, current_time)
            ball_position = context.ball_dict['1']['position']

            time_diff = current_time - self.prev_time
            
            #print(f'The Ball is at {ball_position[0]}, {ball_position[1]}')

            x_error = self.x_target - ball_position[0]
            y_error = self.y_target - ball_position[1]

            self.x_integral += x_error * time_diff
            self.y_integral += y_error * time_diff

            x_derivative = (x_error - self.prev_x_error) / time_diff
            y_derivative = (y_error - self.prev_y_error) / time_diff

            u_x = round(self.k_p * x_error + self.k_i * self.x_integral + self.k_d * x_derivative) #This is the angle away from the midpoint 
            u_y = round(self.k_p * y_error + self.k_i * self.y_integral + self.k_d * y_derivative)

            self.arduino_thread.send_target_positions(u_x, u_y, 201, 799)


            self.prev_x_error = x_error
            self.prev_y_error = y_error
            self.prev_time = current_time
            self.prev_x = ball_position[0]
            self.prev_y = ball_position[1]

            if (current_time - context.last_frame_time) >= 0.05:  # 50 milliseconds 20 fps
                cv2.circle(self.img, (int(self.x_target), int(self.y_target)), 5, (0, 255, 255), -1)
                _, buffer = cv2.imencode('.jpg', self.img)
                jpg_as_text = base64.b64encode(buffer).decode('utf-8') # type: ignore
                context.mqtt_client.publish_image(jpg_as_text)
                context.last_frame_time = current_time  # Update the last frame time

            # Update the target to the next point in the path if conditions are met
            if c % 100 == 0:
                self.update_target(ball_position, context)
            c += 1
            return c

        while self.mqtt_client.jetson_state == "1.3":
            c = update(c)
            # Register the signal handler for SIGINT (Ctrl+C)
            signal.signal(signal.SIGINT, handle_exit)

        self.arduino_thread.send_target_positions(-2, -2, 0, 0)
        print("exiting state 1.3")
