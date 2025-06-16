import time
import base64
import cv2
from state_base import StateBase
from stable_baselines3 import PPO
from camera.ball import get_red_ball, ball_tracking
import numpy as np


class State2_3(StateBase):
    def __init__(self, mqtt_client, camera_thread, arduino_thread):
        self.arduino_thread = arduino_thread
        self.mqtt_client = mqtt_client
        self.camera_thread = camera_thread
        self.path_list = None
        self.leveled = False
        self.finished = False
        
    def calculate_target_positions(self, action_x, action_y):
        base_x = -1 - action_x
        base_y = -1 - action_y
        return base_x, base_y


    def go_to_level(self):
        accepted_value = 0.2
        stable_time_required = .5  # Required time in seconds to stay within the accepted range
        stable_start_time = time.time()
        max_attempt_duration = 100  # Maximum time in seconds to attempt leveling
        start_time = time.time()
        target_angle = 0
        Kp = 45

        while time.time() - start_time < max_attempt_duration:
            # Get the latest orientation from the camera thread
            print(f'Camera orientaion: {self.camera_thread.orientation_deg}')
            orientation = self.camera_thread.orientation_deg
            if orientation is None:
                continue

            x_orientation = orientation[0]
            y_orientation = orientation[1]

            x_error = target_angle - x_orientation
            y_error = target_angle - y_orientation

            if abs(x_orientation) <= accepted_value and abs(y_orientation) <= accepted_value:
                if time.time() - stable_start_time >= stable_time_required:
                    print("Successfully leveled")
                    return
            else:
                stable_start_time = time.time()
            
            x_velocity = Kp * x_error
            y_velocity = Kp * y_error

            x_velocity = max(-80, min(80, x_velocity))
            y_velocity = max(-80, min(80, y_velocity))
            if abs(x_velocity) < 30:
                dirMotor1 = 2
                x_velocity = 0
            elif x_velocity > 0:
                dirMotor1 = 1
                x_velocity = round(x_velocity) 
            else:
                dirMotor1 = 3
                x_velocity = round(abs(x_velocity))

            if abs(y_velocity) < 30:
                dirMotor2 = 2
                y_velocity = 0
            elif y_velocity > 0:
                dirMotor2 = 1
                y_velocity = round(y_velocity) 
            else:
                dirMotor2 = 3
                y_velocity = round(abs(y_velocity))

            if y_velocity == 0 and x_velocity == 0:
                return
            self.arduino_thread.send_target_positions(dirMotor1,dirMotor2,x_velocity,y_velocity)
            time.sleep(0.1)
        print("Failed to level within the maximum duration")

    def fallen_ball(self, context):
        self.arduino_thread.send_target_positions(300, 300, 300, 300)

    def board_running(self, context, last_action):
        while context.mqtt_client.jetson_state == "2.3":
            print(f'Camera orientaion: {self.camera_thread.orientation_deg}')
            if not self.leveled:
                print("Leveling the platform")
                self.go_to_level()
                self.arduino_thread.send_target_positions(-2, -2, 0, 0)
                self.leveled = True
            
            else:
                context.img = context.camera_thread.get_latest_frame()
                current_time = time.time()
                ball_position = get_red_ball(context.img, context.board)
    
                context.ball_no_ball(ball_position, current_time)

                if context.ball_lost_in_last_frames():
                    print("Ball lost in the last 10 frames")
                    context.ball_on_board = False
                    break

                for key, value in context.ball_dict.items():
                    if value['found'] is None:
                        continue
                    else:
                        ball_pos = value['position']
                context.velocity = ball_tracking(context.ball_dict)
                context.velocity = [context.velocity[0] * context.mm_per_pixel[0], context.velocity[1] * context.mm_per_pixel[1]]
                if context.goal_position[0] * context.mm_per_pixel[0] - ball_pos[0] * context.mm_per_pixel[0] < 10:
                    if context.goal_position[1] * context.mm_per_pixel[1] - ball_pos[1] * context.mm_per_pixel[1] < 10:
                        if self.path_list is not None:
                            self.path_list.pop(0)
                            if len(self.path_list) <= 0:
                                self.path_list = None
                                self.finished = True
                        
                            
                if not self.finished and self.path_list is not None:           
                    context.goal_position = [self.path_list[0][0] * context.mm_per_pixel[0], self.path_list[0][1] * context.mm_per_pixel[1]]


                angle = self.camera_thread.orientation_deg

                observation_x = [angle[0], context.velocity[0], context.goal_position[0] - ball_pos[0]]
                observation_x = np.array(observation_x)
                action_x = context.model.predict(observation_x)[0].item()

                observation_y = [angle[1], context.velocity[1], context.goal_position[1] - ball_pos[1]]
                observation_y = np.array(observation_y)
                action_y = context.model.predict(observation_y)[0].item()

                
                if last_action[0] != action_x or last_action[1] != action_y:
                    last_action = [action_x, action_y]
                    target_x, target_y = self.calculate_target_positions(action_x, action_y)
                    self.arduino_thread.send_target_positions(target_x, target_y, 60, 60)


                if (current_time - context.last_frame_time) >= 0.05:  # 50 milliseconds 20 fps
                    _, buffer = cv2.imencode('.jpg', context.img)
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8') # type: ignore
                    context.mqtt_client.publish_image(jpg_as_text)
                    context.last_frame_time = current_time  # Update the last frame time

                if context.mqtt_client.jetson_state == "1.2":
                    break
                elif context.mqtt_client.jetson_state == "0.0":
                    break
      
    def run(self, context):
        self.path_list = context.path
        print("Loading model")
        context.model = self.load_model("ai_part/models/modelRL_final")
        print("Model loaded")
        last_action = [-1,-1]
        while context.mqtt_client.jetson_state == "2.3":
            if context.ball_on_board:
                self.board_running(context, last_action)
            else:
                self.fallen_ball(context)
            
            self.arduino_thread.send_target_positions(-2, -2, 0, 0)
            print("exiting state 1.3")
    
    def load_model(self, model_path):
        return PPO.load(model_path) # type: ignore
