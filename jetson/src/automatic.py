import time

from camera.board import get_mm_per_pixel
from camera.path import get_path
from camera.ball import *
from camera.board import get_board
from stable_baselines3 import PPO
from math_part.math_main import State1_3
from ai_part.ai_main import State2_3
from camera.start_end import *
import cv2

def point_line_distance(point, start, end):
    point = np.array(point)
    start = np.array(start)
    end = np.array(end)
    if np.array_equal(start, end):
        return np.linalg.norm(point - start)
    else:
        return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)


class Automatic:
    def __init__(self, mqtt_client, camera_thread, arduino_thread):
        self.camera_thread = camera_thread
        self.mqtt_client = mqtt_client
        self.arduino_thread = arduino_thread
        self.path = None
        self.board = None
        self.start_list = []
        self.end_list = []
        self.start = None
        self.end = None
        self.old_ball = None
        self.ball_on_board = True
        self.goal_position = [0, 0]
        self.img = None
        self.mm_per_pixel = [0,0]
        self.last_frame_time = time.time()
        self.ball_travel_distance = [0, 0]
        self.mqtt_client.client.publish("jetson/path", "None", 0)
        self.model = None
        self.ball_dict = {}
        self.velocity = [0, 0]
        self.length_dict = 10
        self.create_ball_dict()
        self.states = {
            "1.3": State1_3(self.mqtt_client, self.camera_thread, self.arduino_thread),
            "2.3": State2_3(self.mqtt_client, self.camera_thread, self.arduino_thread),
        }
    
    def create_ball_dict(self):
        for i in range(1, self.length_dict + 1):
            self.ball_dict[str(i)] = {
                'found': None,
                'position': [0, 0],
                'time': 0,
                'confidence': 0.0
            }

    def update_ball_dict(self, new_ball):
        if len(self.ball_dict) >= self.length_dict:
            last_key = sorted(self.ball_dict.keys(), key=lambda x: int(x))[-1]
            del self.ball_dict[last_key]

        new_key = '1'
        if self.ball_dict:
            new_keys = sorted([int(k) for k in self.ball_dict.keys()], reverse=True)
            for key in new_keys:
                self.ball_dict[str(key + 1)] = self.ball_dict.pop(str(key))
        self.ball_dict[new_key] = new_ball
    
    def get_first_ball(self):
        for key in self.ball_dict:
            return self.ball_dict[key]
        
    def ball_no_ball(self, ball, current_time):
        if ball is not None:
            self.old_ball = ball
            self.update_ball_dict({
                'found': True,
                'position': [ball[0],ball[1]],
                'time': time.time(),
                'confidence': ball[2]
            })
        else:
            fist_ball = self.get_first_ball()
            if fist_ball is not None:
                positon_x = fist_ball['position'][0]
                positon_y = fist_ball['position'][1]
                delta_time = current_time - fist_ball['time']
                positon_x += self.velocity[0] * delta_time
                positon_y += self.velocity[1] * delta_time
                pos = [positon_x, positon_y]
                self.update_ball_dict({
                    'found': False,
                    'position': pos,
                    'time': time.time(),
                    'confidence': 0.0
                })
            else:
                self.update_ball_dict({
                    'found': None,
                    'position': [0,0],
                    'time': time.time(),
                    'confidence': 0.0
                })
                
    def ball_lost_in_last_frames(self, frames=10):
        if len(self.ball_dict) < frames:
            return False
        last_entries = list(self.ball_dict.values())[-frames:]
        return all(entry['found'] is False for entry in last_entries)
                
    def load_model(self, model_path):
        model = PPO.load(model_path)
        return model

    def transition_to(self, state_name):
        if state_name in self.states:
            self.states[state_name].run(self)

    
        
    def distance_to_goal(self):
        ball = self.get_first_ball()
        if ball is not None:
            position = ball['position']
        else:
            position = [0,0] # For handeling events where the ball = None
        
        distance = [self.goal_position[0] - position[0], self.goal_position[1] - position[1]]
        return distance
    
    def create_mask(self, hsv_image, lower_bounds, upper_bounds):
        mask = cv2.inRange(hsv_image, np.array(lower_bounds[0]), np.array(upper_bounds[0]))
        for lower, upper in zip(lower_bounds[1:], upper_bounds[1:]):
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, np.array(lower), np.array(upper)))
        return mask

    def find_yellowest_pixel(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = [(20, 100, 100)]
        upper_yellow = [(30, 255, 255)]
        yellow_mask = self.create_mask(hsv_image, lower_yellow, upper_yellow)
        _, max_val, _, max_loc = cv2.minMaxLoc(yellow_mask)
        return max_loc

    def find_redest_pixel(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = [(0, 100, 100), (170, 100, 100)]
        upper_red = [(10, 255, 255), (180, 255, 255)]
        red_mask = self.create_mask(hsv_image, lower_red, upper_red)
        _, max_val, _, max_loc = cv2.minMaxLoc(red_mask)
        return max_loc

    def state_1_0(self):
        print("State 1 running")
        while self.mqtt_client.jetson_state != "0.0":
            if self.path is not None:
                self.mqtt_client.client.publish("pi/command", "1.2", 0)
                self.state_1_2()
            time.sleep(0.1)
            if self.mqtt_client.jetson_state == "1.1":
                self.state_1_1()
                
    def state_1_1(self):
        """
        This method represents the state 1.1 of the automatic process.
        It scans for the board, calculates the mm per pixel, detects the start and end points,
        finds the path, and publishes the path and command to the MQTT client.
        """
        print("Scanning for board")
        self.path = None
        self.board = None
        self.mqtt_client.client.publish("jetson/path", "None", 0)
        
        self.img = self.camera_thread.get_latest_frame()
        self.mm_per_pixel = get_mm_per_pixel(self.img)
        print(f'MM per pixel: {self.mm_per_pixel}')

        # This code in not potimized and should be testen and worked on in the future 
        # to make it more efficient
        """while self.start is None or self.end is None:
            print(f'Length of start list: {len(self.start_list)}, Length of end list{len(self.end_list)}')
            if len(self.end_list) > 20 and len(self.start_list) > 20:
                start_array = np.array(self.start_list)
                end_array = np.array(self.end_list)
                self.start = np.median(start_array, axis=0)
                self.end = np.median(end_array, axis=0)
                print(f'Start: {self.start}, End: {self.end}')
            else:
                line = lines_(self.img)
                if line is not None:   
                    s, e = detect_shapes(line)
                    print(f's: {s}, e: {e}')
                    if s is not None:
                        self.start_list.append(s)
                    if e is not None:
                        self.end_list.append(e)"""
        
        # This code is temp. it only looks for green and red places in the image
        # When doing this, it interfears with ball. so the goal and start has to be removed each time
        """while self.start is None or self.end is None:
            self.img = self.camera_thread.get_latest_frame()
            greenest_pixel = self.find_yellowest_pixel(self.img)
            redest_pixel = self.find_redest_pixel(self.img)
            print("Greenest pixel:", greenest_pixel)
            print("Redest pixel:", redest_pixel)
            self.start = greenest_pixel
            self.end = redest_pixel

        if isinstance(self.start, np.ndarray):
            self.start = self.start.tolist()
        if isinstance(self.end, np.ndarray):
            self.end = self.end.tolist()
        while self.path is None:
            print("Finding path")
            self.img = self.camera_thread.get_latest_frame()
            self.path, self.board = get_path(self.img, self.start, self.end)
        self.path = rdp(self.path, epsilon=1.8)"""
        self.start = [100, 100]
        self.end = [300, 300]
        self.path = [[100, 100], [200, 200], [300, 300]]
        self.board = get_board(self.img)
        
        self.mqtt_client.client.publish("jetson/path", "Path", 0)
        self.mqtt_client.client.publish("pi/command", "1.2", 0)
        time.sleep(2)
        self.state_1_2()

    def state_1_2(self):
        print("State 1.2 running")
        self.arduino_thread.send_target_positions(-2, -2, 0, 0) # Stop the motors
        while self.mqtt_client.jetson_state == "1.2":
            time.sleep(0.1)
            if self.mqtt_client.jetson_state == "1.3":
                self.transition_to("1.3")
            elif self.mqtt_client.jetson_state == "2.3":
                self.transition_to("2.3")
            elif self.mqtt_client.jetson_state == "1.1":
                self.state_1_1()
            elif self.mqtt_client.jetson_state == "0.0":
                break