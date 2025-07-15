# rl_logger.py
import json
import os
import threading
import time
from camera.tracker_service import TrackerService
import low_pass_filter
import numpy as np

class OfflineLogger:
    def __init__(self, save_path="ORL/rl_data", episode_limit=100):
        self.episode_limit = episode_limit
        self.current_episode = []
        self.episodes = []
        self.episode_counter = 0
        os.makedirs(save_path, exist_ok=True)
        self.save_path = save_path

    def log_step(self, state, action, reward, next_state, done):
        self.current_episode.append({
            "state": state,
            "action": action,
            "reward": reward,
            "next_state": next_state,
            "done": done
        })
        if done:
            self.episodes.append(self.current_episode)
            self.episode_counter += 1
            self.save_episode()
            self.current_episode = []

    def save_episode(self):
        path = os.path.join(self.save_path, f"episode_{self.episode_counter:03}.json")
        with open(path, 'w') as f:
            json.dump(self.episodes[-1], f, indent=2)

class LoggingThread(threading.Thread):
    def __init__(self, path):
        super().__init__(daemon=True)
        self.logger = OfflineLogger()
        self.target_hz = 20
        self._stop_event = threading.Event()

        self.path = path
        self.ball_position = None
        self.orientation = None
        self.ball_velocity = None
        self.motor_input = None
        self.reward = 0
        self.done = False
        self.current_waypoint = self.path[0] if self.path else None
        self.prev_waypoint = None
        self.visited_waypoints = []

        self.prev_state = None
        self.prev_action = None
        self.prev_reward = 0

    def run(self):
        LOOP_DT = 1.0 / self.target_hz
        while self.logger.episode_counter < self.logger.episode_limit and not self._stop_event.is_set():
            loop_start = time.time()

            x, y = self.ball_position if self.ball_position is not None else (0, 0)
            theta_x, theta_y = self.orientation if self.orientation is not None else (0, 0)
            vel_x, vel_y = self.ball_velocity if self.ball_velocity is not None else (0, 0)
            input_x, input_y = self.motor_input if self.motor_input is not None else (0, 0)

            state = [x, y, vel_x, vel_y, theta_x, theta_y] # [x, y, vx, vy, theta_x, theta_y]
            action = [input_x, input_y]  # [motor_x, motor_y]
            reward, done = self.calculate_reward()

            if self.prev_state is not None and self.prev_action is not None and state is not None:
                self.reward += reward

                self.logger.log_step(
                    state=self.prev_state,
                    action=self.prev_action,
                    reward=self.prev_reward,
                    next_state=state,
                    done=done
                )

            self.prev_state = state
            self.prev_action = action
            self.prev_reward = self.reward

            loop_duration = time.time() - loop_start
            sleep_time = LOOP_DT - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._stop_event.set()

    def update_state(self, ball_position, orientation, ball_velocity, motor_input):
        self.ball_position = ball_position
        self.orientation = orientation
        self.ball_velocity = ball_velocity
        self.motor_input = motor_input

    def calculate_reward(self):
        reward = self.reward

        progress = (self.compute_total_path_length() - self.distance_from_goal()) / self.compute_total_path_length()
        reward += progress * 20  # Reward for progress towards goal

        if self.ball_position is None:
            reward -= 100  # Penalty for losing ball

        if self.current_waypoint != self.prev_waypoint:
            if self.current_waypoint > self.prev_waypoint:
                if abs(self.current_waypoint - self.prev_waypoint) > 1:
                    reward -= 3  # Penalty for skipping waypoints
                if self.current_waypoint == len(self.path) - 1:
                    reward += 100
                    return reward, True  # Reached goal
                else:
                    reward += 5  # Bonus for moving to a new waypoint
            elif self.current_waypoint < self.prev_waypoint:
                reward -= 10  # Penalty for moving back to a previous waypoint
            self.prev_waypoint = self.current_waypoint

        return reward, False
    
    def set_waypoint(self, waypoint):
        self.current_waypoint = self.path.index(waypoint) if waypoint in self.path else None

    
    def distance_from_goal(self):
        if not self.path or len(self.path) < 2:
            return 0.0

        ball_pos = np.array(self.ball_position)

        # Find the closest segment start index (project ball_pos onto each path segment)
        closest_index = None
        min_dist = float('inf')

        for i in range(len(self.path) - 1):
            p1 = np.array(self.path[i])
            p2 = np.array(self.path[i + 1])
            proj = self.project_point_on_segment(ball_pos, p1, p2)
            dist = np.linalg.norm(proj - ball_pos)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
                closest_proj = proj

        # Distance from ball_pos to projected point on nearest segment
        total_dist = np.linalg.norm(ball_pos - closest_proj)

        # Add remaining segment distances from projection to end of path
        total_dist += np.linalg.norm(closest_proj - np.array(self.path[closest_index + 1]))

        for j in range(closest_index + 1, len(self.path) - 1):
            total_dist += np.linalg.norm(np.array(self.path[j + 1]) - np.array(self.path[j]))

        return total_dist


    def project_point_on_segment(self, p, a, b):
        """
        Projects point p onto line segment a-b and returns the projected point.
        """
        if np.ndim(p) == 0 or np.ndim(a) == 0 or np.ndim(b) == 0:
            print("[Error] One of the points is a 0-d array.")
            return np.array([10000, 10000])
        elif (
            any(x is None for x in p) or
            any(x is None for x in a) or
            any(x is None for x in b)
        ):
            print("[project_point_on_segment] Invalid input detected: p={}, a={}, b={}".format(p, a, b))
            return np.array([10000, 10000])
        ap = p - a
        ab = b - a
        ab_norm_sq = np.dot(ab, ab)
        if ab_norm_sq == 0:
            return a  # segment is a point
        t = np.clip(np.dot(ap, ab) / ab_norm_sq, 0, 1)
        return a + t * ab
    
    def compute_total_path_length(self):
        path = self.path
        if not path or len(path) < 2:
            return 0.0

        points = np.array(path)
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)