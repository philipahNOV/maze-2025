# rl_logger.py
import json
import os
import threading
import time
import numpy as np
from camera.tracker_service import TrackerService

class OfflineLogger:
    def __init__(self, save_path="rl_data", episode_limit=100):
        self.episode_limit = episode_limit
        self.current_episode = []
        self.episodes = []
        self.save_path = save_path
        os.makedirs(save_path, exist_ok=True)

        existing_files = [f for f in os.listdir(save_path) if f.startswith("episode_") and f.endswith(".json")]
        existing_indices = [
            int(f.split("_")[1].split(".")[0])
            for f in existing_files if f.split("_")[1].split(".")[0].isdigit()
        ]
        self.episode_counter = max(existing_indices, default=-1) + 1

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
            self.save_episode()
            self.episode_counter += 1
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
        self.current_waypoint = 0
        self.prev_waypoint = 0
        self.visited_waypoints = []
        self.episode_reward = 0

        self.prev_state = None
        self.prev_action = None
        self.prev_reward = 0

        self.warmup_steps = 100  # ~5 seconds at 20Hz
        self.steps_taken = 0
        self.max_episode_steps = 6000  # 5 minutes at 20Hz
        self.episode_start_time = time.time()

    def run(self):
        LOOP_DT = 1.0 / self.target_hz
        while self.logger.episode_counter < self.logger.episode_limit and not self._stop_event.is_set():
            loop_start = time.time()
            self.steps_taken += 1

            x, y = self.ball_position if self.ball_position is not None else (0, 0)
            theta_x, theta_y = self.orientation if self.orientation is not None else (0, 0)
            vel_x, vel_y = self.ball_velocity if self.ball_velocity is not None else (0, 0)
            input_x, input_y = self.motor_input if self.motor_input is not None else (0, 0)

            state = [x, y, vel_x, vel_y, theta_x, theta_y]
            action = [input_x, input_y]

            # Combined warm-up check
            if self.steps_taken < self.warmup_steps or self.current_waypoint is None:
                reward, done = 0.0, False
                if self.steps_taken % 100 == 0:  # Debug every 5 seconds
                    print(f"[Logger] Warmup: steps={self.steps_taken}/{self.warmup_steps}, waypoint={self.current_waypoint}")
            else:
                reward, done = self.calculate_reward()
                if self.steps_taken % 100 == 0:  # Debug every 5 seconds
                    print(f"[Logger] Active: waypoint={self.current_waypoint}/{len(self.path)-1}, reward={reward:.2f}, done={done}")
            
            # Episode timeout check
            if self.steps_taken >= self.max_episode_steps:
                print(f"[Logger] Episode timeout after {self.steps_taken} steps")
                done = True
                reward -= 50  # Penalty for timeout

            if self.prev_state is not None and self.prev_action is not None and state is not None:
                self.episode_reward += reward
                self.logger.log_step(
                    state=self.prev_state,
                    action=self.prev_action,
                    reward=self.prev_reward,
                    next_state=state,
                    done=done
                )
                if done:
                    episode_duration = time.time() - self.episode_start_time
                    print(f"[LoggingThread] Episode complete. Total reward: {self.episode_reward:.1f}, Duration: {episode_duration:.1f}s, Steps: {self.steps_taken}")
                    self.stop()

            self.prev_state = state
            self.prev_action = action
            self.prev_reward = reward

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
        
        # Calculate velocity if not provided but we have previous position
        if ball_velocity is None and hasattr(self, 'prev_ball_position') and self.prev_ball_position is not None and ball_position is not None:
            if hasattr(self, 'prev_update_time'):
                dt = time.time() - self.prev_update_time
                if dt > 0:
                    vel_x = (ball_position[0] - self.prev_ball_position[0]) / dt
                    vel_y = (ball_position[1] - self.prev_ball_position[1]) / dt
                    self.ball_velocity = (vel_x, vel_y)
        
        self.prev_ball_position = ball_position
        self.prev_update_time = time.time()

    def calculate_reward(self):
        reward = 0
        
        # Terminate episode if ball is lost (fell in hole)
        if self.ball_position is None:
            print(f"[Logger] Ball lost! Terminating episode.")
            return -100, True
        
        # 1. Dense reward: Progress along the path
        path_length = self.compute_total_path_length()
        if path_length > 0:
            distance_to_goal = self.distance_from_goal()
            progress = (path_length - distance_to_goal) / path_length
            progress = np.clip(progress, 0, 1)
            reward += progress * 10  # Reduced base progress reward
        
        # 2. Waypoint progression rewards
        if self.current_waypoint != self.prev_waypoint:
            if self.current_waypoint > self.prev_waypoint:
                # Forward progress
                waypoints_advanced = self.current_waypoint - self.prev_waypoint
                if waypoints_advanced == 1:
                    # Normal progression
                    reward += 20
                elif waypoints_advanced > 1:
                    # Skipped waypoints - less reward per waypoint
                    reward += 15 * waypoints_advanced
                    reward -= 5  # Small penalty for skipping
                
                # Check if reached final waypoint
                if self.current_waypoint == len(self.path) - 1:
                    print(f"[Logger] Goal reached! Waypoint {self.current_waypoint}")
                    reward += 200  # Large completion bonus
                    return reward, True
                    
            elif self.current_waypoint < self.prev_waypoint:
                # Backward movement - penalty
                waypoints_lost = self.prev_waypoint - self.current_waypoint
                reward -= 15 * waypoints_lost
                
            self.prev_waypoint = self.current_waypoint
        
        # 3. Distance-based reward (encourage staying close to path)
        if hasattr(self, 'prev_distance_to_path'):
            current_distance = self.distance_to_nearest_path_point()
            distance_improvement = self.prev_distance_to_path - current_distance
            reward += distance_improvement * 0.1  # Small reward for getting closer to path
            self.prev_distance_to_path = current_distance
        else:
            self.prev_distance_to_path = self.distance_to_nearest_path_point()
        
        # 4. Time penalty (encourage efficiency)
        reward -= 0.1  # Small constant penalty to encourage faster completion
        
        # 5. Velocity reward (encourage movement when far from target)
        if hasattr(self, 'ball_velocity') and self.ball_velocity is not None:
            velocity_magnitude = np.linalg.norm(self.ball_velocity)
            # Reward movement when far from current target waypoint
            if self.current_waypoint < len(self.path):
                target_pos = np.array(self.path[self.current_waypoint])
                distance_to_target = np.linalg.norm(np.array(self.ball_position) - target_pos)
                if distance_to_target > 50:  # If far from target
                    reward += min(velocity_magnitude * 0.01, 2.0)  # Cap velocity reward
        
        return reward, False

    def set_waypoint(self, waypoint):
        # waypoint is already an index from pathFollower.get_current_waypoint()
        if isinstance(waypoint, int) and 0 <= waypoint < len(self.path):
            if waypoint != self.current_waypoint:
                print(f"[Logger] Waypoint updated: {self.current_waypoint} -> {waypoint}")
            self.current_waypoint = waypoint
        else:
            # If it's a coordinate tuple, find its index
            old_waypoint = self.current_waypoint
            self.current_waypoint = self.path.index(waypoint) if waypoint in self.path else None
            if self.current_waypoint != old_waypoint:
                print(f"[Logger] Waypoint updated (coord): {old_waypoint} -> {self.current_waypoint}")

    def distance_to_nearest_path_point(self):
        """Calculate distance from ball to nearest point on the path"""
        if not self.path or self.ball_position is None:
            return float('inf')
        
        ball_pos = np.array(self.ball_position)
        min_distance = float('inf')
        
        # Check distance to all path points
        for point in self.path:
            distance = np.linalg.norm(ball_pos - np.array(point))
            min_distance = min(min_distance, distance)
        
        # Also check distance to line segments between points
        for i in range(len(self.path) - 1):
            p1 = np.array(self.path[i])
            p2 = np.array(self.path[i + 1])
            proj = self.project_point_on_segment(ball_pos, p1, p2)
            distance = np.linalg.norm(ball_pos - proj)
            min_distance = min(min_distance, distance)
        
        return min_distance

    def distance_from_goal(self):
        if not self.path or len(self.path) < 2 or self.ball_position is None:
            return float('inf')

        ball_pos = np.array(self.ball_position)
        closest_index = None
        min_dist = float('inf')
        closest_proj = None

        for i in range(len(self.path) - 1):
            p1 = np.array(self.path[i])
            p2 = np.array(self.path[i + 1])
            proj = self.project_point_on_segment(ball_pos, p1, p2)
            dist = np.linalg.norm(proj - ball_pos)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
                closest_proj = proj

        if closest_proj is None or closest_index is None:
            return float('inf')

        total_dist = np.linalg.norm(ball_pos - closest_proj)
        total_dist += np.linalg.norm(closest_proj - np.array(self.path[closest_index + 1]))

        for j in range(closest_index + 1, len(self.path) - 1):
            total_dist += np.linalg.norm(np.array(self.path[j + 1]) - np.array(self.path[j]))

        return total_dist

    def project_point_on_segment(self, p, a, b):
        if p is None or a is None or b is None:
            print(f"[project_point_on_segment] Invalid input: p={p}, a={a}, b={b}")
            return np.array([10000.0, 10000.0])

        p = np.array(p, dtype=float)
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)

        if p.shape != (2,) or a.shape != (2,) or b.shape != (2,):
            print(f"[project_point_on_segment] Shape error: p={p.shape}, a={a.shape}, b={b.shape}")
            return np.array([10000.0, 10000.0])

        ab = b - a
        ap = p - a
        ab_norm_sq = np.dot(ab, ab)
        if ab_norm_sq == 0:
            return a
        t = np.clip(np.dot(ap, ab) / ab_norm_sq, 0.0, 1.0)
        return a + t * ab

    def compute_total_path_length(self):
        if not self.path or len(self.path) < 2:
            return 0.0

        points = np.array(self.path)
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)