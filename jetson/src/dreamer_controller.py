import numpy as np
import time
import threading
import torch
import sys
import os

# Add dreamer folder to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'dreamer'))

from dreamer.dreamer import Dreamer
from dreamer.utilities import EpisodeBuffer, saveLossesToCSV
import utils.utility_threads as utility_threads

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DreamerMazeController:
    def __init__(self, arduino_connection, tracker_service, goal_position=(763, 49), config=None):
        self.arduino_connection = arduino_connection
        self.tracker_service = tracker_service
        self.goal_position = goal_position  # Goal coordinates from config
        self.config = config or {}
        
        # Initialize Dreamer agent
        self.dreamer = Dreamer()
        
        # Episode management
        self.episode_buffer = EpisodeBuffer(size=50)
        self.current_episode_obs = []
        self.current_episode_actions = []
        self.current_episode_rewards = []
        
        # Control parameters
        self.control_frequency = 20  # Hz
        self.max_episode_steps = 1000
        self.episode_count = 0
        self.total_steps = 0
        self.running = False
        
        # Ball tracking and BlinkRed management (similar to PID controller)
        self.prev_ball_pos = None
        self.blinker = None
        self.ball_not_found_timer = None
        self.ball_not_found_limit = 60  # seconds
        self.ball_lost_timeout = self.config.get("game", {}).get("ball_lost_timeout", 5)
        self.elevator_state = "down"  # Track elevator state like PID controller
        
        # Training parameters
        self.train_every_episodes = 5
        self.save_every_episodes = 10
        
        # Load existing model if available
        self.checkpoint_path = "models/dreamer_maze_checkpoint.pth"
        os.makedirs("models", exist_ok=True)
        if os.path.exists(self.checkpoint_path):
            try:
                self.dreamer.loadCheckpoint(self.checkpoint_path)
                print(f"Loaded Dreamer checkpoint from {self.checkpoint_path}")
            except Exception as e:
                print(f"Failed to load checkpoint: {e}")
        
        print(f"DreamerMazeController initialized with goal at {goal_position}")
    
    def horizontal(self):
        """Level the platform horizontally (similar to PID controller)"""
        print("Dreamer: Stabilizing horizontally...")
        kp = self.config.get("controller", {}).get("horizontal_controller", {}).get("kp", 700)
        tol = self.config.get("controller", {}).get("horizontal_controller", {}).get("tolerance", 0.0015)
        timeLimit = self.config.get("controller", {}).get("horizontal_controller", {}).get("time_limit", 300)
        deadline = time.time() + timeLimit
        
        x_offset = self.config.get("controller", {}).get("arduino", {}).get("x_offset", 0.002)
        y_offset = self.config.get("controller", {}).get("arduino", {}).get("y_offset", 0.001)
        min_velocity = self.config.get("controller", {}).get("arduino", {}).get("minimum_velocity", 22)
        command_delay = self.config.get("controller", {}).get("angular_controller", {}).get("command_delay", 0.0001)
        
        self.arduino_connection.send_speed(0, 0)

        while time.time() < deadline:
            orientation = self.tracker_service.get_orientation()
            if not orientation:
                continue

            theta_x = orientation[1] + x_offset
            theta_y = orientation[0] + y_offset

            if abs(theta_x) < tol and abs(theta_y) < tol:
                print("Dreamer: Orientation is within tolerance, stopping motors.")
                self.arduino_connection.send_speed(0, 0)
                return

            vel_x = 0 if abs(theta_x) < tol else -np.sign(theta_x) * min(max(int(kp * abs(theta_x)), min_velocity), 255)
            vel_y = 0 if abs(theta_y) < tol else -np.sign(theta_y) * min(max(int(kp * abs(theta_y)), min_velocity), 255)
            
            self.arduino_connection.send_speed(vel_x, vel_y)
            time.sleep(command_delay)

        print("Dreamer: Deadline reached, stopping motors.")
        self.arduino_connection.send_speed(0, 0)

    def get_observation(self):
        """Get current maze state as observation vector"""
        ball_pos = self.tracker_service.get_ball_position()
        
        if ball_pos is not None:
            ball_x = ball_pos[0]
            ball_y = ball_pos[1]
            # Calculate velocity if we have previous position
            if hasattr(self, 'prev_ball_pos') and self.prev_ball_pos is not None:
                dt = 1.0 / self.control_frequency  # approximate dt
                ball_vx = (ball_x - self.prev_ball_pos[0]) / dt
                ball_vy = (ball_y - self.prev_ball_pos[1]) / dt
            else:
                ball_vx = ball_vy = 0.0
            self.prev_ball_pos = ball_pos
        else:
            # Ball lost - return None to indicate no valid observation
            return None
        
        # Get platform angles if available
        orientation = self.tracker_service.get_orientation()
        if orientation is not None:
            platform_x = orientation[1]  # x-axis tilt
            platform_y = orientation[0]  # y-axis tilt
        else:
            platform_x = platform_y = 0.0
        
        # Normalize positions to [-1, 1] range for neural network
        # Maze is 655x655 pixels according to config
        ball_x_norm = (ball_x - 327.5) / 327.5  # center and normalize
        ball_y_norm = (ball_y - 327.5) / 327.5
        
        # Scale velocities for better NN input
        ball_vx_norm = np.clip(ball_vx / 100.0, -1, 1)
        ball_vy_norm = np.clip(ball_vy / 100.0, -1, 1)
        
        # Platform angles are already in radians, normalize to [-1, 1]
        platform_x_norm = np.clip(platform_x / 0.1, -1, 1)  # assuming max 0.1 rad
        platform_y_norm = np.clip(platform_y / 0.1, -1, 1)
        
        observation = np.array([
            ball_x_norm,         # ball x position [-1, 1]
            ball_y_norm,         # ball y position [-1, 1] 
            ball_vx_norm,        # ball x velocity (scaled)
            ball_vy_norm,        # ball y velocity (scaled)
            platform_x_norm,     # platform x angle [-1, 1]
            platform_y_norm      # platform y angle [-1, 1]
        ], dtype=np.float32)
        
        return torch.FloatTensor(observation).unsqueeze(0).to(device)

    def get_start_position(self):
        """Get current ball position as start position for the episode"""
        ball_pos = self.tracker_service.get_ball_position()
        if ball_pos is not None:
            return ball_pos
        else:
            # If no ball detected, wait and try again (like PID controller)
            print("Dreamer: Waiting for ball detection...")
            return None
    
    def calculate_reward(self, obs_tensor):
        """Calculate reward based on current state and goal proximity"""
        ball_pos = self.tracker_service.get_ball_position()
        
        # Check if ball is lost
        if ball_pos is None or obs_tensor is None:
            return -50.0  # Large negative reward for losing ball
        
        # Calculate distance to goal
        goal_distance = np.sqrt((ball_pos[0] - self.goal_position[0])**2 + 
                               (ball_pos[1] - self.goal_position[1])**2)
        
        # Goal proximity reward (higher reward for being closer to goal)
        max_distance = 600  # approximate max distance across maze
        goal_reward = 10.0 * (1.0 - goal_distance / max_distance)
        
        # Bonus for reaching goal (within 30 pixel radius)
        if goal_distance < 30:
            goal_reward += 50.0  # Large bonus for reaching goal
        
        # Velocity penalty (encourage controlled movement)
        obs = obs_tensor.squeeze(0).cpu().numpy()
        ball_vx, ball_vy = obs[2], obs[3]
        velocity_magnitude = np.sqrt(ball_vx**2 + ball_vy**2)
        velocity_penalty = -0.5 * velocity_magnitude
        
        # Boundary penalty (maze boundaries)
        ball_x_norm, ball_y_norm = obs[0], obs[1]
        boundary_penalty = 0.0
        if abs(ball_x_norm) > 0.9 or abs(ball_y_norm) > 0.9:
            boundary_penalty = -10.0
        
        total_reward = goal_reward + velocity_penalty + boundary_penalty
        return total_reward
    
    def send_action(self, action_tensor):
        """Send action to Arduino motors"""
        action = action_tensor.squeeze(0).cpu().numpy()
        
        # Actions are already in motor command range from actor network
        x_command = np.clip(action[0], -200, 200)
        y_command = np.clip(action[1], -200, 200)
        
        try:
            self.arduino_connection.send_speed(x_command, y_command)
        except Exception as e:
            print(f"Error sending motor commands: {e}")
    
    def reset_episode(self):
        """Reset for new episode"""
        # Store completed episode if it has data
        if len(self.current_episode_obs) > 10:  # Only store substantial episodes
            obs_tensor = torch.stack(self.current_episode_obs)
            actions_tensor = torch.stack(self.current_episode_actions)
            rewards_tensor = torch.FloatTensor(self.current_episode_rewards).to(device)
            
            self.episode_buffer.addEpisode(obs_tensor, actions_tensor, rewards_tensor)
        
        # Clear current episode data
        self.current_episode_obs = []
        self.current_episode_actions = []
        self.current_episode_rewards = []
        
        # Level platform
        self.arduino_connection.send_speed(0, 0)
        time.sleep(1.0)
        
        self.episode_count += 1
        print(f"Starting episode {self.episode_count}")
    
    def run_episode(self):
        """Run a single episode with BlinkRed functionality like PID controller"""
        self.reset_episode()
        
        # Wait for ball detection at start of episode
        print(f"Episode {self.episode_count}: Waiting for ball detection...")
        ball_detected = False
        detection_timeout = 30  # seconds
        detection_start_time = time.time()
        
        while not ball_detected and self.running:
            start_pos = self.get_start_position()
            if start_pos is not None:
                ball_detected = True
                print(f"Episode {self.episode_count}: Ball detected at {start_pos}, starting episode")
            else:
                time.sleep(0.1)
                if time.time() - detection_start_time > detection_timeout:
                    print(f"Episode {self.episode_count}: Ball detection timeout, skipping episode")
                    return
        
        if not ball_detected:
            return
        
        # Level platform initially like PID controller
        self.horizontal()
        
        # Run the episode control loop
        episode_reward = 0
        episode_steps = 0
        reset = True
        last_valid_pos_time = time.time()
        
        while self.running and episode_steps < self.max_episode_steps:
            # Get current observation
            obs = self.get_observation()
            
            if obs is not None:
                # Ball is detected - normal operation
                last_valid_pos_time = time.time()
                
                # Stop BlinkRed if it was active
                if self.blinker is not None:
                    self.blinker.stop()
                    # Check if elevator was triggered and handle escape
                    if self.blinker.triggered:
                        print("Dreamer: Elevator was triggered, running escape sequence...")
                        escape_thread = utility_threads.EscapeElevatorThread(self.arduino_connection, self)
                        escape_thread.start()
                        time.sleep(escape_thread.duration)
                        self.horizontal()
                    self.blinker = None
                    self.ball_not_found_timer = None
                
                # Get action from Dreamer
                with torch.no_grad():
                    action = self.dreamer.act(obs, reset=reset)
                reset = False
                
                # Send action to motors
                self.send_action(action)
                
                # Wait for control frequency
                time.sleep(1.0 / self.control_frequency)
                
                # Get next observation and reward
                next_obs = self.get_observation()
                reward = self.calculate_reward(next_obs)
                
                # Store transition only if we have valid observations
                if next_obs is not None:
                    self.current_episode_obs.append(obs.squeeze(0))
                    self.current_episode_actions.append(action.squeeze(0))
                    self.current_episode_rewards.append(reward)
                    
                    episode_reward += reward
                    episode_steps += 1
                    self.total_steps += 1
                
                # Check goal reached
                ball_pos = self.tracker_service.get_ball_position()
                if ball_pos is not None:
                    goal_distance = np.sqrt((ball_pos[0] - self.goal_position[0])**2 + 
                                           (ball_pos[1] - self.goal_position[1])**2)
                    if goal_distance < 30:  # Goal reached
                        print(f"Episode {self.episode_count} SUCCESS - Goal reached!")
                        break
                
                # Check if out of bounds
                if next_obs is not None:
                    obs_np = next_obs.squeeze(0).cpu().numpy()
                    if abs(obs_np[0]) > 0.95 or abs(obs_np[1]) > 0.95:
                        print(f"Episode {self.episode_count} ended - Ball out of bounds")
                        break
                        
            else:
                # Ball lost - start BlinkRed sequence like PID controller
                if self.blinker is None:
                    print("Dreamer: Ball lost, starting BlinkRed sequence...")
                    self.arduino_connection.send_speed(0, 0)
                    self.blinker = utility_threads.BlinkRed(self.arduino_connection, self.config, self)
                    self.blinker.start()
                    self.ball_not_found_timer = time.time()
                
                # Check ball lost timeout
                elapsed_time = time.time() - last_valid_pos_time
                if elapsed_time > self.ball_lost_timeout:
                    print(f"Episode {self.episode_count} failed - Ball lost for {elapsed_time:.1f}s > {self.ball_lost_timeout}s")
                    break
                
                # Check overall timeout
                if self.ball_not_found_timer is not None:
                    total_elapsed = time.time() - self.ball_not_found_timer
                    if total_elapsed > self.ball_not_found_limit:
                        print(f"Episode {self.episode_count} timeout - Ball not found for {total_elapsed:.1f}s")
                        break
                
                time.sleep(0.1)  # Short sleep when ball is lost
        
        # Clean up BlinkRed if still running
        if self.blinker is not None:
            self.blinker.stop()
            self.blinker = None
        
        # Stop motors
        self.arduino_connection.send_speed(0, 0)
        
        print(f"Episode {self.episode_count} completed - Steps: {episode_steps}, Reward: {episode_reward:.2f}")
        
        # Train periodically
        if self.episode_count % self.train_every_episodes == 0 and len(self.episode_buffer) >= 5:
            self.train_dreamer()
        
        # Save periodically
        if self.episode_count % self.save_every_episodes == 0:
            self.save_checkpoint()
        
        # Brief pause between episodes
        time.sleep(2.0)
    
    def train_dreamer(self):
        """Train the Dreamer world model and actor-critic"""
        print("Training Dreamer...")
        
        try:
            # Sample episodes for training
            batch_size = min(4, len(self.episode_buffer))
            obs_batch, actions_batch, rewards_batch = self.episode_buffer.sampleEpisodes(batch_size)
            
            # Train world model
            sampled_states, wm_metrics = self.dreamer.trainWorldModel(obs_batch, actions_batch, rewards_batch)
            
            # Train actor-critic
            ac_metrics = self.dreamer.trainActorCritic(sampled_states)
            
            # Combine metrics
            metrics = {**wm_metrics, **ac_metrics}
            metrics['episode'] = self.episode_count
            metrics['total_steps'] = self.total_steps
            
            # Save metrics
            saveLossesToCSV("training_metrics", metrics)
            
            print(f"Training completed - WM Loss: {wm_metrics.get('worldModelLoss', 0):.4f}, "
                  f"Actor Loss: {ac_metrics.get('actorLoss', 0):.4f}, "
                  f"Critic Loss: {ac_metrics.get('criticLoss', 0):.4f}")
                  
        except Exception as e:
            print(f"Training failed: {e}")
    
    def save_checkpoint(self):
        """Save model checkpoint"""
        try:
            self.dreamer.saveCheckpoint(self.checkpoint_path)
            print(f"Checkpoint saved at episode {self.episode_count}")
        except Exception as e:
            print(f"Failed to save checkpoint: {e}")
    
    def start(self):
        """Start the control loop"""
        if self.running:
            print("Controller already running")
            return
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        print("DreamerMazeController started")
    
    def stop(self):
        """Stop the control loop"""
        self.running = False
        
        # Clean up BlinkRed if running
        if self.blinker is not None:
            self.blinker.stop()
            self.blinker = None
        
        # Stop motors
        self.arduino_connection.send_speed(0, 0)
        
        if hasattr(self, 'control_thread'):
            self.control_thread.join(timeout=5.0)
        
        # Final save
        self.save_checkpoint()
        print("DreamerMazeController stopped")
    
    def _control_loop(self):
        """Main control loop"""
        try:
            while self.running:
                self.run_episode()
        except Exception as e:
            print(f"Error in control loop: {e}")
            self.running = False