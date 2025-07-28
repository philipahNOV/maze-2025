# dreamerv3_controller.py
import numpy as np
import time
import threading
from dreamerv3_agent import DreamerV3Agent


class DreamerV3Controller:
    def __init__(self, arduino_connection, tracker_service):
        self.arduino_connection = arduino_connection
        self.tracker_service = tracker_service
        
        self.agent = DreamerV3Agent(obs_dim=6, action_dim=2)
        
        # Load existing model if available
        self.agent.load_model("models/dreamerv3_checkpoint.pth")
        
        self.last_observation = None
        self.last_action = np.array([0.0, 0.0])
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.total_episodes = 0
        
        self.running = False
        self.control_thread = None
        
        # Control parameters
        self.control_frequency = 20  # Hz
        self.max_episode_steps = 1000
        self.save_interval = 10  # episodes
        
        print("DreamerV3 Controller initialized")
    
    def get_observation(self):
        """Get current maze state as observation"""
        ball_data = self.tracker_service.get_ball_data()
        
        if ball_data and len(ball_data) > 0:
            ball = ball_data[0]
            ball_x = ball.get('x', 0.5)
            ball_y = ball.get('y', 0.5)
            ball_vx = ball.get('vx', 0.0)
            ball_vy = ball.get('vy', 0.0)
        else:
            ball_x = ball_y = 0.5
            ball_vx = ball_vy = 0.0
        
        # Get platform angles (assuming they're available)
        platform_x = getattr(self.arduino_connection, 'current_x_angle', 0.0) / 180.0  # normalize to [-1, 1]
        platform_y = getattr(self.arduino_connection, 'current_y_angle', 0.0) / 180.0
        
        observation = np.array([
            ball_x * 2 - 1,      # normalize to [-1, 1]
            ball_y * 2 - 1,      # normalize to [-1, 1]
            ball_vx * 10,        # scale velocity
            ball_vy * 10,        # scale velocity
            platform_x,          # already normalized
            platform_y           # already normalized
        ], dtype=np.float32)
        
        return observation
    
    def calculate_reward(self, observation, action):
        """Calculate reward for current state"""
        ball_x, ball_y = observation[0], observation[1]
        ball_vx, ball_vy = observation[2], observation[3]
        
        # Distance to center reward
        distance_to_center = np.sqrt(ball_x**2 + ball_y**2)
        center_reward = 1.0 - distance_to_center
        
        # Velocity penalty (encourage slow, controlled movement)
        velocity_magnitude = np.sqrt(ball_vx**2 + ball_vy**2)
        velocity_penalty = -0.1 * velocity_magnitude
        
        # Action smoothness reward (discourage jerky movements)
        action_magnitude = np.sqrt(action[0]**2 + action[1]**2)
        action_penalty = -0.01 * action_magnitude
        
        # Boundary penalty
        boundary_penalty = 0.0
        if abs(ball_x) > 0.9 or abs(ball_y) > 0.9:
            boundary_penalty = -1.0
        
        total_reward = center_reward + velocity_penalty + action_penalty + boundary_penalty
        
        return total_reward
    
    def send_action(self, action):
        """Send motor commands to Arduino"""
        # Convert action to motor commands
        x_command = np.clip(action[0], -200, 200)
        y_command = np.clip(action[1], -200, 200)
        
        try:
            self.arduino_connection.send_motor_commands(x_command, y_command)
            self.last_action = action.copy()
        except Exception as e:
            print(f"Error sending motor commands: {e}")
    
    def reset_episode(self):
        """Reset environment for new episode"""
        # Level the platform
        self.arduino_connection.send_motor_commands(0, 0)
        time.sleep(1.0)
        
        # Reset episode tracking
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.agent.reset_episode()
        
        # Get initial observation
        self.last_observation = self.get_observation()
        
        print(f"Episode {self.total_episodes + 1} started")
        
        return self.last_observation
    
    def step(self):
        """Execute one control step"""
        if self.last_observation is None:
            self.last_observation = self.get_observation()
        
        # Get action from agent
        action = self.agent.select_action(self.last_observation, training=True)
        
        # Send action to motors
        self.send_action(action)
        
        # Wait for control frequency
        time.sleep(1.0 / self.control_frequency)
        
        # Get new observation
        new_observation = self.get_observation()
        
        # Calculate reward
        reward = self.calculate_reward(new_observation, action)
        
        # Check if episode is done
        ball_x, ball_y = new_observation[0], new_observation[1]
        done = (abs(ball_x) > 0.95 or abs(ball_y) > 0.95 or 
                self.episode_steps >= self.max_episode_steps)
        
        # Store transition
        self.agent.store_transition(
            self.last_observation, 
            action, 
            reward, 
            new_observation, 
            done
        )
        
        # Update tracking
        self.episode_reward += reward
        self.episode_steps += 1
        self.last_observation = new_observation
        
        return new_observation, reward, done
    
    def run_episode(self):
        """Run a complete episode"""
        self.reset_episode()
        
        while self.running:
            observation, reward, done = self.step()
            
            if done:
                self.total_episodes += 1
                print(f"Episode {self.total_episodes} finished - Steps: {self.episode_steps}, Reward: {self.episode_reward:.2f}")
                
                # Train the agent
                if self.total_episodes % 2 == 0:  # Train every 2 episodes
                    print("Training agent...")
                    training_info = self.agent.train()
                    if training_info:
                        print(f"Training step {training_info.get('training_step', 0)}, "
                              f"WM loss: {training_info.get('world_model_loss', 0):.4f}, "
                              f"Actor loss: {training_info.get('actor_loss', 0):.4f}, "
                              f"Critic loss: {training_info.get('critic_loss', 0):.4f}")
                
                # Save model periodically
                if self.total_episodes % self.save_interval == 0:
                    self.agent.save_model("models/dreamerv3_checkpoint.pth")
                    print(f"Model saved at episode {self.total_episodes}")
                
                # Small break between episodes
                time.sleep(2.0)
                break
    
    def start(self):
        """Start the control loop"""
        if self.running:
            print("Controller already running")
            return
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        print("DreamerV3 Controller started")
    
    def stop(self):
        """Stop the control loop"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=5.0)
        
        # Save final model
        self.agent.save_model("models/dreamerv3_checkpoint.pth")
        print("DreamerV3 Controller stopped")
    
    def _control_loop(self):
        """Main control loop"""
        try:
            while self.running:
                self.run_episode()
        except Exception as e:
            print(f"Error in control loop: {e}")
            self.running = False
