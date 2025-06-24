import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback
import gymnasium as gym
from gymnasium import spaces
import positionController
from typing import Tuple, Optional, Dict, Any, List


class PathFollowerEnv(gym.Env):
    """
    Gymnasium environment for path following using reinforcement learning.
    Optimized for modern Gymnasium API with proper typing and error handling.
    """
    
    metadata = {"render_modes": ["human"], "render_fps": 30}
    
    def __init__(self, 
                 path_array: List[Tuple[float, float]], 
                 camera_offset_x: float = 420, 
                 camera_offset_y: float = 10, 
                 acceptance_radius: float = 30,
                 max_steps: int = 1000):
        super().__init__()
        
        # Environment parameters
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self.acceptance_radius = acceptance_radius
        self.max_steps = max_steps
        self.current_step = 0
        
        # Process path with offset
        if not path_array:
            raise ValueError("Path array cannot be empty")
            
        self.path = np.array([
            [pt[0] + self.camera_offset_x, pt[1] + self.camera_offset_y]
            for pt in path_array
        ], dtype=np.float32)
        
        self.path_length = len(self.path)
        
        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=-10.0, high=10.0, shape=(2,), dtype=np.float32
        )
        
        # Observation: [ball_x, ball_y, target_x, target_y, distance_to_target, waypoint_progress]
        self.observation_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0, 0], dtype=np.float32),
            high=np.array([1000, 1000, 1000, 1000, 1000, 1], dtype=np.float32),
            dtype=np.float32
        )
        
        # Initialize state
        self.ball_pos = None
        self.current_waypoint_idx = 0
        self.previous_distance = None
        
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment to initial state."""
        super().reset(seed=seed)
        
        self.ball_pos = self.path[0].copy()
        self.current_waypoint_idx = 0
        self.current_step = 0
        self.previous_distance = np.linalg.norm(
            self.ball_pos - self.path[self.current_waypoint_idx]
        )
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation."""
        target_pos = self.path[self.current_waypoint_idx]
        distance_to_target = np.linalg.norm(self.ball_pos - target_pos)
        waypoint_progress = self.current_waypoint_idx / (self.path_length - 1)
        
        return np.array([
            self.ball_pos[0],
            self.ball_pos[1],
            target_pos[0],
            target_pos[1],
            distance_to_target,
            waypoint_progress
        ], dtype=np.float32)
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional info."""
        return {
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": self.path_length,
            "distance_to_target": np.linalg.norm(
                self.ball_pos - self.path[self.current_waypoint_idx]
            ),
            "path_complete": self.current_waypoint_idx >= self.path_length - 1
        }
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step in the environment."""
        self.current_step += 1
        
        # Apply action (clamp to prevent extreme movements)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.ball_pos += action
        
        # Calculate distance to current target
        target_pos = self.path[self.current_waypoint_idx]
        current_distance = np.linalg.norm(self.ball_pos - target_pos)
        
        # Calculate reward
        reward = self._calculate_reward(current_distance, action)
        
        # Check if waypoint is reached
        waypoint_reached = current_distance < self.acceptance_radius
        if waypoint_reached:
            reward += 100  # Waypoint bonus
            self.current_waypoint_idx += 1
        
        # Check termination conditions
        terminated = self.current_waypoint_idx >= self.path_length
        truncated = self.current_step >= self.max_steps
        
        if terminated:
            reward += 1000  # Path completion bonus
        
        # Update previous distance for next iteration
        if not terminated:
            self.previous_distance = current_distance
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, reward, terminated, truncated, info
    
    def _calculate_reward(self, current_distance: float, action: np.ndarray) -> float:
        """Calculate reward based on distance and movement efficiency."""
        # Distance-based reward (negative distance encourages getting closer)
        distance_reward = -current_distance * 0.1
        
        # Progress reward (encourage movement towards target)
        if self.previous_distance is not None:
            progress = self.previous_distance - current_distance
            progress_reward = progress * 10
        else:
            progress_reward = 0
        
        # Action penalty (encourage efficient movements)
        action_penalty = -np.linalg.norm(action) * 0.01
        
        # Time penalty (encourage faster completion)
        time_penalty = -0.1
        
        return distance_reward + progress_reward + action_penalty + time_penalty


class OptimizedPathFollower:
    """
    Optimized path follower using PPO with improved training and inference.
    """
    
    def __init__(self, 
                 path_array: List[Tuple[float, float]], 
                 controller: positionController.Controller,
                 agent_path: str = "ppo_agent.zip",
                 train_steps: int = 50000,
                 camera_offset_x: float = 420,
                 camera_offset_y: float = 10,
                 acceptance_radius: float = 30):
        
        self.controller = controller
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self.acceptance_radius = acceptance_radius
        self.agent_path = agent_path
        
        # Validate input
        if not path_array:
            raise ValueError("Path array cannot be empty")
        
        # Process path
        self.path = np.array([
            [pt[0] + camera_offset_x, pt[1] + camera_offset_y]
            for pt in path_array
        ], dtype=np.float32)
        
        self.path_length = len(self.path)
        self.current_waypoint_idx = 0
        self.prev_waypoint_idx = None
        
        # Create and train agent
        self._setup_and_train_agent(path_array, train_steps)
    
    def _setup_and_train_agent(self, path_array: List[Tuple[float, float]], train_steps: int):
        """Setup and train the PPO agent with improved configuration."""
        
        # Create training environment
        def make_env():
            return PathFollowerEnv(
                path_array=path_array,
                camera_offset_x=self.camera_offset_x,
                camera_offset_y=self.camera_offset_y,
                acceptance_radius=self.acceptance_radius
            )
        
        env = DummyVecEnv([make_env])
        
        # Create PPO agent with optimized hyperparameters
        self.agent = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            verbose=1,
            tensorboard_log="./ppo_pathfollower_tensorboard/",
            policy_kwargs=dict(
                net_arch=[dict(pi=[256, 256], vf=[256, 256])]
            )
        )
        
        # Setup evaluation callback for monitoring training
        eval_env = DummyVecEnv([make_env])
        eval_callback = EvalCallback(
            eval_env,
            best_model_save_path="./logs/",
            log_path="./logs/",
            eval_freq=5000,
            deterministic=True,
            render=False
        )
        
        print(f"Training PPO agent for {train_steps} timesteps...")
        self.agent.learn(
            total_timesteps=train_steps,
            callback=eval_callback,
            progress_bar=True
        )
        
        # Save the trained model
        self.agent.save(self.agent_path)
        print(f"Agent saved to {self.agent_path}")
    
    @classmethod
    def load_trained_agent(cls, 
                          path_array: List[Tuple[float, float]], 
                          controller: positionController.Controller,
                          agent_path: str = "ppo_agent.zip",
                          **kwargs) -> 'OptimizedPathFollower':
        """Load a pre-trained agent instead of training from scratch."""
        instance = cls.__new__(cls)
        instance.controller = controller
        instance.camera_offset_x = kwargs.get('camera_offset_x', 420)
        instance.camera_offset_y = kwargs.get('camera_offset_y', 10)
        instance.acceptance_radius = kwargs.get('acceptance_radius', 30)
        instance.agent_path = agent_path
        
        # Process path
        instance.path = np.array([
            [pt[0] + instance.camera_offset_x, pt[1] + instance.camera_offset_y]
            for pt in path_array
        ], dtype=np.float32)
        
        instance.path_length = len(instance.path)
        instance.current_waypoint_idx = 0
        instance.prev_waypoint_idx = None
        
        # Load trained agent
        try:
            instance.agent = PPO.load(agent_path)
            print(f"Loaded trained agent from {agent_path}")
        except FileNotFoundError:
            raise FileNotFoundError(f"No trained agent found at {agent_path}. Train first or provide correct path.")
        
        return instance
    
    def follow_path(self, ball_pos: Tuple[float, float]) -> Optional[np.ndarray]:
        """
        Follow the path using the trained agent.
        
        Args:
            ball_pos: Current position of the ball (x, y)
            
        Returns:
            Action to take or None if path is complete
        """
        if self.current_waypoint_idx >= self.path_length:
            print("Path complete!")
            return None
        
        # Convert ball position to numpy array
        ball_pos_array = np.array(ball_pos, dtype=np.float32)
        
        # Use controller for position control
        target_pos = self.path[self.current_waypoint_idx]
        self.controller.posControl(target_pos)
        
        # Create observation for the agent
        target_pos = self.path[self.current_waypoint_idx]
        distance_to_target = np.linalg.norm(ball_pos_array - target_pos)
        waypoint_progress = self.current_waypoint_idx / (self.path_length - 1)
        
        observation = np.array([
            ball_pos_array[0],
            ball_pos_array[1],
            target_pos[0],
            target_pos[1],
            distance_to_target,
            waypoint_progress
        ], dtype=np.float32)
        
        # Get action from trained agent
        action, _ = self.agent.predict(observation, deterministic=True)
        
        # Check if waypoint is reached
        if distance_to_target < self.acceptance_radius:
            print(f"Waypoint {self.current_waypoint_idx} reached!")
            self.prev_waypoint_idx = self.current_waypoint_idx
            self.current_waypoint_idx += 1
            
            if self.current_waypoint_idx >= self.path_length:
                print("All waypoints completed!")
                return None
        
        return action
    
    def reset_path_following(self):
        """Reset the path following to start from the beginning."""
        self.current_waypoint_idx = 0
        self.prev_waypoint_idx = None
        print("Path following reset to beginning.")
    
    def get_progress(self) -> Dict[str, Any]:
        """Get current progress information."""
        return {
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": self.path_length,
            "progress_percentage": (self.current_waypoint_idx / self.path_length) * 100,
            "waypoints_remaining": self.path_length - self.current_waypoint_idx
        }
