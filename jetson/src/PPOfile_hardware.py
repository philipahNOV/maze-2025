import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import gymnasium as gym
from gymnasium import spaces
import positionController
from typing import Tuple, Optional, Dict, Any, List
import time
import threading
import queue


class RealTimePathFollowerEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 30}
    
    def __init__(self, 
                 path_array: List[Tuple[float, float]], 
                 controller: positionController.Controller,
                 camera_offset_x: float = 420, 
                 camera_offset_y: float = 10, 
                 acceptance_radius: float = 30,
                 max_steps: int = 2000,
                 step_timeout: float = 0.01):
        super().__init__()
        
        self.controller = controller
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self.acceptance_radius = acceptance_radius
        self.max_steps = max_steps
        self.step_timeout = step_timeout
        self.current_step = 0

        if not path_array:
            raise ValueError("Path array cannot be empty")
            
        self.path = np.array([
            [pt[0] + camera_offset_x, pt[1] + camera_offset_y]
            for pt in path_array
        ], dtype=np.float32)
        
        self.path_length = len(self.path)
        self.action_space = spaces.Box(
            low=np.array([-50.0, -50.0], dtype=np.float32),
            high=np.array([50.0, 50.0], dtype=np.float32),
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0, -np.pi, 0], dtype=np.float32),
            high=np.array([1000, 1000, 1000, 1000, 1000, np.pi, 1], dtype=np.float32),
            dtype=np.float32
        )
        
        self.current_ball_pos = None
        self.current_waypoint_idx = 0
        self.previous_distance = None
        self.episode_start_time = None
        self.last_waypoint_time = None
        
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        self.current_waypoint_idx = 0
        self.current_step = 0
        self.episode_start_time = time.time()
        self.last_waypoint_time = time.time()
        
        self.current_ball_pos = self._get_real_ball_position()
        if self.current_ball_pos is None:
            self.current_ball_pos = self.path[0].copy()
        
        self.previous_distance = np.linalg.norm(
            self.current_ball_pos - self.path[self.current_waypoint_idx]
        )
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def _get_real_ball_position(self) -> Optional[np.ndarray]:
        """Get actual ball position from hardware tracker."""
        try:
            pos = self.controller.tracker.get_position()
            if pos is not None:
                return np.array(pos, dtype=np.float32)
            return None
        except Exception as e:
            print(f"Error getting ball position: {e}")
            return None
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation from real hardware state."""
        if self.current_ball_pos is None:
            self.current_ball_pos = self._get_real_ball_position()
            if self.current_ball_pos is None:
                return np.zeros(7, dtype=np.float32)
        
        target_pos = self.path[self.current_waypoint_idx]
        distance_to_target = np.linalg.norm(self.current_ball_pos - target_pos)
        
        diff = target_pos - self.current_ball_pos
        angle_to_target = np.arctan2(diff[1], diff[0])
        
        waypoint_progress = self.current_waypoint_idx / (self.path_length - 1)
        
        return np.array([
            self.current_ball_pos[0],
            self.current_ball_pos[1],
            target_pos[0],
            target_pos[1],
            distance_to_target,
            angle_to_target,
            waypoint_progress
        ], dtype=np.float32)
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional info."""
        return {
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": self.path_length,
            "distance_to_target": np.linalg.norm(
                self.current_ball_pos - self.path[self.current_waypoint_idx]
            ) if self.current_ball_pos is not None else float('inf'),
            "path_complete": self.current_waypoint_idx >= self.path_length,
            "episode_time": time.time() - self.episode_start_time if self.episode_start_time else 0
        }
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        self.current_step += 1
        step_start_time = time.time()
        
        new_ball_pos = self._get_real_ball_position()
        if new_ball_pos is None:
            observation = self._get_observation()
            return observation, -10.0, False, True, self._get_info()
        
        self.current_ball_pos = new_ball_pos
        target_position = self.current_ball_pos + action
        
        try:
            self.controller.posControl(tuple(target_position))
            
            time.sleep(min(self.step_timeout, 0.05))
            
        except Exception as e:
            print(f"Error controlling hardware: {e}")
            observation = self._get_observation()
            return observation, -20.0, False, True, self._get_info()
        
        time.sleep(0.02)  # Small delay for system to respond
        updated_ball_pos = self._get_real_ball_position()
        if updated_ball_pos is not None:
            self.current_ball_pos = updated_ball_pos
        
        # Calculate reward based on actual hardware response
        reward = self._calculate_reward_from_hardware(action)
        
        # Check if waypoint is reached
        current_target = self.path[self.current_waypoint_idx]
        current_distance = np.linalg.norm(self.current_ball_pos - current_target)
        waypoint_reached = current_distance < self.acceptance_radius
        
        if waypoint_reached:
            reward += 100  # Waypoint bonus
            self.current_waypoint_idx += 1
            self.last_waypoint_time = time.time()
            print(f"Waypoint {self.current_waypoint_idx - 1} reached! Distance: {current_distance:.1f}")
        
        # Check termination conditions
        terminated = self.current_waypoint_idx >= self.path_length
        truncated = (self.current_step >= self.max_steps or 
                    (time.time() - self.last_waypoint_time > 30))  # Timeout if stuck
        
        if terminated:
            reward += 500  # Path completion bonus
            print("Path completed successfully!")
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, reward, terminated, truncated, info
    
    def _calculate_reward_from_hardware(self, action: np.ndarray) -> float:
        """Calculate reward based on actual hardware performance."""
        if self.current_ball_pos is None:
            return -10.0
        
        current_target = self.path[self.current_waypoint_idx]
        current_distance = np.linalg.norm(self.current_ball_pos - current_target)
        
        # Distance-based reward
        distance_reward = -current_distance * 0.05
        
        # Progress reward
        if self.previous_distance is not None:
            progress = self.previous_distance - current_distance
            progress_reward = progress * 5  # Reward for getting closer
        else:
            progress_reward = 0
        
        # Action efficiency penalty
        action_penalty = -np.linalg.norm(action) * 0.001
        
        # Time penalty (encourage faster completion)
        time_penalty = -0.05
        
        # Stability reward (penalize erratic movement)
        stability_reward = 0
        if hasattr(self, 'previous_action'):
            action_change = np.linalg.norm(action - self.previous_action)
            stability_reward = -action_change * 0.01
        
        self.previous_action = action.copy()
        self.previous_distance = current_distance
        
        total_reward = distance_reward + progress_reward + action_penalty + time_penalty + stability_reward
        
        return total_reward


class HardwareTrainingCallback(BaseCallback):
    """Callback to monitor training progress with hardware."""
    
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []
        self.waypoints_reached = []
    
    def _on_step(self) -> bool:
        if len(self.locals.get('infos', [])) > 0:
            info = self.locals['infos'][0]
            if 'current_waypoint' in info:
                if self.verbose > 0 and self.num_timesteps % 100 == 0:
                    print(f"Step {self.num_timesteps}: Waypoint {info['current_waypoint']}/{info['total_waypoints']}")
        return True
    
    def _on_rollout_end(self) -> None:
        if self.verbose > 0:
            print(f"Rollout ended at step {self.num_timesteps}")


class RealTimePathFollower:
    def __init__(self, 
                 path_array: List[Tuple[float, float]], 
                 controller: positionController.Controller,
                 agent_path: str = "realtime_ppo_agent.zip",
                 total_training_steps: int = 10000,
                 **env_kwargs):
        
        self.controller = controller
        self.agent_path = agent_path
        self.path_array = path_array
        
        # Create real-time environment
        def make_env():
            return RealTimePathFollowerEnv(
                path_array=path_array,
                controller=controller,
                **env_kwargs
            )
        
        self.env = DummyVecEnv([make_env])
        
        self.agent = PPO(
            policy="MlpPolicy",
            env=self.env,
            learning_rate=1e-4,  # Lower learning rate for hardware
            n_steps=512,         # Smaller rollouts
            batch_size=32,
            n_epochs=5,          # Fewer epochs per update
            gamma=0.95,
            gae_lambda=0.9,
            clip_range=0.1,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            verbose=1,
            policy_kwargs=dict(
                net_arch=dict(pi=[128, 128], vf=[128, 128])
            )
        )
        
        self.callback = HardwareTrainingCallback(verbose=1)
        self._start_training(total_training_steps)
    
    def _start_training(self, total_steps: int):
        print(f"Starting real-time training with hardware for {total_steps} steps...")
        print("WARNING: This will actively control your hardware!")
        
        try:
            self.agent.learn(
                total_timesteps=total_steps,
                callback=self.callback,
                progress_bar=True
            )
            
            self.agent.save(self.agent_path)
            print(f"Training completed! Agent saved to {self.agent_path}")
            
        except KeyboardInterrupt:
            print("\nTraining interrupted by user!")
            self.agent.save(self.agent_path + "_interrupted")
            print(f"Partial training saved to {self.agent_path}_interrupted")
        except Exception as e:
            print(f"Training error: {e}")
            # Try to save partial progress
            try:
                self.agent.save(self.agent_path + "_error")
                print(f"Partial training saved to {self.agent_path}_error")
            except:
                print("Could not save partial training")
    
    def follow_path_realtime(self, max_episodes: int = 5) -> bool:
        print(f"Starting real-time path following for up to {max_episodes} episodes...")
        
        for episode in range(max_episodes):
            print(f"\nEpisode {episode + 1}/{max_episodes}")
            obs, _ = self.env.reset()
            done = False
            step_count = 0
            
            while not done:
                action, _ = self.agent.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.env.step(action)
                done = terminated[0] or truncated[0]
                step_count += 1
                
                if step_count % 50 == 0:
                    current_info = info[0] if info else {}
                    print(f"Step {step_count}: Waypoint {current_info.get('current_waypoint', 0)}/{current_info.get('total_waypoints', 0)}")
                
                # Safety check - stop if taking too long
                if step_count > 5000:
                    print("Episode taking too long, moving to next episode")
                    break
            
            final_info = info[0] if info else {}
            if final_info.get('path_complete', False):
                print(f"Path completed successfully in episode {episode + 1}!")
                return True
            else:
                print(f"Episode {episode + 1} did not complete the path")
        
        return False
    
    def get_training_stats(self) -> Dict[str, Any]:
        """Get training statistics."""
        return {
            "episodes_completed": len(self.callback.episode_rewards),
            "average_reward": np.mean(self.callback.episode_rewards) if self.callback.episode_rewards else 0,
            "average_episode_length": np.mean(self.callback.episode_lengths) if self.callback.episode_lengths else 0,
            "total_training_steps": self.agent.num_timesteps if hasattr(self.agent, 'num_timesteps') else 0
        }


# Modified simple interface for your main script
class SimpleRealTimePathFollower:
    """Simplified interface that trains in the background while providing immediate control."""
    
    def __init__(self, 
                 path_array: List[Tuple[float, float]], 
                 controller: positionController.Controller,
                 quick_training_steps: int = 2000):
        
        self.controller = controller
        self.path = np.array([
            [pt[0] + 420, pt[1] + 10]  # Apply camera offsets
            for pt in path_array
        ], dtype=np.float32)
        self.path_length = len(self.path)
        self.current_waypoint_idx = 0
        self.acceptance_radius = 30
        
        # Start with basic control, improve with training
        self.use_ai_control = False
        self.training_thread = None
        
        # Start background training
        if quick_training_steps > 0:
            self._start_background_training(path_array, quick_training_steps)
    
    def _start_background_training(self, path_array, steps):
        """Start training in background thread."""
        def train():
            try:
                print("[INFO] Starting background PPO training...")
                self.ppo_follower = RealTimePathFollower(
                    path_array=path_array,
                    controller=self.controller,
                    total_training_steps=steps,
                    acceptance_radius=self.acceptance_radius
                )
                self.use_ai_control = True
                print("[INFO] Background training completed!")
            except Exception as e:
                print(f"[ERROR] Background training failed: {e}")
                self.use_ai_control = False
        
        self.training_thread = threading.Thread(target=train, daemon=True)
        self.training_thread.start()
    
    def follow_path(self, ball_pos: Tuple[float, float]) -> Optional[np.ndarray]:
        """Follow path using hybrid control - basic first, then AI when ready."""
        if self.current_waypoint_idx >= self.path_length:
            return None
        
        ball_pos_array = np.array(ball_pos, dtype=np.float32)
        target_pos = self.path[self.current_waypoint_idx]
        
        # Check if waypoint reached
        distance = np.linalg.norm(ball_pos_array - target_pos)
        if distance < self.acceptance_radius:
            print(f"Waypoint {self.current_waypoint_idx} reached!")
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= self.path_length:
                print("Path completed!")
                return None
        
        if self.use_ai_control and hasattr(self, 'ppo_follower'):
            try:
                # This would need to be implemented in the PPO follower
                return np.array([target_pos[0] - ball_pos_array[0], 
                               target_pos[1] - ball_pos_array[1]], dtype=np.float32)
            except:
                pass
        
        # Basic control - just send target to existing controller
        self.controller.posControl(tuple(target_pos))
        return np.array([target_pos[0] - ball_pos_array[0], 
                        target_pos[1] - ball_pos_array[1]], dtype=np.float32)
    
    def get_progress(self) -> Dict[str, Any]:
        """Get current progress."""
        return {
            "current_waypoint": self.current_waypoint_idx,
            "total_waypoints": self.path_length,
            "progress_percentage": (self.current_waypoint_idx / self.path_length) * 100,
            "ai_control_active": self.use_ai_control,
            "training_active": self.training_thread.is_alive() if self.training_thread else False
        }