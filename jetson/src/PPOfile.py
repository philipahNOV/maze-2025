import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import gym
import positionController

class PathFollowerEnv(gym.Env):
    def __init__(self, path_array, camera_offset_x=420, camera_offset_y=10, acceptance_radius=30):
        super(PathFollowerEnv, self).__init__()
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y
        self.acceptance_radius = acceptance_radius

        self.path = [
            (pt[0] + self.camera_offset_x, pt[1] + self.camera_offset_y)
            for pt in path_array
        ]
        self.length = len(self.path)
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=np.array([0, 0, 0, 0], dtype=np.float32),
            high=np.array([1000, 1000, 1000, 1000], dtype=np.float32),
            dtype=np.float32
        )
        self.reset()

    def reset(self):
        self.ballPos = np.array(self.path[0], dtype=np.float32)
        self.next_waypoint = 0
        return self._get_obs()

    def _get_obs(self):
        return np.array(list(self.ballPos) + list(self.path[self.next_waypoint]), dtype=np.float32)

    def step(self, action):
        self.ballPos += action

        done = False
        reward = -np.linalg.norm(self.ballPos - np.array(self.path[self.next_waypoint]))
        if np.linalg.norm(self.ballPos - np.array(self.path[self.next_waypoint])) < self.acceptance_radius:
            self.next_waypoint += 1
            reward += 100
            if self.next_waypoint >= self.length:
                done = True
                reward += 1000

        return self._get_obs(), reward, done, {}

class PathFollower:
    def __init__(self, path_array, controller: positionController.Controller, agent_path="ppo_agent.zip", train_steps=10000):
        self.path = []
        self.controller = controller
        self.length = 0

        self.camera_offset_x = 420
        self.camera_offset_y = 10

        self.prev_waypoint = None
        self.next_waypoint = 0

        if path_array is not None and len(path_array) > 0:
            self.path = [
                (pt[0] + self.camera_offset_x, pt[1] + self.camera_offset_y)
                for pt in path_array
            ]
            self.length = len(self.path)

        self.acceptance_radius = 30

        env = DummyVecEnv([lambda: PathFollowerEnv(path_array)])
        self.agent = PPO("MlpPolicy", env, verbose=1)
        self.agent.learn(total_timesteps=train_steps)
        self.agent.save(agent_path)

    def follow_path(self, ballPos):
        if self.next_waypoint >= self.length:
            print("Path complete.")
            return None 
        self.controller.posControl(self.path[self.next_waypoint])
        if self.next_waypoint < self.length:
            state = np.array(list(ballPos) + list(self.path[self.next_waypoint]), dtype=np.float32)
            action, _ = self.agent.predict(state, deterministic=True)
        else:
            return None

        if self.next_waypoint < self.length and np.linalg.norm(np.array(ballPos) - np.array(self.path[self.next_waypoint])) < self.acceptance_radius:
            self.prev_waypoint = self.next_waypoint
            self.next_waypoint += 1

        return action