from stable_baselines3 import PPO
import numpy as np
from simulation import simulate_ball_rolling
import gym
from gym import spaces
from collections import deque
import wandb
import random


wandb.init(
    project="SB3",
    config={
        "architecture": "q-learning",
        "dataset": "NONE",
    }
)


N_DISCRETE_ACTIONS = 5
OBSERVATIONS = 5
LOG_STEP = 200


class ballRollingEnv(gym.Env):
    def __init__(self):
        super(ballRollingEnv, self).__init__()
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        self.observation_space = spaces.Box(low=-10, high=10,shape=(3*OBSERVATIONS,))
    
    def step(self, action):
        if action == 0:
            action = -0.02
        elif action == 1:
            action = -0.01
        elif action == 2:
            action = 0.0
        elif action == 3:
            action = 0.01
        elif action == 4:
            action = 0.02
        
        self.angle += action
        self.angle = max(-2, min(self.angle, 2))
        self.velocity, pos_change = simulate_ball_rolling(self.angle, self.velocity)
        self.ball_position += pos_change
        self.distance_to_goal = self.goal_position - self.ball_position
        observation = [self.angle, self.velocity, self.distance_to_goal]
        for i in range(1, OBSERVATIONS):
            angle = observation[-3]
            velocity = observation[-2]
            distance_to_goal = observation[-1]
            new_vel , new_pos_change = simulate_ball_rolling(angle, velocity)
            observation.append(angle)
            observation.append(new_vel)
            distance_to_goal += new_pos_change
            observation.append(distance_to_goal)

        self.future_actions = observation

        
        
        # Penalize for moving away from the goal
        if abs(self.future_actions[2]) > abs(self.future_actions[-1]): 
            self.reward = -2
        else:
            self.reward = 0

        if abs(self.velocity) > 0.3: # Penalize for high velocity
            self.reward += -10
        elif abs(self.velocity) > 0.2:
            self.reward += -5
        elif abs(self.velocity) > 0.1:
            self.reward += -3

        # Penalize for moving away from the goal
        if self.velocity > 0 and self.distance_to_goal < 0:
            if abs(self.distance_to_goal) <= 0.1:
                self.reward += -3  # Low penalty
            elif abs(self.distance_to_goal) <= 0.2:
                self.reward += -5  # Medium penalty
            elif abs(self.distance_to_goal) <= 0.3:
                self.reward += -10  # High penalty
        elif self.velocity < 0 and self.distance_to_goal > 0:
            if abs(self.distance_to_goal) <= 0.1:
                self.reward += -3  # Low penalty
            elif abs(self.distance_to_goal) <= 0.2:
                self.reward += -5  # Medium penalty
            elif abs(self.distance_to_goal) <= 0.3:
                self.reward += -10  # High penalty
    

        # Penalize for going out of bounds
        if out_of_bounds(self.ball_position):
            self.done = True
            self.reward = -100
        #print(f'Reward: {self.reward}')

        if -0.01 < self.distance_to_goal < 0.01:
            self.reward = 0
            self.done = True
        
        return observation, self.reward, self.done, {'goal_position': self.goal_position}
        
    def reset(self):
        self.ball_position = 0.0
        self.goal_position = random.choice([random.uniform(-0.5, -0.1), random.uniform(0.1, 0.5)])
        self.angle = 0.0
        self.velocity = 0.0
        self.reward = 0
        self.prev_reward = 0

        self.done = False

        self.distance_to_goal = self.goal_position - self.ball_position
        observation = np.zeros(3*OBSERVATIONS)
        observation[0] = self.angle
        observation[1] = self.velocity
        observation[2] = self.distance_to_goal

        return np.array([observation])


def out_of_bounds(position):
    if abs(position) > 5:
        return True
    else:
        return False        

env = ballRollingEnv()


model = PPO('MlpPolicy', env, verbose=1) # type: ignore
model_path = "AI/models/model.zip"
model.save(model_path)
print(f"Model saved to {model_path}")



iterations = 20

episodes = 5
for iteration in range(iterations):
    model.learn(total_timesteps=10000)
    for episode in range(episodes):
        obs = env.reset()
        done = False
        score = 0

        step = 0
        while not done:
            action, _ = model.predict(obs) # type: ignore
            obs, reward, done, info = env.step(action)
            if step % LOG_STEP == 0:
                wandb.log({"reward": reward, "distance_to_goal": obs[2], "velocity": obs[1], "step": step, "episode": episode, "angle": obs[0], "goal_position": info['goal_position']})
            score += reward



model_path = "AI/models/model.zip"
model.save(model_path)
print(f"Model saved to {model_path}")