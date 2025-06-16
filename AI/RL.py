from stable_baselines3 import PPO
import numpy as np
from simulation import simulate_ball_rolling
import gym
from gym import spaces
from collections import deque
import wandb
import random
import os


N_DISCRETE_ACTIONS = 3
LOG_STEP = 1

class ballRollingEnv(gym.Env):
    def __init__(self):
        super(ballRollingEnv, self).__init__()
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        self.observation_space = spaces.Box(low=-10, high=10,shape=(3,))

    def step(self, action):
        if action == 0:
            action = -0.2
        
        elif action == 1:
            action = 0.0
        
        elif action == 2:
            action = 0.2
        """ elif action == 1:
            action = -0.1
        elif action == 3:
            action = 0.1
            """
        
        self.angle += action
        self.angle = max(-2, min(self.angle, 2))
        self.velocity, pos_change = simulate_ball_rolling(self.angle, self.velocity)
        self.ball_position += pos_change
        self.distance_to_goal = self.goal_position - self.ball_position
        observation = [self.angle, self.velocity, self.distance_to_goal]
        
        
        able_to_stop_ball, pos_stoped, time_over = able_to_stop(self.angle, self.velocity, self.distance_to_goal)
        scale = 0.03
        magnitude = scale / (max(self.distance_to_goal, 0.01))
        able = 1 if able_to_stop_ball else 0
        if able_to_stop_ball:
            if self.distance_to_goal > 0 and self.velocity < 0.0:
                able = 0.5
                if action > 0.0:
                    self.reward = 1
                elif action <= 0.0:
                    self.reward = -5
            elif self.distance_to_goal < 0 and self.velocity > 0.0:
                able = 0.5
                if action < 0.0:
                    self.reward = 1
                elif action >= 0.0:
                    self.reward = -5
            else:
                if self.distance_to_goal > 0:
                    if action > 0.0:
                        self.reward = 1
                    if action == 0.0:
                        self.reward = 0.5
                    elif action < 0.0:
                        self.reward = -5
                elif self.distance_to_goal < 0:
                    if action < 0.0:
                        self.reward = 1
                    if action == 0.0:
                        self.reward = 0.5
                    elif action > 0.0:
                        self.reward = -5
        else:
            if self.distance_to_goal > 0 and self.velocity > 0.0:
                if action >= 0.0:
                    self.reward = -5
                elif action < 0.0:
                    self.reward = 1
            elif self.distance_to_goal < 0 and self.velocity < 0.0:
                if action <= 0.0:
                    self.reward = - 5
                elif action > 0.0:
                    self.reward = 1
        will_exceed, pos, time_over = will_exceed_speed(self.angle, self.distance_to_goal, self.velocity, 30)
        if will_exceed:
            if self.distance_to_goal > 0 and action > 0:
                self.reward = - 10  # Penalize for exceeding the speed limit
            elif self.distance_to_goal < 0 and action < 0:
                self.reward = - 10
                    
            
        
        if abs(self.distance_to_goal) < 0.001*1000 and abs(self.velocity) < 0.001*1000:
            if self.last_able == 0.5 and able == 1 or able == 0:
                self.reward = 10
                self.goal_position = random.uniform(-0.05*1000, 0.05*1000)
        self.last_able = able

        if out_of_bounds(self.ball_position):
            self.done = True
        
                
        return observation, self.reward, self.done, {'goal_position': self.goal_position, 'ball_position': self.ball_position, 'angle': self.angle, 'velocity': self.velocity, 'distance_to_goal': self.distance_to_goal, 'able_to_stop_ball': able}
        

    def reset(self):
        self.ball_position = 0.0
        self.goal_position = random.uniform(-0.05*1000, 0.05*1000)
        self.angle = 0.0
        self.velocity = random.uniform(-0.01*1000, 0.01*1000)
        self.reward = 0
        self.prev_reward = 0
        self.last_able = -1

        self.done = False

        self.distance_to_goal = self.goal_position - self.ball_position
        observation = np.zeros(3*1)
        observation[0] = self.angle
        observation[1] = self.velocity
        observation[2] = self.distance_to_goal

        return np.array([observation])
    
def able_to_stop(angle, velocity, distance):
    pos = 0
    time_over = 0
    if distance > 0:
        sing = -1
    else:
        sing = 1
    while True:
        time_over += 1
        angle += sing * 0.2
        angle = max(-2, min(angle, 2))
        velocity, pos_change = simulate_ball_rolling(angle, velocity)
        pos += pos_change
        if sing == -1:
            if velocity <= 0.0 and pos <= distance:
                return True, pos, time_over
            elif velocity <= 0.0 and pos > distance:
                return False, pos, time_over
        else:
            if velocity >= 0.0 and pos >= distance:
                return True, pos, time_over
            elif velocity >= 0.0 and pos < distance:
                return False, pos, time_over
            
def will_exceed_speed(angle, distance, velocity, max_velocity):
    # Simulate the ball's movement with the maximum turn in the other direction
    pos = 0
    time_over = 0
    if distance > 0:
        sing = -1
    else:
        sing = 1

    while True:
        time_over += 1
        angle += sing * 0.2
        angle = max(-2, min(angle, 2))
        velocity, pos_change = simulate_ball_rolling(angle, velocity)
        pos += pos_change

        # Check if the velocity exceeds the maximum allowed speed
        if abs(velocity) > max_velocity:
            return True, pos, time_over

        if sing == -1:
            if velocity <= 0.0:
                return False, pos, time_over
        else:
            if velocity >= 0.0:
                return False, pos, time_over

            

def no_movment(angle, velocity, distance, rounds=1):           
    start_angle = angle
    start_velocity = velocity
    start_distance = distance
    pos = 0
    time_over = 0
    if distance > 0:
        sing = -1
    else:
        sing = 1
    for i in range(rounds):
        velocity, pos_change = simulate_ball_rolling(angle, velocity)
        pos += pos_change
        time_over += 1
    while True:
        if sing == -1:
            if velocity <= 0.0 and pos <= distance:
                return no_movment(start_angle, start_velocity, start_distance, rounds=rounds+1)
            elif velocity <= 0.0 and pos > distance:
                return False, pos, time_over, rounds
        else:
            if velocity >= 0.0 and pos >= distance:
                return no_movment(start_angle, start_velocity, start_distance, rounds=rounds+1)
            elif velocity >= 0.0 and pos < distance:
                return False, pos, time_over, rounds
            
        time_over += 1
        angle = sing * 0.2
        angle = max(-2, min(angle, 2))
        velocity, pos_change = simulate_ball_rolling(angle, velocity)
        pos += pos_change
        
            
           
def out_of_bounds(position):
    if abs(position) > 1*1000:
        return True
    else:
        return False   
    
logdir = "logs"
if not os.path.exists(logdir):
    os.makedirs(logdir)


env = ballRollingEnv()

#model = PPO('MlpPolicy', env, verbose=1)  # type: ignore
model = PPO.load("AI/models/modelRL_1.zip", env=env) # type: ignore


"""
wandb.init(
    project="RL-Maze",
    config={
        "environment": "ballRolling",
        "algorithm": "PPO",
        "learning_rate": "0.0003",
        "policy": "MlpPolicy",
        "total_timesteps": "10000",
        "n_steps": "int = 2048",
        "batch_size": "int = 64",
        "gamma":" float = 0.99",
        "gae_lambda": "float = 0.95",
        "ent_coef": "float = 0.0",
        "vf_coef": "float = 0.5",
        "max_grad_norm": "float = 0.5",
    }
)


iterations = 1

episodes = 5
for iteration in range(iterations):
    model.learn(total_timesteps=100000)
    for episode in range(episodes):
        obs = env.reset()
        done = False
        score = 0
        step = 0
        while not done:
            action, _ = model.predict(obs) # type: ignore
            obs, reward, done, info = env.step(action)
            score += reward
            if step > 1000:
                done = True
            step += 1
            if step % LOG_STEP == 0:
                wandb.log({"ball_position": info['ball_position'], "reward": reward, "distance_to_goal": info['distance_to_goal'], "velocity": info['velocity'], "step": step, "episode": episode, "angle": info['angle'], "goal_position": info['goal_position'], "score": score, 'able_to_stop_ball': info['able_to_stop_ball']})
            
model_path = "AI/models/modelRL_1_2.zip"
model.save(model_path)
print(f"Model saved to {model_path}")"""