from simulation import simulate_ball_rolling
from NN import DQN
from collections import deque
import torch
import torch.nn.functional as F
import matplotlib.pyplot as plt 
import random
import numpy as np
import wandb
import torch.optim as optim

# Hyperparameters
episodes = 2500
MAX_STEPS = 5000
LEARNING_RATE = 0.01
UPDATE_INTERVAL = 100
BATCH_SIZE = 64
GAMMA = 0.99
SHOW_PLOT = 1000
epsilon = 0.90
MIN_EPSILON = 0.01
EPSILON_DECAY = 0.995
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = episodes // 2
epsilon_decay_value = (epsilon - MIN_EPSILON) / (END_EPSILON_DECAYING - START_EPSILON_DECAYING)
DISCOUNT = 0.95
MIN_REPLAY_MEMORY_SIZE = 500
UPDATE_TARGET_EVERY = 5
UPDATE_LOG = 300

# Replay Memory Class
class ReplayMemory:
    def __init__(self, capacity):
        self.memory = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    def __len__(self):
        return len(self.memory)

# DQN Agent Class
class DQNAgent:
    def __init__(self, state_dim, action_dim, replay_memory_size=10000, batch_size=BATCH_SIZE, gamma=GAMMA, lr=LEARNING_RATE):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.batch_size = batch_size
        self.gamma = gamma
        self.lr = lr
        
        self.memory = ReplayMemory(replay_memory_size)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DQN(state_dim, action_dim).to(self.device)
        self.target_model = DQN(state_dim, action_dim).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
        
        # Copy the initial parameters to the target model
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.eval()
        
    def select_action(self, state, epsilon):
        if random.random() < epsilon:
            return random.randrange(self.action_dim)
        else:
            with torch.no_grad():
                state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
                q_values = self.model(state)
                return q_values.max(1)[1].item()
    
    def update(self):
        if len(self.memory) < self.batch_size:
            return
        
        batch = self.memory.sample(self.batch_size)
        state, action, reward, next_state, done = zip(*batch)
        
        state = torch.FloatTensor(state).to(self.device)
        action = torch.LongTensor(action).to(self.device)
        reward = torch.FloatTensor(reward).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        done = torch.FloatTensor([float(d) for d in done]).to(self.device)
        
        q_values = self.model(state)
        next_q_values = self.model(next_state)
        next_target_q_values = self.target_model(next_state)
        
        q_value = q_values.gather(1, action.unsqueeze(1)).squeeze(1)
        next_q_value = next_target_q_values.gather(1, next_q_values.max(1)[1].unsqueeze(1)).squeeze(1)
        expected_q_value = reward + self.gamma * next_q_value * (1 - done)
        
        loss = F.mse_loss(q_value, expected_q_value.detach())
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
    def update_target_model(self):
        self.target_model.load_state_dict(self.model.state_dict())

# Ball Rolling Environment Class
class ballRollingEnv:
    def __init__(self):
        self.action_space = np.array([-0.02, -0.01, 0.0, 0.01, 0.02])
        self.state = None
    
    def enviorment_reset(self,goal_position):
        self.angle = 0.0
        self.velocity = 0.0
        self.position = 0.0
        self.goal_position = goal_position
        self.distance_to_goal = self.goal_position - self.position
        self.state = [self.angle, self.velocity, self.distance_to_goal, 0]  # Set the initial state
        return self.state

    def step(self, action):
        angle, velocity, distance_to_goal, episode = self.state # type: ignore
        new_angle = angle + self.action_space[action]
        new_angle = max(-2, min(new_angle, 2))
        new_velocity, new_position = simulate_ball_rolling(new_angle, velocity)
        distance_to_goal -= new_position
        distance_reward = calculate_reward(distance_to_goal)
        velocity_reward = vel_reward(new_velocity)
        directional_reward = direction_reward(new_velocity, distance_to_goal)
        reward = distance_reward + velocity_reward + directional_reward
        new_state = [new_angle, new_velocity, distance_to_goal, episode + 1]
        done = is_done(distance_to_goal, new_velocity)
        self.state = new_state  # Update the state attribute
        return new_state, reward, done

# Additional Helper Functions
def calculate_reward(distance_to_goal):
    if abs(distance_to_goal) <= 0.01:
        return 25
    elif abs(distance_to_goal) <= 0.02:
        return 15
    elif abs(distance_to_goal) <= 0.03:
        return 10
    elif abs(distance_to_goal) <= 0.04:
        return 5
    else:
        return -1
    
def vel_reward(vel):
    velocity_reward = np.where(
        (-0.03 < vel) & (vel < 0.03),
        11, 
        np.where(
            (vel > 0.1) | (vel < -0.1),
            -((vel)**2) - 5, 
            (((vel)**(-2))/2)
        )
    )
    return velocity_reward
    
def direction_reward(vel, distance_to_goal):
    if vel > 0 and distance_to_goal > 0:
        return 5
    elif vel < 0 and distance_to_goal < 0:
        return 5
    else:
        return -5
   
    
def is_done(distance_to_goal, velocity):
    return -0.05 < distance_to_goal < 0.05 # and 0.05 < velocity < 0.05


def goal_position_funk(finished_amount, old_finish_amount, goal_position):
    if old_finish_amount != finished_amount:
        old_finish_amount = finished_amount
        return 0.5 #random.choice([random.uniform(-0.5, -0.1), random.uniform(0.1, 0.5)])
    else:
        return goal_position

# Wandb Initialization
wandb.init(
    project="MAZE",
    config={
        "learning_rate": LEARNING_RATE,
        "architecture": "q-learning",
        "dataset": "NONE",
        "epochs": MAX_STEPS,
        "batch_size": BATCH_SIZE,
        "gamma": GAMMA,
        "epsilon": epsilon,
        "epsilon_decay": EPSILON_DECAY,
        "epsilon_min": MIN_EPSILON,
        "epsilon_decay_value": epsilon_decay_value,
        "update_interval": UPDATE_INTERVAL,
        "show_plot": SHOW_PLOT,
    }
)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
if device.type == 'cuda':
    print("Successfully connected to CUDA.")
# Main Training Loop
env = ballRollingEnv()
agent = DQNAgent(state_dim=4, action_dim=len(env.action_space))

episode_rew = []
finished_amount = 0
old_finished_amount = -1
for episode in range(episodes):
    goal_position = goal_position_funk(finished_amount, old_finished_amount, goal_position=None)
    state = env.enviorment_reset(goal_position)  # [angle, velocity, distance_to_goal, episode]
    episode_reward = 0
    step = 1
    done = False
    while not done and step <= MAX_STEPS:
        action = agent.select_action(state, epsilon)
        next_state, reward, done = env.step(action)
        agent.memory.push(state, action, reward, next_state, done)
        state = next_state
        
        agent.update()
        episode_reward += reward

        done = is_done(next_state[2], next_state[1])
        if done:
            finished_amount += 1
        
        step += 1
        if step % UPDATE_LOG == 0:
            wandb.log({"reward": reward, "distance_to_goal": state[2], "velocity": state[1], "step": step, "episode": episode, "epsilon": epsilon, "angle": state[0], 'finished_amount': finished_amount, 'episode_reward': episode_reward})
        

    if episode % UPDATE_TARGET_EVERY == 0:
        agent.update_target_model()
    
    epsilon = max(MIN_EPSILON, epsilon * EPSILON_DECAY)
    episode_rew.append(episode_reward)

    """if episode % SHOW_PLOT == 0:
        plt.plot(episode_rew)
        plt.xlabel('Episode')
        plt.ylabel('Total Reward')
        plt.show()"""
