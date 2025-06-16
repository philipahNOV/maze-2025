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


episodes = 2500
MAX_STEPS = 5000  # Maximum number of steps to simulate the ball rolling
LEARNING_RATE = 0.001  # Learning rate for the optimizer
UPDATE_INTERVAL = 100  # Update the model every N steps
BATCH_SIZE = 64  # Batch size for training the neural network
GAMMA = 0.99  # Discount factor for future rewards
SHOW_PLOT = 1000  # Display the plot of the results
epsilon = 1.0  # Initial epsilon value for exploration
MIN_EPSILON = 0.01  # Minimum epsilon value to ensure some exploration
EPSILON_DECAY = 0.99975  # Decay multiplier to reduce epsilon each episode
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = episodes // 2
epsilon_decay_value = (epsilon - MIN_EPSILON) / (END_EPSILON_DECAYING - START_EPSILON_DECAYING)
DISCOUNT = 0.99
MIN_REPLAY_MEMORY_SIZE = 500
UPDATE_TARGET_EVERY = 5

class ReplayMemory:
    def __init__(self, capacity):
        self.memory = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    def __len__(self):
        return len(self.memory)

    

class DQNAgent:
    def __init__(self, state_dim, action_dim, replay_memory_size=10000, batch_size=64, gamma=0.99, lr=0.001):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.batch_size = batch_size
        self.gamma = gamma
        self.lr = lr
        
        self.memory = ReplayMemory(replay_memory_size)
        self.model = DQN(state_dim, action_dim)
        self.target_model = DQN(state_dim, action_dim)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.lr)
        
        # Copy the initial parameters to the target model
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.eval()
        
    def select_action(self, state, epsilon):
        if random.random() < epsilon:
            return random.randrange(self.action_dim)
        else:
            with torch.no_grad():
                state = torch.FloatTensor(state).unsqueeze(0)
                q_values = self.model(state)
                return q_values.max(1)[1].item()
    
    def update(self):
        if len(self.memory) < self.batch_size:
            return
        
        batch = self.memory.sample(self.batch_size)
        state, action, reward, next_state, done = zip(*batch)
        
        state = torch.FloatTensor(state)
        action = torch.LongTensor(action)
        reward = torch.FloatTensor(reward)
        next_state = torch.FloatTensor(next_state)
        done = torch.FloatTensor(done)
        
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


class ballRollingEnv:
    def step(self, action):
        angle, velocity, distance_to_goal = state
        new_angle = angle + action*0.2
        new_angle = max(-2, min(new_angle, 2))
        new_velocity, new_position = simulate_ball_rolling(new_angle, velocity)
        distance_to_goal -= new_position
        distace_reward = calculate_reward(distance_to_goal)
        velocity_reward = vel_reward(new_velocity)
        reward = distace_reward + velocity_reward
        new_state = [new_angle, new_velocity, distance_to_goal]
        done = is_done(distance_to_goal, new_velocity)
        return new_state, reward, done

    def enviorment_reset(self):
        angle = 0.0
        velocity = 0.0
        position = 0.0
        episode = 0
        return angle, velocity, position, episode 

wandb.init(
    # set the wandb project where this run will be logged
    project="MAZE",

    # track hyperparameters and run metadata
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



def calculate_reward(distance_to_goal):
    if -0.1 < distance_to_goal < 0.1:
        return 25
    else:
        return -1
    
def vel_reward(vel):
    if vel[:, 1] > 0.5:
        return -10
    else:
        return 0

def compute_loss(batch):
    states, actions, rewards, next_states, dones = zip(*batch)
    
    actions = [int(action.item()) if torch.is_tensor(action) else int(action) for action in actions]
    
    states = torch.tensor(states, dtype=torch.float)
    actions = torch.tensor(actions, dtype=torch.long)
    rewards = torch.tensor(rewards, dtype=torch.float)
    next_states = torch.tensor(next_states, dtype=torch.float)
    dones = torch.tensor(dones, dtype=torch.float)
    
    model_output = model(states)
    if model_output.dim() > 1 and model_output.size(1) > 1:
        current_q_values = model_output.gather(1, actions.unsqueeze(-1)).squeeze(-1)
    else:
        current_q_values = model_output.squeeze()
    
    next_state_values = model(next_states).max(1)[0].detach()
    target_q_values = reward + (GAMMA * next_state_values * (1 - dones))
    target_q_values = target_q_values.squeeze(-1)  
    loss = F.mse_loss(current_q_values, target_q_values)
    return loss, reward

def norm_input(state):
    angle_nor = (state[0] + 2) / 4
    vel_std = (state[1] + 3) / 6
    vel_nor = max(0, min(vel_std, 1))
    dis_nor = (state[2] + 10) / 20
    dis_nor = max(0, min(dis_nor, 1))
    i_v = [angle_nor, vel_nor, dis_nor]
    return i_v
 




def is_done(distance_to_goal, velosity):
    if -0.1 < distance_to_goal < 0.1 and 0.5 < velosity < 0.5:
        return True
    else:
        return False
    
def train(done, step, target_updata_counter):
    if len(replay_buffer) < MIN_REPLAY_MEMORY_SIZE:
        return
    
    batch = replay_buffer.sample(BATCH_SIZE)

    current_states = torch.tensor([transition[0] for transition in batch], dtype=torch.float)
    current_qs_list = model(torch.tensor(current_states, dtype=torch.float))

    new_current_states = torch.tensor([transition[3] for transition in batch], dtype=torch.float)
    new_current_qs_list = model(torch.tensor(new_current_states, dtype=torch.float))

    X = []
    y = []

    for index, (current_state, action, reward, new_current_state, done) in enumerate(batch):
        if not done:
            max_future_q = torch.max(new_current_qs_list[index])
            new_q = reward + DISCOUNT * max_future_q
        else:
            new_q = reward

        current_qs = current_qs_list[index]
        current_qs[action] = new_q

        X.append(current_state)
        y.append(current_qs)

    model.fit(torch.tensor(X, dtype=torch.float), torch.tensor(y, dtype=torch.float),batch_size=BATCH_SIZE, verbose=0)

    if done:
        target_updata_counter += 1
    
    if target_updata_counter > UPDATE_TARGET_EVERY:
        target_model.set_weights(model.get_weights())
        target_updata_counter = 0




input_size = 3
output_size = 1
# Create an instance of the neural network
model = NeuralNetwork(input_size, output_size)
target_model = NeuralNetwork(input_size, output_size)
target_model.set_weights(model.get_weights())
replay_buffer = ReplayBuffer(capacity=10000)
optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

episode_rew = []


for episode in range(episodes):
    goal_position = 5.0
    angle, vel, pos, episode = enviorment_reset()
    state = torch.tensor([angle, vel, goal_position - pos], dtype=torch.float)
    episode_reward = 0
    step = 1
    target_updata_counter = 0
    done = False
    while not done:
        if np.random.random() > epsilon:
            action = model(torch.tensor(state, dtype=torch.float))
        else:
            action = torch.tensor(random.uniform(0.0, 1.0), dtype=torch.float)

        new_state, reward, done = step_(state, action)
        episode_reward += reward

        replay_buffer.store((state, action, reward, new_state, done))
        train(done, step, target_updata_counter)

        done = is_done(new_state[2], vel)
        
        step += 1
        wandb.log({"reward": reward, "distance_to_goal": state[2], "velocity": state[1], "step": step, "episode": episode, "epsilon": epsilon, "angle": state[0]})
    
