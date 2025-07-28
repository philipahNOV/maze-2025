# dreamerv3_agent.py
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import threading
import time
from collections import deque
import os


class WorldModel(nn.Module):
    def __init__(self, obs_dim=6, action_dim=2, hidden_dim=256, latent_dim=32):
        super().__init__()
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        self.latent_dim = latent_dim
        
        # Encoder: observations -> latent representations
        self.encoder = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, latent_dim * 2)
        )
        
        # Dynamics model: predicts next latent state
        self.dynamics = nn.Sequential(
            nn.Linear(latent_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, latent_dim * 2)
        )
        
        # Decoder: latent -> observations
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, obs_dim)
        )
        
        # Reward predictor
        self.reward_predictor = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
        # Continue predictor
        self.continue_predictor = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
            nn.Sigmoid()
        )
    
    def encode(self, obs):
        encoded = self.encoder(obs)
        mean, std = torch.chunk(encoded, 2, dim=-1)
        std = F.softplus(std) + 1e-4
        return mean, std
    
    def sample_latent(self, mean, std):
        return mean + std * torch.randn_like(std)
    
    def forward(self, obs, actions):
        mean, std = self.encode(obs)
        latent = self.sample_latent(mean, std)
        
        latent_action = torch.cat([latent[:-1], actions[:-1]], dim=-1)
        next_mean, next_std = torch.chunk(self.dynamics(latent_action), 2, dim=-1)
        next_std = F.softplus(next_std) + 1e-4
        
        decoded_obs = self.decoder(latent)
        rewards = self.reward_predictor(latent[:-1])
        continues = self.continue_predictor(latent[:-1])
        
        return {
            'latent_mean': mean,
            'latent_std': std,
            'latent': latent,
            'next_latent_mean': next_mean,
            'next_latent_std': next_std,
            'decoded_obs': decoded_obs,
            'predicted_rewards': rewards,
            'continues': continues
        }


class Actor(nn.Module):
    def __init__(self, latent_dim=32, action_dim=2, hidden_dim=256):
        super().__init__()
        self.action_dim = action_dim
        
        self.network = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 2)
        )
    
    def forward(self, latent):
        output = self.network(latent)
        mean, std = torch.chunk(output, 2, dim=-1)
        std = F.softplus(std) + 1e-4
        return mean, std
    
    def sample_action(self, latent):
        mean, std = self.forward(latent)
        action = mean + std * torch.randn_like(std)
        return torch.tanh(action)


class Critic(nn.Module):
    def __init__(self, latent_dim=32, hidden_dim=256):
        super().__init__()
        
        self.network = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
    
    def forward(self, latent):
        return self.network(latent)


class DreamerV3Agent:
    def __init__(self, obs_dim=6, action_dim=2):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        
        self.latent_dim = 32
        self.hidden_dim = 256
        self.learning_rate = 3e-4
        self.batch_size = 16
        self.sequence_length = 50
        
        self.world_model = WorldModel(obs_dim, action_dim, self.hidden_dim, self.latent_dim)
        self.actor = Actor(self.latent_dim, action_dim, self.hidden_dim)
        self.critic = Critic(self.latent_dim, self.hidden_dim)
        
        self.world_model_optimizer = torch.optim.Adam(self.world_model.parameters(), lr=self.learning_rate)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=self.learning_rate)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=self.learning_rate)
        
        self.buffer = deque(maxlen=10000)
        self.current_episode = []
        
        self.exploration_noise = 0.1
        self.action_scale = 200.0
        self.training_step = 0
        self.last_latent = None
    
    def reset_episode(self):
        if len(self.current_episode) > 10:
            self.buffer.append(self.current_episode.copy())
        self.current_episode = []
        self.last_latent = None
    
    def select_action(self, observation, training=True):
        obs_tensor = torch.FloatTensor(observation).unsqueeze(0)
        
        with torch.no_grad():
            mean, std = self.world_model.encode(obs_tensor)
            latent = self.world_model.sample_latent(mean, std)
            self.last_latent = latent
            
            action = self.actor.sample_action(latent)
            action = action.squeeze(0).numpy()
            
            if training:
                noise = np.random.normal(0, self.exploration_noise, size=action.shape)
                action = np.clip(action + noise, -1, 1)
        
        scaled_action = action * self.action_scale
        return scaled_action.astype(np.float32)
    
    def store_transition(self, obs, action, reward, next_obs, done):
        normalized_action = np.clip(action / self.action_scale, -1, 1)
        
        transition = {
            'obs': obs.copy(),
            'action': normalized_action.copy(),
            'reward': float(reward),
            'next_obs': next_obs.copy() if next_obs is not None else obs.copy(),
            'done': bool(done)
        }
        
        self.current_episode.append(transition)
    
    def train(self):
        if len(self.buffer) < 2:
            return {}
        
        episodes = list(self.buffer)[-self.batch_size:]
        
        obs_batch = []
        action_batch = []
        reward_batch = []
        done_batch = []
        
        for episode in episodes:
            if len(episode) < self.sequence_length:
                continue
            
            start_idx = np.random.randint(0, max(1, len(episode) - self.sequence_length))
            subseq = episode[start_idx:start_idx + self.sequence_length]
            
            obs_seq = [step['obs'] for step in subseq]
            action_seq = [step['action'] for step in subseq]
            reward_seq = [step['reward'] for step in subseq]
            done_seq = [step['done'] for step in subseq]
            
            obs_batch.append(obs_seq)
            action_batch.append(action_seq)
            reward_batch.append(reward_seq)
            done_batch.append(done_seq)
        
        if len(obs_batch) == 0:
            return {}
        
        obs_tensor = torch.FloatTensor(obs_batch)
        action_tensor = torch.FloatTensor(action_batch)
        reward_tensor = torch.FloatTensor(reward_batch)
        done_tensor = torch.FloatTensor(done_batch)
        
        world_model_loss = self._train_world_model(obs_tensor, action_tensor, reward_tensor, done_tensor)
        actor_loss, critic_loss = self._train_actor_critic(obs_tensor, action_tensor, reward_tensor, done_tensor)
        
        self.training_step += 1
        
        return {
            'world_model_loss': world_model_loss,
            'actor_loss': actor_loss,
            'critic_loss': critic_loss,
            'training_step': self.training_step,
            'buffer_size': len(self.buffer)
        }
    
    def _train_world_model(self, obs, actions, rewards, dones):
        self.world_model_optimizer.zero_grad()
        
        batch_size, seq_len = obs.shape[:2]
        obs_flat = obs.view(-1, self.obs_dim)
        actions_flat = actions.view(-1, self.action_dim)
        rewards_flat = rewards.view(-1, 1)
        dones_flat = dones.view(-1, 1)
        
        outputs = self.world_model(obs_flat.view(batch_size, seq_len, -1), 
                                  actions_flat.view(batch_size, seq_len, -1))
        
        recon_loss = F.mse_loss(outputs['decoded_obs'], obs_flat)
        
        if seq_len > 1:
            true_next_latent_mean = outputs['latent_mean'][:, 1:]
            pred_next_latent_mean = outputs['next_latent_mean']
            dynamics_loss = F.mse_loss(pred_next_latent_mean, true_next_latent_mean)
            
            reward_loss = F.mse_loss(outputs['predicted_rewards'].view(-1), 
                                   rewards_flat[:-1].view(-1))
            
            continue_targets = 1.0 - dones_flat[:-1]
            continue_loss = F.binary_cross_entropy(outputs['continues'].view(-1), 
                                                  continue_targets.view(-1))
        else:
            dynamics_loss = torch.tensor(0.0)
            reward_loss = torch.tensor(0.0)
            continue_loss = torch.tensor(0.0)
        
        latent_mean = outputs['latent_mean']
        latent_std = outputs['latent_std']
        kl_loss = 0.5 * torch.sum(latent_mean**2 + latent_std**2 - torch.log(latent_std**2) - 1)
        kl_loss = kl_loss / (batch_size * seq_len)
        
        total_loss = recon_loss + dynamics_loss + reward_loss + continue_loss + 0.1 * kl_loss
        
        total_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.world_model.parameters(), 1.0)
        self.world_model_optimizer.step()
        
        return total_loss.item()
    
    def _train_actor_critic(self, obs, actions, rewards, dones):
        batch_size, seq_len = obs.shape[:2]
        
        with torch.no_grad():
            obs_flat = obs.view(-1, self.obs_dim)
            mean, std = self.world_model.encode(obs_flat)
            latent = self.world_model.sample_latent(mean, std)
            latent = latent.view(batch_size, seq_len, -1)
        
        returns = torch.zeros_like(rewards)
        gamma = 0.99
        
        for b in range(batch_size):
            running_return = 0
            for t in reversed(range(seq_len)):
                running_return = rewards[b, t] + gamma * running_return * (1 - dones[b, t])
                returns[b, t] = running_return
        
        self.critic_optimizer.zero_grad()
        values = self.critic(latent.view(-1, self.latent_dim)).view(batch_size, seq_len)
        critic_loss = F.mse_loss(values, returns)
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        self.actor_optimizer.zero_grad()
        with torch.no_grad():
            advantages = returns - values.detach()
        
        action_mean, action_std = self.actor(latent.view(-1, self.latent_dim))
        action_mean = action_mean.view(batch_size, seq_len, -1)
        action_std = action_std.view(batch_size, seq_len, -1)
        
        log_probs = -0.5 * ((actions - action_mean) / action_std)**2 - 0.5 * torch.log(2 * np.pi * action_std**2)
        log_probs = log_probs.sum(dim=-1)
        
        actor_loss = -(log_probs * advantages).mean()
        
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        return actor_loss.item(), critic_loss.item()
    
    def save_model(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save({
            'world_model': self.world_model.state_dict(),
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'training_step': self.training_step
        }, path)
    
    def load_model(self, path):
        if os.path.exists(path):
            checkpoint = torch.load(path)
            self.world_model.load_state_dict(checkpoint['world_model'])
            self.actor.load_state_dict(checkpoint['actor'])
            self.critic.load_state_dict(checkpoint['critic'])
            self.training_step = checkpoint.get('training_step', 0)
            print(f"Model loaded from {path}, training step: {self.training_step}")
        else:
            print(f"No model found at {path}, starting fresh")
