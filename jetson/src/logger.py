# rl_logger.py
import json
import os

class OfflineLogger:
    def __init__(self, save_path="ORL/rl_data", episode_limit=100):
        self.episode_limit = episode_limit
        self.current_episode = []
        self.episodes = []
        self.episode_counter = 0
        os.makedirs(save_path, exist_ok=True)
        self.save_path = save_path

    def log_step(self, state, action, reward, next_state, done):
        self.current_episode.append({
            "state": state,
            "action": action,
            "reward": reward,
            "next_state": next_state,
            "done": done
        })
        if done:
            self.episodes.append(self.current_episode)
            self.episode_counter += 1
            self.save_episode()
            self.current_episode = []

    def save_episode(self):
        path = os.path.join(self.save_path, f"episode_{self.episode_counter:03}.json")
        with open(path, 'w') as f:
            json.dump(self.episodes[-1], f, indent=2)
