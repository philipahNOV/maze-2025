import time
import os
import pickle

class DataLogger:
    def __init__(self, save_dir="ORL/logged_data", episode_id=None):
        os.makedirs(save_dir, exist_ok=True)
        self.save_dir = save_dir
        self.episode = []
        self.last_progress = 0
        self.episode_id = episode_id or int(time.time())

    def compute_reward(self, progress, goal_reached, wrong_way=False, fell=False, timeout=False):
        if goal_reached:
            return 1.0
        elif fell or timeout:
            return -1.0
        elif wrong_way:
            return -0.1
        else:
            delta_progress = progress - self.last_progress
            self.last_progress = progress
            return delta_progress - 0.01

    def record(self, state, action, reward, next_state, done):
        self.episode.append({
            "state": state,
            "action": action,
            "reward": reward,
            "next_state": next_state,
            "done": done
        })

    def save(self):
        filename = os.path.join(self.save_dir, f"episode_{self.episode_id}.pkl")
        with open(filename, 'wb') as f:
            pickle.dump(self.episode, f)
        print(f"[LOGGER] Saved {len(self.episode)} transitions to {filename}")
