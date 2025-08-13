import os
import json
import numpy as np

class PathMemory:
    def __init__(self, config):
        self.paths = []
        self.max_paths = config['path_finding'].get('path_cache_size', 30)
        self.tolerance = config['path_finding'].get("path_cache_tolerance", 10)
        self.cache_file = config['path_finding'].get("path_cache_file", "path_cache.json")
        self.load_cache()

    def _within_tolerance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2)) <= self.tolerance

    def get_cached_path(self, start, goal):
        for entry in self.paths:
            s, g, path = entry["start"], entry["goal"], entry["path"]
            if self._within_tolerance(start, s) and self._within_tolerance(goal, g):
                return path
        return None

    def cache_path(self, start, goal, path):
        if len(self.paths) >= self.max_paths:
            self.paths.pop(0)

        self.paths.append({
            "start": list(start),
            "goal": list(goal),
            "path": [list(p) for p in path]
        })
        print(f"[PathMemory] Cached path. Total stored: {len(self.paths)}")
        self.save_cache()

    def save_cache(self):
        try:
            with open(self.cache_file, "w") as f:
                json.dump(self.paths, f)
            print(f"[PathMemory] Cache saved to {self.cache_file}")
        except Exception as e:
            print(f"[PathMemory] Error saving cache: {e}")

    def load_cache(self):
        if not os.path.exists(self.cache_file):
            return
        try:
            with open(self.cache_file, "r") as f:
                self.paths = json.load(f)
            print(f"[PathMemory] Loaded {len(self.paths)} paths from cache.")
        except Exception as e:
            print(f"[PathMemory] Error loading cache: {e}")