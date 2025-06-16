from stable_baselines3 import PPO
import numpy as np
import random

from pathlib import Path

file_path = Path("AI/models/modelRL_final.zip")
print(file_path.exists())

N_DISCRETE_ACTIONS = 3


model = PPO.load("AI/models/modelRL_final.zip")
print(model.predict(np.array([-5.0, 0.0, 5.0])))

for i in range(10):
    print(model.predict(np.array([random.uniform(-5,5), random.uniform(-5,5), random.uniform(-5,5)])))
