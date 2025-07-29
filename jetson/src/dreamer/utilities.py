import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import csv
from collections import deque
import random
import os
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def sequentialModel1D(inputSize, hiddenSizes, outputSize, finishWithActivation = False, activationFunction = nn.ReLU):
    layers = []
    currentInputSize = inputSize

    for hiddenSize in hiddenSizes:
        layers.append(nn.Linear(currentInputSize, hiddenSize))
        layers.append(activationFunction())
        currentInputSize = hiddenSize
    
    layers.append(nn.Linear(currentInputSize, outputSize))
    if finishWithActivation:
        layers.append(activationFunction())

    return nn.Sequential(*layers).to(device)

def sequentialModel3D(inputChannels, hiddenChannels, activationFunction = nn.Tanh):
        layers = []
        currentInputSize = inputChannels
        kernelSize = 7
        
        for hiddenSize in hiddenChannels:
            stride = kernelSize // 2
            layers.append(nn.Conv2d(currentInputSize, hiddenSize, kernelSize, stride))
            layers.append(activationFunction())
            currentInputSize = hiddenSize
            kernelSize = max(3, kernelSize - 2)

        layers.append(nn.Flatten())
        return nn.Sequential(*layers).to(device)

def calculateConvNetOutputSize(net, inputShape):
    return torch.numel(net(torch.ones(inputShape)))
    
def displayImage(imageNdarray):
    import matplotlib.pyplot as plt
    plt.imshow(imageNdarray)
    plt.axis('off')
    plt.show()

def saveImage(imageNdarray, filename):
    from PIL import Image
    image = Image.fromarray(imageNdarray)
    image.save(filename)

@torch.no_grad()
def symlog(x):
    return torch.sign(x) * torch.log1p(torch.abs(x))

@torch.no_grad()
def symexp(x):
    return torch.sign(x) * (torch.exp(torch.abs(x)) - 1)

def saveLossesToCSV(filename, metrics):
    fileAlreadyExists = os.path.isfile(filename + ".csv")
    with open(filename + ".csv", mode='a', newline='') as file:
        writer = csv.writer(file)
        if not fileAlreadyExists:
            writer.writerow(metrics.keys())
        writer.writerow(metrics.values())

def saveLossesToCSV(filename, metrics):
    fileAlreadyExists = os.path.isfile(filename + ".csv")
    with open(filename + ".csv", mode='a', newline='') as file:
        writer = csv.writer(file)
        if not fileAlreadyExists:
            writer.writerow(metrics.keys())
        writer.writerow(metrics.values())

class EpisodeBuffer:
    def __init__(self, size=20):
        self.size = size
        self.observations = deque(maxlen=size)
        self.actions = deque(maxlen=size)
        self.rewards = deque(maxlen=size)

    def addEpisode(self, observations, actions, rewards):
        self.observations.append(observations)
        self.actions.append(actions)
        self.rewards.append(rewards)

    def sampleEpisode(self):
        episodeIndex = random.randint(0, len(self) - 1)
        return self.observations[episodeIndex], self.actions[episodeIndex], self.rewards[episodeIndex]
    
    def sampleEpisodes(self, numEpisodes):
        if numEpisodes > len(self):
            raise ValueError("Requested more samples than available episodes.")
        
        episodeIndices = random.sample(range(len(self)), numEpisodes)
        observationsList = [self.observations[i] for i in episodeIndices]
        actionsList = [self.actions[i] for i in episodeIndices]
        rewardsList = [self.rewards[i] for i in episodeIndices]

        observationsStacked = torch.stack(observationsList)
        actionsStacked = torch.stack(actionsList)
        rewardsStacked = torch.stack(rewardsList)

        return observationsStacked, actionsStacked, rewardsStacked

    def getNewestEpisode(self):
        episodeIndex = len(self) - 1
        return self.observations[episodeIndex], self.actions[episodeIndex], self.rewards[episodeIndex]
    
    def __len__(self):
        return len(self.observations)
    
def saveVideoFrom4DTensor(observations, filename, fps=30):
    # Simple placeholder - not needed for maze project
    pass

def saveVideoFromGymEnv(actor, envName, filename, frameLimit=512, fps=30, macroBlockSize=16):
    # Simple placeholder - not needed for maze project  
    pass

class Moments(nn.Module):
    def __init__( self, decay = 0.99, min_=1, percentileLow = 0.05, percentileHigh = 0.95):
        super().__init__()
        self._decay = decay
        self._min = torch.tensor(min_)
        self._percentileLow = percentileLow
        self._percentileHigh = percentileHigh
        self.register_buffer("low", torch.zeros((), dtype=torch.float32, device=device))
        self.register_buffer("high", torch.zeros((), dtype=torch.float32, device=device))

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        x = x.detach()
        low = torch.quantile(x, self._percentileLow)
        high = torch.quantile(x, self._percentileHigh)
        self.low = self._decay*self.low + (1 - self._decay)*low
        self.high = self._decay*self.high + (1 - self._decay)*high
        inverseScale = torch.max(self._min, self.high - self.low)
        return self.low.detach(), inverseScale.detach()
    