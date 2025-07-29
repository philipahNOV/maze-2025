# neuralNets.py
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as distributions
from utilities import sequentialModel1D, symlog, symexp
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class SequenceModel(nn.Module):
    def __init__(self, representationSize, actionSize, recurrentStateSize):
        super().__init__()
        self.recurrent = nn.GRUCell(representationSize + actionSize, recurrentStateSize)

    def forward(self, representation, action, recurrentState):
        return self.recurrent(torch.cat((representation, action), -1), recurrentState)

    def initializeRecurrentState(self, size=1):
        return torch.zeros((size, self.recurrent.hidden_size)).to(device)

class PriorNet(nn.Module):
    def __init__(self, inputSize, representationLength=8, representationClasses=8):
        super().__init__()
        self.length = representationLength
        self.classes = representationClasses
        self.size = representationLength * representationClasses
        self.mlp = sequentialModel1D(inputSize, [128, 128], self.size)
        self.uniformMix = 0.01

    def forward(self, x):
        logits = self.mlp(x).view(-1, self.length, self.classes)
        probs = F.softmax(logits, -1)
        mixed = (1 - self.uniformMix) * probs + self.uniformMix * torch.ones_like(probs) / self.classes
        final_logits = distributions.utils.probs_to_logits(mixed)
        sample = F.gumbel_softmax(final_logits, hard=True).reshape(-1, self.size)
        return sample, final_logits

class PosteriorNet(nn.Module):
    def __init__(self, inputSize, representationLength=8, representationClasses=8):
        super().__init__()
        self.length = representationLength
        self.classes = representationClasses
        self.size = representationLength * representationClasses
        self.mlp = sequentialModel1D(inputSize, [128, 128], self.size)
        self.uniformMix = 0.01

    def forward(self, x):
        logits = self.mlp(x).view(-1, self.length, self.classes)
        probs = F.softmax(logits, -1)
        mixed = (1 - self.uniformMix) * probs + self.uniformMix * torch.ones_like(probs) / self.classes
        final_logits = distributions.utils.probs_to_logits(mixed)
        sample = F.gumbel_softmax(final_logits, hard=True).reshape(-1, self.size)
        return sample, final_logits

class RewardPredictor(nn.Module):
    def __init__(self, inputSize):
        super().__init__()
        self.mlp = sequentialModel1D(inputSize, [128, 128], 1)
        self.setLastLayerToZeros(self.mlp)

    def forward(self, x, useSymexp=False):
        out = self.mlp(x).squeeze(-1)
        return symexp(out) if useSymexp else out

    def setLastLayerToZeros(self, net):
        nn.init.zeros_(net[-1].weight)
        nn.init.zeros_(net[-1].bias)

LOG_STD_MAX = 2
LOG_STD_MIN = -5

class Actor(nn.Module):
    def __init__(self, inputSize, actionSize, actionLow=[-200.0, -200.0], actionHigh=[200.0, 200.0]):
        super().__init__()
        self.mean = sequentialModel1D(inputSize, [128, 128], actionSize)
        self.logStd = sequentialModel1D(inputSize, [128, 128], actionSize)
        self.register_buffer("actionScale", (torch.tensor(actionHigh) - torch.tensor(actionLow)) / 2.0)
        self.register_buffer("actionBias", (torch.tensor(actionHigh) + torch.tensor(actionLow)) / 2.0)

    def forward(self, x, training=True):
        mean = self.mean(x)
        logStd = torch.tanh(self.logStd(x))
        logStd = LOG_STD_MIN + 0.5 * (LOG_STD_MAX - LOG_STD_MIN) * (logStd + 1)
        std = torch.exp(logStd)
        dist = distributions.Normal(mean, std)
        sample = dist.rsample()
        sampleTanh = torch.tanh(sample)
        action = sampleTanh * self.actionScale + self.actionBias

        if training:
            logProb = dist.log_prob(sample) - torch.log(self.actionScale * (1 - sampleTanh.pow(2)) + 1e-6)
            return action, logProb.sum(-1), dist.entropy().sum(-1)
        return action

class Critic(nn.Module):
    def __init__(self, inputSize):
        super().__init__()
        self.mlp = sequentialModel1D(inputSize, [128, 128], 1)
        self.setLastLayerToZeros(self.mlp)

    def forward(self, x):
        return self.mlp(x).squeeze(-1)

    def setLastLayerToZeros(self, net):
        nn.init.zeros_(net[-1].weight)
        nn.init.zeros_(net[-1].bias)