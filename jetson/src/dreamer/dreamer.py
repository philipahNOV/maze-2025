import torch
import torch.nn.functional as F
import torch.optim as optim
import copy
from utilities import *
from neuralNets import *
import os
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class StateEncoder(nn.Module):
    def __init__(self, inputSize, outputSize):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(inputSize, 128),
            nn.ReLU(),
            nn.Linear(128, outputSize)
        )

    def forward(self, x):
        return self.fc(x)

class StateDecoder(nn.Module):
    def __init__(self, inputSize, outputSize):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(inputSize, 128),
            nn.ReLU(),
            nn.Linear(128, outputSize)
        )

    def forward(self, x):
        return self.fc(x)

class Dreamer:
    def __init__(self):
        self.worldModelBatchSize        = 4
        self.actorCriticBatchSize       = 8
        self.representationLength       = 8
        self.representationClasses      = 8
        self.representationSize         = self.representationLength * self.representationClasses
        self.actionSize                 = 2  # For motor_x, motor_y
        self.recurrentStateSize         = 256
        self.compressedObservationSize  = 64
        self.obsSize                    = 6  # [x, y, vx, vy, theta_x, theta_y]
        self.imaginationHorizon         = 10
        self.betaPrior                  = 1
        self.betaPosterior              = 0.1
        self.betaReconstruction         = 1
        self.betaReward                 = 1
        self.betaKL                     = 1
        self.entropyScale               = 0.0003
        self.tau                        = 0.02
        self.gamma                      = 0.997
        self.lambda_                    = 0.95
        self.worldModelLR               = 1e-4
        self.criticLR                   = 1e-4
        self.actorLR                    = 1e-4

        self.encoder = StateEncoder(self.obsSize, self.compressedObservationSize).to(device)
        self.decoder = StateDecoder(self.representationSize + self.recurrentStateSize, self.obsSize).to(device)

        self.sequenceModel   = SequenceModel(self.representationSize, self.actionSize, self.recurrentStateSize).to(device)
        self.priorNet        = PriorNet(self.recurrentStateSize, self.representationLength, self.representationClasses).to(device)
        self.posteriorNet    = PosteriorNet(self.recurrentStateSize + self.compressedObservationSize, self.representationLength, self.representationClasses).to(device)
        self.rewardPredictor = RewardPredictor(self.recurrentStateSize + self.representationSize).to(device)
        self.actor           = Actor(self.recurrentStateSize + self.representationSize, self.actionSize, actionHigh=[200.0, 200.0], actionLow=[-200.0, -200.0])
        self.critic          = Critic(self.recurrentStateSize + self.representationSize).to(device)
        self.targetCritic    = copy.deepcopy(self.critic)

        self.recurrentState  = self.sequenceModel.initializeRecurrentState()
        self.valueMoments    = Moments()
        self.totalUpdates    = 0
        self.freeNats        = 1
        self.clipGradients   = False

        self.worldModelParams = (
            list(self.encoder.parameters()) + list(self.decoder.parameters()) +
            list(self.sequenceModel.parameters()) + list(self.priorNet.parameters()) +
            list(self.posteriorNet.parameters()) + list(self.rewardPredictor.parameters())
        )

        self.worldModelOptimizer = optim.AdamW(self.worldModelParams, lr=self.worldModelLR)
        self.criticOptimizer     = optim.AdamW(self.critic.parameters(), lr=self.criticLR)
        self.actorOptimizer      = optim.AdamW(self.actor.parameters(), lr=self.actorLR)

    @torch.no_grad()
    def act(self, observation, reset=False):
        if reset:
            self.recurrentState = self.sequenceModel.initializeRecurrentState()
        encodedObservation = self.encoder(observation)
        latentState, _ = self.posteriorNet(torch.cat((self.recurrentState, encodedObservation), -1))
        fullState = torch.cat((self.recurrentState, latentState), -1)
        return self.actor(fullState, training=False)

    def trainWorldModel(self, observations, actions, rewards):
        seqLen = observations.shape[1]

        encodedObs = self.encoder(observations.view(-1, self.obsSize))
        encodedObs = encodedObs.view(self.worldModelBatchSize, seqLen, -1)

        posteriors, recurrentStates, priorLogits, posteriorLogits = [], [], [], []

        for t in range(seqLen - 1):
            if t == 0:
                with torch.no_grad():
                    rec = self.sequenceModel.initializeRecurrentState(self.worldModelBatchSize)
                    post, _ = self.posteriorNet(torch.cat((rec, encodedObs[:, t]), -1))
                    recurrentStates.append(rec)
                    posteriors.append(post)
            else:
                rec = recurrentStates[t]
                post = posteriors[t]
            action = actions[:, t]
            nextRec = self.sequenceModel(post, action, rec)
            _, priorLog = self.priorNet(nextRec)
            nextPost, postLog = self.posteriorNet(torch.cat((nextRec, encodedObs[:, t + 1]), -1))

            recurrentStates.append(nextRec)
            priorLogits.append(priorLog)
            posteriors.append(nextPost)
            posteriorLogits.append(postLog)

        recurrentStates = torch.stack(recurrentStates[1:], dim=1)
        posteriors = torch.stack(posteriors[1:], dim=1)
        priorLogits = torch.stack(priorLogits, dim=1)
        posteriorLogits = torch.stack(posteriorLogits, dim=1)
        fullStates = torch.cat((recurrentStates, posteriors), -1)

        decoded = self.decoder(fullStates.view(-1, fullStates.size(-1)))
        decoded = decoded.view(self.worldModelBatchSize, seqLen - 1, self.obsSize)
        predictedRewards = self.rewardPredictor(fullStates)

        reconstructionLoss = self.betaReconstruction * F.mse_loss(decoded, observations[:, 1:], reduction="mean")
        rewardLoss = self.betaReward * F.mse_loss(predictedRewards, symlog(rewards))

        priorDist = torch.distributions.Categorical(logits=priorLogits)
        postDist = torch.distributions.Categorical(logits=posteriorLogits)
        klLoss = self.betaKL * (
            self.betaPrior * torch.maximum(torch.distributions.kl_divergence(postDist.detach(), priorDist), torch.tensor(self.freeNats)).mean() +
            self.betaPosterior * torch.maximum(torch.distributions.kl_divergence(postDist, priorDist.detach()), torch.tensor(self.freeNats)).mean()
        )

        worldModelLoss = reconstructionLoss + rewardLoss + klLoss
        self.worldModelOptimizer.zero_grad()
        worldModelLoss.backward()
        if self.clipGradients:
            torch.nn.utils.clip_grad_norm_(self.worldModelParams, 100.0)
        self.worldModelOptimizer.step()

        flatFullStates = fullStates.view(-1, fullStates.size(-1))
        sampledFullStates = flatFullStates[torch.randperm(flatFullStates.size(0))[:self.actorCriticBatchSize]].detach()

        return sampledFullStates, {
            "worldModelLoss": worldModelLoss.item(),
            "reconstructionLoss": reconstructionLoss.item(),
            "rewardLoss": rewardLoss.item(),
            "klLoss": klLoss.item(),
            "avgPredictedReward": predictedRewards.mean().item()
        }

    def trainActorCritic(self, initialFullState):
        fullState = initialFullState.detach()
        rec, lat = torch.split(fullState, [self.recurrentStateSize, self.representationSize], dim=-1)

        fullStates, logProbs, entropies = [fullState], [], []
        action, logProb, entropy = self.actor(fullState)
        logProbs.append(logProb)
        entropies.append(entropy)

        for _ in range(self.imaginationHorizon):
            rec = self.sequenceModel(lat, action, rec)
            lat, _ = self.priorNet(rec)
            fullState = torch.cat((rec, lat), -1)
            action, logProb, entropy = self.actor(fullState)
            fullStates.append(fullState)
            logProbs.append(logProb)
            entropies.append(entropy)

        fullStates = torch.stack(fullStates, dim=1)
        logProbs = torch.stack(logProbs[:-1], dim=1)
        entropies = torch.stack(entropies[:-1], dim=1)

        predictedRewards = self.rewardPredictor(fullStates[:, :-1], useSymexp=True)
        targetValues = self.targetCritic(fullStates)
        lambdaReturns = self.lambdaValues(predictedRewards, targetValues)

        _, invScale = self.valueMoments(lambdaReturns)
        advantages = (lambdaReturns - targetValues[:, :-1]) / invScale

        actorLoss = -torch.mean(advantages.detach() * logProbs + self.entropyScale * entropies)
        self.actorOptimizer.zero_grad()
        actorLoss.backward()
        self.actorOptimizer.step()

        criticValues = self.critic(fullStates.detach())
        criticLoss = F.mse_loss(criticValues[:, :-1], lambdaReturns.detach())
        self.criticOptimizer.zero_grad()
        criticLoss.backward()
        self.criticOptimizer.step()

        for p, tp in zip(self.critic.parameters(), self.targetCritic.parameters()):
            tp.data.copy_(self.tau * p.data + (1 - self.tau) * tp.data)

        return {
            "actorLoss": actorLoss.item(),
            "criticLoss": criticLoss.item(),
            "avgReward": predictedRewards.mean().item(),
            "entropy": entropies.mean().item()
        }

    def lambdaValues(self, rewards, values):
        returns = torch.zeros_like(rewards)
        last = values[:, -1]
        for t in reversed(range(rewards.shape[1])):
            returns[:, t] = rewards[:, t] + self.gamma * ((1 - self.lambda_) * values[:, t] + self.lambda_ * last)
            last = returns[:, t]
        return returns

    def saveCheckpoint(self, path):
        torch.save({
            "actor": self.actor.state_dict(),
            "critic": self.critic.state_dict(),
            "targetCritic": self.targetCritic.state_dict(),
            "encoder": self.encoder.state_dict(),
            "decoder": self.decoder.state_dict(),
            "rewardPredictor": self.rewardPredictor.state_dict(),
            "worldModel": self.sequenceModel.state_dict(),
            "priorNet": self.priorNet.state_dict(),
            "posteriorNet": self.posteriorNet.state_dict(),
            "actorOpt": self.actorOptimizer.state_dict(),
            "criticOpt": self.criticOptimizer.state_dict(),
            "worldOpt": self.worldModelOptimizer.state_dict(),
        }, path)

    def loadCheckpoint(self, path):
        ckpt = torch.load(path)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic.load_state_dict(ckpt["critic"])
        self.targetCritic.load_state_dict(ckpt["targetCritic"])
        self.encoder.load_state_dict(ckpt["encoder"])
        self.decoder.load_state_dict(ckpt["decoder"])
        self.rewardPredictor.load_state_dict(ckpt["rewardPredictor"])
        self.sequenceModel.load_state_dict(ckpt["worldModel"])
        self.priorNet.load_state_dict(ckpt["priorNet"])
        self.posteriorNet.load_state_dict(ckpt["posteriorNet"])
        self.actorOptimizer.load_state_dict(ckpt["actorOpt"])
        self.criticOptimizer.load_state_dict(ckpt["criticOpt"])
        self.worldModelOptimizer.load_state_dict(ckpt["worldOpt"])
