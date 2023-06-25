import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

class DQN(nn.Module):

    def __init__(self, n_observations, n_actions):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)

def calcval(pi,a,p,ss,r,gamma,vpi):
    s_pi = 0
    for i in range(len(a)):
        s_pi = s_pi + pi[i]
    sum = 0
    for i in range(len(ss)):
        for j in range(len(r)):
            sum = sum + p[i][j]*(r+gamma*vpi[i])
    return s_pi * sum
    
def reward():



def train(model, state, reward):
