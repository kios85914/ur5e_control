

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
import numpy as np

def init_layer_uniform(layer: nn.Linear, init_w: float = 3e-3) -> nn.Linear:
    """Init uniform parameters on the single layer."""
    layer.weight.data.uniform_(-init_w, init_w)
    layer.bias.data.uniform_(-init_w, init_w)

    return layer

class Actor(nn.Module):
    def __init__(
        self,        
        max_act,
        min_act,
        state_dim: int,
        action_dim: int,
        init_w: float = 3e-3,
    ):
        """Initialize."""
        super(Actor, self).__init__()
        
#         self.Conv = Conv
        self.device = torch.device("cuda")
        self.hidden1 = nn.Linear(state_dim, 256)
        self.hidden2 = nn.Linear(256, 256)

        self.log_std_layer = nn.Linear(256, action_dim)
        self.log_std_layer = init_layer_uniform(self.log_std_layer)
        
        self.mu_layer = nn.Linear(256, action_dim)
        self.mu_layer = init_layer_uniform(self.mu_layer)
        
        self.max_action = torch.from_numpy(np.array(max_act)).to(self.device)
        self.min_action = torch.from_numpy(np.array(min_act)).to(self.device)
        self.max_log_std = 2
        self.min_log_std = -20
        self.action_dim = action_dim
        
    def action_norm(self, action: np.ndarray) -> np.ndarray:
        """Change the range (-1, 1) to (low, high)."""
        
        
        
        act_dim = int(self.action_dim/2)
        action = action.reshape(-1,self.action_dim)
#         low = -np.array(self.min_action*act_dim)
#         high = np.array(self.max_action*act_dim)
#         lim = [0.69, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2]
        low = self.min_action
        high = self.max_action
#         low = -torch.from_numpy(np.array(self.min_action*act_dim)).to(self.device)
#         high = torch.from_numpy(np.array(self.max_action*act_dim)).to(self.device)

        scale_factor = (high - low) / 2
        reloc_factor = high - scale_factor
        action = action * scale_factor + reloc_factor
        action = torch.clip(action, low, high)
        
        
        return action.to(torch.float32)
    
    def sample(self):
        
        action = np.random.sample((self.action_dim,))
        action = (action - 0.5) * 2 #[0, 1] to [-1, 1]
        action = self.action_norm(torch.from_numpy(action).to(self.device))
        
        return action

    def forward(self, state, is_stochastic=True, with_log_prob=True):
        """Forward method implementation."""
        #print("actor_state",state.shape)
#         x = self.Conv(state)
#         x = x.view(-1,1280)
        x = F.relu(self.hidden1(state))
        x = F.relu(self.hidden2(x))
        mu = self.mu_layer(x)
        
        log_std = self.log_std_layer(x).tanh()
        log_std = self.min_log_std + 0.5 * (
            self.max_log_std - self.min_log_std
        ) * (log_std + 1)
        std = torch.exp(log_std)
        dist = Normal(mu, std)
        if is_stochastic:
            #log_std = self.min_log_std + 0.5 * (self.max_log_std - self.min_log_std) * (log_std + 1)
            z = dist.rsample()
        else:
            z = dist.mean
        action = z.tanh()

        if with_log_prob:
            log_prob = dist.log_prob(z) - torch.log(1 - action.pow(2) + 1e-2)
            log_prob = log_prob.sum(-1, keepdims=True)
        else:
            log_prob = None
    
        action = self.action_norm(action)
        
        return action, log_prob