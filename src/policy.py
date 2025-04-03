import numpy as np
from numpy.polynomial.polynomial import Polynomial
import torch
from torch import nn
import torch.nn.functional as F
from collections import deque

class MLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim=256):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, output_dim)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(PolicyNetwork, self).__init__()
        # self.mu = MLP(state_dim, action_dim)
        # self.log_std = nn.Parameter(torch.zeros(action_dim))
        self.p = MLP(state_dim, 2*action_dim)
    
    def forward(self, state):
        # mean = self.mu(state)
        # log_std = torch.clamp(self.log_std, min=-20, max=2)
        # std = torch.exp(log_std)
        mu_sigma = self.p(state)
        mean, log_std = mu_sigma.chunk(2, dim=-1)
        log_std = torch.clamp(log_std, min=-20, max=2)
        std = torch.exp(log_std)
        return mean, std
    
    def predict(self, state):
        epsilon_tanh = 1e-6
        mean, std = self.forward(state)
        dist = torch.distributions.Normal(mean, std)
        action_unbounded = dist.rsample()
        action_bounded = torch.tanh(action_unbounded) * (1 - epsilon_tanh)
        return action_bounded, std
    
    def to(self, device):
        self.device = device
        return super(PolicyNetwork, self).to(device)

class ctrl_policy:
    def __init__(self, fps, path_to_model="actor_4000000.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.obs_dim = 45
        self.act_dim = 6
        self.actor = PolicyNetwork(self.obs_dim, self.act_dim).to(self.device)
        self.actor.load_state_dict(torch.load(path_to_model, map_location=torch.device(device=self.device), weights_only=True))
        self.dt = 1.0 / fps
        self.cap_pos_batch_size = 10
        self.cap_pos_batch = deque(maxlen=self.cap_pos_batch_size)
        self.action_batch = deque(maxlen=2)
        self.t_window = np.linspace(-self.dt*(self.cap_pos_batch_size-1), 0, self.cap_pos_batch_size)
        pass

    def get_action(self, cap_pos):
        self._update_cap_pos_batch(cap_pos)
        if len(self.cap_pos_batch) < self.cap_pos_batch_size:
            action = np.ones(6)
            self.action_batch.append(action)
            return action
        cap_rel_pos = self.cap_pos_batch[-1]
        cap_vel = self._get_cap_vel()
        tendon_len = self._get_tendon_len(cap_rel_pos)
        obs = np.concatenate([cap_rel_pos, cap_vel, tendon_len])
        action = self._predict(obs, self.action_batch[0])
        self.action_batch.append(action)
        return action

    def _predict(self, obs, last_action):
        action_scaled, _ = self.actor.predict(torch.from_numpy(obs).float())
        action_unscaled = action_scaled.detach().cpu().numpy() # [-1, 1] -> [min, max]
        filtered_action = self._action_filter(action_unscaled, last_action)
        return filtered_action
    
    def _action_filter(self, action, last_action):
        k_FILTER = 1
        filtered_action = last_action + k_FILTER*(action - last_action)*self.dt
        return filtered_action
    
    def _update_cap_pos_batch(self, cap_pos):
        CoM = np.mean(cap_pos, axis=0)
        cap_pos = cap_pos - CoM
        self.cap_pos_batch.append(cap_pos)
        pass

    def _get_cap_vel(self):
        vel_list = []
        for i in range(18):
            cap_pos_batch = np.array(self.cap_pos_batch)
            coeff = np.polyfit(self.t_window, cap_pos_batch[:,i], 5)
            pos = Polynomial(coeff[::-1])
            vel = pos.deriv()
            vel_list.append(vel(0))
        return np.array(vel_list)
    
    def _get_tendon_len(self, cap_pos):
        # corresponding relationship between cap_pos and tendon_len
        tendon_len=np.ones(9)
        return tendon_len

''' for test
cp = ctrl_policy(50)
cap_pos = np.random.randn(18)
while True:
    action = cp.get_action(cap_pos)
    print(action)
'''