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

class ctrl_policy_vel:
    def __init__(self, fps, path_to_model="actors/actor_5800000_osn.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.obs_dim = 36
        self.act_dim = 6
        self.actor = PolicyNetwork(self.obs_dim, self.act_dim).to(self.device)
        state_dict = torch.load(path_to_model, map_location=torch.device(device=self.device), weights_only=True)
        state_dict = {k.replace("module.", ""): v for k, v in state_dict.items()}
        self.actor.load_state_dict(state_dict)

        self.dt = 1.0 / fps
        self.idle_action = np.ones(6)
        self.action_limitation = [0, 1] # actor command limitation
        self.len_limitation = [0.1, 0.2] # active tendon length limitation with unit: m
        self.vel_max = 0.1 # max velocity for actors with unit: m/s

        self.cap_pos_batch_size = 10
        self.cap_pos_batch = deque(maxlen=self.cap_pos_batch_size)
        self.last_action_state = None
        self.t_window = np.linspace(-self.dt*(self.cap_pos_batch_size-1), 0, self.cap_pos_batch_size)
        pass

    def get_action(self, cap_pos, actor_state):
        # cap_pos: numpy.array, [6, 3], positions of 6 end caps from s0 to s5
        # actor_state: numpy.array, [6,], actor position provided by motors, [0, 1] ^ 6

        self._update_cap_pos_batch(cap_pos)
        self.last_action_state = actor_state

        if len(self.cap_pos_batch) < self.cap_pos_batch_size:
            action = self.idle_action
            return action
        cap_rel_pos = self.cap_pos_batch[-1]
        cap_vel = self._get_cap_vel()
        observation = np.concatenate([cap_rel_pos, cap_vel])

        action = self._predict(observation, self.last_action_state)
        return action

    def _predict(self, obs, last_action):
        action_scaled, _ = self.actor.predict(torch.from_numpy(obs).float()) # action = vel_cmd / vel_max
        action_scaled = action_scaled.cpu().detach().numpy()
        next_action = self._action_transformer(action_scaled, last_action)
        next_action = np.clip(next_action, self.action_limitation[0], self.action_limitation[1])
        return next_action
    
    def _action_transformer(self, action, last_action):
        # transform vel_cmd to pos_cmd
        del_len = action * self.vel_max * self.dt
        del_action = del_len * (self.action_limitation[1]-self.action_limitation[0]) / (self.len_limitation[1] - self.len_limitation[0])
        next_action = last_action + del_action
        return next_action
    
    def _update_cap_pos_batch(self, cap_pos):
        CoM = np.mean(cap_pos, axis=0)
        cap_pos = cap_pos - CoM
        self.cap_pos_batch.append(cap_pos.flatten())
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