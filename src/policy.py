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
    def __init__(self, fps, path_to_model="actors/actor_9900000_wpik4af.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.obs_dim = 38 # 6*3 end cap positions + 6*3 end cap velocities + 1*2 target point relative position
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
        self.last_action = np.ones(6) * (self.action_limitation[0] + self.action_limitation[1]) / 2

        self.iniyaw_bias = -np.pi/15
        self.target_distance = 1.0
        self.oript = None
        self.iniyaw = None
        self.target_pt = None
        self._use_lp_filter = True
        self._determined_action = True

        self.cap_pos_batch_size = 10
        self.cap_pos_batch = deque(maxlen=self.cap_pos_batch_size)
        self.last_action_state = None
        self.t_window = np.linspace(-self.dt*(self.cap_pos_batch_size-1), 0, self.cap_pos_batch_size)
        pass
    
    def reset_target_point(self, init_cap_pos):
        # init_cap_pos: numpy.array, [6, 3], positions of 6 end caps from s0 to s5
        if self.oript is not None:
            return
        left_CoM = (init_cap_pos[0] + init_cap_pos[2] + init_cap_pos[4]) / 3
        right_CoM = (init_cap_pos[1] + init_cap_pos[3] + init_cap_pos[5]) / 3
        self.oript = (left_CoM[:2] + right_CoM[:2]) / 2
        self.iniyaw = np.arctan2(right_CoM[0] - left_CoM[0], left_CoM[1] - right_CoM[1]) + self.iniyaw_bias
        self.iniyaw = np.arctan2(np.sin(self.iniyaw), np.cos(self.iniyaw))
        self.target_pt = np.array([self.oript[0] + self.target_distance * np.cos(self.iniyaw), self.oript[1] + self.target_distance * np.sin(self.iniyaw)])

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

        CoM = np.mean(cap_pos, axis=0)
        target_vec = self.target_pt - CoM[:2]
        target_vec_norm = np.linalg.norm(target_vec)
        if target_vec_norm > 1.0:
            target_vec = target_vec / target_vec_norm

        observation = np.concatenate([cap_rel_pos, cap_vel, target_vec])

        action = self._predict(observation)
        return action

    def _predict(self, obs):
        if self._determined_action:
            action_scaled, _ = self.actor.forward(torch.from_numpy(obs).float())
            action_scaled = torch.tanh(action_scaled)
        else:
            action_scaled, _ = self.actor.predict(torch.from_numpy(obs).float()) # action = vel_cmd / vel_max
        action_scaled = action_scaled.cpu().detach().numpy()

        action = action_scaled * (self.action_limitation[1] - self.action_limitation[0]) / 2 + (self.action_limitation[1] + self.action_limitation[0]) / 2
        action = np.clip(action, self.action_limitation[0], self.action_limitation[1])
        if self._use_lp_filter:
            next_action = self._action_lp_filter(action)
        else:
            next_action = action
        return next_action
    
    def _action_lp_filter(self, action):
        k_FILTER = 1.0
        del_action = k_FILTER*(action - self.last_action)*self.dt
        next_action = self.last_action + del_action
        self.last_action = next_action
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