import json
from logging import Logger
from pathlib import Path
from termios import N_SLIP
import numpy as np

import torch.nn
from torch.distributions import Uniform

from gnn_simulator.model_predictive_control import mppi_utils
from gnn_simulator.model_predictive_control import util_heuristic
from gnn_simulator.simulators.abstract_simulator import AbstractSimulator
from gnn_simulator.utilities import torch_quaternion


class TensegrityMPPIPlanner(torch.nn.Module):

    def __init__(self,
                 sim: AbstractSimulator | str | Path,
                 n_samples: int,
                 horizon: float,
                 ctrl_interval: float,
                 device: str = 'cpu',
                 u_bounds: tuple = (-1., 1.),
                 gamma: float = 1.0,
                 rest_len_bounds: tuple = (0.3, 2.0),
                 goal: tuple | None = None,
                 obstacles: tuple = (),
                 boundary: tuple = (),
                 logger: Logger | None = None,
                 strategy: 'str' = 'min',
                 cost_weights: tuple = (1.0, 0.0, 0.0),
                 grid_step=0.1,
                 tol: float = 0.5,
                 min_vel_dt=0.04,
                 use_motion_prim_heuristic: bool = False,
                 torch_compile=False,
                 sim_config=None,
                 ceiling=False):
        super().__init__()
        self.logger = logger

        if isinstance(sim, str) or isinstance(sim, Path):
            self.sim = torch.load(sim, map_location='cpu', weights_only=False)
            self.sim.to(device)
        else:
            self.sim = sim
        self.sim.reset()

        if sim_config:
            print(sim_config)
            if isinstance(sim_config, str) or isinstance(sim_config, Path):
                sim_config = json.load(open(sim_config, 'r'))
            self.sim.all_env_planar_objs = self.sim._build_env_objs(sim_config['environment'])
            self.sim.curr_env_planar_objs = [obj for obj in self.sim.all_env_planar_objs]
            # env_mapping = {e.name: e for e in self.sim.all_env_planar_objs}
            # self.sim.curr_env_planar_objs = [env_mapping[k] for k in sim_config['environment'].keys()]
            self.sim.to(device)

        if torch_compile:
            self.sim.run_compile()

        self.dt = self.sim.data_processor.dt.item()

        self.min_vel_dt = min_vel_dt
        self.n_samples = n_samples
        self.horizon = round(horizon / self.dt)
        self.ctrl_interval = round(ctrl_interval / self.dt)
        self.dtype = self.sim.dtype
        self.device = device

        self.has_ceiling = ceiling

        self.goal = None
        self.set_goals([np.array(goal)])

        self.obs_cost_gain = 50.0
        self.obs_min_dist = 0.5
        self.terminal_reward = -100.0
        self.goal_threshold = tol

        assert strategy in ['min', 'weighted']
        self.strategy = strategy

        self.gamma = gamma
        self.gamma_seq = torch.cumprod(
            torch.full((1, 1, self.horizon),
                       gamma,
                       dtype=self.dtype),
            dim=-1
        )

        self.ctrl_min, self.ctrl_max = u_bounds
        self.rest_min, self.rest_max = rest_len_bounds
        self.n_ctrls = len(self.sim.robot.actuated_cables)
        self.prev_ctrls = torch.zeros(
            (1, self.n_ctrls, self.horizon // self.ctrl_interval),
            dtype=self.dtype,
            device=self.device
        )

        self.cost_weights = cost_weights

        self.boundary = boundary
        self.box_obstacles = obstacles

        self.use_motion_prim_heuristic = use_motion_prim_heuristic
        self.grid_step = grid_step

        if self.use_motion_prim_heuristic:
            gaits = [
                (-0.27132362127304077, -1.126994013786316, -0.4933106303215027),
                (0.5869942903518677, 0.699185848236084, 0.26279446482658386),
                (-0.12006855010986328, -1.7110869884490967, -0.21380454301834106),
                (-0.016680419445037842, -1.8212430477142334, -0.21363916993141174),
                (0.2103751301765442, -1.8690084218978882, -0.22674056887626648),
                (-0.1999402493238449, -1.7540470361709595, -0.18494027853012085),
                (-0.04838576167821884, -1.8556764125823975, -0.17369386553764343),
                (0.15124797821044922, -1.9940990209579468, -0.21512864530086517),
                (-0.25401318073272705, -1.7648557424545288, -0.22380636632442474),
                (-0.0854407325387001, -1.9190342426300049, -0.18813318014144897),
                (0.14104518294334412, -2.0836660861968994, -0.1659887731075287)
            ]
            self.dist_cost_grid, self.obs_cost_grid = mppi_utils.wave_heuristic_dict_to_arr_se2(
                goal,
                gaits,
                boundary,
                obstacles,
                self.grid_step,
                self.device,
                self.dtype
            )
        else:
            self.dist_cost_grid, self.obs_cost_grid = mppi_utils.wave_heuristic_dict_to_arr(
                goal,
                boundary,
                obstacles,
                grid_step,
                self.device,
                self.dtype
            )

    def to(self, device):
        self.device = device
        self.dist_cost_grid = self.dist_cost_grid.to(device)
        self.obs_cost_grid = self.obs_cost_grid.to(device)
        self.gamma_seq = self.gamma_seq.to(device)
        self.sim = self.sim.to(device)
        self.prev_ctrls = self.prev_ctrls.to(device)
        self.goal = self.goal.to(device)

        return self

    @staticmethod
    def _compute_weight(cost, beta, factor):
        return torch.exp(-factor * (cost - beta))

    def map(self, data):
        if isinstance(data, np.ndarray):
            data = torch.from_numpy(data)
        elif isinstance(data, list):
            if isinstance(data[0], np.ndarray):
                data = np.vstack(data)
                data = torch.from_numpy(data)
            else:
                data = torch.tensor(data)

        data = data.to(self.sim.device).to(self.sim.dtype)
        return data

    @staticmethod
    def endpts_to_5d_pose(end_pts):
        end_pts_ = end_pts.reshape(-1, 6, 1)
        curr_pos = (end_pts_[:, :3] + end_pts_[:, 3:]) / 2.
        prin = (end_pts_[:, 3:] - end_pts_[:, :3])
        curr_quat = torch_quaternion.compute_quat_btwn_z_and_vec(prin)

        return torch.hstack([curr_pos, curr_quat])

    def rollout(self, curr_state_, batch_actions):
        states = [curr_state_.clone()]
        state_to_graph_kwargs = {'dataset_idx': torch.tensor([8]).repeat(curr_state_.shape[0]).reshape(-1, 1)}
        sim_states, _, _ = self.sim.run(
            curr_state_,
            ctrls=batch_actions,
            state_to_graph_kwargs=state_to_graph_kwargs,  # HACK
            show_progress=True
        )

        costs, dist_costs, dir_costs, obs_costs = [], [], [], []
        # curr_dir = self.get_goal_dir(curr_state_.reshape(-1, 13, 1)[:, :7])
        curr_dir = torch.tensor([[0.0, -1.]], dtype=self.dtype, device=self.device)
        for state in sim_states:
            dist_cost, dir_cost, obs_cost = self.all_costs(state, curr_dir)
            cost = (self.cost_weights[0] * dist_cost
                    + self.cost_weights[1] * dir_cost
                    + self.cost_weights[2] * obs_cost)

            costs.append(cost)
            dist_costs.append(self.cost_weights[0] * dist_cost.detach())
            dir_costs.append(self.cost_weights[1] * dir_cost.detach())
            obs_costs.append(self.cost_weights[2] * obs_cost.detach())

        states.extend(sim_states)
        states = torch.concat(states, dim=-1)

        return states, (costs, dist_costs, dir_costs, obs_costs)

    def set_goals(self, com_goal):
        self.goal = self.map(com_goal)
        self.goal[:, 2] = 0.

    def compute_angle(self, curr_dir):
        x = torch.zeros_like(curr_dir)
        x[:, 0] = 1.
        return self.rel_2d_angle(curr_dir, x)

    def heuristic_grid_cost(self, curr_state):
        curr_pose = self.map(curr_state).reshape(-1, 13, 1)[:, :7].reshape(curr_state.shape[0], 3, 7)
        com = curr_pose[..., :3].mean(dim=1).unsqueeze(-1)

        dist_cost = torch.zeros_like(com[:, 0, 0])

        if self.use_motion_prim_heuristic:
            quat = curr_pose.transpose(1, 2)[:, 3:7]
            curr_dir = torch_quaternion.compute_prin_axis(quat).mean(dim=-1, keepdim=True)[:, :2]
            curr_dir = curr_dir / curr_dir.norm(dim=1, keepdim=True)
            angle = self.compute_angle(curr_dir).reshape(com[:, :1].shape)

            se2 = torch.hstack([com[:, :2], angle])
            idx = mppi_utils.snap_to_grid_torch_se2(se2, self.grid_step, self.boundary).squeeze(-1)

            in_grid = torch.logical_and(
                torch.logical_and(0 <= idx[:, 0], idx[:, 0] < self.dist_cost_grid.shape[0]),
                torch.logical_and(0 <= idx[:, 1], idx[:, 1] < self.dist_cost_grid.shape[1])
            )
            in_grid = torch.logical_and(
                in_grid,
                torch.logical_and(0 <= idx[:, 2], idx[:, 2] < self.dist_cost_grid.shape[2])
            )
            dist_cost[in_grid] = self.dist_cost_grid[idx[in_grid, 0], idx[in_grid, 1], idx[in_grid, 2]]
        else:
            idx = mppi_utils.snap_to_grid_torch(com[:, :2], self.grid_step, self.boundary).squeeze(-1)

            in_grid = torch.logical_and(
                torch.logical_and(0 <= idx[:, 0], idx[:, 0] < self.dist_cost_grid.shape[0]),
                torch.logical_and(0 <= idx[:, 1], idx[:, 1] < self.dist_cost_grid.shape[1])
            )
            dist_cost[in_grid] = self.dist_cost_grid[idx[in_grid, 0], idx[in_grid, 1]]

        dist_cost[~in_grid] = (com[~in_grid, :2, 0] - self.goal[:, :2]).norm(dim=1)

        if self.has_ceiling:
            end_pts = self.compute_end_pts(curr_state).reshape(-1, 6, 3)
            # under_ceiling_x = torch.logical_and(end_pts[..., 0] > 3.5, end_pts[..., 0] < 4.25) # floating ceiling
            under_ceiling_x = torch.logical_and(end_pts[..., 0] > 16, end_pts[..., 0] < 20) # 3d obs course
            under_ceiling_y = torch.logical_and(end_pts[..., 1] > 0.0, end_pts[..., 1] < 0.75)
            under_ceiling = torch.logical_and(under_ceiling_x, under_ceiling_y)
            z = (end_pts[..., 2] * under_ceiling)
            obs_cost = (10 / (2.25 - z).clamp_min(1e-5)).sum(dim=1) - (6 * 10 / 2.25)
        else:
            obs_cost = torch.zeros_like(dist_cost)

        return dist_cost, obs_cost

    def multi_goal_costs(self, curr_state):
        curr_state = self.map(curr_state)
        com = (curr_state.reshape(curr_state.shape[0], -1, 13)[..., :3]
               .mean(dim=1).unsqueeze(-1))

        com_vecs = com[:, :2] - self.goals[:, :2, :-1]
        com_vecs2 = com[:, :2] - self.goals[:, :2, 1:]

        avg_dists = 0.5 * (com_vecs.norm(dim=1) + com_vecs2.norm(dim=1)).unsqueeze(1)
        min_goal_idx = avg_dists.argmin(dim=-1).flatten()
        goal = self.goals.repeat(com.shape[0], 1, 1)[torch.arange(com.shape[0]), :, min_goal_idx].unsqueeze(-1)
        goal_vec = self.goal_vecs.repeat(com.shape[0], 1, 1)[torch.arange(com.shape[0]), :, min_goal_idx].unsqueeze(-1)
        goal_vec_norm = self.goal_vecs_norm.repeat(com.shape[0], 1, 1)[torch.arange(com.shape[0]), :,
                        min_goal_idx].unsqueeze(-1)
        com_vec = com_vecs[torch.arange(com.shape[0]), :, min_goal_idx].unsqueeze(-1)

        t = torch.linalg.vecdot(com_vec, goal_vec, dim=1).unsqueeze(1)
        d = goal[:, 2:] - self.cost_slope * t
        d_lim = self.goals[0, 2:, min_goal_idx + 1].T.unsqueeze(-1)
        d = torch.clamp_min(d, d_lim)
        v = com_vec - t * goal_vec / (goal_vec_norm ** 2)
        v_norm = v.norm(dim=1, keepdim=True)

        off_traj_costs = (self.off_traj_cost * v_norm + 1) ** 2
        ramp_costs = (self.off_traj_cost * v_norm + 1) ** 2 + d
        # ramp_costs = ramp_costs.min(dim=-1, keepdim=True).values

        final_goal_dist_cost = (com[:, :2] - self.goals[:, :2, -1:]).norm(dim=1, keepdim=True)
        # costs = torch.maximum(ramp_costs, final_goal_dist_cost)
        costs = ramp_costs + final_goal_dist_cost
        # if ramp_costs.min(dim=0).values.item() < final_goal_dist_cost.

        return costs, torch.zeros_like(costs)[:, :1]

    def compute_end_pts(self, curr_state):
        return self.sim.robot.compute_end_pts(curr_state)

    @staticmethod
    def _dist2d_pt_to_line_seg(pt, line_seg):
        pt = pt.reshape(pt.shape[0], pt.shape[1], 1)
        v = line_seg[1] - line_seg[0]
        length = v.norm(dim=1, keepdim=True)

        rel_pt = pt - line_seg[0]
        proj = (v * rel_pt).sum(dim=1, keepdim=True) / length
        proj = torch.clamp(proj, 0.0, 1.0)

        closest_pt = line_seg[0] + proj * v
        dist = (closest_pt - pt).norm(dim=1, keepdim=True)

        return dist

    def box_obstacle_costs(self, curr_state):
        """
        end_pts: (batch_size, 3, num_end_pts)
        """
        curr_state = self.map(curr_state)

        if len(self.box_obstacles) == 0:
            return torch.zeros_like(curr_state[:, :1]).flatten()

        end_pts = self.compute_end_pts(curr_state)[:, :2]
        n_end_pts = end_pts.shape[-1]
        end_pts = end_pts.transpose(1, 2).reshape(-1, 2)

        # sphere_r = 0.175
        sphere_r = self.sim.robot.sphere_radius

        # costs = torch.zeros_like(curr_state[:, :1])

        lines_0 = torch.tensor(
            [
                p
                for xmin, xmax, ymin, ymax in self.box_obstacles
                for p in [[xmin, ymin], [xmin, ymin], [xmax, ymax], [xmax, ymax]]
            ], dtype=self.dtype, device=self.device
        ).T.unsqueeze(0)

        lines_1 = torch.tensor(
            [
                p
                for xmin, xmax, ymin, ymax in self.box_obstacles
                for p in [[xmin, ymax], [xmax, ymin], [xmin, ymax], [xmax, ymin]]
            ], dtype=self.dtype, device=self.device
        ).T.unsqueeze(0)

        dists = self._dist2d_pt_to_line_seg(end_pts, (lines_0, lines_1))
        dists = dists.min(dim=2).values.reshape(-1, n_end_pts).min(dim=1).values

        dists = torch.clamp_min(dists - sphere_r - self.obs_min_dist, 1e-8)
        costs = self.obs_cost_gain / (dists ** 0.5)

        return costs.flatten()

    def dist_costs(self, curr_state):
        curr_state = self.map(curr_state)
        com = (curr_state.reshape(curr_state.shape[0], -1, 13)[..., :3]
               .mean(dim=1).unsqueeze(-1))

        goal_dir = self.goals[:, :2] - com[:, :2]
        dist_costs = goal_dir.norm(dim=1, keepdim=True)
        dist_costs = dist_costs ** 2

        return dist_costs.flatten()

    def multi_goal_dists_cost(self, curr_state):
        curr_state = self.map(curr_state)
        com = (curr_state.reshape(curr_state.shape[0], -1, 13)[..., :3]
               .mean(dim=1).unsqueeze(-1))

        goal_dir = com[:, :2] - self.goals[:, :2]
        dists = goal_dir.norm(dim=1, keepdim=True)

        final_dist = dists[..., -1:]

        total_dist = final_dist

        goal_dir = goal_dir[..., -1:] / final_dist
        prin = torch_quaternion.quat_as_rot_mat(
            curr_state.reshape(-1, 13, 1)[:, 3:7]
        )[..., -1:].reshape(curr_state.shape[0], -1, 3)
        prin = prin.mean(dim=1).unsqueeze(-1)
        prin[:, 2] = 0.0
        prin = prin / prin.norm(dim=1, keepdim=True)

        z = torch.zeros_like(prin)
        z[:, 2] = 1.

        y = torch.cross(z, prin, dim=1)
        y = y / y.norm(dim=1, keepdim=True)

        dir_costs = -torch.linalg.vecdot(goal_dir, y[:, :2], dim=1)

        return total_dist.squeeze(-1), dir_costs

    def dir_cost(self, curr_state, goal_dir):
        if goal_dir is None:
            return torch.zeros_like(curr_state[:, :1]).flatten()

        q = curr_state.reshape(-1, 13, 1)[:, 3:7]
        prin = torch_quaternion.compute_prin_axis(q).reshape(curr_state.shape[0], -1, 3).mean(dim=1)[:, :2]
        prin = prin / prin.norm(dim=1, keepdim=True)
        heading = -torch.hstack([-prin[:, 1:], prin[:, :1]])

        angle = self.rel_2d_angle(heading, goal_dir)
        dir_cost = torch.exp(8 * angle) - 1.

        return dir_cost.flatten()

    def terminal_cost(self, curr_state):
        xy_com = curr_state.reshape(curr_state.shape[0], -1, 13)[..., :2].mean(dim=1)
        goal = self.goal[:, :2]
        dist = (goal - xy_com).norm(dim=1)
        close = dist < self.goal_threshold
        terminal = torch.full_like(dist, self.terminal_reward) * close

        return terminal.flatten()

    def all_costs(self, curr_state, curr_dir=None):
        dist_cost, obs_cost = self.heuristic_grid_cost(curr_state)
        # _dist_cost, dir_cost = self._dist_costs(curr_state)
        # _obs_cost = self.box_obstacle_costs(curr_state)
        dir_cost = self.dir_cost(curr_state, curr_dir)
        terminal_cost = self.terminal_cost(curr_state)
        dist_cost = dist_cost + terminal_cost

        return dist_cost, dir_cost, obs_cost

    def reset_sim_pose(self, curr_pose_time, prev_pose_time, rest_lengths=None, motor_speeds=None, batch_size=1):
        rest_lengths = self.map(rest_lengths).unsqueeze(0)
        motor_speeds = self.map(motor_speeds).unsqueeze(0)

        curr_pose, curr_timestamp = curr_pose_time
        curr_pose = self.map(curr_pose).reshape(-1, 7, 1)
        curr_pos, curr_quat = curr_pose[:, :3], curr_pose[:, 3:7]

        if prev_pose_time and (curr_timestamp - prev_pose_time[1]) > 1e-6:
            prev_pose, prev_timestamp = prev_pose_time
            prev_pose = self.map(prev_pose).reshape(-1, 7, 1)
            prev_pos, prev_quat = prev_pose[:, :3], prev_pose[:, 3:7]

            dt = curr_timestamp - prev_timestamp

            lin_vel = (curr_pos - prev_pos) / dt
            ang_vel = torch_quaternion.compute_ang_vel_quat(prev_quat, curr_quat, dt)
        else:
            lin_vel = torch.zeros_like(curr_pose[:, :3])
            ang_vel = torch.zeros_like(curr_pose[:, :3])

        vels = torch.hstack([lin_vel, ang_vel])

        curr_state = torch.hstack([curr_pose, vels]).reshape(1, -1, 1)
        self.reset_sim_state(curr_state, motor_speeds, rest_lengths, batch_size)

    def reset_sim_state(self, curr_state, motor_speeds, rest_lengths, batch_size=1):
        self.sim.reset()

        curr_state = self.map(curr_state).reshape(1, -1, 1)
        if rest_lengths is not None:
            rest_lengths = self.map(rest_lengths).reshape(1, -1, 1)
        if motor_speeds is not None:
            motor_speeds = self.map(motor_speeds).reshape(1, -1, 1)

        curr_state = curr_state.repeat(batch_size, 1, 1)
        self.sim.update_state(curr_state)
        self.reset_cables(rest_lengths, motor_speeds, batch_size)

    def reset_cables(self, rest_lengths, motor_speeds, batch_size):
        cables = self.sim.robot.actuated_cables.values()
        for i, c in enumerate(cables):
            if rest_lengths is not None:
                c.set_rest_length(rest_lengths[:, i: i + 1].repeat(batch_size, 1, 1))
            if motor_speeds is not None:
                c.motor.motor_state.omega_t = motor_speeds[:, i: i + 1].repeat(batch_size, 1, 1)

    def plan(self, prev_n_pose_time_tups, rest_lens, motor_speeds):
        curr_pose, curr_timestamp = prev_n_pose_time_tups[-1]
        prev_pose_timestep = None
        for prev_pose_timestep in prev_n_pose_time_tups[:-1][::-1]:
            prev_pose, prev_timestamp = prev_pose_timestep
            if (curr_timestamp - prev_timestamp) >= self.min_vel_dt:
                break

        self.reset_sim_pose(prev_n_pose_time_tups[-1], prev_pose_timestep, rest_lens, motor_speeds)

        curr_state = self.sim.get_curr_state()

        com = curr_state.reshape(-1, 13)[:, :3].mean(dim=0)
        under_ceiling_x = torch.logical_and(com[0] > 16,  com[0] < 20)
        under_ceiling_y = torch.logical_and(com[1] > -0.5, com[1] < 1.5)
        under_ceiling = torch.logical_and(under_ceiling_x, under_ceiling_y)

        n_samples = 500 if under_ceiling.any() else self.n_samples

        mode = 'simple'
        if mode == 'simple':
            mppi_func, nsamples = self.mppi_simple, n_samples
        elif mode == 'perturb':
            mppi_func, nsamples = self.mppi_perturb, n_samples
        elif mode == 'target_lengths':
            mppi_func, nsamples = self.mppi_target_lengths, n_samples

        min_actions, min_act_states, batch_states = mppi_func(
            curr_state,
            rest_lens,
            motor_speeds,
            nsamples
        )
        # self.U = min_actions[..., ::self.ctrl_interval].clone().to(self.device)

        return min_actions, min_act_states, batch_states

    def compute_ctrl_lims(self, rest_lens, motor_speeds, curr_state):
        end_pts = self.compute_end_pts(curr_state).transpose(0, 2)
        cable_lens = []
        for i, c in enumerate(self.sim.robot.actuated_cables.values()):
            idx0, idx1 = c.end_pts[0][2], c.end_pts[1][2]
            end_pt0, end_pt1 = end_pts[int(idx0)].unsqueeze(0), end_pts[int(idx1)].unsqueeze(0)
            length = (end_pt1 - end_pt0).norm(dim=1, keepdim=True)
            cable_lens.append(length)

        cable_lens = torch.vstack(cable_lens).flatten()
        upper = (cable_lens >= self.cable_len_min).to(self.dtype).flatten()
        lower = -(cable_lens <= self.cable_len_max).to(self.dtype).flatten()

        # rest_lens = self.map(rest_lens)
        # upper_rest = (rest_lens >= self.rest_min).to(self.dtype).flatten()
        # lower_rest = -(rest_lens <= self.rest_max).to(self.dtype).flatten()

        # upper = upper_cable * upper_rest
        # lower = lower_cable * lower_rest

        return lower, upper

    def compute_ctrl_lims2(self, rest_lens, motor_speeds):
        rest_lens = rest_lens.unsqueeze(-1)
        motor_speeds = motor_speeds.unsqueeze(-1)

        cables = self.sim.robot.actuated_cables.values()
        s = torch.vstack([c.motor.speed for c in cables])
        m = torch.vstack([c.motor.max_omega for c in cables])
        # s, m = 0.8, 220 * 2 * np.pi / 60
        sm_inv = 1. / (s * m)
        # r_w = 0.035
        r_w = torch.vstack([c.winch_r for c in cables])

        alpha = 2 * sm_inv / (self.dt * r_w)

        lower = alpha * (rest_lens - self.rest_max) - motor_speeds * sm_inv
        upper = alpha * (rest_lens - self.rest_min) - motor_speeds * sm_inv

        lower = torch.clamp(lower, self.ctrl_min, self.ctrl_max - 1e-2)
        upper = torch.clamp(upper, self.ctrl_min + 1e-2, self.ctrl_max)

        # lower = torch.clamp(lower, self.u_min, 0.0).flatten()
        # upper = torch.clamp(upper, 0.0, self.u_max).flatten()

        # if (lower > -1).any() or (upper < 1.).any():
        #     s=0

        return lower.flatten(), upper.flatten()

    def mppi_simple(self, curr_state, curr_rest_lens, curr_motor_speeds, nsamples):
        lower, upper = self.compute_ctrl_lims(curr_rest_lens, curr_motor_speeds)
        # lower = -ones(6, ref_tensor=curr_state)
        # upper = ones(6, ref_tensor=curr_state)
        #
        dist = Uniform(lower, upper)
        batch_actions = dist.sample(
            (nsamples, self.horizon // self.ctrl_interval)
        ).to(self.device).to(self.dtype)
        batch_actions = (batch_actions.repeat(1, 1, self.ctrl_interval)
                         .reshape(-1, self.horizon, self.n_ctrls)
                         .transpose(1, 2))

        return self.mppi(batch_actions, curr_motor_speeds, curr_rest_lens, curr_state, nsamples)

    def mppi_perturb(self, curr_state, curr_rest_lens, curr_motor_speeds, nsamples):
        lower, upper = self.compute_ctrl_lims(curr_rest_lens, curr_motor_speeds)
        batch_act_perturb = 0.1 * torch.randn((nsamples, self.n_ctrls, self.horizon // self.ctrl_interval),
                                              dtype=self.dtype, device=self.device)
        batch_actions = torch.roll(self.prev_ctrls, -1, -1) + batch_act_perturb
        batch_actions[..., -1:] = (Uniform(lower, upper).sample((nsamples, 1))
                                   .transpose(1, 2)
                                   .to(self.device)
                                   .to(self.dtype))
        # batch_actions = torch.clamp(batch_actions, -1., 1.)
        for i in range(batch_actions.shape[1]):
            batch_actions[:, i] = torch.clamp(batch_actions[:, i], lower[i], upper[i])
        batch_actions = (batch_actions.repeat(1, self.ctrl_interval, 1)
                         .transpose(1, 2)
                         .reshape(-1, self.horizon, self.n_ctrls)
                         .transpose(1, 2))

        min_actions, min_act_states, batch_states = (
            self.mppi(batch_actions,
                      curr_motor_speeds,
                      curr_rest_lens,
                      curr_state,
                      nsamples))

        return min_actions, min_act_states, batch_states

    def mppi_target_lengths(self, curr_state, curr_rest_lens, curr_motor_speeds, nsamples,
                            target_lengths_lower=0.9, target_lengths_upper=2.2):
        """
        Sample target lengths uniformly and run simulations in batch.
        
        Args:
            curr_state: Current state tensor
            curr_rest_lens: Current rest lengths
            curr_motor_speeds: Current motor speeds
            nsamples: Number of samples to generate
            target_lengths_lower: Lower bounds for target lengths (shape: (n_ctrls,) or scalar)
            target_lengths_upper: Upper bounds for target lengths (shape: (n_ctrls,) or scalar)
        
        Returns:
            Results from run_target_length_n_steps (batched)
        """
        # Map inputs to tensors
        # target_lengths_lower = self.map(target_lengths_lower)
        # target_lengths_upper = self.map(target_lengths_upper)
        #
        # # Ensure bounds are the right shape
        # if target_lengths_lower.dim() == 0:
        #     target_lengths_lower = target_lengths_lower.expand(self.n_ctrls)
        # if target_lengths_upper.dim() == 0:
        #     target_lengths_upper = target_lengths_upper.expand(self.n_ctrls)

        # Uniformly sample target lengths: (nsamples, n_ctrls)
        dist = Uniform(target_lengths_lower, target_lengths_upper)
        batch_target_lengths = dist.sample((nsamples, self.n_ctrls, 1)).to(self.device).to(self.dtype)

        # Prepare current state for batch processing
        curr_state_ = curr_state.clone().repeat(
            (nsamples if curr_state.shape[0] == 1 else 1), 1, 1)

        # Prepare state_to_graph_kwargs for batch processing
        state_to_graph_kwargs = {
            'dataset_idx': torch.tensor([8]).repeat(nsamples).reshape(-1, 1).to(self.device)
        }

        # Batch process all samples: call run_target_length_n_steps with batched state
        # Since run_target_length_n_steps expects target_lengths of shape (n_ctrls,),
        # but we have (nsamples, n_ctrls), we need to process each sample.
        # However, we can make this efficient by using vectorized operations.
        # The function processes curr_state in batch, so we'll call it once per unique
        # target_lengths, or process all samples together if possible.

        # For true batch processing with different target_lengths per sample,
        # we process all samples in a single batched call by passing the batched state
        # and handling target_lengths appropriately. Since the function signature
        # expects (n_ctrls,), we'll pass batch_target_lengths and let the function
        # handle it, or we process efficiently.

        # Process all samples in batch: pass batched state and batched target_lengths
        # The function should handle batched curr_state, and we pass target_lengths
        # as (nsamples, n_ctrls). If the function doesn't support it natively,
        # it will need to be handled per sample, but we do it in a single call.
        states, actions = self.sim.run_target_length_n_steps(
            curr_state_,
            batch_target_lengths,  # Shape: (nsamples, n_ctrls) - pass as batched
            num_steps=self.horizon,
            state_to_graph_kwargs=state_to_graph_kwargs,
            show_progress=True
        )

        # Compute total cost for each batched state
        costs, other_costs = [], []
        curr_dir = torch.tensor([[0.0, -1.]], dtype=self.dtype, device=self.device)
        for state in states:
            dist_cost, dir_cost, obs_cost = self.all_costs(state, curr_dir)
            cost = (self.cost_weights[0] * dist_cost
                    + self.cost_weights[1] * dir_cost
                    + self.cost_weights[2] * obs_cost)
            other_costs.append((dist_cost, dir_cost, obs_cost))
            costs.append(cost)

        # Sum costs across horizon for each sample
        # costs is a list of tensors, each of shape (nsamples,)
        # Stack them to get (horizon, nsamples), then sum along horizon dimension
        costs_tensor = torch.stack(costs, dim=-1)  # Shape: (horizon, nsamples)
        total_costs = costs_tensor.sum(dim=-1)  # Shape: (nsamples,)

        # Find the index of the minimum total cost
        min_cost_idx = total_costs.argmin(dim=0)

        # Filter states and actions for the minimum cost sample (dim=0 is the batch dimension)
        # states is a list of tensors, each of shape (nsamples, ...)
        batch_states = torch.cat(states, dim=-1)
        min_act_states = batch_states[min_cost_idx:min_cost_idx + 1]
        min_actions = torch.cat([action[min_cost_idx:min_cost_idx + 1] for action in actions], dim=-1)

        self.logger.info(f"Total: {total_costs[min_cost_idx]}")

        return min_actions, min_act_states, batch_states

    def get_curr_dir(self, curr_pose):
        """
        Only valid for 3 bar tensegrity
        """
        pose = self.map(curr_pose).reshape(-1, 7, 1)
        curr_state = torch.hstack([pose, torch.zeros_like(pose[:, :6])]).reshape(1, -1, 1)
        end_pts = self.compute_end_pts(curr_state)
        mid_left = end_pts[..., ::2].mean(dim=-1)
        mid_right = end_pts[..., 1::2].mean(dim=-1)
        prin = (mid_right - mid_left)[:, :2]
        prin /= prin.norm(dim=1, keepdim=True)

        curr_dir = torch.hstack([-prin[:, 1:2], prin[:, :1]])
        return curr_dir

    def rel_2d_angle(self, curr_dir, goal_dir):
        curr_dir = self.map(curr_dir).reshape(-1, 2, 1)
        goal_dir = self.map(goal_dir).reshape(-1, 2, 1)

        cross = goal_dir[:, 0] * curr_dir[:, 1] - goal_dir[:, 1] * curr_dir[:, 0]  # 2D cross product (scalar)
        dot = goal_dir[:, 0] * curr_dir[:, 0] + goal_dir[:, 1] * curr_dir[:, 1]
        # angle = torch.acos(dot)
        angle = torch.atan2(cross, dot)

        return angle

    def get_goal_dir(self, curr_pose):
        curr_pose = self.map(curr_pose).reshape(-1, 7, 1)
        start_com = curr_pose.reshape(-1, 7, 1)[:, :2].mean(dim=0, keepdim=True)

        try:
            if self.use_motion_prim_heuristic:
                start_angle = self.compute_angle(self.get_curr_dir(curr_pose)).reshape(-1, 1, 1)
                start_se2 = torch.hstack([start_com, start_angle])
                snapped_se2 = mppi_utils.snap_to_grid_torch_se2(start_se2, self.grid_step, self.boundary)
                cost_grid = self.dist_cost_grid[..., snapped_se2[0, 2, 0]]
                best_pt = mppi_utils.unsnap_to_grid_torch(
                    mppi_utils.heuristic_dir_r2(cost_grid, snapped_se2[:, :2], 2),
                    self.grid_step[:2],
                    self.boundary
                )
            else:
                snapped_com = mppi_utils.snap_to_grid_torch(start_com, self.grid_step, self.boundary)
                best_pt = mppi_utils.unsnap_to_grid_torch(
                    mppi_utils.heuristic_dir_r2(self.dist_cost_grid, snapped_com, 6),
                    self.grid_step[:2],
                    self.boundary
                )
        except:
            print("Error in get_goal_dir, return goal as best point")
            best_pt = self.goal[:, :2].flatten().cpu().numpy().tolist()

        curr_dir = torch.hstack([best_pt[0] - start_com[:, :1], best_pt[1] - start_com[:, 1:2]])
        curr_dir = curr_dir / curr_dir.norm(dim=1, keepdim=True)
        return curr_dir

    def mppi(self, batch_actions, curr_motor_speeds, curr_rest_lens, curr_state, nsamples):
        curr_state_ = curr_state.clone().repeat(
            (nsamples if curr_state.shape[0] == 1 else 1), 1, 1)

        batch_states, all_costs = self.rollout(curr_state_, batch_actions)
        costs, other_costs = all_costs[0], all_costs[1:]

        if self.strategy == 'weighted':
            # costs = torch.stack(costs, dim=-1).cpu()
            # costs = costs.reshape(costs.shape[0], 1, -1).sum(dim=-1)
            costs = costs[-1].cpu()
            beta = torch.min(costs)
            cost_total_non_zero = self._compute_weight(costs, beta, 10.0)
            eta = torch.sum(cost_total_non_zero)
            omega = ((1. / eta) * cost_total_non_zero).reshape(-1, 1, 1)
            print(omega.max().cpu())

            # print(omega.device, batch_actions.device)
            min_actions = (omega * batch_actions.cpu()).sum(dim=0, keepdim=True)

            device = self.device
            self.to('cpu')
            self.reset_sim_state(curr_state_[:1].cpu(),
                                 curr_motor_speeds,
                                 curr_rest_lens)
            min_act_states, all_min_costs = (
                self.rollout(curr_state[:1].cpu(), min_actions.cpu()))
            min_costs, min_other_costs = all_min_costs[0], all_min_costs[1:]

            # other_cost = [(torch.vstack(c)).sum(dim=0).mean().cpu().item()
            #               for c in min_other_costs]
            # cost = (torch.vstack(min_costs)).sum(dim=0).mean().cpu().item()
            other_cost = [c[-1].mean().cpu().item()
                          for c in min_other_costs]
            cost = min_costs[-1].mean().cpu().item()
            # cost = self.weights[0] * dist_c + self.weights[1] * dir_c + self.weights[2] * obs_c

            self.to(device)
        else:
            # batch_cost = torch.stack(costs, dim=-1).sum(dim=-1)
            batch_cost = costs[-1]
            idx = batch_cost.argmin(dim=0)

            min_actions = batch_actions[idx: idx + 1]
            min_act_states = batch_states[idx: idx + 1]
            other_cost = [c[-1][idx].cpu().item()
                          for c in other_costs]
            cost = costs[-1][idx].cpu().item()

        self.logger.info(f"Total: {cost}, "
                         f"Other: {other_cost}")

        return min_actions, min_act_states, batch_states
