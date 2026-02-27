import threading
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from logging import Logger
from pathlib import Path

import numpy as np
import torch
import torch.nn
from torch.distributions import Uniform

from gnn_simulator.model_predictive_control import mppi_utils
from gnn_simulator.mujoco_physics_engine.cable_motor import DCMotor
from gnn_simulator.mujoco_physics_engine.tensegrity_mjc_simulation import TensegrityMuJoCoSimulator
from gnn_simulator.utilities import torch_quaternion


class MjcMPPIPlanner(torch.nn.Module):
    """MPPI planner backed by multi-threaded MuJoCo simulators."""

    def __init__(self,
                 xml_path: Path | str,
                 n_samples: int,
                 horizon: float,
                 ctrl_interval: float,
                 n_threads: int = 20,
                 device: str = 'cpu',
                 u_bounds: tuple = (-1., 1.),
                 gamma: float = 1.0,
                 rest_len_bounds: tuple = (1.0, 1.9),
                 goal: tuple | None = None,
                 obstacles: tuple = (),
                 boundary: tuple = (),
                 logger: Logger | None = None,
                 strategy: str = 'min',
                 cost_weights: tuple = (1.0, 0.0, 0.0),
                 grid_step=0.1,
                 tol: float = 1.0,
                 min_vel_dt=0.01,
                 use_motion_prim_heuristic: bool = False,
                 cost_grids=None,
                 # MuJoCo sim kwargs
                 attach_type='real_attach',
                 num_rods=3,
                 n_actuators=6,
                 sphere_radius=0.175,
                 motor_speed=0.8,
                 winch_r=0.035,
                 scale_factor=10.0,
                 min_cable_len=0.2,
                 max_cable_len=2.7,
                 visualize: bool = False,
                 use_remote_viewer: bool = False,
                 remote_port: int = 8765,
                 overlay_callback=None,
                 remote_max_fps: int = None):
        super().__init__()
        self.logger = logger
        self.xml_path = Path(xml_path)
        self.dtype = torch.float64
        self.device = device

        # Store MuJoCo sim construction kwargs for cloning
        self._sim_kwargs = dict(
            xml_path=self.xml_path,
            visualize=False,
            attach_type=attach_type,
            num_rods=num_rods,
            n_actuators=n_actuators,
            sphere_radius=sphere_radius,
            motor_speed=motor_speed,
            winch_r=winch_r,
            scale_factor=scale_factor,
            min_len=min_cable_len,
            max_len=max_cable_len,
            use_remote_viewer=False,
        )

        # Primary simulator (used for single rollouts and state queries)
        self.sim = TensegrityMuJoCoSimulator(
            xml_path=self.xml_path,
            visualize=visualize,
            attach_type=attach_type,
            num_rods=num_rods,
            n_actuators=n_actuators,
            sphere_radius=sphere_radius,
            motor_speed=motor_speed,
            winch_r=winch_r,
            scale_factor=scale_factor,
            min_len=min_cable_len,
            max_len=max_cable_len,
            use_remote_viewer=use_remote_viewer,
            remote_port=remote_port,
            overlay_callback=overlay_callback,
            remote_max_fps=remote_max_fps,
        )

        self.dt = self.sim.dt
        self.num_rods = num_rods
        self.n_actuators = n_actuators
        self.sphere_radius = sphere_radius
        self.scale_factor = scale_factor

        self.min_vel_dt = min_vel_dt
        self.n_samples = n_samples
        self.horizon = round(horizon / self.dt)
        self.ctrl_interval = round(ctrl_interval / self.dt)

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
            torch.full((1, 1, self.horizon), gamma, dtype=self.dtype),
            dim=-1,
        )

        self.ctrl_min, self.ctrl_max = u_bounds
        self.rest_min, self.rest_max = rest_len_bounds
        self.n_ctrls = n_actuators
        self.prev_ctrls = torch.zeros(
            (1, self.n_ctrls, self.horizon // self.ctrl_interval),
            dtype=self.dtype,
            device=self.device,
        )

        self.cost_weights = cost_weights
        self.boundary = boundary
        self.box_obstacles = obstacles

        self.use_motion_prim_heuristic = use_motion_prim_heuristic
        self.grid_step = grid_step

        if cost_grids is None and self.use_motion_prim_heuristic:
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
                (0.14104518294334412, -2.0836660861968994, -0.1659887731075287),
            ]
            self.dist_cost_grid, self.obs_cost_grid = mppi_utils.wave_heuristic_dict_to_arr_se2(
                goal, gaits, boundary, obstacles, self.grid_step,
                self.device, self.dtype,
            )
        else:
            self.dist_cost_grid, self.obs_cost_grid = mppi_utils.wave_heuristic_dict_to_arr(
                goal, boundary, obstacles, self.grid_step[:2],
                self.device, self.dtype,
            )

        # Thread pool and worker simulators
        self.n_threads = n_threads
        self._worker_sims: list[TensegrityMuJoCoSimulator] = []
        for _ in range(n_threads):
            self._worker_sims.append(TensegrityMuJoCoSimulator(**self._sim_kwargs))
        self._pool = ThreadPoolExecutor(max_workers=n_threads)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def map(self, data):
        if isinstance(data, np.ndarray):
            data = torch.from_numpy(data)
        elif isinstance(data, list):
            if isinstance(data[0], np.ndarray):
                data = np.vstack(data)
                data = torch.from_numpy(data)
            else:
                data = torch.tensor(data)
        return data.to(self.device).to(self.dtype)

    def set_goals(self, com_goal):
        self.goal = self.map(com_goal)
        self.goal[:, 2] = 0.0

    # ------------------------------------------------------------------
    # State helpers (work with MuJoCo sim)
    # ------------------------------------------------------------------

    def _get_state_tensor(self, sim: TensegrityMuJoCoSimulator) -> torch.Tensor:
        """Return the current MuJoCo state as a torch tensor of shape (1, num_rods*13, 1)."""
        pose = sim.get_pose()          # (num_rods, 7)
        vels = sim.get_vels()          # (num_rods, 6)
        state = np.hstack([pose, vels])  # (num_rods, 13)
        return torch.from_numpy(state).to(self.dtype).reshape(1, -1, 1)

    def _set_sim_state(self, sim: TensegrityMuJoCoSimulator,
                       qpos: np.ndarray, qvel: np.ndarray,
                       rest_lengths: np.ndarray | None = None,
                       motor_speeds: np.ndarray | None = None):
        """Set a simulator's full state."""
        sim.mjc_data.qpos[:] = qpos.flatten()
        sim.mjc_data.qvel[:] = qvel.flatten()

        if rest_lengths is not None:
            for i in range(sim.n_actuators):
                sim.mjc_model.tendon_lengthspring[i, 0] = rest_lengths[i]
                sim.mjc_model.tendon_lengthspring[i, 1] = rest_lengths[i]

        if motor_speeds is not None:
            for i in range(sim.n_actuators):
                sim.cable_motors[i].motor_state.omega_t[:] = motor_speeds[i]

        sim.forward()

    def _snapshot_sim(self, sim: TensegrityMuJoCoSimulator):
        """Capture qpos, qvel, rest_lengths, motor_speeds from a sim."""
        qpos = sim.mjc_data.qpos.copy()
        qvel = sim.mjc_data.qvel.copy()
        rest_lengths = sim.mjc_model.tendon_lengthspring[:sim.n_actuators, 0].copy()
        motor_speeds = np.array([m.motor_state.omega_t.copy() for m in sim.cable_motors])
        return qpos, qvel, rest_lengths, motor_speeds

    # ------------------------------------------------------------------
    # Rollout helpers
    # ------------------------------------------------------------------

    def _rollout_single(self, sim: TensegrityMuJoCoSimulator,
                        qpos, qvel, rest_lengths, motor_speeds,
                        actions: np.ndarray) -> list[np.ndarray]:
        """Run a single rollout on *sim* and return a list of state arrays.

        actions: (n_ctrls, horizon) numpy array
        Returns: list of (num_rods*13,) arrays, one per timestep (length = horizon).
        """
        self._set_sim_state(sim, qpos, qvel, rest_lengths, motor_speeds)

        states = []
        for t in range(actions.shape[1]):
            ctrl = actions[:, t].reshape(1, -1)
            sim.sim_step(ctrl)
            pose = sim.get_pose()    # (num_rods, 7)
            vels = sim.get_vels()    # (num_rods, 6)
            state = np.hstack([pose, vels]).flatten()
            states.append(state)

        return states

    def _rollout_batch_threaded(self, batch_actions: torch.Tensor,
                                qpos, qvel, rest_lengths, motor_speeds):
        """Distribute *n_samples* rollouts across worker threads.

        batch_actions: (n_samples, n_ctrls, horizon) tensor
        Returns: states tensor (n_samples, num_rods*13, horizon)
        """
        n_samples = batch_actions.shape[0]
        actions_np = batch_actions.detach().cpu().numpy()

        # Split samples across workers
        chunk_size = (n_samples + self.n_threads - 1) // self.n_threads
        futures = []
        for w in range(self.n_threads):
            start = w * chunk_size
            end = min(start + chunk_size, n_samples)
            if start >= end:
                break
            sim = self._worker_sims[w]
            chunk_actions = actions_np[start:end]
            futures.append(
                self._pool.submit(
                    self._rollout_chunk, sim, qpos, qvel, rest_lengths,
                    motor_speeds, chunk_actions,
                )
            )

        # Collect results
        all_states = []
        for f in futures:
            all_states.extend(f.result())

        # all_states: list of n_samples items, each (horizon, state_dim)
        states_np = np.array(all_states)  # (n_samples, horizon, state_dim)
        states_t = torch.from_numpy(states_np).to(self.dtype).to(self.device)
        # Reshape to (n_samples, state_dim, horizon) to match GNN planner convention
        states_t = states_t.transpose(1, 2)
        return states_t

    def _rollout_chunk(self, sim: TensegrityMuJoCoSimulator,
                       qpos, qvel, rest_lengths, motor_speeds,
                       chunk_actions: np.ndarray):
        """Run multiple rollouts sequentially on one sim.

        chunk_actions: (n_chunk, n_ctrls, horizon)
        Returns: list of (horizon, state_dim) arrays
        """
        results = []
        for i in range(chunk_actions.shape[0]):
            states = self._rollout_single(
                sim, qpos, qvel, rest_lengths, motor_speeds,
                chunk_actions[i],
            )
            results.append(np.array(states))  # (horizon, state_dim)
        return results

    # ------------------------------------------------------------------
    # Rollout entry point (matches GNN planner interface)
    # ------------------------------------------------------------------

    def rollout(self, batch_actions: torch.Tensor):
        """Run batched rollouts using multi-threaded MuJoCo sims.

        batch_actions: (n_samples, n_ctrls, horizon) tensor
        Returns:
            states: (n_samples, state_dim, horizon) tensor
            costs_tuple: (costs, dist_costs, dir_costs, obs_costs)
                each list of tensors per timestep
        """
        qpos, qvel, rest_lengths, motor_speeds = self._snapshot_sim(self.sim)

        states = self._rollout_batch_threaded(
            batch_actions, qpos, qvel, rest_lengths, motor_speeds,
        )

        # Compute costs per timestep
        costs, dist_costs, dir_costs, obs_costs = [], [], [], []
        curr_dir = torch.tensor([[0.0, -1.0]], dtype=self.dtype, device=self.device)

        for t in range(states.shape[2]):
            state_t = states[:, :, t]  # (n_samples, state_dim)
            dist_cost, dir_cost, obs_cost = self.all_costs(state_t, curr_dir)
            cost = (self.cost_weights[0] * dist_cost
                    + self.cost_weights[1] * dir_cost
                    + self.cost_weights[2] * obs_cost)

            costs.append(cost)
            dist_costs.append(self.cost_weights[0] * dist_cost.detach())
            dir_costs.append(self.cost_weights[1] * dir_cost.detach())
            obs_costs.append(self.cost_weights[2] * obs_cost.detach())

        return states, (costs, dist_costs, dir_costs, obs_costs)

    # ------------------------------------------------------------------
    # Cost functions (adapted from TensegrityMPPIPlanner)
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_weight(cost, beta, factor):
        return torch.exp(-factor * (cost - beta))

    @staticmethod
    def endpts_to_5d_pose(end_pts):
        end_pts_ = end_pts.reshape(-1, 6, 1)
        curr_pos = (end_pts_[:, :3] + end_pts_[:, 3:]) / 2.0
        prin = end_pts_[:, 3:] - end_pts_[:, :3]
        curr_quat = torch_quaternion.compute_quat_btwn_z_and_vec(prin)
        return torch.hstack([curr_pos, curr_quat])

    def compute_end_pts(self, curr_state):
        """Compute end-points from state tensor.

        curr_state: (..., num_rods*13) — each rod has [pos(3) quat(4) linvel(3) angvel(3)]
        Returns end-point positions derived from rod poses.
        """
        curr_state = self.map(curr_state)
        poses = curr_state.reshape(-1, self.num_rods, 13)[..., :7]  # (B, num_rods, 7)
        pos = poses[..., :3]   # (B, num_rods, 3)
        quat = poses[..., 3:]  # (B, num_rods, 4)

        # Compute principal axis from quaternion, scale by sphere_radius to get endpoints
        # For each rod, endpoints = pos +/- half_length * axis
        # We use the quaternion to rotate the z-axis (principal axis convention)
        z_axis = torch.zeros((*quat.shape[:-1], 3, 1), dtype=self.dtype, device=self.device)
        z_axis[..., 2, :] = 1.0
        prin = torch_quaternion.rotate_vec_quat(
            quat.reshape(-1, 4, 1), z_axis.reshape(-1, 3, 1)
        ).reshape(*quat.shape[:-1], 3)

        # Read half-length from the MuJoCo model body positions
        # Approximate: use sim endpoint sensor data at initialization is complex,
        # so we use the stored endpoint names to get relative offsets.
        # For simplicity, compute from pos +/- prin*half_rod_len
        # The actual half-length can be inferred from the initial endpoint positions.
        sim = self.sim
        sim.forward()
        endpts_np = sim.get_endpts()  # (2*num_rods, 3)
        pose_np = sim.get_pose()      # (num_rods, 7)
        half_lens = []
        for r in range(self.num_rods):
            p = pose_np[r, :3]
            e0 = endpts_np[2 * r]
            half_len = np.linalg.norm(e0 - p)
            half_lens.append(half_len)
        half_lens = torch.tensor(half_lens, dtype=self.dtype, device=self.device)

        # end_pt_0 = pos - half_len * prin, end_pt_1 = pos + half_len * prin
        hl = half_lens.reshape(1, self.num_rods, 1)
        ep0 = pos - hl * prin
        ep1 = pos + hl * prin

        # Interleave: s0, s1, s2, s3, ...
        end_pts = torch.stack([ep0, ep1], dim=2).reshape(-1, 2 * self.num_rods, 3)
        # Transpose to (B, 3, 2*num_rods)
        return end_pts.transpose(1, 2)

    def heuristic_grid_cost(self, curr_state):
        curr_state = self.map(curr_state)
        batch = curr_state.shape[0]
        poses = curr_state.reshape(batch, self.num_rods, 13)[..., :7]  # (B, num_rods, 7)
        com = poses[..., :3].mean(dim=1).unsqueeze(-1)  # (B, 3, 1)

        dist_cost = torch.zeros(batch, dtype=self.dtype, device=self.device)

        if self.use_motion_prim_heuristic:
            quat = poses[..., 3:7].transpose(1, 2)  # (B, 4, num_rods)
            curr_dir = torch_quaternion.compute_prin_axis(quat).mean(dim=-1, keepdim=True)[:, :2]
            curr_dir = curr_dir / curr_dir.norm(dim=1, keepdim=True)
            angle = self.compute_angle(curr_dir).reshape(com[:, :1].shape)

            se2 = torch.hstack([com[:, :2], angle])
            idx = mppi_utils.snap_to_grid_torch_se2(se2, self.grid_step, self.boundary).squeeze(-1)

            in_grid = torch.logical_and(
                torch.logical_and(0 <= idx[:, 0], idx[:, 0] < self.dist_cost_grid.shape[0]),
                torch.logical_and(0 <= idx[:, 1], idx[:, 1] < self.dist_cost_grid.shape[1]),
            )
            in_grid = torch.logical_and(
                in_grid,
                torch.logical_and(0 <= idx[:, 2], idx[:, 2] < self.dist_cost_grid.shape[2]),
            )
            dist_cost[in_grid] = self.dist_cost_grid[idx[in_grid, 0], idx[in_grid, 1], idx[in_grid, 2]]
        else:
            idx = mppi_utils.snap_to_grid_torch(com[:, :2], self.grid_step, self.boundary).squeeze(-1)
            in_grid = torch.logical_and(
                torch.logical_and(0 <= idx[:, 0], idx[:, 0] < self.dist_cost_grid.shape[0]),
                torch.logical_and(0 <= idx[:, 1], idx[:, 1] < self.dist_cost_grid.shape[1]),
            )
            dist_cost[in_grid] = self.dist_cost_grid[idx[in_grid, 0], idx[in_grid, 1]]

        dist_cost[~in_grid] = (com[~in_grid, :2, 0] - self.goal[:, :2]).norm(dim=1)

        obs_cost = torch.zeros_like(dist_cost)
        return dist_cost, obs_cost

    def compute_angle(self, curr_dir):
        x = torch.zeros_like(curr_dir)
        x[:, 0] = 1.0
        return self.rel_2d_angle(curr_dir, x)

    def dir_cost(self, curr_state, goal_dir):
        if goal_dir is None:
            return torch.zeros(curr_state.shape[0], dtype=self.dtype, device=self.device)

        curr_state = self.map(curr_state)
        q = curr_state.reshape(-1, 13, 1)[:, 3:7]
        prin = torch_quaternion.compute_prin_axis(q).reshape(curr_state.shape[0], -1, 3).mean(dim=1)[:, :2]
        prin = prin / prin.norm(dim=1, keepdim=True)
        heading = -torch.hstack([-prin[:, 1:], prin[:, :1]])

        angle = self.rel_2d_angle(heading, goal_dir)
        dir_cost = torch.exp(8 * angle) - 1.0
        return dir_cost.flatten()

    def terminal_cost(self, curr_state):
        curr_state = self.map(curr_state)
        xy_com = curr_state.reshape(curr_state.shape[0], -1, 13)[..., :2].mean(dim=1)
        goal = self.goal[:, :2]
        dist = (goal - xy_com).norm(dim=1)
        close = dist < self.goal_threshold
        terminal = torch.full_like(dist, self.terminal_reward) * close
        return terminal.flatten()

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
        curr_state = self.map(curr_state)
        if len(self.box_obstacles) == 0:
            return torch.zeros(curr_state.shape[0], dtype=self.dtype, device=self.device)

        end_pts = self.compute_end_pts(curr_state)[:, :2]
        n_end_pts = end_pts.shape[-1]
        end_pts = end_pts.transpose(1, 2).reshape(-1, 2)

        lines_0 = torch.tensor(
            [p for xmin, xmax, ymin, ymax in self.box_obstacles
             for p in [[xmin, ymin], [xmin, ymin], [xmax, ymax], [xmax, ymax]]],
            dtype=self.dtype, device=self.device,
        ).T.unsqueeze(0)

        lines_1 = torch.tensor(
            [p for xmin, xmax, ymin, ymax in self.box_obstacles
             for p in [[xmin, ymax], [xmax, ymin], [xmin, ymax], [xmax, ymin]]],
            dtype=self.dtype, device=self.device,
        ).T.unsqueeze(0)

        dists = self._dist2d_pt_to_line_seg(end_pts, (lines_0, lines_1))
        dists = dists.min(dim=2).values.reshape(-1, n_end_pts).min(dim=1).values
        dists = torch.clamp_min(dists - self.sphere_radius - self.obs_min_dist, 1e-8)
        costs = self.obs_cost_gain / (dists ** 0.5)
        return costs.flatten()

    def all_costs(self, curr_state, curr_dir=None):
        dist_cost, obs_cost = self.heuristic_grid_cost(curr_state)
        dir_cost = self.dir_cost(curr_state, curr_dir)
        terminal_cost = self.terminal_cost(curr_state)
        dist_cost = dist_cost + terminal_cost
        return dist_cost, dir_cost, obs_cost

    # ------------------------------------------------------------------
    # Reset sim state
    # ------------------------------------------------------------------

    def reset_sim_pose(self, curr_pose_time, prev_pose_time,
                       rest_lengths=None, motor_speeds=None):
        """Set the primary sim state from pose observations."""
        curr_pose, curr_timestamp = curr_pose_time
        curr_pose = self.map(curr_pose).reshape(-1, 7).numpy()  # (num_rods, 7)
        curr_pos = curr_pose[:, :3]
        curr_quat = curr_pose[:, 3:7]

        if prev_pose_time and (curr_timestamp - prev_pose_time[1]) > 1e-6:
            prev_pose, prev_timestamp = prev_pose_time
            prev_pose = self.map(prev_pose).reshape(-1, 7).numpy()
            dt = curr_timestamp - prev_timestamp

            lin_vel = (curr_pos - prev_pose[:, :3]) / dt
            # Approximate angular velocity from quaternion difference
            prev_quat_t = torch.from_numpy(prev_pose[:, 3:7]).unsqueeze(-1)
            curr_quat_t = torch.from_numpy(curr_quat).unsqueeze(-1)
            ang_vel = torch_quaternion.compute_ang_vel_quat(
                prev_quat_t, curr_quat_t, dt
            ).squeeze(-1).numpy()
        else:
            lin_vel = np.zeros_like(curr_pos)
            ang_vel = np.zeros_like(curr_pos)

        qpos = np.hstack([curr_pos, curr_quat]).flatten()
        qvel = np.hstack([lin_vel, ang_vel]).flatten()

        rest_lens_np = None
        if rest_lengths is not None:
            rest_lens_np = np.asarray(rest_lengths, dtype=np.float64).flatten()

        motor_speeds_np = None
        if motor_speeds is not None:
            motor_speeds_np = np.asarray(motor_speeds, dtype=np.float64).flatten()

        self._set_sim_state(self.sim, qpos, qvel, rest_lens_np, motor_speeds_np)

    def reset_sim_state(self, curr_state, motor_speeds=None, rest_lengths=None):
        """Reset the primary sim from a full state tensor (num_rods x 13)."""
        curr_state = self.map(curr_state).reshape(-1, 13).numpy()
        qpos = curr_state[:, :7].flatten()
        qvel = curr_state[:, 7:].flatten()

        rest_lens_np = None
        if rest_lengths is not None:
            rest_lens_np = np.asarray(rest_lengths, dtype=np.float64).flatten()

        motor_speeds_np = None
        if motor_speeds is not None:
            motor_speeds_np = np.asarray(motor_speeds, dtype=np.float64).flatten()

        self._set_sim_state(self.sim, qpos, qvel, rest_lens_np, motor_speeds_np)

    # ------------------------------------------------------------------
    # Control limits
    # ------------------------------------------------------------------

    def compute_ctrl_lims(self, rest_lens, motor_speeds):
        rest_lens = self.map(rest_lens)
        upper = (rest_lens >= self.rest_min).to(self.dtype).flatten()
        lower = -(rest_lens <= self.rest_max).to(self.dtype).flatten()
        return lower, upper

    def compute_ctrl_lims2(self, rest_lens, motor_speeds):
        rest_lens = self.map(rest_lens).flatten()
        motor_speeds = self.map(motor_speeds).flatten()

        s = torch.tensor([m.speed for m in self.sim.cable_motors], dtype=self.dtype)
        m = torch.tensor([m.max_omega for m in self.sim.cable_motors], dtype=self.dtype)
        sm_inv = 1.0 / (s * m)
        r_w = torch.tensor(self.sim.winch_r, dtype=self.dtype)

        alpha = 2 * sm_inv / (self.dt * r_w)

        lower = alpha * (rest_lens - self.rest_max) - motor_speeds * sm_inv
        upper = alpha * (rest_lens - self.rest_min) - motor_speeds * sm_inv

        lower = torch.clamp(lower, self.ctrl_min, self.ctrl_max - 1e-2)
        upper = torch.clamp(upper, self.ctrl_min + 1e-2, self.ctrl_max)

        return lower.flatten(), upper.flatten()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan(self, prev_n_pose_time_tups, rest_lens, motor_speeds):
        curr_pose, curr_timestamp = prev_n_pose_time_tups[-1]
        prev_pose_timestep = None
        for prev_pose_timestep in prev_n_pose_time_tups[:-1][::-1]:
            prev_pose, prev_timestamp = prev_pose_timestep
            if (curr_timestamp - prev_timestamp) >= self.min_vel_dt:
                break

        self.reset_sim_pose(prev_n_pose_time_tups[-1], prev_pose_timestep,
                            rest_lens, motor_speeds)

        curr_state = self._get_state_tensor(self.sim)

        min_actions, min_act_states, batch_states = self.mppi_simple(
            curr_state, rest_lens, motor_speeds, self.n_samples,
        )

        return min_actions, min_act_states, batch_states

    def mppi_simple(self, curr_state, curr_rest_lens, curr_motor_speeds, nsamples):
        lower, upper = self.compute_ctrl_lims(curr_rest_lens, curr_motor_speeds)

        dist = Uniform(lower, upper)
        batch_actions = dist.sample(
            (nsamples, self.horizon // self.ctrl_interval)
        ).to(self.device).to(self.dtype)
        batch_actions = (
            batch_actions.repeat(1, 1, self.ctrl_interval)
            .reshape(-1, self.horizon, self.n_ctrls)
            .transpose(1, 2)
        )

        return self.mppi(batch_actions, curr_motor_speeds, curr_rest_lens,
                         curr_state, nsamples)

    def mppi_perturb(self, curr_state, curr_rest_lens, curr_motor_speeds, nsamples):
        lower, upper = self.compute_ctrl_lims(curr_rest_lens, curr_motor_speeds)
        batch_act_perturb = 0.1 * torch.randn(
            (nsamples, self.n_ctrls, self.horizon // self.ctrl_interval),
            dtype=self.dtype, device=self.device,
        )
        batch_actions = torch.roll(self.prev_ctrls, -1, -1) + batch_act_perturb
        batch_actions[..., -1:] = (
            Uniform(lower, upper)
            .sample((nsamples, 1))
            .transpose(1, 2)
            .to(self.device)
            .to(self.dtype)
        )
        for i in range(batch_actions.shape[1]):
            batch_actions[:, i] = torch.clamp(batch_actions[:, i], lower[i], upper[i])
        batch_actions = (
            batch_actions.repeat(1, self.ctrl_interval, 1)
            .transpose(1, 2)
            .reshape(-1, self.horizon, self.n_ctrls)
            .transpose(1, 2)
        )

        return self.mppi(batch_actions, curr_motor_speeds, curr_rest_lens,
                         curr_state, nsamples)

    def mppi(self, batch_actions, curr_motor_speeds, curr_rest_lens,
             curr_state, nsamples):
        batch_states, all_costs = self.rollout(batch_actions)
        costs, other_costs = all_costs[0], all_costs[1:]

        if self.strategy == 'weighted':
            costs = torch.stack(costs, dim=-1)
            costs = costs.reshape(costs.shape[0], 1, -1).sum(dim=-1)
            beta = torch.min(costs)
            cost_total_non_zero = self._compute_weight(costs, beta, 0.01)
            eta = torch.sum(cost_total_non_zero)
            omega = ((1.0 / eta) * cost_total_non_zero).reshape(-1, 1, 1)

            min_actions = (omega * batch_actions).sum(dim=0, keepdim=True)

            # Re-rollout the weighted-average action to get its trajectory
            self.reset_sim_state(curr_state, curr_motor_speeds, curr_rest_lens)
            qpos, qvel, rl, ms = self._snapshot_sim(self.sim)
            min_states = self._rollout_single(
                self.sim, qpos, qvel, rl, ms,
                min_actions[0].detach().cpu().numpy(),
            )
            min_act_states = torch.from_numpy(np.array(min_states)).to(self.dtype).to(self.device)
            min_act_states = min_act_states.T.unsqueeze(0)  # (1, state_dim, horizon)

            other_cost = [torch.vstack(c).sum(dim=0).mean().cpu().item() for c in other_costs]
            cost = torch.vstack(costs).sum(dim=0).mean().cpu().item()
        else:
            batch_cost = costs[-1]
            idx = batch_cost.argmin(dim=0)

            min_actions = batch_actions[idx: idx + 1]
            min_act_states = batch_states[idx: idx + 1]
            other_cost = [c[-1][idx].cpu().item() for c in other_costs]
            cost = costs[-1][idx].cpu().item()

        if self.logger:
            self.logger.info(f"Total: {cost}, Other: {other_cost}")

        return min_actions, min_act_states, batch_states

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def rel_2d_angle(self, curr_dir, goal_dir):
        curr_dir = self.map(curr_dir).reshape(-1, 2, 1)
        goal_dir = self.map(goal_dir).reshape(-1, 2, 1)
        cross = goal_dir[:, 0] * curr_dir[:, 1] - goal_dir[:, 1] * curr_dir[:, 0]
        dot = goal_dir[:, 0] * curr_dir[:, 0] + goal_dir[:, 1] * curr_dir[:, 1]
        angle = torch.atan2(cross, dot)
        return angle

    def get_curr_dir(self, curr_pose):
        curr_pose = self.map(curr_pose).reshape(-1, self.num_rods, 7).transpose(1, 2)
        prin = torch_quaternion.compute_prin_axis(curr_pose[:, 3:7]).mean(dim=2, keepdim=True)
        prin /= prin.norm(dim=1, keepdim=True)
        curr_dir = torch.hstack([-prin[:, 1:2], prin[:, :1]])
        return curr_dir

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
                    self.grid_step[:2], self.boundary,
                )
            else:
                snapped_com = mppi_utils.snap_to_grid_torch(start_com, self.grid_step, self.boundary)
                best_pt = mppi_utils.unsnap_to_grid_torch(
                    mppi_utils.heuristic_dir_r2(self.dist_cost_grid, snapped_com, 2),
                    self.grid_step[:2], self.boundary,
                )
        except Exception:
            best_pt = self.goal[:, :2].flatten().cpu().numpy().tolist()

        curr_dir = torch.hstack([best_pt[0] - start_com[:, :1], best_pt[1] - start_com[:, 1:2]])
        curr_dir = curr_dir / curr_dir.norm(dim=1, keepdim=True)
        return curr_dir

    # ------------------------------------------------------------------
    # Convenience accessors matching GNN planner interface
    # ------------------------------------------------------------------

    def get_se2(self):
        """Get (com_xy, heading_angle) from the primary sim."""
        return self.sim.get_se2()

    def get_rest_lengths(self):
        return self.sim.get_rest_lengths()

    def get_motor_speeds(self):
        return self.sim.get_motor_speeds()
