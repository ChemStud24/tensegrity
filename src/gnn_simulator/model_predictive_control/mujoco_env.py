import multiprocessing

from gnn_simulator.mujoco_physics_engine.tensegrity_mjc_simulation import *


class MJCTensegrityEnv:

    def __init__(self, xml, env_type, **env_kwargs):
        self.env = self._get_env(xml, env_type, **env_kwargs)
        self.end_pt_noise = 0.0
        self.rest_lens_noise = 0.0
        self.motor_noise = 0.0

    def _get_env(self, xml_path, env_type, **env_kwargs):
        if env_type == '3bar':
            return ThreeBarTensegrityMuJoCoSimulator(xml_path, visualize=False, **env_kwargs)
        elif env_type == '6bar':
            return SixBarTensegrityMuJoCoSimulator(xml_path, visualize=False, **env_kwargs)
        else:
            raise Exception(f'Unknown env type {env_type}, need to be one of [3bar, 6bar]')

    def sense(self):
        self.forward()
        endpt_sensor_readings = self.env.get_endpts()
        # endpt_sensor_readings += np.random.randn(*endpt_sensor_readings.shape) * self.end_pt_noise

        rest_lengths = self.env.mjc_model.tendon_lengthspring[:self.env.n_actuators, 0].copy()
        # rest_lengths += np.random.randn(*rest_lengths.shape) * self.rest_lens_noise

        motor_speeds = np.concatenate([m.motor_state.omega_t.copy() for m in self.env.cable_motors])
        # motor_speeds += np.random.randn(*motor_speeds.shape) * self.motor_noise

        return endpt_sensor_readings, rest_lengths, motor_speeds

    def sense_full(self):
        self.forward()
        endpt_sensor_readings = np.hstack([
            self.env.mjc_data.qpos.reshape(-1, 7),
            self.env.mjc_data.qvel.reshape(-1, 6)
        ]).reshape(1, -1, 1)
        # endpt_sensor_readings += np.random.randn(*endpt_sensor_readings.shape) * self.end_pt_noise

        rest_lengths = self.env.mjc_model.tendon_lengthspring[:self.env.n_actuators, 0].copy()
        # rest_lengths += np.random.randn(*rest_lengths.shape) * self.rest_lens_noise

        motor_speeds = np.concatenate([m.motor_state.omega_t for m in self.env.cable_motors]).copy()
        # motor_speeds += np.random.randn(*motor_speeds.shape) * self.motor_noise

        return endpt_sensor_readings, rest_lengths, motor_speeds

    def forward(self):
        self.env.forward()

    def step(self, ctrls):
        self.env.sim_step(ctrls)


class MJCSim:

    def __init__(self, xml, env_type, attach_type, tmp_dir=None):
        self.env = self._get_env(xml, env_type, attach_type=attach_type)
        self.dtype = torch.float64
        self.device = "cpu"
        self.robot = None
        self.dt = self.env.dt
        self.tmp_dir = tmp_dir
        if self.tmp_dir:
            self.tmp_dir = Path(tmp_dir)
            self.tmp_dir.mkdir(exist_ok=True)

    def _get_env(self, xml_path, env_type, **env_kwargs):
        if env_type == '3bar':
            return ThreeBarTensegrityMuJoCoSimulator(xml_path, visualize=False, **env_kwargs)
        elif env_type == '6bar':
            return SixBarTensegrityMuJoCoSimulator(xml_path, visualize=False, **env_kwargs)
        else:
            raise Exception(f'Unknown env type {env_type}, need to be one of [3bar, 6bar]')

    @property
    def num_rods(self):
        return self.env.num_rods

    def to(self, device):
        return self

    def reset(self):
        pass

    def get_curr_state(self):
        self.env.forward()
        curr_state = torch.from_numpy(np.hstack([
            self.env.mjc_data.qpos.reshape(-1, 7),
            self.env.mjc_data.qvel.reshape(-1, 6)
        ]).reshape(1, -1, 1)).to(self.dtype)
        return curr_state

    def reset_cables(self, rest_lengths, motor_speeds):
        self.env.mjc_model.tendon_lengthspring[:self.env.n_actuators, :1] = rest_lengths.reshape(-1, 1).copy()
        self.env.mjc_model.tendon_lengthspring[:self.env.n_actuators, 1:] = rest_lengths.reshape(-1, 1).copy()

        for k, m in enumerate(self.env.cable_motors):
            m.motor_state.omega_t = motor_speeds[0, k].copy()

    def rollout(self, start_state, actions):
        # num_threads = 1
        num_threads = 20
        nsamples = int(start_state.shape[0] / num_threads)
        if start_state.shape[0] / num_threads != nsamples:
            num_threads = 1
            nsamples = start_state.shape[0]
        envs = [deepcopy(self.env) for _ in range(num_threads)]
        queues = [multiprocessing.Queue() for _ in range(num_threads)]
        processes = [
            multiprocessing.Process(
                target=self.proc_rollout,
                args=(start_state[i * nsamples: (i + 1) * nsamples],
                      actions[i * nsamples: (i + 1) * nsamples],
                      envs[i], i, queues[i])
            ) for i in range(num_threads)
        ]
        for p in processes:
            p.start()

        for p in processes:
            p.join()

        status = [False for _ in range(num_threads)]
        while not all(status):
            for i in range(num_threads):
                status[i] = queues[i].get()

        all_states = torch.vstack([
            torch.load(Path(self.tmp_dir, f"tmp{j}.pt"))
            for j in range(num_threads)
        ])
        all_states = torch.concat([start_state, all_states], dim=-1)
        return all_states

    def proc_rollout(self, start_state, actions, env, idx, queue):
        rest_lengths = env.mjc_model.tendon_lengthspring.copy()[:self.env.n_actuators, :1]
        motor_speeds = np.array([[m.motor_state.omega_t.copy() for m in env.cable_motors]])

        start_states = [start_state[i: i + 1].cpu().clone().numpy() for i in range(start_state.shape[0])]
        all_next_states = []

        for i, s in enumerate(tqdm.tqdm(start_states)):
            curr_state = s.copy().reshape(-1, 13)
            env.mjc_data.qpos = curr_state[:, :7].flatten()
            env.mjc_data.qvel = curr_state[:, 7:].flatten()
            env.mjc_model.tendon_lengthspring[:self.env.n_actuators, :1] = rest_lengths.reshape(-1, 1).copy()
            env.mjc_model.tendon_lengthspring[:self.env.n_actuators, 1:] = rest_lengths.reshape(-1, 1).copy()

            for k, m in enumerate(env.cable_motors):
                m.motor_state.omega_t = motor_speeds[0, k].copy()

            next_states = []
            for j in range(actions.shape[2]):
                env.sim_step(actions[i, :, j])
                env.forward()
                next_state = torch.from_numpy(np.hstack([
                    env.mjc_data.qpos.reshape(-1, 7),
                    env.mjc_data.qvel.reshape(-1, 6)
                ]).reshape(1, -1, 1)).to(self.dtype)
                next_states.append(next_state)

            next_states = torch.concat(next_states, dim=-1)
            all_next_states.append(next_states)

        all_next_states = torch.vstack(all_next_states)
        torch.save(all_next_states,
                   Path(self.tmp_dir, f"tmp{idx}.pt"))
        queue.put(True)
        # return all_next_states

    def rollout2(self, start_state, actions):
        actions = actions.cpu().clone().numpy()

        rest_lengths = self.env.mjc_model.tendon_lengthspring.copy()[:self.env.n_actuators, :1]
        motor_speeds = np.array([[m.motor_state.omega_t.copy() for m in self.env.cable_motors]])

        start_states = [start_state[i: i + 1].cpu().clone().numpy() for i in range(start_state.shape[0])]
        all_next_states = []

        for i, s in enumerate(tqdm.tqdm(start_states)):
            curr_state = s.copy().reshape(-1, 13)
            self.env.mjc_data.qpos = curr_state[:, :7].flatten()
            self.env.mjc_data.qvel = curr_state[:, 7:].flatten()
            self.reset_cables(rest_lengths, motor_speeds)

            next_states = [self.get_curr_state().to(self.dtype)]
            for j in range(actions.shape[2]):
                ctrls = actions[i, :, j]
                self.env.sim_step(ctrls)
                next_state = self.get_curr_state().to(self.dtype)
                next_states.append(next_state)

            next_states = torch.concat(next_states, dim=-1)
            all_next_states.append(next_states)

        all_next_states = torch.vstack(all_next_states)
        return all_next_states

    def step(self, curr_state, dt, control_signals):
        rest_lengths = self.env.mjc_model.tendon_lengthspring.copy()
        motor_speeds = [m.motor_state.omega_t for m in self.env.cable_motors]

        next_state = curr_state.clone()

        curr_state_ = curr_state.cpu().clone().numpy().squeeze(-1)
        for i in range(curr_state_.shape[0]):
            self.env.mjc_model.tendon_lengthspring = rest_lengths

            for j, m in enumerate(self.env.cable_motors):
                m.motor_state.omega_t = motor_speeds[j]

            state = curr_state_[i].reshape(-1, 13)
            self.env.mjc_data.qpos = state[:, :7].flatten()
            self.env.mjc_data.qvel = state[:, 7:].flatten()

            self.env.sim_step(control_signals[i].cpu().clone().numpy())

            mjc_next_pose = self.env.mjc_data.qpos.reshape(-1, 7)
            mjc_next_vel = self.env.mjc_data.qvel.reshape(-1, 6)
            mjc_next_state = np.hstack([mjc_next_pose, mjc_next_vel]).flatten()

            next_state[i, :, 0] = torch.from_numpy(mjc_next_state)

        return next_state

    def update_state(self, state):
        state_ = state.clone().numpy().reshape(-1, 13)
        pose = state_[:, :7].flatten()
        vels = state_[:, 7:].flatten()

        self.env.mjc_data.qpos = pose
        self.env.mjc_data.qvel = vels
        self.env.forward()