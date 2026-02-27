import torch
import tqdm
from torch.utils.data import Dataset

from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE, compute_num_steps


class TensegrityDataset(Dataset):

    def __init__(self,
                 raw_data_dict,
                 num_steps_fwd=1,
                 num_hist=1,
                 dt=0.01,
                 dtype=DEFAULT_DTYPE):
        self.num_steps_fwd = num_steps_fwd
        self.num_hist = num_hist
        self.dt = dt
        self.dtype = dtype

        self.processed_data = self._process_raw_data(raw_data_dict)

    def _process_raw_data(self, raw_data_dict):
        processed_data = []

        states = raw_data_dict['states']
        times = raw_data_dict['times']
        data_gt_end_pts = raw_data_dict["gt_end_pts"]

        for i in tqdm.tqdm(range(len(states)), desc='Loading torch dataset'):
            controls = torch.vstack(raw_data_dict['controls'][i])

            act_lengths = raw_data_dict['act_lengths'][i][:-self.num_steps_fwd]
            motor_omegas = raw_data_dict['motor_omegas'][i][:-self.num_steps_fwd]
            x = states[i][:-self.num_steps_fwd]

            x = [x[0].clone() for _ in range(self.num_hist - 1)] + x
            act_lengths = [act_lengths[0].clone() for _ in range(self.num_hist - 1)] + act_lengths
            motor_omegas = [motor_omegas[0].clone() for _ in range(self.num_hist - 1)] + motor_omegas

            gt_end_pts = torch.vstack([torch.hstack(d) for d in data_gt_end_pts[i]])
            next_act_lengths = torch.vstack(raw_data_dict['act_lengths'][i])

            traj_x = [torch.concat(x[j - self.num_hist + 1: j + 1], 2)
                      for j in range(self.num_hist - 1, len(x))]
            traj_act_lens = [torch.concat(act_lengths[j - self.num_hist + 1: j + 1], 2)
                             for j in range(self.num_hist - 1, len(act_lengths))]
            traj_motor_speeds = [torch.concat(motor_omegas[j - self.num_hist + 1: j + 1], 2)
                                 for j in range(self.num_hist - 1, len(motor_omegas))]

            traj_y, traj_ctrls = [], []
            traj_next_act_lens = []
            for j in range(1, self.num_steps_fwd + 1):
                end = -(self.num_steps_fwd - j) \
                    if j < self.num_steps_fwd else gt_end_pts.shape[0]
                traj_y.append(gt_end_pts[j:end])
                traj_next_act_lens.append(next_act_lengths[j:end])
                traj_ctrls.append(controls[j - 1: end])

            traj_y = torch.concat(traj_y, dim=2)
            traj_next_act_lens = torch.concat(traj_next_act_lens, dim=2)
            traj_ctrls = torch.concat(traj_ctrls, dim=2)

            traj_y = [traj_y[j: j + 1] for j in range(traj_y.shape[0])]
            traj_next_act_lens = [traj_next_act_lens[j: j + 1] for j in range(traj_next_act_lens.shape[0])]
            traj_ctrls = [traj_ctrls[j: j + 1] for j in range(traj_ctrls.shape[0])]

            traj_dt = [times[i][j + self.num_steps_fwd] - times[i][j]
                       for j in range(len(times[i]) - self.num_steps_fwd)]

            traj_elms = zip(traj_x, traj_y, traj_dt, traj_ctrls,
                            traj_act_lens, traj_next_act_lens, traj_motor_speeds)
            for x, y, dt, ctrl, act_len, next_act_lens, motor_omega in traj_elms:
                delta_t = compute_num_steps(dt, self.dt) * self.dt

                data_instance = {
                    'x': x,
                    'y': y,
                    'ctrl': ctrl,
                    'dt': torch.tensor([[[dt]]], dtype=self.dtype),
                    'act_len': act_len,
                    'next_act_lens': next_act_lens,
                    'motor_omega': motor_omega,
                    'delta_t': torch.tensor([[[delta_t]]], dtype=self.dtype),
                }
                processed_data.append(data_instance)

        return processed_data

    def __len__(self):
        return len(self.processed_data)

    def __getitem__(self, index):
        return self.processed_data[index]

    @staticmethod
    def collate_fn(data_instances):
        batch = {
            k: torch.vstack([d[k] for d in data_instances])
            for k in data_instances[0].keys()
        }

        return batch


class TensegrityMotorDataset(TensegrityDataset):

    def __init__(self,
                 raw_data_dict,
                 num_steps_fwd=1,
                 num_hist=1,
                 dt=0.01,
                 dtype=DEFAULT_DTYPE,
                 num_ctrls_hist=20):
        self.num_ctrls_hist = num_ctrls_hist
        super().__init__(raw_data_dict,
                         num_steps_fwd,
                         num_hist,
                         dt,
                         dtype)

    @staticmethod
    def append_ctrls_hist(raw_data_dict, processed_data, num_steps_fwd, num_ctrls_hist):
        k = 0
        for i in range(len(raw_data_dict['states'])):
            controls = torch.vstack(raw_data_dict['controls'][i])

            end = controls.shape[0] if num_steps_fwd == 1 else -num_steps_fwd + 1
            ctrls_hist = raw_data_dict['controls'][i][:end]
            ctrls_hist = [torch.zeros_like(ctrls_hist[0]) for _ in range(num_ctrls_hist)] + ctrls_hist

            traj_ctrls_hist = [torch.concat(ctrls_hist[j - num_ctrls_hist: j], 2)
                               if num_ctrls_hist > 0 else torch.empty(ctrls_hist[0].shape)
                               for j in range(num_ctrls_hist, len(ctrls_hist))]

            for ctrls_hist in traj_ctrls_hist:
                processed_data[k]['ctrls_hist'] = ctrls_hist
                k += 1

        return processed_data

    def _process_raw_data(self, raw_data_dict):
        processed_data = super()._process_raw_data(raw_data_dict)
        processed_data = self.append_ctrls_hist(raw_data_dict,
                                                processed_data,
                                                self.num_steps_fwd,
                                                self.num_ctrls_hist)

        return processed_data


class TensegrityMultiSimDataset(TensegrityDataset):

    @staticmethod
    def append_dataset_idx(raw_data_dict, processed_data, num_steps_fwd):
        k = 0
        for i in range(len(raw_data_dict['states'])):
            dataset_idx = raw_data_dict['dataset_idx'][i]

            for _ in range(len(raw_data_dict['states'][i]) - num_steps_fwd):
                processed_data[k]['dataset_idx'] = torch.tensor([[dataset_idx]], dtype=torch.int)
                k += 1

        return processed_data

    def _process_raw_data(self, raw_data_dict):
        processed_data = super()._process_raw_data(raw_data_dict)
        processed_data = self.append_dataset_idx(raw_data_dict, processed_data, self.num_steps_fwd)

        return processed_data


class TensegrityMultiSimMotorDataset(TensegrityDataset):

    def __init__(self,
                 raw_data_dict,
                 num_steps_fwd=1,
                 num_hist=1,
                 dt=0.01,
                 dtype=DEFAULT_DTYPE,
                 num_ctrls_hist=1):
        self.num_ctrls_hist = num_ctrls_hist
        super().__init__(raw_data_dict,
                         num_steps_fwd,
                         num_hist,
                         dt,
                         dtype)

    def _process_raw_data(self, raw_data_dict):
        processed_data = super()._process_raw_data(raw_data_dict)
        processed_data = TensegrityMultiSimDataset.append_dataset_idx(
            raw_data_dict,
            processed_data,
            self.num_steps_fwd
        )
        processed_data = TensegrityMotorDataset.append_ctrls_hist(
            raw_data_dict,
            processed_data,
            self.num_steps_fwd,
            self.num_ctrls_hist
        )

        return processed_data
