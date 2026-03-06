import random

import torch

from gnn_simulator.nn_training.datasets.tensegrity_dataset import TensegrityDataset
from gnn_simulator.utilities import misc_utils
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE


class RealMultiSimTensegrityDataset(TensegrityDataset):

    def __init__(self,
                 raw_real_data_dict,
                 raw_sim_data_dict,
                 min_dt,
                 max_dt,
                 target_dt,
                 mix_ratio,
                 num_steps_fwd=1,
                 num_hist=1,
                 num_ctrls_hist=20,
                 dt=0.01,
                 dtype=DEFAULT_DTYPE):
        self.min_dt = min_dt
        self.max_dt = max_dt
        self.target_dt = target_dt
        self.mix_ratio = mix_ratio
        self.num_steps_fwd = num_steps_fwd
        self.num_hist = num_hist
        self.dt = dt
        self.dtype = dtype
        self.num_ctrls_hist = num_ctrls_hist
        self.real_y_dt = 0.0

        self.real_processed_data = self._process_raw_data(raw_real_data_dict, is_sim=False)
        self.sim_processed_data = self._process_raw_data(raw_sim_data_dict, is_sim=True)

        self.processed_data = self._combine_processed_data(
            self.real_processed_data,
            self.sim_processed_data,
            self.mix_ratio
        )

    @staticmethod
    def _combine_processed_data(real_data, sim_data, mix_ratio):
        if mix_ratio == 0.0 or len(sim_data) == 0:
            return real_data
        elif mix_ratio == 1.0 or len(real_data) == 0:
            return sim_data

        real_pct, sim_pct = (1 - mix_ratio), mix_ratio

        if (len(sim_data) / sim_pct) >= (len(real_data) / real_pct):
            new_total = round(len(sim_data) / sim_pct)

            indices = list(range(len(real_data)))
            random.shuffle(indices)
            real_data = [real_data[i] for i in indices[:new_total - len(sim_data)]]
        else:
            new_total = round(len(real_data) / real_pct)
            indices = list(range(len(sim_data)))
            random.shuffle(indices)
            sim_data = [sim_data[i] for i in indices[:new_total - len(real_data)]]

        processed_data = sim_data + real_data

        return processed_data

    def _get_ctrls_hist(self, controls):
        # end = len(controls) if self.num_steps_fwd == 1 else -self.num_steps_fwd + 1
        ctrls_hist = controls
        ctrls_hist_pad = [torch.zeros_like(ctrls_hist[0]) for _ in range(self.num_ctrls_hist)]
        ctrls_hist = ctrls_hist_pad + ctrls_hist

        traj_ctrls_hist = [torch.cat(ctrls_hist[j - self.num_ctrls_hist: j], 2)
                           if self.num_ctrls_hist > 0 else torch.empty(ctrls_hist[0].shape)
                           for j in range(self.num_ctrls_hist, len(ctrls_hist))]
        return traj_ctrls_hist

    def _process_raw_data(self, raw_data_dict, is_sim=False):
        processed_data = []

        all_states, all_times, all_end_pts = (
            raw_data_dict['states'], raw_data_dict['times'], raw_data_dict["gt_end_pts"])
        all_ctrls, all_act_lens = (
            raw_data_dict["controls"], raw_data_dict["act_lengths"])

        all_motor_omegas = raw_data_dict["motor_omegas"]

        for i in range(len(all_states)):
            states, times, end_pts = all_states[i], all_times[i], all_end_pts[i]
            controls, act_lengths = all_ctrls[i], [torch.hstack(a) for a in all_act_lens[i]]
            dataset_idx = torch.tensor([[raw_data_dict['dataset_idx'][i]]], dtype=torch.int)

            motor_omegas = all_motor_omegas[i]

            traj_ctrls_hist = self._get_ctrls_hist(controls)

            for j in range(len(states) - 1):
                next_t, next_idx = self._find_next_t(times[j], times[j + 1:])
                if next_t is None or next_idx is None:
                    continue

                curr_times = times[j: j + next_idx + 2]
                t0, t1 = curr_times[0], curr_times[-1]
                dt = t1 - t0
                num_steps = misc_utils.compute_num_steps(dt, self.dt)
                delta_t = num_steps * self.dt

                x = states[j]

                mask = torch.zeros((1, 1, num_steps), dtype=torch.int)
                y = end_pts[0].reshape(1, -1, 1).repeat(1, 1, num_steps)
                for m, t in enumerate(curr_times[1:]):
                    if not is_sim and round(t - t0, 5) < self.real_y_dt:
                        continue
                    idx = round((t - t0) / self.dt) - 1
                    y[..., idx] = end_pts[j + 1 + m].reshape(1, -1)
                    mask[..., idx] = 1

                idx0 = round(t0 / self.dt)
                idx1 = round(t1 / self.dt)
                act_lens = act_lengths[idx0]
                motor_speed = motor_omegas[idx0]
                ctrls = torch.cat(controls[idx0:idx1], dim=2)
                ctrls_hist = traj_ctrls_hist[idx0]
                next_act_lens = torch.concat(act_lengths[idx0 + 1: idx1 + 1], dim=2)

                processed_instance = {
                    'x': x,
                    'y': y,
                    'ctrl': ctrls,
                    'ctrls_hist': ctrls_hist,
                    'dt': torch.tensor([[[dt]]], dtype=self.dtype),
                    'act_len': act_lens,
                    'next_act_lens': next_act_lens,
                    'delta_t': torch.tensor([[[delta_t]]], dtype=self.dtype),
                    'dataset_idx': dataset_idx.clone(),
                    'mask': mask,
                    'motor_omega': motor_speed
                }
                processed_data.append(processed_instance)

        # max_dt = torch.vstack([d['delta_t'] for d in processed_data]).max().item()
        # for d in processed_data:
        #     dt = d['delta_t'].item()
        #     if abs(max_dt - dt) > 1e-5:
        #         n = round((max_dt - dt) / self.dt)
        #         ctrl, next_act_lens, mask, y = d['ctrl'], d['next_act_lens'], d['mask'], d['y']
        #
        #         ctrl_pad = zeros((ctrl.shape[0], ctrl.shape[1], n), ref_tensor=ctrl)
        #         next_act_lens_pad = next_act_lens[..., -1:].repeat(1, 1, n)
        #         mask_pad = torch.zeros((1, 1, n), dtype=torch.int)
        #         y_pad = y[..., -1:].repeat(1, 1, n)
        #
        #         d['ctrl'] = torch.cat([ctrl, ctrl_pad], dim=-1)
        #         d['next_act_lens'] = torch.cat([next_act_lens, next_act_lens_pad], dim=-1)
        #         d['mask'] = torch.cat([mask, mask_pad], dim=-1)
        #         d['y'] = torch.cat([y, y_pad], dim=-1)

        return processed_data

    def _find_next_t(self, t, future_times):
        best_dt, best_next_t, best_next_idx = 9e9, None, None
        for i, f_t in enumerate(future_times):
            dt = round(f_t - t, 5)
            if self.min_dt <= dt <= self.max_dt and abs(dt - self.target_dt) < best_dt:
                best_dt = dt
                best_next_t = f_t
                best_next_idx = i
            elif dt > self.max_dt:
                break

        return best_next_t, best_next_idx

    @staticmethod
    def collate_fn(batch):
        max_len = max([b['y'].shape[-1] for b in batch])
        for b in batch:
            y_len = b['y'].shape[-1]
            if y_len < max_len:
                n = max_len - y_len
                ctrl, next_act_lens, mask, y = b['ctrl'], b['next_act_lens'], b['mask'], b['y']

                ctrl_pad = torch.zeros_like(ctrl[..., :1]).repeat(1, 1, n)
                next_act_lens_pad = next_act_lens[..., -1:].repeat(1, 1, n)
                mask_pad = torch.zeros((1, 1, n), dtype=torch.int)
                y_pad = y[..., -1:].repeat(1, 1, n)

                b['ctrl'] = torch.cat([ctrl, ctrl_pad], dim=-1)
                b['next_act_lens'] = torch.cat([next_act_lens, next_act_lens_pad], dim=-1)
                b['mask'] = torch.cat([mask, mask_pad], dim=-1)
                b['y'] = torch.cat([y, y_pad], dim=-1)

        collated_batch_dict = {}
        for k in batch[0].keys():
            collated_batch_dict[k] = torch.vstack([b[k] for b in batch])

        return collated_batch_dict


class RealMultiSimRecurMotorTensegrityDataset(RealMultiSimTensegrityDataset):

    def __init__(self,
                 raw_real_data_dict,
                 raw_sim_data_dict,
                 min_dt,
                 max_dt,
                 target_dt,
                 mix_ratio,
                 num_steps_fwd=1,
                 num_hist=1,
                 num_ctrls_hist=20,
                 dt=0.01,
                 dtype=DEFAULT_DTYPE):
        self.num_ctrls_hist = num_ctrls_hist
        super().__init__(raw_real_data_dict,
                         raw_sim_data_dict,
                         min_dt,
                         max_dt,
                         target_dt,
                         mix_ratio,
                         num_steps_fwd,
                         num_hist,
                         dt,
                         dtype)

    def _process_raw_data(self, raw_data_dict, is_sim=False):
        processed_data = super()._process_raw_data(raw_data_dict, is_sim=is_sim)

        k = 0
        for i in range(len(raw_data_dict['states'])):
            controls = torch.vstack(raw_data_dict['controls'][i])

            end = controls.shape[0] if self.num_steps_fwd == 1 else -self.num_steps_fwd + 1
            ctrls_hist = raw_data_dict['controls'][i][:end]
            ctrls_hist_pad = [torch.zeros_like(ctrls_hist[0]) for _ in range(self.num_ctrls_hist)]
            ctrls_hist = ctrls_hist_pad + ctrls_hist

            traj_ctrls_hist = [torch.stack(ctrls_hist[j - self.num_ctrls_hist: j], 2)
                               if self.num_ctrls_hist > 0 else torch.empty(ctrls_hist[0].shape)
                               for j in range(self.num_ctrls_hist, len(ctrls_hist))]

            for ctrls_hist in traj_ctrls_hist:
                processed_data[k]['ctrls_hist'] = ctrls_hist
                k += 1

        return processed_data