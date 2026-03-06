import json
import logging
import math
import random
from copy import deepcopy
from pathlib import Path
from typing import Union, Dict, List

import torch
import tqdm
from torch.nn.modules.loss import _Loss
from torch_geometric.data import Data as Graph

import numpy as np

from gnn_simulator.nn_training.datasets.real_tensegrity_dataset import RealMultiSimTensegrityDataset
from gnn_simulator.nn_training.tensegrity_gnn_training_engine import TensegrityMultiSimGNNTrainingEngine
from gnn_simulator.simulators.tensegrity_gnn_simulator import TensegrityMultiSimGNNSimulator, \
    TensegrityMultiSimRecurrentGNNSimulator, MultiSimMultiStepMotorTensegrityGNNSimulator
from gnn_simulator.simulators.tensegrity_physics_simulator import Tensegrity5dRobotSimulator
from gnn_simulator.utilities import misc_utils
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.tensor_utils import zeros


class RealTensegrity5DGNNTrainingEngine(TensegrityMultiSimGNNTrainingEngine):
    target_dt: float
    min_dt: float
    max_dt: float
    vel_min_dt: float
    mix_ratio: float
    use_all_data: bool

    def __init__(self,
                 training_config: Dict,
                 criterion: _Loss,
                 dt: Union[float, torch.Tensor],
                 logger: logging.Logger):
        self.target_dt = training_config['target_dt']
        self.min_dt = max(self.target_dt - training_config['dt_delta'] - 1e-6, 0.01 - 1e-6)
        self.max_dt = self.target_dt + training_config['dt_delta'] + 1e-6
        self.vel_min_dt = training_config['vel_min_dt'] if 'vel_min_dt' in training_config else 0.0
        self.mix_ratio = training_config['mix_ratio']
        self.eval_weight_real = training_config.get('eval_weight_real', 0.5)
        self.use_all_data = True
        # self.num_ctrls_hist = 0
        training_config['num_steps_fwd'] = round(self.target_dt / dt)
        super().__init__(training_config, criterion, dt, logger)

        # self.train_dataloader = self.build_batches(self.train_data_dict)
        # self.val_dataloader = self.build_batches(self.val_data_dict)

    def _get_dataset(self, data_dict):
        real_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' in n]
        sim_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' not in n]

        real_data_dict = {k: [v[i] for i in real_idxs] for k, v in data_dict.items()}
        sim_data_dict = {k: [v[i] for i in sim_idxs] for k, v in data_dict.items()}

        return RealMultiSimTensegrityDataset(
            real_data_dict,
            sim_data_dict,
            self.min_dt,
            self.max_dt,
            self.target_dt,
            self.mix_ratio,
            self.num_steps_fwd,
            self.num_hist,
            self.num_ctrls_hist,
            self.dt,
            self.dtype
        )

    def _get_endpts(self, data_jsons):
        with torch.no_grad():
            data_end_pts = []
            for data_json in data_jsons:
                end_pts = [
                    torch.tensor(d['end_pts'], dtype=self.dtype).unsqueeze(-1)
                    for d in data_json
                ]
                data_end_pts.append(end_pts)
        return data_end_pts

    def load_json_files(self, paths):
        data_jsons, target_gait_jsons, extra_state_infos, vis_data = [], [], [], []
        for path in paths:
            processed_data_path = Path(path, 'processed_data.json')
            if not processed_data_path.exists():
                processed_data_path = Path(path, 'interp_processed_data.json')
            with processed_data_path.open('r') as fp:
                data_jsons.append(json.load(fp))

            extra_info_path = Path(path, f"extra_state_data.json")
            if not extra_info_path.exists():
                extra_info_path = Path(path, 'interp_extra_state_data.json')

            if extra_info_path.exists():
                with extra_info_path.open('r') as fp:
                    extra_state_infos.append(json.load(fp))
            else:
                extra_state_infos.append(None)

            vis_path = Path(path, 'vis_data_0.01.json')
            if vis_path.exists():
                with vis_path.open('r') as fp:
                    vis_data.append(json.load(fp))
            else:
                vis = deepcopy(data_jsons[-1])
                for i in range(len(vis)):
                    pos, quat = np.array(vis[i]['pos']), np.array(vis[i]['quat'])
                    pose = np.hstack([pos.reshape(-1, 3), quat.reshape(-1, 4)]).flatten()
                    vis[i]['pos'] = pose.tolist()
                vis_data.append(vis)

        return data_jsons, target_gait_jsons, extra_state_infos, vis_data

    def _init_data(self, data_paths):
        data_dict = {}
        data_dict['names'] = [Path(p).name for p in data_paths]

        data_jsons, target_gaits, extra_state_infos, vis_data = (
            self.load_json_files(data_paths))

        # augment sim data so that length of extra_data == length of data_jsons
        for i, n in enumerate(data_dict['names']):
            if 'real' in n:
                continue

            if len(extra_state_infos[i]) == len(data_jsons[i]) - 1:
                extra_state_infos[i].append({
                    'time': extra_state_infos[i][-1]['time'] + self.dt,
                    'dt': extra_state_infos[i][-1]['dt'],
                    'controls': extra_state_infos[i][-1]['controls'],
                    'rest_lengths': extra_state_infos[i][-1]['rest_lengths'],
                })

        for i in range(len(data_jsons)):
            assert len(data_jsons[i]) == len(extra_state_infos[i]), \
                "Mismatched between processed data and extra info data"

        extra_state_infos_cpy = deepcopy(extra_state_infos)
        real_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' in n]
        real_data_jsons = [data_jsons[i] for i in real_idxs]
        real_extra_state_infos = [extra_state_infos[i] for i in real_idxs]

        real_data_jsons, real_extra_state_infos = self.filter_data(
            real_data_jsons, real_extra_state_infos, data_dict['names'])

        for i, real_idx in enumerate(real_idxs):
            data_jsons[real_idx] = real_data_jsons[i]
            extra_state_infos[real_idx] = real_extra_state_infos[i]

        max_times = [e[-1]['time'] for e in extra_state_infos]
        data_controls, data_dl_rates, data_act_lens = (
            self._get_interp_ctrls_dl_spds_act_lens(extra_state_infos_cpy, max_times))

        data_motor_omegas = [
            [torch.zeros_like(controls[0])] + [controls[max(j - 1, 0)] * 0.7 * 220 * 2 * torch.pi / 60
             for j in range(1, len(controls))]
            for controls in data_controls
        ]

        data_dict.update({
            'data_jsons': data_jsons,
            'extra_state_infos': extra_state_infos,
            'controls': data_controls,
            'act_lengths': data_act_lens,
            'motor_omegas': data_motor_omegas,
            'dl_rates': data_dl_rates,
            'vis_data': vis_data,
        })

        data_dict['gt_end_pts'] = self._get_endpts(data_jsons)

        data_dict['times'] = [[d['time'] - data_json[0]['time'] for d in data_json]
                              for data_json in data_jsons]

        data_dict['states'], _ = self.data_json_to_states(data_jsons,
                                                          data_dict['gt_end_pts'],
                                                          data_dict['times'],
                                                          extra_state_infos,
                                                          data_dict['names'])

        # data_dict['act_lengths'], data_dict['motor_omegas'] = (
        #     self.get_act_lens_motor_omegas(extra_state_infos, data_dict['times'])
        # )
        data_dict['dataset_idx'] = self.enum_dataset(data_paths)

        return data_dict

    def _get_interp_ctrls_dl_spds_act_lens(self, extra_state_infos_cpy, max_times):
        data_controls, data_dl_rates = [], []
        data_act_lens = []
        for j, e_data in enumerate(extra_state_infos_cpy):
            controls, dl_rates = [], []
            for i in range(len(e_data) - 1):
                if e_data[i + 1]['time'] > max_times[j]:
                    continue

                dt = e_data[i + 1]['time'] - e_data[i]['time']
                num_steps = round(dt / self.dt)
                ctrl = torch.tensor(e_data[i]['controls'], dtype=self.dtype).reshape(1, -1, 1)
                controls.extend([ctrl] * num_steps)

            gt_rest_lens = []
            last_idx = -1
            for i in range(len(e_data) - 1):
                if e_data[i + 1]['time'] > max_times[j]:
                    last_idx = i
                    break

                r0, r1 = e_data[i]['rest_lengths'], e_data[i + 1]['rest_lengths']
                t0, t1 = e_data[i]['time'], e_data[i + 1]['time']
                n = round((t1 - t0) / 0.01)
                gt_rest_lens.append(r0)
                for k in range(1, n):
                    t = t0 + k * self.dt
                    w = (t1 - t) / (t1 - t0)
                    r = [w * r0[m] + (1 - w) * r1[m] for m in range(len(r0))]
                    gt_rest_lens.append(r)
            gt_rest_lens.append(e_data[last_idx]['rest_lengths'])

            rest_rest_lens = [c._rest_length for c in self.simulator.robot.actuated_cables.values()]
            data_act_lens.append([
                [r - g for g, r in zip(gt_rest_lens[i], rest_rest_lens)]
                for i in range(len(gt_rest_lens))
            ])

            for i in range(len(gt_rest_lens) - 1):
                r0, r1 = gt_rest_lens[i], gt_rest_lens[i + 1]
                ctrl = controls[i]
                dl_rate = torch.tensor(
                    [[(r0[m] - r1[m]) / self.dt / ctrl[:, m].item()
                      if ctrl[:, m] != 0 else 0.
                      for m in range(len(r0))]],
                    dtype=self.dtype)
                dl_rates.append(dl_rate)

            data_controls.append(controls)
            data_dl_rates.append(dl_rates)
        return data_controls, data_dl_rates, data_act_lens

    def enum_dataset(self, data_paths):
        dataset_idx = [int(Path(p).parent.name.split("_")[-1]) for p in data_paths]
        return dataset_idx

    def data_json_to_states(self,
                            data_jsons,
                            gt_end_pts,
                            times,
                            extra_state_jsons=None,
                            data_names=None):
        data_pos, data_quats, data_controls = self.data_to_pos_quat_ctrls(
            data_jsons,
            gt_end_pts,
            extra_state_jsons
        )
        data_vels = self.get_ins_vels(data_jsons)
        data_states = self.pos_quat_to_states(
            data_pos,
            data_quats,
            times,
            data_vels,
            data_names
        )

        return data_states, data_controls

    def data_to_pos_quat_ctrls(self, data_jsons, gt_end_pts, extra_state_jsons):
        data_pos, data_quats, data_controls = [], [], []
        for i, data_json in enumerate(data_jsons):
            pos, quats, controls = [], [], []
            for j, d in enumerate(data_json):
                end_pts = gt_end_pts[i][j]
                end_pts = [(end_pts[k: k + 1], end_pts[k + 1: k + 2])
                           for k in range(0, len(end_pts), 2)]

                pos.append([(e[1] + e[0]) / 2 for e in end_pts])
                quats.append([
                    torch_quaternion.compute_quat_btwn_z_and_vec(
                        e[1] - e[0]
                    ) for e in end_pts
                ])

            data_pos.append(pos)
            data_quats.append(quats)

        if extra_state_jsons is not None:
            times = [[d['time'] - data_json[0]['time']
                      for d in data_json]
                     for data_json in data_jsons]
            data_controls = self.load_controls(extra_state_jsons, times)

        return data_pos, data_quats, data_controls

    def shift_ctrls(self, extra_state_jsons):
        for extra_state_json in extra_state_jsons:
            for i in range(len(extra_state_json) - 1):
                t0 = extra_state_json[i]['time']
                t1 = extra_state_json[i + 1]['time']

                curr_rest_lens = extra_state_json[i]['rest_lengths']
                next_rest_lens = extra_state_json[i + 1]['rest_lengths']

                change_rates = [(r1 - r0) / (t1 - t0)
                                for r0, r1 in zip(curr_rest_lens, next_rest_lens)]
                for j in range(len(change_rates)):
                    rate = change_rates[j]
                    if abs(rate) > 0.25:
                        ctrl = -rate / abs(rate)
                        extra_state_json[i]['controls'][j] = ctrl
                    else:
                        extra_state_json[i]['controls'][j] = 0

        return extra_state_jsons

    def load_controls(self, extra_state_jsons, traj_times):
        # extra_state_jsons = self.shift_ctrls(extra_state_jsons)
        return super().load_controls(extra_state_jsons, traj_times)

    def get_start_states(self, data_jsons, max_steps=10000, threshold=1e-2):
        with torch.no_grad():
            sim = self.get_dummy_simulator()

            start_state = []
            for d in data_jsons:
                end_pts = torch.tensor(d[0]['end_pts'], dtype=self.dtype).reshape(-1, 6, 1)
                pos = (end_pts[:, 3:] + end_pts[:, :3]) / 2.
                prin = (end_pts[:, 3:] - end_pts[:, :3])
                quat = torch_quaternion.compute_quat_btwn_z_and_vec(prin)
                vels = zeros((pos.shape[0], 6, 1), ref_tensor=pos)

                start_state.append(torch.hstack([pos, quat, vels]).reshape(1, -1, 1))

            start_state = torch.vstack(start_state)
            curr_state = start_state.clone()
            sim.update_state(curr_state)

            step = 0
            curr_vel = torch.tensor(1)
            while curr_vel.abs().max() > threshold and step < max_steps:
                step += 1
                curr_state = sim.step(
                    curr_state=curr_state,
                    dt=0.001
                )
                curr_vel = curr_state.reshape(-1, 13, 1)[:, 7:]
                # flag = curr_vel.abs().max(dim=1).flatten() > threshold

                if step % 200 == 0:
                    print(step, curr_vel.abs().max().item())

            curr_state = self._shift_robot(curr_state, start_state)
            start_state = [curr_state[i: i + 1] for i in range(curr_state.shape[0])]

            return start_state

    def _shift_robot(self, curr_end_pts, shift_end_pts):
        n = len(self.simulator.robot.rods)

        curr_end_pts_ = curr_end_pts.reshape(-1, 3, 1)
        shift_end_pts_ = shift_end_pts.reshape(-1, 3, 1)

        curr_com = curr_end_pts_.mean(dim=0, keepdim=True)
        curr_prin = curr_end_pts_[1::2].mean(dim=0, keepdim=True) - curr_end_pts_[::2].mean(dim=0, keepdim=True)
        curr_prin[:, 2] = 0.
        curr_prin /= curr_prin.norm(dim=1, keepdim=True)

        shift_com = shift_end_pts_.mean(dim=0, keepdim=True)
        shift_prin = shift_end_pts_[1::2].mean(dim=0, keepdim=True) - shift_end_pts_[::2].mean(dim=0, keepdim=True)
        shift_prin[:, 2] = 0.
        shift_prin /= shift_prin.norm(dim=1, keepdim=True)

        angle = torch.linalg.vecdot(shift_prin, curr_prin, dim=1).unsqueeze(1)
        angle = torch.acos(torch.clamp(angle, -1, 1)) / 2.
        rot_axis = torch.cross(curr_prin, shift_prin, dim=1)
        q = torch.hstack([torch.cos(angle), rot_axis * torch.sin(angle)])
        q = q.repeat(n, 1, 1)

        pos = (curr_end_pts_[::2] + curr_end_pts_[1::2]) / 2.
        pos = torch_quaternion.rotate_vec_quat(q, pos - curr_com)
        pos[:, :2] += shift_com[:, :2]
        pos[:, 2] += curr_com[:, 2]
        quat = torch_quaternion.compute_quat_btwn_z_and_vec(curr_prin)
        quat = torch_quaternion.quat_prod(q, quat)
        vels = torch.zeros_like(pos).repeat(1, 2, 1)

        next_state = torch.hstack([pos, quat, vels]).reshape(1, -1, 1)

        return next_state

    def get_ins_vels(self, data_jsons):
        return None

    def pos_quat_to_states(self, data_pos, data_quats, times, data_vels, data_names=None):
        num_rods = len(self.simulator.robot.rods)

        data_states = []
        for k, (pos, quats, times) in enumerate(zip(data_pos, data_quats, times)):
            states = []
            for i in range(len(pos)):
                prev_i = max(i - 1, 0)
                if data_names is not None and 'real' in data_names[k]:
                    while round(times[i] - times[prev_i], 5) < self.vel_min_dt and prev_i > 0:
                        prev_i -= 1

                pos_0, pos_1 = pos[prev_i], pos[i]
                quat_0, quat_1 = quats[prev_i], quats[i]
                dt = times[i] - times[prev_i]

                if i > 0:
                    lin_vels = [(pos_1[j] - pos_0[j]) / dt for j in range(num_rods)]
                    ang_vels = [
                        torch_quaternion.compute_ang_vel_quat(
                            quat_0[j],
                            quat_1[j],
                            dt
                        ) for j in range(num_rods)
                    ]
                else:
                    lin_vels = [torch.zeros_like(pos_0[j]) for j in range(num_rods)]
                    ang_vels = [torch.zeros_like(pos_0[j]) for j in range(num_rods)]

                state = torch.hstack([
                    torch.hstack([pos_1[j], quat_1[j], lin_vels[j], ang_vels[j]])
                    for j in range(num_rods)
                ])
                states.append(state)
            data_states.append(states)

        return data_states

    def filter_endcaps(self, all_end_caps, threshold_cycle=0.5, threshold_upright=0.5):
        keep_indices = []
        for i, end_caps in enumerate(all_end_caps):
            left = end_caps[::2].mean(dim=0, keepdim=True)
            right = end_caps[1::2].mean(dim=0, keepdim=True)
            prin = right - left
            prin[:, 2] = 0.
            prin = prin / prin.norm(dim=1, keepdim=True)

            # left cycle
            l0 = end_caps[2:3] - end_caps[:1]
            l1 = end_caps[4:5] - end_caps[:1]
            l2 = torch.cross(l1, l0, dim=1)
            l2_upright = l2[:, 2:].abs() / l2.norm(dim=1, keepdim=True)
            l2[:, 2] = 0.
            l2 = l2 / l2.norm(dim=1, keepdim=True)

            # right cycle
            r0 = end_caps[3:4] - end_caps[1:2]
            r1 = end_caps[5:] - end_caps[1:2]
            r2 = torch.cross(r1, r0, dim=1)
            r2_upright = r2[:, 2:].abs() / r2.norm(dim=1, keepdim=True)
            r2[:, 2] = 0.
            r2 = r2 / r2.norm(dim=1, keepdim=True)

            z = torch.zeros_like(l2)
            z[:, 2] = 1.

            l2_cycle = torch.linalg.vecdot(l2, prin, dim=1).unsqueeze(1)
            r2_cycle = torch.linalg.vecdot(r2, prin, dim=1).unsqueeze(1)

            if l2_cycle > threshold_cycle and r2_cycle > threshold_cycle \
                    and l2_upright < threshold_upright and r2_upright < threshold_upright:
                keep_indices.append(i)

        return keep_indices

    def filter_mismatched_endcap_lens(self, all_end_caps, all_sensor_lens, threshold=0.5):
        # sensor_order = [(3, 5), (1, 3), (1, 5), (0, 2), (0, 4),
        #                 (2, 4), (2, 5), (0, 3), (1, 4)]
        sensor_order = [[3, 1, 1, 0, 0, 2, 2, 0, 1], [5, 3, 5, 2, 4, 4, 5, 3, 4]]
        keep_indices = []
        for j, end_caps in enumerate(all_end_caps):
            # diffs = [
            #     (end_caps[x0] - end_caps[x1]).norm().item() - all_sensor_lens[j][k]
            #     for k, (x0, x1) in enumerate(sensor_order)
            # ]
            # if all([d < threshold for d in diffs]):
            #     keep_indices.append(j)

            diffs = (end_caps[sensor_order[1]] - end_caps[sensor_order[0]]).norm(dim=1, keepdim=True)
            diffs = (diffs.flatten() - all_sensor_lens[j]).abs()
            if (diffs < threshold).all():
                keep_indices.append(j)

        return keep_indices

    def filter_data(self, all_data_jsons, all_extra_state_jsons, all_data_names):
        all_filtered_data_jsons, all_filtered_extra_state_jsons = [], []
        for i in range(len(all_data_jsons)):
            data_json = all_data_jsons[i]
            extra_state_json = all_extra_state_jsons[i]

            filtered_data_json = data_json[:-10]
            filtered_extra_state_json = extra_state_json[:-10]

            end_caps = [torch.tensor(d['end_pts'], dtype=self.dtype)
                        for d in filtered_data_json]
            sensor_lens = [torch.tensor(e['sensor_lens'], dtype=self.dtype)
                           for e in filtered_extra_state_json]

            keep_indices = self.filter_mismatched_endcap_lens(end_caps, sensor_lens)
            filtered_end_caps = [end_caps[i] for i in keep_indices]
            filtered_data_json = [data_json[i] for i in keep_indices]
            filtered_extra_state_json = [extra_state_json[i] for i in keep_indices]

            if keep_indices[0] != 0:
                filtered_end_caps = [end_caps[0]] + filtered_end_caps
                filtered_data_json = [data_json[0]] + filtered_data_json
                filtered_extra_state_json = [extra_state_json[0]] + filtered_extra_state_json

            keep_indices = self.filter_endcaps(filtered_end_caps)
            filtered_data_json = [filtered_data_json[i] for i in keep_indices]
            filtered_extra_state_json = [filtered_extra_state_json[i] for i in keep_indices]

            if keep_indices[0] != 0:
                filtered_data_json = [data_json[0]] + filtered_data_json
                filtered_extra_state_json = [extra_state_json[0]] + filtered_extra_state_json

            all_filtered_data_jsons.append(filtered_data_json)
            all_filtered_extra_state_jsons.append(filtered_extra_state_json)

        return all_filtered_data_jsons, all_filtered_extra_state_jsons

    def find_next_t(self, t, future_times):
        best_dt, best_next_t, best_next_idx = 999999., None, None
        for i, f_t in enumerate(future_times):
            dt = f_t - t
            if self.min_dt <= dt <= self.max_dt and abs(dt - self.target_dt) < best_dt:
                best_dt = dt
                best_next_t = f_t
                best_next_idx = i
            elif dt > self.max_dt:
                break

        return best_next_t, best_next_idx

    def build_batches(self,
                      data_dict):
        real_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' in n]
        sim_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' not in n]

        real_data_dict = {k: [v[i] for i in real_idxs] for k, v in data_dict.items()}
        sim_data_dict = {k: [v[i] for i in sim_idxs] for k, v in data_dict.items()}

        real_batch_dict = self.build_batch_dict(real_data_dict)
        real_batch_dict = self.shuffle_batches(real_batch_dict)

        sim_batch_dict = self.build_batch_dict(sim_data_dict)
        sim_batch_dict = self.shuffle_batches(sim_batch_dict)

        batches = self.combine_sim_real_to_batches(real_batch_dict, sim_batch_dict)

        return batches

    def combine_sim_real_to_batches(self, real_batch_dict, sim_batch_dict):
        all_keys = set(list(real_batch_dict.keys()) + list(sim_batch_dict.keys()))

        batches = []
        for k in all_keys:
            r_v = real_batch_dict[k]
            s_v = sim_batch_dict[k]

            total_real, total_sim = len(r_v['x']), len(s_v['x'])

            n_real = round((1 - self.mix_ratio) * self.max_batch_size)
            n_sim = self.max_batch_size - n_real
            i_real, i_sim = 0, 0
            while i_real < total_real and i_sim < total_sim:
                end_real = min(i_real + n_real, total_real)
                end_sim = min(i_sim + n_sim, total_sim)

                batch = {
                    key: torch.vstack(r_v[key][i_real: end_real] + s_v[key][i_sim: end_sim])
                    for key in r_v.keys()
                }
                batches.append((k, batch))

                i_real, i_sim = end_real, end_sim

        return batches

    def build_batch_dict(self, kwargs_dict):
        batch_dict = {}
        keys = ['x', 'y', 'ctrl', 'dt', 'act_len', 'next_act_lens',
                'delta_t', 'dl_spds', 'dataset_idx']

        all_states, all_times, all_end_pts = (
            kwargs_dict['states'], kwargs_dict['times'], kwargs_dict["gt_end_pts"])
        all_ctrls, all_act_lens = (
            kwargs_dict["controls"], kwargs_dict["act_lengths"])
        all_dl_rates = kwargs_dict["dl_rates"]
        n = 0
        for i in range(len(all_states)):
            states, times, end_pts = all_states[i], all_times[i], all_end_pts[i]
            controls, act_lengths = all_ctrls[i], all_act_lens[i]
            dl_rates = all_dl_rates[i]
            dataset_idx = torch.tensor([[kwargs_dict['dataset_idx'][i]]], dtype=torch.int)

            act_lengths = [torch.hstack(a) for a in act_lengths]

            # ctrls_hist = [torch.zeros_like((controls[0]))] * self.num_ctrls_hist + controls

            for j in range(len(states) - 1):
                next_t, next_idx = self.find_next_t(times[j], times[j + 1:])
                if next_t is None or next_idx is None:
                    continue

                t0, t1 = times[j], times[j + next_idx + 1]
                dt = t1 - t0
                delta_t = misc_utils.compute_num_steps(dt, self.dt) * self.dt

                x, y = states[j], end_pts[j + next_idx + 1].reshape(1, -1, 1)

                idx0 = round(t0 / self.dt)
                idx1 = round(t1 / self.dt)
                act_lens = act_lengths[idx0]
                ctrls = torch.stack(controls[idx0:idx1], dim=2)
                dl_spds = torch.stack(dl_rates[idx0:idx1], dim=2)
                next_act_lens = torch.concat(act_lengths[idx0 + 1: idx1 + 1], dim=2)

                batch = batch_dict.get(delta_t, {k: [] for k in keys})
                batch['x'].append(x)
                batch['y'].append(y)
                batch['ctrl'].append(ctrls)
                batch['dt'].append(torch.tensor([[[dt]]], dtype=self.dtype))
                batch['act_len'].append(act_lens)
                batch['next_act_lens'].append(next_act_lens)
                batch['delta_t'].append(torch.tensor([[[delta_t]]], dtype=self.dtype))
                batch['dl_spds'].append(dl_spds)
                batch['dataset_idx'].append(dataset_idx.clone())
                batch_dict[delta_t] = batch

        # batch_dict = {
        #     k: [torch.vstack(v[m]) for m in range(9)]
        #     for k, v in batch_dict.items()
        # }

        max_k = max(batch_dict.keys())
        new_batch_dict = {max_k: {k: [] for k in keys}}
        for k, v in batch_dict.items():  # {str: dict(str: list)}
            for kk, vv in v.items():
                for vvv in vv:
                    if kk in ['ctrl', 'dl_spds'] and k != max_k:
                        n = round((max_k - k) / self.dt)
                        v_pad = zeros((vvv.shape[0], vvv.shape[1], n), ref_tensor=vvv)
                        vvv = torch.cat([vvv, v_pad], dim=2)
                    elif kk == 'next_act_lens' and k != max_k:
                        n = round((max_k - k) / self.dt)
                        v_pad = vvv[..., -1:].repeat(1, 1, n)
                        vvv = torch.cat([vvv, v_pad], dim=2)
                    new_batch_dict[max_k][kk].append(vvv)
        # batch = {k: torch.vstack(b) for k, b in batch.items()}
        # batch_dict = {max_k: batch}

        return new_batch_dict

    def evaluate_rollouts(self, data_dict, prefix=''):
        real_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' in n]
        sim_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' not in n]

        real_data_dict = {k: [v[i] for i in real_idxs] for k, v in data_dict.items()}
        sim_data_dict = {k: [v[i] for i in sim_idxs] for k, v in data_dict.items()}

        real_n_step_loss = self.eval_n_step_aheads(200, real_data_dict)
        real_com_loss, real_angle_loss = self.eval_rollout_fixed_ctrls(real_data_dict, prefix)

        sim_n_step_loss = self.eval_n_step_aheads(200, sim_data_dict)
        sim_com_loss, sim_angle_loss = self.eval_rollout_fixed_ctrls(sim_data_dict, prefix)

        # n_real, n_sim = len(real_idxs), len(sim_idxs)
        # n_total = n_real + n_sim
        # full_traj_loss = (real_com_loss * n_real + sim_com_loss * n_sim) / n_total
        # n_step_loss = (real_n_step_loss * n_real + sim_n_step_loss * n_sim) / n_total
        full_traj_loss = (self.eval_weight_real * real_com_loss + (1 - self.eval_weight_real) * sim_com_loss)
        n_step_loss = (self.eval_weight_real * real_n_step_loss + (1 - self.eval_weight_real) * sim_n_step_loss)

        other_losses = [
            real_com_loss,
            real_angle_loss,
            real_n_step_loss,
            sim_com_loss,
            sim_angle_loss,
            sim_n_step_loss
        ]

        return full_traj_loss, n_step_loss, other_losses

        # def _get_n_step_act_lens_ctrls(self, act_lengths, all_controls, idxs, ctrl_idxs, n):
        #     if self.use_gt_act_lens:
        #         gt_act_lens, ctrls = [], None
        #         for k in range(n):
        #             end_idx = k + len(all_controls) - n
        #             act_lens = torch.vstack(act_lengths[k + 1: end_idx + 1])[idxs].to(self.device)
        #             gt_act_lens.append(act_lens)
        #         gt_act_lens = torch.concat(gt_act_lens, dim=-1)
        #     else:
        #         gt_act_lens, ctrls = None, []
        #         for k in range(n):
        #             end_idx = k + len(all_controls) - n
        #             controls = torch.vstack(all_controls[k:end_idx])[idxs].to(self.device).squeeze(-1)
        #             ctrls.append(controls)
        #         ctrls = torch.stack(ctrls, dim=-1)

        return gt_act_lens, ctrls

    def eval_n_step_aheads(self, n, data_dict, num_samples=1000):
        trajs = data_dict['states']
        all_act_lengths = data_dict['act_lengths']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']
        times = data_dict['times']
        data_names = data_dict['names']

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(trajs)):
            states = [trajs[i][0].clone() for _ in range(self.num_hist - 1)] + trajs[i]
            act_lengths = torch.vstack([torch.hstack(a) for a in all_act_lengths[i]])

            curr_state = torch.vstack(states).to(self.device)

            idx_starts, idx_ends = self.compute_eval_n_steps_idxs(
                times[i], n * self.dt, max_sample=num_samples
            )
            num_steps = [round((times[i][e] - times[i][s]) / self.dt)
                         for s, e in zip(idx_starts, idx_ends)]
            max_steps, min_steps = max(num_steps), min(num_steps)
            num_steps = [num - min_steps for num in num_steps]

            all_ctrls = torch.vstack(
                all_controls[i] + [torch.zeros_like(all_controls[i][0])] * (max_steps - min_steps)
            ).to(self.device)
            ctrl_idx_starts = torch.tensor([round(times[i][idx] / self.dt) for idx in idx_starts])
            assert all_ctrls.shape[0] > max_steps

            self.simulator.reset(
                act_len=act_lengths[idx_starts].clone().to(self.device),
            )

            curr_state = curr_state[idx_starts]

            dataset_idx = torch.full(
                (curr_state.shape[0], 1),
                data_dict['dataset_idx'][i],
                dtype=torch.int
            )

            gt_act_lens, ctrls = self._get_n_step_act_lens_ctrls(
                act_lengths[i], all_controls[i], ctrl_idx_starts, max_steps
            )

            states, _ = self.simulator.run(
                curr_state=curr_state,
                dt=self.dt,
                ctrls=ctrls,
                gt_act_lens=gt_act_lens,
                state_to_graph_kwargs={'dataset_idx': dataset_idx},
            )

            # pred_endpts = []
            # for k in tqdm.tqdm(range(max_steps)):
            #     curr_idx = ctrl_idx_starts + k
            #
            #     if self.use_gt_act_lens:
            #         for j, cable in enumerate(cables):
            #             next_act_len = act_lengths[curr_idx + 1, j: j + 1]
            #             cable.actuation_length = next_act_len.to(self.device)
            #         controls = None
            #     else:
            #         controls = all_ctrls[curr_idx].unsqueeze(-1)
            #
            #     curr_state, _ = self.simulator.step(
            #         curr_state,
            #         self.dt,
            #         ctrls=controls,
            #         state_to_graph_kwargs={'dataset_idx': dataset_idx},
            #     )
            #     if k > min_steps - 2:
            #         endpts = self._batch_compute_end_pts(curr_state)
            #         pred_endpts.append(endpts)

            pred_endpts = torch.concat([self._batch_compute_end_pts(s) for s in states[min_steps - 1:]], dim=2)
            pred_endpts = pred_endpts[torch.arange(curr_state.shape[0]), :, num_steps]
            pred_endpts = pred_endpts.unsqueeze(-1)

            gt_endpts = torch.vstack([
                e.reshape(1, -1, 1)
                for e in all_gt_endpts[i]
            ]).to(self.device)[idx_ends]

            loss = ((pred_endpts - gt_endpts) ** 2).mean().detach().item()

            com_loss, angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)
            com_loss, angle_loss = com_loss.detach().item(), angle_loss.detach().item()
            total_com_loss += com_loss
            total_angle_loss += angle_loss

            self.logger.info(f'Eval {n}-steps: {data_names[i]}, {loss}, {com_loss}, {angle_loss}')

        total_loss /= len(trajs) if len(trajs) > 0 else 1
        total_com_loss /= len(trajs) if len(trajs) > 0 else 1
        total_angle_loss /= len(trajs) if len(trajs) > 0 else 1

        self.logger.info(f'Avg {n}-steps: {total_loss}, {total_com_loss}, {total_angle_loss}')

        return total_loss

    def eval_rollout_fixed_ctrls(self, data_dict, prefix=''):
        states = data_dict['states']
        data_jsons = data_dict['data_jsons']
        all_act_lengths = data_dict['act_lengths']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']
        data_names = data_dict['names']
        all_dataset_idxs = data_dict['dataset_idx']

        device = self.device
        self.device = 'cpu'
        self.to('cpu')
        torch.set_num_threads(8)

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(states)):
            act_lengths = [torch.hstack(a) for a in all_act_lengths[i]]
            self.simulator.reset(
                act_lens=act_lengths[0].clone().to(self.device),
            )
            dataset_idx = torch.tensor([[all_dataset_idxs[i]]], dtype=torch.int)

            curr_state = states[i][0].clone().to(self.device)
            controls = torch.cat(all_controls[i], dim=-1).to(self.device) if not self.use_gt_act_lens else None
            gt_act_lens = torch.cat(act_lengths[1:], dim=-1).to(self.device) if self.use_gt_act_lens else None

            all_states, graphs, _ = self.simulator.run(
                curr_state=curr_state,
                dt=self.dt,
                ctrls=controls,
                gt_act_lens=gt_act_lens,
                state_to_graph_kwargs={'dataset_idx': dataset_idx},
                show_progress=True
            )

            idxs = [round(data_jsons[i][j]['time'] / self.dt) - 1
                    for j in range(1, len(data_jsons[i]))]

            pred_endpts = torch.vstack([
                self._batch_compute_end_pts(all_states[j])
                for j in idxs
            ])
            gt_endpts = torch.vstack([
                e.reshape(1, -1, 1)
                for e in all_gt_endpts[i][1:]
            ]).to(self.device)

            loss = ((pred_endpts - gt_endpts) ** 2).mean().detach().item()
            com_loss, mean_angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)

            self.logger.info(f"Eval Rollout: {data_names[i]}, "
                             f"{loss}, "
                             f"{com_loss}, "
                             f"{mean_angle_loss}")

            total_com_loss += com_loss
            total_angle_loss += mean_angle_loss

            # gt_vis_data = data_dict['vis_data'][i]
            # gt_poses = [np.array(d['pos']) for d in gt_vis_data]
            #
            # poses = [{
            #     'time': k * self.dt,
            #     'pos': np.concatenate([pred, gt])
            # } for k, (pred, gt) in enumerate(zip(pred_poses, gt_poses))]
            #
            # vis = MuJoCoVisualizer()
            # vis.set_xml_path(Path("mujoco_physics_engine/xml_models/3bar_new_platform_all_cables_vis_w_gt.xml"))
            # vis.set_camera("camera")
            # vis.data = poses
            # vis.visualize(
            #     Path(self.output_dir,
            #          f"{data_names[i]}_{self.target_dt}_epoch{self.epoch_num}.mp4"),
            #     0.01
            # )

        total_loss /= len(states) if len(states) > 0 else 1
        total_com_loss /= len(states) if len(states) > 0 else 1
        total_angle_loss /= len(states) if len(states) > 0 else 1
        self.device = device
        self.to(device)

        self.logger.info(f'Total loss: {total_loss}, com loss: {total_com_loss}, angle loss: {total_angle_loss}')

        return total_com_loss, total_angle_loss

    def compute_eval_n_steps_idxs(self, times, time_ahead, pct_tol=0.03, max_sample=200):
        min_dt, max_dt = time_ahead * (1 - pct_tol), time_ahead * (1 + pct_tol)

        idx_starts, idx_ends, tmp = [], [], []
        for i in range(len(times) - 1):
            t0 = times[i]
            best_, best_idx, best_dt = 99999., None, 99999.
            for j in range(i + 1, len(times)):
                t1 = times[j]
                dt = t1 - t0
                if min_dt <= dt <= max_dt <= (times[-1] - t1) and abs(dt - time_ahead) < best_:
                    best_, best_idx = abs(dt - time_ahead), j
                    best_dt = dt
            tmp.append(best_dt)
            if best_idx is not None:
                idx_starts.append(i)
                idx_ends.append(best_idx)

        idxs = list(range(len(idx_starts)))
        random.shuffle(idxs)
        idxs = idxs[:max_sample]
        idx_starts = [idx_starts[i] for i in idxs]
        idx_ends = [idx_ends[i] for i in idxs]

        return idx_starts, idx_ends

    def batch_sim_ctrls(self, batch) -> List[Graph]:
        batch_state = batch['x']
        controls = batch['ctrl']
        delta_t = batch['delta_t']
        act_lens = batch['act_len']
        next_act_lens = batch['next_act_lens']
        dataset_idx = batch['dataset_idx']
        time_gaps = batch['dt']
        # dl_speeds = batch['dl_speeds']

        self.simulator.reset(
            act_lens=act_lens[..., -1:].clone()
        )
        num_steps = int(torch.round(delta_t.max() / self.dt).cpu())

        cables = self.simulator.robot.actuated_cables.values()
        for i, cable in enumerate(cables):
            cable.actuation_length = act_lens[:, i: i + 1, -1:].clone()

        curr_state = batch_state[..., -1:].clone()
        if self.use_gt_act_lens:
            gt_act_lens, ctrls = [], None
            for i in range(self.num_steps_fwd):
                gt_act_lens.append(next_act_lens[:, :, i: i + 1].clone())
                ctrls = None
            gt_act_lens = torch.cat(gt_act_lens, dim=-1)
        else:
            gt_act_lens, ctrls = None, controls.clone()

        states, graphs = self.simulator.run(
            curr_state=curr_state,
            dt=self.dt,
            ctrls=ctrls,
            gt_act_lens=gt_act_lens,
            state_to_graph_kwargs={"dataset_idx": dataset_idx}
        )

        return graphs

    def run_one_epoch(self,
                      batches,
                      grad_required=True,
                      shuffle_data=False,
                      rot_aug=False) -> List[float]:
        if shuffle_data:
            random.shuffle(batches)

        total_loss, total_other_losses = 0.0, []
        num_train, curr_batch = 0, 0
        for batch in tqdm.tqdm(batches):
            if batch['x'].shape[0] != self.batch_size_per_step:
                continue
            curr_batch += 1

            batch = {k: v.to(self.device) for k, v in batch.items()}
            num_train += batch['x'].shape[0]

            if rot_aug:
                batch['x'], batch['y'] = self.rotate_data_aug(batch['x'], batch['y'])

            graphs = self.batch_sim_ctrls(batch)
            losses = self.compute_node_loss(graphs, batch['y'], self.dt, batch['mask'])

            # If gradient updates required, run backward pass
            if grad_required:
                self.backward(losses[0])

            total_loss += losses[0].detach().item() * batch['x'].shape[0]
            total_other_losses.append([
                l * batch['x'].shape[0] for l in losses[1:]
            ])

            if curr_batch % self.PRINT_STEP == 0:
                avg_other_losses = [
                    sum(l) / num_train
                    for l in zip(*total_other_losses)
                ]
                print(total_loss / num_train, avg_other_losses)

        total_loss /= num_train
        avg_other_losses = [
            sum(l) / num_train
            for l in zip(*total_other_losses)
        ]

        losses = [total_loss] + avg_other_losses

        return losses

    def _accum_dv_normalizer(self, gt_nodes_pos):
        p_vel = (gt_nodes_pos[:, 6:] - gt_nodes_pos[:, 3:-3]) / self.dt
        vel = (gt_nodes_pos[:, 3:-3] - gt_nodes_pos[:, :-6]) / self.dt
        gt_dv = p_vel - vel
        self.simulator.data_processor.normalizers['node_dv'](gt_dv)

    def compute_node_loss(self, graphs, gt_end_pts, dt, mask=None):
        def reshape_node_pos_tensor(pos):
            return pos.unsqueeze(1).reshape(pos.shape[0], -1, 3).transpose(1, 2)

        assert mask is not None, "Mask cannot be None"

        num_out_steps = self.simulator.num_out_steps \
            if hasattr(self.simulator, 'num_out_steps') else 1
        body_mask = graphs[0].body_mask.flatten()

        all_norm_gt_pos, all_norm_p_pos = [], []
        all_gt_pos, all_p_pos = [], []
        for i in range(len(graphs)):
            start = i * num_out_steps
            end = min(start + num_out_steps, gt_end_pts.shape[-1])

            gt_nodes_pos_lis = [graphs[i].pos[body_mask]]
            for j in range(start, end):
                end_pts = gt_end_pts[..., j:j + 1].reshape(-1, 6, 1)
                gt_pos, gt_quat = self.endpts2pos(end_pts[:, :3], end_pts[:, 3:])
                # gt_nodes_pos = self.simulator.data_processor.pose2node(
                #     torch.hstack([gt_pos, gt_quat]),
                #     augment_grnd=False
                # )
                single_gt_nodes_pos = self.simulator.data_processor.pose2node(
                    gt_pos, gt_quat, gt_end_pts.shape[0]
                )
                gt_nodes_pos_lis.append(single_gt_nodes_pos)

            if i == 0 and not self.load_sim:
                gt_prev_pos = gt_nodes_pos_lis[0] - graphs[i].vel[body_mask] * self.dt
                gt_node_pos = torch.hstack([gt_prev_pos] + gt_nodes_pos_lis)
                self._accum_dv_normalizer(gt_node_pos)

            # gt_prev_pos = gt_nodes_pos_lis[0] - graphs[i].vel[body_mask] * self.dt
            # gt_node_pos = torch.hstack([gt_prev_pos] + gt_nodes_pos_lis)
            # p_vel = (gt_node_pos[:, 6:] - gt_node_pos[:, 3:-3]) / self.dt
            # vel = (gt_node_pos[:, 3:-3] - gt_node_pos[:, :-6]) / self.dt
            # gt_dv = p_vel - vel
            #
            # norm_gt_dv = self.simulator.data_processor.normalizers['node_dv'](gt_dv)
            # norm_pred_dv = graphs[i].decode_output[body_mask]

            gt_nodes_pos = torch.hstack(gt_nodes_pos_lis[1:])

            # pred_dv = self.simulator.data_processor.normalizers['node_dv'].inverse(norm_pred_dv)
            #
            # pred_pvel, pred_ppos = [], []
            # pred_vel, pred_pos = graphs[i].vel[body_mask], graphs[i].pos[body_mask]
            # for h in range(num_out_steps):
            #     pred_vel = pred_vel + pred_dv[:, h * 3 : (h + 1) * 3]
            #     pred_pos = pred_pos + self.dt * pred_vel
            #     pred_ppos.append(pred_pos.clone())
            #
            # pred_node_pos = torch.hstack(pred_ppos)

            dim = gt_nodes_pos.shape[1]
            pred_node_pos = graphs[i].p_pos[body_mask].transpose(1, 2).reshape(gt_nodes_pos.shape[0], -1)[:, :dim]

            dv_normalizer = self.simulator.data_processor.normalizers['node_dv']
            norm_gt_pos = gt_nodes_pos / dv_normalizer.std_w_eps[:, :dim] / self.dt
            norm_p_pos = pred_node_pos / dv_normalizer.std_w_eps[:, :dim] / self.dt

            # tmp_loss = self.loss_fn(norm_p_pos, norm_gt_pos).detach().item()
            # tmp_loss2 = self.loss_fn(norm_gt_dv, norm_pred_dv).detach().item()

            all_gt_pos.append(reshape_node_pos_tensor(gt_nodes_pos))
            all_p_pos.append(reshape_node_pos_tensor(pred_node_pos))
            all_norm_gt_pos.append(reshape_node_pos_tensor(norm_gt_pos))
            all_norm_p_pos.append(reshape_node_pos_tensor(norm_p_pos))

        num_nodes = all_norm_p_pos[0].shape[0] // mask.shape[0]
        mask = mask.repeat_interleave(num_nodes, dim=0)

        all_norm_gt_pos = torch.concat(all_norm_gt_pos, dim=-1) * mask
        all_norm_p_pos = torch.concat(all_norm_p_pos, dim=-1) * mask
        loss = self.loss_fn(all_norm_p_pos, all_norm_gt_pos)

        with torch.no_grad():
            all_gt_pos = torch.concat(all_gt_pos, dim=-1) * mask
            all_p_pos = torch.concat(all_p_pos, dim=-1) * mask
            pos_loss = self.loss_fn(all_gt_pos, all_p_pos)

        return loss, pos_loss.detach().item()

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu", weights_only=False)
            sim.reset()
            sim.cpu()
            print("Loaded simulator")
        else:
            sim_config_cpy = deepcopy(self.sim_config)
            sim_config_cpy.pop('gravity')
            sim_config_cpy.pop('contact_params')
            # sim = FastTensegrityMultiSimGNNSimulator(**sim_config_cpy, num_sims=10)
            sim = TensegrityMultiSimGNNSimulator(**sim_config_cpy, num_sims=10)

        sim.data_processor.training = True
        return sim


class RealTensegrityPhysicsTrainingEngine(RealTensegrity5DGNNTrainingEngine):

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu", weights_only=False)
            sim.reset()
            sim.cpu()
            self.logger.info("Loaded simulator")
        else:
            sim = Tensegrity5dRobotSimulator(
                self.sim_config['tensegrity_cfg'],
                self.sim_config['gravity'],
                self.sim_config['contact_params'],
                learn_contact_params=True
            )

        return sim

    def batch_sim_ctrls(self, batch) -> List[Graph]:
        batch_state = batch['x']
        controls = batch['ctrl']
        act_lens = batch['act_len']
        next_act_lens = batch['next_act_lens']
        motor_omegas = batch['motor_omega']

        self.simulator.reset(
            act_lens=act_lens[..., -1:].clone(),
            motor_speeds=motor_omegas[..., -1:].clone()
        )

        curr_state = batch_state[..., -1:].clone()

        if self.use_gt_act_lens:
            gt_act_lens, ctrls = [], None
            for i in range(self.num_steps_fwd):
                gt_act_lens.append(next_act_lens[:, :, i: i + 1].clone())
                ctrls = None
            gt_act_lens = torch.cat(gt_act_lens, dim=-1)
        else:
            gt_act_lens, ctrls = None, controls.clone()

        states, graphs = self.simulator.run(
            curr_state=curr_state,
            dt=self.dt,
            ctrls=ctrls,
            gt_act_lens=gt_act_lens,
        )

        return states

    def compute_node_loss(self, states, gt_endpts, dt, mask=None):
        assert mask is not None

        pred_end_pts = torch.cat([
            self._batch_compute_end_pts(s)
            for s in states
        ], dim=2)
        loss = self.loss_fn(
            pred_end_pts * mask / (dt ** 1),
            gt_endpts * mask / (dt ** 1)
        )

        end_pts_loss = self.loss_fn(pred_end_pts, gt_endpts).detach().item()
        return loss, end_pts_loss

    def backward(self, loss: torch.Tensor) -> None:
        (loss / self.num_grad_accum).backward()

        if self.curr_accum_step >= self.num_grad_accum:
            self.optimizer.step()
            # torch.nn.utils.clip_grad_norm_(self.parameters(), max_norm=100)
            self.optimizer.zero_grad()

            if 'param_ranges' in self.config:
                param_ranges = self.config['param_ranges']
                for p_name, param in self.state_dict().items():
                    if p_name in param_ranges:
                        lower, upper = param_ranges[p_name]
                        param.data.clamp_(lower, upper)

            self.curr_accum_step = 1
        else:
            self.curr_accum_step += 1

    def run_one_epoch(self,
                      dataloader,
                      grad_required=True,
                      rot_aug=False) -> List[float]:
        losses = super().run_one_epoch(dataloader, grad_required=grad_required, rot_aug=rot_aug)
        print([(k.split('.')[-1], v.detach().item()) for k, v in self.state_dict().items() if 'contact' in k])
        print(
            f'restitution: {self.state_dict()["simulator.collision_resp_gen.contact_params.restitution"].detach().item()} '
            f'baumgarte: {self.state_dict()["simulator.collision_resp_gen.contact_params.baumgarte"].detach().item()} '
            f'friction: {self.state_dict()["simulator.collision_resp_gen.contact_params.friction"].detach().item()} '
            f'friction_damping: {self.state_dict()["simulator.collision_resp_gen.contact_params.friction_damping"].detach().item()} '
            f'rolling_friction: {self.state_dict()["simulator.collision_resp_gen.contact_params.rolling_friction"].detach().item()}')

        contact_params = self.simulator.collision_resp_gen.contact_params
        print(f'restitution: {contact_params.restitution.detach().item()} '
              f'baumgarte: {contact_params.baumgarte.detach().item()} '
              f'friction: {contact_params.friction.detach().item()} '
              f'friction_damping: {contact_params.friction_damping.detach().item()} '
              f'rolling_friction: {contact_params.rolling_friction.detach().item()}')

        return losses

    def eval_n_step_aheads(self, n, data_dict, num_samples=100):
        device = self.device
        self.to('cpu')

        trajs = data_dict['states']
        all_act_lengths = data_dict['act_lengths']
        all_motor_omegas = data_dict['motor_omegas']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']
        times = data_dict['times']
        data_names = data_dict['names']

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(trajs)):
            states = [trajs[i][0].clone() for _ in range(self.num_hist - 1)] + trajs[i]
            act_lengths = torch.vstack([torch.hstack(a) for a in all_act_lengths[i]])
            motor_speeds = torch.vstack([m for m in all_motor_omegas[i]])

            curr_state = torch.vstack(states).to(self.device)

            idx_starts, idx_ends = self.compute_eval_n_steps_idxs(
                times[i], n * self.dt, max_sample=num_samples
            )
            num_steps = [round((times[i][e] - times[i][s]) / self.dt)
                         for s, e in zip(idx_starts, idx_ends)]
            max_steps, min_steps = max(num_steps), min(num_steps)
            num_steps = [num - min_steps for num in num_steps]

            all_ctrls = torch.vstack(
                all_controls[i] + [torch.zeros_like(all_controls[i][0])] * (max_steps - min_steps)
            ).to(self.device)
            ctrl_idx_starts = torch.tensor([round(times[i][idx] / self.dt) for idx in idx_starts])
            assert all_ctrls.shape[0] > max_steps
            curr_state = curr_state[idx_starts]

            gt_act_lens, ctrls = self._get_n_step_act_lens_ctrls(
                act_lengths[i], all_controls[i], ctrl_idx_starts, max_steps
            )
            start_act_lens = act_lengths[idx_starts].clone().to(self.device)
            start_motor_speeds = motor_speeds[idx_starts].clone().to(self.device)

            pred_endpts = []
            for k in tqdm.tqdm(range(curr_state.shape[0])):
                self.simulator.reset(
                    act_lens=start_act_lens[k: k + 1],
                    motor_speeds=start_motor_speeds[k: k + 1],
                )

                states, _ = self.simulator.run(
                    curr_state=curr_state[k: k + 1],
                    dt=self.dt,
                    ctrls=ctrls[k: k + 1],
                    gt_act_lens=gt_act_lens,
                )

                endpts = torch.concat([self._batch_compute_end_pts(s) for s in states[min_steps - 1:]], dim=2)
                endpts = endpts[torch.arange(1), :, num_steps[k]]
                endpts = endpts.unsqueeze(-1)
                pred_endpts.append(endpts)

            pred_endpts = torch.vstack(pred_endpts)
            gt_endpts = torch.vstack([
                e.reshape(1, -1, 1)
                for e in all_gt_endpts[i]
            ]).to(self.device)[idx_ends]

            loss = ((pred_endpts - gt_endpts) ** 2).mean().detach().item()

            com_loss, angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)
            # com_loss, angle_loss = com_loss.detach().item(), angle_loss.detach().item()
            total_com_loss += com_loss
            total_angle_loss += angle_loss

            self.logger.info(f'Eval {n}-steps: {data_names[i]}, {loss}, {com_loss}, {angle_loss}')

        total_loss /= len(trajs) if len(trajs) > 0 else 1
        total_com_loss /= len(trajs) if len(trajs) > 0 else 1
        total_angle_loss /= len(trajs) if len(trajs) > 0 else 1

        self.logger.info(f'Avg {n}-steps: {total_loss}, {total_com_loss}, {total_angle_loss}')
        self.to(device)

        return total_loss

    def eval_n_step_aheads2(self, n, data_dict, num_samples=100):
        super().eval_n_step_aheads()
        trajs = data_dict['states']
        act_lengths = data_dict['act_lengths']
        motor_omegas = data_dict['motor_omegas']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(trajs)):
            states = [trajs[i][0].clone() for _ in range(self.num_hist - 1)] + trajs[i]

            idxs = list(range(len(states) - self.num_hist - n - 1))
            random.shuffle(idxs)
            idxs = idxs[:num_samples]

            curr_states = torch.vstack(states[self.num_hist - 1:-n]).to(self.device)[idxs]
            gt_act_lens, ctrls = self._get_n_step_act_lens_ctrls(act_lengths[i], all_controls[i], idxs, n)

            last_states = []
            for j, idx in enumerate(tqdm.tqdm(idxs)):
                self.simulator.reset(
                    act_lens=torch.vstack(act_lengths[i][:-n + 1])[idx: idx + 1].to(self.device),
                    motor_speeds=torch.vstack(motor_omegas[i][:-n + 1])[idx: idx + 1].to(self.device)
                )

                curr_state = curr_states[j: j + 1]
                gt_act_lens_, ctrls_ = gt_act_lens[j: j + 1], ctrls[j: j + 1]

                states, _ = self.simulator.run(
                    curr_state=curr_state,
                    dt=self.dt,
                    ctrls=ctrls_,
                    num_steps=n,
                    gt_act_lens=gt_act_lens_,
                )

                last_states.append(states[-1].clone())

            last_states = torch.vstack(last_states)
            pred_endpts = self._batch_compute_end_pts(last_states)
            gt_endpts = torch.vstack([
                torch.hstack(e)
                for e in all_gt_endpts[i][n:]
            ]).to(self.device)[idxs]

            loss = self.loss_fn(pred_endpts, gt_endpts).detach().item()
            total_loss += loss

            com_loss, angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)
            total_com_loss += com_loss
            total_angle_loss += angle_loss

            self.logger.info(f'{loss}, {com_loss}, {angle_loss}')

        total_loss /= len(trajs) if len(trajs) > 0 else 1
        total_com_loss /= len(trajs) if len(trajs) > 0 else 1
        total_angle_loss /= len(trajs) if len(trajs) > 0 else 1

        self.logger.info(f'Avg {n}-step: {total_loss}, {total_com_loss}, {total_angle_loss}')

        return total_loss, total_com_loss, total_angle_loss

    def eval_rollout_fixed_ctrls(self, data_dict, prefix=''):
        states = data_dict['states']
        data_jsons = data_dict['data_jsons']
        all_act_lengths = data_dict['act_lengths']
        all_motor_omegas = data_dict['motor_omegas']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']
        data_names = data_dict['names']

        device = self.device
        self.device = 'cpu'
        self.to('cpu')
        torch.set_num_threads(8)

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(states)):
            act_lengths = [torch.hstack(a) for a in all_act_lengths[i]]
            self.simulator.reset(
                act_lens=act_lengths[0].clone().to(self.device),
                motor_speeds=all_motor_omegas[i][0].clone().to(self.device),
            )

            curr_state = states[i][0].clone().to(self.device)
            controls = torch.stack(all_controls[i], dim=-1).to(self.device) if not self.use_gt_act_lens else None
            gt_act_lens = torch.cat(act_lengths[1:], dim=-1).to(self.device) if self.use_gt_act_lens else None

            all_states, _ = self.simulator.run(
                curr_state=curr_state,
                dt=self.dt,
                ctrls=controls,
                gt_act_lens=gt_act_lens
            )

            idxs = [round(data_jsons[i][j]['time'] / self.dt) - 1
                    for j in range(1, len(data_jsons[i]))]

            pred_endpts = torch.vstack([
                self._batch_compute_end_pts(all_states[j])
                for j in idxs
            ])
            gt_endpts = torch.vstack([
                e.reshape(1, -1, 1)
                for e in all_gt_endpts[i][1:]
            ]).to(self.device)

            loss = ((pred_endpts - gt_endpts) ** 2).mean().detach().item()
            com_loss, mean_angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)

            self.logger.info(f"Eval Rollout: {data_names[i]}, "
                             f"{loss}, "
                             f"{com_loss}, "
                             f"{mean_angle_loss}")

            total_com_loss += com_loss
            total_angle_loss += mean_angle_loss

            # gt_vis_data = data_dict['vis_data'][i]
            # gt_poses = [np.array(d['pos']) for d in gt_vis_data]
            #
            # poses = [{
            #     'time': k * self.dt,
            #     'pos': np.concatenate([pred, gt])
            # } for k, (pred, gt) in enumerate(zip(pred_poses, gt_poses))]
            #
            # vis = MuJoCoVisualizer()
            # vis.set_xml_path(Path("mujoco_physics_engine/xml_models/3bar_new_platform_all_cables_vis_w_gt.xml"))
            # vis.set_camera("camera")
            # vis.data = poses
            # vis.visualize(
            #     Path(self.output_dir,
            #          f"{data_names[i]}_{self.target_dt}_epoch{self.epoch_num}.mp4"),
            #     0.01
            # )

        total_loss /= len(states) if len(states) > 0 else 1
        total_com_loss /= len(states) if len(states) > 0 else 1
        total_angle_loss /= len(states) if len(states) > 0 else 1
        self.device = device
        self.to(device)

        self.logger.info(f'Total loss: {total_loss}, com loss: {total_com_loss}, angle loss: {total_angle_loss}')

        return total_com_loss, total_angle_loss

    def compute_init_losses(self):
        train_loss = self.run_one_epoch(
            self.train_dataloader,
            grad_required=False,
            rot_aug=True
        )

        val_loss = self.run_one_epoch(
            self.val_dataloader,
            grad_required=False
        )

        # if True:
        if self.load_sim:
            # train_rollout_loss = self.evaluate_rollouts(
            #     self.train_data_dict
            # )
            # self.logger.info("Avg rollout loss:", train_rollout_loss)
            self.simulator.eval()
            val_rollout_loss = self.evaluate_rollouts(
                self.val_data_dict
            )
            self.simulator.train()
            self.logger.info(val_rollout_loss)

        losses = (
            train_loss,
            val_loss,
        )

        return losses


class RealTensegrity5DRecurrentGNNTrainingEngine(RealTensegrity5DGNNTrainingEngine):

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu", weights_only=False)
            sim.reset()
            sim.cpu()
            print("Loaded simulator")
        else:
            sim_config_cpy = deepcopy(self.sim_config)
            sim_config_cpy.pop('gravity')
            sim_config_cpy.pop('contact_params')
            sim = TensegrityMultiSimRecurrentGNNSimulator(**sim_config_cpy, num_sims=10)

        sim.data_processor.training = True
        return sim


class RealTensegrity5DRecurrentMotorGNNTrainingEngine(RealTensegrity5DGNNTrainingEngine):

    def __init__(self,
                 training_config: Dict,
                 criterion: _Loss,
                 dt: Union[float, torch.Tensor],
                 logger: logging.Logger):
        self.num_ctrls_hist = training_config.get('num_ctrls_hist', 20)
        super().__init__(training_config, criterion, dt, logger)
        self.use_gt_act_lens = False

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu", weights_only=False)
            sim.reset()
            sim.cpu()
            print("Loaded simulator")
        else:
            sim_config_cpy = deepcopy(self.sim_config)
            sim_config_cpy.pop('gravity')
            sim_config_cpy.pop('contact_params')
            sim = MultiSimMultiStepMotorTensegrityGNNSimulator(
                **sim_config_cpy,
                num_sims=10,
                num_ctrls_hist=self.num_ctrls_hist,
                torch_compile=False
            )

        return sim

    def _get_dataset(self, data_dict):
        real_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' in n]
        sim_idxs = [i for i, n in enumerate(data_dict['names']) if 'real' not in n]

        real_data_dict = {k: [v[i] for i in real_idxs] for k, v in data_dict.items()}
        sim_data_dict = {k: [v[i] for i in sim_idxs] for k, v in data_dict.items()}

        return RealMultiSimTensegrityDataset(
            real_data_dict,
            sim_data_dict,
            self.min_dt,
            self.max_dt,
            self.target_dt,
            self.mix_ratio,
            self.num_steps_fwd,
            self.num_hist,
            self.num_ctrls_hist,
            self.dt,
            self.dtype
        )

    def compute_cable_dl_loss(self, graphs, gt_cable_dls):
        pred_cable_dls = torch.hstack([g.act_cable_dl for g in graphs])
        gt_act_cable_dls = gt_cable_dls.repeat_interleave(2, dim=1).reshape(-1, gt_cable_dls.shape[-1])

        n_out = self.simulator.num_out_steps
        num_steps = math.ceil(round(gt_act_cable_dls.shape[1] / n_out, 2))
        normalizer = self.simulator.data_processor.normalizers['cable_dl']

        norm_gt_cable_dls = []
        for i in range(num_steps):
            cable_dls = gt_act_cable_dls[:, i * n_out: (i + 1) * n_out]
            dim = cable_dls.shape[1]
            step_diff = n_out - dim
            if step_diff > 0:
                pad = torch.zeros_like(cable_dls[:, :1]).repeat(1, step_diff)
                cable_dls = torch.hstack([cable_dls, pad])
            norm_cable_dls = normalizer(cable_dls)
            norm_gt_cable_dls.append(norm_cable_dls[:, :dim])
        norm_gt_cable_dls = torch.hstack(norm_gt_cable_dls)

        # norm_gt_cable_dls = torch.hstack([
        #     normalizer(gt_act_cable_dls[:, i * n_out: (i + 1) * n_out])
        #     for i in range(num_steps)
        # ])
        # norm_gt_cable_dls = torch.hstack([
        #     normalizer(gt_act_cable_dls.transpose(1, 2))
        #     for i in range(num_steps)
        # ])

        act_cable_mask = graphs[0].cable_actuated_mask.flatten()
        norm_pred_cable_dls = torch.hstack([g.cable_decode_output[act_cable_mask] for g in graphs])

        norm_cable_dl_loss = self.loss_fn(
            norm_pred_cable_dls[:, :gt_act_cable_dls.shape[1]],
            norm_gt_cable_dls
        )
        cable_dl_loss = self.loss_fn(
            pred_cable_dls[:, :gt_act_cable_dls.shape[1]],
            gt_act_cable_dls
        ).detach().item()

        return norm_cable_dl_loss, cable_dl_loss

    def compute_cable_dl_loss2(self, graphs, gt_rel_cable_act_lens):
        num_act = len(self.simulator.robot.actuated_cables)
        num_nonact = len(self.simulator.robot.non_actuated_cables)
        normalizer = self.simulator.data_processor.normalizers['cable_dl']

        pred_cable_dls = torch.stack([g.cable_dl for g in graphs], dim=2)
        pred_cumm_cable_lens = torch.cumsum(pred_cable_dls, dim=2)[..., :2 * num_act, :-1]

        gt_act_cable_dls = gt_rel_cable_act_lens.repeat_interleave(2, axis=1)
        gt_act_cable_dls[..., 1:] = gt_act_cable_dls[..., 1:] - pred_cumm_cable_lens
        # gt_act_cable_dls = gt_act_cable_dls - torch.stack([g.mean_cable_dl for g in graphs], dim=2)
        gt_nonact_cable_dls = zeros(
            (gt_act_cable_dls.shape[0], 2 * num_nonact, gt_act_cable_dls.shape[-1]),
            ref_tensor=gt_act_cable_dls
        )
        gt_cable_dls = torch.hstack([gt_act_cable_dls, gt_nonact_cable_dls])

        norm_pred_cable_dls = torch.stack(
            [g.cable_decode_output.reshape(gt_cable_dls.shape[0], -1) for g in graphs], dim=2
        )
        # norm_pred_cable_dls = (norm_pred_cable_dls
        #                        .reshape(-1, 2)
        #                        .mean(dim=1, keepdim=True)
        #                        .reshape(gt_cable_dls.shape))

        norm_gt_cable_dls = torch.zeros_like(gt_cable_dls)
        norm_gt_cable_dls[:, ::2] = normalizer(gt_cable_dls[:, ::2])
        norm_gt_cable_dls[:, 1::2] = normalizer(gt_cable_dls[:, 1::2])

        norm_cable_dl_loss = self.loss_fn(norm_pred_cable_dls, norm_gt_cable_dls)
        cable_dl_loss = self.loss_fn(
            pred_cable_dls[:, :2 * num_act],
            gt_cable_dls[:, :2 * num_act]
        ).detach().item()

        return norm_cable_dl_loss, cable_dl_loss

    def run_one_epoch(self,
                      batches,
                      grad_required=True,
                      shuffle_data=False,
                      rot_aug=False) -> List[float]:
        if grad_required:
            self.simulator.train()
        else:
            self.simulator.eval()

        if shuffle_data:
            random.shuffle(batches)

        total_loss, total_other_losses = 0.0, []
        num_train, curr_batch = 0, 0
        for batch in tqdm.tqdm(batches):
            curr_batch += 1

            batch = {k: v.to(self.device) for k, v in batch.items()}
            num_train += batch['x'].shape[0]

            if rot_aug:
                batch['x'], batch['y'] = self.rotate_data_aug(batch['x'], batch['y'])

            graphs = self.batch_sim_ctrls(batch)
            losses = self.compute_node_loss(graphs, batch['y'], self.dt, batch['mask'])

            # cable_dls = batch['next_act_lens'] - batch['act_len']
            act_lens, next_act_lens = batch['act_len'], batch['next_act_lens']
            cable_dls = next_act_lens - torch.concat([act_lens, next_act_lens[..., :-1]], dim=2)
            norm_cable_loss, cable_loss = self.compute_cable_dl_loss(graphs, cable_dls)

            backward_loss = losses[0] + (4 / self.num_steps_fwd) * norm_cable_loss
            losses = [backward_loss, norm_cable_loss.detach().item(), cable_loss] + [l for l in losses[1:]]

            # If gradient updates required, run backward pass
            if grad_required:
                self.backward(losses[0])
                # self.backward(backward_loss)

            total_loss += losses[0].detach().item() * batch['x'].shape[0]
            total_other_losses.append([
                l * batch['x'].shape[0] for l in losses[1:]
            ])

            if curr_batch % self.PRINT_STEP == 0:
                avg_other_losses = [
                    sum(l) / num_train
                    for l in zip(*total_other_losses)
                ]
                self.logger.info(f"{total_loss / num_train} {avg_other_losses}")

        total_loss /= num_train
        avg_other_losses = [
            sum(l) / num_train
            for l in zip(*total_other_losses)
        ]

        losses = [total_loss] + avg_other_losses

        return losses

    def batch_sim_ctrls(self, batch) -> List[Graph]:
        batch_state = batch['x']
        controls = batch['ctrl']
        delta_t = batch['delta_t']
        act_lens = batch['act_len']
        next_act_lens = batch['next_act_lens']
        dataset_idx = batch['dataset_idx']
        time_gaps = batch['dt']
        # dl_speeds = batch['dl_speeds']

        self.simulator.reset(
            act_lens=act_lens[..., -1:].clone(),
            ctrls_hist=batch['ctrls_hist']
        )
        num_steps = int(torch.round(delta_t.max() / self.dt).cpu())

        curr_state = batch_state[..., -1:].clone()
        ctrls = controls.clone()

        states, graphs, _ = self.simulator.run(
            curr_state=curr_state,
            dt=self.dt,
            ctrls=ctrls,
            state_to_graph_kwargs={"dataset_idx": dataset_idx}
        )

        return graphs

    def eval_n_step_aheads(self, n, data_dict, num_samples=1000):
        trajs = data_dict['states']
        all_act_lengths = data_dict['act_lengths']
        all_controls = data_dict['controls']
        all_gt_endpts = data_dict['gt_end_pts']
        times = data_dict['times']
        data_names = data_dict['names']

        total_loss = 0.0
        total_com_loss, total_angle_loss = 0.0, 0.0
        for i in range(len(trajs)):
            states = [trajs[i][0].clone() for _ in range(self.num_hist - 1)] + trajs[i]
            act_lengths = torch.vstack([torch.hstack(a) for a in all_act_lengths[i]]).to(self.device)

            curr_state = torch.vstack(states).to(self.device)

            idx_starts, idx_ends = self.compute_eval_n_steps_idxs(
                times[i], n * self.dt, max_sample=num_samples
            )
            num_steps = [round((times[i][e] - times[i][s]) / self.dt)
                         for s, e in zip(idx_starts, idx_ends)]
            max_steps, min_steps = max(num_steps), min(num_steps)
            num_steps = [num - min_steps for num in num_steps]

            all_ctrls = torch.vstack(
                all_controls[i] + [torch.zeros_like(all_controls[i][0])] * (max_steps - min_steps)
            ).to(self.device)
            ctrl_idx_starts = torch.tensor([round(times[i][idx] / self.dt) for idx in idx_starts])
            assert all_ctrls.shape[0] > max_steps

            curr_state = curr_state[idx_starts]

            padded_controls = torch.vstack([torch.zeros_like(all_ctrls[:self.num_ctrls_hist]), all_ctrls])
            ctrl_hist = torch.cat([padded_controls[ctrl_idx_starts + j] for j in range(self.num_ctrls_hist)], dim=2)

            self.simulator.reset(
                act_lens=act_lengths[idx_starts].clone(),
                ctrls_hist=ctrl_hist
            )

            dataset_idx = torch.full(
                (curr_state.shape[0], 1),
                data_dict['dataset_idx'][i],
                dtype=torch.int,
                device=self.device
            )

            gt_act_lens, ctrls = self._get_n_step_act_lens_ctrls(
                act_lengths[i], all_controls[i], ctrl_idx_starts, max_steps
            )

            states, _, _ = self.simulator.run(
                curr_state=curr_state,
                dt=self.dt,
                ctrls=ctrls,
                gt_act_lens=gt_act_lens,
                state_to_graph_kwargs={'dataset_idx': dataset_idx},
                show_progress=True
            )

            # pred_endpts = []
            # for k in tqdm.tqdm(range(max_steps)):
            #     curr_idx = ctrl_idx_starts + k
            #
            #     if self.use_gt_act_lens:
            #         for j, cable in enumerate(cables):
            #             next_act_len = act_lengths[curr_idx + 1, j: j + 1]
            #             cable.actuation_length = next_act_len.to(self.device)
            #         controls = None
            #     else:
            #         controls = all_ctrls[curr_idx].unsqueeze(-1)
            #
            #     curr_state, _ = self.simulator.step(
            #         curr_state,
            #         self.dt,
            #         ctrls=controls,
            #         state_to_graph_kwargs={'dataset_idx': dataset_idx},
            #     )
            #     if k > min_steps - 2:
            #         endpts = self._batch_compute_end_pts(curr_state)
            #         pred_endpts.append(endpts)

            pred_endpts = torch.concat([self._batch_compute_end_pts(s) for s in states[min_steps - 1:]], dim=2)
            pred_endpts = pred_endpts[torch.arange(curr_state.shape[0]), :, num_steps]
            pred_endpts = pred_endpts.unsqueeze(-1)

            gt_endpts = torch.vstack([
                e.reshape(1, -1, 1)
                for e in all_gt_endpts[i]
            ]).to(self.device)[idx_ends]

            loss = ((pred_endpts - gt_endpts) ** 2).mean().detach().item()
            total_loss += loss

            com_loss, angle_loss = self.compute_com_and_rot_loss(pred_endpts, gt_endpts)
            com_loss, angle_loss = com_loss, angle_loss
            total_com_loss += com_loss
            total_angle_loss += angle_loss

            self.logger.info(f'Eval {n}-steps: {data_names[i]}, {loss}, {com_loss}, {angle_loss}')

        total_loss /= len(trajs) if len(trajs) > 0 else 1
        total_com_loss /= len(trajs) if len(trajs) > 0 else 1
        total_angle_loss /= len(trajs) if len(trajs) > 0 else 1

        self.logger.info(f'Avg {n}-steps: {total_loss}, {total_com_loss}, {total_angle_loss}')

        return total_loss
