import json
from pathlib import Path
import random

import torch
from matplotlib import pyplot as plt
from torch import nn
from torch import optim

from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE


def norm_end_pts(end_pts, rod_length=2.95):
    end_pts = end_pts.reshape(end_pts.shape[0], -1, 3).transpose(1, 2)

    com = end_pts.mean(dim=2, keepdim=True)
    end_pts = end_pts - com
    end_pts = end_pts / rod_length

    left = end_pts[..., ::2].mean(dim=2, keepdim=True)
    right = end_pts[..., 1::2].mean(dim=2, keepdim=True)
    prin = right - left
    prin[:, 2] = 0
    prin = prin / prin.norm(dim=1, keepdim=True)

    x = torch.tensor([1, 0, 0], dtype=DEFAULT_DTYPE, device=prin.device).reshape(1, 3, 1)
    rot_dir = torch.cross(prin, x, dim=1)
    angle = torch.linalg.vecdot(prin, x, dim=1).unsqueeze(1)
    angle = torch.acos(torch.clamp(angle, -0.9999999, 0.9999999)) / 2
    q = torch.hstack([torch.cos(angle), rot_dir * torch.sin(angle)])

    end_pts = torch_quaternion.rotate_vec_quat(q, end_pts)

    return end_pts


class MotorModel(nn.Module):

    def __init__(self,
                 n_in,
                 n_out,
                 hidden_size,
                 dropout_rate,
                 rest_len_stats,
                 rod_length):
        super(MotorModel, self).__init__()

        self.mean_rest_lens, self.std_rest_lens = rest_len_stats
        self.rod_length = rod_length

        self.motor_model = nn.Sequential(
            nn.Linear(n_in, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(hidden_size, n_out),
        )

    def to(self, device):
        super(MotorModel, self).to(device)
        self.mean_rest_lens = self.mean_rest_lens.to(device)
        self.std_rest_lens = self.std_rest_lens.to(device)

        self.motor_model = self.motor_model.to(device)

        return self

    def forward(self, x):
        end_pts = norm_end_pts(x[:, :18].reshape(x.shape[0], -1, 3).transpose(1, 2)).reshape(x.shape[0], -1)
        rest_lens = (x[:, 18:24] - self.mean_rest_lens) / self.std_rest_lens

        x = torch.hstack([end_pts, rest_lens, x[:, 24:]])
        pred = self.motor_model(x)

        return pred


class MotorTrainingEngine(nn.Module):

    def __init__(self, cfg):
        super(MotorTrainingEngine, self).__init__()
        self.train_cfg = cfg
        self.device = 'cpu'

        self.num_hist = cfg['num_hist']
        self.batch_size = cfg['batch_size']
        self.eval_step = cfg['eval_step'] if 'eval_step' in cfg else 50
        self.output_dir = Path(cfg['output_dir'])
        self.output_dir.mkdir(exist_ok=True)

        self.best_val_loss = 1e10
        self.best_rollout_loss = 1e10

        self.mean_rest_lens, self.std_rest_lens = 0., 1.
        self.init_data()

        # endpts: 18, rest_lengths: 6, ctrls: num_hist * 6
        n_in, n_out = 18 + 6 + self.num_hist * 6, 6
        hidden_size = 512
        dropout_rate = 0.1

        self.motor_model = MotorModel(
            n_in,
            n_out,
            hidden_size,
            dropout_rate,
            (self.mean_rest_lens, self.std_rest_lens),
            2.95
        )
        if self.train_cfg['load_model']:
            model_weights = torch.load(cfg['load_model_path'])
            self.motor_model.load_state_dict(model_weights)

        self.optimizer = optim.Adam(self.motor_model.parameters(),
                                    lr=cfg['learning_rate'])
        self.loss_fn = nn.MSELoss()

    def init_data(self):
        train_data_paths = self.train_cfg['train_data_paths']
        val_data_paths = self.train_cfg['val_data_paths']

        train_data = []
        for p in train_data_paths:
            gt_data = json.load(Path(p, 'processed_data.json').open('r'))
            extra_data = json.load(Path(p, 'extra_state_data.json').open('r'))
            data = [{**g, **e} for g, e in zip(gt_data, extra_data)]
            train_data.append(data)
        train_data = self.shift_ctrls(train_data)

        val_data = []
        for p in val_data_paths:
            gt_data = json.load(Path(p, 'processed_data.json').open('r'))
            extra_data = json.load(Path(p, 'extra_state_data.json').open('r'))
            data = [{**g, **e} for g, e in zip(gt_data, extra_data)]
            val_data.append(data)
        val_data = self.shift_ctrls(val_data)

        self.train_batches, self.train_data_dict = self.build_batches(train_data)
        self.val_batches, self.val_data_dict = self.build_batches(val_data)

        self.train_data_dict['names'] = [n.split('/')[-2] for n in self.train_cfg['train_data_paths']]
        self.val_data_dict['names'] = [n.split('/')[-2] for n in self.train_cfg['val_data_paths']]

        self.mean_rest_lens = torch.vstack(
            [d for d in self.train_data_dict['rest_lengths']]
        ).mean(dim=0, keepdim=True)
        self.std_rest_lens = torch.vstack(
            [d ** 2 for d in self.train_data_dict['rest_lengths']]
        ).mean(dim=0, keepdim=True)
        self.std_rest_lens = torch.sqrt(self.std_rest_lens - self.mean_rest_lens ** 2)

    def to(self, device):
        super().to(device)
        self.device = device

        self.motor_model = self.motor_model.to(device)

        return self

    def shift_ctrls(self, data_jsons):
        for data in data_jsons:
            for i in range(len(data) - 1):
                t0 = data[i]['time']
                t1 = data[i + 1]['time']

                curr_rest_lens = data[i]['rest_lengths']
                next_rest_lens = data[i + 1]['rest_lengths']

                change_rates = [(r1 - r0) / (t1 - t0)
                                for r0, r1 in zip(curr_rest_lens, next_rest_lens)]

                for j in range(len(change_rates)):
                    rate = change_rates[j]
                    if abs(rate) > 0.05:
                        ctrl = -rate / abs(rate)
                        data[i]['controls'][j] = ctrl
                    else:
                        data[i]['controls'][j] = 0

        return data_jsons

    def build_batches(self, all_data):
        all_end_pts, all_rest_lengths = [], []
        all_gt_cable_dls, all_ctrls = [], []

        for data in all_data:
            data_end_pts, data_rest_lens = [], []
            data_gt_cable_dls, data_ctrls = [], []
            for i in range(len(data) - 1):
                end_pts = torch.tensor(data[i]['end_pts'],
                                       dtype=DEFAULT_DTYPE
                                       ).reshape(1, -1)
                rest_lens = torch.tensor(data[i]['rest_lengths'],
                                         dtype=DEFAULT_DTYPE
                                         ).reshape(1, -1)
                next_rest_lens = torch.tensor(data[i + 1]['rest_lengths'],
                                              dtype=DEFAULT_DTYPE
                                              ).reshape(1, -1)
                cable_dls = next_rest_lens - rest_lens

                if i < self.num_hist - 1:
                    pad_ctrls = torch.zeros((self.num_hist - 1 - i, 6), dtype=DEFAULT_DTYPE)
                    ctrls = torch.tensor([e['controls'] for e in data[:i + 1]],
                                         dtype=DEFAULT_DTYPE)
                    controls = torch.vstack([pad_ctrls, ctrls]).reshape(1, -1)
                else:
                    controls = torch.tensor([e['controls'] for e in data[i - self.num_hist + 1:i + 1]],
                                            dtype=DEFAULT_DTYPE).reshape(1, -1)

                data_end_pts.append(end_pts)
                data_rest_lens.append(rest_lens)
                data_gt_cable_dls.append(cable_dls)
                data_ctrls.append(controls)

            all_end_pts.append(torch.vstack(data_end_pts))
            all_rest_lengths.append(torch.vstack(data_rest_lens))
            all_gt_cable_dls.append(torch.vstack(data_gt_cable_dls))
            all_ctrls.append(torch.vstack(data_ctrls))

        data_dict = {
            'end_pts': all_end_pts,
            'rest_lengths': all_rest_lengths,
            'gt_cable_dls': all_gt_cable_dls,
            'ctrls': all_ctrls,
        }

        all_data_in = torch.vstack([
            torch.hstack([e, r, c])
            for e, r, c in zip(all_end_pts, all_rest_lengths, all_ctrls)
        ])
        all_data_out = torch.vstack(all_gt_cable_dls)

        idxs = list(range(all_data_out.shape[0]))
        random.shuffle(idxs)

        all_data_in = all_data_in[idxs]
        all_data_out = all_data_out[idxs]

        i = 0
        batches = []
        while i < all_data_out.shape[0]:
            end = min(i + self.batch_size, all_data_out.shape[0])
            batches.append((all_data_in[i:end], all_data_out[i:end]))
            i = end

        return batches, data_dict

    def rollout(self, data_dict):
        self.motor_model.eval()
        self.motor_model.to('cpu')

        with torch.no_grad():
            all_rest_lens = []
            total_error = 0.0
            for i in range(len(data_dict['end_pts'])):
                end_pts = data_dict['end_pts'][i]
                gt_rest_lens = data_dict['rest_lengths'][i]
                controls = data_dict['ctrls'][i]

                curr_rest_lens = gt_rest_lens[:1].clone()
                pred_rest_lens = []
                for j in range(end_pts.shape[0] - 1):
                    in_data = torch.hstack([end_pts[j: j + 1], curr_rest_lens, controls[j: j + 1]])
                    pred_dl = self.motor_model(in_data)

                    curr_rest_lens = curr_rest_lens + pred_dl
                    pred_rest_lens.append(curr_rest_lens.clone())

                pred_rest_lens = torch.vstack(pred_rest_lens)
                all_rest_lens.append(pred_rest_lens)

                error = ((gt_rest_lens[1:] - pred_rest_lens) ** 2).mean()
                total_error += error.detach().item()
                print(error)

                times = [0.01 * (j + 1) for j in range(len(pred_rest_lens))]
                # times = np.arange(0.01, (len(pred_rest_lens) + 1) * 0.01, 0.01)

                plt.figure()
                figs, axes = plt.subplots(nrows=2, ncols=3)
                for k in range(pred_rest_lens.shape[1]):
                    k0 = int(k / 3)
                    k1 = k - k0 * 3
                    axes[k0, k1].plot(times, pred_rest_lens[:, k], label='pred')
                    axes[k0, k1].plot(times, gt_rest_lens[1:, k], label='gt')

                plt.legend()
                plt.savefig(self.output_dir / f'{data_dict["names"][i]}_rollout.png')
                plt.close()

            total_error /= len(data_dict['end_pts'])

        self.motor_model.train()
        self.motor_model.to(self.device)

        return total_error

    def run_one_epoch(self, batches, grad_required=True):
        if grad_required:
            self.motor_model.train()
        else:
            self.motor_model.eval()

        total_loss = 0.0
        num_data = 0
        for batch in batches:
            batch = [b.to(self.device) for b in batch]
            batch_x, batch_y = batch

            pred_cable_dls = self.motor_model(batch_x)
            loss = self.loss_fn(pred_cable_dls, batch_y)

            if grad_required:
                loss.backward()
                self.optimizer.step()
                self.optimizer.zero_grad()

            num_data += batch_x.shape[0]
            total_loss += loss.detach().item() * batch_x.shape[0]

        total_loss /= num_data

        return total_loss

    def run(self):
        for n in range(self.train_cfg['num_epochs']):
            train_loss = self.run_one_epoch(self.train_batches, grad_required=True)
            val_loss = self.run_one_epoch(self.val_batches, grad_required=False)

            print(f'Epoch: {n + 1}, Train loss: {train_loss}, Val loss: {val_loss}')

            if (n + 1) % self.eval_step == 0:
                if val_loss < self.best_val_loss:
                    self.best_val_loss = val_loss
                    torch.save(self.motor_model.state_dict(), self.output_dir / f'best_loss_model.pt')

                train_rollout_loss = self.rollout(self.train_data_dict)
                val_rollout_loss = self.rollout(self.val_data_dict)
                print(f'Epoch: {n + 1}, Train Rollout: {train_rollout_loss}, Val Rollout: {val_rollout_loss}')

                if val_rollout_loss < self.best_rollout_loss:
                    self.best_rollout_loss = val_rollout_loss
                    torch.save(self.motor_model.state_dict(), self.output_dir / f'best_rollout_model.pt')
