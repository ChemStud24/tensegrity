import random
from pathlib import Path
from typing import Tuple
import gc

import torch
import tqdm

from gnn_simulator.nn_training.tensegrity_gnn_training_engine import TensegrityGNNTrainingEngine
from gnn_simulator.simulators.tensegrity_gnn_simulator import TensegrityHybridTwistGNNSimulator
from gnn_simulator.simulators.tensegrity_physics_simulator import TensegrityRobotSimulator
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.tensor_utils import zeros


class TrajTensegrityGNNTrainingEngine(TensegrityGNNTrainingEngine):

    def init_data(self, data_paths):
        random.shuffle(data_paths)

        with torch.no_grad():
            data_dict = {}
            data_dict['names'] = [p.split("/")[-2] for p in data_paths]

            data_jsons, target_gaits, _ = (
                self.load_json_files(data_paths))
            data_dict.update({'data_jsons': data_jsons,
                              'target_gaits': target_gaits})

            data_dict['gt_end_pts'] = self._get_endpts(data_jsons)

            data_dict['times'] = [[d['time'] - data_json[0]['time'] for d in data_json]
                                  for data_json in data_jsons]

            data_dict['states'], data_dict['controls'] = (
                self.data_json_to_states(data_jsons,
                                         data_dict['gt_end_pts'],
                                         data_dict['times'],
                                         None)
            )
            data_dict['kf_end_pts'] = self._get_kf_endpts(
                data_dict['gt_end_pts'], data_dict['target_gaits'])

            data_dict['start_states'] = self.get_start_states(data_jsons)

            return data_dict, []

    def get_ins_vels(self, data_jsons):
        n = len(self.simulator.robot.rods)
        data_vels = [[torch.zeros((1, 6 * n, 1), dtype=self.dtype)]
                     for _ in range(len(data_jsons))]

        return data_vels

    def _get_kf_endpts(self, end_pts, target_gaits):
        return [
            torch.vstack(
                [torch.hstack(e[v['idx']])
                 for v in t[1:]]
                + [torch.hstack(e[-1])]
            ) for e, t in zip(end_pts, target_gaits)
        ]

    def get_dummy_simulator(self):
        sim = TensegrityRobotSimulator(self.sim_config['tensegrity_cfg'],
                                       self.sim_config['gravity'],
                                       self.sim_config['contact_params'])
        return sim

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu")
            sim.reset_actuation()
            sim.cpu()
            print("Loaded simulator")
        else:
            sim = TensegrityHybridTwistGNNSimulator(**self.sim_config)

        sim.data_processor.training = True
        return sim

    def get_pid_params(self, name):
        min_length = 70
        range_ = 100
        tol = 0.1

        if "cw" in name:
            min_length = 100
            range_ = 100
            tol = 0.1
        elif "rolling" in name:
            min_length = 100
            range_ = 100
            tol = 0.1

        return {'min_length': min_length, 'RANGE': range_, 'tol': tol}

    def log_status(self, losses: Tuple, epoch_num: int, name='') -> None:
        """
        Method to print training status to console

        :param losses: Train loss, Val loss, Val rollout KF loss
        :param epoch_num: Current epoch
        """
        losses = [f'{l:.4}' if isinstance(l, float) else [f'{ll:.4}' for ll in l] for l in losses]

        loss_file = Path(self.output_dir, "loss.txt")
        loss_msg = (f'Epoch {epoch_num}, '
                    f'Trajectory {name}, '
                    f'KF Node Losses: {losses}')

        try:
            with loss_file.open('a') as fp:
                fp.write(loss_msg + "\n")
        except:
            with loss_file.open('w') as fp:
                fp.write(loss_msg + "\n")

        print(loss_msg)

    def compute_init_loss(self):
        with torch.no_grad():
            if not self.config['load_sim']:
                self.simulator.data_processor.start_normalizers()

            self.eval()

            train_avg_loss = self.run_all_trajs(self.train_data_dict,
                                                0,
                                                False,
                                                False)
            self.log_status(tuple([train_avg_loss]), 0, "Average Train")
            self.simulator.data_processor.stop_normalizers()

            val_avg_loss = self.run_all_trajs(self.val_data_dict,
                                              0,
                                              False,
                                              False)
            self.log_status(tuple([val_avg_loss]), 0, "Average Val")

            self.train()

        return train_avg_loss, val_avg_loss

    def rotate_data_aug(self, batch_x, kf_gt_end_pts):
        n = len(self.simulator.robot.rods)

        angle = 2 * torch.pi * (torch.rand((batch_x.shape[0], 1, 1), device=batch_x.device) - 0.5)
        xyz = torch.tensor([0, 0, 1],
                           dtype=self.dtype,
                           device=batch_x.device
                           ).reshape(1, 3, 1)
        q = torch.hstack([torch.cos(angle / 2), xyz * torch.sin(angle / 2)])
        q = q.repeat(n, 1, 1)

        batch_x_ = batch_x.reshape(-1, 13, 1)
        pos = torch_quaternion.rotate_vec_quat(q, batch_x_[:, :3])
        quat = torch_quaternion.quat_prod(q, batch_x_[:, 3:7])
        linvel = torch_quaternion.rotate_vec_quat(q, batch_x_[:, 7:10])
        angvel = torch_quaternion.rotate_vec_quat(q, batch_x_[:, 10:])
        rot_batch_x = torch.hstack([pos, quat, linvel, angvel]).reshape(batch_x.shape)

        rot_kf_gt_end_pts = []
        for gt_end_pts in kf_gt_end_pts:
            gt_end_pts_ = gt_end_pts.reshape(-1, 6, 1)
            rot_endpt_0 = torch_quaternion.rotate_vec_quat(q, gt_end_pts_[:, :3])
            rot_endpt_1 = torch_quaternion.rotate_vec_quat(q, gt_end_pts_[:, 3:])
            rot_gt_end_pts = torch.hstack([
                rot_endpt_0, rot_endpt_1
            ]).reshape(gt_end_pts.shape)

            rot_kf_gt_end_pts.append(rot_gt_end_pts)

        return rot_batch_x, rot_kf_gt_end_pts

    def run_all_trajs(self, data_dict, epoch_num, grad_required=True, rot_aug=True):
        avg_loss = 0
        for i, name in enumerate(data_dict['names'][:2]):
            start_state = data_dict['start_states'][i].clone()
            kf_gt_end_pts = data_dict['kf_end_pts'][i]

            if rot_aug:
                start_state, kf_gt_end_pts = self.rotate_data_aug(start_state, kf_gt_end_pts)

            states, pred_key_endpts, kf_graphs = self.run_traj(
                start_state,
                data_dict['target_gaits'][i][:18],
                name,
                data_dict['times'][i]
            )

            loss = self.compute_node_loss(kf_graphs,
                                          kf_gt_end_pts[:18],
                                          self.dt)

            if grad_required:
                loss.backward()
                self.optimizer.step()
            self.optimizer.zero_grad()

            loss = loss.detach().item()
            self.log_status(tuple([loss]), epoch_num, name)
            avg_loss += loss

            del states, pred_key_endpts, kf_graphs
            gc.collect()

        avg_loss /= len(data_dict['names'])



        return avg_loss

    def run_traj(self, start_state, target_gaits, name, times):
        """

        """
        # self.simulator.reset_actuation()
        self.simulator.robot.detach_state()
        sites_dict = self.simulator.robot.system_topology.sites_dict
        self.simulator.robot.system_topology.sites_dict = \
            {k: v.detach() for k, v in sites_dict.items()}
        for c in self.simulator.robot.actuated_cables.values():
            c.actuation_length = 0.0
        pid_params = self.get_pid_params(name)

        curr_state = start_state.clone().detach()
        all_states, pred_key_endpts, kf_graphs = [], [], []
        all_states.append({"time": 0.0, "state": curr_state.detach().flatten().numpy().tolist()})
        global_steps = 0
        for i, target_gait in enumerate(tqdm.tqdm(target_gaits)):
            start = target_gait['idx']
            end = target_gaits[i + 1]['idx'] if i < len(target_gaits) - 1 else -1
            dtime = times[end] - times[start]
            # num_steps = misc_utils.compute_num_steps(dtime * 3.1, self.dt)

            target_gait_dict = {f"cable_{j}": g
                                for j, g in enumerate(target_gait['target_gait'])}
            states, graphs = self.simulator.run_with_target_gait(
                curr_state,
                self.dt,
                target_gait_dict,
                pid_params,
                # num_steps
            )
            curr_state = states[-1]
            global_steps += len(states)

            kf_graphs.append(graphs[-1].clone())
            # pred_key_endpts.append(self._batch_compute_end_pts(states[-1]))

            all_states.extend([
                {"time": 0.0,
                 "pos": s.reshape(-1, 13)[:, :7].flatten().detach().numpy().tolist()}
            for s in states])

        # pred_key_endpts = torch.vstack(pred_key_endpts)

        # with Path(self.output_dir, f"{name}_output_states.json").open("w") as fp:
        #     json.dump(all_states, fp)

        return all_states, pred_key_endpts, kf_graphs

    def compute_node_loss(self, kf_graphs, gt_kf_end_pts, dt):
        pred_kf_node_pos = torch.stack([g.p_pos for g in kf_graphs], dim=2)

        gt_kf_node_pos = []
        for gt_end_pts in gt_kf_end_pts:
            end_pts = gt_end_pts.reshape(-1, 6, 1)
            gt_pos, gt_quat = self.endpts2pos(end_pts[:, :3], end_pts[:, 3:])
            gt_nodes_pos = self.simulator.data_processor.pose2node(
                torch.hstack([gt_pos, gt_quat]),
                augment_grnd=True
            )

            gt_kf_node_pos.append(gt_nodes_pos[:, :3])

        gt_kf_node_pos = torch.stack(gt_kf_node_pos, dim=2)
        loss = self.loss_fn(pred_kf_node_pos, gt_kf_node_pos)
        return loss

    def get_start_states(self, data_jsons, max_steps=400, threshold=1e-2):
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

    def _shift_robot(self, curr_state, shift_state):
        n = len(self.simulator.robot.rods)

        curr_state_ = curr_state.reshape(-1, n, 13).transpose(1, 2)
        shift_state_ = shift_state.reshape(-1, n, 13).transpose(1, 2)

        curr_com = curr_state_[:, :3].mean(dim=2, keepdim=True)
        curr_end_pts = self._batch_compute_end_pts(curr_state).reshape(-1, n, 6).transpose(1, 2)
        curr_mid_pts = curr_end_pts.mean(dim=2, keepdim=True)
        curr_prin = curr_mid_pts[:, 3:] - curr_mid_pts[:, :3]
        curr_prin /= curr_prin.norm(dim=1, keepdim=True)

        shift_com = shift_state_[:, :3].mean(dim=2, keepdim=True)
        shift_end_pts = self._batch_compute_end_pts(shift_state).reshape(-1, n, 6).transpose(1, 2)
        shift_mid_pts = shift_end_pts.mean(dim=2, keepdim=True)
        shift_prin = shift_mid_pts[:, 3:] - shift_mid_pts[:, :3]
        shift_prin /= shift_prin.norm(dim=1, keepdim=True)

        angle = torch.linalg.vecdot(shift_prin[:, :2], curr_prin[:, :2], dim=1).unsqueeze(1)
        angle = torch.acos(torch.clamp(angle, -1, 1)) / 2.

        cross_prods = torch.sign(curr_prin[:, 0:1] * shift_prin[:, 1:2]
                                 - curr_prin[:, 1:2] * shift_prin[:, 0:1])
        rot = torch.tensor([0, 0, 1], dtype=self.dtype).reshape(1, 3, 1)
        q = torch.hstack([torch.cos(angle), cross_prods * rot * torch.sin(angle)])
        q = q.repeat(1, 1, n)

        pos = torch_quaternion.rotate_vec_quat(q, curr_state_[:, :3] - curr_com)
        pos[:, :2] += shift_com[:, :2]
        pos[:, 2] += curr_com[:, 2]
        quat = torch_quaternion.quat_prod(q, curr_state_[:, 3:7])
        vels = torch.zeros_like(curr_end_pts)

        next_state = (torch.hstack([pos, quat, vels])
                      .transpose(1, 2)
                      .reshape(-1, 1, 13 * n)
                      .transpose(1, 2))

        return next_state

    def run(self, num_epochs: int):
        # with torch.no_grad():
        #     train_avg_loss, val_avg_loss = self.compute_init_loss()
        #     self.best_val_loss = val_avg_loss

        for i in range(num_epochs):
            self.train()
            train_loss = self.run_all_trajs(self.train_data_dict,
                                            i + 1,
                                            True,
                                            True)
            self.log_status(tuple([train_loss]), i + 1, "Average Train")

            with torch.no_grad():
                self.eval()
                val_loss = self.run_all_trajs(self.val_data_dict,
                                              i + 1,
                                              False,
                                              False)
                self.log_status(tuple([val_loss]), i + 1, "Average Val")

                if val_loss < self.best_val_loss:
                    self.best_val_loss = val_loss
                    torch.save(
                        self.simulator,
                        Path(self.output_dir, "best_loss_model.pt")
                    )
