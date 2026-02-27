import torch

from gnn_simulator.nn_training.tensegrity_gnn_training_engine import TensegrityGNNTrainingEngine
from gnn_simulator.simulators.tensegrity_gnn_simulator import TensegrityHybridTwistGNNSimulator
from gnn_simulator.simulators import TensegrityRobotSimulator
from gnn_simulator.utilities import misc_utils
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE
from gnn_simulator.utilities.tensor_utils import zeros, safe_norm


class TensegrityTwistGNNTrainingEngine(TensegrityGNNTrainingEngine):

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

    def compute_gt_twist_dv(self, graph, ang_vel):
        body_mask = graph.body_mask.flatten()
        robot = self.simulator.robot

        num_nodes = robot.num_nodes_per_rod
        pos = graph.pos[body_mask]

        idx_0 = list(robot.rods.values())[0].sphere_idx0
        idx_1 = list(robot.rods.values())[0].sphere_idx1

        sphere1, sphere2 = pos[idx_0::num_nodes], pos[idx_1::num_nodes]
        gt_prin = (sphere2 - sphere1).unsqueeze(-1)
        gt_prin = gt_prin / gt_prin.norm(dim=1, keepdim=True)

        gt_twist_vel = torch.linalg.vecdot(gt_prin, ang_vel, dim=1)
        gt_twist_vel = (gt_twist_vel
                        .repeat(1, pos.shape[0] // gt_twist_vel.shape[0])
                        .reshape(-1, 1))
        twist_gt_dv = gt_twist_vel - graph.node_twist_vel[body_mask]
        return twist_gt_dv

    def compute_node_loss(self, graphs, ys, dt):
        norm_pred_dvs, norm_gt_dvs = [], []
        body_mask = graphs[0].body_mask.flatten()

        for i in range(len(graphs)):
            graph = graphs[i]
            norm_pred_dvs.append(graph.decode_output[body_mask])

            y = ys[:, :, i].reshape(-1, 13, 1)
            next_pose = y[:, :7]

            gt_nodes_pose = self.simulator.data_processor.pose2node(next_pose)
            gt_nodes_pos = gt_nodes_pose[:, :3]
            node_gt_dv = ((gt_nodes_pos - graph.pos[body_mask]) / dt
                          - graph.vel[body_mask, :3])

            twist_gt_dv = self.compute_gt_twist_dv(graph, y[:, 10:13])

            gt_dv = torch.hstack([node_gt_dv, twist_gt_dv]) - graph.pf_dv[body_mask]
            norm_gt_dv = self.simulator.data_processor.normalizers['dv'](gt_dv)
            norm_gt_dvs.append(norm_gt_dv)

        norm_pred_dvs = torch.stack(norm_pred_dvs, dim=2)
        norm_gt_dvs = torch.stack(norm_gt_dvs, dim=2)

        loss = self.loss_fn(norm_pred_dvs, norm_gt_dvs)
        pos_loss = self.loss_fn(graphs[-1].p_pos[body_mask], gt_nodes_pos).detach().item()
        pos_loss2 = self.loss_fn(graphs[-1].pos[body_mask], gt_nodes_pos).detach().item()

        twist_dv_loss = self.loss_fn(gt_dv[:, 3:],
                                     graphs[-1].p_dv[body_mask, 3:]
                                     ).detach().item()

        return loss, pos_loss, twist_dv_loss

    def get_dummy_simulator(self):
        sim = TensegrityRobotSimulator(self.sim_config['tensegrity_cfg'],
                                       self.sim_config['gravity'],
                                       self.sim_config['contact_params'])
        return sim

    def compute_quat_from_sites(self, rod_sites_dict, rod_end_pts, ref_sites_dict=None):
        if ref_sites_dict is None:
            ref_sites_dict = self._get_ref_sites()

        inv_quat_fn = lambda q, v: torch_quaternion.rotate_vec_quat(
            torch_quaternion.inverse_unit_quat(q), v)

        z = zeros((1, 3, 1), dtype=self.dtype, device=self.device)
        z[:, 2] = 1.0

        quats = []
        for k in rod_sites_dict.keys():
            r_sites = ref_sites_dict[k].keys()

            new_sites = torch.concat([rod_sites_dict[k][kk] for kk in r_sites], dim=2)
            ref_sites = torch.concat([ref_sites_dict[k][kk] for kk in r_sites], dim=2).repeat(
                new_sites.shape[0], 1, 1)
            end_pts = rod_end_pts[k]

            pos_new = (end_pts[1] + end_pts[0]) / 2
            prin_new = safe_norm(end_pts[1] - end_pts[0])
            quat_new = torch_quaternion.compute_quat_btwn_z_and_vec(end_pts[1] - end_pts[0])

            ref_new_sites = inv_quat_fn(quat_new, new_sites - pos_new)

            ref_sites1_norm = ref_sites[:, :2].norm(dim=1, keepdim=True)
            ref_sites2_norm = ref_new_sites[:, :2].norm(dim=1, keepdim=True)
            angles = torch.linalg.vecdot(ref_sites[:, :2], ref_new_sites[:, :2], dim=1).unsqueeze(1)
            angles = angles / ref_sites1_norm / ref_sites2_norm

            lim = 0.9999999 if DEFAULT_DTYPE == torch.float32 else 1.
            angles = torch.arccos(torch.clamp(angles, -lim, lim))
            avg_angle = angles.mean(dim=2, keepdim=True)

            cross_prods = (ref_sites[:, 0:1] * ref_new_sites[:, 1:2]
                           - ref_sites[:, 1:2] * ref_new_sites[:, 0:1])
            rot_dir = torch.sign(cross_prods.mean(dim=2, keepdim=True))

            avg_angle = rot_dir * avg_angle / 2.
            q = torch.hstack([torch.cos(avg_angle), z * torch.sin(avg_angle)])
            quat = torch_quaternion.quat_prod(quat_new, q)
            quats.append(quat)

        quats = torch.hstack(quats)

        return quats

    def _get_ref_sites(self):
        rods = self.simulator.robot.rods.values()
        sys_top_sites = self.simulator.robot.system_topology.sites_dict

        ref_sites = {
            rod.name: {
                s: torch_quaternion.rotate_vec_quat(
                    torch_quaternion.inverse_unit_quat(rod.quat),
                    sys_top_sites[s] - rod.pos
                ) for s in rod.sites.keys() if len(s) > 3
            } for rod in rods
        }

        return ref_sites

    def data_to_pos_quat_ctrls(self, data_jsons, gt_end_pts, extra_state_jsons):
        data_pos, data_quats, data_controls = [], [], []
        num_rods = len(self.simulator.robot.rods)

        ref_sites = self._get_ref_sites()
        for data_json, end_pts in zip(data_jsons, gt_end_pts):
            sites = [
                {k: torch.tensor(v, dtype=self.dtype).reshape(1, 3, 1)
                 for k, v in d['sites'].items()}
                for d in data_json
            ]

            sites = {k: torch.vstack([s[k] for s in sites])
                     for k in sites[0].keys()}
            sites = {k: {kk: sites[kk] for kk in v.keys()}
                     for k, v in ref_sites.items()}

            end_pts = torch.vstack([torch.hstack(e) for e in end_pts]).reshape(-1, 6, 1)
            pos = ((end_pts[:, :3] + end_pts[:, 3:]) / 2.).reshape(-1, 3 * num_rods, 1)
            pos = [[pos[i: i + 1, 3 * j: 3 * (j + 1)] for j in range(num_rods)]
                   for i in range(pos.shape[0])]

            end_pts = {k: [end_pts[i::num_rods, :3], end_pts[i::num_rods, 3:]]
                       for i, k in enumerate(ref_sites.keys())}
            quats = self.compute_quat_from_sites(sites, end_pts, ref_sites)
            quats = [[quats[i: i + 1, 4 * j: 4 * (j + 1)] for j in range(num_rods)]
                     for i in range(quats.shape[0])]

            data_pos.append(pos)
            data_quats.append(quats)

        if extra_state_jsons is not None:
            times = [[d['time'] - data_json[0]['time']
                      for d in data_json]
                     for data_json in data_jsons]
            data_controls = self.load_controls(extra_state_jsons, times)

        return data_pos, data_quats, data_controls

    def rotate_data_aug(self, batch_x, batch_y):
        n = len(self.simulator.robot.rods)

        angle = 2 * torch.pi * (torch.rand((batch_x.shape[0], 1, 1), device=batch_x.device) - 0.5)
        w = torch.cos(angle / 2)
        xyz = torch.tensor([0, 0, 1], dtype=self.dtype, device=batch_x.device).reshape(1, 3, 1)
        xyz = (xyz.repeat(batch_x.shape[0], 1, 1) * torch.sin(angle / 2))
        q = torch.hstack([w, xyz]).repeat(1, n, 1).reshape(-1, 4, 1)

        batch_x_ = batch_x.reshape(-1, 13, 1)
        pos = torch_quaternion.rotate_vec_quat(q, batch_x_[:, :3])
        quat = torch_quaternion.quat_prod(q, batch_x_[:, 3:7])
        linvel = torch_quaternion.rotate_vec_quat(q, batch_x_[:, 7:10])
        angvel = torch_quaternion.rotate_vec_quat(q, batch_x_[:, 10:])

        batch_x_rots = torch.hstack([pos, quat, linvel, angvel]).reshape(batch_x.shape)

        batch_y_rots = []
        for i in range(batch_y.shape[2]):
            batch_y_ = batch_y[:, :, i: i + 1].reshape(-1, 13, 1)
            pos = torch_quaternion.rotate_vec_quat(q, batch_y_[:, :3])
            quat = torch_quaternion.quat_prod(q, batch_y_[:, 3:7])
            linvel = torch_quaternion.rotate_vec_quat(q, batch_y_[:, 7:10])
            angvel = torch_quaternion.rotate_vec_quat(q, batch_y_[:, 10:])

            batch_y_rot = torch.hstack([pos, quat, linvel, angvel]).reshape(batch_x.shape)
            batch_y_rots.append(batch_y_rot)

        batch_y_rots = torch.concat(batch_y_rots, dim=2)

        return batch_x_rots, batch_y_rots

    def build_batch_dict(self, **kwargs):
        batch_dict = {}
        states = kwargs['states']
        times = kwargs['times']

        for i in range(len(states)):
            controls = torch.vstack(kwargs['controls'][i])
            ctrls_hist = kwargs['controls'][i][:-self.num_steps_fwd]
            act_lengths = kwargs['act_lengths'][i][:-self.num_steps_fwd]
            motor_omegas = kwargs['motor_omegas'][i][:-self.num_steps_fwd]
            x = states[i][:-self.num_steps_fwd]

            x = [x[0].clone() for _ in range(self.num_hist - 1)] + x
            act_lengths = [act_lengths[0].clone() for _ in range(self.num_hist - 1)] + act_lengths
            motor_omegas = [motor_omegas[0].clone() for _ in range(self.num_hist - 1)] + motor_omegas
            ctrls_hist = [zeros(ctrls_hist[0].shape, ref_tensor=ctrls_hist[0])
                          for _ in range(self.num_hist - 1)] + ctrls_hist

            batch_x = [torch.concat(x[j - self.num_hist + 1: j + 1], 2)
                       for j in range(self.num_hist - 1, len(x))]
            batch_act_lens = [torch.concat(act_lengths[j - self.num_hist + 1: j + 1], 2)
                              for j in range(self.num_hist - 1, len(act_lengths))]
            batch_motor_speeds = [torch.concat(motor_omegas[j - self.num_hist + 1: j + 1], 2)
                                  for j in range(self.num_hist - 1, len(motor_omegas))]
            batch_ctrls_hist = [torch.concat(ctrls_hist[j - self.num_hist + 1: j + 1], 2)
                                for j in range(self.num_hist - 1, len(ctrls_hist))]

            batch_y, batch_ctrls = [], []
            gt_states = torch.vstack(states[i])
            for j in range(1, self.num_steps_fwd + 1):
                end = -(self.num_steps_fwd - j) \
                    if j < self.num_steps_fwd else gt_states.shape[0]
                batch_y.append(gt_states[j:end])
                batch_ctrls.append(controls[j - 1: -(self.num_steps_fwd - j + 1)])

            batch_y = torch.concat(batch_y, dim=2)
            batch_ctrls = torch.concat([torch.vstack(batch_ctrls_hist)] + batch_ctrls[1:], dim=2)

            batch_y = [batch_y[j: j + 1] for j in range(batch_y.shape[0])]
            batch_ctrls = [batch_ctrls[j: j + 1] for j in range(batch_ctrls.shape[0])]

            batch_dt = [times[i][j + self.num_steps_fwd] - times[i][j]
                        for j in range(len(times[i]) - self.num_steps_fwd)]

            batch_elms = zip(batch_x, batch_y,
                             batch_dt, batch_ctrls,
                             batch_act_lens, batch_motor_speeds)
            for x, y, dt, ctrl, act_len, motor_omega in batch_elms:
                delta_t = misc_utils.compute_num_steps(dt, self.dt) * self.dt

                batch = batch_dict.get(delta_t, [[] for _ in range(7)])
                batch[0].append(x)
                batch[1].append(y)
                batch[2].append(ctrl)
                batch[3].append(torch.tensor([[[dt]]], dtype=self.dtype))
                batch[4].append(act_len)
                batch[5].append(motor_omega)
                batch[6].append(torch.tensor([[[delta_t]]], dtype=self.dtype))
                batch_dict[delta_t] = batch

        return batch_dict
