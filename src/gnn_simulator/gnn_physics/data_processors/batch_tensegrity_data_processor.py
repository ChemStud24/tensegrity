from collections import OrderedDict
from typing import Dict, List, Tuple

import torch
from torch_geometric.data import Data as GraphData

from gnn_simulator.gnn_physics.data_processors.abstract_tensegrity_data_processor import BaseRobotGraphDataProcessor
from gnn_simulator.gnn_physics.normalizer import AccumulatedNormalizer, DummyNormalizer
from gnn_simulator.robots.tensegrity import TensegrityRobotGNN
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.tensor_utils import zeros, safe_norm


class BatchTensegrityDataProcessor(BaseRobotGraphDataProcessor):

    robot: TensegrityRobotGNN

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5):
        """

        @param tensegrity: robot object
        @param con_edge_threshold: threshold to attach edge between ground and endcap node
        @param num_steps_ahead: how many steps training traj will be
        @param num_hist: how many steps behind to attach to features
        @param dt: timestep size
        @param max_dist_to_grnd: clip value for dist to ground feature
        """
        self.MAX_DIST_TO_GRND = max_dist_to_grnd
        self.CONTACT_EDGE_THRESHOLD = con_edge_threshold

        node_feat_dict = OrderedDict({
            'node_vel': 3,
            'node_inv_mass': 1,
            'node_inv_inertia': 3,
            'node_dist_to_ground': 1,
            'node_body_verts': 3,
            # 'node_dist_to_first_node': 3,
            # 'node_dist_to_first_node_norm': 1,
            # 'node_dir_from_com': 3,
            # 'node_dist_from_com_norm': 1,
            # 'node_prin_axis': 3
        })

        # if num_hist > 1:
        #     node_feat_dict['node_prev_vels'] = 3 * (num_hist - 1)

        body_edge_feat_dict = OrderedDict({
            'body_dist': 3,
            'body_dist_norm': 1,
            'body_rest_dist': 3,
            'body_rest_dist_norm': 1,
            # 'body_edge_type': 1
        })

        cable_edge_feat_dict = OrderedDict({
            'cable_dist': 3,
            'cable_dist_norm': 1,
            'cable_dir': 3,
            'cable_rel_vel_norm': 1,
            # 'cable_dl': 1,
            'cable_rest_length': 1,
            'cable_stiffness': 1,
            'cable_damping': 1,
            'cable_stiffness_force_mag': 1,
            'cable_damping_force_mag': 1,
            # 'cable_type': 1
        })

        contact_edge_feat_dict = OrderedDict({
            'contact_dist': 3,
            'contact_normal': 3,
            'contact_tangent': 3,
            'contact_rel_vel_normal': 1,
            'contact_rel_vel_tangent': 1,
            # 'contact_type': 1
        })

        node_dict = {'node': node_feat_dict}
        edge_dict = {
            'body': body_edge_feat_dict,
            'cable': cable_edge_feat_dict,
            'contact': contact_edge_feat_dict
        }

        super().__init__(tensegrity,
                         node_dict,
                         edge_dict,
                         dt)

        # Add normalizer specifically for model output: dv
        self.normalizers['dv'] = AccumulatedNormalizer(
            (1, 3),
            name='dv',
            dtype=self.dtype
        )

    def _get_body_verts(self, batch_size: int) -> torch.Tensor:
        """
        Method to get node positions in body frame from robot
        @param batch_size: size of batch
        @return: torch tensor of body verts repeated batch_size times
        """
        body_verts = torch.vstack([
            bv.to(self.device)
            for bv in self.robot.rod_body_verts
        ])
        body_verts = torch.vstack([
            body_verts,
            zeros(body_verts[0:1].shape, ref_tensor=body_verts)
        ]).repeat(batch_size, 1, 1).squeeze(-1)

        return body_verts

    def _body_edge_index(self) -> torch.Tensor:
        """
        Get
        @return:
        """
        senders = self.robot.template_idx[:1].to(self.device)
        receivers = self.robot.template_idx[1:].to(self.device)

        edge_index = torch.vstack([senders, receivers])

        return edge_index

    def node2com(self, node_pos, batch_size):
        num_nodes = self.robot.num_nodes_per_rod

        node_pos_no_grnd = node_pos.reshape(batch_size, -1)[:, :-3].reshape(-1, 3)
        com_pos = node_pos_no_grnd.reshape(-1, num_nodes, 3).mean(dim=1)

        return com_pos

    def node2pose(self,
                  node_pos: torch.Tensor,
                  prev_node_pos: torch.Tensor,
                  num_nodes: int,
                  **kwargs):
        """
        Method to map node poses to SE(3) poses

        @param node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param prev_node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param num_nodes: num nodes per rod
        @return: torch tensor of SE(3) poses
        """
        curr_com_pos = node_pos.reshape(-1, num_nodes, 3).mean(dim=1)
        prev_com_pos = prev_node_pos.reshape(-1, num_nodes, 3).mean(dim=1)

        lin_vel = (curr_com_pos - prev_com_pos).unsqueeze(-1) / self.dt

        idx_0 = list(self.robot.rods.values())[0].sphere_idx0
        idx_1 = list(self.robot.rods.values())[0].sphere_idx1

        curr_sphere0 = node_pos[idx_0::num_nodes]
        curr_sphere1 = node_pos[idx_1::num_nodes]
        prev_sphere0 = prev_node_pos[idx_0::num_nodes]
        prev_sphere1 = prev_node_pos[idx_1::num_nodes]

        curr_prin = safe_norm(curr_sphere1 - curr_sphere0).unsqueeze(-1)
        prev_prin = safe_norm(prev_sphere1 - prev_sphere0).unsqueeze(-1)

        ang_vel = torch_quaternion.compute_ang_vel_vecs(prev_prin, curr_prin, self.dt)
        quat = torch_quaternion.compute_quat_btwn_z_and_vec(curr_prin)

        n_rods = len(self.robot.rods)
        state = torch.hstack([curr_com_pos.unsqueeze(-1), quat, lin_vel, ang_vel])
        state = state.reshape(-1, state.shape[1] * n_rods, 1)

        return state

    def _compute_node_feats(self,
                            node_pos: torch.Tensor,
                            prev_node_pos: torch.Tensor,
                            batch_size: int,
                            **kwargs
                            ) -> Dict[str, torch.Tensor]:
        """
        Method to compute all node feats based on curr and prev node poses

        @param node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param prev_node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param batch_size: size of batch
        @return: Dictionary of feat tensors
        """
        num_nodes = self.robot.num_nodes_per_rod
        # com_pos = self.node2com(node_pos, batch_size).repeat(1, num_nodes)
        com_pos = self.robot.pos.reshape(-1, 3).repeat(1, num_nodes)
        com_pos = torch.hstack([
            com_pos.reshape(batch_size, -1),
            zeros((batch_size, 3), ref_tensor=node_pos)
        ]).reshape(-1, 3)
        dist_from_com = node_pos - com_pos
        dist_from_com_norm = dist_from_com.norm(dim=1, keepdim=True)
        dir_from_com = safe_norm(dist_from_com)

        node_vels = (node_pos - prev_node_pos) / self.dt

        inv_mass = torch.vstack([self.robot.inv_mass.to(node_pos.device),
                                 zeros((1, 1), ref_tensor=node_pos)])
        inv_inertia = torch.vstack([self.robot.inv_inertia.to(node_pos.device),
                                    zeros((1, 3), ref_tensor=node_pos)])

        inv_mass = inv_mass.repeat(batch_size, 1)
        inv_inertia = inv_inertia.repeat(batch_size, 1)

        sphere_radius = list(self.robot.rods.values())[0].sphere_radius.squeeze(-1)
        dist_to_ground = node_pos[:, 2:3] - sphere_radius
        dist_to_ground = torch.clamp_max(dist_to_ground, self.MAX_DIST_TO_GRND)
        dist_to_ground[24::25] = 0.0
        body_verts = self._get_body_verts(batch_size)

        dist_first_node, dist_first_node_norm = (
            self._compute_shape_feats(batch_size, node_pos[:, :3])
        )
        prin = self._compute_prin_feat(node_pos)

        node_feats = {
            "node_pos": node_pos,
            "node_prev_pos": prev_node_pos,
            "node_vel": node_vels,
            "node_inv_mass": inv_mass,
            "node_inv_inertia": inv_inertia,
            "node_dir_from_com": dir_from_com,
            "node_dist_from_com_norm": dist_from_com_norm,
            "node_dist_to_ground": dist_to_ground,
            "node_body_verts": body_verts,
            'node_dist_to_first_node': dist_first_node,
            'node_dist_to_first_node_norm': dist_first_node_norm,
            'node_prin_axis': prin
        }

        return node_feats

    def _compute_prin_feat(self, node_pos):
        num_nodes, num_nodes_per_rod = self.robot.num_nodes, self.robot.num_nodes_per_rod

        sphere0_idx = list(self.robot.rods.values())[0].sphere_idx0 * 3
        sphere1_idx = list(self.robot.rods.values())[0].sphere_idx1 * 3
        node_pos_ = (node_pos
                     .reshape(-1, 3 * num_nodes + 3)[:, :-3]
                     .reshape(-1, 3 * num_nodes_per_rod))
        prin = node_pos_[:, sphere1_idx: sphere1_idx + 3] - node_pos_[:, sphere0_idx: sphere0_idx + 3]
        prin = prin / prin.norm(dim=1, keepdim=True)
        prin = prin.repeat(1, num_nodes_per_rod)
        ground_prin = zeros((node_pos.shape[0] // (num_nodes + 1), 3), ref_tensor=node_pos)
        ground_prin[:, 2] = 1.
        prin = torch.hstack([prin.reshape(-1, 3 * num_nodes), ground_prin]).reshape(-1, 3)

        return prin

    def _compute_shape_feats(self, batch_size, node_pos):
        num_nodes = node_pos.shape[0] // batch_size
        rods = list(self.robot.rods.values())

        first_node_idx = list(self.robot.rods.values())[0].sphere_idx0
        last_node_idx = list(self.robot.rods.values())[-1].sphere_idx1
        last_node_idx = last_node_idx + sum([r.body_verts.shape[0] for r in rods[:-1]])
        first_node = node_pos[first_node_idx::num_nodes].repeat(1, num_nodes - 1).reshape(-1, 3)
        last_node = node_pos[last_node_idx::num_nodes].repeat(1, num_nodes - 1).reshape(-1, 3)

        x_dir = torch.hstack([(last_node - first_node)[:, :2],
                              torch.zeros_like(last_node[:, :1])])
        x_dir = safe_norm(x_dir)
        z_dir = torch.tensor([[0, 0, 1]],
                             dtype=self.dtype,
                             device=self.device
                             ).repeat(x_dir.shape[0], 1)
        y_dir = torch.cross(z_dir, x_dir, dim=1)
        y_dir = safe_norm(y_dir)
        rot_mat = torch.stack([x_dir, y_dir, z_dir], dim=2)

        node_pos_no_grnd = node_pos.reshape(-1, 3 * num_nodes)[:, :-3].reshape(-1, 3)
        dist_first_node = (node_pos_no_grnd - first_node).unsqueeze(-1)
        dist_first_node = rot_mat.transpose(1, 2) @ dist_first_node
        dist_first_node = dist_first_node.reshape(-1, 3 * (num_nodes - 1))
        dist_first_node = torch.hstack([dist_first_node,
                                        torch.zeros_like(dist_first_node[:, :3])]
                                       ).reshape(-1, 3)
        dist_first_node_norm = dist_first_node.norm(dim=1, keepdim=True)

        return dist_first_node, dist_first_node_norm

    def _insert_grnd_node(self,
                          node_pos: torch.Tensor,
                          batch_size: int
                          ) -> torch.Tensor:
        """
        Method to insert grnd node pos into node pos tensor
        @param node_pos: node positions
        @param batch_size: size of batch
        @return: augmented node poses with ground nodes
        """
        aug_node_pos = node_pos.reshape(batch_size, -1)
        grnd_pos = zeros((batch_size, node_pos.shape[1]), ref_tensor=node_pos)
        aug_node_pos = (torch.hstack([aug_node_pos, grnd_pos])
                        .reshape(-1, node_pos.shape[1]))

        return aug_node_pos

    def _compute_body_mask(self, batch_size: int, num_robot_nodes: int) -> torch.Tensor:
        """
        Method to compute a mask indicating which nodes are body nodes (non-ground)

        @param batch_size: size of batch
        @param num_robot_nodes: number of nodes in the robot's body graph
        @return: mask tensor
        """
        body_mask = torch.full(
            (batch_size, num_robot_nodes + 1),
            True,
            device=self.device
        )
        body_mask[:, -1] = False
        body_mask = body_mask.reshape(-1, 1)
        return body_mask

    def _compute_node_type(self, batch_size):
        num_nodes_per_rod = self.robot.num_nodes_per_rod + 1
        num_rod = len(self.robot.rods)

        node_type = torch.tensor(
            [
                ([0] * (num_nodes_per_rod - 1)) * num_rod + [1]
            ], dtype=torch.int, device=self.device
        ).T.repeat(batch_size, 1)

        return node_type

    def build_graph(self,
                    node_pose: torch.Tensor,
                    prev_node_pose: torch.Tensor,
                    batch_size: int,
                    **kwargs) -> GraphData:
        """
        Method to build graph based on node poses

        @param node_pose: (batch_size * num nodes per graph, 3 * num_hist) at timestep t
        @param prev_node_pose: (batch_size * num nodes per graph, 3 * num_hist) at time t-1
        @param batch_size: size of current batch
        @return: constructed graph
        """

        # Compute node feats
        node_pos = self._insert_grnd_node(node_pose, batch_size)
        prev_node_pos = self._insert_grnd_node(prev_node_pose, batch_size)
        body_mask = self._compute_body_mask(batch_size, self.robot.num_nodes)
        node_type = self._compute_node_type(batch_size)

        node_feats = self._compute_node_feats(node_pos,
                                              prev_node_pos,
                                              batch_size)
        node_pos = node_feats['node_pos']

        # Compute edge indices and feats
        edge_idx_dict, edge_type = self._compute_edge_idxs()
        # edge_indices = torch.hstack(list(edge_idx_dict.values()))
        # edge_indices = self.batch_edge_index(edge_indices[0:1, :],
        #                                      edge_indices[1:2, :],
        #                                      batch_size)
        edge_type = edge_type.repeat(batch_size, 1)
        for k, v in edge_idx_dict.items():
            edge_idx_dict[k] = self.batch_edge_index(
                v[:1], v[1:2], batch_size, edge_idx_dict['contact_edge_index'].max() + 1
            )

        # Filter out edges that are beyond threshold
        # edge_idx_dict['contact_edge_index'], body_rcvrs = self._filter_contact_edge_idx(
        #     edge_idx_dict['contact_edge_index'],
        #     # edge_type,
        #     node_feats['node_pos'],
        #     batch_size
        # )

        n = len(self.robot.rods) * 2
        body_rcvrs = torch.tensor(
            [-1] * n + [1] * n, device=node_pos.device
        ).repeat(batch_size).reshape(-1, 1)

        edge_feats = self._compute_edge_feats(
            node_feats,
            batch_size,
            body_rcvrs=body_rcvrs,
            # body_edge_index=edge_indices[:, edge_type.flatten() == 0],
            # cable_edge_index=edge_indices[:, edge_type.flatten() == 1],
            # contact_edge_index=edge_indices[:, edge_type.flatten() == 2],
            **edge_idx_dict
        )

        edge_type_map = {"body": 0, 'cable': 1, 'contact': 2}
        # all_edge_indices = {k + "_edge_index": edge_indices[:, edge_type.flatten() == v]
        #                     for k, v in edge_type_map.items()}

        # Final graph
        graph = GraphData(
            # edge_index=edge_indices,
            num_nodes=node_pos.shape[0],
            # edge_type=edge_type,
            node_type=node_type,
            body_mask=body_mask,
            edge_type_map=edge_type_map,
            pos=node_feats['node_pos'].clone(),
            vel=node_feats['node_vel'].clone(),
            **node_feats,
            **edge_feats,
            # **all_edge_indices
            **edge_idx_dict
        )

        return graph

    def _contact_edge_index(self,
                            contact_node_idxs: List,
                            grnd_idx: int
                            ) -> torch.Tensor:
        """
        Method to get contact edge indices

        @param contact_node_idxs: indices of nodes that are involved in contact events
        @param grnd_idx: index of ground in non-batched graph
        @return:
        """
        senders = torch.tensor([contact_node_idxs],
                               dtype=torch.long,
                               device=self.device,
                               requires_grad=False)
        receivers = torch.full((1, senders.shape[1]),
                               grnd_idx,
                               dtype=torch.long,
                               device=self.device,
                               requires_grad=False)
        edge_index = torch.vstack([
            torch.hstack([senders, receivers]),
            torch.hstack([receivers, senders])
        ]).detach()

        return edge_index

    def _get_cable_edge_idxs(self):
        return self.robot.get_cable_edge_idxs().to(self.device)

    def _compute_edge_idxs(self) \
            -> Tuple[dict, torch.Tensor]:
        """
        Method to compute different edge type indices
        """
        body_edge_idx = self._body_edge_index()
        cable_edge_idx = self._get_cable_edge_idxs()
        contact_edge_idx = self._contact_edge_index(
            self.robot.get_contact_nodes(),
            body_edge_idx.max() + 1
        )
        edge_type = torch.tensor([
            [0] * body_edge_idx.shape[1]
            + [1] * cable_edge_idx.shape[1]
            + [2] * contact_edge_idx.shape[1]
        ], dtype=torch.long, device=self.device).reshape(-1, 1)

        edge_idx_dict = {
            "body_edge_index": body_edge_idx,
            "cable_edge_index": cable_edge_idx,
            "contact_edge_index": contact_edge_idx
        }

        return edge_idx_dict, edge_type

    def _compute_edge_feats(self,
                            node_feats,
                            batch_size,
                            **kwargs):
        """
        Method to compute all edge feats

        @param node_feats: dictionary of node feats
        @param edge_indices: (2, num edges)
        @param batch_size: size of batch
        @return: Dictionary of feat tensors
        """
        node_pos = node_feats['node_pos']
        node_vels = node_feats['node_vel']
        body_verts = node_feats['node_body_verts']
        body_rcvrs = kwargs['body_rcvrs']

        # Split edge indices
        # edge_type = edge_type.flatten()
        # body_edge_idx = edge_indices[:, edge_type == 0]
        # cable_edge_idx = edge_indices[:, edge_type == 1]
        # contact_edge_idx = edge_indices[:, edge_type == 2]

        body_edge_idx = kwargs['body_edge_index']
        cable_edge_idx = kwargs['cable_edge_index']
        contact_edge_idx = kwargs['contact_edge_index']

        # body edges
        body_edge_feats = self.compute_body_edge_feats(body_edge_idx,
                                                       body_verts,
                                                       node_pos,
                                                       "body")

        # contact edges
        contact_edge_feats = self.compute_contact_edge_feats(
            body_rcvrs,
            contact_edge_idx,
            node_pos,
            node_vels
        )

        # cable edges
        cable_edge_feats = self.compute_cable_edge_feats(
            batch_size,
            cable_edge_idx,
            node_pos,
            node_vels
        )

        edge_feats = {
            **body_edge_feats,
            **contact_edge_feats,
            **cable_edge_feats
        }

        return edge_feats

    def compute_cable_edge_feats(self, batch_size, cable_edge_idx, node_pos, node_vels):
        cable_dists = node_pos[cable_edge_idx[1]] - node_pos[cable_edge_idx[0]]
        cable_dists_norm = cable_dists.norm(dim=1, keepdim=True)
        cable_dir = cable_dists / cable_dists_norm
        cable_rel_vel = node_vels[cable_edge_idx[1], :3] - node_vels[cable_edge_idx[0], :3]
        cable_rel_vel_norm = torch.linalg.vecdot(
            cable_rel_vel,
            cable_dir,
            dim=1
        ).unsqueeze(1)
        cable_stiffness = self.robot.cable_stiffness.repeat(batch_size, 1)
        cable_damping = self.robot.cable_damping.repeat(batch_size, 1)
        act_lengths = torch.hstack([s.actuation_length
                                    for cable in self.robot.actuated_cables.values()
                                    for s in [cable, cable]])
        nonact_lengths = zeros(
            (act_lengths.shape[0], len(self.robot.non_actuated_cables) * 2, 1),
            ref_tensor=act_lengths
        )
        act_lengths = torch.hstack([act_lengths, nonact_lengths]).reshape(-1, 1)
        cable_rest_lengths = (self.robot.cable_rest_length
                              .squeeze(-1)
                              .repeat(batch_size, 1) - act_lengths)
        cable_dl = torch.clamp_min(cable_dists_norm - cable_rest_lengths, 0)
        cable_stiffness_force_mag = cable_stiffness * cable_dl
        cable_damping_force_mag = cable_damping * cable_rel_vel_norm
        cable_edge_feats = {
            'cable_dist': cable_dists,
            'cable_dist_norm': cable_dists_norm,
            'cable_dir': cable_dir,
            'cable_dl': cable_dl,
            'cable_rel_vel_norm': cable_rel_vel_norm,
            'cable_rest_length': cable_rest_lengths,
            'cable_stiffness': cable_stiffness,
            'cable_damping': cable_damping,
            'cable_stiffness_force_mag': cable_stiffness_force_mag,
            'cable_damping_force_mag': cable_damping_force_mag
        }
        return cable_edge_feats

    def compute_housing_contact_edge_feats(self, housing_contact_edge_idx, contact_normal, node_pos, node_vels):
        sphere_radius = list(self.robot.rods.values())[0].sphere_radius
        node_pos0 = node_pos[housing_contact_edge_idx][0] + contact_normal * sphere_radius
        node_pos1 = node_pos[housing_contact_edge_idx][1] - contact_normal * sphere_radius
        contact_dists = node_pos1 - node_pos0

        contact_rel_vel = (node_vels[housing_contact_edge_idx[1], :3] -
                           node_vels[housing_contact_edge_idx[0], :3])
        contact_rel_vel_normal = torch.linalg.vecdot(
            contact_rel_vel,
            contact_normal,
            dim=1
        ).unsqueeze(1)
        contact_tangent = contact_rel_vel - contact_rel_vel_normal * contact_normal
        contact_rel_vel_tangent = contact_tangent.norm(dim=1, keepdim=True)
        contact_rel_vel_tangent = torch.clamp_min(contact_rel_vel_tangent, 1e-8)
        contact_tangent = contact_tangent / contact_rel_vel_tangent
        contact_edge_feats = {
            'contact_dist': contact_dists,
            'contact_normal': contact_normal,
            'contact_tangent': contact_tangent,
            'contact_rel_vel_normal': contact_rel_vel_normal,
            'contact_rel_vel_tangent': contact_rel_vel_tangent,
        }
        return contact_edge_feats

    def compute_contact_edge_feats2(self,
                                    body_rcvrs,
                                    grnd_contact_edge_idx,
                                    housing_contact_edge_idx,
                                    node_pos,
                                    node_vels,
                                    batch_size):
        sphere_radius = list(self.robot.rods.values())[0].sphere_radius

        grnd_con_rcvr = node_pos[grnd_contact_edge_idx[1]]
        grnd_con_rcvr[::2, 2:] = grnd_con_rcvr[::2, 2:] - sphere_radius.squeeze(-1)
        grnd_con_rcvr[1::2, :2] = grnd_con_rcvr[::2, :2]
        grnd_con_sndr = node_pos[grnd_contact_edge_idx[0]]
        grnd_con_sndr[1::2, 2:] = grnd_con_sndr[1::2, 2:] - sphere_radius.squeeze(-1)
        grnd_con_sndr[::2, :2] = grnd_con_sndr[1::2, :2]

        z = torch.tensor([[0, 0, 1]], dtype=node_vels.dtype, device=node_vels.device)
        grnd_con_normal = z * body_rcvrs

        prins = [rod.get_principal_axis() for rod in self.robot.rods.values()]
        housing_con_normal = []
        for i in range(len(prins) - 1):
            for j in range(i + 1, len(prins)):
                n = torch.cross(prins[i], prins[j], dim=1)
                housing_con_normal.extend([n, -n] * len(prins))

        housing_con_normal = torch.hstack(housing_con_normal).reshape(-1, 3)
        housing_node_sndr = node_pos[housing_contact_edge_idx][0] + housing_con_normal * sphere_radius
        housing_node_rcvr = node_pos[housing_contact_edge_idx][1] - housing_con_normal * sphere_radius

        combine_fn = lambda x, y: torch.hstack([
            x.reshape(batch_size, -1),
            y.reshape(batch_size, -1)
        ]).reshape(-1, 3)

        node_sndr_pos = combine_fn(grnd_con_sndr, housing_node_sndr)
        node_rcvr_pos = combine_fn(grnd_con_rcvr, housing_node_rcvr)

        contact_normals = combine_fn(grnd_con_normal, housing_con_normal)
        con_edge_idx = torch.hstack([
            grnd_contact_edge_idx.T.reshape(batch_size, -1),
            housing_contact_edge_idx.T.reshape(batch_size, -1)
        ]).reshape(-1, 2).T

        contact_edge_feats = self._compute_contact_edge_feats(
            node_sndr_pos,
            node_rcvr_pos,
            node_vels[con_edge_idx[0]],
            node_vels[con_edge_idx[1]],
            contact_normals
        )

        return contact_edge_feats

    def _compute_contact_edge_feats(self, snder_pos, rcvr_pos, snder_vel, rcvr_vel, contact_normal):
        contact_dists = rcvr_pos - snder_pos
        contact_rel_vel = rcvr_vel - snder_vel
        contact_rel_vel_normal = torch.linalg.vecdot(
            contact_rel_vel,
            contact_normal,
            dim=1
        ).unsqueeze(1)
        contact_tangent = contact_rel_vel - contact_rel_vel_normal * contact_normal
        contact_rel_vel_tangent = contact_tangent.norm(dim=1, keepdim=True)
        contact_rel_vel_tangent = torch.clamp_min(contact_rel_vel_tangent, 1e-8)
        contact_tangent = contact_tangent / contact_rel_vel_tangent
        contact_edge_feats = {
            'contact_dist': contact_dists,
            'contact_normal': contact_normal,
            'contact_tangent': contact_tangent,
            'contact_rel_vel_normal': contact_rel_vel_normal,
            'contact_rel_vel_tangent': contact_rel_vel_tangent,
        }
        return contact_edge_feats

    def compute_contact_edge_feats(self, body_rcvrs, contact_edge_idx, node_pos, node_vels):
        sphere_radius = list(self.robot.rods.values())[0].sphere_radius
        contact_dists = (node_pos[contact_edge_idx[1], 2:3] - node_pos[contact_edge_idx[0], 2:3])

        contact_close_mask = contact_dists * body_rcvrs - sphere_radius.squeeze(-1) < self.CONTACT_EDGE_THRESHOLD
        contact_dists = contact_dists - body_rcvrs * sphere_radius.squeeze(-1)

        z = torch.tensor([[0, 0, 1]], dtype=node_vels.dtype, device=node_vels.device)
        contact_normal = z * body_rcvrs
        contact_rel_vel = node_vels[contact_edge_idx[1], :3] - node_vels[contact_edge_idx[0], :3]
        contact_rel_vel_normal = torch.linalg.vecdot(
            contact_rel_vel,
            contact_normal,
            dim=1
        ).unsqueeze(1)
        contact_tangent = contact_rel_vel - contact_rel_vel_normal * contact_normal
        contact_rel_vel_tangent = contact_tangent.norm(dim=1, keepdim=True)
        contact_rel_vel_tangent = torch.clamp_min(contact_rel_vel_tangent, 1e-8)
        contact_tangent = contact_tangent / contact_rel_vel_tangent

        contact_edge_feats = {
            'contact_dist': contact_dists,
            'contact_normal': contact_normal,
            'contact_tangent': contact_tangent,
            'contact_rel_vel_normal': contact_rel_vel_normal,
            'contact_rel_vel_tangent': contact_rel_vel_tangent,
            'contact_close_mask': contact_close_mask
        }
        return contact_edge_feats

    def compute_body_edge_feats(self, body_edge_idx, body_verts, node_pos, prefix):
        body_dists = node_pos[body_edge_idx[1]] - node_pos[body_edge_idx[0]]
        body_dists_norm = body_dists.norm(dim=1, keepdim=True)
        body_rest_dists = body_verts[body_edge_idx[1]] - body_verts[body_edge_idx[0]]
        body_rest_dists_norm = body_rest_dists.norm(dim=1, keepdim=True)
        body_edge_feats = {
            prefix + "_dist": body_dists,
            prefix + "_dist_norm": body_dists_norm,
            prefix + "_rest_dist": body_rest_dists,
            prefix + "_rest_dist_norm": body_rest_dists_norm
        }
        return body_edge_feats

    def _filter_contact_edge_idx(self,
                                 contact_edges: torch.Tensor,
                                 # edge_indices: torch.Tensor,
                                 # edge_type: torch.Tensor,
                                 node_pos: torch.Tensor,
                                 batch_size: int
                                 ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Method to filter out edges that are outside of threshold
        @param edge_indices: tensor of edge indices
        @param edge_type: tensor of edge types
        @param node_pos: tensor of node positions
        @param batch_size: size of batch
        @return: Filter edges, edge types, and body receiving nodes
        """
        n = len(self.robot.rods) * 2
        # mask = torch.tensor([True] * edge_indices.shape[1], device=node_pos.device)
        # contact_mask = edge_type.flatten() == 2
        # contact_edges = edge_indices[:, contact_mask]

        body_rcvrs = torch.tensor(
            [-1] * n + [1] * n, device=node_pos.device
        ).repeat(batch_size).reshape(-1, 1)
        dists = node_pos[contact_edges[1], 2:] - node_pos[contact_edges[0], 2:]
        dists = dists * body_rcvrs

        sphere_radius = list(self.robot.rods.values())[0].sphere_radius.squeeze(-1)
        close_dist = dists - sphere_radius < self.CONTACT_EDGE_THRESHOLD

        # mask[contact_mask] = close_dist.flatten()
        # edge_indices = edge_indices[:, mask]
        # edge_type = edge_type[mask]
        contact_edges = contact_edges[:, close_dist.flatten()]
        body_rcvrs = body_rcvrs[close_dist, None]

        return contact_edges, body_rcvrs
        # return edge_indices, edge_type, body_rcvrs


class BatchTensegrityCtrlsDataProcessor(BatchTensegrityDataProcessor):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5):
        super().__init__(tensegrity,
                         con_edge_threshold,
                         dt,
                         max_dist_to_grnd)
        self.NUM_CTRLS_HIST = num_hist

        self.hier_edge_feat_dict['cable']['cable_ctrls'] = self.NUM_CTRLS_HIST

        self.normalizers['cable_ctrls'] = AccumulatedNormalizer(
            (1, self.hier_edge_feat_dict['cable']['cable_ctrls']),
            name='cable_ctrls',
            dtype=self.dtype
        )
        self.normalizers['cable_dl'] = AccumulatedNormalizer(
            (1, len(self.robot.cables)), name='cable_dl',
            dtype=self.dtype
        )

    def build_graph(self,
                    node_pose: torch.Tensor,
                    prev_node_pose: torch.Tensor,
                    batch_size: int,
                    **kwargs) -> GraphData:
        graph = super().build_graph(node_pose, prev_node_pose, batch_size, **kwargs)

        # add ctrl feats to act cables
        ctrls = kwargs['ctrls']
        num_nonact_cables = len(self.robot.non_actuated_cables)
        nonact_ctrls = zeros((ctrls.shape[0], num_nonact_cables, ctrls.shape[2]), ref_tensor=ctrls)
        ctrls = torch.hstack([ctrls, nonact_ctrls]).reshape(-1, self.NUM_CTRLS_HIST)
        ctrls = ctrls.repeat(1, 2).reshape(-1, self.NUM_CTRLS_HIST)
        graph['cable_ctrls'] = ctrls

        return graph


class MultiSimTensegrityDataProcessor(BatchTensegrityDataProcessor):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5,
                 num_sims=10):
        super().__init__(tensegrity,
                         con_edge_threshold,
                         dt,
                         max_dist_to_grnd)
        self.num_sims = n = num_sims
        self.hier_node_feat_dict['node']['node_sim_type'] = num_sims
        self.node_feat_lens['node'] += num_sims

        self.normalizers['node_sim_type'] = DummyNormalizer((1, n), dtype=self.dtype, name="node_sim_type")

    def one_hot_encode(self, batch_idxs):
        num_nodes = self.robot.num_nodes + 1
        batch_idxs = batch_idxs.repeat(1, num_nodes).reshape(-1, 1)
        vecs = torch.zeros(
            (batch_idxs.shape[0], self.num_sims),
            dtype=self.dtype,
            device=self.device
        )
        vecs[torch.arange(batch_idxs.shape[0]), batch_idxs.flatten()] = 1.

        return vecs

    def batch_state_to_graph(self, states: List[torch.Tensor], **kwargs) -> GraphData:
        graph = super().batch_state_to_graph(states)

        batch_idxs = kwargs['dataset_idx']
        feat = self.one_hot_encode(batch_idxs)

        graph['node_sim_type'] = feat

        return graph


class MultiSimMotorTensegrityDataProcessor(BatchTensegrityCtrlsDataProcessor):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5,
                 num_sims=10):
        super().__init__(tensegrity,
                         con_edge_threshold,
                         num_hist,
                         dt,
                         max_dist_to_grnd)
        self.num_sims = n = num_sims
        self.hier_node_feat_dict['node']['node_sim_type'] = num_sims
        self.node_feat_lens['node'] += num_sims

        self.normalizers['node_sim_type'] = DummyNormalizer(
            (1, n), dtype=self.dtype, name="node_sim_type"
        )

    def one_hot_encode(self, batch_idxs):
        num_nodes = self.robot.num_nodes + 1
        batch_idxs = batch_idxs.repeat(1, num_nodes).reshape(-1, 1)
        vecs = torch.zeros(
            (batch_idxs.shape[0], self.num_sims),
            dtype=self.dtype,
            device=self.device
        )
        vecs[torch.arange(batch_idxs.shape[0]), batch_idxs.flatten()] = 1.

        return vecs

    def batch_state_to_graph(self, states: List[torch.Tensor], **kwargs) -> GraphData:
        graph = super().batch_state_to_graph(states, **kwargs)

        batch_idxs = kwargs['dataset_idx']
        feat = self.one_hot_encode(batch_idxs)

        graph['node_sim_type'] = feat

        return graph
