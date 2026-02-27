from typing import List, Tuple, Union, NamedTuple

import torch

from gnn_simulator.gnn_physics.normalizer import AccumulatedNormalizer, DummyNormalizer
from gnn_simulator.robots.tensegrity import TensegrityRobotGNN
from gnn_simulator.state_objects.base_state_object import BaseStateObject
from gnn_simulator.state_objects.primitive_shapes import StaticPrism
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.contact_detection_utils import get_dist_fn
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE
from gnn_simulator.utilities.tensor_utils import zeros, safe_norm


class NodeFeats(NamedTuple):
    node_vel: torch.Tensor
    node_inv_mass: torch.Tensor
    node_inv_inertia: torch.Tensor
    node_dir_from_com: torch.Tensor
    node_dist_from_com_norm: torch.Tensor
    node_dist_to_ground: torch.Tensor
    node_body_verts: torch.Tensor
    node_dist_to_first_node: torch.Tensor
    node_dist_to_first_node_norm: torch.Tensor
    node_prin_axis: torch.Tensor
    node_pos: torch.Tensor
    node_prev_pos: torch.Tensor
    node_sim_type: torch.Tensor
    body_mask: torch.Tensor


class BodyEdgeFeats(NamedTuple):
    body_dist: torch.Tensor
    body_dist_norm: torch.Tensor
    body_rest_dist: torch.Tensor
    body_rest_dist_norm: torch.Tensor


class CableEdgeFeats(NamedTuple):
    cable_dist: torch.Tensor
    cable_dist_norm: torch.Tensor
    cable_dir: torch.Tensor
    # cable_dl: torch.Tensor
    cable_rel_vel_norm: torch.Tensor
    cable_rest_length: torch.Tensor
    cable_stiffness: torch.Tensor
    cable_damping: torch.Tensor
    # cable_stiffness_force_mag: torch.Tensor
    # cable_damping_force_mag: torch.Tensor
    cable_ctrls: torch.Tensor
    cable_actuated_mask: torch.Tensor


class ContactEdgeFeats(NamedTuple):
    contact_dist: torch.Tensor
    contact_normal: torch.Tensor
    contact_tangent: torch.Tensor
    contact_rel_vel_normal: torch.Tensor
    contact_rel_vel_tangent: torch.Tensor
    contact_close_mask: torch.Tensor


class CacheableFeats(NamedTuple):
    node_inv_mass: torch.Tensor
    node_inv_inertia: torch.Tensor
    node_body_verts: torch.Tensor
    body_rest_dist: torch.Tensor
    body_rest_dist_norm: torch.Tensor
    cable_stiffness: torch.Tensor
    cable_damping: torch.Tensor
    cable_actuated_mask: torch.Tensor
    contact_normal: torch.Tensor
    body_edge_idx: torch.Tensor
    body_edge_agg_idx: torch.Tensor
    cable_edge_idx: torch.Tensor
    cable_edge_agg_idx: torch.Tensor
    contact_edge_idx: torch.Tensor
    contact_edge_agg_idx: torch.Tensor
    body_mask: torch.Tensor


class GraphFeats(NamedTuple):
    node_x: torch.Tensor
    body_edge_attr: torch.Tensor
    body_edge_idx: torch.Tensor
    body_edge_agg_idx: torch.Tensor
    cable_edge_attr: torch.Tensor
    cable_edge_idx: torch.Tensor
    cable_edge_agg_idx: torch.Tensor
    contact_edge_attr: torch.Tensor
    contact_edge_idx: torch.Tensor
    contact_edge_agg_idx: torch.Tensor
    contact_close_mask: torch.Tensor
    node_hidden_state: torch.Tensor


class PredGnnAttrs(NamedTuple):
    pos: torch.Tensor
    vel: torch.Tensor
    p_pos: torch.Tensor
    p_vel: torch.Tensor
    pf_dv: torch.Tensor
    p_dv: torch.Tensor
    norm_dv: torch.Tensor
    body_mask: torch.Tensor
    node_hidden_state: torch.Tensor


class FastTensegrityGraphDataProcessor(BaseStateObject):
    robot: TensegrityRobotGNN

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_out_steps: int = 1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5,
                 cache_batch_sizes: List | None = None,
                 num_sims=10,
                 recur_latent_dim=1024,
                 num_ctrls_hist=2,
                 rest_lens_or_ctrls='rest_lens'):
        super().__init__('fast data processor')
        """
        @param tensegrity: robot object
        @param con_edge_threshold: threshold to attach edge between ground and endcap node
        @param num_steps_ahead: how many steps training traj will be
        @param num_hist: how many steps behind to attach to features
        @param dt: timestep size
        @param max_dist_to_grnd: clip value for dist to ground feature
        """
        with torch.no_grad():
            self.MAX_DIST_TO_GRND = max_dist_to_grnd
            self.CONTACT_EDGE_THRESHOLD = con_edge_threshold
            self.NUM_OUT_STEPS = num_out_steps
            self.NUM_HIST = num_hist
            self.NUM_CTRLS_HIST = num_ctrls_hist
            self.NUM_SIMS = num_sims

            self.recur_latent_dim = recur_latent_dim
            self.rest_lens_or_ctrls = rest_lens_or_ctrls

            self.node_feat_dict = {
                'node_vel': 3,
                'node_inv_mass': 1,
                'node_inv_inertia': 3,
                'node_dist_to_ground': 1,
                'node_body_verts': 3,
                'node_dist_to_first_node': 3,
                'node_dist_to_first_node_norm': 1,
                'node_dir_from_com': 3,
                'node_dist_from_com_norm': 1,
                'node_prin_axis': 3,
                'node_sim_type': num_sims
            }

            self.body_edge_feat_dict = {
                'body_dist': 3,
                'body_dist_norm': 1,
                'body_rest_dist': 3,
                'body_rest_dist_norm': 1,
            }

            self.cable_edge_feat_dict = {
                'cable_dist': 3,
                'cable_dist_norm': 1,
                'cable_dir': 3,
                'cable_rel_vel_norm': 1,
                'cable_stiffness': 1,
                'cable_damping': 1,
                # 'cable_stiffness_force_mag': 1,
                # 'cable_damping_force_mag': 1,
                # 'cable_ctrls': num_ctrls_hist + num_out_steps
            }
            self.cable_edge_feat_dict['cable_rest_length'] = (
                1 if rest_lens_or_ctrls == 'ctrls' else self.NUM_OUT_STEPS)

            if rest_lens_or_ctrls == 'ctrls':
                self.cable_edge_feat_dict['cable_ctrls'] = num_ctrls_hist + num_out_steps

            self.contact_edge_feat_dict = {
                'contact_dist': 3,
                'contact_normal': 3,
                'contact_tangent': 3,
                'contact_rel_vel_normal': 1,
                'contact_rel_vel_tangent': 1,
            }

            self.hier_node_feat_dict = {
                'node': self.node_feat_dict
            }
            self.hier_edge_feat_dict = {
                'body': self.body_edge_feat_dict,
                'cable': self.cable_edge_feat_dict,
                'contact': self.contact_edge_feat_dict
            }

            self.dt = torch.tensor([[dt]], dtype=DEFAULT_DTYPE)
            self.robot = tensegrity

            # Compute node and edge feat sizes, used for initializing encoders' input size
            self.node_feat_lens = {k: sum(v.values()) for k, v in self.hier_node_feat_dict.items()}
            self.edge_feat_lens = {k: sum(v.values()) for k, v in self.hier_edge_feat_dict.items()}

            # flatten node and edge feats dicts to initialize feat normalizers
            flatten_node_feats = {k2: v
                                  for k1, d in self.hier_node_feat_dict.items()
                                  for k2, v in d.items()}
            flatten_edge_feats = {k2: v
                                  for k1, d in self.hier_edge_feat_dict.items()
                                  for k2, v in d.items()}

            # Initialize normalizer dict
            self.normalizers = {
                k: AccumulatedNormalizer((1, v), name=k, dtype=self.dtype)
                for k, v in {**flatten_node_feats, **flatten_edge_feats}.items()
            }

            if self.rest_lens_or_ctrls == 'ctrls':
                self.normalizers['cable_ctrls'] = DummyNormalizer(
                    (1, self.hier_edge_feat_dict['cable']['cable_ctrls']),
                    name='cable_ctrls',
                    dtype=self.dtype,
                )

            self.normalizers['node_sim_type'] = DummyNormalizer(
                (1, 1),
                name='node_sim_type',
                dtype=self.dtype,
            )

            self.normalizers['node_dv'] = AccumulatedNormalizer(
                (1, 3 * num_out_steps),
                name='node_dv',
                dtype=self.dtype
            )
            self.normalizers['cable_dl'] = AccumulatedNormalizer(
                (1, num_out_steps),
                name='cable_dl',
                dtype=self.dtype
            )

            robot_rods = list(self.robot.rods.values())
            self.first_node_idx = robot_rods[0].sphere_idx0
            self.last_node_idx = robot_rods[-1].sphere_idx1 + sum([r.body_verts.shape[0] for r in robot_rods[:-1]])
            self.sphere0_idx = robot_rods[0].sphere_idx0
            self.sphere1_idx = robot_rods[0].sphere_idx1
            self.sphere_radius = robot_rods[0].sphere_radius.squeeze(-1)

            contact_node_idx = self.robot.num_nodes
            self.body_edge_idx_template = self._body_edge_index()
            self.cable_edge_idx_template = self._get_cable_edge_idxs()
            self.contact_edge_idx_template = self._contact_edge_index(contact_node_idx)

            self.body_mask = self._get_body_mask(1, self.device)

            num_nodes = contact_node_idx + 1
            self.body_edge_agg_idx_template = self._compute_edge_agg_idx(self.body_edge_idx_template, num_nodes)
            self.cable_edge_agg_idx_template = self._compute_edge_agg_idx(self.cable_edge_idx_template, num_nodes)
            self.contact_edge_agg_idx_template = self._compute_edge_agg_idx(self.contact_edge_idx_template, num_nodes)

            self.robot_inv_mass = torch.vstack([
                self.robot.inv_mass, torch.zeros_like(self.robot.inv_mass[:1])
            ])
            self.robot_inv_inertia = torch.vstack([
                self.robot.inv_inertia.clone(), torch.zeros_like(self.robot.inv_inertia[:1])
            ])

            self.robot_cable_stiffness = self.robot.cable_stiffness.clone()
            self.robot_cable_damping = self.robot.cable_damping.clone()

            self.body_verts = self.robot.body_verts.squeeze(-1)
            self.body_verts = torch.vstack((self.body_verts, torch.zeros_like(self.body_verts[:1])))

            body_senders_idx, body_rcvrs_idx = self.body_edge_idx_template[0], self.body_edge_idx_template[1]
            self.body_rest_dists = (
                    self.body_verts[body_rcvrs_idx] - self.body_verts[body_senders_idx]
            )
            self.body_rest_dists_norm = self.body_rest_dists.norm(dim=1, keepdim=True)

            n_rods = len(self.robot.rods) * 2
            body_rcvrs = torch.tensor(
                [[-1] * n_rods + [1] * n_rods], device=self.device,
            ).reshape(-1, 1)
            self.contact_normal = body_rcvrs * torch.tensor([[0., 0., 1.]],
                                                            dtype=self.dtype,
                                                            device=self.device)

            num_act_cables = len(self.robot.actuated_cables) * 2
            num_nonact_cables = len(self.robot.non_actuated_cables) * 2
            self.cable_act_mask = torch.tensor(
                [True] * num_act_cables + [False] * num_nonact_cables,
                device=self.device,
            ).reshape(-1, 1)

            self._feats_batch_cache = {}
            if cache_batch_sizes is not None:
                self.precompute_and_cache_batch_sizes(cache_batch_sizes)

    def to(self, device: Union[str, torch.device]):
        super().to(device)
        self.robot.to(device)
        self.dt = self.dt.to(device)
        self.sphere_radius = self.sphere_radius.to(device)
        self.body_mask = self.body_mask.to(device)

        self.body_edge_idx_template = self.body_edge_idx_template.to(device)
        self.cable_edge_idx_template = self.cable_edge_idx_template.to(device)
        self.contact_edge_idx_template = self.contact_edge_idx_template.to(device)

        self.body_edge_agg_idx_template = self.body_edge_agg_idx_template.to(device)
        self.cable_edge_agg_idx_template = self.cable_edge_agg_idx_template.to(device)
        self.contact_edge_agg_idx_template = self.contact_edge_agg_idx_template.to(device)

        self.robot_inv_mass = self.robot_inv_mass.to(device)
        self.robot_inv_inertia = self.robot_inv_inertia.to(device)
        self.robot_cable_stiffness = self.robot_cable_stiffness.to(device)
        self.robot_cable_damping = self.robot_cable_damping.to(device)

        self.body_verts = self.body_verts.to(device)
        self.body_rest_dists = self.body_rest_dists.to(device)
        self.body_rest_dists_norm = self.body_rest_dists_norm.to(device)

        self.contact_normal = self.contact_normal.to(device)

        for normalizer in self.normalizers.values():
            normalizer.to(device)

        for k, cache in self._feats_batch_cache.items():
            tmp_dict = cache._asdict()
            for kk, v in tmp_dict.items():
                tmp_dict[kk] = v.to(device)
            self._feats_batch_cache[k] = CacheableFeats(**tmp_dict)

        return self

    @property
    def cached_batch_size_keys(self):
        return list(self._feats_batch_cache.keys())

    def precompute_and_cache_batch_sizes(self, batch_sizes, overwrite=False):
        for bsize in batch_sizes:
            if overwrite or bsize not in self._feats_batch_cache:
                self._feats_batch_cache[bsize] = self._batch_feats(bsize)

    def start_normalizers(self):
        """
        Set accumulation flag of all normalizers to true
        """
        for normalizer in self.normalizers.values():
            normalizer.start_accum()

    def stop_normalizers(self):
        """
        Set accumulation flag of all normalizers to talse
        """
        for normalizer in self.normalizers.values():
            normalizer.stop_accum()

    def normalizer_to_dict(self):
        normalizer_dict = {
            k: v.to_dict()
            for k, v in self.normalizers.items()
        }
        return normalizer_dict

    def _build_csr_agg_mat(self, edge_index, num_nodes):
        node_idx = edge_index[1:]
        edge_attr_idx = torch.arange(node_idx.shape[1]).reshape(1, -1).to(edge_index.device)
        mat_indices = torch.vstack([node_idx, edge_attr_idx])
        vals = torch.ones(mat_indices.shape[1], device=node_idx.device)

        agg_mat = torch.sparse_coo_tensor(
            mat_indices, vals, (num_nodes, node_idx.shape[1]), device=node_idx.device
        ).coalesce().to_sparse_csr()

        return agg_mat

    def _batch_feats(self, bsize: int):
        nnodes = self.contact_edge_idx_template.max() + 1
        body_edge_idx = self.batch_edge_index(self.body_edge_idx_template, bsize, nnodes)
        cable_edge_idx = self.batch_edge_index(self.cable_edge_idx_template, bsize, nnodes)
        contact_edge_idx = self.batch_edge_index(self.contact_edge_idx_template, bsize, nnodes)

        body_edge_agg_idx = self._batch_edge_agg_idx(
            self.body_edge_agg_idx_template, self.body_edge_idx_template.shape[1], bsize
        )
        cable_edge_agg_idx = self._batch_edge_agg_idx(
            self.cable_edge_agg_idx_template, self.cable_edge_idx_template.shape[1], bsize
        )
        contact_edge_agg_idx = self._batch_edge_agg_idx(
            self.contact_edge_agg_idx_template, self.contact_edge_idx_template.shape[1], bsize
        )

        robot_inv_mass = self.robot_inv_mass.repeat(bsize, 1)
        robot_inv_inertia = self.robot_inv_inertia.repeat(bsize, 1)
        robot_cable_stiffness = self.robot_cable_stiffness.repeat(bsize, 1)
        robot_cable_damping = self.robot_cable_damping.repeat(bsize, 1)

        body_verts = self.body_verts.repeat(bsize, 1)
        body_rest_dists = self.body_rest_dists.repeat(bsize, 1)
        body_rest_dists_norm = self.body_rest_dists_norm.repeat(bsize, 1)

        contact_normal = self.contact_normal.repeat(bsize, 1)
        body_mask = self.body_mask.repeat(bsize, 1)

        cable_act_mask = self.cable_act_mask.repeat(bsize, 1)

        return CacheableFeats(
            node_inv_mass=robot_inv_mass,
            node_inv_inertia=robot_inv_inertia,
            node_body_verts=body_verts,
            body_rest_dist=body_rest_dists,
            body_rest_dist_norm=body_rest_dists_norm,
            contact_normal=contact_normal,
            cable_stiffness=robot_cable_stiffness,
            cable_damping=robot_cable_damping,
            cable_actuated_mask=cable_act_mask,
            body_edge_idx=body_edge_idx,
            body_edge_agg_idx=body_edge_agg_idx,
            cable_edge_idx=cable_edge_idx,
            cable_edge_agg_idx=cable_edge_agg_idx,
            contact_edge_idx=contact_edge_idx,
            contact_edge_agg_idx=contact_edge_agg_idx,
            body_mask=body_mask,
        )

    def _compute_edge_agg_idx(self, edge_idx_template, template_max_node):
        edge_agg_idx = [[] for _ in range(template_max_node)]
        for i in range(edge_idx_template.shape[1]):
            edge_agg_idx[edge_idx_template[1, i]].append(i)

        max_len = max([len(e) for e in edge_agg_idx])
        for i in range(len(edge_agg_idx)):
            if len(edge_agg_idx[i]) < max_len:
                edge_agg_idx[i] = edge_agg_idx[i] + [-1] * (max_len - len(edge_agg_idx[i]))

        edge_agg_idx = torch.tensor(edge_agg_idx, dtype=torch.int, device=edge_idx_template.device)
        return edge_agg_idx

    def _batch_edge_agg_idx(self, edge_agg_idx_template, num_template_edges, bsize):
        edge_agg_idxs = []
        for i in range(bsize):
            edge_agg_idx_copy = edge_agg_idx_template.clone()
            edge_agg_idx_copy[edge_agg_idx_copy != -1] += num_template_edges * i
            edge_agg_idxs.append(edge_agg_idx_copy)

        edge_agg_idxs = torch.vstack(edge_agg_idxs)
        return edge_agg_idxs

    def batch_edge_index(self,
                         edge_index: torch.Tensor,
                         batch_size: int,
                         num_nodes: torch.Tensor,
                         ) -> torch.Tensor:
        """
        Expand edge indices from one graph to a batch of graphs. Method assumes
        same size and connections

        @param senders: indices of starting nodes
        @param receivers: indices of ending nodes
        @param batch_size: int
        @return:
        """
        # Assume graphs are the same size and have the same connections
        senders = edge_index[:1].repeat(batch_size, 1)
        receivers = edge_index[1:].repeat(batch_size, 1)

        offsets = num_nodes * torch.arange(
            0, batch_size,
            dtype=torch.int,
            device=senders.device
        ).reshape(-1, 1)

        senders = (senders + offsets).reshape(1, -1)
        receivers = (receivers + offsets).reshape(1, -1)

        edge_indices = torch.vstack([senders, receivers])
        return edge_indices

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

        def compute_state(node_pos, prev_node_pos):
            curr_com_pos = node_pos.reshape(-1, num_nodes, 3).mean(dim=1)
            prev_com_pos = prev_node_pos.reshape(-1, num_nodes, 3).mean(dim=1)

            lin_vel = (curr_com_pos - prev_com_pos).unsqueeze(-1) / self.dt

            idx_0 = self.sphere0_idx
            idx_1 = self.sphere1_idx

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

        node_pos = node_pos.reshape(node_pos.shape[0], node_pos.shape[1], -1)
        prev_node_pos = prev_node_pos.unsqueeze(-1)
        all_node_pos = torch.cat([prev_node_pos, node_pos], dim=-1)

        states = []
        for i in range(node_pos.shape[-1]):
            node_pos = all_node_pos[..., i + 1]
            prev_node_pos = all_node_pos[..., i]

            se3_state = compute_state(node_pos, prev_node_pos)
            states.append(se3_state)

        states = torch.cat(states, dim=-1)
        return states

    def _normalize_and_hstack(self, raw_feats, feat_dict):
        feats_list = [
            self.normalizers[k](getattr(raw_feats, k))
            for k in feat_dict.keys()
        ]
        feats = torch.hstack(feats_list)
        return feats

    def get_normalize_feats(
            self,
            node_raw_feats: NodeFeats,
            body_edge_feats: BodyEdgeFeats,
            cable_edge_feats: CableEdgeFeats,
            contact_edge_feats: ContactEdgeFeats
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Normalize and concat all node and edge feats to form input feat vectors

        @param graph: graph data object with raw features
        @return: graph filled with node and edge feats
        """
        node_x = self._normalize_and_hstack(
            node_raw_feats, self.node_feat_dict
        )
        body_edge_attr = self._normalize_and_hstack(
            body_edge_feats, self.body_edge_feat_dict
        )
        cable_edge_attr = self._normalize_and_hstack(
            cable_edge_feats, self.cable_edge_feat_dict
        )
        contact_edge_attr = self._normalize_and_hstack(
            contact_edge_feats, self.contact_edge_feat_dict
        )

        return node_x, body_edge_attr, cable_edge_attr, contact_edge_attr

    def _inject_grnd_feat(self, feat, grnd_val_tensor):
        num_nodes = self.robot.num_nodes
        hsize = feat.shape[1]

        feat = feat.reshape(-1, num_nodes * hsize)
        grnd_val_tensor = grnd_val_tensor.repeat(feat.shape[0], 1)
        feat_w_grnd = torch.hstack([feat, grnd_val_tensor]).reshape(-1, hsize)

        return feat_w_grnd

    def pose2node(self,
                  pos: torch.Tensor,
                  quat: torch.Tensor,
                  batch_size: int,
                  augment_grnd=False,
                  ) -> torch.Tensor:
        """
        SE(3) pose to 3D node poses
        @param pose: (batch size * num rods, 7)
        @return: tensor (batch_size * num nodes per graph, 3)
        """
        # Get positions of nodes in body frame
        body_verts = torch.vstack(
            [r.body_verts.transpose(0, 2) for r in self.robot.rods.values()]
        ).to(pos.device).repeat(batch_size, 1, 1)

        # Rotate and translate body verts to world frame
        node_pos = torch_quaternion.rotate_vec_quat(quat, body_verts)
        node_pos = node_pos + pos
        node_pos = node_pos.transpose(1, 2).reshape(-1, 3)

        if augment_grnd:
            grnd_node_pos = zeros((batch_size, 3), ref_tensor=node_pos)
            node_pos = torch.hstack([node_pos.reshape(batch_size, -1), grnd_node_pos])
            node_pos = node_pos.reshape(-1, 3)

        return node_pos

    def _get_body_verts(self, batch_size, device):
        body_verts = (self.robot.body_verts
                      .to(device)
                      .transpose(0, 2)
                      .repeat(batch_size, 1, 1))
        return body_verts

    def _compute_shape_feats(self, node_pos, batch_size):
        """
        Assume no ground node in node_pos yet
        """
        num_nodes = node_pos.shape[0] // batch_size

        first_node = node_pos[self.first_node_idx::num_nodes].repeat(1, num_nodes).reshape(-1, 3)
        last_node = node_pos[self.last_node_idx::num_nodes].repeat(1, num_nodes).reshape(-1, 3)

        x_dir = torch.hstack([
            (last_node - first_node)[:, :2],
            torch.zeros_like(last_node[:, :1])
        ])
        x_dir = safe_norm(x_dir)
        z_dir = torch.tensor(
            [[0, 0, 1]],
            dtype=self.dtype,
            device=self.device
        ).repeat(x_dir.shape[0], 1)
        y_dir = torch.cross(z_dir, x_dir, dim=1)
        y_dir = safe_norm(y_dir)
        rot_mat = torch.stack([x_dir, y_dir, z_dir], dim=2)

        dist_first_node = (node_pos - first_node).unsqueeze(-1)
        dist_first_node = rot_mat.transpose(1, 2) @ dist_first_node
        dist_first_node = dist_first_node.squeeze(-1)
        dist_first_node_norm = dist_first_node.norm(dim=1, keepdim=True)

        return dist_first_node, dist_first_node_norm

    def _compute_prin_feat(self, node_pos):
        num_nodes, num_nodes_per_rod = self.robot.num_nodes, self.robot.num_nodes_per_rod

        node_pos_ = node_pos.reshape(-1, 3 * num_nodes_per_rod)
        prin = (node_pos_[:, 3 * self.sphere1_idx: 3 * self.sphere1_idx + 3]
                - node_pos_[:, 3 * self.sphere0_idx: 3 * self.sphere0_idx + 3])
        prin = prin / prin.norm(dim=1, keepdim=True)
        prin = prin.repeat(1, num_nodes_per_rod).reshape(-1, 3)

        return prin

    def _compute_node_feats(self,
                            node_pos: torch.Tensor,
                            prev_node_pos: torch.Tensor,
                            batch_size: int,
                            batch_pos: torch.Tensor,
                            **kwargs
                            ) -> NodeFeats:
        """
        Method to compute all node feats based on curr and prev node poses

        @param node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param prev_node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param batch_size: size of batch
        @return: Dictionary of feat tensors
        """

        # Pre adding ground node
        num_nodes = self.robot.num_nodes_per_rod
        com_pos = batch_pos.repeat(1, num_nodes, 1).reshape(-1, 3)
        dist_from_com = node_pos - com_pos
        dist_from_com_norm = dist_from_com.norm(dim=1, keepdim=True)
        dir_from_com = safe_norm(dist_from_com)

        node_vels = (node_pos - prev_node_pos) / self.dt.squeeze(-1)

        dist_to_ground = node_pos[:, 2:3] - self.sphere_radius
        dist_to_ground = torch.clamp_max(dist_to_ground, self.MAX_DIST_TO_GRND)

        dist_first_node, dist_first_node_norm = self._compute_shape_feats(node_pos, batch_size)
        node_prin = self._compute_prin_feat(node_pos)

        # Post ground node
        grnd_ten1 = zeros((1, 3), ref_tensor=node_pos)
        grnd_ten2 = torch.tensor([0., 0., 1.], dtype=self.dtype, device=self.device)

        node_pos = self._inject_grnd_feat(node_pos, grnd_ten1)
        node_vels = self._inject_grnd_feat(node_vels, grnd_ten1)
        dist_from_com_norm = self._inject_grnd_feat(dist_from_com_norm, grnd_ten1[:, :1])
        dir_from_com = self._inject_grnd_feat(dir_from_com, grnd_ten1)
        dist_to_ground = self._inject_grnd_feat(dist_to_ground, grnd_ten1[:, :1])
        dist_first_node = self._inject_grnd_feat(dist_first_node, grnd_ten1)
        dist_first_node_norm = self._inject_grnd_feat(dist_first_node_norm, grnd_ten1[:, :1])
        node_prin = self._inject_grnd_feat(node_prin, grnd_ten2)

        # Need to cache batch size before data processor call
        body_verts = self._feats_batch_cache[batch_size].node_body_verts
        inv_mass = self._feats_batch_cache[batch_size].node_inv_mass
        inv_inertia = self._feats_batch_cache[batch_size].node_inv_inertia
        body_mask = self._feats_batch_cache[batch_size].body_mask

        dataset_idx = kwargs['dataset_idx']
        if isinstance(dataset_idx, int):
            dataset_idx = torch.tensor([[dataset_idx]], dtype=torch.int, device=self.device)

        node_feats = NodeFeats(
            node_vel=node_vels,
            node_inv_mass=inv_mass,
            node_inv_inertia=inv_inertia,
            node_dir_from_com=dir_from_com,
            node_dist_from_com_norm=dist_from_com_norm,
            node_dist_to_ground=dist_to_ground,
            node_body_verts=body_verts,
            node_dist_to_first_node=dist_first_node,
            node_dist_to_first_node_norm=dist_first_node_norm,
            node_pos=node_pos,
            node_prev_pos=prev_node_pos,
            node_prin_axis=node_prin,
            node_sim_type=self._one_hot_encode(dataset_idx),
            body_mask=body_mask
        )

        return node_feats

    def _one_hot_encode(self, batch_idxs):
        num_nodes = self.robot.num_nodes + 1
        batch_idxs = batch_idxs.repeat(1, num_nodes).reshape(-1, 1)
        vecs = torch.zeros(
            (batch_idxs.shape[0], self.NUM_SIMS),
            dtype=self.dtype,
            device=self.device
        )
        vecs[torch.arange(batch_idxs.shape[0], dtype=torch.int), batch_idxs.flatten()] = 1.

        return vecs

    def _body_edge_index(self) -> torch.Tensor:
        """
        Get
        @return:
        """
        senders = self.robot.template_idx[:1].to(self.device)
        receivers = self.robot.template_idx[1:].to(self.device)

        edge_index = torch.vstack([senders, receivers])

        return edge_index

    def _get_cable_edge_idxs(self) -> torch.Tensor:
        return self.robot.get_cable_edge_idxs().to(self.device)

    def _contact_edge_index(self,
                            grnd_idx: int
                            ) -> torch.Tensor:
        """
        Method to get contact edge indices

        @param contact_node_idxs: indices of nodes that are involved in contact events
        @param grnd_idx: index of ground in non-batched graph
        @return:
        """
        senders = torch.tensor([self.robot.get_contact_nodes()],
                               dtype=torch.int,
                               device=self.device)
        receivers = torch.full((1, len(self.robot.get_contact_nodes())),
                               grnd_idx,
                               dtype=torch.int,
                               device=self.device)
        edge_index = torch.vstack([
            torch.hstack([senders, receivers]),
            torch.hstack([receivers, senders])
        ]).detach()

        return edge_index

    def _compute_edge_idxs(self, batch_size) \
            -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor,
            torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Method to compute different edge type indices
        """
        # Need to cache batch size before data processor call
        body_edge_idx = self._feats_batch_cache[batch_size].body_edge_idx
        body_edge_agg_idx = self._feats_batch_cache[batch_size].body_edge_agg_idx
        cable_edge_idx = self._feats_batch_cache[batch_size].cable_edge_idx
        cable_edge_agg_idx = self._feats_batch_cache[batch_size].cable_edge_agg_idx
        contact_edge_idx = self._feats_batch_cache[batch_size].contact_edge_idx
        contact_edge_agg_idx = self._feats_batch_cache[batch_size].contact_edge_agg_idx

        return (body_edge_idx, body_edge_agg_idx,
                cable_edge_idx, cable_edge_agg_idx,
                contact_edge_idx, contact_edge_agg_idx)

    def _compute_body_edge_feats(self, body_edge_idx, node_pos, batch_size) -> BodyEdgeFeats:
        body_dists = node_pos[body_edge_idx[1]] - node_pos[body_edge_idx[0]]
        body_dists_norm = body_dists.norm(dim=1, keepdim=True)

        # Need to cache batch size before data processor call
        body_rest_dists = self._feats_batch_cache[batch_size].body_rest_dist
        body_rest_dists_norm = self._feats_batch_cache[batch_size].body_rest_dist_norm

        body_edge_feats = BodyEdgeFeats(
            body_dist=body_dists,
            body_dist_norm=body_dists_norm,
            body_rest_dist=body_rest_dists,
            body_rest_dist_norm=body_rest_dists_norm
        )
        return body_edge_feats

    def _compute_contact_edge_feats(self, contact_edge_idx, node_pos, node_vels, batch_size) -> ContactEdgeFeats:
        n_rods = len(self.robot.rods) * 2
        body_rcvrs = torch.tensor(
            [[-1] * n_rods + [1] * n_rods], device=node_pos.device
        ).repeat(batch_size, 1).reshape(-1, 1)
        contact_dists = (node_pos[contact_edge_idx[1], 2:3] - node_pos[contact_edge_idx[0], 2:3])

        contact_close_mask = contact_dists * body_rcvrs - self.sphere_radius.squeeze(-1) < self.CONTACT_EDGE_THRESHOLD
        contact_dists = contact_dists - body_rcvrs * self.sphere_radius.squeeze(-1)

        # Need to cache batch size before data processor call
        contact_normal = self._feats_batch_cache[batch_size].contact_normal

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

        contact_edge_feats = ContactEdgeFeats(
            contact_dist=contact_dists,
            contact_normal=contact_normal,
            contact_tangent=contact_tangent,
            contact_rel_vel_normal=contact_rel_vel_normal,
            contact_rel_vel_tangent=contact_rel_vel_tangent,
            contact_close_mask=contact_close_mask
        )
        return contact_edge_feats

    def _compute_cable_edge_feats(self, cable_edge_idx, node_pos, node_vels, batch_size, ctrls) -> CableEdgeFeats:
        cable_dists = node_pos[cable_edge_idx[1]] - node_pos[cable_edge_idx[0]]
        cable_dists_norm = cable_dists.norm(dim=1, keepdim=True)
        cable_dir = cable_dists / cable_dists_norm
        cable_rel_vel = node_vels[cable_edge_idx[1], :3] - node_vels[cable_edge_idx[0], :3]
        cable_rel_vel_norm = torch.linalg.vecdot(
            cable_rel_vel,
            cable_dir,
            dim=1
        ).unsqueeze(1)

        # Need to cache batch size before data processor call
        cable_stiffness = self._feats_batch_cache[batch_size].cable_stiffness
        cable_damping = self._feats_batch_cache[batch_size].cable_damping

        if self.rest_lens_or_ctrls == 'ctrls':
            cable_act_rest_lengths = torch.hstack([
                c.rest_length
                for cable in self.robot.actuated_cables.values()
                for c in [cable, cable]
            ])
        else:
            cable_act_rest_lengths = self.robot.gnn_rest_lens.repeat_interleave(2, dim=1)

        cable_non_act_rest_lengths = torch.hstack([
            c.rest_length.repeat(batch_size, 1, cable_act_rest_lengths.shape[-1])
            for cable in self.robot.non_actuated_cables.values()
            for c in [cable, cable]
        ])
        cable_rest_lengths = torch.hstack([
            cable_act_rest_lengths,
            cable_non_act_rest_lengths
        ]).reshape(-1, self.cable_edge_feat_dict['cable_rest_length'])

        # cable_rest_lengths = cable_act_rest_lengths.reshape(-1, self.cable_edge_feat_dict['cable_rest_length'])

        # cable_dl = torch.clamp_min(cable_dists_norm - cable_rest_lengths, 0)
        # cable_stiffness_force_mag = cable_stiffness * cable_dl
        # cable_damping_force_mag = cable_damping * cable_rel_vel_norm

        if self.rest_lens_or_ctrls == 'ctrls':
            num_nonact_cables = len(self.robot.non_actuated_cables)
            num_ctrls = self.NUM_CTRLS_HIST + self.NUM_OUT_STEPS
            nonact_ctrls = zeros((ctrls.shape[0], num_nonact_cables, num_ctrls), ref_tensor=ctrls)
            cable_ctrls = (torch.hstack([ctrls, nonact_ctrls])
                           .repeat_interleave(2, dim=1)
                           .reshape(-1, num_ctrls))
        else:
            cable_ctrls = None

        cable_act_mask = self._feats_batch_cache[batch_size].cable_actuated_mask

        cable_edge_feats = CableEdgeFeats(
            cable_dist=cable_dists,
            cable_dist_norm=cable_dists_norm,
            cable_dir=cable_dir,
            # cable_dl=cable_dl,
            cable_rel_vel_norm=cable_rel_vel_norm,
            cable_rest_length=cable_rest_lengths,
            cable_stiffness=cable_stiffness,
            cable_damping=cable_damping,
            # cable_stiffness_force_mag=cable_stiffness_force_mag,
            # cable_damping_force_mag=cable_damping_force_mag,
            cable_ctrls=cable_ctrls,
            cable_actuated_mask=cable_act_mask
        )
        return cable_edge_feats

    def _compute_edge_feats(self,
                            node_feats,
                            body_edge_idx,
                            cable_edge_idx,
                            contact_edge_idx,
                            batch_size,
                            **kwargs):
        """
        Method to compute all edge feats

        @param node_feats: dictionary of node feats
        @param edge_indices: (2, num edges)
        @param batch_size: size of batch
        @return: Dictionary of feat tensors
        """
        # body edges
        body_edge_feats = self._compute_body_edge_feats(
            body_edge_idx,
            node_feats.node_pos,
            batch_size
        )

        # contact edges
        contact_edge_feats = self._compute_contact_edge_feats(
            contact_edge_idx,
            node_feats.node_pos,
            node_feats.node_vel,
            batch_size,
        )

        # cable edges
        cable_edge_feats = self._compute_cable_edge_feats(
            cable_edge_idx,
            node_feats.node_pos,
            node_feats.node_vel,
            batch_size,
            ctrls=kwargs['ctrls'],
        )

        return body_edge_feats, cable_edge_feats, contact_edge_feats

    def _get_body_mask(self, batch_size, device):
        body_mask = torch.tensor(
            [True] * self.robot.num_nodes + [False],
            dtype=torch.bool,
            device=device
        ).repeat(batch_size, 1).reshape(-1, 1)
        return body_mask

    def forward(self,
                batch_state: torch.Tensor,
                **kwargs: torch.Tensor):
        batch_size = batch_state.shape[0]

        # Convert batch state to node_pos and prev_node_pos
        batch_state_ = batch_state.reshape(-1, 13, 1)
        batch_pos = batch_state_[:, :3]
        batch_quat = batch_state_[:, 3:7]
        batch_lin_vel = batch_state_[:, 7:10]
        batch_ang_vel = batch_state_[:, 10:13]

        batch_prev_pos = batch_state_[:, :3] - self.dt * batch_lin_vel
        batch_prev_quat = torch_quaternion.update_quat(
            batch_state_[:, 3:7], -batch_ang_vel, self.dt
        )

        node_pos = self.pose2node(
            batch_pos, batch_quat, batch_size
        )
        prev_node_pos = self.pose2node(
            batch_prev_pos, batch_prev_quat, batch_size
        )

        # Compute node feats
        node_raw_feats = self._compute_node_feats(
            node_pos,
            prev_node_pos,
            batch_size,
            batch_pos,
            dataset_idx=kwargs['dataset_idx']
        )

        node_hidden_state = zeros(
            (node_raw_feats.node_pos.shape[0], self.recur_latent_dim),
            ref_tensor=node_raw_feats.node_pos
        )

        # Compute edge indices
        edge_vals = self._compute_edge_idxs(batch_size)
        body_edge_idx, body_edge_agg_idx = edge_vals[:2]
        cable_edge_idx, cable_edge_agg_idx, = edge_vals[2:4]
        contact_edge_idx, contact_edge_agg_idx = edge_vals[4:]

        # Compute edge feats
        body_edge_feats, cable_edge_feats, contact_edge_feats = self._compute_edge_feats(
            node_raw_feats,
            body_edge_idx,
            cable_edge_idx,
            contact_edge_idx,
            batch_size,
            ctrls=kwargs['ctrls'],
        )

        node_x, body_edge_attr, cable_edge_attr, contact_edge_attr = (
            self.get_normalize_feats(node_raw_feats, body_edge_feats, cable_edge_feats, contact_edge_feats)
        )

        raw_feats = (node_raw_feats, body_edge_feats, cable_edge_feats, contact_edge_feats)

        graph_feats = GraphFeats(
            node_x=node_x,
            body_edge_idx=body_edge_idx,
            cable_edge_idx=cable_edge_idx,
            contact_edge_idx=contact_edge_idx,
            body_edge_attr=body_edge_attr,
            cable_edge_attr=cable_edge_attr,
            contact_edge_attr=contact_edge_attr,
            body_edge_agg_idx=body_edge_agg_idx,
            cable_edge_agg_idx=cable_edge_agg_idx,
            contact_edge_agg_idx=contact_edge_agg_idx,
            contact_close_mask=contact_edge_feats.contact_close_mask,
            node_hidden_state=node_hidden_state
        )

        return graph_feats, raw_feats


class MultiPlaneTensegrityGraphDataProcessor(FastTensegrityGraphDataProcessor):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_out_steps: int = 1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_dist_to_grnd: float = 0.5,
                 cache_batch_sizes: List | None = None,
                 num_sims=10,
                 recur_latent_dim=1024,
                 num_ctrls_hist=20,
                 rest_lens_or_ctrls='rest_lens',
                 num_node_contact_dist_feats=3):
        self.NUM_CON_DISTS = num_node_contact_dist_feats
        super().__init__(tensegrity,
                         con_edge_threshold,
                         num_out_steps,
                         num_hist,
                         dt,
                         max_dist_to_grnd,
                         cache_batch_sizes,
                         num_sims,
                         recur_latent_dim,
                         num_ctrls_hist,
                         rest_lens_or_ctrls)

    # def forward(self,
    #             batch_state: torch.Tensor,
    #             env_planar_objs: List[StaticPrism | StaticRectPlane],
    #             **kwargs: torch.Tensor):
    #     batch_size = batch_state.shape[0]
    #
    #     # Convert batch state to node_pos and prev_node_pos
    #     batch_state_ = batch_state.reshape(-1, 13, 1)
    #     batch_pos = batch_state_[:, :3]
    #     batch_quat = batch_state_[:, 3:7]
    #     batch_lin_vel = batch_state_[:, 7:10]
    #     batch_ang_vel = batch_state_[:, 10:13]
    #
    #     batch_prev_pos = batch_state_[:, :3] - self.dt * batch_lin_vel
    #     batch_prev_quat = torch_quaternion.update_quat(
    #         batch_state_[:, 3:7], -batch_ang_vel, self.dt
    #     )
    #
    #     node_pos = self.pose2node(
    #         batch_pos, batch_quat, batch_size
    #     )
    #     prev_node_pos = self.pose2node(
    #         batch_prev_pos, batch_prev_quat, batch_size
    #     )
    #
    #     # Compute node feats
    #     node_raw_feats = self._compute_node_feats(
    #         node_pos,
    #         prev_node_pos,
    #         batch_size,
    #         batch_pos,
    #         dataset_idx=kwargs['dataset_idx']
    #     )
    #
    #     node_hidden_state = zeros(
    #         (node_raw_feats.node_pos.shape[0], self.recur_latent_dim),
    #         ref_tensor=node_raw_feats.node_pos
    #     )
    #
    #     # Compute edge indices
    #     edge_vals = self._compute_edge_idxs(batch_size)
    #     body_edge_idx, body_edge_agg_idx = edge_vals[:2]
    #     cable_edge_idx, cable_edge_agg_idx, = edge_vals[2:4]
    #     contact_edge_idx, contact_edge_agg_idx = edge_vals[4:]
    #
    #     # Compute edge feats
    #     body_edge_feats, cable_edge_feats, contact_edge_feats = self._compute_edge_feats(
    #         node_raw_feats,
    #         body_edge_idx,
    #         cable_edge_idx,
    #         contact_edge_idx,
    #         batch_size,
    #         ctrls=kwargs['ctrls'],
    #     )
    #
    #     node_x, body_edge_attr, cable_edge_attr, contact_edge_attr = (
    #         self.get_normalize_feats(node_raw_feats, body_edge_feats, cable_edge_feats, contact_edge_feats)
    #     )
    #
    #     raw_feats = (node_raw_feats, body_edge_feats, cable_edge_feats, contact_edge_feats)
    #
    #     graph_feats = GraphFeats(
    #         node_x=node_x,
    #         body_edge_idx=body_edge_idx,
    #         cable_edge_idx=cable_edge_idx,
    #         contact_edge_idx=contact_edge_idx,
    #         body_edge_attr=body_edge_attr,
    #         cable_edge_attr=cable_edge_attr,
    #         contact_edge_attr=contact_edge_attr,
    #         body_edge_agg_idx=body_edge_agg_idx,
    #         cable_edge_agg_idx=cable_edge_agg_idx,
    #         contact_edge_agg_idx=contact_edge_agg_idx,
    #         contact_close_mask=contact_edge_feats.contact_close_mask,
    #         node_hidden_state=node_hidden_state
    #     )
    #
    #     return graph_feats, raw_feats