from collections import defaultdict
from typing import List, Tuple, Union, NamedTuple, Dict

import torch

from gnn_simulator.gnn_physics.normalizer import AccumulatedNormalizer, DummyNormalizer
from gnn_simulator.robots.tensegrity import TensegrityRobotGNN
from gnn_simulator.state_objects.base_state_object import BaseStateObject
from gnn_simulator.state_objects.primitive_shapes import StaticPrism, StaticRectPlane, FlatGround
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.contact_detection_utils import (
    get_dist_fn,
    sphere_static_prism_signed_distance,
    sphere_static_rect_plane_signed_distance,
    cylinder_cylinder_signed_distance,
    sphere_sphere_signed_distance,
)
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE
from gnn_simulator.utilities.tensor_utils import zeros, safe_norm

EPS = 1e-8


# ---------------------------------------------------------------------------
# Lightweight proxy objects for batched distance computation.
# These present the same attribute interface that the distance functions expect,
# allowing us to stack multiple pairs into a single batched call.
# ---------------------------------------------------------------------------

class _SphereProxy:
    """Batched sphere proxy (needs .pos, .radius)."""
    __slots__ = ('pos', 'radius')

    def __init__(self, pos, radius):
        self.pos = pos
        self.radius = radius


class _CylinderProxy:
    """Batched cylinder proxy (needs ._compute_end_pts(), .radius)."""
    __slots__ = ('_end_pts', 'radius')

    def __init__(self, end_pt1, end_pt2, radius):
        self._end_pts = [end_pt1, end_pt2]
        self.radius = radius

    def _compute_end_pts(self):
        return self._end_pts


class _PrismProxy:
    """Batched StaticPrism proxy (needs .pos, .rot_mat, .half_lens)."""
    __slots__ = ('pos', 'rot_mat', 'half_lens')

    def __init__(self, pos, rot_mat, half_lens):
        self.pos = pos
        self.rot_mat = rot_mat
        self.half_lens = half_lens


class _RectPlaneProxy:
    """Batched StaticRectPlane proxy (needs .pos, axes, .half_lens tuple)."""
    __slots__ = ('pos', 'x_axis', 'y_axis', 'z_axis', 'half_lens')

    def __init__(self, pos, x_axis, y_axis, z_axis, half_lens):
        self.pos = pos
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.half_lens = half_lens


class NodeFeats(NamedTuple):
    node_vel: torch.Tensor | None
    node_inv_mass: torch.Tensor | None
    node_inv_inertia: torch.Tensor | None
    node_dir_from_com: torch.Tensor | None
    node_dist_from_com_norm: torch.Tensor | None
    node_dist_to_ground: torch.Tensor | None
    node_contact_dist: torch.Tensor | None
    node_body_verts: torch.Tensor | None
    node_dist_to_first_node: torch.Tensor | None
    node_dist_to_first_node_norm: torch.Tensor | None
    node_prin_axis: torch.Tensor | None
    node_pos: torch.Tensor | None
    node_prev_pos: torch.Tensor | None
    node_sim_type: torch.Tensor | None
    body_mask: torch.Tensor | None


class BodyEdgeFeats(NamedTuple):
    body_dist: torch.Tensor | None
    body_dist_norm: torch.Tensor | None
    body_rest_dist: torch.Tensor | None
    body_rest_dist_norm: torch.Tensor | None


class CableEdgeFeats(NamedTuple):
    cable_dist: torch.Tensor | None
    cable_dist_norm: torch.Tensor | None
    cable_dir: torch.Tensor | None
    # cable_dl: torch.Tensor | None
    cable_rel_vel_norm: torch.Tensor | None
    cable_rest_length: torch.Tensor | None
    cable_stiffness: torch.Tensor | None
    cable_damping: torch.Tensor | None
    # cable_stiffness_force_mag: torch.Tensor | None
    # cable_damping_force_mag: torch.Tensor | None
    cable_ctrls: torch.Tensor | None
    cable_actuated_mask: torch.Tensor | None


class ContactEdgeFeats(NamedTuple):
    contact_dist: torch.Tensor | None
    contact_normal: torch.Tensor | None
    contact_tangent: torch.Tensor | None
    contact_rel_vel_normal: torch.Tensor | None
    contact_rel_vel_tangent: torch.Tensor | None
    contact_rel_z_pt: torch.Tensor | None
    contact_close_mask: torch.Tensor | None


class CacheableFeats(NamedTuple):
    node_inv_mass: torch.Tensor | None
    node_inv_inertia: torch.Tensor | None
    node_body_verts: torch.Tensor | None
    body_rest_dist: torch.Tensor | None
    body_rest_dist_norm: torch.Tensor | None
    cable_stiffness: torch.Tensor | None
    cable_damping: torch.Tensor | None
    cable_actuated_mask: torch.Tensor | None
    contact_normal: torch.Tensor | None
    body_edge_idx: torch.Tensor | None
    body_edge_agg_idx: torch.Tensor | None
    cable_edge_idx: torch.Tensor | None
    cable_edge_agg_idx: torch.Tensor | None
    contact_edge_idx: torch.Tensor | None
    contact_edge_agg_idx: torch.Tensor | None
    body_mask: torch.Tensor | None


class GraphFeats(NamedTuple):
    node_x: torch.Tensor | None
    body_edge_attr: torch.Tensor | None
    body_edge_idx: torch.Tensor | None
    body_edge_agg_idx: torch.Tensor | None
    cable_edge_attr: torch.Tensor | None
    cable_edge_idx: torch.Tensor | None
    cable_edge_agg_idx: torch.Tensor | None
    contact_edge_attr: torch.Tensor | None
    contact_edge_idx: torch.Tensor | None
    contact_edge_agg_idx: torch.Tensor | None
    contact_close_mask: torch.Tensor | None
    node_hidden_state: torch.Tensor | None


class PredGnnAttrs(NamedTuple):
    pos: torch.Tensor | None
    vel: torch.Tensor | None
    p_pos: torch.Tensor | None
    p_vel: torch.Tensor | None
    pf_dv: torch.Tensor | None
    p_dv: torch.Tensor | None
    norm_dv: torch.Tensor | None
    body_mask: torch.Tensor | None
    node_hidden_state: torch.Tensor | None


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
            self.contact_nodes_idxs_tensor = torch.as_tensor(
                self.robot.get_contact_nodes(),
                dtype=torch.long,
                device=self.device,
            )
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
            self._contact_edge_template_cache = {}
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
            node_contact_dist=None,
            node_z_rels=None,
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
            contact_rel_z_pt=None,
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


class MultiPlaneTensegrityGraphDataProcessor(BaseStateObject):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_out_steps: int = 1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_con_dist: float = 0.5,
                 cache_batch_sizes: List | None = None,
                 num_sims=1,
                 recur_latent_dim=1024,
                 num_ctrls_hist=20,
                 rest_lens_or_ctrls='rest_lens',
                 num_node_contact_dist_feats=3):
        super().__init__('fast data processor')
        """
        @param tensegrity: robot object
        @param con_edge_threshold: threshold to attach edge between ground and endcap node
        @param num_steps_ahead: how many steps training traj will be
        @param num_hist: how many steps behind to attach to features
        @param dt: timestep size
        @param max_con_dist: clip value for contact dist feature
        """
        with torch.no_grad():
            self.NUM_CON_DISTS = num_node_contact_dist_feats
            self.MAX_CON_DIST = max_con_dist
            self.CON_EDGE_THRES = con_edge_threshold
            self.NUM_OUT_STEPS = num_out_steps
            self.NUM_HIST = num_hist
            self.NUM_CTRLS_HIST = num_ctrls_hist
            self.NUM_SIMS = num_sims
            self.RECUR_LATENT_DIM = recur_latent_dim
            self.REST_LENS_OR_CTRLS = rest_lens_or_ctrls

            self.node_feat_dict = {
                'node_vel': 3,
                'node_inv_mass': 1,
                'node_inv_inertia': 3,
                'node_dist_to_ground': self.NUM_CON_DISTS,
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
                'cable_rest_length': 1 if rest_lens_or_ctrls == 'ctrls' else self.NUM_OUT_STEPS
                # 'cable_stiffness_force_mag': 1,
                # 'cable_damping_force_mag': 1,
            }
            if rest_lens_or_ctrls == 'ctrls':
                self.cable_edge_feat_dict['cable_ctrls'] = num_ctrls_hist + num_out_steps

            self.contact_edge_feat_dict = {
                'contact_dist': 1,
                'contact_normal': 3,
                'contact_tangent': 3,
                'contact_rel_vel_normal': 1,
                'contact_rel_vel_tangent': 1,
                'contact_rel_z_pt': 1
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

            if self.REST_LENS_OR_CTRLS == 'ctrls':
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

            self.body_edge_idx_template = self._body_edge_index()
            self.cable_edge_idx_template = self._get_cable_edge_idxs()

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

            num_act_cables = len(self.robot.actuated_cables) * 2
            num_nonact_cables = len(self.robot.non_actuated_cables) * 2
            self.cable_act_mask = torch.tensor(
                [True] * num_act_cables + [False] * num_nonact_cables,
                device=self.device,
            ).reshape(-1, 1)

            self._feats_batch_cache = {}
            if cache_batch_sizes is not None:
                self.precompute_and_cache_batch_sizes(cache_batch_sizes)

            # --- Precompute contact pair metadata (avoids rebuilding every call) ---
            self._hm_bodies_by_rod = []
            self._sphere_bodies_by_rod = []
            self._node_to_body = {}
            self._node_to_rod_idx = {}
            self._precompute_contact_pair_info()

    def _precompute_contact_pair_info(self):
        """Precompute body categorisation and self-collision pair list.

        Stores:
            _hm_bodies_by_rod:  list[list[(name, body, node_idx)]]
            _sphere_bodies_by_rod: list[list[(name, body, node_idx)]]
            _node_to_body:      dict  node_idx -> body object
            _node_to_rod_idx:   dict  node_idx -> rod index
            _self_collision_pairs: list[(ni, nj, body_i, body_j, rod_i, rod_j)]
            _self_collision_by_dist_fn: dict  dist_fn -> list of pair tuples
        """
        rods = list(self.robot.rods.values())
        num_rods = len(rods)
        node_mapping = self.robot.node_mapping

        for rod_idx, rod in enumerate(rods):
            hm, sp = [], []
            for name, body in rod.rigid_bodies.items():
                ni = node_mapping[name]
                self._node_to_body[ni] = body
                self._node_to_rod_idx[ni] = rod_idx
                if 'housing' in name or 'motor' in name:
                    hm.append((name, body, ni))
                elif 'sphere' in name:
                    sp.append((name, body, ni))
            self._hm_bodies_by_rod.append(hm)
            self._sphere_bodies_by_rod.append(sp)

        # Precompute self-collision pairs and group by distance function type
        self._self_collision_pairs = []
        self._self_collision_by_dist_fn = defaultdict(list)
        for rod_i in range(num_rods):
            for rod_j in range(rod_i + 1, num_rods):
                for _name_i, body_i, ni in self._hm_bodies_by_rod[rod_i]:
                    for _name_j, body_j, nj in self._hm_bodies_by_rod[rod_j]:
                        pair = (ni, nj, body_i, body_j, rod_i, rod_j)
                        self._self_collision_pairs.append(pair)
                        dist_fn = get_dist_fn(body_i, body_j)
                        self._self_collision_by_dist_fn[dist_fn].append(pair)

    def to(self, device: Union[str, torch.device]):
        super().to(device)
        self.robot.to(device)
        self.dt = self.dt.to(device)
        self.sphere_radius = self.sphere_radius.to(device)

        self.body_edge_idx_template = self.body_edge_idx_template.to(device)
        self.cable_edge_idx_template = self.cable_edge_idx_template.to(device)

        self.robot_inv_mass = self.robot_inv_mass.to(device)
        self.robot_inv_inertia = self.robot_inv_inertia.to(device)
        self.robot_cable_stiffness = self.robot_cable_stiffness.to(device)
        self.robot_cable_damping = self.robot_cable_damping.to(device)

        self.body_verts = self.body_verts.to(device)
        self.body_rest_dists = self.body_rest_dists.to(device)
        self.body_rest_dists_norm = self.body_rest_dists_norm.to(device)

        for normalizer in self.normalizers.values():
            normalizer.to(device)

        for k, cache in self._feats_batch_cache.items():
            tmp_dict = cache._asdict()
            for kk, v in tmp_dict.items():
                if isinstance(v, torch.Tensor):
                    tmp_dict[kk] = v.to(device)
            self._feats_batch_cache[k] = CacheableFeats(**tmp_dict)
        self._contact_edge_template_cache = {}

        contact_nodes_idxs_tensor = getattr(self, 'contact_nodes_idxs_tensor', None)
        if contact_nodes_idxs_tensor is None:
            self.contact_nodes_idxs_tensor = torch.as_tensor(
                self.robot.get_contact_nodes(),
                dtype=torch.long,
                device=device,
            )
        else:
            self.contact_nodes_idxs_tensor = contact_nodes_idxs_tensor.to(device)

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

    def _batch_feats(self, bsize: int):
        robot_inv_mass = self.robot_inv_mass.repeat(bsize, 1)
        robot_inv_inertia = self.robot_inv_inertia.repeat(bsize, 1)
        robot_cable_stiffness = self.robot_cable_stiffness.repeat(bsize, 1)
        robot_cable_damping = self.robot_cable_damping.repeat(bsize, 1)

        body_verts = self.body_verts.repeat(bsize, 1)
        body_rest_dists = self.body_rest_dists.repeat(bsize, 1)
        body_rest_dists_norm = self.body_rest_dists_norm.repeat(bsize, 1)

        cable_act_mask = self.cable_act_mask.repeat(bsize, 1)

        return CacheableFeats(
            node_inv_mass=robot_inv_mass,
            node_inv_inertia=robot_inv_inertia,
            node_body_verts=body_verts,
            body_rest_dist=body_rest_dists,
            body_rest_dist_norm=body_rest_dists_norm,
            contact_normal=None,
            cable_stiffness=robot_cable_stiffness,
            cable_damping=robot_cable_damping,
            cable_actuated_mask=cable_act_mask,
            body_edge_idx=None,
            body_edge_agg_idx=None,
            cable_edge_idx=None,
            cable_edge_agg_idx=None,
            contact_edge_idx=None,
            contact_edge_agg_idx=None,
            body_mask=None,
        )

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

    def pose2node(self,
                  pos: torch.Tensor,
                  quat: torch.Tensor,
                  batch_size: int
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

    def _inject_env_obj_feat(self, feat, env_obj_val_tensor):
        num_nodes = self.robot.num_nodes
        hsize = feat.shape[1]

        feat = feat.reshape(-1, num_nodes * hsize)
        env_obj_val_tensor = env_obj_val_tensor.repeat(feat.shape[0], 1)
        feat_w_env_obj = torch.hstack([feat, env_obj_val_tensor]).reshape(-1, hsize)

        return feat_w_env_obj

    def _compute_node_feats(self,
                            node_pos: torch.Tensor,
                            prev_node_pos: torch.Tensor,
                            batch_size: int,
                            batch_pos: torch.Tensor,
                            num_env_objs: int,
                            dataset_idx: torch.Tensor | int
                            ) -> Dict:
        """
        Method to compute all node feats based on curr and prev node poses

        @param node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param prev_node_pos: (batch_size * num nodes per graph, 3 * num_hist)
        @param batch_size: size of batch
        @return: Dictionary of feat tensors
        """

        # Pre adding ground node
        com_pos = batch_pos.repeat(1, self.robot.num_nodes_per_rod, 1).reshape(-1, 3)
        dist_from_com = node_pos - com_pos
        dist_from_com_norm = dist_from_com.norm(dim=1, keepdim=True)
        dir_from_com = safe_norm(dist_from_com)

        node_vels = (node_pos - prev_node_pos) / self.dt.squeeze(-1)

        dist_first_node, dist_first_node_norm = self._compute_shape_feats(node_pos, batch_size)
        node_prin = self._compute_prin_feat(node_pos)

        # Post ground node - create env object padding tensors
        # env_obj_ten3d: for 3D features (3 values per env object)
        # env_obj_ten1d: for 1D features (1 value per env object)
        env_obj_ten3d = zeros((1, 3 * num_env_objs), ref_tensor=node_pos)
        env_obj_ten1d = zeros((1, num_env_objs), ref_tensor=node_pos)
        env_obj_ten_prin = torch.tensor([[0., 0., 1.] * num_env_objs], dtype=self.dtype, device=self.device)

        node_pos = self._inject_env_obj_feat(node_pos, env_obj_ten3d)
        prev_node_pos = self._inject_env_obj_feat(prev_node_pos, env_obj_ten3d)
        node_vels = self._inject_env_obj_feat(node_vels, env_obj_ten3d)
        dist_from_com_norm = self._inject_env_obj_feat(dist_from_com_norm, env_obj_ten1d)
        dir_from_com = self._inject_env_obj_feat(dir_from_com, env_obj_ten3d)
        dist_first_node = self._inject_env_obj_feat(dist_first_node, env_obj_ten3d)
        dist_first_node_norm = self._inject_env_obj_feat(dist_first_node_norm, env_obj_ten1d)
        node_prin = self._inject_env_obj_feat(node_prin, env_obj_ten_prin)

        # Need to cache batch size before data processor call
        # Cached features already have (num_robot_nodes + 1) nodes (1 ground node added during init)
        # We need (num_robot_nodes + num_env_objs) nodes total
        # So we need to add (num_env_objs - 1) more env object nodes
        extra_env_objs = num_env_objs - 1
        num_cached_nodes = self.robot.num_nodes + 1  # Cache includes 1 ground node

        body_verts = self._feats_batch_cache[batch_size].node_body_verts
        inv_mass = self._feats_batch_cache[batch_size].node_inv_mass
        inv_inertia = self._feats_batch_cache[batch_size].node_inv_inertia

        if extra_env_objs > 0:
            # Pad cached features with zeros for extra env objects
            # For 3D features: reshape to (batch, nodes*3), pad, reshape back
            body_verts = body_verts.reshape(batch_size, -1)
            body_verts = torch.hstack([body_verts, zeros((batch_size, 3 * extra_env_objs), ref_tensor=body_verts)])
            body_verts = body_verts.reshape(-1, 3)

            inv_inertia = inv_inertia.reshape(batch_size, -1)
            inv_inertia = torch.hstack([inv_inertia, zeros((batch_size, 3 * extra_env_objs), ref_tensor=inv_inertia)])
            inv_inertia = inv_inertia.reshape(-1, 3)

            # For 1D features: reshape to (batch, nodes), pad, reshape back
            inv_mass = inv_mass.reshape(batch_size, -1)
            inv_mass = torch.hstack([inv_mass, zeros((batch_size, extra_env_objs), ref_tensor=inv_mass)])
            inv_mass = inv_mass.reshape(-1, 1)

        body_mask = torch.hstack([
            torch.ones((batch_size, self.robot.num_nodes), dtype=torch.bool, device=self.device),
            torch.zeros((batch_size, num_env_objs), dtype=torch.bool, device=self.device),
        ]).reshape(-1, 1)

        dataset_idx = torch.as_tensor(dataset_idx, dtype=torch.long, device=self.device).reshape(1, -1)

        node_feats = dict(
            node_vel=node_vels,
            node_inv_mass=inv_mass,
            node_inv_inertia=inv_inertia,
            node_dir_from_com=dir_from_com,
            node_dist_from_com_norm=dist_from_com_norm,
            node_body_verts=body_verts,
            node_dist_to_first_node=dist_first_node,
            node_dist_to_first_node_norm=dist_first_node_norm,
            node_pos=node_pos,
            node_prev_pos=prev_node_pos,
            node_prin_axis=node_prin,
            node_sim_type=self._one_hot_encode(dataset_idx, num_env_objs),
            body_mask=body_mask
        )

        return node_feats

    def _one_hot_encode(self, batch_idxs, num_env_objs=1):
        num_nodes = self.robot.num_nodes + num_env_objs
        batch_idxs = batch_idxs.repeat(1, num_nodes).reshape(-1, 1)
        vecs = torch.zeros(
            (batch_idxs.shape[0], self.NUM_SIMS),
            dtype=self.dtype,
            device=self.device
        )
        vecs[torch.arange(batch_idxs.shape[0], dtype=torch.long), batch_idxs.flatten()] = 1.

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

    def _get_contact_edge_index(self,
                                num_env_objs: int
                                ) -> torch.Tensor:
        """
        Edge index ordering:
        - Self collisions
            -(rod0_body_0, rod1_body_0)
            -(rod1_body_0, rod0_body_0)
            -(rod0_body_0, rod1_body_1)
            -(rod1_body_1, rod0_body_0)
            -...
        - Robot-environment object collisions
            -(body_0, env_obj_0)
            -(body_1, env_obj_0)
            ...
            -(env_m, body_{n-1})
            -(env_m, body_n)
        """
        contact_nodes_idxs = getattr(self, 'contact_nodes_idxs_tensor', None)
        if contact_nodes_idxs is None:
            contact_nodes_idxs = torch.as_tensor(
                self.robot.get_contact_nodes(),
                dtype=torch.long,
                device=self.device,
            )
            self.contact_nodes_idxs_tensor = contact_nodes_idxs

        start_idx = self.robot.num_nodes
        env_obj_idxs = torch.arange(
            start_idx,
            start_idx + num_env_objs,
            dtype=torch.long,
            device=self.device,
        )

        env_obj_idxs = env_obj_idxs.repeat_interleave(contact_nodes_idxs.shape[0], dim=0).reshape(1, -1)
        contact_nodes_idxs = contact_nodes_idxs.repeat(1, num_env_objs).reshape(1, -1)

        env_con_edge_idx = torch.vstack([
            torch.hstack([contact_nodes_idxs, env_obj_idxs]),
            torch.hstack([env_obj_idxs, contact_nodes_idxs])
        ]).detach()

        if self.NUM_CON_DISTS == 1:
            contact_edge_idx = env_con_edge_idx
        else:
            self_collision_edge_idx = self.robot.self_collision_template_idx.clone()
            contact_edge_idx = torch.hstack([self_collision_edge_idx, env_con_edge_idx])

        return contact_edge_idx

    def _get_cached_contact_template(self, num_env_objs: int):
        cache = getattr(self, '_contact_edge_template_cache', None)
        if cache is None:
            cache = {}
            self._contact_edge_template_cache = cache

        cached = cache.get(num_env_objs)
        if cached is not None:
            return cached

        template = self._get_contact_edge_index(num_env_objs)
        template_pairs = list(zip(template[0].tolist(), template[1].tolist()))
        cached = (template, template_pairs)
        cache[num_env_objs] = cached
        return cached

    def _compute_edge_idxs(self, batch_size, num_env_objs) \
            -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, List[Tuple[int, int]]]:
        """
        Method to compute different edge type indices
        """
        num_nodes = self.robot.num_nodes + num_env_objs
        # Need to cache batch size before data processor call
        body_edge_idx = self.batch_edge_index(self.body_edge_idx_template, batch_size, num_nodes)
        cable_edge_idx = self.batch_edge_index(self.cable_edge_idx_template, batch_size, num_nodes)
        contact_template, template_pairs = self._get_cached_contact_template(num_env_objs)
        contact_edge_idx = self.batch_edge_index(contact_template, batch_size, num_nodes)

        return body_edge_idx, cable_edge_idx, contact_edge_idx, template_pairs

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

        if self.REST_LENS_OR_CTRLS == 'ctrls':
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

        if self.REST_LENS_OR_CTRLS == 'ctrls':
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

    def _compute_non_con_edge_feats(self,
                                    node_feats,
                                    body_edge_idx,
                                    cable_edge_idx,
                                    batch_size,
                                    ctrls):
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
            node_feats['node_pos'],
            batch_size
        )

        # cable edges
        cable_edge_feats = self._compute_cable_edge_feats(
            cable_edge_idx,
            node_feats['node_pos'],
            node_feats['node_vel'],
            batch_size,
            ctrls=ctrls,
        )

        return body_edge_feats, cable_edge_feats

    def _get_body_mask(self, batch_size, num_env_objs, device):
        body_mask = torch.tensor(
            [True] * self.robot.num_nodes + [False] * num_env_objs,
            dtype=torch.bool,
            device=device
        ).repeat(batch_size, 1).reshape(-1, 1)
        return body_mask

    def _compute_pt_vel(self, graph_obj, pt, mode='average', mask=None):
        """
        Compute point velocity

        Args:
            graph_obj: Rod object with pos, linear_vel, ang_vel, etc.
            pt: Point position tensor (B, 3, 1) or (B_subset, 3, 1) if mask is used
            mode: 'instantaneous' or 'average'
            mask: Optional boolean mask (B,) to subset rod state
        """
        # Subset rod state if mask is provided
        if mask is not None:
            rod_pos = graph_obj.pos[mask]
            rod_linear_vel = graph_obj.linear_vel[mask]
            rod_ang_vel = graph_obj.ang_vel[mask]
            if mode == 'average':
                rod_quat = graph_obj.quat[mask]
        else:
            rod_pos = graph_obj.pos
            rod_linear_vel = graph_obj.linear_vel
            rod_ang_vel = graph_obj.ang_vel
            if mode == 'average':
                rod_quat = graph_obj.quat

        if mode == 'instantaneous':
            return rod_linear_vel + torch.cross(rod_ang_vel, pt - rod_pos, dim=1)
        elif mode == 'average':
            rel_pt = torch_quaternion.inv_rot_vec_quat(rod_quat, pt - rod_pos)
            prev_rod_pos = rod_pos - rod_linear_vel * self.dt
            prev_rod_quat = torch_quaternion.update_quat(rod_quat, -rod_ang_vel, self.dt)
            new_pt = prev_rod_pos + torch_quaternion.rotate_vec_quat(prev_rod_quat, rel_pt)
            return (new_pt - pt) / self.dt
        else:
            raise ValueError(f"Invalid mode: {mode}")

    def _compute_contact_related_feats(
            self,
            env_planar_objs: List[Union[StaticPrism, StaticRectPlane]],
            contact_edge_idx: torch.Tensor,
            batch_size: int,
            template_pairs: List[Tuple[int, int]],
    ):
        """
        Compute contact-related features for robot self-collision and
        robot-environment collision.

        Uses batched distance function calls (one per geometry-type combination)
        instead of per-pair Python loops.
        """
        rods = list(self.robot.rods.values())
        num_rods = len(rods)
        graph_objs = rods + env_planar_objs

        # Trigger lazy inner-body state update for all rods.
        # rod.rigid_bodies is a property that propagates the rod's current
        # (batched) state to each inner body only on first access after
        # update_state().  The precomputed body references stored in
        # _self_collision_pairs / _sphere_bodies_by_rod are the *same*
        # Python objects, so this call brings them up to date.
        for rod in rods:
            _ = rod.rigid_bodies

        # Use precomputed body/node info; add env nodes for this call
        node_to_body = dict(self._node_to_body)
        node_to_graph_obj_idx = dict(self._node_to_rod_idx)
        for k, env_obj in enumerate(env_planar_objs):
            node_to_body[k + self.robot.num_nodes] = env_obj
            node_to_graph_obj_idx[k + self.robot.num_nodes] = k + num_rods

        device = self.device
        dtype = self.dtype

        num_graph_nodes = self.robot.num_nodes + len(env_planar_objs)
        min_dists = torch.full(
            (batch_size, num_graph_nodes, self.NUM_CON_DISTS),
            self.MAX_CON_DIST,
            device=device,
            dtype=dtype,
        )

        pair_data = {}   # (ni, nj) -> (pt_i, pt_j, sd, z_rel_i, z_rel_j)
        edge_mask_dict = {}

        # =============================================================
        # PART 1: Self-collision distances — BATCHED by dist-fn type
        # =============================================================
        if self.NUM_CON_DISTS > 1 and self._self_collision_pairs:
            min_dists = self._batched_self_collision(
                batch_size, pair_data, min_dists)

            # Build edge masks for self-collision pairs
            for (ni, nj) in pair_data.keys():
                sd = pair_data[(ni, nj)][2]
                mask = sd < self.CON_EDGE_THRES
                edge_mask_dict[(ni, nj)] = mask
                edge_mask_dict[(nj, ni)] = mask

        # =============================================================
        # PART 2: Sphere-env collision — BATCHED by env type
        # =============================================================
        if len(env_planar_objs) > 0:
            self._batched_sphere_env_collision(
                env_planar_objs, batch_size,
                pair_data, edge_mask_dict, min_dists)

        # =============================================================
        # PART 3: Contact edge features (per-pair, only for close pairs)
        # =============================================================
        contact_edge_feats = {}
        for (ni, nj), (pt_i, pt_j, sd, z_rel_i, z_rel_j, normal_i, normal_j) in pair_data.items():
            edge_mask = edge_mask_dict[(ni, nj)].flatten()  # (B,)

            # Zero-initialised dummy features for both directions
            zeros_sd = torch.zeros_like(sd)
            zeros_pt = torch.zeros_like(pt_i[..., 0])
            contact_edge_feats[(ni, nj)] = {
                'contact_dist': zeros_sd.clone(),
                'contact_normal': zeros_pt.clone(),
                'contact_tangent': zeros_pt.clone(),
                'contact_rel_vel_normal': zeros_sd.clone(),
                'contact_rel_vel_tangent': zeros_sd.clone(),
                'contact_rel_z_pt': zeros_sd.clone(),
            }
            contact_edge_feats[(nj, ni)] = {
                k: v.clone() for k, v in contact_edge_feats[(ni, nj)].items()
            }

            vel_i = self._compute_pt_vel(
                graph_objs[node_to_graph_obj_idx[ni]], pt_i
            )  # full batch
            vel_j = self._compute_pt_vel(
                graph_objs[node_to_graph_obj_idx[nj]], pt_j
            )  # full batch
            rel_vel = vel_j - vel_i

            contact_edge_feats[(nj, ni)] = self._compute_single_con_edge_feats(
                contact_edge_feats[(nj, ni)], edge_mask,
                normal_i, rel_vel, sd, z_rel_i
            )
            contact_edge_feats[(ni, nj)] = self._compute_single_con_edge_feats(
                contact_edge_feats[(ni, nj)], edge_mask,
                normal_j, -rel_vel, sd, z_rel_j
            )

        node_min_dists, contact_edge_feats, filtered_contact_edge_idx = (
            self._assemble_contact_feats(
                contact_edge_idx, batch_size, len(env_planar_objs),
                min_dists, edge_mask_dict, contact_edge_feats, template_pairs))

        node_min_dists = torch.clamp_max(node_min_dists, self.MAX_CON_DIST)

        return node_min_dists, contact_edge_feats, filtered_contact_edge_idx

    # -----------------------------------------------------------------
    # Batched self-collision helper
    # -----------------------------------------------------------------
    def _batched_self_collision(self, batch_size, pair_data, min_dists):
        """Compute all self-collision distances with one call per geometry type."""
        B = batch_size
        device = min_dists.device
        dtype = min_dists.dtype

        # Collect all (node_idx, sd) pairs to avoid in-place modifications
        all_node_indices = []  # List of node indices
        all_sd_values = []     # List of sd tensors (B, 1)

        for dist_fn, pairs in self._self_collision_by_dist_fn.items():
            P = len(pairs)
            if P == 0:
                continue

            # --- Precompute per-body state once (avoid redundant _compute_end_pts) ---
            body_cache = {}
            for _, _, b_i, b_j, _, _ in pairs:
                for b in (b_i, b_j):
                    bid = id(b)
                    if bid not in body_cache:
                        body_cache[bid] = {
                            'pos': b.pos,
                            'prin_axis': b.get_principal_axis(),
                        }
                        if hasattr(b, '_compute_end_pts'):
                            ep = b._compute_end_pts()
                            body_cache[bid]['end_pt0'] = ep[0]
                            body_cache[bid]['end_pt1'] = ep[1]

            # --- Build batched proxy inputs ---
            if dist_fn is cylinder_cylinder_signed_distance:
                proxy1 = _CylinderProxy(
                    torch.cat([body_cache[id(b_i)]['end_pt0'] for _, _, b_i, _, _, _ in pairs]),
                    torch.cat([body_cache[id(b_i)]['end_pt1'] for _, _, b_i, _, _, _ in pairs]),
                    torch.cat([b_i.radius.expand(B, -1, -1) for _, _, b_i, _, _, _ in pairs]),
                )
                proxy2 = _CylinderProxy(
                    torch.cat([body_cache[id(b_j)]['end_pt0'] for _, _, _, b_j, _, _ in pairs]),
                    torch.cat([body_cache[id(b_j)]['end_pt1'] for _, _, _, b_j, _, _ in pairs]),
                    torch.cat([b_j.radius.expand(B, -1, -1) for _, _, _, b_j, _, _ in pairs]),
                )
                pt_i_all, pt_j_all, sd_all, normal_i_all, normal_j_all = dist_fn(proxy1, proxy2)

            elif dist_fn is sphere_sphere_signed_distance:
                proxy1 = _SphereProxy(
                    torch.cat([b_i.pos for _, _, b_i, _, _, _ in pairs]),
                    torch.cat([b_i.radius.expand(B, -1, -1) for _, _, b_i, _, _, _ in pairs]),
                )
                proxy2 = _SphereProxy(
                    torch.cat([b_j.pos for _, _, _, b_j, _, _ in pairs]),
                    torch.cat([b_j.radius.expand(B, -1, -1) for _, _, _, b_j, _, _ in pairs]),
                )
                pt_i_all, pt_j_all, sd_all, normal_i_all, normal_j_all = dist_fn(proxy1, proxy2)

            else:
                # Fallback: per-pair (unknown geometry combination)
                for ni, nj, b_i, b_j, rod_i, rod_j in pairs:
                    pt_i, pt_j, sd, normal_i, normal_j = dist_fn(b_i, b_j)
                    z_rel_i = ((pt_i - b_i.pos) * b_i.get_principal_axis()).sum(dim=1)
                    z_rel_j = ((pt_j - b_j.pos) * b_j.get_principal_axis()).sum(dim=1)
                    pair_data[(ni, nj)] = (pt_i, pt_j, sd, z_rel_i, z_rel_j, normal_i, normal_j)
                    # Collect for later scatter_reduce (avoid in-place modification)
                    all_node_indices.extend([ni, nj])
                    all_sd_values.extend([sd, sd])
                continue

            # --- Unpack batched results -> per-pair ---
            pt_i_all = pt_i_all.reshape(P, B, 3, 1)
            pt_j_all = pt_j_all.reshape(P, B, 3, 1)
            sd_all = sd_all.reshape(P, B, 1)
            normal_i_all = normal_i_all.reshape(P, B, 3, 1)
            normal_j_all = normal_j_all.reshape(P, B, 3, 1)

            # Gather per-body principal axis & pos (cached)
            for pidx, (ni, nj, b_i, b_j, rod_i, rod_j) in enumerate(pairs):
                pt_i = pt_i_all[pidx]
                pt_j = pt_j_all[pidx]
                sd = sd_all[pidx]
                normal_i = normal_i_all[pidx]
                normal_j = normal_j_all[pidx]

                bc_i = body_cache[id(b_i)]
                bc_j = body_cache[id(b_j)]
                z_rel_i = ((pt_i - bc_i['pos']) * bc_i['prin_axis']).sum(dim=1)
                z_rel_j = ((pt_j - bc_j['pos']) * bc_j['prin_axis']).sum(dim=1)

                pair_data[(ni, nj)] = (pt_i, pt_j, sd, z_rel_i, z_rel_j, normal_i, normal_j)
                # Collect for later scatter_reduce (avoid in-place modification)
                all_node_indices.extend([ni, nj])
                all_sd_values.extend([sd, sd])

        # Apply all minimum updates at once using scatter_reduce (avoids in-place ops)
        if all_node_indices:
            # Stack all sd values: each sd is (B, 1), stack to (N, B, 1) then squeeze and transpose to (B, N)
            all_sd_stacked = torch.stack(all_sd_values, dim=0).squeeze(-1).T  # (B, N)
            # Create index tensor for scatter: (N,) -> expand to (B, N)
            node_idx_tensor = torch.tensor(all_node_indices, device=device, dtype=torch.long)
            node_idx_expanded = node_idx_tensor.unsqueeze(0).expand(B, -1)  # (B, N)
            
            # Get current min_dists last column and scatter_reduce with amin
            current_mins = min_dists[:, :, -1].clone()  # (B, num_nodes)
            # Use scatter_reduce to compute element-wise minimum
            updated_mins = current_mins.scatter_reduce(
                dim=1, 
                index=node_idx_expanded, 
                src=all_sd_stacked,
                reduce='amin',
                include_self=True
            )
            # Create new min_dists tensor (out-of-place)
            min_dists = torch.cat([min_dists[:, :, :-1], updated_mins.unsqueeze(-1)], dim=-1)

        return min_dists

    # -----------------------------------------------------------------
    # Batched sphere-env collision helper
    # -----------------------------------------------------------------
    def _batched_sphere_env_collision(
            self, env_planar_objs, batch_size,
            pair_data, edge_mask_dict, min_dists):
        """Compute sphere ↔ env distances with one call per env-geometry type."""
        B = batch_size
        num_rods = len(self.robot.rods)
        num_env = len(env_planar_objs)
        num_env_dists = max(self.NUM_CON_DISTS - 1, 1)
        device = self.device
        dtype = self.dtype

        # --- Collect all (sphere, env_obj) pairs grouped by env type ---
        prism_pairs = []   # [(ni_s, env_idx, body_s, prism, rod_idx)]
        plane_pairs = []   # [(ni_s, env_idx, body_s, plane, rod_idx)]

        for rod_idx in range(num_rods):
            for _name_s, body_s, ni_s in self._sphere_bodies_by_rod[rod_idx]:
                for k, env_obj in enumerate(env_planar_objs):
                    env_idx = k + self.robot.num_nodes
                    if isinstance(env_obj, StaticPrism):
                        prism_pairs.append((ni_s, env_idx, body_s, env_obj, rod_idx))
                    elif isinstance(env_obj, StaticRectPlane):
                        plane_pairs.append((ni_s, env_idx, body_s, env_obj, rod_idx))

        # results keyed by (ni_s, env_idx) -> (pt_s, pt_e, sd, normal_s, normal_e)
        env_results = {}

        # --- Batched sphere-prism ---
        if prism_pairs:
            P = len(prism_pairs)
            s_proxy = _SphereProxy(
                torch.cat([b.pos for _, _, b, _, _ in prism_pairs]),
                prism_pairs[0][2].radius,  # all spheres share radius
            )
            p_proxy = _PrismProxy(
                torch.cat([p.pos for _, _, _, p, _ in prism_pairs]),
                torch.cat([p.rot_mat for _, _, _, p, _ in prism_pairs]),
                torch.cat([p.half_lens.expand(B, -1, -1) for _, _, _, p, _ in prism_pairs]),
            )
            pt_s_all, pt_e_all, sd_all, normal_s_all, normal_e_all = sphere_static_prism_signed_distance(
                s_proxy, p_proxy)

            for idx, (ni_s, env_idx, _, _, _) in enumerate(prism_pairs):
                s, e = idx * B, (idx + 1) * B
                env_results[(ni_s, env_idx)] = (
                    pt_s_all[s:e], pt_e_all[s:e], sd_all[s:e],
                    normal_s_all[s:e], normal_e_all[s:e])

        # --- Batched sphere-plane ---
        if plane_pairs:
            P = len(plane_pairs)
            s_proxy = _SphereProxy(
                torch.cat([b.pos for _, _, b, _, _ in plane_pairs]),
                plane_pairs[0][2].radius,
            )
            p_proxy = _RectPlaneProxy(
                torch.cat([p.pos for _, _, _, p, _ in plane_pairs]),
                torch.cat([p.x_axis.expand(B, -1, -1) for _, _, _, p, _ in plane_pairs]),
                torch.cat([p.y_axis.expand(B, -1, -1) for _, _, _, p, _ in plane_pairs]),
                torch.cat([p.z_axis.expand(B, -1, -1) for _, _, _, p, _ in plane_pairs]),
                (
                    torch.cat([(p.half_lens[0].reshape(1, 1, 1)
                                if p.half_lens[0].ndim == 0
                                else p.half_lens[0]).expand(B, 1, 1)
                               for _, _, _, p, _ in plane_pairs]),
                    torch.cat([(p.half_lens[1].reshape(1, 1, 1)
                                if p.half_lens[1].ndim == 0
                                else p.half_lens[1]).expand(B, 1, 1)
                               for _, _, _, p, _ in plane_pairs]),
                ),
            )
            pt_s_all, pt_e_all, sd_all, normal_s_all, normal_e_all = (
                sphere_static_rect_plane_signed_distance(s_proxy, p_proxy))

            for idx, (ni_s, env_idx, _, _, _) in enumerate(plane_pairs):
                s, e = idx * B, (idx + 1) * B
                env_results[(ni_s, env_idx)] = (
                    pt_s_all[s:e], pt_e_all[s:e], sd_all[s:e],
                    normal_s_all[s:e], normal_e_all[s:e])

        # --- Post-process: z_rel, edge masks, min_dists per sphere ---
        for rod_idx in range(num_rods):
            for _name_s, body_s, ni_s in self._sphere_bodies_by_rod[rod_idx]:
                z_axis_s = body_s.rot_mat[:, :, 2:]  # (B,3,1)
                all_dists = []

                for k, env_obj in enumerate(env_planar_objs):
                    env_idx = k + self.robot.num_nodes
                    pt_s, pt_e, sd, normal_s, normal_e = env_results[(ni_s, env_idx)]

                    z_rel = ((pt_s - body_s.pos) * z_axis_s).sum(dim=1)
                    dummy_z_rel = torch.zeros_like(z_rel)

                    all_dists.append(sd)

                    edge_mask = sd < self.CON_EDGE_THRES

                    pair_data[(ni_s, env_idx)] = (pt_s, pt_e, sd, z_rel, dummy_z_rel, normal_s, normal_e)
                    edge_mask_dict[(ni_s, env_idx)] = edge_mask
                    edge_mask_dict[(env_idx, ni_s)] = edge_mask

                # Find bottom-k distances
                stacked_dists = torch.cat(all_dists, dim=1)  # (B, E)
                topk = min(num_env_dists, num_env)
                vals, _ = torch.topk(stacked_dists, topk, dim=1, largest=False)

                if topk < num_env_dists:
                    pad_d = torch.full(
                        (B, num_env_dists - topk),
                        self.MAX_CON_DIST, device=device, dtype=dtype)
                    min_dists[:, ni_s, :-1] = torch.hstack([vals, pad_d])
                else:
                    end_col = -1 if self.NUM_CON_DISTS > 1 else min_dists.shape[2]
                    min_dists[:, ni_s, :end_col] = vals

    def _compute_single_con_edge_feats(self, contact_edge_feats_ij, edge_mask, normal, rel_vel, sd, z_rel):
        """Compute contact edge features on the full batch, masking with torch.where.

        All inputs are full-batch tensors (B, ...).  ``edge_mask`` is (B,).
        Only entries where ``edge_mask`` is True are written; the rest keep
        their zero-initialised defaults.
        """
        rv_n_i = (rel_vel * normal).sum(dim=1, keepdim=True)  # (B,1,1)
        tang_vel_i = rel_vel - rv_n_i * normal  # (B,3,1)
        tang_mag_i = tang_vel_i.norm(dim=1, keepdim=True)  # (B,1,1)
        tang_mag_c_i = torch.clamp_min(tang_mag_i, EPS)
        tangent_i = tang_vel_i / tang_mag_c_i  # (B,3,1)

        m = edge_mask.unsqueeze(-1)  # (B, 1) – broadcasts over feature dims
        contact_edge_feats_ij['contact_dist'] = torch.where(m, sd, contact_edge_feats_ij['contact_dist'])
        contact_edge_feats_ij['contact_normal'] = torch.where(m, normal.squeeze(-1), contact_edge_feats_ij['contact_normal'])
        contact_edge_feats_ij['contact_tangent'] = torch.where(m, tangent_i.squeeze(-1), contact_edge_feats_ij['contact_tangent'])
        contact_edge_feats_ij['contact_rel_vel_normal'] = torch.where(m, rv_n_i.squeeze(-1), contact_edge_feats_ij['contact_rel_vel_normal'])
        contact_edge_feats_ij['contact_rel_vel_tangent'] = torch.where(m, tang_mag_i.squeeze(-1), contact_edge_feats_ij['contact_rel_vel_tangent'])
        contact_edge_feats_ij['contact_rel_z_pt'] = torch.where(m, z_rel, contact_edge_feats_ij['contact_rel_z_pt'])

        return contact_edge_feats_ij

    def _assemble_contact_feats(
            self,
            contact_edge_idx: torch.Tensor,
            batch_size: int,
            num_env_objs: int,
            min_dists: torch.Tensor,
            edge_mask_dict: dict,
            contact_edge_feats_dict: dict,
            template_pairs: List[Tuple[int, int]],
    ):
        """
        Assemble outputs of _compute_contact_related_feats into tensors
        matching the ordering of contact_edge_idx and node features.
        """
        device = contact_edge_idx.device
        dtype = self.dtype

        # Filter contact_edge_idx by edge masks
        edge_mask = torch.cat([
            edge_mask_dict[(ni, nj)] for ni, nj in template_pairs
        ]).flatten()
        filtered_contact_edge_idx = contact_edge_idx[:, edge_mask]

        # Stack contact edge features matching contact_edge_idx order.
        # Use torch.cat (single kernel) instead of hstack in a loop.
        final = {}
        for name, ndim in self.contact_edge_feat_dict.items():
            stacked = torch.cat([
                contact_edge_feats_dict[(ni, nj)][name]
                for ni, nj in template_pairs
            ]).reshape(-1, ndim)
            final[name] = stacked[edge_mask]

        contact_edge_feats = ContactEdgeFeats(**final, contact_close_mask=None)

        # Flatten per-node min_dists in node order.
        num_robot_nodes = self.robot.num_nodes
        num_graph_nodes = num_robot_nodes + num_env_objs

        node_min_dists = min_dists[:, :num_graph_nodes, :].reshape(-1, self.NUM_CON_DISTS)

        return node_min_dists, contact_edge_feats, filtered_contact_edge_idx

    def forward(self,
                batch_state: torch.Tensor,
                env_objs: List[Union[StaticPrism, StaticRectPlane]],
                dataset_idx: torch.Tensor | int,
                ctrls: torch.Tensor):
        batch_size = batch_state.shape[0]
        for env_obj in env_objs:
            if env_obj.pos.shape[0] != batch_size:
                env_obj.repeat_state(batch_size)

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
            len(env_objs),
            dataset_idx=dataset_idx,
        )

        node_hidden_state = zeros(
            (node_raw_feats['node_pos'].shape[0], self.RECUR_LATENT_DIM),
            ref_tensor=node_raw_feats['node_pos']
        )

        # Compute edge indices
        body_edge_idx, cable_edge_idx, contact_edge_idx, template_pairs = (
            self._compute_edge_idxs(batch_size, len(env_objs))
        )

        # Compute edge feats
        body_edge_feats, cable_edge_feats = self._compute_non_con_edge_feats(
            node_raw_feats,
            body_edge_idx,
            cable_edge_idx,
            batch_size,
            ctrls=ctrls,
        )

        node_min_dists, contact_edge_feats, contact_edge_idx = self._compute_contact_related_feats(
            env_objs,
            contact_edge_idx,
            batch_size,
            template_pairs,
        )

        node_raw_feats = NodeFeats(
            node_contact_dist=node_min_dists,
            node_dist_to_ground=node_min_dists,
            **node_raw_feats
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
            body_edge_agg_idx=None,
            cable_edge_agg_idx=None,
            contact_edge_agg_idx=None,
            contact_close_mask=contact_edge_feats.contact_close_mask,
            node_hidden_state=node_hidden_state
        )

        return graph_feats, raw_feats


class GroundOnlyTensegrityGraphDataProcessor(MultiPlaneTensegrityGraphDataProcessor):

    def __init__(self,
                 tensegrity: TensegrityRobotGNN,
                 con_edge_threshold: float = 2e-1,
                 num_out_steps: int = 1,
                 num_hist: int = 1,
                 dt: float = 0.01,
                 max_con_dist: float = 0.5,
                 cache_batch_sizes: List | None = None,
                 num_sims=1,
                 recur_latent_dim=1024,
                 num_ctrls_hist=20,
                 rest_lens_or_ctrls='rest_lens',
                 use_self_collision_feats=True):
        super().__init__(
            tensegrity,
            con_edge_threshold,
            num_out_steps,
            num_hist,
            dt,
            max_con_dist,
            cache_batch_sizes,
            num_sims,
            recur_latent_dim,
            num_ctrls_hist,
            rest_lens_or_ctrls,
            num_node_contact_dist_feats=(1 + use_self_collision_feats)
        )
        self.ground_plane = FlatGround()

    def to(self, device):
        super().to(device)
        self.ground_plane = self.ground_plane.to(device)

        return self

    def forward(self,
                batch_state: torch.Tensor,
                **kwargs: torch.Tensor):
        return super().forward(batch_state, [self.ground_plane], **kwargs)
