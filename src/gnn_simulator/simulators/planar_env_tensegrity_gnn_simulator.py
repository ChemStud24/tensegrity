from typing import Dict, List, Union

import torch
from torch_geometric.data import Data as GraphData

from gnn_simulator.gnn_physics.data_processors.fast_tensegrity_graph_data_processor import (
    MultiPlaneTensegrityGraphDataProcessor,
)
from gnn_simulator.simulators.tensegrity_gnn_simulator import MultiSimMultiStepMotorTensegrityGNNSimulator
from gnn_simulator.state_objects.primitive_shapes import StaticPrism, StaticRectPlane
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE


class PlanarEnvTensegrityGNNSimulator(MultiSimMultiStepMotorTensegrityGNNSimulator):

    def __init__(self,
                 n_out: int,
                 latent_dim: int,
                 nmessage_passing_steps: int,
                 nmlp_layers: int,
                 mlp_hidden_dim: int,
                 tensegrity_cfg,
                 processor_shared_weights=False,
                 dt=0.01,
                 additional_data_proc_kwargs: Dict | None = None,
                 num_sims: int = 10,
                 num_ctrls_hist: int = 20,
                 cache_batch_sizes: List[int] | None = None,
                 torch_compile: bool = False,
                 environment: Dict | None = None):
        self.all_env_planar_objs: List[Union[StaticPrism, StaticRectPlane]] = []
        if environment is not None:
            self.all_env_planar_objs = self._build_env_objs(environment)
        self.curr_env_planar_objs = [obj for obj in self.all_env_planar_objs]

        super().__init__(
            n_out,
            latent_dim,
            nmessage_passing_steps,
            nmlp_layers,
            mlp_hidden_dim,
            tensegrity_cfg,
            processor_shared_weights,
            dt,
            additional_data_proc_kwargs,
            num_sims,
            num_ctrls_hist,
            cache_batch_sizes,
            torch_compile,
        )

    def _build_env_objs(self, env_dict: Dict) -> List[Union[StaticPrism, StaticRectPlane]]:
        objs = []
        for obj_name, cfg in env_dict.items():
            obj_type = cfg['type']
            center = torch.tensor(cfg['pos'], dtype=DEFAULT_DTYPE).reshape(1, 3, 1)
            rot_mat = torch.tensor(cfg['rot_mat'], dtype=DEFAULT_DTYPE).reshape(1, 3, 3)
            half_lens = tuple(torch.tensor([h], dtype=DEFAULT_DTYPE) for h in cfg['half_lens'])

            if obj_type == 'StaticPrism':
                obj = StaticPrism(obj_name, center, rot_mat, half_lens)
            elif obj_type == 'StaticRectPlane':
                obj = StaticRectPlane(obj_name, center, rot_mat, half_lens)
            else:
                raise ValueError(f"Unknown env object type: {obj_type}")

            objs.append(obj)
        return objs

    def _get_data_processor(self):
        self.data_processor_kwargs['rest_lens_or_ctrls'] = 'ctrls'
        self.data_processor_kwargs['num_ctrls_hist'] = self.num_ctrls_hist
        return MultiPlaneTensegrityGraphDataProcessor(**self.data_processor_kwargs)

    def add_env_objs(self, environment: dict):
        """Merge new env objects into all_env_planar_objs; skip duplicates by name."""
        existing_names = {obj.name for obj in self.all_env_planar_objs}
        for obj in self._build_env_objs(environment):
            if obj.name not in existing_names:
                self.all_env_planar_objs.append(obj)
                existing_names.add(obj.name)

    def to(self, device):
        super().to(device)
        for obj in self.all_env_planar_objs:
            obj.to(device)
        return self

    def generate_graph(self, state, **kwargs):
        batch_size = state.shape[0]
        if batch_size not in self.data_processor.cached_batch_size_keys:
            self.data_processor.precompute_and_cache_batch_sizes([batch_size])

        dataset_idx = kwargs.pop('dataset_idx', 0)
        ctrls = kwargs.pop('ctrls', None)
        graph_feats, raw_feats = self.data_processor(
            state,
            self.curr_env_planar_objs,
            dataset_idx,
            ctrls,
        )

        combined_graph_feats = {
            **graph_feats._asdict(),
            **{k: v for raw_feat in raw_feats for k, v in raw_feat._asdict().items()},
            'cable_edge_index': graph_feats.cable_edge_idx.to(torch.long),
            'contact_edge_index': graph_feats.contact_edge_idx.to(torch.long),
            'body_edge_index': graph_feats.body_edge_idx.to(torch.long),
            'pos': raw_feats[0].node_pos,
            'vel': raw_feats[0].node_vel,
        }
        graph = GraphData(**combined_graph_feats)
        graph = self.add_hidden_state(graph)

        return graph
