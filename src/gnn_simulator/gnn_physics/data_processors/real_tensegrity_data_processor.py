from batch_tensegrity_data_processor import *


class MultiSimRealTensegrityDataProcessor(MultiSimTensegrityDataProcessor):

    def __init__(self, **data_processor_kwargs):
        super().__init__(**data_processor_kwargs)

        self.hier_node_feat_dict['node']['node_vel_dt'] = 1
        self.node_feat_lens['node'] += 1

        self.normalizers['node_vel_dt'] = AccumulatedNormalizer(
            (1, 1),
            dtype=self.dtype,
            name="node_vel_dt"
        )

    def batch_state_to_graph(self, states: List[torch.Tensor], **kwargs) -> GraphData:
        graph = super().batch_state_to_graph(states, **kwargs)
        vel_dt = kwargs['vel_dt']

        if isinstance(vel_dt, float):
            vel_dt = vel_dt * torch.ones_likes(graph.pos)
        elif isinstance(vel_dt, torch.Tensor) and vel_dt.shape[0] == len(states[0]):
            num_nodes = graph.pos.shape[0] // len(states[0])
            vel_dt = vel_dt.repeat(1, num_nodes).reshape(-1, 1)

        graph['node_vel_dt'] = vel_dt
        return graph
