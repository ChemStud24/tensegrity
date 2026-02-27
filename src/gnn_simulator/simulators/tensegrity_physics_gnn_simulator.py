from typing import Type

from torch_geometric.data import Data as Graph

from gnn_simulator.simulators.abstract_simulator import AbstractSimulator
from gnn_simulator.simulators.tensegrity_gnn_simulator import LearnedSimulator
from gnn_simulator.simulators.tensegrity_physics_simulator import TensegrityRobotSimulator
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.tensor_utils import zeros


class TensegrityHybridGNNSimulator(AbstractSimulator):

    def __init__(self,
                 physics_sim_cls: Type[TensegrityRobotSimulator],
                 physics_based_params: Dict,
                 gnn_sim_cls: Type[LearnedSimulator],
                 gnn_based_params: Dict):
        super().__init__()

        self.physics_sim = physics_sim_cls(
            **physics_based_params
        )
        self.gnn_sim = gnn_sim_cls(
            **gnn_based_params
        )

        # Force the two sims to share same robot object
        del self.gnn_sim.robot
        del self.gnn_sim.data_processor.robot
        self.gnn_sim.robot = self.physics_sim.robot
        self.gnn_sim.data_processor.robot = self.physics_sim.robot
        self.robot = self.physics_sim.robot


    def reset(self, **kwargs):
        self.physics_sim.reset()
        self.gnn_sim.reset(**kwargs)

    def to(self, device):
        super().to(device)
        self.physics_sim = self.physics_sim.to(device)
        self.gnn_sim = self.gnn_sim.to(device)

        return self

    @property
    def data_processor(self):
        return self.gnn_sim.data_processor

    def compute_contact_deltas(self,
                               pre_next_state: torch.Tensor,
                               dt: Union[torch.Tensor, float]
                               ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        pre_next_state_ = pre_next_state.reshape(-1, 13, 1)

        delta_v = zeros((pre_next_state_.shape[0], 3, 1),
                        ref_tensor=pre_next_state)
        delta_w = zeros((pre_next_state_.shape[0], 3, 1),
                        ref_tensor=pre_next_state)
        toi = zeros((pre_next_state_.shape[0], 1, 1),
                    ref_tensor=pre_next_state)

        return delta_v, delta_w, toi

    def resolve_contacts(self,
                         pre_next_state: torch.Tensor,
                         dt: Union[torch.Tensor, float],
                         delta_v,
                         delta_w,
                         toi) -> Tuple[torch.Tensor, Graph]:
        curr_state = self.get_curr_state()
        graph = self.gnn_sim.process_gnn(curr_state)

        curr_state_ = curr_state.reshape(-1, 13, 1)
        pre_next_state_ = pre_next_state.reshape(-1, 13, 1)
        dv = pre_next_state_[:, 7:10] - curr_state_[:, 7:10]
        dw = pre_next_state_[:, 10:] - curr_state_[:, 10:]
        pos = curr_state_[:, :3] + dt * dv
        quat = torch_quaternion.update_quat(curr_state_[:, 3:7], dw, dt)
        # #
        pf_pos = self.data_processor.pose2node(
            torch.hstack([pos, quat]),
            augment_grnd=False
        )
        body_mask = graph.node_type.flatten() == 0
        pf_dv = torch.zeros_like(graph.pos)
        pf_dv[body_mask] = (pf_pos - graph.pos[body_mask]) / dt.squeeze(-1)

        graph['pf_dv'] = pf_dv
        graph['p_vel'] = graph.p_vel + pf_dv
        graph['p_pos'] = graph.p_pos + pf_dv * dt.squeeze(-1)

        next_state = self.data_processor.node2pose(
            graph.p_pos[body_mask],
            graph.pos[body_mask],
            self.robot.num_nodes_per_rod
        )

        return next_state, graph


class TensegrityHybridRecurrentGNNSimulator(TensegrityHybridGNNSimulator):

    def __init__(self,
                 tensegrity_cfg,
                 gravity,
                 contact_params,
                 n_out: int,
                 latent_dim: int,
                 nmessage_passing_steps: int,
                 nmlp_layers: int,
                 mlp_hidden_dim: int,
                 processor_shared_weights=False,
                 dt=0.01,
                 n_hist=1,
                 hidden_state_dim=64):
        self.hidden_state_dim = hidden_state_dim
        super().__init__(tensegrity_cfg,
                         gravity,
                         contact_params,
                         n_out,
                         latent_dim,
                         nmessage_passing_steps,
                         nmlp_layers,
                         mlp_hidden_dim,
                         processor_shared_weights,
                         dt,
                         n_hist)

    def get_gnn_sim(self, **kwargs):
        return TensegrityRecurrentGNNSimulator(
            n_out=kwargs['n_out'],
            latent_dim=kwargs['latent_dim'],
            nmessage_passing_steps=kwargs['nmessage_passing_steps'],
            nmlp_layers=kwargs['nmlp_layers'],
            mlp_hidden_dim=kwargs['mlp_hidden_dim'],
            processor_shared_weights=kwargs['processor_shared_weights'],
            dt=kwargs['dt'],
            robot=self.robot,
            hidden_state_dim=self.hidden_state_dim
        )

    def resolve_contacts(self,
                         pre_next_state: torch.Tensor,
                         dt: Union[torch.Tensor, float],
                         delta_v,
                         delta_w,
                         toi) -> Tuple[torch.Tensor, Graph]:
        curr_state = self.get_curr_state()
        graph = self.gnn_sim.process_gnn(curr_state)

        body_mask = graph.body_mask.flatten()
        pf_dv = torch.zeros_like(graph.pos)

        graph['pf_dv'] = pf_dv
        graph['p_vel'] = graph.p_vel + pf_dv
        graph['p_pos'] = graph.p_pos + pf_dv * dt.squeeze(-1)

        next_state = self.data_processor.node2pose(
            graph.p_pos[body_mask],
            graph.pos[body_mask],
            self.robot.num_nodes_per_rod
        )

        return next_state, graph


class TensegrityMotorRecurrentGNNSimulator(TensegrityHybridRecurrentGNNSimulator):

    def __init__(self,
                 tensegrity_cfg,
                 gravity,
                 contact_params,
                 n_out: int,
                 latent_dim: int,
                 nmessage_passing_steps: int,
                 nmlp_layers: int,
                 mlp_hidden_dim: int,
                 processor_shared_weights=False,
                 dt=0.01,
                 n_hist=1,
                 hidden_state_dim=64):
        super().__init__(
            tensegrity_cfg,
            gravity,
            contact_params,
            n_out,
            latent_dim,
            nmessage_passing_steps,
            nmlp_layers,
            mlp_hidden_dim,
            processor_shared_weights,
            dt=dt,
            n_hist=n_hist,
            hidden_state_dim=hidden_state_dim)
        self.ctrl_hist = None

    def get_gnn_sim(self, **kwargs):
        return TensegrityRecurrentMotorGNNSimulator(
            n_out=kwargs['n_out'],
            latent_dim=kwargs['latent_dim'],
            nmessage_passing_steps=kwargs['nmessage_passing_steps'],
            nmlp_layers=kwargs['nmlp_layers'],
            mlp_hidden_dim=kwargs['mlp_hidden_dim'],
            processor_shared_weights=kwargs['processor_shared_weights'],
            dt=kwargs['dt'],
            robot=self.robot
        )

    def apply_control(self, control_signals, dt):
        # do nothing
        pass

    def step(self,
             curr_state: torch.Tensor,
             dt: Union[torch.Tensor, float],
             external_forces: Optional[torch.Tensor] = None,
             external_pts: Optional[torch.Tensor] = None,
             control_signals: Optional[torch.Tensor] = None,
             **kwargs) -> torch.Tensor:
        control_signals = torch.concat([self.ctrl_hist, control_signals], dim=2)

        self.update_state(curr_state)
        graph = self.gnn_sim.process_gnn(curr_state,
                                         ctrls=control_signals)

        body_mask = graph.node_type.flatten() == 0
        next_state = self.data_processor.node2pose(
            graph.p_pos[body_mask],
            graph.pos[body_mask],
            self.robot.num_nodes_per_rod
        )

        self.ctrl_hist = control_signals[..., 1:].clone()

        return next_state, graph

