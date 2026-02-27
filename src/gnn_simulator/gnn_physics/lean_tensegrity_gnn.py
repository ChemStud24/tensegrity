from typing import Tuple, Callable

import torch
from torch import nn


def build_mlp(input_dim: int,
              hidden_dim: int,
              num_layers: int,
              output_dim: int
              ) -> nn.Sequential:
    """
    Builds an MLP with ReLU activations and no normalization at the intermediate layers.
    """
    layers = []

    # Input layer
    layers.append(nn.Linear(input_dim, hidden_dim))
    layers.append(nn.ReLU())

    # Hidden layers
    for _ in range(num_layers - 1):
        layers.append(nn.Linear(hidden_dim, hidden_dim))
        layers.append(nn.ReLU())

    # Output layer
    layers.append(nn.Linear(hidden_dim, output_dim))

    return nn.Sequential(*layers)


class Encoder(nn.Module):
    node_encoder: nn.Module
    body_edge_encoder: nn.Module
    cable_edge_encoder: nn.Module
    contact_edge_encoder: nn.Module

    def __init__(self,
                 node_input_dim: int,
                 body_edge_input_dim: int,
                 cable_edge_input_dim: int,
                 contact_edge_input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int):
        """
        Constructs an encoder with four separate MLPs:
        - node_encoder
        - body_edge_encoder
        - cable_edge_encoder
        - contact_edge_encoder

        Each MLP has the same architecture and is followed by LayerNorm.

        Args:
            input_dim (int): Input feature size for all encoders
            hidden_dim (int): Width of hidden layers
            num_layers (int): Number of hidden layers
            output_dim (int): Output feature size for all encoders
        """
        super().__init__()

        self.node_encoder = nn.Sequential(
            build_mlp(node_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.body_edge_encoder = nn.Sequential(
            build_mlp(body_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.cable_edge_encoder = nn.Sequential(
            build_mlp(cable_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.contact_edge_encoder = nn.Sequential(
            build_mlp(contact_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

    def forward(self,
                node_x: torch.Tensor,
                body_edge_attr: torch.Tensor,
                cable_edge_attr: torch.Tensor,
                contact_edge_attr: torch.Tensor
                ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass through all four encoders.

        Args:
            x_node, x_body, x_cable, x_contact (Tensor): Input tensors for each encoder

        Returns:
            Tuple[Tensor, Tensor, Tensor, Tensor]: Encoded outputs
        """
        out_node = self.node_encoder(node_x)
        out_body = self.body_edge_encoder(body_edge_attr)
        out_cable = self.cable_edge_encoder(cable_edge_attr)
        out_contact = self.contact_edge_encoder(contact_edge_attr)

        return out_node, out_body, out_cable, out_contact


class RecurrentEncoder(nn.Module):
    node_encoder: nn.Module
    body_edge_encoder: nn.Module
    cable_edge_encoder: nn.Module
    contact_edge_encoder: nn.Module
    recurrent_type: str
    node_recurrent_block: nn.Module
    node_recur_layer_norm: nn.Module
    forward_fn: Callable

    def __init__(self,
                 node_input_dim: int,
                 body_edge_input_dim: int,
                 cable_edge_input_dim: int,
                 contact_edge_input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int,
                 recurrent_type='lstm'):
        super().__init__()

        self.node_encoder = nn.Sequential(
            build_mlp(node_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.body_edge_encoder = nn.Sequential(
            build_mlp(body_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.cable_edge_encoder = nn.Sequential(
            build_mlp(cable_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        self.contact_edge_encoder = nn.Sequential(
            build_mlp(contact_edge_input_dim, hidden_dim, num_layers, output_dim),
            nn.LayerNorm(output_dim)
        )

        def mlp(in_feats, nout, num_layers=num_layers):
            """
            method to quickly augment mlp with LayerNorm as last layer
            @param in_feats:
            @return:
            """
            return nn.Sequential(
                *[build_mlp(in_feats,
                            [hidden_dim
                             for _ in range(num_layers)],
                            nout),
                  nn.LayerNorm(nout)]
            )

        self.recurrent_type = recurrent_type

        if recurrent_type == 'mlp':
            self.node_recurrent_block = mlp(2 * output_dim, output_dim, 2)
            self.forward_fn = self.mlp_forward
        elif recurrent_type == 'rnn':
            self.node_recurrent_block = nn.RNNCell(output_dim, output_dim)
            self.forward_fn = self.rnn_gru_forward
        elif recurrent_type == 'lstm':
            self.node_recurrent_block = nn.LSTMCell(output_dim, output_dim)
            self.forward_fn = self.lstm_forward
        elif recurrent_type == 'gru':
            self.node_recurrent_block = nn.GRUCell(output_dim, output_dim)
            self.forward_fn = self.rnn_gru_forward
        else:
            raise Exception("recurrent_type not valid")

        self.node_recur_layer_norm = nn.LayerNorm(output_dim)

    def to(self, device):
        super().to(device)
        self.node_recurrent_block = self.node_recurrent_block.to(device)
        self.node_recur_layer_norm = self.node_recur_layer_norm.to(device)

        return self

    def mlp_forward(self, node_x, hidden_state) -> Tuple[torch.Tensor, torch.Tensor]:
        node_x = self.node_recurrent_block(
            torch.hstack([node_x, hidden_state])
        )
        node_x = self.node_recur_layer_norm(node_x)

        return node_x, node_x.clone()

    def rnn_gru_forward(self, node_x, hidden_state) -> Tuple[torch.Tensor, torch.Tensor]:
        node_x = self.node_recurrent_block(node_x, hidden_state)
        node_x = self.node_recur_layer_norm(node_x)
        return node_x, node_x.clone()

    def lstm_forward(self, node_x, hidden_state) -> Tuple[torch.Tensor, torch.Tensor]:
        ndim = node_x.shape[1]
        node_x, memory = self.node_recurrent_block(
            node_x,
            (hidden_state[:ndim], hidden_state[ndim:])
        )
        node_x = self.node_recur_layer_norm(node_x)

        new_hidden_state = torch.hstack([
            node_x.clone(),
            memory.clone()
        ])

        return node_x, new_hidden_state

    def forward(self,
                node_x: torch.Tensor,
                body_edge_attr: torch.Tensor,
                cable_edge_attr: torch.Tensor,
                contact_edge_attr: torch.Tensor,
                node_hidden_state: torch.Tensor
                ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass through all four encoders.

        Args:
            x_node, x_body, x_cable, x_contact (Tensor): Input tensors for each encoder

        Returns:
            Tuple[Tensor, Tensor, Tensor, Tensor]: Encoded outputs
        """
        out_node = self.node_encoder(node_x)
        out_node, out_hidden_state = self.forward_fn(out_node, node_hidden_state)

        out_body = self.body_edge_encoder(body_edge_attr)
        out_cable = self.cable_edge_encoder(cable_edge_attr)
        out_contact = self.contact_edge_encoder(contact_edge_attr)

        return out_node, out_body, out_cable, out_contact, out_hidden_state


class InteractionNetwork(nn.Module):
    body_mp: nn.Module
    cable_mp: nn.Module
    contact_mp: nn.Module
    node_updater: nn.Module

    def __init__(self,
                 input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int):
        super().__init__()
        self.body_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.cable_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.contact_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.node_updater = build_mlp(input_dim * 4, hidden_dim, num_layers, output_dim)

    def msg_pass(self, x, edge_attr, edge_idx, mp_nn):
        x_i = x[edge_idx[0]]
        x_j = x[edge_idx[1]]
        concat_vec = torch.hstack([x_i, x_j, edge_attr])

        msg = mp_nn(concat_vec)
        edge_attr = edge_attr + msg

        return edge_attr

    def aggregate_msg(self, edge_attr, node_idxs, mask):
        tmp = torch.vstack([edge_attr * mask, torch.zeros_like(edge_attr[:1])])
        stacked_edges = tmp[node_idxs]
        agg_msg = stacked_edges.sum(dim=1)
        return agg_msg

    def update(self, node_x, agg_body_edges, agg_cable_edges, agg_contact_edges, update_nn):
        concat_vec = torch.hstack([node_x, agg_body_edges, agg_cable_edges, agg_contact_edges])
        node_x = node_x + update_nn(concat_vec)
        return node_x

    def forward(self,
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_idx,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_idx,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_idx,
                contact_mask):
        body_mask = torch.ones_like(body_edge_attr[:, :1])
        next_body_edge_attr = self.msg_pass(node_x, body_edge_attr, body_edge_idx, self.body_mp)
        agg_body_node = self.aggregate_msg(next_body_edge_attr, body_edge_agg_idx, body_mask)

        cable_mask = torch.ones_like(cable_edge_attr[:, :1])
        next_cable_edge_attr = self.msg_pass(node_x, cable_edge_attr, cable_edge_idx, self.cable_mp)
        agg_cable_node = self.aggregate_msg(next_cable_edge_attr, cable_edge_agg_idx, cable_mask)

        next_contact_edge_attr = self.msg_pass(node_x, contact_edge_attr, contact_edge_idx, self.contact_mp)
        agg_contact_node = self.aggregate_msg(next_contact_edge_attr, contact_edge_agg_idx, contact_mask)

        node_x = self.update(node_x, agg_body_node, agg_cable_node, agg_contact_node, self.node_updater)

        return node_x, next_body_edge_attr, next_cable_edge_attr, next_contact_edge_attr


class InteractionNetwork2(nn.Module):
    body_mp: nn.Module
    cable_mp: nn.Module
    contact_mp: nn.Module
    node_updater: nn.Module

    def __init__(self,
                 input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int):
        super().__init__()
        self.body_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.cable_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.contact_mp = build_mlp(input_dim * 3, hidden_dim, num_layers, output_dim)
        self.node_updater = build_mlp(input_dim * 4, hidden_dim, num_layers, output_dim)

    def msg_pass(self, x, edge_attr, edge_idx, mp_nn):
        x_i = x[edge_idx[0]]
        x_j = x[edge_idx[1]]
        concat_vec = torch.hstack([x_i, x_j, edge_attr])

        msg = mp_nn(concat_vec)
        edge_attr = edge_attr + msg

        return edge_attr

    def aggregate_msg(self, edge_attr, edge_agg_mat, mask):
        edge_attr = mask * edge_attr
        agg_msg = torch.mm(edge_agg_mat, edge_attr)
        return agg_msg

    def update(self, node_x, agg_body_edges, agg_cable_edges, agg_contact_edges, update_nn):
        concat_vec = torch.hstack([node_x, agg_body_edges, agg_cable_edges, agg_contact_edges])
        node_x = node_x + update_nn(concat_vec)
        return node_x

    def forward(self,
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_mat,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_mat,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_mat,
                contact_mask):
        body_mask = torch.ones_like(body_edge_attr[:, :1])
        next_body_edge_attr = self.msg_pass(node_x, body_edge_attr, body_edge_idx, self.body_mp)
        agg_body_node = self.aggregate_msg(next_body_edge_attr, body_edge_agg_mat, body_mask)

        cable_mask = torch.ones_like(cable_edge_attr[:, :1])
        next_cable_edge_attr = self.msg_pass(node_x, cable_edge_attr, cable_edge_idx, self.cable_mp)
        agg_cable_node = self.aggregate_msg(next_cable_edge_attr, cable_edge_agg_mat, cable_mask)

        next_contact_edge_attr = self.msg_pass(node_x, contact_edge_attr, contact_edge_idx, self.contact_mp)
        agg_contact_node = self.aggregate_msg(next_contact_edge_attr, contact_edge_agg_mat, contact_mask)

        node_x = self.update(node_x, agg_body_node, agg_cable_node, agg_contact_node, self.node_updater)

        return node_x, next_body_edge_attr, next_cable_edge_attr, next_contact_edge_attr


class Processor(nn.Module):
    interaction_networks: nn.ParameterList

    def __init__(self,
                 input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int,
                 num_msg_passes: int):
        super().__init__()
        self.interaction_networks = nn.ParameterList([
            InteractionNetwork(input_dim, hidden_dim, num_layers, output_dim)
            for _ in range(num_msg_passes)
        ])

    def forward(self,
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_idx,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_idx,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_idx,
                contact_mask):
        for interaction_network in self.interaction_networks:
            node_x, body_edge_attr, cable_edge_attr, contact_edge_attr = interaction_network(
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_idx,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_idx,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_idx,
                contact_mask
            )

        return node_x, body_edge_attr, cable_edge_attr, contact_edge_attr


class Processor2(nn.Module):
    interaction_networks: nn.ParameterList

    def __init__(self,
                 input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int,
                 num_msg_passes: int):
        super().__init__()
        self.interaction_networks = nn.ParameterList([
            InteractionNetwork2(input_dim, hidden_dim, num_layers, output_dim)
            for _ in range(num_msg_passes)
        ])

    def forward(self,
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_mat,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_mat,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_mat,
                contact_mask):
        for interaction_network in self.interaction_networks:
            node_x, body_edge_attr, cable_edge_attr, contact_edge_attr = interaction_network(
                node_x,
                body_edge_attr,
                body_edge_idx,
                body_edge_agg_mat,
                cable_edge_attr,
                cable_edge_idx,
                cable_edge_agg_mat,
                contact_edge_attr,
                contact_edge_idx,
                contact_edge_agg_mat,
                contact_mask
            )

        return node_x, body_edge_attr, cable_edge_attr, contact_edge_attr


class Decoder(nn.Module):
    decoder: nn.Module

    def __init__(self,
                 input_dim: int,
                 hidden_dim: int,
                 num_layers: int,
                 output_dim: int):
        super().__init__()
        self.decoder = build_mlp(input_dim, hidden_dim, num_layers, output_dim)

    def forward(self, node_x):
        norm_dv = self.decoder(node_x)
        return norm_dv


class FastEncoderProcessorDecoder(nn.Module):
    def __init__(self,
                 node_input_dim: int,
                 body_edge_input_dim: int,
                 cable_edge_input_dim: int,
                 contact_input_dim: int,
                 mlp_hidden_dim: int,
                 latent_dim: int,
                 nmlp_layers: int,
                 nmessage_passing_steps: int,
                 n_out: int):
        super().__init__()
        self._encoder = Encoder(
            node_input_dim=node_input_dim,
            body_edge_input_dim=body_edge_input_dim,
            cable_edge_input_dim=cable_edge_input_dim,
            contact_edge_input_dim=contact_input_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=latent_dim
        )
        self._processor = Processor(
            input_dim=latent_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=latent_dim,
            num_msg_passes=nmessage_passing_steps)
        self._decoder = Decoder(
            input_dim=latent_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=n_out
        )

    def forward(self, graph_feats):
        x, body_edge_attr, cable_edge_attr, contact_edge_attr = (
            self._encoder(
                graph_feats.node_x,
                graph_feats.body_edge_attr,
                graph_feats.cable_edge_attr,
                graph_feats.contact_edge_attr
            )
        )
        x, body_edge_attr, cable_edge_attr, contact_edge_attr = (
            self._processor(
                x,
                body_edge_attr,
                graph_feats.body_edge_idx,
                graph_feats.body_edge_agg_idx,
                cable_edge_attr,
                graph_feats.cable_edge_idx,
                graph_feats.cable_edge_agg_idx,
                contact_edge_attr,
                graph_feats.contact_edge_idx,
                graph_feats.contact_edge_agg_idx,
                graph_feats.contact_close_mask
            )
        )
        norm_dv = self._decoder(x)
        return norm_dv


class FastRecurEncoderProcessorDecoder(nn.Module):
    def __init__(self,
                 node_input_dim: int,
                 body_edge_input_dim: int,
                 cable_edge_input_dim: int,
                 contact_input_dim: int,
                 mlp_hidden_dim: int,
                 latent_dim: int,
                 nmlp_layers: int,
                 nmessage_passing_steps: int,
                 n_out: int):
        super().__init__()
        self._encoder = RecurrentEncoder(
            node_input_dim=node_input_dim,
            body_edge_input_dim=body_edge_input_dim,
            cable_edge_input_dim=cable_edge_input_dim,
            contact_edge_input_dim=contact_input_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=latent_dim
        )
        self._processor = Processor(
            input_dim=latent_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=latent_dim,
            num_msg_passes=nmessage_passing_steps)
        self._decoder = Decoder(
            input_dim=latent_dim,
            hidden_dim=mlp_hidden_dim,
            num_layers=nmlp_layers,
            output_dim=n_out
        )

    def forward(self, graph_feats):
        node_x, body_edge_attr, cable_edge_attr, contact_edge_attr, hidden_state = (
            self._encoder(
                graph_feats.node_x,
                graph_feats.body_edge_attr,
                graph_feats.cable_edge_attr,
                graph_feats.contact_edge_attr
            )
        )
        node_x, body_edge_attr, cable_edge_attr, contact_edge_attr = (
            self._processor(
                node_x,
                body_edge_attr,
                graph_feats.body_edge_idx,
                graph_feats.body_edge_agg_idx,
                cable_edge_attr,
                graph_feats.cable_edge_idx,
                graph_feats.cable_edge_agg_idx,
                contact_edge_attr,
                graph_feats.contact_edge_idx,
                graph_feats.contact_edge_agg_idx,
                graph_feats.contact_close_mask
            )
        )
        norm_dv = self._decoder(node_x)
        return norm_dv, hidden_state