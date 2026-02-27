from typing import List

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import MultivariateNormal, Categorical, MixtureSameFamily

from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE
from gnn_simulator.utilities.tensor_utils import zeros


def build_mlp(
        input_size: int,
        hidden_layer_sizes: List[int],
        output_size: int = None,
        output_activation: nn.Module = nn.Identity,
        activation: nn.Module = nn.ReLU) -> nn.Module:
    """Build a MultiLayer Perceptron.
    Args:
      input_size: Size of input layer.
      hidden_layer_sizes: An array of input size for each hidden layer.
      output_size: Size of the output layer.
      output_activation: Activation function for the output layer.
      activation: Activation function for the hidden layers.
    Returns:
      mlp: An MLP sequential container.
    """
    # Size of each layer
    layer_sizes = [input_size] + hidden_layer_sizes
    if output_size:
        layer_sizes.append(output_size)

    # Number of layers
    nlayers = len(layer_sizes) - 1

    # Create a list of activation functions and
    # set the last element to output activation function
    act = [activation for i in range(nlayers)]
    act[-1] = output_activation

    # Create a torch sequential container
    mlp = nn.Sequential()
    for i in range(nlayers):
        mlp.add_module("NN" + str(i), nn.Linear(layer_sizes[i],
                                                layer_sizes[i + 1]))
        mlp.add_module("Act" + str(i), act[i]())

    return mlp


class MaskedLinear(nn.Linear):
    """ same as Linear except has a configurable mask on the weights """

    def __init__(self, in_features, out_features, bias=True):
        super().__init__(in_features, out_features, bias)
        self.register_buffer('mask', torch.ones(out_features, in_features))

    def set_mask(self, mask):
        self.mask.data.copy_(torch.from_numpy(mask.astype(np.uint8).T))

    def forward(self, input):
        return F.linear(input, self.mask * self.weight, self.bias)


class MADE(nn.Module):
    def __init__(self, nin, hidden_sizes, nout, num_masks=1, natural_ordering=False):
        """
        Adapted from https://github.com/karpathy/pytorch-made
        nin: integer; number of inputs
        hidden sizes: a list of integers; number of units in hidden layers
        nout: integer; number of outputs, which usually collectively parameterize some kind of 1D distribution
              note: if nout is e.g. 2x larger than nin (perhaps the mean and std), then the first nin
              will be all the means and the second nin will be stds. i.e. output dimensions depend on the
              same input dimensions in "chunks" and should be carefully decoded downstream appropriately.
              the output of running the tests for this file makes this a bit more clear with examples.
        num_masks: can be used to train ensemble over orderings/connections
        natural_ordering: force natural ordering of dimensions, don't use random permutations
        """

        super().__init__()
        self.nin = nin
        self.nout = nout
        self.hidden_sizes = hidden_sizes
        assert self.nout % self.nin == 0, "nout must be integer multiple of nin"

        # define a simple MLP neural net
        self.net = []
        hs = [nin] + hidden_sizes + [nout]
        for h0, h1 in zip(hs, hs[1:]):
            self.net.extend([
                MaskedLinear(h0, h1),
                nn.ReLU(),
            ])
        self.net.pop()  # pop the last ReLU for the output layer
        self.net = nn.Sequential(*self.net)

        # seeds for orders/connectivities of the model ensemble
        self.natural_ordering = natural_ordering
        self.num_masks = num_masks
        self.seed = 0  # for cycling through num_masks orderings

        self.m = {}
        self.update_masks()  # builds the initial self.m connectivity
        # note, we could also precompute the masks and cache them, but this
        # could get memory expensive for large number of masks.

    def update_masks(self):
        if self.m and self.num_masks == 1:
            return  # only a single seed, skip for efficiency
        L = len(self.hidden_sizes)

        # fetch the next seed and construct a random stream
        rng = np.random.RandomState(self.seed)
        self.seed = (self.seed + 1) % self.num_masks

        # sample the order of the inputs and the connectivity of all neurons
        self.m[-1] = np.arange(
            self.nin) if self.natural_ordering else rng.permutation(self.nin)
        for l in range(L):
            self.m[l] = rng.randint(
                self.m[l - 1].min(), self.nin - 1, size=self.hidden_sizes[l])

        # construct the mask matrices
        masks = [self.m[l - 1][:, None] <= self.m[l][None, :] for l in range(L)]
        masks.append(self.m[L - 1][:, None] < self.m[-1][None, :])

        # handle the case where nout = nin * k, for integer k > 1
        if self.nout > self.nin:
            k = int(self.nout / self.nin)
            # replicate the mask across the other outputs
            masks[-1] = np.concatenate([masks[-1]] * k, axis=1)

        # set the masks in all MaskedLinear layers
        layers = [l for l in self.net.modules() if isinstance(l, MaskedLinear)]
        for l, m in zip(layers, masks):
            l.set_mask(m)

    def forward(self, x):
        return self.net(x)


class ARMLP(nn.Module):
    """ a 4-layer auto-regressive MLP, wrapper around MADE net """

    def __init__(self, nin, nout, nh):
        super().__init__()
        self.net = MADE(nin,
                        [nh] * 8,
                        nout,
                        num_masks=2,
                        natural_ordering=True)

    def forward(self, x):
        return self.net(x)


class MAF(nn.Module):
    """ Masked Autoregressive Flow that uses a MADE-style network for fast forward """

    def __init__(self,
                 input_dim,
                 hidden_dim,
                 cond_dim,
                 parity,
                 net_class=ARMLP):
        super().__init__()
        self.dim = input_dim
        self.net = net_class(input_dim, input_dim * 2, hidden_dim)
        self.parity = parity
        self.context_s = build_mlp(input_dim + cond_dim,
                                   [hidden_dim
                                    for _ in range(2)],
                                   input_dim)
        self.context_t = build_mlp(input_dim + cond_dim,
                                   [hidden_dim
                                    for _ in range(2)],
                                   input_dim)

    def move_tensors(self, device):
        self.net = self.net.to(device)
        self.context_s = self.context_s.to(device)
        self.context_t = self.context_t.to(device)

        return self

    def forward(self, x, h):
        # Evaluate Z in parallel, density estimation is fast
        # x_cond = torch.hstack([x, h])
        st = self.net(x)
        s, t = st.split(self.dim, dim=1)
        s = s + self.context_s(torch.hstack([s, h]))
        t = t + self.context_t(torch.hstack([t, h]))
        z = (x - t) * torch.exp(-0.5 * s)
        z = z.flip(dims=(1,)) if self.parity else z
        log_det = 0.5 * torch.sum(-s, dim=1)
        return z, log_det

    def backward(self, z, h):
        # Decode X one at a time, sequentially, sampling will be slow
        x = torch.zeros_like(z)
        log_det = torch.zeros(z.size(0), dtype=z.dtype, device=z.device)
        z = z.flip(dims=(1,)) if self.parity else z
        for i in range(self.dim):
            # x_cond = torch.hstack([x.clone(), h.clone()])
            st = self.net(x.clone())
            s, t = st.split(self.dim, dim=1)
            s = s + self.context_s(torch.hstack([s, h]))
            t = t + self.context_t(torch.hstack([t, h]))
            x[:, i] = z[:, i] * torch.exp(0.5 * s[:, i]) + t[:, i]
            log_det += s[:, i]
        return x, 0.5 * log_det


class IAF(MAF):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        """
        reverse the flow, giving an Inverse Autoregressive Flow (IAF) instead, 
        where sampling will be fast but density estimation slow
        """
        self.forward, self.backward = self.backward, self.forward


class NormalizingFlow(nn.Module):
    """ A sequence of Normalizing Flows is a Normalizing Flow """

    def __init__(self, flows):
        super().__init__()
        self.flows = nn.ModuleList(flows)

    def move_tensors(self, device):
        for i in range(len(self.flows)):
            self.flows[i] = self.flows[i].move_tensors(device)
        return self

    def forward(self, x, h):
        m, _ = x.shape
        log_det = torch.zeros(m).to(x.device)
        zs = [x]
        for flow in self.flows:
            x, ld = flow.forward(x, h)
            log_det += ld
            zs.append(x)
        return zs, log_det

    def backward(self, z, h):
        m, _ = z.shape
        log_det = torch.zeros(m).to(z.device)
        xs = [z]
        for flow in self.flows[::-1]:
            z, ld = flow.backward(z, h)
            log_det += ld
            xs.append(z)
        return xs, log_det


class NormalizingFlowModel(nn.Module):
    """ A Normalizing Flow Model is a (prior, flow) pair """

    def __init__(self, prior, flows):
        super().__init__()
        self.prior = prior
        self.flow = NormalizingFlow(flows)

    def move_tensors(self, device):
        self.flow = self.flow.move_tensors(device)
        return self

    # def double(self: T) -> T:
    #     self.

    def forward(self, x, h):
        zs, log_det = self.flow.forward(x, h)
        prior_logprob = self.prior.log_prob(zs[-1].cpu()).view(x.size(0), -1).sum(1)
        prior_logprob = prior_logprob.to(x.device)
        return zs, prior_logprob, log_det

    def backward(self, z, h):
        xs, log_det = self.flow.backward(z, h)
        return xs, log_det

    def sample(self, num_samples, h):
        z = self.prior.sample((num_samples,))
        prior_logprob = self.prior.log_prob(z).to(h.device)
        z = z.to(h.device)
        xs, log_det = self.flow.backward(z, h)
        return xs, prior_logprob, log_det

    def sample_center(self, num_samples, h):
        z = zeros((num_samples, self.prior.mean.shape[0]), ref_tensor=h)
        prior_logprob = self.prior.log_prob(z.cpu()).to(h.device)
        xs, log_det = self.flow.backward(z, h)

        return xs, prior_logprob, log_det


class GaussianDecoder(nn.Module):

    def __init__(self,
                 cond_dim,
                 hidden_dim,
                 output_dim):
        super().__init__()

        self.t = build_mlp(cond_dim,
                           [hidden_dim for _ in range(2)],
                           output_dim)
        self.s = build_mlp(cond_dim,
                           [hidden_dim for _ in range(2)],
                           output_dim)
        self.prior = MultivariateNormal(
            torch.zeros(output_dim, dtype=DEFAULT_DTYPE),
            torch.eye(output_dim, dtype=DEFAULT_DTYPE)
        )

    def forward(self, graph):
        graph['decode_output'] = self.sample_max_prob(graph.x)

        return graph

    def log_prob(self, x, y):
        t, s = self.t(x), self.s(x)
        z = (y - t) * torch.exp(-0.5 * s)

        # c = torch.log(torch.tensor((2 * torch.pi) ** (-3/2), dtype=torch.float64))
        # log_det = 0.0
        log_det = 0.5 * torch.sum(s, dim=1)
        logprob = self.prior.log_prob(z.cpu()).to(x.device) - log_det

        return logprob, log_det

    def sample(self, x):
        t = self.t(x)
        s = self.s(x)

        dist = self.prior
        z = dist.sample(x.shape[0]).to(x.device)

        # y = z + t
        y = z * torch.exp(0.5 * s) + t

        return y

    def sample_max_prob(self, x):
        t = self.t(x)
        s = self.s(x)

        z = zeros((x.shape[0], 3), ref_tensor=x)
        # y = z + t
        y = z * torch.exp(0.5 * s) + t

        # prior_logprob = self.prior.log_prob(z.cpu())

        return y


class MixGaussianDecoder(nn.Module):

    def __init__(self,
                 cond_dim,
                 hidden_dim,
                 output_dim,
                 nmlp_layers,
                 num_gaussians=1,
                 max_sigma=10.0):
        super().__init__()
        self.output_dim = output_dim
        self.num_gaussians = num_gaussians
        self.max_sigma = max_sigma
        self.sample_strategy = "max_prob"

        self.mix_weights_nn = build_mlp(cond_dim,
                                        [hidden_dim for _ in range(nmlp_layers)],
                                        num_gaussians)
        self.mix_weights_nn.add_module("softmax", nn.Softmax(dim=1))

        self.t = build_mlp(cond_dim,
                           [hidden_dim for _ in range(nmlp_layers)],
                           output_dim * num_gaussians)
        self.s = build_mlp(cond_dim,
                           [hidden_dim for _ in range(nmlp_layers)],
                           output_dim * num_gaussians)

    def forward(self, graph):
        # graph['x'] = graph.x + graph.vel_x

        # graph['decode_output'] = self.sample_max_prob(graph.x) \
        #     if self.sample_strategy == 'max_prob' \
        #     else self.sample(graph.x)
        graph['decode_output'], graph['sigma'] = self.sample_max_prob_w_sigma(graph.x)

        return graph

    def forward_w_sigma(self, graph):
        graph['decode_output'], graph['sigma'] = self.sample_max_prob_w_sigma(graph.x)
        return graph

    def compute_dist(self, x):
        s, t, weights = self.compute_gmm_params(x)

        w = Categorical(weights)
        c = MultivariateNormal(t, s)
        dist = MixtureSameFamily(w, c)

        return dist

    def compute_gmm_params(self, x):
        weights = self.mix_weights_nn(x)
        t = self.t(x).reshape(-1, self.num_gaussians, self.output_dim)
        s = self.s(x).reshape(-1, self.num_gaussians, self.output_dim)
        s = self.max_sigma * torch.sigmoid(s)
        s = torch.diag_embed(s)
        #
        # s = 0.1 * (torch.eye(3, dtype=x.dtype, device=x.device)
        #            .unsqueeze(0)
        #            .unsqueeze(0)
        #            .repeat(x.shape[0], self.num_gaussians, 1, 1))

        return s, t, weights

    def log_prob(self, x, y):
        dist = self.compute_dist(x)

        # c = torch.log(torch.tensor((2 * torch.pi) ** (-3/2), dtype=torch.float64))
        log_det = 0.0
        logprob = dist.log_prob(y)

        return logprob, log_det

    def sample(self, x):
        dist = self.compute_dist(x)
        y = dist.sample()

        return y

    def sample_max_prob(self, x):
        s, t, weights = self.compute_gmm_params(x)

        w = Categorical(weights)
        c = MultivariateNormal(t, s)
        dist = MixtureSameFamily(w, c)

        log_probs = torch.stack([
            dist.log_prob(t[:, i])
            for i in range(t.shape[1])
        ], dim=1)
        idxs = log_probs.argmax(dim=1)
        y = t[torch.arange(t.shape[0]), idxs]

        return y

    def sample_max_prob_w_sigma(self, x):
        weights = self.mix_weights_nn(x)
        t = self.t(x).reshape(-1, self.num_gaussians, self.output_dim)
        s_diag = self.s(x).reshape(-1, self.num_gaussians, self.output_dim)
        s_diag = self.max_sigma * torch.sigmoid(s_diag)
        s = torch.diag_embed(s_diag)

        w = Categorical(weights)
        c = MultivariateNormal(t, s)
        dist = MixtureSameFamily(w, c)

        log_probs = torch.stack([
            dist.log_prob(t[:, i])
            for i in range(t.shape[1])
        ], dim=1)
        idxs = log_probs.argmax(dim=1)
        y = t[torch.arange(t.shape[0]), idxs]

        return y, s_diag.detach()

    def recon_loss(self, x, y):
        s, t, weights = self.compute_gmm_params(x)
        y = y.unsqueeze(-1).transpose(1, 2).repeat(1, self.num_gaussians, 1)
        sq_diff = (t - y) ** 2
        loss = sq_diff.mean(dim=2).max(dim=1).values.mean()

        return loss
