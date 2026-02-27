import torch
from torch.nn.modules.loss import _Loss


class GeneralLoss(_Loss):

    def __init__(self, alpha):
        super().__init__()
        self.alpha = alpha

    def forward(self, pred, gt):
        res = pred - gt
        if self.alpha == 2:
            return (res ** 2).mean()
        else:
            d = abs(2 - self.alpha)
            c = d / self.alpha
            p = self.alpha / 2
            return c * ((res ** 2 / d + 1) ** p - 1)


class PsuedoHuberLoss(GeneralLoss):

    def __init__(self):
        super().__init__(1)


class CauchyLoss(_Loss):

    def __init__(self, scale=1.0):
        super().__init__()
        self.scale = scale

    def forward(self, pred, gt):
        res = pred - gt
        loss = torch.log(0.5 * (res / self.scale) ** 2 + 1).mean()
        return loss


class GemanMcClureLoss(GeneralLoss):

    def __init__(self):
        super().__init__(-2.0)
