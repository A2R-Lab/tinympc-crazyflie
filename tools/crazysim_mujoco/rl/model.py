"""Small temporal image policy and latent transition ensemble."""

from __future__ import annotations

import torch
from torch import nn

from . import ACTION_COUNT


class TemporalPolicy(nn.Module):
    """Deployment graph: two HM01B0 frames to three categorical logits."""

    def __init__(self, latent_dim: int = 7):
        super().__init__()
        self.latent_dim = latent_dim
        self.encoder = nn.Sequential(
            nn.Conv2d(2, 8, 5, stride=3, padding=2), nn.ReLU(),
            nn.Conv2d(8, 16, 5, stride=3, padding=2), nn.ReLU(),
            nn.Conv2d(16, 24, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((3, 3)), nn.Flatten(),
            nn.Linear(24 * 3 * 3, 64), nn.ReLU(),
            nn.Linear(64, latent_dim),
        )
        self.actor = nn.Sequential(
            # ReLU is supported by the NeMO/DORY integer deployment path;
            # Tanh is not lowerable by the local DORY frontend.
            nn.Linear(latent_dim, 32), nn.ReLU(), nn.Linear(32, ACTION_COUNT))

    def encode(self, frames: torch.Tensor) -> torch.Tensor:
        return self.encoder(frames)

    def forward(self, frames: torch.Tensor) -> torch.Tensor:
        return self.actor(self.encode(frames))


class LatentDynamics(nn.Module):
    """Predict normalized latent delta, reward, and termination logit."""

    def __init__(self, latent_dim: int = 7):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(latent_dim + ACTION_COUNT, 64), nn.SiLU(),
            nn.Linear(64, 64), nn.SiLU(),
            nn.Linear(64, latent_dim + 2),
        )

    def forward(self, latent: torch.Tensor, action_one_hot: torch.Tensor):
        prediction = self.net(torch.cat((latent, action_one_hot), dim=-1))
        return prediction[..., :-2], prediction[..., -2], prediction[..., -1]


class QNetwork(nn.Module):
    def __init__(self, latent_dim: int = 7):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(latent_dim, 64), nn.ReLU(), nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, ACTION_COUNT),
        )

    def forward(self, latent: torch.Tensor) -> torch.Tensor:
        return self.net(latent)
