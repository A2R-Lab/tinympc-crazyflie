"""Small two-frame shared encoder with the proposed compact ONNX contract."""
from __future__ import annotations

import torch
from torch import nn


class JointTemporalPolicy(nn.Module):
    """160x160 previous/current grayscale frames to TRACK/LEFT/RIGHT logits."""
    def __init__(self, width: int = 12):
        super().__init__()
        self.width = int(width)
        self.encoder = nn.Sequential(
            nn.Conv2d(2, width, 5, stride=3, padding=2), nn.ReLU(),
            nn.Conv2d(width, width * 2, 5, stride=3, padding=2), nn.ReLU(),
            nn.Conv2d(width * 2, width * 3, 3, stride=2, padding=1), nn.ReLU(),
            # Fixed 160x160 input reaches 9x9 here.  A fixed pool avoids the
            # nondeterministic CUDA backward used by adaptive average pooling.
            nn.AvgPool2d(kernel_size=3, stride=3), nn.Flatten(),
            nn.Linear(width * 3 * 9, 48), nn.ReLU())
        self.actor = nn.Linear(48, 3)
        # Gate geometry is a deployed output; risk/value are training-only
        # privileged auxiliaries that shape the same compact visual encoder.
        self.gate_head = nn.Linear(48, 9)  # four normalized corners + confidence
        self.risk_head = nn.Linear(48, 1)
        self.value_head = nn.Linear(48, 1)  # training-only return baseline

    def features(self, frames: torch.Tensor) -> torch.Tensor:
        return self.encoder(frames)

    def forward(self, frames: torch.Tensor):
        features = self.features(frames)
        gate = self.gate_head(features)
        return self.actor(features), torch.sigmoid(gate[:, :8]).reshape(-1, 4, 2), gate[:, 8]

    def supervised(self, frames: torch.Tensor):
        features = self.features(frames)
        gate = self.gate_head(features)
        return (self.actor(features), torch.sigmoid(gate[:, :8]).reshape(-1, 4, 2),
                gate[:, 8], self.risk_head(features), self.value_head(features))
