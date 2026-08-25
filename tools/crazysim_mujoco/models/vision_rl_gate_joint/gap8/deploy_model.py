"""Convolution-only, single-output re-expression of ``JointTemporalPolicy``.

This module does not train a second policy.  It maps an existing compact
checkpoint into an algebraically equivalent Conv/BN/ReLU graph accepted by the
local NEMO/DORY frontend.  The DORY output is twelve encoded scalars:
``[TRACK, LEFT, RIGHT, TLx, TLy, TRx, TRy, BRx, BRy, BLx, BLy, gate_logit]``.
The affine encoding prevents DORY's uint8 terminal ReLU from discarding signed
logits; decoding and corner sigmoid remain explicit firmware-side operations.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


OUTPUT_LAYOUT = [
    "action_track_logit", "action_left_logit", "action_right_logit",
    "gate_tl_x_logit", "gate_tl_y_logit", "gate_tr_x_logit",
    "gate_tr_y_logit", "gate_br_x_logit", "gate_br_y_logit",
    "gate_bl_x_logit", "gate_bl_y_logit", "gate_confidence_logit",
]


class ConvBNReLU(nn.Sequential):
    """DORY/NEMO-compatible fused layer; BN holds a source Conv bias."""
    def __init__(self, cin: int, cout: int, kernel: int, stride: int = 1,
                 groups: int = 1, padding: int | None = None) -> None:
        if padding is None:
            padding = kernel // 2
        super().__init__(
            nn.Conv2d(cin, cout, kernel, stride=stride, padding=padding,
                      groups=groups, bias=False),
            nn.BatchNorm2d(cout), nn.ReLU(inplace=False),
        )


class JointDoryPackedNet(nn.Module):
    """Exact 5913 inference graph, expressed using Conv/BN/ReLU only."""
    def __init__(self, width: int) -> None:
        super().__init__()
        self.width = int(width)
        self.conv0 = ConvBNReLU(2, width, 5, stride=3)
        self.conv1 = ConvBNReLU(width, width * 2, 5, stride=3)
        self.conv2 = ConvBNReLU(width * 2, width * 3, 3, stride=2)
        # NEMO has an integer AvgPool representation. Sparse depthwise 1/9
        # weights instead quantize to zero in its legacy per-tensor PTQ path.
        self.average3 = nn.AvgPool2d(kernel_size=3, stride=3)
        # Linear(36*3*3, 48) is exactly Conv2d(36, 48, 3) on its 3x3 input.
        self.feature = ConvBNReLU(width * 3, 48, 3, stride=1, padding=0)
        # The packed terminal projection has no branch/Concat/Slice operation.
        self.terminal = ConvBNReLU(48, len(OUTPUT_LAYOUT), 1)
        self.register_buffer("output_scale", torch.ones(len(OUTPUT_LAYOUT)))
        self.register_buffer("output_shift", torch.zeros(len(OUTPUT_LAYOUT)))

    def forward(self, frames: torch.Tensor) -> torch.Tensor:
        x = self.conv0(frames)
        x = self.conv1(x)
        x = self.conv2(x)
        x = self.average3(x)
        x = self.feature(x)
        return self.terminal(x)

    def pre_terminal_logits(self, frames: torch.Tensor) -> torch.Tensor:
        """Terminal Conv/BN value before DORY's required output ReLU."""
        x = self.conv0(frames)
        x = self.conv1(x)
        x = self.conv2(x)
        x = self.average3(x)
        x = self.feature(x)
        conv, bn, _relu = self.terminal
        return bn(conv(x))[:, :, 0, 0]

    def decode(self, encoded: torch.Tensor) -> torch.Tensor:
        """Decode DORY's positive terminal tensor to original raw logits."""
        return ((encoded[:, :, 0, 0] - self.output_shift) / self.output_scale)

    @staticmethod
    def _load_conv_bn(layer: ConvBNReLU, weight: torch.Tensor,
                      bias: torch.Tensor | None, multiplier: torch.Tensor | None = None,
                      offset: torch.Tensor | None = None) -> None:
        """Set Conv+BN to ``multiplier * (Conv(weight)+bias) + offset``."""
        conv, bn, _relu = layer
        if multiplier is None:
            multiplier = torch.ones(conv.out_channels, dtype=weight.dtype)
        if offset is None:
            offset = torch.zeros(conv.out_channels, dtype=weight.dtype)
        if bias is None:
            bias = torch.zeros(conv.out_channels, dtype=weight.dtype)
        conv.weight.data.copy_(weight)
        bn.running_mean.zero_()
        bn.running_var.fill_(1.0)
        # gamma/sqrt(1+eps) = multiplier.
        bn.weight.data.copy_(multiplier * np.sqrt(1.0 + bn.eps))
        bn.bias.data.copy_(multiplier * bias + offset)

    def load_joint_checkpoint(self, checkpoint: Path) -> dict[str, Any]:
        source = torch.load(checkpoint, map_location="cpu", weights_only=False)
        return self.load_joint_state_dict(source["model"], int(source["width"]), source.get("config", {}))

    def load_joint_state_dict(self, state: dict[str, torch.Tensor], width: int,
                              config: dict[str, Any] | None = None) -> dict[str, Any]:
        if width != self.width:
            raise ValueError("checkpoint width does not match deployment model")
        self._load_conv_bn(self.conv0, state["encoder.0.weight"], state["encoder.0.bias"])
        self._load_conv_bn(self.conv1, state["encoder.2.weight"], state["encoder.2.bias"])
        self._load_conv_bn(self.conv2, state["encoder.4.weight"], state["encoder.4.bias"])
        linear = state["encoder.8.weight"].reshape(48, width * 3, 3, 3)
        self._load_conv_bn(self.feature, linear, state["encoder.8.bias"])
        packed_weight = torch.cat((state["actor.weight"], state["gate_head.weight"]), 0)
        packed_bias = torch.cat((state["actor.bias"], state["gate_head.bias"]), 0)
        self._load_conv_bn(self.terminal, packed_weight[:, :, None, None], packed_bias)
        return {"width": width, "config": {} if config is None else config}

    def set_terminal_affine_from_logits(self, frames: torch.Tensor,
                                        margin: float = 0.25,
                                        scale: float = 1.0) -> None:
        """Choose a positive affine envelope from representative float clips."""
        if frames.ndim != 4 or frames.shape[1:] != (2, 160, 160):
            raise ValueError("expected uint8/float clips [N,2,160,160]")
        if scale <= 0.0 or abs(np.log2(scale) - round(np.log2(scale))) > 1.0e-9:
            raise ValueError("terminal scale must be a positive power of two")
        if margin <= 0.0:
            raise ValueError("terminal margin must be positive")
        was_training = self.training
        self.eval()
        with torch.no_grad():
            # Before the affine is applied terminal BN is an exact raw head.
            raw = self.pre_terminal_logits(
                frames.float() / (255.0 if frames.max() > 1 else 1.0)
            )
        minimum = raw.min(0).values
        # Stable unit scale avoids changing original logits before NEMO QAT/PTQ.
        with torch.no_grad():
            self.output_scale.copy_(torch.full_like(minimum, float(scale)))
            self.output_shift.copy_((-self.output_scale * minimum + float(margin)).clamp_min(float(margin)))
            terminal = self.terminal
            source_weight = terminal[1].weight.detach().clone()
            source_bias = terminal[1].bias.detach().clone()
            # Current BN implements raw Conv+bias.  Make it raw + per-channel shift.
            terminal[1].weight.copy_(source_weight * self.output_scale)
            terminal[1].bias.copy_(source_bias * self.output_scale + self.output_shift)
        if was_training:
            self.train()

    def contract(self) -> dict[str, Any]:
        return {
            "schema": "tinympc-joint-gate-dory-packed-v1",
            "input": {"shape": [1, 2, 160, 160], "dtype": "uint8",
                      "normalization": "frames / 255", "temporal_order": ["previous", "current"]},
            "output": {"shape": [1, 12, 1, 1], "dtype": "float32_affine_pre_quantization",
                       "layout": OUTPUT_LAYOUT,
                       "decode": "raw_logit=(affine_output-output_shift)/output_scale; gate_corners=sigmoid(raw[3:11])",
                       "quantized_contract": "written after NeMO determines terminal epsilon"},
            "actions": ["TRACK", "LEFT", "RIGHT"],
        }


def write_contract(path: Path, model: JointDoryPackedNet, provenance: dict[str, Any]) -> None:
    payload = model.contract()
    payload["terminal_affine"] = {"output_scale": model.output_scale.detach().cpu().tolist(),
                                   "output_shift": model.output_shift.detach().cpu().tolist()}
    payload["provenance"] = provenance
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
