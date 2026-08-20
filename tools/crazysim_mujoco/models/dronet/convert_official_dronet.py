#!/usr/bin/env python3
"""Convert the official 2018 DroNet Keras weights to a portable ONNX graph."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np
import onnx
import torch
from torch import nn


class ResidualBlock(nn.Module):
    def __init__(self, source, prefix: int, in_channels: int, out_channels: int):
        super().__init__()
        self.bn1 = batch_norm(source, f"batch_normalization_{2 * prefix - 1}", in_channels)
        self.conv1 = convolution(source, f"conv2d_{3 * prefix - 1}", in_channels,
                                 out_channels, 3, 2, 1)
        self.bn2 = batch_norm(source, f"batch_normalization_{2 * prefix}", out_channels)
        self.conv2 = convolution(source, f"conv2d_{3 * prefix}", out_channels,
                                 out_channels, 3, 1, 1)
        self.shortcut = convolution(source, f"conv2d_{3 * prefix + 1}", in_channels,
                                    out_channels, 1, 2, 0)

    def forward(self, values):
        main = self.conv1(torch.relu(self.bn1(values)))
        main = self.conv2(torch.relu(self.bn2(main)))
        return main + self.shortcut(values)


def array(source, layer: str, value: str) -> np.ndarray:
    return np.asarray(source[f"{layer}/{layer}/{value}:0"], dtype=np.float32)


def convolution(source, layer, in_channels, out_channels, kernel, stride, padding):
    module = nn.Conv2d(in_channels, out_channels, kernel, stride=stride,
                       padding=padding, bias=True)
    weights = array(source, layer, "kernel").transpose(3, 2, 0, 1)
    module.weight.data.copy_(torch.from_numpy(weights.copy()))
    module.bias.data.copy_(torch.from_numpy(array(source, layer, "bias")))
    return module


def batch_norm(source, layer, channels):
    module = nn.BatchNorm2d(channels, eps=0.001)
    module.weight.data.copy_(torch.from_numpy(array(source, layer, "gamma")))
    module.bias.data.copy_(torch.from_numpy(array(source, layer, "beta")))
    module.running_mean.copy_(torch.from_numpy(array(source, layer, "moving_mean")))
    module.running_var.copy_(torch.from_numpy(array(source, layer, "moving_variance")))
    return module


def linear(source, layer):
    weights = array(source, layer, "kernel")
    module = nn.Linear(weights.shape[0], weights.shape[1])
    module.weight.data.copy_(torch.from_numpy(weights.T.copy()))
    module.bias.data.copy_(torch.from_numpy(array(source, layer, "bias")))
    return module


class Dronet(nn.Module):
    def __init__(self, weights: Path):
        super().__init__()
        with h5py.File(weights, "r") as source:
            self.conv = convolution(source, "conv2d_1", 1, 32, 5, 2, 2)
            self.block1 = ResidualBlock(source, 1, 32, 32)
            self.block2 = ResidualBlock(source, 2, 32, 64)
            self.block3 = ResidualBlock(source, 3, 64, 128)
            self.steering = linear(source, "dense_1")
            self.collision = linear(source, "dense_2")

    def forward(self, image):
        values = self.conv(image)
        values = nn.functional.max_pool2d(values, 3, stride=2)
        values = self.block1(values)
        values = self.block2(values)
        values = self.block3(values)
        # Keras Flatten consumed NHWC memory order.
        values = torch.flatten(values.permute(0, 2, 3, 1), 1)
        values = torch.relu(values)
        return self.steering(values), torch.sigmoid(self.collision(values))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    model = Dronet(args.weights).eval()
    example = torch.zeros(1, 1, 200, 200, dtype=torch.float32)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(model, example, args.out, input_names=["image"],
                      output_names=["steering", "collision_probability"],
                      opset_version=17)
    # Newer PyTorch exporters default to an adjacent external-data file even
    # for this small graph. Re-save it as one portable repository artifact.
    graph = onnx.load(args.out, load_external_data=True)
    onnx.save_model(graph, args.out, save_as_external_data=False)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
