# Full PULP-DroNet v3

`pulp_dronet_v3.onnx` is an inference-only ONNX export of the full
PULP-DroNet v3 ResBlock model (`depth_mult=1.0`, bypass enabled). This is the
repository's only bundled DroNet model and is selected by both the `dronet`
and `dronet-v3` runner aliases.

Upstream source: <https://github.com/pulp-platform/pulp-dronet>

- pinned source commit: `98f15cfe2e9c34c3a546358e22e45b3b09f11a79`
- upstream checkpoint: `tiny-pulp-dronet-v3/model/pulp-dronet-v3-resblock-1.0.pth`
- checkpoint SHA-256: `943ab4cbbc28cd711e874559dcb3ba0419a0ae1250dc8c94665f4a46d878ff6e`
- ONNX SHA-256: `dd940ce428e9e10421e81d4591a7804968c8fbe8b09e9fab5c44c97599bcab67`

The input is one grayscale image with shape `[1, 1, 200, 200]`. Match the
published pipeline by taking the centered 200x200 crop of the 324x244 Himax
frame, converting `uint8` pixels to `float32`, and dividing by 255. There is no
mean/std normalization.

The two outputs are:

1. `yaw_rate_normalized`: yaw rate divided by 90 degrees/s;
2. `collision_probability`: sigmoid collision output in `[0, 1]`.

The upstream evaluation code classifies collision at `>= 0.5`.

The model and architecture are published under Apache License 2.0. See
`LICENSE` in this directory.
