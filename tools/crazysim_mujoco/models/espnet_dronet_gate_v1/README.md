# ESPNet DroNet gate v1

This directory vendors the finalized two-frame float ONNX used by CrazySim's
`espnet` vision adapter.

- Source repository: `cookacola/tinympc-perception`
- Source branch: `agent/espnet-dory-deployment`
- Source path: `gap8_perception/releases/espnet_dronet_gate_v1/onnx/espnet_dronet_gate_seed2027_float.onnx`
- SHA-256: `6e9fc06581e29f0681ca63e8630ab21c16c72b9ad9dbfa8646234edee967c7a2`

The input is a float32 NCHW tensor shaped `[batch, 2, 160, 160]`, with the
previous grayscale frame in channel 0 and the current frame in channel 1.
Pixels are scaled to `[0, 1]`.

The model has a gate-detection branch (four corner heatmaps, gate mask, and
gate-presence logit) and a DroNet-compatible navigation branch (yaw and
collision logits). The output contract and temporal frame buffer are enforced
by `vision_bridge.py`.
