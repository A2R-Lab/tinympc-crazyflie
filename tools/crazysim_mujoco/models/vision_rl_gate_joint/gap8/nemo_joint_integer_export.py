#!/usr/bin/env python
"""NEMO PTQ export for the packed joint temporal DORY graph.

Run this with the repository's Python-3.7 frontnet environment.  It writes the
NEMO integer ONNX plus DORY golden activations, then measures the requested
200-clip semantic parity before any GVSOC claim is made.
"""
from __future__ import print_function

import argparse
import json
from pathlib import Path

import nemo
import numpy as np
import onnx
import torch

from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.deploy_model import JointDoryPackedNet
from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.export_contract import signed_int8_weight_ranges
from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.quantized_contract import build_quantized_contract


def sigmoid(values):
    return 1.0 / (1.0 + np.exp(-np.clip(values, -30.0, 30.0)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True,
                        help="nemo_bridge_state.npz emitted by the modern exporter")
    parser.add_argument("--float-export", type=Path, required=True,
                        help="Directory written by export_joint_dory.py")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration-clips", type=int, default=200)
    parser.add_argument("--requantization-factor", type=int, default=32)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    if hasattr(torch.backends, "mkldnn"):
        torch.backends.mkldnn.enabled = False
    if torch.cuda.is_available():
        raise RuntimeError("NEMO export must run CPU-only under Slurm")
    corpus = np.load(str(args.float_export / "calibration_200_clips.npz"), allow_pickle=False)
    frames = np.asarray(corpus["frames"])
    reference = np.asarray(corpus["reference_logits"])
    if frames.shape != (200, 2, 160, 160) or frames.dtype != np.uint8:
        raise RuntimeError("expected sealed uint8[200,2,160,160] parity corpus")
    bridge = np.load(str(args.bridge), allow_pickle=False)
    width = int(np.asarray(bridge["width"]).item())
    state = {key: torch.from_numpy(np.asarray(bridge[key])) for key in bridge.files if key != "width"}
    model = JointDoryPackedNet(width).eval()
    model.load_joint_state_dict(state, width)
    contract = json.loads((args.float_export / "deployment_contract.json").read_text())
    requested_scale = float(np.asarray(contract["terminal_affine"]["output_scale"])[0])
    if not np.allclose(np.asarray(contract["terminal_affine"]["output_scale"]), requested_scale):
        raise RuntimeError("only one shared terminal scale is supported")
    provenance = contract.get("provenance", {})
    requested_margin = float(provenance.get("terminal_margin", 0.25))
    model.set_terminal_affine_from_logits(torch.from_numpy(frames), scale=requested_scale,
                                          margin=requested_margin)
    expected = np.asarray(contract["terminal_affine"]["output_shift"], dtype=np.float32)
    if not np.allclose(expected, model.output_shift.detach().cpu().numpy(), atol=1.0e-6):
        raise RuntimeError("terminal affine is not reproducible from sealed corpus")
    model = nemo.transform.quantize_pact(model, dummy_input=torch.ones(1, 2, 160, 160))
    model.change_precision(bits=8)
    model.reset_alpha_weights()
    model.set_statistics_act()
    with torch.no_grad():
        for start in range(0, len(frames), min(args.calibration_clips, 32)):
            image = torch.from_numpy(frames[start:start + 32]).float() / 255.0
            model(image)
    model.unset_statistics_act()
    model.reset_alpha_act()
    for module in model.modules():
        if hasattr(module, "requantization_factor"):
            module.requantization_factor = args.requantization_factor
    model.qd_stage(eps_in=1.0 / 255.0)
    model.id_stage()
    # GAP8 pulp-nn consumes convolution weights as signed int8.  NeMO's
    # nominal 8-bit PACT integerization can emit a few coefficients outside
    # [-128,127]; DORY previously cast those modulo 256.  Saturate the
    # integerized weights before both golden inference and ONNX export so the
    # deployment graph, checksums, and kernel bytes describe one model.
    saturated_weight_count = 0
    pact_convolution_count = 0
    for module in model.modules():
        if module.__class__.__name__ != "PACT_Conv2d":
            continue
        pact_convolution_count += 1
        saturated_weight_count += int(torch.sum(
            ((module.weight.data < -128) | (module.weight.data > 127)).long()).item())
        module.weight.data.clamp_(-128, 127)
    if pact_convolution_count == 0:
        raise RuntimeError("integerized NeMO model contains no PACT_Conv2d layers")
    hooks = []
    golden = []

    def capture(_module, _inputs, output):
        golden.append(output.detach().cpu())

    for module in model.modules():
        if module.__class__.__name__ in ("PACT_Act", "PACT_IntegerAct", "PACT_IntegerAvgPool2d"):
            hooks.append(module.register_forward_hook(capture))
    with torch.no_grad():
        integer = model(torch.from_numpy(frames).float()).detach().cpu().numpy()
    for hook in hooks:
        hook.remove()
    if integer.shape != (200, 12, 1, 1):
        raise RuntimeError("NEMO integer output shape changed: %r" % (integer.shape,))
    if not np.any(integer):
        raise RuntimeError("refusing degenerate all-zero NEMO terminal output")
    for index, activation in enumerate(golden):
        values = activation[0]
        if values.ndim == 3:
            values = values.permute(1, 2, 0)
        np.savetxt(str(args.output / ("out_layer%d.txt" % index)), values.numpy().flatten(),
                   header="NEMO integer activation shape %s" % list(values.shape),
                   fmt="%.3f", delimiter=",", newline=",\n")
    np.savetxt(str(args.output / "input.txt"), frames[0].transpose(1, 2, 0).flatten(),
               header="two temporal uint8 frames HWC channels=[previous,current]",
               fmt="%d", delimiter=",", newline=",\n")
    onnx_path = args.output / "joint_dory_packed_int.onnx"
    nemo.utils.export_onnx(str(onnx_path), model, model, (2, 160, 160), perm=None)
    graph = onnx.load(str(onnx_path))
    weight_ranges = signed_int8_weight_ranges(graph)
    terminal_quantizer = model.terminal[-1]
    epsilon = float(terminal_quantizer.eps_out)
    scale = np.asarray(contract["terminal_affine"]["output_scale"], dtype=np.float32)
    shift = np.asarray(contract["terminal_affine"]["output_shift"], dtype=np.float32)
    decoded = (integer[:, :, 0, 0] * epsilon - shift[None, :]) / scale[None, :]
    reference_actions = reference[:, :3].argmax(1)
    integer_actions = decoded[:, :3].argmax(1)
    agreement = float((reference_actions == integer_actions).mean())
    visible = np.asarray(corpus["gate_visible"]).astype(bool)
    reference_corners = sigmoid(reference[:, 3:11]).reshape(-1, 4, 2) * 160.0
    integer_corners = sigmoid(decoded[:, 3:11]).reshape(-1, 4, 2) * 160.0
    reference_center = reference_corners.mean(1)
    integer_center = integer_corners.mean(1)
    gate_center_increase = (float(np.mean(np.linalg.norm(integer_center[visible] - np.asarray(corpus["gate_corners"])[visible].mean(1) * 160.0, axis=1))) -
                            float(np.mean(np.linalg.norm(reference_center[visible] - np.asarray(corpus["gate_corners"])[visible].mean(1) * 160.0, axis=1)))) if visible.any() else 0.0
    report = {"schema": "joint-dory-nemo-integer-v1", "clips": 200,
              "nemo_integer_onnx": str(onnx_path),
              "operators": sorted(set(node.op_type for node in graph.graph.node)),
              "golden_activation_layers": len(golden), "terminal_epsilon": epsilon,
              "activation_precision_bits": 8, "weight_precision_bits": 8,
              "weight_storage_contract": "signed-int8 saturating export",
              "saturated_weight_count": saturated_weight_count,
              "signed_int8_weight_ranges": weight_ranges,
              "terminal_scale": requested_scale, "terminal_margin": requested_margin,
              "integer_terminal_output_min": int(integer.min()),
              "integer_terminal_output_max": int(integer.max()),
              "action_agreement": agreement, "action_agreement_requirement": 0.95,
              "visible_gate_center_error_increase_px": gate_center_increase,
              "visible_gate_center_error_increase_requirement_px": 5.0,
              "passed_semantic_parity": agreement >= 0.95 and gate_center_increase <= 5.0,
              "gvsoc": "not yet run; this report is pre-GVSOC NEMO integer parity"}
    (args.output / "nemo_joint_report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    quantized_contract = build_quantized_contract(contract, epsilon)
    (args.output / "quantized_deployment_contract.json").write_text(
        json.dumps(quantized_contract, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
