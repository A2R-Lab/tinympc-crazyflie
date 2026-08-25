#!/usr/bin/env python3
"""DORY frontend gate for the 12-value joint-policy packed terminal tensor."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from dory.Frontend_frameworks.NEMO.Parser import onnx_manager
from dory.Hardware_targets.PULP.GAP8.HW_Parser import onnx_manager as gap8_backend

from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.dory_wide_bnrelu import (
    install_wide_bnrelu_lowering,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--app-dir", type=Path)
    args = parser.parse_args()
    # The GAP8 backend already supplies matching pulp-nn/64bit kernels.  The
    # local lowering preserves NeMO arithmetic and uses that existing widened
    # coefficient path when a non-power-of-two post-BN multiplier cannot fit
    # legacy int32 k/lambda storage.
    config = {"BNRelu_bits": 64, "input_bits": 8, "input_signed": False}
    wide_requantization = install_wide_bnrelu_lowering(config)
    frontend = onnx_manager(str(args.onnx), config, "gap8_").full_graph_parsing()
    backend_config = dict(config, onnx_file=args.onnx.name, **{"code reserved space": 95000})
    backend = gap8_backend(frontend, backend_config, str(args.onnx.parent), 1).full_graph_parsing()
    l1 = [int(sum(float(node.tiling_dimensions["L1"].get(key, 0) or 0)
                  for key in ("weight_memory", "bias_memory", "constants_memory",
                              "input_activation_memory", "output_activation_memory")))
          for node in backend]
    final = backend[-1]
    output_bytes = int(final.output_activation_memory)
    if int(final.output_activation_bits) != 8 or output_bytes != 12:
        raise RuntimeError("expected uint8[1,12,1,1] terminal, got %d bits / %d bytes" %
                           (int(final.output_activation_bits), output_bytes))
    report = {"passed": True, "single_output_graph": True, "dory_frontend": "NEMO",
              "wide_requantization": wide_requantization,
              "frontend_nodes": len(frontend), "gap8_hardware_nodes": len(backend),
              "gap8_total_macs": int(sum(node.MACs for node in backend)),
              "gap8_max_l1_tile_bytes_estimate": max(l1), "gap8_l1_capacity_bytes": 64000,
              "gap8_final_output_bytes": output_bytes}
    if args.app_dir:
        from network_generate import network_generate
        args.app_dir.mkdir(parents=True, exist_ok=True)
        config_path = args.onnx.parent / "dory_config.json"
        config_path.write_text(json.dumps(dict(backend_config), indent=2) + "\n")
        network_generate("NEMO", "PULP.GAP8", str(config_path),
                         verbose_level="Last+Perf_final", perf_layer="No", optional="8bit",
                         appdir=str(args.app_dir), prefix="gap8")
        report["generated_app_dir"] = str(args.app_dir)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
