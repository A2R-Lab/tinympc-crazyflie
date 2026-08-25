#!/usr/bin/env python3
"""Create the authoritative generated-byte decode contract after NeMO PTQ."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path


def build_quantized_contract(float_contract: dict, terminal_epsilon: float) -> dict:
    if not math.isfinite(terminal_epsilon) or terminal_epsilon <= 0.0:
        raise ValueError("terminal epsilon must be finite and positive")
    result = copy.deepcopy(float_contract)
    result["schema"] = "tinympc-joint-gate-dory-quantized-v1"
    result["output"] = {
        **result["output"],
        "dtype": "uint8_encoded",
        "decode": "raw_logit=(encoded*terminal_epsilon-output_shift)/output_scale; gate_corners=sigmoid(raw[3:11])",
    }
    result["terminal_quantization"] = {
        "encoded_dtype": "uint8",
        "terminal_epsilon": float(terminal_epsilon),
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--float-contract", type=Path, required=True)
    parser.add_argument("--nemo-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = json.loads(args.float_contract.read_text())
    report = json.loads(args.nemo_report.read_text())
    contract = build_quantized_contract(source, float(report["terminal_epsilon"]))
    args.output.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
