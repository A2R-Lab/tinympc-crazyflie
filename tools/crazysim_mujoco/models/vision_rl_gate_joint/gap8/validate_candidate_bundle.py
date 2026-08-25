#!/usr/bin/env python3
"""Fail-closed linkage between the runtime bundle and GVSOC checkpoint."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def validate(bundle: Path, checkpoint: Path, expected_checkpoint: str) -> dict:
    bundle = bundle.resolve()
    checkpoint = checkpoint.resolve()
    data = json.loads(bundle.read_text())
    if data.get("format") != "tinympc-joint-gate-obstacle-student-v1":
        raise ValueError("unexpected candidate bundle format")
    if data.get("runtime_adapter") != "joint_gate_rl":
        raise ValueError("candidate bundle is not for joint_gate_rl")
    artifacts = data.get("artifacts", {})
    resolved = {}
    hashes = {}
    for name in ("checkpoint", "policy_onnx"):
        entry = artifacts.get(name, {})
        relative = entry.get("path")
        expected = entry.get("sha256")
        if (not isinstance(relative, str) or Path(relative).name != relative or
                not isinstance(expected, str) or len(expected) != 64):
            raise ValueError(f"invalid {name} artifact declaration")
        path = (bundle.parent / relative).resolve()
        if path.parent != bundle.parent or not path.is_file():
            raise ValueError(f"{name} must be a direct bundle artifact")
        actual = digest(path)
        if actual != expected:
            raise ValueError(f"{name} hash mismatch")
        resolved[name] = path
        hashes[name] = actual
    if checkpoint != resolved["checkpoint"]:
        raise ValueError("GVSOC checkpoint is not the bundle checkpoint")
    if expected_checkpoint != hashes["checkpoint"]:
        raise ValueError("requested checkpoint hash differs from the bundle")
    return {
        "schema": "joint-gvsoc-model-provenance-v1",
        "bundle": str(bundle),
        "bundle_sha256": digest(bundle),
        "checkpoint": str(resolved["checkpoint"]),
        "checkpoint_sha256": hashes["checkpoint"],
        "policy_onnx": str(resolved["policy_onnx"]),
        "policy_onnx_sha256": hashes["policy_onnx"],
        "runtime_adapter": data["runtime_adapter"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-checkpoint-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.bundle, args.checkpoint,
                      args.expected_checkpoint_sha256)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
