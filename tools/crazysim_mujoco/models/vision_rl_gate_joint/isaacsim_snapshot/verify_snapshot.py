#!/usr/bin/env python3
"""Fail closed if the v3 source snapshot or pinned main inputs drift."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(base: Path, expected: dict[str, str], label: str) -> list[str]:
    errors = []
    for relative, digest in expected.items():
        path = base / relative
        if not path.is_file():
            errors.append(f"missing {label}: {relative}")
        elif sha256(path) != digest:
            errors.append(f"hash drift in {label}: {relative}")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-external-root", type=Path,
                        help="optional IsaacSim overlay root; verifies external course textures too")
    args = parser.parse_args()
    manifest = json.loads((HERE / "dataset_v3_source_manifest.json").read_text())
    errors = verify(HERE, manifest["snapshot_files_sha256"], "snapshotted source")
    errors += verify(REPO, manifest["main_repo_dependencies_sha256"], "main-repo dependency")
    if args.check_external_root is not None:
        errors += verify(args.check_external_root, manifest["optional_external_course_textures_sha256"],
                         "optional external texture")
    if errors:
        raise SystemExit("\n".join(errors))
    print("v3 IsaacSim snapshot hashes verified")


if __name__ == "__main__":
    main()
