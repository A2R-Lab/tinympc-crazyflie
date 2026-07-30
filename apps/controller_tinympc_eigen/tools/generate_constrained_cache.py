#!/usr/bin/env python3
"""Regenerate the fixed-rho Riccati blocks in params_constrained.h.

The firmware ADMM implementation requires Kinf, Pinf, Quu_inv, AmBKt, and
coeff_d2p to match its configured rho.  A scalar-only rho change is invalid.
This script preserves the model and cost matrices already stored in the header
and mechanically replaces only the rho-dependent blocks.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np


APP_DIR = Path(__file__).resolve().parents[1]
DEFAULT_HEADER = APP_DIR / "src" / "params_constrained.h"
SHAPES = {
    "Kinf": (4, 12),
    "Pinf": (12, 12),
    "A": (12, 12),
    "B": (12, 4),
    "Quu_inv": (4, 4),
    "AmBKt": (12, 12),
    "coeff_d2p": (12, 4),
    "Q": (12, 12),
    "R": (4, 4),
}
NUMBER = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def read_matrix(text: str, name: str) -> np.ndarray:
    rows, columns = SHAPES[name]
    match = re.search(rf"\b{name}\s*<<\s*(.*?);", text, re.DOTALL)
    if not match:
        raise ValueError(f"missing matrix assignment for {name}")
    values = [float(value) for value in NUMBER.findall(match.group(1))]
    if len(values) != rows * columns:
        raise ValueError(
            f"{name} contains {len(values)} values, expected {rows * columns}"
        )
    return np.asarray(values, dtype=np.float64).reshape(rows, columns)


def riccati_cache(
    a: np.ndarray,
    b: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    rho: float,
) -> dict[str, np.ndarray]:
    q_augmented = q + rho * np.eye(q.shape[0])
    r_augmented = r + rho * np.eye(r.shape[0])
    gain_previous = np.zeros((r.shape[0], q.shape[0]))
    p_previous = rho * np.eye(q.shape[0])

    for _iteration in range(1000):
        gain = np.linalg.solve(
            r_augmented + b.T @ p_previous @ b,
            b.T @ p_previous @ a,
        )
        p = q_augmented + a.T @ p_previous @ (a - b @ gain)
        if np.max(np.abs(gain - gain_previous)) < 1e-5:
            break
        gain_previous = gain
        p_previous = p
    else:
        raise RuntimeError("Riccati iteration did not converge")

    quu_inverse = np.linalg.inv(r_augmented + b.T @ p @ b)
    closed_loop_transpose = (a - b @ gain).T
    d_to_p = (
        gain.T @ r_augmented
        - closed_loop_transpose @ p @ b
    )
    return {
        "Kinf": gain,
        "Pinf": p,
        "Quu_inv": quu_inverse,
        "AmBKt": closed_loop_transpose,
        "coeff_d2p": d_to_p,
    }


def assignment(name: str, matrix: np.ndarray) -> str:
    rows = []
    for row in matrix:
        rows.append(",".join(f"{value:.7f}f" for value in row))
    return f"{name} << \n" + ",\n".join(rows) + ";\n"


def replace_assignment(text: str, name: str, matrix: np.ndarray) -> str:
    pattern = re.compile(rf"\b{name}\s*<<\s*.*?;\s*", re.DOTALL)
    updated, count = pattern.subn(assignment(name, matrix) + "\n", text, count=1)
    if count != 1:
        raise ValueError(f"could not uniquely replace {name}")
    return updated


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rho", type=float, required=True)
    parser.add_argument("--header", type=Path, default=DEFAULT_HEADER)
    args = parser.parse_args()
    if not np.isfinite(args.rho) or args.rho <= 0:
        parser.error("--rho must be finite and positive")

    text = args.header.read_text()
    cache = riccati_cache(
        read_matrix(text, "A"),
        read_matrix(text, "B"),
        read_matrix(text, "Q"),
        read_matrix(text, "R"),
        args.rho,
    )
    for name, matrix in cache.items():
        text = replace_assignment(text, name, matrix)
    text = re.sub(
        r"\A(?:\/\/ Constrained parameters.*\n)?",
        f"// Constrained 50 Hz parameters; exact fixed-rho cache (rho = {args.rho:g})\n",
        text,
        count=1,
    )
    args.header.write_text(text)
    print(f"wrote rho={args.rho:g} cache to {args.header}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
