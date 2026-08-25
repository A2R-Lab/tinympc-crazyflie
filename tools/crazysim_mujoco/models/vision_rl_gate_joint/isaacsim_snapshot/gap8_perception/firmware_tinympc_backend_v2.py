"""ctypes backend for the exact generated TinyMPC firmware specialization."""
from __future__ import annotations

import ctypes
import hashlib
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from gap8_perception.tinympc_execution_v2 import MODEL_DT_S, TinyMpcSolution


REQUIRED_MACROS = {
    "TINYMPC_GENERATED_STATE_DIM": "12",
    "TINYMPC_GENERATED_INPUT_DIM": "4",
    "TINYMPC_GENERATED_HORIZON_KNOTS": "20",
    "TINYMPC_GENERATED_SOLVE_RATE_HZ": "50",
    "TINYMPC_GENERATED_IS_BRUSHLESS": "1",
    "TINYMPC_GENERATED_HAS_AIDECK": "1",
    "TINYMPC_GENERATED_HAS_FLOWDECK": "1",
    "TINYMPC_GENERATED_HAS_PROPELLER_GUARDS": "1",
}


def _controller_commit(repository: Path) -> str:
    return subprocess.run(
        ("git", "rev-parse", "HEAD"), cwd=repository, check=True,
        text=True, stdout=subprocess.PIPE,
    ).stdout.strip()


def _source_paths(repository: Path) -> tuple[Path, list[Path], Path]:
    app = repository / "apps/controller_tinympc_eigen"
    source = app / "tools/pybullet_simulation/tinympc_admm_host.cpp"
    solver = app / "TinyMPC-ADMM/src/tinympc"
    sources = [source] + [solver / name for name in (
        "model.cpp", "auxil.cpp", "cost_lqr.cpp", "lqr.cpp", "constraint_linear.cpp",
        "admm.cpp", "rho_benchmark.cpp", "utils.cpp",
    )]
    return app, sources, app / "src/tinympc_generated_params.h"


def validate_generated_specialization(header: Path) -> None:
    content = header.read_text()
    for macro, expected in REQUIRED_MACROS.items():
        match = re.search(rf"^#define\s+{macro}\s+([^\s]+)", content, flags=re.MULTILINE)
        if match is None or match.group(1) != expected:
            raise ValueError(f"generated firmware specialization requires {macro}={expected}")
    timestep = re.search(
        r"^#define\s+TINYMPC_GENERATED_MODEL_DT_S\s+\(\(float\)([0-9.eE+-]+)\)",
        content,
        flags=re.MULTILINE,
    )
    if timestep is None:
        raise ValueError("generated firmware specialization has no model timestep")
    base_dt_s = float(timestep.group(1))
    compositions = round(MODEL_DT_S / base_dt_s) if base_dt_s > 0.0 else 0
    if (compositions < 1 or
            abs(MODEL_DT_S - compositions * base_dt_s) > 1.0e-6):
        raise ValueError(
            "generated firmware base timestep must compose exactly to the "
            f"{MODEL_DT_S:g} s prediction knot"
        )


def build_firmware_host_library(repository: str | Path) -> Path:
    repository = Path(repository).resolve()
    app, sources, header = _source_paths(repository)
    validate_generated_specialization(header)
    digest = hashlib.sha256()
    for path in [*sources, header]:
        digest.update(path.read_bytes())
    output = Path(tempfile.gettempdir()) / "horizon_tinympc_v2" / f"libtinympc_{digest.hexdigest()[:16]}.so"
    if output.exists():
        return output
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "g++", "-std=c++14", "-O2", "-shared", "-fPIC", "-DNHORIZON=20",
        f"-I{app / 'TinyMPC-ADMM/src'}", f"-I{app / 'TinyMPC-ADMM/ext/Eigen'}",
        *map(str, sources), "-o", str(output),
    ]
    subprocess.run(command, cwd=app, check=True)
    return output


class FirmwareGeneratedTinyMpcBackend:
    """Cold-start isolated solve using the exact generated embedded arithmetic."""

    backend_id = "tinympc_generated_firmware_sil_v2"
    firmware_equivalent = True

    def __init__(self, repository: str | Path) -> None:
        self.repository = Path(repository).resolve()
        self.controller_commit = _controller_commit(self.repository)
        self.library_path = build_firmware_host_library(self.repository)
        self._library = ctypes.CDLL(str(self.library_path))
        self._configure()
        if not self._library.tinympc_admm_host_init(
            5, ctypes.c_float(250.0), False, ctypes.c_float(MODEL_DT_S)
        ):
            raise RuntimeError("exact generated TinyMPC host initialization failed")

    def _configure(self) -> None:
        f32 = ctypes.POINTER(ctypes.c_float)
        i32 = ctypes.POINTER(ctypes.c_int)
        library = self._library
        library.tinympc_admm_host_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_bool, ctypes.c_float]
        library.tinympc_admm_host_init.restype = ctypes.c_bool
        library.tinympc_admm_host_reset_duals.argtypes = []
        library.tinympc_admm_host_reset_duals.restype = None
        library.tinympc_admm_host_solve.argtypes = [
            f32, f32, f32, f32, f32, i32, ctypes.c_int, f32, f32,
            i32, i32, f32, f32,
        ]
        library.tinympc_admm_host_solve.restype = ctypes.c_bool

    def solve(
        self,
        initial_state: np.ndarray,
        reference: np.ndarray,
        *,
        halfspace_a: np.ndarray | None = None,
        halfspace_b: np.ndarray | None = None,
        halfspace_enabled: np.ndarray | None = None,
    ) -> TinyMpcSolution:
        # Reinitialize the complete static host workspace, not only ADMM duals.
        # The firmware host keeps primal trajectories in global storage; merely
        # zeroing duals makes a nominally cold solve depend on the prior call.
        if not self._library.tinympc_admm_host_init(
            5, ctypes.c_float(250.0), False, ctypes.c_float(MODEL_DT_S)
        ):
            raise RuntimeError("exact generated TinyMPC host reinitialization failed")
        x0 = np.ascontiguousarray(initial_state, dtype=np.float32)
        xref = np.ascontiguousarray(reference, dtype=np.float32)
        if x0.shape != (12,) or xref.shape != (20, 12):
            raise ValueError("firmware backend requires x0 [12] and reference [20,12]")
        uref = np.zeros((19, 4), np.float32)
        supplied = (halfspace_a is not None, halfspace_b is not None, halfspace_enabled is not None)
        if any(supplied) and not all(supplied):
            raise ValueError("halfspace_a, halfspace_b, and halfspace_enabled must be supplied together")
        if all(supplied):
            halfspace_a = np.ascontiguousarray(halfspace_a, dtype=np.float32)
            halfspace_b = np.ascontiguousarray(halfspace_b, dtype=np.float32)
            halfspace_enabled = np.ascontiguousarray(halfspace_enabled, dtype=np.int32)
            if (halfspace_a.shape, halfspace_b.shape, halfspace_enabled.shape) != (
                (20, 1, 3), (20, 1), (20, 1)
            ):
                raise ValueError("firmware half-spaces require [20,1,3], [20,1], and [20,1]")
            if not np.isfinite(halfspace_a).all() or not np.isfinite(halfspace_b).all():
                raise ValueError("firmware half-spaces must be finite")
        else:
            halfspace_a = np.zeros((20, 1, 3), np.float32)
            halfspace_b = np.zeros((20, 1), np.float32)
            halfspace_enabled = np.zeros((20, 1), np.int32)
        states = np.empty((20, 12), np.float32)
        controls = np.empty((19, 4), np.float32)
        status, iterations = ctypes.c_int(), ctypes.c_int()
        primal, dual = ctypes.c_float(), ctypes.c_float()
        f32 = ctypes.POINTER(ctypes.c_float)
        i32 = ctypes.POINTER(ctypes.c_int)
        # Each counterfactual gets an isolated cold-start workspace. Deployment
        # must benchmark/freeze this schedule or replace it with isolated warm starts.
        self._library.tinympc_admm_host_reset_duals()
        success = bool(self._library.tinympc_admm_host_solve(
            x0.ctypes.data_as(f32), xref.ctypes.data_as(f32), uref.ctypes.data_as(f32),
            halfspace_a.ctypes.data_as(f32), halfspace_b.ctypes.data_as(f32),
            halfspace_enabled.ctypes.data_as(i32), int(np.any(halfspace_enabled)),
            states.ctypes.data_as(f32),
            controls.ctypes.data_as(f32), ctypes.byref(status), ctypes.byref(iterations),
            ctypes.byref(primal), ctypes.byref(dual),
        ))
        residuals_finite = np.isfinite((primal.value, dual.value)).all()
        enabled = halfspace_enabled.astype(bool)
        if enabled.any():
            position = states[:, None, :3]
            violation = np.einsum("khj,khj->kh", halfspace_a, position) - halfspace_b
            maximum_halfspace_violation = float(violation[enabled].max())
        else:
            maximum_halfspace_violation = 0.0
        success = (
            success and np.isfinite(states).all() and np.isfinite(controls).all()
            and residuals_finite and primal.value <= 5.0e-2 and dual.value <= 5.0e-2
            and float(np.abs(states).max()) < 1.0e3
            and maximum_halfspace_violation <= 1.0e-3
        )
        return TinyMpcSolution(
            states=states, control_deviation_n=controls, success=success,
            status=status.value, iterations=iterations.value,
            primal_residual=primal.value, dual_residual=dual.value,
        )
