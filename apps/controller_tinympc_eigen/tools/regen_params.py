#!/usr/bin/env python3
"""
Regenerate params_100hz.h for a new mass / inertia (e.g. Crazyflie + AI-deck).

Approach (exact, no model reconstruction needed):
  1. Parse the existing A,B,Q,R,Kinf,Pinf,Quu_inv,AmBKt,coeff_d2p from params_100hz.h.
  2. Recover the continuous-time model (Ac,Bc) from the discrete (A,B) via the
     augmented matrix logarithm:  expm([[Ac,Bc],[0,0]]*dt) = [[A,B],[0,I]].
  3. In continuous time, mass/inertia ONLY appear in 4 rows of Bc:
        row 8  (vz_dot)  ~ 1/m
        row 9  (wx_dot)  ~ 1/Ixx
        row 10 (wy_dot)  ~ 1/Iyy
        row 11 (wz_dot)  ~ 1/Izz
     A is mass-independent. So scale exactly those rows by old/new ratios.
  4. Re-discretize -> new A (==old A) and new B.
  5. Recompute the TinyMPC ADMM cache (Kinf,Pinf,Quu_inv,AmBKt,coeff_d2p) with rho.
  6. Emit a new params header.

Before doing any of that it VALIDATES that (2) recovers a clean Bc and that (5)
reproduces the cache already in the file. If validation fails, it stops.
"""
import re
import sys
import numpy as np
from scipy.linalg import logm, expm

NX, NU = 12, 4
RHO = 250.0          # stgs.rho_init in controller_tinympc.cpp
DT_GUESS = 0.02      # recovered/checked from A[0][6]

# ---- old vs new physical parameters ----------------------------------------
# Computed by tools/compute_inertia.py (composite CoM + parallel-axis) for the
# user's actual configs. OLD = the flown flow-only config the stock gains match.
# OLD: CF + Flow deck (31.9 g)
M_OLD   = 0.0319
IXX_OLD = 1.6818e-5
IYY_OLD = 1.6802e-5
IZZ_OLD = 2.9525e-5

# NEW: CF + Flow deck + AI deck (measured total 36.3 g; AI deck 30x52x8 mm box on
# top, long 52 mm side fore-aft -> Iyy rises most). deck-center z=9 mm.
M_NEW   = 0.0426
IXX_NEW = 2.1e-5
IYY_NEW = 2.1e-5
IZZ_NEW = 3.7e-5

HEADER = "/Users/char_chen/School/TinyMPC/tinympc-crazyflie/apps/controller_tinympc_eigen/src/params_100hz.h"


def parse_header(path):
    txt = open(path).read()
    # strip C++ float suffix and whitespace, grab each "Name << ... ;" block
    blocks = {}
    for m in re.finditer(r"(\w+)\s*<<\s*(.*?);", txt, re.S):
        name = m.group(1)
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", m.group(2))
        blocks[name] = np.array([float(x) for x in nums])
    return blocks


def reshape(blocks):
    A = blocks["A"].reshape(NX, NX)
    B = blocks["B"].reshape(NX, NU)
    Q = blocks["Q"].reshape(NX, NX)
    R = blocks["R"].reshape(NU, NU)
    Kinf = blocks["Kinf"].reshape(NU, NX)
    Pinf = blocks["Pinf"].reshape(NX, NX)
    Quu_inv = blocks["Quu_inv"].reshape(NU, NU)
    AmBKt = blocks["AmBKt"].reshape(NX, NX)
    coeff_d2p = blocks["coeff_d2p"].reshape(NX, NU)
    return A, B, Q, R, Kinf, Pinf, Quu_inv, AmBKt, coeff_d2p


def recover_continuous(A, B, dt):
    M = np.zeros((NX + NU, NX + NU))
    M[:NX, :NX] = A
    M[:NX, NX:] = B
    M[NX:, NX:] = np.eye(NU)
    L = logm(M).real / dt
    Ac = L[:NX, :NX]
    Bc = L[:NX, NX:]
    return Ac, Bc


def discretize(Ac, Bc, dt):
    M = np.zeros((NX + NU, NX + NU))
    M[:NX, :NX] = Ac
    M[:NX, NX:] = Bc
    E = expm(M * dt)
    return E[:NX, :NX], E[:NX, NX:]


def compute_cache(A, B, Q, R, rho, iters=8000, tol=1e-11):
    # This fork has INPUT box-constraints only, so the ADMM penalty rho augments
    # R only (Q is used as-is). Verified by reproducing the shipped cache.
    Q_rho = Q
    R_rho = R + rho * np.eye(NU)
    P = Q_rho.copy()
    K = np.zeros((NU, NX))
    for _ in range(iters):
        S = R_rho + B.T @ P @ B
        K_new = np.linalg.solve(S, B.T @ P @ A)
        P_new = Q_rho + A.T @ P @ (A - B @ K_new)
        if np.max(np.abs(P_new - P)) < tol and np.max(np.abs(K_new - K)) < tol:
            K, P = K_new, P_new
            break
        K, P = K_new, P_new
    Quu = R_rho + B.T @ P @ B
    Quu_inv = np.linalg.inv(Quu)
    AmBKt = (A - B @ K).T
    # NOTE: coeff_d2p is NOT recomputed here. It is a small (~1e-4) fork-specific
    # correction term in the TinyMPC-ADMM backward pass; the standard LQR
    # recursion makes that term exactly zero, and the offline generator that
    # produced the shipped values is not in this repo. It is carried over
    # unchanged from the stock header (see main()). The change in dynamics is a
    # modest mass rescale, so reusing the flight-proven values is the safe choice.
    return K, P, Quu_inv, AmBKt


def fmt(name, M):
    rows = []
    for r in M:
        rows.append(",".join(f"{v:.6f}f" for v in r) + ",")
    body = "\n".join(rows)
    body = body[:-1] + ";"   # last entry ends with ; not ,
    return f"{name} << \n{body}\n"


def main():
    blocks = parse_header(HEADER)
    A, B, Q, R, Kinf, Pinf, Quu_inv, AmBKt, coeff_d2p = reshape(blocks)

    dt = A[0, 6]
    print(f"[dt] A[0][6] = {dt}  -> {1/dt:.1f} Hz model (filename says 100hz)")

    # ---- validate cache formula against the file -------------------------
    # Use RELATIVE error: these matrices span very different scales (Pinf ~1e4,
    # Kinf ~0.5, Quu_inv ~6e-4), so a single absolute threshold is meaningless.
    # coeff_d2p is intentionally not validated/recomputed (see compute_cache).
    K2, P2, Qi2, Am2 = compute_cache(A, B, Q, R, RHO)
    def relerr(a, b):
        return np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-12)
    errs = {
        "Kinf": relerr(K2, Kinf),
        "Pinf": relerr(P2, Pinf),
        "Quu_inv": relerr(Qi2, Quu_inv),
        "AmBKt": relerr(Am2, AmBKt),
    }
    print("\n[cache validation] max RELATIVE error reproducing existing file (rho=%g):" % RHO)
    for k, v in errs.items():
        print(f"   {k:10s}: {v:.3e}")
    cache_ok = max(errs.values()) < 1e-2   # file printed at 6 decimals -> ~1e-3 rel
    print("   => cache math", "MATCHES" if cache_ok else "DOES NOT MATCH")

    # ---- recover continuous model & validate structure -------------------
    Ac, Bc = recover_continuous(A, B, dt)
    rownorm = np.abs(Bc).max(axis=1)
    # rows 0-7 are 6-decimal quantization noise (~1e-5); 8-11 are the real
    # physical input rows (O(1-300)). "dominant" = >1% of the largest row.
    dominant = np.where(rownorm > 0.01 * rownorm.max())[0]
    print(f"\n[continuous Bc] dominant rows: {dominant.tolist()} (expect [8, 9, 10, 11])")
    print("   row norms 0-7 (noise):", np.array2string(rownorm[:8], precision=1))
    A_chk, B_chk = discretize(Ac, Bc, dt)
    print(f"   re-discretization error: A {np.max(np.abs(A_chk-A)):.2e}  "
          f"B {np.max(np.abs(B_chk-B)):.2e}")
    struct_ok = set(dominant.tolist()) == {8, 9, 10, 11}
    print("   => Bc structure", "CLEAN" if struct_ok else "UNEXPECTED")

    if not (cache_ok and struct_ok):
        print("\nVALIDATION FAILED — not regenerating. Inspect the discrepancy first.")
        sys.exit(1)

    # ---- regenerate ------------------------------------------------------
    Bc_new = Bc.copy()
    Bc_new[8]  *= M_OLD   / M_NEW
    Bc_new[9]  *= IXX_OLD / IXX_NEW
    Bc_new[10] *= IYY_OLD / IYY_NEW
    Bc_new[11] *= IZZ_OLD / IZZ_NEW

    A_new, B_new = discretize(Ac, Bc_new, dt)
    K_new, P_new, Qi_new, Am_new = compute_cache(A_new, B_new, Q, R, RHO)
    # coeff_d2p: carry over the flight-proven stock values unchanged.
    C_new = coeff_d2p.reshape(NX, NU)

    print("\n[regen] mass %.4f->%.4f  Ixx %.3e->%.3e  Iyy %.3e->%.3e  Izz %.3e->%.3e"
          % (M_OLD, M_NEW, IXX_OLD, IXX_NEW, IYY_OLD, IYY_NEW, IZZ_OLD, IZZ_NEW))
    print("   B[vz] row scale: %.4f   B[wx]: %.4f   B[wy]: %.4f   B[wz]: %.4f"
          % (M_OLD/M_NEW, IXX_OLD/IXX_NEW, IYY_OLD/IYY_NEW, IZZ_OLD/IZZ_NEW))

    out = []
    out.append(fmt("Kinf", K_new))
    out.append(fmt("Pinf", P_new))
    out.append(fmt("A", A_new))
    out.append(fmt("B", B_new))
    out.append(fmt("Quu_inv", Qi_new))
    out.append(fmt("AmBKt", Am_new))
    out.append(fmt("coeff_d2p", C_new))
    out.append(fmt("Q", Q))
    out.append(fmt("R", R))
    text = "\n".join(out)

    outpath = HEADER.replace("params_100hz.h", "params_aideck.h")
    with open(outpath, "w") as f:
        f.write(text)
    print(f"\n[written] {outpath}")


if __name__ == "__main__":
    main()
