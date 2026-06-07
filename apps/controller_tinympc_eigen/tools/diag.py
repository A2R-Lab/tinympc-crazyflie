#!/usr/bin/env python3
import numpy as np
from scipy.linalg import logm, expm
import importlib.util, sys
spec = importlib.util.spec_from_file_location("rp",
    "/Users/char_chen/School/TinyMPC/tinympc-crazyflie/apps/controller_tinympc_eigen/tools/regen_params.py")
rp = importlib.util.module_from_spec(spec); spec.loader.exec_module(rp)

b = rp.parse_header(rp.HEADER)
A,B,Q,R,Kinf,Pinf,Quu_inv,AmBKt,coeff_d2p = rp.reshape(b)
dt = A[0,6]

# --- diagnose continuous recovery ---
Ac,Bc = rp.recover_continuous(A,B,dt)
print("Bc row norms:")
for i,r in enumerate(np.abs(Bc).max(axis=1)):
    print(f"  row {i:2d}: {r:.3e}")
M = np.zeros((16,16)); M[:12,:12]=A; M[:12,12:]=B; M[12:,12:]=np.eye(4)
L = logm(M).real/dt
print("bottom-left block max (should be ~0):", np.abs(L[12:,:12]).max())
print("bottom-right block max (should be ~0):", np.abs(L[12:,12:]).max())

# --- rho sweep for cache ---
print("\nrho sweep (max err vs file Kinf / Pinf):")
best=None
for rho in [1,5,10,20,40,50,65,72,80,85,90,100,150,200,250,300,500]:
    K,P,Qi,Am,C = rp.compute_cache(A,B,Q,R,rho)
    ek=np.max(np.abs(K-Kinf)); ep=np.max(np.abs(P-Pinf))
    print(f"  rho={rho:4g}  Kinf_err={ek:.3e}  Pinf_err={ep:.3e}")
    if best is None or ek<best[1]: best=(rho,ek,ep)
print("best rho by Kinf:", best)
