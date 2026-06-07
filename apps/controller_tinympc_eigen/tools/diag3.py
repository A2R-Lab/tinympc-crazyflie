#!/usr/bin/env python3
import numpy as np
import importlib.util
spec = importlib.util.spec_from_file_location("rp",
    "/Users/char_chen/School/TinyMPC/tinympc-crazyflie/apps/controller_tinympc_eigen/tools/regen_params.py")
rp = importlib.util.module_from_spec(spec); spec.loader.exec_module(rp)
b = rp.parse_header(rp.HEADER)
A,B,Q,R,Kinf,Pinf,Quu_inv,AmBKt,coeff_d2p = rp.reshape(b)
np.set_printoptions(precision=6, suppress=True, linewidth=160)

K,P,Qi,Am,C = rp.compute_cache(A,B,Q,R,250.0)
Rr = R + 250*np.eye(4); Quu = Rr + B.T@P@B
print("per-row max err of coeff_d2p (mine vs file):")
err = np.abs(C-coeff_d2p)
for i in range(12):
    print(f"  row {i:2d}: {err[i].max():.3e}")
print("\nrow 5 mine:", C[5])
print("row 5 file:", coeff_d2p[5])
print("\nrow 8 mine:", C[8])
print("row 8 file:", coeff_d2p[8])

# try variants
v1 = K.T@Quu - A.T@P@B
v2 = K.T@Rr - A.T@P@B
v3 = -A.T@P@B
v4 = K.T@Quu
for name,V in [("K'Quu-A'PB",v1),("K'Rr-A'PB",v2),("-A'PB",v3),("K'Quu",v4)]:
    print(f"{name:14s} maxerr={np.max(np.abs(V-coeff_d2p)):.3e}")
