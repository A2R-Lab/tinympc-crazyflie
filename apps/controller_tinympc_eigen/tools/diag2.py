#!/usr/bin/env python3
import numpy as np
import importlib.util
spec = importlib.util.spec_from_file_location("rp",
    "/Users/char_chen/School/TinyMPC/tinympc-crazyflie/apps/controller_tinympc_eigen/tools/regen_params.py")
rp = importlib.util.module_from_spec(spec); spec.loader.exec_module(rp)

b = rp.parse_header(rp.HEADER)
A,B,Q,R,Kinf,Pinf,Quu_inv,AmBKt,coeff_d2p = rp.reshape(b)
dt0 = A[0,6]
Ac,Bc = rp.recover_continuous(A,B,dt0)

def cache_variant(A,B,Q,R,rho,addQ=True,addR=True,iters=8000,tol=1e-11):
    Qr = Q + (rho*np.eye(12) if addQ else 0)
    Rr = R + (rho*np.eye(4) if addR else 0)
    P=Qr.copy(); K=np.zeros((4,12))
    for _ in range(iters):
        S=Rr+B.T@P@B
        Kn=np.linalg.solve(S,B.T@P@A)
        Pn=Qr+A.T@P@(A-B@Kn)
        if np.max(np.abs(Pn-P))<tol: K,P=Kn,Pn; break
        K,P=Kn,Pn
    return K,P

print("Search dt x rho x augmentation for best Kinf match (err on Kinf):")
best=(1e9,None)
for dt in [0.002,0.0025,0.004,0.005,0.008,0.01,0.0125,0.02]:
    Ad,Bd = rp.discretize(Ac,Bc,dt)
    for addQ,addR in [(True,True),(False,True),(True,False),(False,False)]:
        for rho in [0,1,5,20,50,72,85,100,250]:
            K,P = cache_variant(Ad,Bd,Q,R,rho,addQ,addR)
            ek=np.max(np.abs(K-Kinf))
            if ek<best[0]: best=(ek,(dt,rho,addQ,addR))
print("BEST:",best)
dt,rho,addQ,addR = best[1]
Ad,Bd = rp.discretize(Ac,Bc,dt)
K,P = cache_variant(Ad,Bd,Q,R,rho,addQ,addR)
print(f"\nAt dt={dt} rho={rho} addQ={addQ} addR={addR}:")
print("  Kinf err:", np.max(np.abs(K-Kinf)))
print("  Pinf err:", np.max(np.abs(P-Pinf)))
print("  A(dt) err vs file:", np.max(np.abs(Ad-A)))
print("  Kinf[0,:4] mine:", K[0,:4])
print("  Kinf[0,:4] file:", Kinf[0,:4])
