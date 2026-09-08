#!/usr/bin/env python3
"""Reject a timing harness that no longer matches the production solve block."""
from pathlib import Path
base=Path(__file__).resolve().parents[1]/'src'
def block(path):
 s=path.read_text(); start=s.index('    // Cold-start both input and state ADMM variables together.')
 end=s.index('    mpc_constraints[3] = (float)(usecTimestamp() - startTimestamp);',start)
 return s[start:end]
assert block(base/'controller_tinympc.cpp')==block(base/'tinympc_benchmark.inc')
print('Benchmark solve block matches production')
