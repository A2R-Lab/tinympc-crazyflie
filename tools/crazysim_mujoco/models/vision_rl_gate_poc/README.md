# Combined gate + obstacle-avoidance proof of concept

This manifest-only bundle composes two existing two-frame teachers without
duplicating either ONNX file:

- `vision_rl_mpc_balanced_v2/policy.onnx` supplies the unchanged categorical
  obstacle action `[TRACK, LEFT, RIGHT]` and its existing packet mapping.
- `espnet_dronet_gate_v1/espnet_dronet_gate_seed2027_float.onnx` supplies gate
  corners, confidence, validity, and camera intrinsics.

`vision_bridge.py --adapter auto --model .../vision_rl_gate_poc` recognizes
this manifest as `combined_gate_rl`. The combined adapter forwards the same
camera frame to both teachers. On the two POC courses, `run.sh` first converts
a 192x192 MuJoCo overscan render to the calibrated 160x160 HM01B0 domain using
the IsaacSim camera matrix, simulation-safe lens distortion, and deterministic
gray/noise proxy. The tracked CrazySim model patch places the optical center at
the AI-deck mount `[0, 0, 0.010]` m. Every normal course and model path remains
unchanged; `run_config.json` records the complete acquisition and transform
provenance. The response/noise model is a lightweight Isaac proxy, not proof
that the physical sensor response is reproduced exactly.

The opt-in controller experiment has no gate
map or known gate coordinates: after conservative upright-gate association it
may apply only a bounded image-center reference shift. Obstacle avoidance
retains authority during a dodge, constraint, recovery, or stale association.

The bounded A2R calibration job is reproducible from
`tools/crazysim_mujoco/gate_poc/`. Its higher-recall candidate produced too
many false acceptances on the untouched test split, so the POC deliberately
retains the existing conservative confidence calibration. A compact record is
stored in `evidence/gate_acceptance_calibration.json`.

## Closed-loop result

The corrected, strictly sequential three-seed matrix is recorded under
`evidence/closed_loop_v3/`. It does **not** establish deployment feasibility:

- the combined candidate passed the hardware-scale gate in 0/3 trials and
  contacted it in 2/3; the contact-free trial went around the gate;
- candidate and exact-policy baseline obstacle-only runs both avoided contact
  in 3/3 trials, but neither completed the course because the existing
  dodge/rejoin behavior remained too slow or too far off the path;
- candidate-minus-baseline obstacle-clearance deltas were +0.081, -0.157, and
  -0.063 m, so the small sample does not support a no-regression claim; and
- combined p95 inference remained 4.28--4.67 ms, comfortably inside one 30 Hz
  camera period, so runtime is not the observed blocker.

The frozen obstacle model remains the fallback and no default deployment is
changed. A next experiment should train gate/opening association jointly with
the bypass decision (and separately repair rejoin/completion) before repeating
the same held-out matrix. This two-teacher host composition is evidence only;
it is not yet a single GAP8/DORY deployment artifact.
