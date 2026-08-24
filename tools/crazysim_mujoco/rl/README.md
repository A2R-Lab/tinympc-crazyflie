# Vision-RL + TinyMPC proof of concept

This experiment replaces only DroNet's two navigation outputs. The firmware
packet ABI, TinyMPC dynamics, fixed matrices, tunnel constraints, actuator
limits, and failure handling remain unchanged.

The deployment policy consumes two consecutive 160 x 160 grayscale frames and
chooses `TRACK`, `LEFT`, or `RIGHT`. Training uses simulator truth to supervise
a seven-value visual latent state, learns an ensemble transition model, and
uses at most five-step Dyna/MBPO rollouts to train the actor/critic. Only the
image encoder and actor are exported to ONNX.

## Camera and course provenance

The tracked `dronet_u.json` course is the repository's reconstruction of the
published DroNet U-course. Its turn centerline and longitudinal obstacle
positions are diagram-derived approximations; they are not IsaacSim ground
truth.

Camera numbers come from `tinympc-perception` tag `hm01b0-v4`, commit
`890bbd6923a9459d4d7ab7bee92542e4cb61c64d`, file
`gap8_perception/configs/hm01b0_calibration.json`. MuJoCo renders 160 x 160 at
the matching 83.6091095 degree vertical field of view. The recorded OpenCV lens
distortion is preserved in manifests but is not applied by MuJoCo. No texture,
scene asset, or other third-party file is copied.

## Reproduction

Collect training runs with simulator seeds 11, 22, and 33 using the same
course, camera, timing, noise, turbulence, and one-frame delivery delay planned
for evaluation. Each retained run must include `run_config.json`, `state.csv`,
`vision.csv`, and `fpv_camera.mp4`.

The bounded collection/evaluation matrix is encoded in
`slurm/run_crazysim_matrix.sbatch`. Set `PURPOSE` to `calibration`, `training`,
`baseline`, or `hybrid` and provide a fresh `OUTPUT_ROOT`. Calibrate once with
DroNet and use that same factor for every training and paired evaluation run.

Submit training through SLURM; do not run the full job on the login node:

```bash
RUN_DIRS=/absolute/seed_11:/absolute/seed_22:/absolute/seed_33 \
OUTPUT_DIR=/absolute/training_artifacts \
sbatch --output=/absolute/training_artifacts/slurm-%j.out \
  tools/crazysim_mujoco/rl/slurm/train_vision_rl.sbatch
```

The output contains `checkpoint.pt`, `policy.onnx`, `bundle.json`, and
`training_metrics.json`. `bundle.json` hashes every source run artifact and the
exported checkpoint/model.

Evaluate DroNet and the hybrid policy on held-out seeds 101, 202, and 303 with
byte-identical firmware and otherwise identical run options. Then aggregate:

```bash
python3 tools/crazysim_mujoco/compare_hybrid_rl.py \
  --baseline-run /absolute/baseline/seed_101 \
  --baseline-run /absolute/baseline/seed_202 \
  --baseline-run /absolute/baseline/seed_303 \
  --hybrid-run /absolute/hybrid/seed_101 \
  --hybrid-run /absolute/hybrid/seed_202 \
  --hybrid-run /absolute/hybrid/seed_303 \
  --training-manifest /absolute/training_artifacts/bundle.json \
  --out /absolute/comparison
```

The comparator rejects missing/duplicate seeds, training/evaluation seed
overlap, model drift within a controller, and configuration drift other than
the paired model/adapter identity.

## Scope and limitations

This is a small offline, course-specific proof of concept. It learns encounter
timing and pass-side selection through the existing dodge state machine; it
does not learn motor commands, arbitrary offsets, online Q/R changes, or a
guaranteed visual obstacle barrier. The simulator-truth latent, transition
ensemble, and critic are training-only. Held-out dynamics/noise seeds measure
repeatability on this reconstructed course, not geometric or sim-to-real
generalization.
