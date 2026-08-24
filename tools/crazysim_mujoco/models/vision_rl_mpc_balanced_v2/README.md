# Balanced vision-RL + TinyMPC candidate

This is the retrained two-frame obstacle-avoidance candidate produced from a
balanced procedural MuJoCo dataset and U-course camera examples. It is stored
separately from `vision_rl_mpc_poc`: it has passed offline acceptance, but has
not yet passed a closed-loop CrazySim flight and is therefore not the default
deployed policy.

The policy input is normalized float32 grayscale with shape `[1,2,160,160]`
in `[previous,current]` order. Output logits use the canonical action order
`[TRACK,LEFT,RIGHT]` and the existing `hybrid_rl` packet mapping recorded in
`bundle.json`.

## Data

The procedural dataset contains 37,155 transitions from 60 episode groups,
12 obstacle layouts, six appearances, and three noise variants. It deliberately
includes successful, contact, near-miss, and timeout outcomes plus balanced
TRACK/LEFT/RIGHT expert actions, hard recovery tracks, privileged latent state,
and counterfactual action scores. The train/validation split is episode-based.

A second 5,464-sample oracle shard uses actual U-course camera frames from
training seeds 11, 22, and 33 with mirror and photometric augmentation. Held-out
evaluation seeds 101, 202, and 303 are rejected by the oracle generator and do
not overlap training. Actor batches are balanced across both source domain and
action class so this smaller camera-domain shard is not diluted by the larger
procedural dataset.

## Training and acceptance

Slurm job 5881 trained the final candidate on an RTX 5090 for 30 epochs with
optimizer seed 20260824. Checkpoint selection required balanced avoidance,
left/right recall, hard-TRACK recovery, and easy-TRACK false-dodge gates to pass
simultaneously. Epoch 28 was selected.

On the isolated procedural validation split (7,431 samples):

- overall accuracy: 0.9388
- balanced accuracy: 0.9380
- LEFT/RIGHT decision recall: 0.9440 / 0.9301
- hard-TRACK recall: 0.9005
- mirror side equivariance: 0.8557
- PyTorch/ONNX action agreement: 1.0
- maximum PyTorch/ONNX logit error: 5.96e-6

On retained, untouched U-course camera/state streams from seeds 101/202/303,
the off-policy diagnostic reports 0.7897 decision accuracy, with LEFT/RIGHT
recall of 0.8932 / 0.6185. The original proof-of-concept policy scored 0.0575
with recalls of 0.0 / 0.1526 on the same frames.

These U-course actions did not affect the retained trajectories. Consequently,
the result establishes improved visual action classification and transfer, not
collision avoidance, stability, course completion, or real-flight safety. A
fresh closed-loop CrazySim evaluation is required before promotion.

## Integrity

- `checkpoint.pt`: `228e204c74849ae2ac36135c22840dec8daf9b08974c239dae57af6b3da63c47`
- `policy.onnx`: `291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e`

Full metrics and compact provenance are retained in `training_metrics.json`,
`bundle.json`, and `evidence/`. The large raw NPZ shards remain under the
ignored `apps/controller_tinympc_eigen/sim_runs/crazysim/hybrid_rl_balanced_v2`
workspace output tree.
