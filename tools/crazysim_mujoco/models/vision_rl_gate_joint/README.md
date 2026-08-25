# Joint gate + obstacle student pipeline

This isolated offline pipeline creates a compact two-frame grayscale policy with one shared encoder. Its ONNX contract is `frames [1,2,160,160] -> action_logits [1,3], gate_corners [1,4,2], gate_confidence_logit [1]`. Runtime use is explicitly opt-in through `joint_gate_rl`; it does not replace the frozen obstacle baseline or the default controller.

`generate_dataset.py` renders deterministic analytic proxy scenes. It is not a closed-loop RL or MuJoCo acceptance claim. Gate plane/opening/frame geometry, ordered passage, obstacle clearance, contact, and reward are privileged training signals only. A scene/layout seed is an indivisible provenance group: its temporal samples remain entirely train, validation, or sealed test according to a stable SHA-256 split. For representative training, replace the proxy NPZs with external IsaacSim/MuJoCo rollout shards using the same fields; do not relabel static composites as rollouts.

The checked-in representative exporter is
`isaacsim_snapshot/user_workflows/export_joint_gate_obstacle_rollouts.py`.
Following `isaacsim_snapshot/README.md` overlays that frozen source into the
pinned external IsaacSim workspace as
`user_workflows/export_joint_gate_obstacle_rollouts.py`. Its fixed-course-yaw
physical demonstrations cover centered gate passage, alternating left/right
obstacle avoidance, and gate passage followed by a post-gate dodge. Each split
is rejected unless every gate episode has usable gate imagery and a physical
in-opening crossing, both dodge directions are present, and expert trajectories
are contact-free. It records a privileged reward with positive course progress
and one-time ordered gate passage, plus penalties for gate-center error, unsafe
ordinary/frame clearance, switching, and physical contact. The trainer uses
clipped advantage-weighted action regression alongside ordinary action
supervision and anti-forgetting samples from the frozen obstacle dataset. Gate
corners and visibility supervise the deployed gate outputs; obstacle risk and
return value remain training-only auxiliaries.

The representative Slurm launch deliberately does not mix in the older auxiliary gate renders. The joint rollout exporter imports the authoritative CrazySim NewBeeDrone four-piece OBJ mesh and `newbeedrone_gate_front_rgba_v1.png` texture, so gate appearance, rail-center labels, physical passage, obstacle actions, and privileged reward all come from the same episode. The optional `--newbee-images/--newbee-targets` loader remains isolated and train-split-only for controlled ablations, but is not enabled by the representative launcher.

Run locally (with existing NumPy/PyTorch/ONNX support):

```bash
python3 -m tools.crazysim_mujoco.models.vision_rl_gate_joint.generate_dataset --config tools/crazysim_mujoco/models/vision_rl_gate_joint/config.json --output /tmp/joint-data
python3 -m tools.crazysim_mujoco.models.vision_rl_gate_joint.train --config tools/crazysim_mujoco/models/vision_rl_gate_joint/config.json --dataset /tmp/joint-data --output /tmp/joint-model
python3 -m unittest tools.crazysim_mujoco.models.vision_rl_gate_joint.test_pipeline
```

The Slurm launcher retains the frozen 37,155-transition obstacle `train.npz` as action-only anti-forgetting supervision; it never fabricates gate labels for those samples. Its checked-in `slurm_logs/` directory exists before Slurm opens its logs. Always verify current `sinfo`, set `PYTHON_BIN`, and select a partition from the live inventory before submitting it.

The representative launcher additionally requires `RUNTIME_TRACK_RETENTION` to name a strictly validated runtime NPZ with `uint8[N,2,160,160]` frames, all-TRACK `int64[N]` actions, normalized `float32[N,4,2]` gate rail centers, binary `float32[N]` gate visibility, and per-sample run/frame/time provenance. Build a cumulative DAgger artifact by repeating `--run` in oldest-to-newest order when calling `make_runtime_track_retention.py`; each run keeps its independent frame clock and contributes an entry to the canonical `source_runs_json` list with its absolute path and video/state/config hashes. For every selected camera time, the builder interpolates the measured position and scalar-first body-to-world quaternion, applies the run-config AI-deck offset `[0,0,0.01]`, constructs the real-size retention course's authoritative 0.555 m square NewBee rail centers in TL/TR/BR/BL order, transforms them with camera axes `+u=-bodyY`, `+v=-bodyZ`, depth `+bodyX`, and calls `cv2.projectPoints` with that run's HM01B0 intrinsics and simulation distortion. A row is visible only when all four rail centers are finite, in front of the camera, and inside the 160x160 image; invisible corners are exactly zero. This retention builder is deliberately locked to that real-size label geometry; it must not label the 1.11 m or 1.233333 m scaled acceptance gates without a separately validated, span-aware contract.

The course hash remains one pinned scalar for the whole artifact. Duplicate run paths are rejected so an old correction cannot be accidentally overweighted. This corrective set contributes weighted TRACK actor cross-entropy plus masked gate-corner and gate-confidence losses (weights 0.5, 0.35, and 0.2 respectively); it cannot add risk, reward, or value losses. Exact array types/ranges, the canonical projection convention and per-run calibrations, input NPZ hash, sample/run counts, selection rule, course hash, and run/frame/time/label provenance hashes are retained in both the training metrics and bundle.

The launcher also requires `RUNTIME_OBSTACLE_RETENTION`, a separate actor-only artifact locked to the contact-free frozen-baseline obstacle-only seed-3201 run and teacher SHA-256 `291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e`. Build it with `make_runtime_obstacle_retention.py --run <baseline_obstacle_only_seed3201> --course tools/crazysim_mujoco/courses/gate_obstacle_poc_obstacle_only.json --output <artifact.npz>`. The builder requires the exact checked source hashes, `gate_obstacle_poc_obstacle_only`, seed 3201, `hybrid_rl`, the pinned teacher, zero state/summary contacts, pass-point completion, and the post-HM01B0 900-frame 160x160 H.264/vision sequence contract. Vision sequence `N` is aligned to adjacent decoded frames `[N-2,N-1]`; sequence 1 is excluded because it has no distinct previous frame, and samples stop at the summary's course-completion time. The resulting 664 labels contain `[618 TRACK, 0 LEFT, 46 RIGHT]` and contribute only actor cross-entropy at weight 1.0—never gate, risk, reward, or value supervision. The source file hashes, teacher/course hashes, action counts, selection rule, and sequence/frame/time hashes are recorded in training metrics and the deployment bundle.

The opt-in bundle in this directory is the sealed output of bounded Slurm
training job 5931. It uses the v3 physical rollout dataset rendered by job 5917
and immutably compacted by recovery job 5929 at
`/home/cchen/isaacsim-workspace/workspace/joint_gate_obstacle_rollouts_v3_job_5917`.
The dataset manifest SHA-256 is
`e543f9e2b94a9248225ce9e637d91405f6d203f9cbed0236c80babfec3dc6c24`;
the checkpoint SHA-256 is
`5548f36c5ae86d938db3545f6fd5616ac6440d8ba4f27fe45cb6c4e0812b5f3a`;
and the exported ONNX SHA-256 is
`7adcc87649790c014ed9b8fb8277a5255fd3becf970e50ef2953b92a2536512f`.

Closed-loop job 5939 was accepted on the pre-final controller: 9/10 strict
ordered completions, 10/10 physical gate passages, and zero gate-frame or
obstacle contacts. Its matched obstacle-only arm completed 10/10 without
contact versus 8/10 for the frozen baseline, also without contact. Later
held-out diagnosis changed the association state machine, so that job is
retained as development evidence and is not presented as terminal evidence for
the final code. Fresh terminal job 5989 evaluated the final state machine on
seeds 4001--4010 and achieved 10/10 ordered gate-plus-obstacle completions with
zero contacts; both candidate and frozen-baseline obstacle-only controls also
completed 10/10 without collision. The candidate remains available only
through explicit `joint_gate_rl` selection; the repository default remains
unchanged. Final held-out job 5990 is deliberately retained as negative
evidence: it passed 19/20 physical gates but only 3/20 complete contact-free
runs, with 0/5 completion on each curved course after repeated post-gate
attitude/altitude loss. See `DESIGN_AND_EVALUATION.md` for the complete design,
final matrices, limitations, and GVSOC-only deployment evidence.
