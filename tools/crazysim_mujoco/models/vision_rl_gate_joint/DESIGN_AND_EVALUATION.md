# Joint gate-passage and obstacle-avoidance design

This document is the paper-oriented design and evidence record for the compact
two-frame vision policy integrated with TinyMPC. It describes the accepted
system as implemented, distinguishes training-only privileged information from
deployed inputs, and records unsuccessful approaches as well as final results.
The experiment remains opt-in; it does not replace the repository's frozen
default vision policy, TinyMPC state cost `Q`, input cost `R`, dynamics, or
cached banking matrices.

## Research question and scope

The experiment asks whether a single compact visual encoder can learn the
transition from gate centering to reactive obstacle avoidance without erasing
the previously demonstrated obstacle behavior, while remaining compatible
with the GAP8/DORY deployment flow.

The learned component does **not** choose MPC weights and is not a replacement
for the model-based controller. TinyMPC continues to predict the vehicle
dynamics and track a geometric course reference. The network supplies:

1. a discrete visual navigation intent, `TRACK`, `LEFT`, or `RIGHT`; and
2. four image-plane NewBeeDrone gate rail centers plus a gate-presence score.

The controller converts those outputs into bounded, short-lived modifications
of the reference/avoidance state machine. World position, course progress,
physical constraints, motor commands, and MPC optimization remain outside the
network.

## System decomposition

The perception/control path is:

```text
previous + current HM01B0-like grayscale frames
                       |
             compact shared CNN encoder
                 /                 \
       TRACK/LEFT/RIGHT       gate corners + confidence
                 |                 |
       reactive dodge state    bounded gate association
                 |                 |
                 +------ TinyMPC reference/constraint update
                                      |
                              motor-level command
```

This separation is intentional. Gate supervision improves the shared visual
representation, but a gate detection is not interpreted as an unconstrained
world pose or as proof that all apparent obstacles are free space. Conversely,
the navigation action remains valid when gate geometry is rejected by the
runtime checks.

## Camera and temporal input

The deployed input is a normalized tensor

\[
I_k = [I_{k-1}, I_k] \in [0,1]^{1\times2\times160\times160},
\]

ordered as previous frame then current frame. The first inference duplicates
the current frame. The two-frame input preserves motion evidence while avoiding
a recurrent state and its deployment complexity.

The simulator path emulates the HM01B0/AI-deck geometry used by the existing
IsaacSim data tools. The fixed normalized intrinsics exported with the bundle
are derived from

```text
fx = 89.1558392549 px
fy = 89.4608171623 px
cx = 81.1038105230 px
cy = 73.3473030288 px
image = 160 x 160 px
```

For the gate/obstacle experiment, CrazySim renders a calibrated overscan image
and the bridge applies the same principal-point, distortion, grayscale-response,
and deterministic noise path before inference. A camera frame clock, rather
than host wall time, imposes the configured delivery delay so simulator load
cannot silently change sensor latency.

## Compact policy architecture

The float model is `JointTemporalPolicy` in `model.py`. For width `w`, the
shared encoder is:

```text
Conv(2 -> w, 5x5, stride 3) + ReLU
Conv(w -> 2w, 5x5, stride 3) + ReLU
Conv(2w -> 3w, 3x3, stride 2) + ReLU
AvgPool(3x3, stride 3)
Linear(27w -> 48) + ReLU
```

The deployment heads are a three-logit actor and a nine-value gate head. The
latter represents eight normalized corner coordinates in TL/TR/BR/BL order and
one confidence logit. Risk and return-value heads exist only during training;
they shape the shared encoder but are absent from the deployed contract.

The ONNX interface is:

```text
frames                  float32 [1,2,160,160], range [0,1]
action_logits           float32 [1,3]
gate_corners            float32 [1,4,2], range [0,1]
gate_confidence_logit   float32 [1]
```

## Training environments and demonstrations

Representative rollouts are generated in IsaacSim under Slurm from three
families:

- centered gate passage;
- obstacle-only avoidance with balanced left/right passes; and
- an ordered gate passage followed by a downstream obstacle dodge.

Each episode is a physical rollout. A layout seed is an indivisible provenance
group assigned to exactly one of train, validation, or sealed test using a
stable SHA-256 rule; frames from one scene cannot leak between splits. The
authoritative four-piece NewBeeDrone mesh and texture are used for both
rendering and collision geometry. A rollout is rejected if it contacts the gate
or obstacle, misses the gate opening, violates the ordered transition, or lacks
usable gate imagery where required.

Every stored transition contains two frames, expert action, normalized gate
rail centers and visibility, obstacle risk, privileged reward, episode/layout
identity, and physical geometry/contact provenance. The privileged fields are
training targets only and never enter the deployed network.

The easier transition acceptance course scales the physical NewBee gate by
2.222x in its plane to a disclosed 1.00 m opening and places a 0.80 m-wide downstream
obstacle at `(6.2, 0.6)` m. Inflating the
obstacle by the 0.10 m vehicle radius leaves the nominal centerline 0.10 m of
clearance but intersects the gate-shifted lane. The task therefore measures a
post-gate recovery transition rather than demanding an abrupt full-width
sidestep. Its exact dimensions and hashes are sealed in
`courses/gate_obstacle_easy_transition*.json` and the corresponding MuJoCo
scenes. The easier course is an evaluation choice, not a hidden firmware map:
the runtime controller stores no obstacle coordinates or encounter index.
To keep the metric focused on that transition rather than terminal stopping,
completion is the first sample jointly inside a 0.40 m sphere centered at
`(8.75, 0)` on the 9 m route and within 0.35 m of the centerline. Gate passage,
all physical contacts, and obstacle/room clearance remain independent hard
conditions.
Because completion time is defined as the *first* joint terminal sample, the
reported terminal cross-track will naturally lie close to 0.35 m for a vehicle
rejoining from outside; that value is a threshold-crossing timestamp, not a
claim of final steady-state margin.

## Reward and learning objective

The privileged per-transition reward combines positive course progress and a
one-time ordered gate-passage bonus with penalties for gate-center error,
ordinary-obstacle clearance, gate-frame clearance, action switching, and
physical contact. The reward does not directly command the drone. It changes
the relative weight of demonstrated actions using clipped advantage-weighted
regression.

For return

\[
G_k = r_k + \gamma G_{k+1}, \qquad \gamma=0.98,
\]

the actor weight is

\[
w_k = \operatorname{clip}\left(
\exp\left((G_k-V(I_k))/T\right), e^{-c}, e^c\right),
\]

with temperature `T=4` and clip `c=2`. Ordinary cross entropy remains in the
loss so poor value estimates cannot erase supervised behavior. The complete
training loss is

\[
\begin{aligned}
L={}&L_{\rm actor}+\lambda_{\rm AWR}L_{\rm AWR}
+\lambda_g L_{\rm corner}+\lambda_c L_{\rm gate\ conf}\\
&+\lambda_r L_{\rm risk}+\lambda_v L_{\rm value}
+L_{\rm retention}.
\end{aligned}
\]

Corner loss is masked Smooth L1 on visible gates. Gate confidence uses binary
cross entropy; risk and value use mean squared error. Hyperparameters and the
exact selected checkpoint are preserved in `config.json`,
`training_metrics.json`, and `bundle.json`.

This is best described as reward-weighted offline policy learning, or an
offline-RL-inspired student, rather than unconstrained online reinforcement
learning. The expert demonstrations anchor the policy and the physical reward
ranks their transitions.

## Anti-forgetting retention

Three sources prevent the joint objective from erasing previously useful
behavior:

1. the frozen obstacle-policy training shard contributes actor-only labels;
2. cumulative runtime gate-passage corrections contribute TRACK labels and
   geometrically projected gate corners/confidence; and
3. a contact-free frozen-baseline obstacle-only CrazySim rollout contributes
   actor-only labels under an exact teacher and course hash.

Retention data cannot supply fabricated reward, risk, or value targets. Each
artifact validates array types, temporal order, sample counts, source run and
video hashes, course identity, calibration, and selection rule before training.

## Runtime gate association and reference shift

The bridge rejects malformed manifests, artifact hash mismatches, unexpected
ONNX names/shapes, non-finite outputs, and implausible gate quadrilaterals. A
gate association requires two consecutive fresh geometric observations. The
last accepted bearing can be held briefly across the near-gate blind interval,
but weak presence alone cannot update the bearing. In the joint build, current
geometric opening evidence receives a bounded 20-frame association grace (at
most 0.67 s at the 30 Hz camera rate). This prevents gate rails from latching a
full dodge during a short near-plane dropout, while restoring ordinary obstacle
authority well before the downstream encounter. The same finite bound governs
the retained center-bearing correction, so confidence without accepted geometry
cannot extend either behavior. Another active avoidance constraint also takes
priority immediately.
These proof-of-concept courses contain one physical gate. Once
the retained association expires, the controller marks that gate complete for
the remainder of the flight, preventing a rear view or a gate-like downstream
obstacle from stealing avoidance authority back. Supporting multiple gates
would require an explicit re-arm rule or course-level gate identity and is left
for future work.
Continuous gate-like false geometry cannot extend the association indefinitely:
a separate monotonic 300-camera-sample lifetime caps both servoing and collision
suppression at 10 s after acquisition, independent of the dropout counter.

For accepted corners, the controller estimates a bounded image-bearing
correction. With normalized gate width `w_i`, normalized focal length `f_x`,
and known rail-center span `d_g`, the approximate depth is

\[
\hat z = \operatorname{clip}(d_g f_x/w_i, 0.4, 4.0).
\]

For the real-size NewBee class, `d_g=0.555 m`. The disclosed 2x held-out-room
gates use `d_g=1.11 m`; the 2.222x exact-transition gate uses
`d_g=1.233333 m`. The appropriate value is injected only into each opt-in
firmware build and recorded in its run acceptance contract. Default and
real-size builds retain 0.555 m.

The center bearing is converted to lateral/vertical displacement, limited to
`+/-0.35 m` and `+/-0.25 m`, filtered with coefficient `0.25`, and applied as a
ramp over the MPC horizon. A small positive vertical crossing margin prevents
a near-fill outlier from lowering the reference into the bottom rail. For the
opt-in joint build only, development evidence justified a bounded 0.12--0.16 m
upward feedforward; legacy POC and default builds retain 0.08--0.12 m. A dodge,
active avoidance half-space, or recovery phase immediately takes priority over
gate servoing.

The joint policy's action is the only collision intent in this experiment:
`TRACK` leaves collision inactive, while `LEFT` or `RIGHT` enters the existing
reactive avoidance state machine. The controller shifts the planned reference
and local tunnel consistently, executes the pass, and rejoins the canonical
path. It does not memorize obstacle locations.

## TinyMPC invariants

This experiment deliberately leaves the model-based controller unchanged:

- the generated TinyMPC parameters and `Q/R` matrices are hash checked;
- cached attitude/banking matrices and their provenance are hash checked;
- the actuator LTI bundle is hash checked; and
- the joint policy must be explicitly selected with the `joint_gate_rl`
  adapter and the matching compile flag.

Thus any measured change in this experiment is attributable to data, policy,
or bounded perception integration rather than a simultaneous MPC retune.

## Closed-loop acceptance protocol

### Gate-to-obstacle transition

The exact acceptance matrix contains ten fresh seeds and three matched arms per
seed:

1. candidate on the easier gate-plus-obstacle course;
2. candidate on the matched obstacle-only course; and
3. frozen baseline on the matched obstacle-only course.

A transition success requires the measured vehicle center to cross the gate
plane inside the physical opening, then complete the downstream obstacle pass,
with zero gate-frame and obstacle contacts. The candidate must achieve at least
8/10 successes, zero gate-frame contacts, and an obstacle-only collision rate
no worse than the frozen baseline. Invalid, missing, overwritten, or provenance-
mismatched runs fail the matrix.

### Held-out canonical trajectories

The candidate is also evaluated in fake rooms on straight, circle, oval, and
smoothed figure-eight courses, five fresh seeds per course. The gate pose, a
later obstacle anchor, and a still-later completion point are generated from
strictly ordered checked trajectory samples and analytic tangents. Each 2.2
m-tall obstacle is offset 0.25 m from the route so the inflated nominal
footprint intrudes without creating a centered hard block. Completion search
begins only after the ordered physical gate crossing, and obstacle/room
clearance is measured on that post-gate interval. The evaluator verifies
course/scene/header hashes, exact header-derived gate and obstacle poses,
the XML gate-body pose, tangent dot product at least 0.999,
timing/calibration configuration, finite telemetry, model hash, and unchanged
controller artifacts.

For this first cross-trajectory matrix, the visual four-piece NewBee mesh and
its collision rails are uniformly scaled 2x in the gate plane, producing a
disclosed 0.90 m physical opening in MuJoCo. This easier-course choice absorbs
known bearing error without changing the trajectory anchor or silently
widening only the evaluator's pass condition.

Reported per-course metrics are physical gate passage, contact-free completion,
minimum clearance, speed, and inference latency. The aggregate acceptance
threshold is at least 70% contact-free completion across all 20 runs with no
invalid or missing trial, plus at least 2/5 contact-free completions on each
individual trajectory so aggregate success cannot hide a wholly unsupported
course.

## GAP8/DORY and GVSOC validation

The deployment branch re-expresses the accepted checkpoint as a single-output
NEMO/DORY graph while retaining the two-frame input. The packed terminal output
is

```text
[3 action logits, 8 gate-corner logits, 1 gate-confidence logit]
```

Sigmoid and terminal affine decoding run outside DORY. This avoids unsupported
multi-output and graph-manipulation operators without changing the decoded
semantics. Export provenance includes the source checkpoint/ONNX hashes,
terminal scale, quantization epsilon, sealed sample indices, and every generated
artifact hash. The float export contract is explicitly pre-quantization; NeMO
emits the authoritative quantized byte contract, and the checker verifies its
terminal epsilon before decoding generated-C output.

Exactly 200 training-excluded two-frame clips are packaged as ordered READFS
files. Terminal-affine fitting and NeMO calibration consume this same set, so
the report conservatively calls it a calibration/parity corpus rather than a
post-quantization held-out generalization set. The generated C harness reuses
one 51,200-byte HyperRAM staging buffer rather than
placing the 10.24 MB corpus in target memory, and logs one ordered record per
clip containing inference cycles and all twelve output bytes. Acceptance
requires:

- all 200 samples present exactly once and in order;
- at least 95% action agreement with the accepted float ONNX policy;
- visible-gate center error degradation no greater than 5 px;
- p95 inference below 33 ms using cycles divided by the declared GAP8 clock;
- supplemental confidence, individual-corner, and static runtime-admission
  parity, explicitly labeled as post-audit criteria;
- generated-code and dynamic-buffer L1/L2 use within the target capacities;
- a conservative deployment flash upper bound within 8 MiB, computed from the
  complete built flash image minus only the exact 200-clip calibration corpus;
  and
- fail-closed proof that the exported checkpoint and SITL policy ONNX are the
  two declared artifacts in the same accepted runtime bundle.

This is simulator validation only. No physical GAP8 is flashed, and no
real-hardware latency or energy claim is made.

## Final evidence

The tables below must be filled only from sealed terminal Slurm outputs. Failed
or canceled jobs remain part of the provenance record and are not silently
reused as acceptance evidence.

### Training and sealed offline test

| Item | Result |
|---|---:|
| Dataset/export Slurm job | 5917 render; 5929 immutable compaction recovery |
| Training Slurm job | 5931 |
| Dataset manifest SHA-256 | `e543f9e2b94a9248225ce9e637d91405f6d203f9cbed0236c80babfec3dc6c24` |
| Candidate ONNX SHA-256 | `7adcc87649790c014ed9b8fb8277a5255fd3becf970e50ef2953b92a2536512f` |
| Sealed-test action macro recall | 0.8373 |
| Sealed-test gate visibility F1 | 0.5222 |
| Sealed-test visible corner MAE | 9.41 px |

### Gate-to-obstacle closed loop

| Arm | Successes | Gate-frame contacts | Obstacle contacts |
|---|---:|---:|---:|
| Candidate, gate + obstacle | 9/10 strict completions; 10/10 physical gate passes | 0 | 0 |
| Candidate, obstacle only | 10/10 | N/A | 0 |
| Frozen baseline, obstacle only | 8/10 | N/A | 0 |

This was bounded Slurm job 5939 on fresh seeds 3121--3130 and is retained as
accepted **pre-final-controller** evidence. All 30
artifacts passed the evaluator's model, course, calibration, controller, and
matrix-provenance checks. Candidate gate-plus-obstacle runs had 0.397 m
minimum obstacle clearance, 0.581 m/s mean speed, and 18.17 ms maximum per-run
inference p95. Candidate obstacle-only runs had 0.467 m minimum clearance,
0.576 m/s mean speed, and 19.11 ms maximum p95. The frozen baseline had 0.603
m minimum clearance, 0.498 m/s mean speed, and 13.01 ms maximum p95. Seed 3123
crossed the physical gate safely and remained contact-free, but missed the
strict downstream completion sphere; it is therefore counted as a failure.
Because later held-out diagnosis changed the gate-association state machine,
job 5939 is not terminal evidence for the final controller; a fresh serialized
matrix supersedes it below when complete.

The final-controller matrix was bounded Slurm job 5989 on fresh seeds
4001--4010. It completed successfully and passed every fail-closed provenance
and acceptance check:

| Arm | Successes | Gate-frame contacts | Obstacle contacts | Min obstacle clearance | Mean speed | Max run p95 |
|---|---:|---:|---:|---:|---:|---:|
| Candidate, 1.00 m gate + obstacle | 10/10 | 0 | 0 | 0.558 m | 0.531 m/s | 19.54 ms |
| Candidate, obstacle only | 10/10 | N/A | 0 | 0.390 m | 0.522 m/s | 17.59 ms |
| Frozen baseline, obstacle only | 10/10 | N/A | 0 | 0.409 m | 0.490 m/s | 12.86 ms |

Every primary run physically crossed the gate in order and completed the
post-obstacle terminal rejoin. The minimum physical gate-frame clearance across
those ten runs was 0.094 m. Candidate and baseline obstacle-only collision rates
were both zero, satisfying the no-worse-than-baseline retention criterion
without claiming that the candidate improved minimum clearance. The compact,
hash-sealed report is under `evidence/gate_obstacle_final_job5989/`.

### Held-out fake rooms

| Course | Contact-free completions | Gate passes | Min obstacle clearance | Speed | p95 inference |
|---|---:|---:|---:|---:|---:|
| Straight | 3/5 | 4/5 | 0.275 m | 0.481 m/s | 17.31 ms |
| Circle | 0/5 | 5/5 | 0.429 m | 0.464 m/s | 17.63 ms |
| Oval | 0/5 | 5/5 | 0.491 m | 0.568 m/s | 20.05 ms |
| Smoothed figure-eight | 0/5 | 5/5 | -0.003 m | 0.476 m/s | 15.59 ms |
| **Aggregate** | **3/20 (15%)** | **19/20 (95%)** | — | — | — |

Terminal held-out Slurm job 5990 therefore **failed** the preregistered 70%
aggregate and 2/5-per-course criteria. Its nonzero exit was the evaluator's
intended fail-closed response, not a Slurm or camera-bridge infrastructure
failure. All 20 trial directories and summaries exist; two runs (straight 5302
and circle 5304) additionally failed the evaluator's ordered
gate-to-completion interval validity check, so the no-invalid/missing criterion
also failed even though no run was missing and `matrix_errors` was empty.

The failure is localized rather than a claim that the perception head was
unused: 19/20 runs physically crossed the gate, and every curved-course run
crossed it. Circle and oval obstacle clearances remained positive in every
reported aggregate; four of five figure-eight obstacle margins were positive,
with one 2.7 mm penetration. The repeated curved-course pattern was a rapid
altitude/attitude loss after gate passage, ending in contact before valid
terminal rejoin. This shows that the single-gate association and discrete
avoidance handoff generalized more strongly than the combined curved-reference
recovery. Post-run source and telemetry inspection identified a concrete frame
mismatch in that integration: gate bearing is camera/body-lateral, but the
servo treats solve-local Y as the path normal for the entire future horizon,
subtracting `Xref[0](1)` and applying its ramp directly to `Xref[k](1)`.
Solve-local Y follows current vehicle yaw and is a reasonable yaw-level
first-knot approximation, but it does not rotate with each future curved-path
knot. The avoidance adapter correctly constructs a per-knot normal,
but a displaced curved path still retains mostly nominal curvature/bank
feedforward. Several curved runs also acquired gate association only near or
after physical crossing, allowing an obstacle dodge to begin before gate
suppression. The result combined incompatible position, velocity, and bank
references: curved runs reached approximately 180 degrees attitude error,
29--170 rad/s peak angular rates, and near-ground altitude. Motor saturation
varied from negligible to substantial, so it was an amplifier rather than the
common root cause. It is not evidence for real-flight readiness. The complete
negative report is retained under `evidence/heldout_rooms_final_job5990/` and
must not be replaced by a post-hoc easier rerun under the same claim.

The integration correction identified above is now implemented: the accepted
camera-relative bearing is projected onto the first-knot path normal, every
future reference knot is displaced along its own rotating normal, and the
path-tunnel half-spaces are rebuilt around that displaced horizon. Velocity,
attitude feedforward, generated Q/R, cached bank models, and actuator artifacts
remain unchanged. This is a structural consistency fix, not a post-hoc MPC
retune.

A fresh serialized diagnostic on a2r-tiger (Slurm job 5993, seed 5402) then ran
circle, oval, and smoothed figure-eight with the corrected controller. All
three physically crossed the gate, but all three still contacted the scene
before completion. More importantly, the first accepted gate frame arrived
*after* physical gate crossing on every course: circle accepted one frame at
7.233 s after crossing at 6.186 s; oval first accepted at 4.767 s after crossing
at 4.711 s; and figure-eight first accepted at 10.200 s after crossing at
4.540 s. Thus the new path-normal servo could not guide the approach in this
diagnostic. Circle and figure-eight had already entered a banked dodge before
association, while oval acquired only a short late association. The crashes
again reached approximately 180 degrees attitude error and 17.8--56.0 rad/s.
This falsifies the narrower hypothesis that the reference-frame defect alone
caused the curved-course failures. The remaining primary limitation is low or
late gate admission under curved/rolled views, followed by an unstable
discrete dodge/recovery handoff. Job 5993 is preserved as negative post-fix
evidence and does not supersede the preregistered job-5990 matrix.

### DORY/GVSOC

| Item | Result |
|---|---:|
| DORY/GVSOC Slurm job | 5968, completed 0:0 |
| Ordered samples | 200/200; six clip-zero layer checks exact |
| Action agreement | 0.990 (requirement >= 0.950) |
| Visible gate-center degradation | +3.246 px (requirement <= 5 px) |
| Confidence decision agreement / visible-recall degradation | 0.940 / 0.113 |
| Visible corner-coordinate MAE degradation | +1.777 px |
| Static runtime gate-admission agreement / visible-recall degradation | 0.945 / 0.099 |
| Cycle p95 / declared-clock latency | 1,482,162 cycles / 14.822 ms at 100 MHz |
| L1 use / capacity | 36,728 / 64,000 B |
| L2 use / capacity | 466,152 / 512,000 B |
| Deployment flash upper bound / capacity | 372,256 / 8,388,608 B |

The 372,256 B flash figure begins with the 10,612,256 B complete
instrumented image and subtracts only the exact 10,240,000 B raw evaluation
corpus. The generated-C application, signed-safe DORY weights, filesystem and
alignment overhead, boot payload, and default input all remain in the bound.
The report binds checkpoint `5548f36c...` and runtime ONNX `7adcc876...` to
bundle `bf5d5b6c...`; job 5968 used GVSOC only and performed no hardware flash.
The confidence/corner/admission checks were added after a semantic audit and
are labeled supplemental in the machine report. The 200 clips establish
conversion parity, not unseen-scene accuracy, because they also calibrate the
terminal affine and integer graph.

## Negative results and limitations

- Earlier joint candidates could pass the gate but sometimes selected or held
  the wrong downstream avoidance side. This motivated explicit transition
  demonstrations and matched obstacle-only retention rather than hand-changing
  MPC matrices per course.
- A prior export rendered all episodes but failed its final far/small-gate
  validator. The raw rollouts were retained; any recovery must use a geometry-
  justified threshold and compact the immutable transitions without rerendering
  or changing split membership.
- Initial GVSOC attempts exposed environment and payload-packaging defects after
  successful NEMO/DORY compilation. They are retained as failed infrastructure
  evidence and cannot be quoted as deployment performance.
- GVSOC job 5941 stopped before code generation because NeMO emitted a
  non-power-of-two BN post-multiplier of 51 whose legacy packed `int32`
  coefficients would overflow. The guard correctly prevented wraparound. The
  final export uses DORY's existing GAP8 64-bit PULP-NN coefficient path via a
  local, exact, fail-closed lowering; it neither clips coefficients nor relaxes
  the overflow check.
- Replacement job 5942 retained the same passing 200-clip NeMO precheck but
  stopped before DORY because the isolated Python invocation omitted the
  repository root from `PYTHONPATH`. The exact import was reproduced from a
  neutral working directory before the wrapper-only correction was resubmitted.
- Held-out-room job 5940 was canceled after its first two trials exposed a
  host-calibration mismatch: Slurm placed it on `a2r-fox`, while the pinned
  `0.2/0.1347944` simulator/firmware timing pair was measured and accepted on
  `a2r-tiger`. Their 0.483 s and 0.282 s motor launches violated the requested
  1.0 +/- 0.5 s contract. Those flight outcomes are not used as held-out
  evidence; the unchanged matrix is rerun on the calibrated host.
- A node-pinned retry still launched at 0.777 s because the fake-room rendering
  workload differs from the earlier transition scene. Same-workload tiger
  preflight job 5946 corrected the firmware factor to `0.1047352488` and
  measured a 1.038 s launch (0.038 s error). Its seed 4101 is calibration-only;
  the first centered-gate development block was 4201--4205. Its first validly
  timed run was contact-free but missed the centered gate after the gate image
  triggered a premature full dodge. That matrix was canceled and is not
  acceptance evidence. A second development run with an offset gate (seed
  4301) exposed a wrong-sign bearing under the fake-room appearance and was
  also rejected; layout tuning was abandoned in favor of the short-lived
  geometric opening guard above. Seeds 4801--4805 later became an incomplete
  development block (job 5971), not final evidence. Every reported terminal
  run uses the new 5301--5305 block and must independently remain within a
  stricter +/-0.10 s launch window.
- GVSOC job 5943 compiled the complete generated application and processed
  ordered clips 0--149, then exited at clip 150. The generated READFS loader
  opened every input file without closing its descriptor; the finite descriptor
  table was exhausted. The final harness applies an idempotent, fail-closed
  patch to close each generated weight/input descriptor after a complete read.
- GVSOC job 5948 then processed all 200 clips and passed latency and memory
  limits, but failed semantic parity (0.895 action agreement and +76.44 px
  visible gate-center degradation). Per-layer checks isolated the first error
  to layer-0 channel 10. NeMO's nominal 8-bit quantizer had emitted ten
  convolution coefficients outside signed `int8` across the graph; DORY cast
  them modulo 256. The final exporter instead saturates those integerized
  weights before generating both golden activations and ONNX, then audits every
  serialized weight range. It does not compare wrapped target output against an
  unattainable host reference.
- Centered physical-gate held-out seed 4401 (job 5953) contacted the 0.45 m
  frame after intermittent gate geometry restored a false avoidance trigger.
  The first explicitly scaled-gate retry was contact-free but still detoured
  around the opening. Those are development failures, not acceptance runs.
  The final hybrid guard retains opening authority for a bounded 20 frames,
  while the held-out rooms disclose a 2x physical NewBee mesh and collision
  envelope rather than changing only the evaluator tolerance.
- Held-out development jobs 5962 and 5965 were stopped after their first two
  straight-room seeds rather than mixed into final evidence. Job 5962 exposed
  an unbounded confidence-only association latch; the corrected state machine
  now releases both the retained path shift and collision guard after the same
  finite geometry-outage window. Job 5965 then isolated a separate false dodge:
  a coherent but globally rotated gate quadrilateral was rejected by an
  image-axis-alignment test just before five `RIGHT` actions. The final gate
  check is rotation-invariant while retaining edge length, opposing-edge
  parallelism, ratio, diagonal, and convexity checks. Seeds 4801--4805 were
  not used in either diagnosis, but they are no longer final evidence: job
  5971 was canceled after four straight-path runs when seeds 4803 and 4804
  showed that the same gate could be reacquired downstream. The partial matrix
  has no evaluator report and is excluded. It motivated the final one-shot
  completion latch; the terminal held-out matrix uses a new seed block.
- Exact-matrix job 5973 is invalid infrastructure evidence. It was mistakenly
  launched concurrently with job 5971; all trial launches failed before flight
  because the vision bridge could not bind its shared camera UDP endpoint
  (`EADDRINUSE`). The old wrapper nevertheless exited zero after recording
  nonzero trial codes. The final runner now aborts on the first failed run, its
  evaluator returns nonzero on rejected acceptance, and final matrices are
  serialized. No job-5973 row is interpreted as controller behavior.
- Finalization attempt 5979 was deliberately canceled before its first summary
  when the read-only audit found that the checked-in 0.90 m gate and its
  canonical generator still disagreed. No trial or evaluator result is reused.
  The generator, metric-span compile contract, XML collider checks, and fresh
  seed enforcement were repaired before the terminal jobs were submitted.
- Exact attempt 5983 was canceled after its first seed exposed an analyzer
  ordering defect, not a contact or control failure. The vehicle passed the
  gate and obstacle and later rejoined, but the analyzer froze terminal
  cross-track at the first entry into the finish sphere rather than searching
  for the first sample satisfying both terminal conditions. The corrected
  analyzer searches their intersection; seed 3701 and the never-started
  dependent 5001--5005 block are excluded, and terminal matrices use entirely
  new seeds.
- Exact attempt 5985 then showed that `[8.5, 0]` with a 0.36 m radius was too
  brittle for an experiment intended to measure handoff rather than terminal
  stopping: seed 3801 passed the gate and obstacle without contact, ended at
  only 0.060 m cross-track, but missed that arbitrary sphere by 1.5 mm. The
  disclosed easier course now places the checkpoint at 8.75 m on the 9 m path,
  uses a 0.40 m sphere, and still independently requires cross-track <=0.35 m
  at completion. Seed 3801 and dependent 5101--5105 are excluded; the final
  exact/held-out blocks were advanced beyond 3901--3910 and 5201--5205.
- Exact attempt 5987 then found the remaining physical bottleneck: seed 3902
  struck the 0.90 m gate's lateral rail by 9.4 mm, before reaching the
  downstream obstacle. Because zero gate contact is a hard requirement, that
  block is excluded. The final exact course uniformly scales both mesh and
  colliders to a disclosed 1.00 m opening (0.40 m vehicle-center clearance
  after the 0.10 m radius); its metric rail-center span is simultaneously
  changed from 1.11 m to 1.233333 m. Final exact/held-out seeds are
  4001--4010 and 5301--5305.
- GVSOC job 5963 consumed the signed-safe payload and all 200 clips but still
  failed semantic parity because it was compiled with `CORE=7`. The copied
  PULP-NN kernels use power-of-two row partitioning; at seven cores the 1x1
  feature layer computes a zero-sized chunk and is skipped. A redundant
  seven-core retry (5967) was canceled once this was proven statically. The
  final build uses all eight GAP8 cluster PEs and requires exact generated-C
  checksums after every layer on clip zero before terminal metrics can pass.
- IsaacSim and MuJoCo camera-response matching is a calibrated approximation,
  not proof of a zero sim-to-real gap.
- The discrete action vocabulary cannot express continuous avoidance magnitude.
  The existing controller supplies maneuver shape and limits, so performance
  depends on that hybrid interface.
- GVSOC validates generated software behavior and simulated cycles, not board
  power, thermal behavior, sensor timing, or physical flight safety.
- Absolute gate admission on the calibration corpus is modest: the float
  bridge/controller predicate admits 23/71 labeled visible gates and the
  quantized GVSOC output admits 16/71. The supplemental parity gate limits the
  conversion loss and false positives; it does not turn the compact head into
  a high-recall detector. Closed-loop success therefore depends on temporal
  association and a few reliable early observations, and broader real-camera
  training remains the main pre-flight follow-up.
- The final canonical-room matrix (job 5990) passed 19/20 physical gates but
  only 3/20 full contact-free completions. Straight retained 3/5; circle, oval,
  and figure-eight each retained 0/5. The curved runs typically lost altitude
  and attitude after the gate and before terminal rejoin. The identified
  current-yaw/per-knot-normal mismatch has been corrected, including rebuilding
  the shifted tunnel. Post-fix diagnostic job 5993 still failed all three
  curved courses because gate geometry was admitted only after physical
  crossing, too late to prevent the preceding dodge/handoff. Low admission
  recall under curved/rolled views and the resulting discrete recovery remain
  unresolved and block a real-flight robustness claim even though the isolated
  exact transition and GVSOC conversion passed.

## Reproduction map

| Purpose | Entry point |
|---|---|
| IsaacSim rollout export | `isaacsim_snapshot/user_workflows/export_joint_gate_obstacle_rollouts.py`, overlaid into the pinned workspace per `isaacsim_snapshot/README.md` |
| Slurm training | `train_joint_gate_obstacle.sbatch` |
| Network/training | `model.py`, `train.py`, `config.json` |
| CrazySim runtime adapter | `tools/crazysim_mujoco/vision_bridge.py` |
| Controller integration | `apps/controller_tinympc_eigen/src/controller_tinympc.cpp` |
| Exact transition matrix | `tools/crazysim_mujoco/gate_joint/` |
| Held-out room matrix | `tools/crazysim_mujoco/heldout_rooms/` |
| GAP8 export and GVSOC | `tools/crazysim_mujoco/models/vision_rl_gate_joint/gap8/` |

The final manifests and evaluator reports, rather than this narrative alone,
are the authoritative source for all numeric claims.
