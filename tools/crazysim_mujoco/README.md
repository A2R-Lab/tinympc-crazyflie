# TinyMPC validation in CrazySim/MuJoCo

This integration runs the repository's actual out-of-tree Crazyflie controller
and TinyMPC solver inside CrazySim SITL. MuJoCo supplies rigid-body dynamics,
the CrazySim brushless motor model, contacts, and simulated sensors. An optional
offscreen FPV camera feeds a repository-local ONNX bridge, which sends the same
versioned perception observation consumed by the firmware controller.
The PWM bridge applies the firmware's quadratic normalized-speed law
(`thrust = command^2 * 0.200 N`), matching the active generated controller
model. CrazySim then independently converts that thrust target through its RPM
polynomial and 65 ms rotor dynamics. The `--pwm-thrust-full` option is retained
only for explicit plant-robustness perturbations.

The dependency is cloned under this directory at a pinned commit and patched at
setup time. Nothing is loaded from `tinympc-vision`, `tinympc-perception`, or a
sibling CrazySim checkout. The build and Python environment run in Docker.

CrazySim and the POSIX firmware do not naturally share a simulation clock: the
physics loop advances fixed 1 ms steps while the firmware's FreeRTOS port ticks
from wall time. The harness therefore runs the physics with a 1.0 real-time cap
and calibrates the firmware tick to a 0.8 simulation/wall-time rate on the
rootless-Docker validation host. This pair produced 12/12 accepted launch times
across 1.0, 1.5, 2.0, and 3.0 m/s progress-circle trials (three seeds each). A
run is rejected if the first motor command occurs more than 0.5 s from the
requested simulation launch time. Use `--realtime-factor` and
`--firmware-time-factor` to recalibrate when moving to a materially different
host or workload.

## Run

From the repository root:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory circle --duration 30 --stop-on-contact \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/circle \
  --overwrite
```

Each run produces `state.csv`, firmware and simulator logs, `summary.json`,
and `validation.png`. Add `--video` for a post-run 3-D replay.

### Progress-indexed level routes

Figure-eight, oval, and circle use the progress-indexed reference policy:

```sh
tools/crazysim_mujoco/run.sh --trajectory figure8 \
  --duration 130 --stop-on-contact \
  --random-seed 1 --out apps/controller_tinympc_eigen/sim_runs/crazysim/example
```

The controller projects measured position onto a bounded path window and builds
the MPC horizon forward by arc length. Yaw references are tracked inside
TinyMPC; no post-solve outer yaw correction is applied.

## Vision and DroNet baseline

The bundled baseline uses the full PULP-DroNet v3 weights published for the
AI-deck/Crazyflie, converted once to ONNX without retraining. It is
self-contained under `models/pulp_dronet_v3/` and does not download code or
models at run time. The camera path also matches deployment: a 324x244
Himax-style grayscale frame followed by the centered 200x200 crop used by the
v3 training and GAP8 deployment code:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory straight --duration 18 \
  --launch-prespin 1 --vision-model dronet --vision-scene obstacle \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/dronet_obstacle \
  --overwrite
```

`--launch-prespin 1` models entry from an existing hover, which is the correct
initial condition for mid-flight obstacle avoidance. The zero-RPM handoff is
retained for deliberate motor-start stress tests. Camera rendering changes the
simulator/wall-time ratio; calibrate `--firmware-time-factor` if the run's
automatic launch-time check fails.

DroNet provides only steering and one collision-risk score. The paper warns
that the latter is not a calibrated Bayesian probability. The firmware keeps
the published speed/yaw filtering, then uses those signals as evidence for a
racing-specific banked dodge: two risk samples at or above 0.50 latch the
steering-selected pass side on an unstructured straight. On a known curve,
the trajectory curvature deterministically selects the outside passing lane;
this prevents small rendering/timing changes from flipping the racing line.
TinyMPC uses a 0.70 m lateral envelope on straight courses and 0.62 m on
tangent-heading curves, then rejoins after visual-clear evidence and 0.75 m of
measured forward travel. Motion during the banked sidestep counts toward that
passage distance.
Yaw remains aligned with the stored track, so avoidance uses the direct-motor
TinyMPC bank authority without accumulating a fragile 90-degree heading
change. DroNet still does **not** claim metric clearance, spatial sectors, or
gate corners. The bridge adapters preserve that distinction.

Straight references re-arm after 0.40 m; tangent-heading curves use 1.0 m.
The shipped policy completes REJOIN before starting a new dodge. This prevents
a late or view-dependent response to the same obstacle from reversing the lane
mid-rejoin. The state machine still supports an explicitly enabled overlapping
redirect for experiments, but none of the accepted controller configurations
enable it.

Run the repeatability gate with:

```sh
python3 tools/crazysim_mujoco/run_dronet_acceptance.py \
  --profile nominal --seeds 1 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/dronet_acceptance \
  --overwrite
```

The default suite runs four manifest-backed courses: a centered box, an offset
cylinder, two alternating boxes, and an obstacle on a tangent-heading left
turn. The report computes clearance to every physical obstacle, corridor-wall
clearance, cross-track error, pass-region arrival, required heading change,
contact status, and dodge-phase validity. The removed 2018 model's acceptance
reports do not validate the v3 replacement; generate fresh v3 evidence before
making obstacle-avoidance acceptance claims.

The disturbed profile retains the stock `cf21B_500` mass, inertia, thrust,
and rotor dynamics while enabling sensor noise, the simulated Flow deck,
0.25 m/s wind, and light turbulence. Every new simulation writes
`run_config.json` with its complete configuration and artifact hashes.

- `espnet`/`tinyracer`: the finalized two-frame DroNet/gate float ONNX from
  perception commit `193fa12`; it supplies raw yaw/collision navigation plus
  corner heatmaps, gate mask, and gate presence. The file and its upstream
  contract are vendored under `models/espnet_dronet_gate_v1/`;
- `espnet-candidate`: the older August 19 float candidate, retained only for
  regression comparison;
- `sequential`: the previous single-frame deployment, retained for regression
  comparisons, with four metric clearances/confidences plus gate heatmaps;
- `stdc`: the earlier real-flight STDC deployment, with its dense danger map
  conservatively pooled into four symmetric regions plus gate corners;
- `dronet`/`dronet-v3`: the full PULP-DroNet v3 ResBlock model; steering and
  collision probability only. `dronet` is retained as the short CLI alias.

Use `--vision-model tinyracer-candidate`, `--vision-model sequential`,
`--vision-model stdc`, or `--vision-model dronet` for the self-contained
bundles. The explicit `-candidate` suffix prevents the staged ESPNet from
being mistaken for the finalized custom head. For example:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory straight --launch-prespin 1 \
  --vision-model tinyracer-candidate --vision-scene obstacle \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/tinyracer_obstacle \
  --overwrite
```

The candidate's portable graph is a deterministic ONNX export of the
published float student checkpoint. The NanoCockpit candidate is a
hybrid integer QAT/PTQ graph published as generated GAP8 C, not ONNX. The
simulator bundle pins the exact upstream commits, source checkpoint, hashes,
float metrics, hardware manifest, quantization constants, and both calibrated
thresholds. It deliberately labels the ONNX result as float-student inference;
it does not claim bit-exact equality with GAP8. Both use `[previous, current]`
frame order, and the first inference repeats the first frame just as the
firmware ring buffer does.

The older candidate is not the active baseline. Its upstream documentation reports
a 0.472 integer collision false-positive rate and large real-flight gate-
corner tail error. Use `dronet` as the published baseline, `tinyracer` for the
new portable multitask float graph, and exact GVSOC replay when bit-exact GAP8
integer behavior is required.

In the corrected gate-scene diagnostic, this candidate emitted collision risk
1.0 for every frame despite occasional valid gate locks. The controller now
correctly activates a front danger plane after two relevant-sector samples;
the candidate therefore stops instead of being allowed through by a control-
side loophole. This is why it remains explicitly staged.

Vision packet v3 carries normalized pinhole intrinsics (`fx`, `fy`, `cx`,
`cy`) alongside gate corners. This avoids silently applying the legacy
160 x 120 camera calibration to the candidate's calibrated 160 x 160 stream.
The firmware receiver remains backward compatible with v1/v2 packets.

The exact integer bundle pins perception deployment `60f82d9`, training workflow
`3854e33`, and NanoCockpit firmware `1cdd562`. Its input is HWC uint8
`160x160x2`, ordered `[previous,current]`. Its exact 2003-byte output contains
four 20x20 corner heatmaps, a 20x20 gate mask, gate presence, navigation yaw,
and collision logit. Replay a captured 51,200-byte tensor through the exact
generated network and weights with:

```sh
python3 tools/crazysim_mujoco/replay_gap8_generated.py \
  --input apps/controller_tinympc_eigen/sim_runs/crazysim/RUN/vision_frames/first_gate_input_hardware.raw \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/RUN/gap8_first_gate
```

This invokes the GAP SDK 3.8.1 GVSOC Docker image and writes `output_u8.bin`,
`gvsoc.log`, and a decoded `summary.json`. GVSOC startup takes roughly ten
seconds, so it is an exact selected-frame validator rather than the live 20 Hz
bridge. A matching float comparison is optional via `--float-release`; none is
invented for the finalized release.

Aggregate multiple replay summaries, deduplicated by exact input CRC, with:

```sh
python3 tools/crazysim_mujoco/summarize_gap8_parity.py \
  RUN_A/gap8_FRAME/summary.json RUN_B/gap8_FRAME/summary.json \
  --out replay_summary.json
```

For the finalized release this produces exact GAP8 observation statistics and
sets `float_parity_claimed` to false. If every input summary contains the older
candidate's optional float comparison, the same tool instead emits the legacy
float/GAP8 parity metrics. It rejects mixed sets rather than conflating the two.

Exact finalized-model replays are checked in under
`models/tinyracer_espnet_dronet_gate/metrics/`. On the simulated obstacle frame
the model reported collision 0.777 above its calibrated 0.494 threshold and
gate confidence 0.012. On the gate frame it reported gate confidence 0.979
above 0.617, four confident corners, and collision 0.013.

You may also pass a new model with `--vision-model PATH --vision-adapter NAME`.
Its complete parent bundle is mounted read-only so adjacent quantization
manifests remain available. All shipped adapter/runtime dependencies live in
this repository and Docker image. Vision is available on supported level-flight approach trajectories.

Vision runs additionally produce `vision.csv`, `vision.png`, and a
`vision_frames/` directory containing the full first camera frame plus the
exact first, first-critical, minimum-risk, and maximum-risk network inputs.
Spatial models rank the extrema by continuous raw sector probability rather
than their already-thresholded collision bit.
`validation.png` overlays model-appropriate activation samples and the physical
obstacle/gate on the top-down path only when the recorded DroNet-style
collision risk is above 0.77. Clearance and sector outputs remain visible in
`vision.png` and `vision.csv`, but no longer create ambiguous purple markers.
`vision.png` retains the raw ESPNet sector scores and its 0.33 float-model
threshold separately from the binary sector evidence sent to control. Purple
points show a neural activation; they do not falsely imply that a half-plane
was activated.
For obstacle scenes, `summary.json` additionally reports conservative vehicle-
envelope clearance, corridor-wall clearance, far-face passage, and a strict
`avoidance_success` boolean. Survival without passing the obstacle is not
counted as successful avoidance.

## Canonical racing suite

The current obstacle-only validation suite contains five repository-local
tracks. Gate geometry and gate-control logic are deliberately detached until
the final deployment head and matching real gate setup are ready:

- `canonical_corridor`: 5 m corridor with an offset box;
- `canonical_circle`: tangent-heading oval and a left-arc obstacle;
- `canonical_figure8`: smooth zero-speed-start figure-eight and lobe obstacle;
- `canonical_chicane`: slowed S-turn and post-turn obstacle;
- `canonical_hairpin`: slowed 180-degree turn with an exit obstacle.

Run all five headlessly with the finalized portable vision head:

```sh
python3 tools/crazysim_mujoco/run_canonical_acceptance.py \
  --out canonical_nominal \
  --profile nominal \
  --vision-model tinyracer \
  --overwrite
```

Each canonical trial renders `flight.mp4` by default. Use `--no-video` for a
faster data-only sweep. Videos run at 2x playback by default; a single run can
select real time with `run.sh --video --video-speed 1`.

Each trial writes `state.csv`, `vision.csv`, `firmware.log`, `summary.json`,
`validation.png`, `vision.png`, selected network-input images, and complete
artifact/configuration hashes. `acceptance.json` requires nonnegative
conservative obstacle/wall clearance, pass-region arrival, heading and final
cross-track limits, complete dodge phases, no contact.

The controller uses actuator-aware level TinyMPC with live vision throughout supported tracks.

The vision bridge applies a fixed one-camera-frame (50 ms at 20 Hz) delivery
delay indexed by camera sequence, rather than host inference completion time.

### Combined Flow deck and passive AI-deck camera capture

CrazySim external pose is the default state source. The simulated Flow deck is
currently opt-in with `--flowdeck` while its observation-model mismatch is
investigated.

Use `--flowdeck --camera-only` to pass the simulated Flow deck through to
CrazySim while also generating and recording an AI-deck-style 324 x 244
grayscale camera stream. `run_config.json` records `flowdeck_enabled`,
`camera_only_enabled`, `camera_capture_enabled`, and
`camera_inference_enabled` as first-class booleans. Passive capture writes
`camera.csv`, `camera_frames/first_camera_frame.{png,raw}`, and
`camera_frames/metadata.json`.

Camera-only mode is deliberately disconnected from flight control: it loads
no model, performs no inference, creates no firmware sender socket, and is
launched without a firmware destination. Consequently it cannot provide a
navigation observation, reference, constraint, or motor command. Existing
`--vision-model` runs retain their inference and firmware-packet behavior, and
the runner rejects combining `--camera-only` with `--vision-model`.

Each one-obstacle canonical route carries a coarse reference-index approach
window. The neural collision output still decides whether and when to dodge
inside it; the window prevents a background-high output elsewhere on the route
from consuming the sole encounter. The pass-distance counter begins only after
the full lateral bypass lane has been established. Curved routes retain their
time-consistent reference knot on rejoin, avoiding circle or figure-eight
branch jumps.

Current evidence is deliberately not presented as a 5/5 robust result. Nominal
corridor, circle, and chicane trials have completed without contact; the circle
retained 0.203 m conservative clearance after a 369 degree heading change. The
figure-eight now uses tangent heading and a rate-limited reference, but its
stable 0.50 m bypass misses the conservative obstacle envelope by about 0.003 m;
a 0.53 m command is outside the fixed model's repeatable stability envelope.
The hairpin's lane-held neural avoidance retains about 0.11 m clearance, but the fixed level model still diverges later in the high-yaw segment. Disturbed seeds remain unqualified. These
failures remain in the acceptance reports; substituting ground-truth detection
would hide the current perception/control boundary.

## Fidelity boundary

The controller source, TinyMPC core, 50 Hz scheduler, five ADMM iterations,
20-step horizon, generated reference, direct PWM output
are the same files used by the hardware build. There is no secondary
attitude/rate or position controller. CrazySim's
`cf21B_500` model independently supplies mass/inertia, thrust and torque curves,
motor lag, sensor transport, estimator execution, and contact physics.

Two SITL-only accommodations are explicit in the patch:

- The out-of-tree controller is selected at compile time and direct PWM is
  routed through CrazySim's power distribution interface.
- CrazySim's legacy gyro calibration gate is bypassed. It otherwise waits
  indefinitely with this deterministic sensor stream. The Kalman estimator and
  controller still run; this does not alter the hardware build.

An airborne pose handoff is triggered by the firmware's first nonzero PWM
packet. The default handoff starts the stock motor state at zero, so the
identified rotor lag is active from the first command; `--launch-prespin 1`
exists only for comparison with older staged runs. The harness also defaults to
the selected model's stock mass. Ground contact after handoff is classified as
a crash.
