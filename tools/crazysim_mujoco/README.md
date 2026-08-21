# TinyMPC validation in CrazySim/MuJoCo

This integration runs the repository's actual out-of-tree Crazyflie controller
and TinyMPC solver inside CrazySim SITL. MuJoCo supplies rigid-body dynamics,
the CrazySim brushless motor model, contacts, and simulated sensors. An optional
offscreen FPV camera feeds a repository-local ONNX bridge, which sends the same
versioned perception observation consumed by the firmware controller.
The PWM bridge applies the firmware's quadratic normalized-speed law
(`thrust = command^2 * 0.312852 N`), then CrazySim independently converts that
thrust target through its measured RPM polynomial, 65 ms rotor dynamics, and
0.200 N `cf21B_500` motor/propeller limit.

The dependency is cloned under this directory at a pinned commit and patched at
setup time. Nothing is loaded from `tinympc-vision`, `tinympc-perception`, or a
sibling CrazySim checkout. The build and Python environment run in Docker.

CrazySim and the POSIX firmware do not naturally share a simulation clock: the
physics loop advances fixed 1 ms steps while the firmware's FreeRTOS port ticks
from wall time. The harness therefore runs the physics with a 0.1 real-time cap
and calibrates the firmware tick to the measured 0.07 simulation/wall-time rate
on the validation host. A run is rejected if the first motor command occurs
more than 0.5 s from the requested simulation launch time. Use
`--firmware-time-factor` to recalibrate this value when moving to a materially
different host.

## Run

From the repository root:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 \
  --stored-ltv 1 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_ltv \
  --overwrite
```

Compare against the original fixed dynamics matrices:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 \
  --stored-ltv 0 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_fixed \
  --overwrite
```

Other available references are `front_flip_360`, `roll_flip_360`, and
`barrel_roll_forward_360`. Run `tools/crazysim_mujoco/run.sh --help` for noise,
Flow-deck, wind, ground-effect, plant-parameter, and timing options. The default
1.5 m handoff altitude is the validated nominal entry height; the reference is
anchored to the actual handoff pose.

For racing, use `barrel_roll_forward_360` as a declared track primitive rather
than triggering a flip directly from DroNet collision risk:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory barrel_roll_forward_360 --stored-ltv 1 --duration 16 \
  --launch-time 4 --spawn-z 1.5 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/racing_barrel_roll \
  --overwrite
```

The primitive carries up to 0.54 m/s forward speed, advances 1.45 m, completes
one body-x roll, and returns its reference to the entry altitude. Its energy
management arc needs 2.26 m of overhead clearance, so the track planner must
reserve that corridor before selecting it. The runner independently regenerates
the CSV, firmware reference, and all horizon-wise stored LTV matrices before an
acrobatic build; stale or mismatched artifacts stop the run. The final report
marks `acrobatics_success` only with no contact, a complete rotation, upright
recovery, and at most 0.50 m terminal position error.

The division of responsibility is deliberate: DroNet chooses a banked lateral
dodge for an unstructured obstacle because its output has no obstacle height or
free-space volume. A track declaration may select the barrel roll when its full
3-D corridor is known. The current harness uses separate level/vision and
acrobatic firmware builds, so camera inference is disabled for the whole stored
primitive. A future runtime track sequencer must disable vision before the
rangefinder tilts away from the ground and reacquire it only after the stored
maneuver reaches its upright recovery segment.

For the full noisy validation window used during tuning:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 --stored-ltv 1 --duration 15 --sensor-noise \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_noise --overwrite
```

Each run produces `state.csv` (MuJoCo ground truth), `firmware.log`,
`simulator.log`, `summary.json`, and `validation.png`. Add `--video` to also
produce `flight.mp4`, a post-run 3-D replay of the logged ground-truth pose.
Because it is rendered after the simulator exits, encoding cannot change the
controller, vision, or physics timing. The plot includes the
top-down path, altitude/crash marker, attitude, and all four motor speeds. The
summary also reports integrated maneuver-axis rotation, final horizontal error
and speed, saturation, and contact/crash status.

### Level-route reference comparison

Figure-eight, oval, and circle can be flown with three matched reference
policies:

```sh
tools/crazysim_mujoco/run.sh --trajectory figure8 \
  --reference-mode waypoint --duration 130 --stop-on-contact \
  --random-seed 1 --out apps/controller_tinympc_eigen/sim_runs/crazysim/example
```

`waypoint` holds discrete route goals, `progress` builds a smooth horizon from
local geometric path progress, and `trajectory` samples the dense route by
elapsed time. All three use the same trajectory artifact, plant, controller
costs, local-frame transform, and launch protocol. Yaw references are tracked
inside TinyMPC; no post-solve outer yaw correction is applied.

The reproducible benchmark runs all 27 shape/mode/seed combinations and retains
contact failures as evidence:

```sh
python3 tools/crazysim_mujoco/run_reference_benchmark.py
```

Its output directory contains per-run raw logs and summaries plus
`comparison.json`, `comparison.csv`, `comparison.png`, `flight_paths.png`, and
`REPORT.md`. Common geometric metrics are truncated at first contact. Dense
trajectory runs additionally report errors against their wall-clock schedule.

## Vision and DroNet baseline

The bundled baseline uses the full PULP-DroNet v3 weights published for the
AI-deck/Crazyflie, converted once to ONNX without retraining. It is
self-contained under `models/dronet/` and does not download code or models at
run time. The camera path also matches deployment: a 324x244 Himax-style
grayscale frame followed by the bottom-centered 200x200 crop:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory straight --stored-ltv 0 --duration 18 \
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
racing-specific banked dodge: two risk samples at or above 0.25 latch the
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
contact status, and dodge-phase validity. The checked-in nominal evidence
also requires at most 0.15 m final cross-track error. It passes 4/4 with
minimum obstacle clearances 0.055, 0.278, 0.054, and 0.197 m;
see `models/dronet/crazysim_multicourse_acceptance.json`.

The disturbed profile retains the stock `cf21B_500` mass, inertia, thrust,
and rotor dynamics while enabling sensor noise, the simulated Flow deck,
0.25 m/s wind, and light turbulence. The earlier single-obstacle disturbed
gate remains checked in as `models/dronet/crazysim_acceptance.json`: seeds 1,
7, and 19 passed 3/3, with minimum obstacle and wall clearances 0.261 m and
0.361 m. Every new simulation writes `run_config.json` with its complete
configuration and artifact hashes.

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
- `dronet`: steering and collision probability only.

Use `--vision-model tinyracer-candidate`, `--vision-model sequential`,
`--vision-model stdc`, or `--vision-model dronet` for the self-contained
bundles. The explicit `-candidate` suffix prevents the staged ESPNet from
being mistaken for the finalized custom head. For example:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory straight --stored-ltv 0 --launch-prespin 1 \
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
this repository and Docker image. Vision is rejected for stored flip/roll
runs. It is active only on ordinary approach trajectories, and an acrobatic
handoff must latch vision out until the primitive has completed and the
altitude/attitude estimator has recovered.

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

- `canonical_corridor`: 5 m corridor, offset box, and forward barrel roll;
- `canonical_circle`: tangent-heading oval and a left-arc obstacle;
- `canonical_figure8`: smooth zero-speed-start figure-eight and lobe obstacle;
- `canonical_chicane`: slowed S-turn and post-turn obstacle;
- `canonical_hairpin`: slowed 180-degree turn, exit obstacle, and backflip.

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
cross-track limits, complete dodge phases, no contact, and—where scheduled—
matching firmware events plus a physics-derived 315–430 degree rotation and
recovery.

The hybrid controller uses the fixed actuator-aware level TinyMPC and live
vision during ordinary track segments. During a scheduled primitive it latches
vision out, switches to the existing offline horizon-wise stored-LTV direct-
motor controller, applies the rangefinder-invalid altitude freeze/blend, then
rejoins the nearest forward level-track knot. It does not perform online
relinearization or online Riccati updates.

The vision bridge applies a fixed one-camera-frame (50 ms at 20 Hz) delivery
delay indexed by camera sequence, rather than host inference completion time.
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
The hairpin's lane-held neural avoidance retains about 0.11 m clearance, but
the fixed level model still diverges later in the high-yaw segment before a
valid stored-LTV backflip handoff. Disturbed seeds remain unqualified. These
failures remain in the acceptance reports; substituting ground-truth detection
would hide the current perception/control boundary.

## Fidelity boundary

The controller source, TinyMPC core, 50 Hz scheduler, five ADMM iterations,
20-step horizon, generated reference, direct PWM output, and stored LTV
matrices are the same files used by the hardware build. There is no secondary
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

The acrobatic optimizer has 16 states: 12 reference-centered rigid-body errors
plus four normalized rotor-RPM errors. Its offline model uses CrazySim's
64.9 ms RPM response, RPM-to-thrust/torque curves, nominal inertia, and
the rigid-body equations. The four optimized commanded-thrust corrections go directly
to the firmware PWM conversion. Motor RPM is not measured on the current
hardware path, so firmware propagates the same four-state first-order estimate
from its previous commands.

A stock-mass/no-pre-spin backflip completed three of three clock-valid clean
repetitions, plus separate seeded sensor-noise and Flow-deck trials. A severe
combined case—10% thrust loss, 10% extra mass, 15% extra inertia, 25% slower
motors, noise, Flow, wind, and turbulence—completed the rotation without contact
from a 2.0 m entry, but contacted the ground from 1.5 m and accumulated about
1.8 m of lateral drift. See
`apps/controller_tinympc_eigen/docs/crazysim_direct_motor_robustness_2026-08-19.md`
for the complete matrix. This establishes a simulation envelope, not approval
for unrestricted flight testing.

As a negative control, the same trajectory and motor feedforward with the stored
LTV sequence disabled achieved only -1.97 degrees of net rotation and contacted
the ground. The successful flip therefore is not a feedforward-only artifact.
