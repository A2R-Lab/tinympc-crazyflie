# TinyMPC Maneuver Simulation

`run_maneuver_sim.py` is the vision-free simulation entry point for controller
and vehicle-dynamics work. It does not run a camera, ONNX inference, sector
logic, or perception-generated constraints. The earlier perception benchmark
and depth-constraint runner remain separate tools for later work.

```bash
python3 -m pip install -r tools/pybullet_simulation/requirements.txt

python3 tools/pybullet_simulation/run_maneuver_sim.py \
  --out sim_runs/maneuver_smoke \
  --duration 8 \
  --control-mode admm \
  --trajectory-file sim/trajectories/straight.csv \
  --plot \
  --overwrite
```

The first fixed-operating-point bank experiment uses three fully offline
TinyMPC bundles (level, 15-degree left, and 15-degree right):

```bash
python3 tools/firmware_codegen/generate_banked_model_bank.py
python3 tools/pybullet_simulation/generate_banked_turn_trajectory.py
python3 tools/pybullet_simulation/run_maneuver_sim.py \
  --out sim_runs/banked_turn_fixed_linearization \
  --duration 3.2 \
  --control-mode admm \
  --model-schedule bank15 \
  --trajectory-file sim/trajectories/banked_left_15.csv \
  --plot \
  --overwrite
```

The scheduler examines the reference roll near the middle of the MPC horizon.
It enters a bank bundle at 7.5 degrees and returns to level below 5 degrees.
Each bundle contains its own discrete `A`, `B`, affine residual, terminal
Riccati solution, feedback gain, and primal-recursion caches; no Riccati solve
or relinearization occurs at runtime. The maneuver runner also mirrors the
firmware's per-solve local frame: current position becomes the origin and
world yaw is removed from state and reference before TinyMPC is called.

The generated model bank also contains signed 30-degree and 60-degree bundles,
selected with `--model-schedule bank30` and `--model-schedule bank60`. Their
coordinated-turn operating speeds are 2 m/s and 5 m/s, respectively. The
60-degree schedule traverses the level, 15-degree, and 30-degree offline
bundles on entry before selecting the 60-degree bundle; it reverses that
sequence on exit instead of making one large level-to-60-degree model jump.
These schedules are currently host-side experiments: the onboard controller
still installs only `tinympc_generated_params.h`. Use `--model-schedule level`
when the objective is parity with the firmware that can be flashed today.

For an A/B comparison against the older global-state formulation, add
`--controller-frame world`. This deliberately supplies world position,
velocity, and full world-yaw attitude directly to the hover-linearized
controller. The default, `--controller-frame firmware-local`, matches
`updateInitialState()` in the firmware.

The maneuver runner rejects neural/perception arguments. Its default scene has
no obstacles. Explicit boxes may still be added to test physical collision,
but they do not produce controller constraints.

## Firmware reference parity

The maneuver runner defaults to the firmware's generated ADMM `rho=250`. Its
trajectory sampler follows `sampleTrajectoryReference()` by interpolating all
stored fields linearly and normalizing the interpolated quaternion. Input
references follow `updateHorizonReference()`: adjacent velocity samples define
specific-force magnitude, which scales hover thrust equally on all four
motors. Per-motor torque feedforward columns in a CSV are deliberately ignored.

Generate stationary yaw-boundary tests with:

```bash
python3 tools/pybullet_simulation/generate_yaw_sweep_trajectory.py \
  --out sim/trajectories/yaw_170.csv --yaw-deg 170
```

To reproduce the old world-frame widening-turn instability, sustain the bank
through a full revolution rather than stopping before the vehicle reaches the
problematic yaw:

```bash
python3 tools/pybullet_simulation/generate_banked_turn_trajectory.py \
  --out sim/trajectories/banked_left_15_360.csv --yaw-deg 360
python3 tools/pybullet_simulation/run_maneuver_sim.py \
  --out sim_runs/sustained_turn_ideal_world --duration 5 \
  --control-mode admm --model-schedule level --controller-frame world \
  --trajectory-file sim/trajectories/banked_left_15_360.csv --plot --overwrite
```

Run the same command with `--controller-frame firmware-local` and output to
`sim_runs/sustained_turn_ideal_local`, then create the focused comparison with
`plot_yaw_frame_instability.py`.

## Dynamics fidelity

The Gym/PyBullet adapter is parameterized from the same generated brushless
Crazyflie model used by TinyMPC:

- 45 g mass for AI deck + Flow deck;
- generated diagonal inertia;
- generated arm offset and yaw thrust-to-torque ratio;
- 9.81 m/s^2 gravity and generated hover thrust;
- physical motor-thrust bounds;
- world-frame position/linear velocity and body-frame angular velocity;
- Rodrigues attitude `q_xyz / q_w`, matching the generated model.

Run the quantitative one-step comparison with:

```bash
python3 tools/pybullet_simulation/validate_plant_fidelity.py \
  --out sim_runs/plant_fidelity/report.json
```

This checks hover, collective, roll, pitch, yaw, tilted collective, and coupled
rate/torque cases against the generated nonlinear equations.

## Current aggressive-maneuver boundary

The plant agrees closely with the generated rigid-body model, and the first
fixed 15-degree bank model bank is available for simulation experiments. It is
not a general trajectory-dependent relinearization scheme and it is not yet
suitable for flips. Rodrigues coordinates become singular at 180 degrees, so
flips require chart switching or a different attitude-error state rather than
applying the current chart globally.

With all robustness options left at zero or one, the plant intentionally
matches the generated mathematical model. Agreement with that ideal case is
therefore not proof of hardware fidelity for aggressive flight. The runner can
now add first-order motor lag, command and state-estimate delay, global and
per-motor thrust error, mass/inertia error, gym-pybullet-drones' generic
rotor-drag approximation, and white position/velocity/attitude/gyro measurement
noise. The rotor-drag coefficients were not identified on this 45 g
brushless/deck configuration. It
still does not model battery voltage sag, propeller inflow and blade flapping,
ground effect, flexible structure, estimator bias/drift, packet jitter, or
wind. Those effects need measured Crazyflie parameters before they should be
treated as predictive rather than as arbitrary failure injection.

Plant and estimator robustness can be exercised without changing the
controller model. This repeatable `realistic-v1` stress profile uses a 10 ms
motor time constant, a one-physics-step command delay, a 5% heavier plant, 10%
larger inertia, 5% lower global thrust effectiveness, fixed per-motor mismatch,
generic rotor drag, a 4 ms state-estimate delay, and modest white sensor
noise:

```bash
python3 tools/pybullet_simulation/run_maneuver_sim.py \
  --out sim_runs/bank60_realistic_v1 --duration 8.24 \
  --control-mode admm --model-schedule level \
  --trajectory-file sim/trajectories/banked_left_60_1080_5mps.csv \
  --motor-time-constant-ms 10 --motor-command-delay-ms 2 \
  --controller-compute-delay-ms 10 \
  --plant-mass-scale 1.05 --plant-inertia-scale 1.10 \
  --plant-thrust-scale 0.95 \
  --plant-motor-thrust-scales 0.98,1.01,0.97,1.00 \
  --rotor-drag-scale 1 \
  --state-estimate-delay-ms 4 \
  --position-noise-std-m 0.003 \
  --velocity-noise-std-mps 0.02 \
  --attitude-noise-std-deg 0.15 \
  --gyro-noise-std-deg-s 0.5 \
  --plot --overwrite
```

The firmware-parity solver defaults are `--mpc-rate-hz 50`,
`--tinympc-max-iter 5`, `--horizon 20`, and `--model-dt 0.02`. Thus there are
20 state knots (19 control intervals) and the last predicted state is 0.38 s
after the first knot. The plant integrates at 500 Hz; it does not solve the MPC
at the plant rate. `run_maneuver_sim.py` also reproduces the firmware's 0.4 s
trajectory-handoff hold. The realistic profile's 10 ms compute delay is the
rounded median of the recorded onboard solve latency (9.534 ms; observed range
6.518--11.718 ms in `capture_runs/admm_static_from_flow_01.csv`).

## Racing maneuver suite

Generate and test the racing-relevant maneuver set with:

```bash
python3 tools/pybullet_simulation/generate_racing_trajectories.py
python3 tools/pybullet_simulation/run_racing_maneuver_suite.py \
  --out sim_runs/racing_maneuver_suite_realistic_v1 \
  --profile realistic-v1 --overwrite
```

The suite contains a 90-degree sweeper, 180-degree hairpin, left-right chicane,
four-transition slalom, elevation-changing chicane, and a 5 m/s/60-degree
high-speed sweeper. Each case runs once with its signed stored-bank schedule
and once using only the level matrices. Runs are deliberately sequential to
avoid loading multiple native TinyMPC/PyBullet instances in one Python process.

Each maneuver directory contains `comparison.png`, the two ordinary top-down
plots, and the complete CSV logs. The suite root contains `summary.json` and
`results.csv`. In addition to collision and motor utilization, the report
measures position/altitude/roll error and the fraction of the flight within a
configurable centerline tolerance. The default racing criterion requires 95%
of samples within 0.30 m, reaching the final waypoint, no collision, and at
least 0.25 m altitude. This is a tracking criterion, not a claim that a
particular physical gate was cleared.

## Tier-3 TinyMPC acrobatics

Full roll, front flip, backflip, and forward barrel-roll primitives now use the
same TinyMPC ADMM solver as the ordinary simulation (50 Hz, 20 knots, five
iterations, rho 250):

```bash
python3 tools/pybullet_simulation/generate_acrobatic_trajectories.py
python3 tools/pybullet_simulation/run_acrobatic_suite.py \
  --out sim_runs/tier3_tinympc_suite --controller tinympc-acro \
  --profile realistic-v1 --overwrite
```

At each solve, position and velocity error are expressed in the current
reference frame. Attitude is the shortest-sign quaternion error
`q_ref^-1 * q_actual`, converted to the model's Rodrigues coordinates only
after recentering. Body-rate error is likewise referenced in the actual body
frame. TinyMPC therefore remains close to its zero-error hover chart even while
the absolute vehicle attitude crosses 180 degrees. The fixed offline hover
matrices and Riccati caches remain unchanged; no runtime relinearization or
Riccati solve is introduced.

The stored trajectory supplies feasible physical per-motor feedforward.
TinyMPC optimizes a feedback correction around that operating input, and its
input constraint is shifted each solve so `feedforward + correction` remains
within physical motor limits. The simulator still passes the result through
the shared saturation, delay, lag, mismatch, and PyBullet plant. The optional
`--controller geometric` path remains only as a comparison controller.

The generated primitives use a boost, low-thrust 0.8 s rotation, recovery, and
settle sequence. Peak reference rate is about 842 deg/s. The suite reports
integrated body rotation, quaternion error, position error, altitude, motor
clipping, recovery speed, and collision. Its `acrobatics.png` marks the old
180-degree chart singularity explicitly.

The current ideal and `realistic-v1` TinyMPC results both pass for the
360-degree roll and front flip. In the stressed profile they achieve 360.9 and
360.2 degrees, with 18.4 and 21.2 degree peak quaternion error, 0.049 and
0.074 m position RMSE, and 1.092 m minimum altitude. These are simulation
results, not authorization to fly the primitive without restrained test-stand
and flight-envelope validation.

The same error-state path is compiled onboard by selecting an acrobatic stored
trajectory:

```bash
python3 tools/pybullet_simulation/generate_acrobatic_trajectories.py \
  --firmware-header-dir src/trajectories/50hz
make TINYMPC_TRAJECTORY=roll_flip_360
# or: make TINYMPC_TRAJECTORY=front_flip_360
# or: make TINYMPC_TRAJECTORY=backflip_360
# or: make TINYMPC_TRAJECTORY=barrel_roll_forward_360
```

The default `make` continues to compile the circle. Acrobatic builds detach the
vision/racing halfspaces because those planes are expressed in the ordinary
local-position chart. Power loops, split-S turns, Matty flips, and a deliberate
inverted pause remain future reference primitives.

### Flow-deck altitude during inversion

The downward range is a slant distance and is rejected when body-z is more
than 35 degrees from world-z. During that outage, the acrobatic controller
propagates altitude with vertical velocity. Once the deck faces the floor, it
blends back to the estimator altitude with a 120 ms time constant. Every run
writes `flow_altitude.csv` and `flow_altitude.png`.

The simulator exposes `--flow-z-mode ideal|raw-range|freeze|inertial`.
`realistic-v1` uses `inertial`. In the current forward barrel-roll comparison,
raw slant range reaches the 4 m sensor cap and the vehicle collides; freezing
survives the barrel roll but produces 0.155 m position RMSE and 35.4 degrees
peak attitude error (and fails the backflip acceptance test). Inertial bridging
passes with 0.074 m position RMSE, 16.2 degrees peak
attitude error, and 0.045 m maximum altitude-estimation error.

Use the ordinary controller as a repeatable negative control:

```bash
python3 tools/pybullet_simulation/run_acrobatic_suite.py \
  --out sim_runs/tier3_tinympc_suite --controller admm \
  --profile ideal --overwrite
```

This deliberately omits the moving error chart and per-motor maneuver
feedforward. Both current cases must report `FAIL` and collision; if either
passes, the simulator/controller plumbing should be audited before trusting an
acrobatic result.
