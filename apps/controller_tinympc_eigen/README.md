# TinyMPC Eigen Controller

This is the supported out-of-tree TinyMPC controller on `main`. It has been further modified to use Nanocockpit's aideck drivers to facilitate better communication with the aideck and aid in vision tasks.

## Build

Run the build from this directory:

Note: In order to use the new drivers, you must set `CONFIG_DECK_AI=n` in your app's app-config/oot-config to dropthe old aideck.c and CPX from the build. The old CPX protocol will no longer work as communication between STM and GAP8.

```bash
make clean
make
```

The generated firmware artifacts are written to `build/`.

## Generate TinyMPC firmware parameters

The controller consumes a fixed-size specialization generated from the current
upstream TinyMPC checkout. `TinyMPC` at the repository root must point to that
checkout, and Python must have `autograd` and `numpy` installed.

From `apps/controller_tinympc_eigen`, run:

```bash
python3 tools/firmware_codegen/tinympc_to_crazyflie_adapter.py
make
```

The adapter prompts for the Crazyflie type, attached decks, propeller guards,
horizon, model timestep, solve rate, ADMM iteration cap, half-space capacity,
and constrained portion of the horizon. Press Enter to accept any displayed
default. It then:

1. builds and linearizes the selected Crazyflie dynamics with Autograd;
2. calls current TinyMPC through `tools/firmware_codegen/tinympc_cpp_bridge.cpp` to precompute
   the Riccati and affine caches; and
3. writes `src/tinympc_generated_params.h` for the STM32 build.

The generated header is the source of truth for the runtime horizon, prediction
timestep, solve rate, iteration cap, A/B/f model, Q/R costs, bounds, hover input,
and solver caches. Do not edit it manually. Regenerate it whenever the vehicle
configuration or solver parameters change, then rebuild and check the reported
flash/RAM use before flying. The embedded half-space store supports at most 25
horizon knots; the adapter rejects larger configurations.

## Flash

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

## Controller Behavior

- The controller uses the Crazyflie out-of-tree app/controller build flow.
- `en_traj` in `src/controller_tinympc.cpp` is a compile-time flag, not a runtime Crazyflie parameter.
- On `main`, `en_traj` currently defaults to `true`, so the compiled trajectory
  is active after the controller handoff. Set it to `false` in source when the
  controller should follow commander/setpoints instead.

### Local MPC frame

Each solve uses a local NWU frame anchored at the current position and yaw.
States, references, and obstacle planes are transformed into this frame. The
ordinary racing path sends TinyMPC's first optimized input directly to the
motors. Acrobatic stored-LTV builds do the same; there is no secondary
attitude, rate, or position controller.
Trajectory headers store:

```text
[p_W(3), q_WB(wxyz), v_W(3), omega_B(3)]
```

The quaternion is converted to local Rodrigues coordinates for the 12-state
MPC. Reference yaw is derived and unwrapped online from the quaternion sequence.
Regenerate the headers with:

```bash
python3 tools/firmware_codegen/generate_crazyflie_trajectory.py
```

### Acrobatic TinyMPC build

The roll and flip trajectories use a reference-centered quaternion-error chart,
stored per-motor command feedforward, and horizon-wise offline LTV matrices.
The LTV state is the 12-state tracking error plus four normalized rotor-RPM
errors. It includes the measured 64.9 ms rotor response, RPM-to-thrust/torque
curves, and the firmware's nominal Crazyflie inertia. TinyMPC runs at
50 Hz with the usual 20-knot horizon and five ADMM iterations, then sends all
four optimized motor commands directly to PWM. No online relinearization,
Riccati pass, or high-rate corrective cascade is used.

```bash
python3 tools/pybullet_simulation/generate_acrobatic_trajectories.py \
  --firmware-header-dir src/trajectories/50hz
make TINYMPC_TRAJECTORY=roll_flip_360 TINYMPC_STORED_LTV=1
# Substitute front_flip_360, backflip_360, or barrel_roll_forward_360.
```

Regenerate one fully offline trajectory-linearized model sequence with:

```bash
python3 tools/pybullet_simulation/generate_stored_ltv.py \
  sim/trajectories/acrobatics/roll_flip_360.csv \
  --out sim_runs/stored_ltv_models/roll_flip_360.npz \
  --firmware-header src/trajectories/50hz/ltv/stored_ltv_roll_flip_360_50hz.h
make TINYMPC_TRAJECTORY=roll_flip_360 TINYMPC_STORED_LTV=1
```

Omitting `TINYMPC_STORED_LTV=1` deliberately selects the original 12-state
fixed hover matrices and is retained as a negative-control build.

The generator performs nonlinear differentiation and the time-varying Riccati
pass on the host. Firmware only indexes stored `A/B/affine/P_affine/K/Hinv` data; it
never relinearizes, factors a Hessian, or runs a Riccati recursion onboard.

Run the stressed headless validation before producing firmware:

```bash
python3 tools/pybullet_simulation/run_acrobatic_suite.py \
  --out sim_runs/tier3_tinympc_suite --controller tinympc-acro \
  --profile realistic-v1 --overwrite
```

Plain `make` still selects the conservative circle trajectory. The acrobatic
build is experimental and must be validated on a restrained test stand before
free flight. A current stationary-flip image uses about 79% flash, 87% RAM,
and 85% CCM; the longer barrel-roll image is close to the flash limit.

### Independent CrazySim/MuJoCo validation

For a second physics implementation that executes this controller inside the
Crazyflie firmware SITL, use the repository-local CrazySim harness:

```bash
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 \
  --stored-ltv 1 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_ltv \
  --overwrite
```

It pins and patches its dependency under `tools/crazysim_mujoco/.deps`, builds
in Docker, and writes MuJoCo ground truth, controller logs, crash metrics, and a
four-panel validation plot. The harness couples the POSIX firmware tick to the
measured physics rate and rejects timing-invalid runs. The revised backflip
passed three clock-valid clean repetitions plus individual sensor-noise and
Flow-deck trials. A severe combined plant/estimator/weather stress completes
without contact from 2.0 m, although its lateral drift remains too large for
racing. The fixed hover-matrix negative control crashes without completing the
rotation. See
`docs/crazysim_direct_motor_robustness_2026-08-19.md` for the results and
`tools/crazysim_mujoco/README.md` for the fidelity boundary.
