# TinyMPC Eigen Controller

This is the supported out-of-tree TinyMPC controller on `main`.

## Build

Run the build from this directory:

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
python3 tools/tinympc_to_crazyflie_adapter.py
make
```

The adapter prompts for the Crazyflie type, attached decks, propeller guards,
horizon, model timestep, solve rate, ADMM iteration cap, half-space capacity,
and constrained portion of the horizon. Press Enter to accept any displayed
default. It then:

1. builds and linearizes the selected Crazyflie dynamics with Autograd;
2. calls current TinyMPC through `tools/tinympc_cpp_bridge.cpp` to precompute
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
- On `mpc-hover`, `en_traj` is now `true` and the regenerated small-circle header
  is selected. Set it to `false` to return to commander hover mode.

### Local-frame hover port

This branch starts from the hardware-tested `fcef0e0` controller and ports the
world-to-local coordinate math from `0091e2c`. Each 100 Hz solve captures current
position and quaternion yaw. Position targets are translated and yaw-rotated;
world velocities are yaw-rotated; measured and reference attitudes have that
same yaw removed before Rodrigues conversion. Body angular rates and motor
thrust inputs keep their original coordinates. World z remains vertical.

Commander positions and yaw remain absolute world targets; this transform does
not automatically capture a hover target or change commander mode semantics.
The existing 12-state stored trajectory is also transformed on every solve,
including when its selected world point is held. `MPC local` console positions
are therefore zero and references are local offsets, not world coordinates.

The synchronous direct-motor output, generated model/gains, 100 Hz solve rate,
20 ms model step, 25 knots and two ADMM iterations are retained. The port does
not import the source commit's PID cascade, background task or new trajectories.
Input-only ADMM has no state slacks/duals to reset; state calculations are
overwritten each solve and per-motor input warm starts remain frame-independent.

Validation: `python3 tests/test_local_frame.py` compiles production transform
and reference functions with the pinned math3d and Eigen headers under address
and undefined-behavior sanitizers. It checks translation/yaw invariance, body
rate preservation, quaternion sign and yaw-wrap behavior, and held-reference
refresh. `make cf21bl_defconfig` followed by `make -j4` builds Brushless firmware.
The coordinate port passed the Brushless build and host tests; it has not been
flight-tested.

### Regenerated small circle

`src/traj_circle_small.h` contains a 0.5 m radius circle centered at world
(0,0), at z=0.5 m, with fixed world yaw zero. One counterclockwise lap takes
5 seconds at 100 Hz: 501 state samples include the closing point and 500 input
samples cover the intervals. Speed is 0.6283185 m/s. Regenerate with:

```sh
python3 tools/generate_circle_reference.py
python3 tests/test_circle_reference.py
```

The generator requires NumPy and SciPy. It reconstructs the continuous hover
model from the current generated A/B matrices and solves a periodic reference
for position, Rodrigues attitude, velocity, body rates and motor thrust
deviations. Inputs are encoded in the controller's legacy normalized-command
offset convention. The adjacent provenance JSON records the model hash,
settings and continuous/discrete residuals. No MPC matrices are regenerated.
These are linearized hover references, not exact nonlinear flight dynamics.

The circle is selected for the firmware build. The controller consumes all
samples through the closing point, then holds its position with level attitude,
zero velocity/body rates and zero thrust-deviation feedforward. This is an
abrupt reference change, not a generated deceleration ramp or automatic landing.
The periodic reference also starts with nonzero velocity; no entry ramp was
added. Progress begins when the OOT controller is selected, including while
disarmed. The local frame conversion does not shift the circle's absolute
world center. No circle flight validation has been performed.
