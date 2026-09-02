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

The visual residual policy remains opt-in.  To compile its V4 reference-rate
receiver and TinyMPC reference integration for the hardware target, use
`make TINYMPC_VISION_RL_RESIDUAL_ENABLE=1`; the default is `0` and preserves
the existing reference path. Its physical authority is centralized in
`src/tinympc_vision_residual_authority.h` as contract version 1. The build
exposes corresponding `TINYMPC_VISION_RESIDUAL_*` limit variables, but their
defaults use the controller-wide maximum lateral rate: lateral/vertical rates
of `2.25/0.20 m/s`, gate/full lateral offsets of `0.25/0.45 m`, and vertical
offset of `0.25 m`. Reactive left/right avoidance uses that same
`TINYMPC_MAXIMUM_LATERAL_RATE_MPS` variable; the ESPNet state-machine profile
triggers at `0.80` risk and caps its path-relative offset at `1.00 m`. Once an
avoidance side is selected, it completes the full out-and-back maneuver even
if risk clears, changes sides, navigation becomes stale, or center risk rises;
emergency braking can only begin from `TRACK`. A larger profile requires new
closed-loop flight evidence; the retained residual-policy matrix did not pass
behavioral acceptance.

NanoFlowNet-style vertical active sensing is also a separate, opt-in nominal
reference overlay. Enable it with
`make TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE=1`. Its project defaults are an
additive altitude square wave of `+/-0.10 m`, a `2.0 s` period, and an initial
low phase measured from controller handoff. The learned V4 vertical residual
is not used to create this motion. Amplitude and period can be overridden with
`TINYMPC_VERTICAL_ACTIVE_SENSING_AMPLITUDE_M` and
`TINYMPC_VERTICAL_ACTIVE_SENSING_PERIOD_S`.

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

### Precomputed turn and braking caches

The direct actuator-aware controller atomically selects complete offline
TinyMPC bundles from `src/tinympc_banked_model_bank.h`; it never interpolates
matrices or runs a Riccati solve in firmware. In addition to the level and ten
signed coordinated-turn bundles, the bank contains five straight-line phase
points for the 6 m/s^2 braking schedule:

| Forward speed | Exact drag-aware pitch |
| ---: | ---: |
| 1.0 m/s | -29.297591 deg |
| 1.5 m/s | -28.185407 deg |
| 2.0 m/s | -27.049596 deg |
| 2.5 m/s | -25.890320 deg |
| 3.0 m/s | -24.707822 deg |

The braking selector requires the explicit 6 m/s^2 reference, near-level
reference and measured roll, and local measured pitch/speed before switching.
Consequently a pitch-only bundle cannot displace a coordinated-turn bundle
during combined roll/pitch motion. Every switch resets the optimizer and uses
100 ms dwell plus speed-tier hysteresis. The straight braking horizon slews
pitch at no more than 4 rad/s to prevent an unphysical one-knot attitude/rate
step while still converging to the exact cached pitch. Regenerate the header
and provenance instead of editing either artifact:

```bash
python3 tools/firmware_codegen/generate_banked_model_bank.py
```

The optional identified rate-cascade path retains its separate coordinated-
turn-only model bank; the braking bundles apply to the default direct actuator
path (`TINYMPC_RATE_CASCADE=0`, `TINYMPC_ACTUATOR_LTI=1`).

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
motors. There is no secondary attitude, rate, or position controller.

### Progress-indexed path following

`TinyMpcProgressPath` projects measured position onto a bounded local window of
the dense route and maintains a monotonic fractional route coordinate. It caps
projection advance by physical distance, slows from 0.15 m/s toward 0.05 m/s
as curvature increases, and walks forward by arc length to build each MPC
horizon. References include interpolated position, tangent velocity, bounded
tangent yaw and yaw rate, and curvature-derived roll/pitch and collective
feed-forward. Because advancement is spatial rather than clock-driven, a slow
or disturbed vehicle never receives a catch-up command. Completion is enabled
only near the terminal path window, so a closed route's colocated start and end
cannot finish immediately.

Progress following is the sole level-route reference policy. There is no
reference-mode build switch; `run.sh` always builds this policy. Run the
CrazySim figure eight with:

```bash
tools/crazysim_mujoco/run.sh \
  --trajectory figure8 --actuator-lti 1 \
  --duration 130 --launch-time 2 --vision-scene none \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/progress_figure8 \
  --overwrite
```

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
