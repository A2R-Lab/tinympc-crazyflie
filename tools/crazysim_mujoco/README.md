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

For the full noisy validation window used during tuning:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 --stored-ltv 1 --duration 15 --sensor-noise \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_noise --overwrite
```

Each run produces `state.csv` (MuJoCo ground truth), `firmware.log`,
`simulator.log`, `summary.json`, and `validation.png`. The plot includes the
top-down path, altitude/crash marker, attitude, and all four motor speeds. The
summary also reports integrated maneuver-axis rotation, final horizontal error
and speed, saturation, and contact/crash status.

## Vision and DroNet baseline

The bundled DroNet baseline uses the exact weights published with *DroNet:
Learning to Fly by Driving*, converted once from Keras 2.0.2 to ONNX. It is
self-contained under `models/dronet/` and does not download code or models at
run time:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory straight --stored-ltv 0 \
  --launch-prespin 1 --vision-model dronet --vision-scene obstacle \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/dronet_obstacle \
  --overwrite
```

`--launch-prespin 1` models entry from an existing hover, which is the correct
initial condition for mid-flight obstacle avoidance. The zero-RPM handoff is
retained for deliberate motor-start stress tests. Camera rendering changes the
simulator/wall-time ratio; calibrate `--firmware-time-factor` if the run's
automatic launch-time check fails.

DroNet provides only steering and one collision score. It therefore exercises
continuous navigation and critical-probability stopping, but it does **not**
claim metric clearance, spatial sectors, or gate corners. The bridge adapters
preserve that distinction:

- `sequential`: four metric clearances/confidences plus gate heatmaps;
- `stdc`: four spatial danger regions plus gate corners;
- `dronet`: steering and collision probability only.

Pass a new model with `--vision-model PATH --vision-adapter NAME`. The model may
live outside the checkout; it is mounted read-only into the container, while
all adapter/runtime dependencies remain inside this repository and Docker
image. Vision is rejected for stored flip/roll runs. It is active only on
ordinary approach trajectories, and an acrobatic handoff must latch vision out
until the primitive has completed and the altitude/attitude estimator has
recovered.

Vision runs additionally produce `vision.csv` and `vision.png`.
`validation.png` overlays neural-risk samples and the physical obstacle/gate on
the top-down path. Purple points mean reported collision risk at or above 0.25;
they do not falsely imply that a half-plane was activated.

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
