# Simulation fidelity audit (2026-08-19)

## Verdict

The maneuver simulator is a high-fidelity shadow of the current TinyMPC
controller and its nominal 45 g rigid-body model. It is not yet a calibrated
digital twin of the physical Crazyflie for aggressive flight. Use it to catch
controller, timing, reference, coordinate-frame, saturation, and nominal
dynamics problems. Do not use it alone to certify a maneuver for hardware.

The current firmware installs only the level model in
`tinympc_generated_params.h`. Therefore `--model-schedule level` is the
firmware-parity setting. The banked schedules are host-only experiments until
the firmware explicitly installs and switches those matrices.

## What is matched

| Path | Audit result |
| --- | --- |
| TinyMPC implementation | The host bridge compiles the same `TinyMPC-ADMM` C++ solver sources and generated level matrices used by the firmware. |
| Solver configuration | 50 Hz solve rate, 20 knots, 0.02 s model step, five ADMM iterations, and rho 250. |
| Controller rate | The plant and held motor command run at 500 Hz; TinyMPC is not solved at 500 Hz. |
| State/frame conversion | Current position/yaw local frame, world linear velocity rotated into that frame, `q_xyz/q_w` Rodrigues attitude, and body angular rate match `updateInitialState()`. |
| Reference generation | Linear component interpolation, normalized linear quaternion interpolation, local-frame conversion, equal collective `Uref`, and the 0.4 s activation hold match the firmware path. CSV per-motor feedforward is deliberately ignored because the firmware ignores it. |
| Output pipeline | The first optimized motor-thrust delta is held until the next solve. A 10 ms delayed update models the rounded median measured onboard solve time. |
| Nominal plant | Mass 0.045 kg, diagonal inertia `[2.3951e-5, 2.3951e-5, 3.2347e-5]` kg m2, arm offset 0.03535 m, thrust coefficient `3.72e-8`, yaw-torque coefficient `7.73e-11`, gravity, saturation, and the firmware X-motor allocation. |
| Contact | PyBullet ground/obstacle collision is reported as a crash. |

`pycffirmware` cannot directly execute this out-of-tree controller: its stock
Python binding does not contain this app. The simulator therefore compiles the
same TinyMPC solver code and mirrors the app-level data path. FreeRTOS task
scheduling, the Crazyflie estimator, and the motor driver are represented by
explicit timing/actuator models rather than executed firmware binaries.

## Corrections made in this audit

1. Added the firmware's 0.4 s trajectory handoff hold.
2. Added asynchronous solver-output latency. The local flight log contains 479
   nonzero solve samples: 6.518 ms minimum, 9.534 ms median, 10.913 ms p90,
   11.401 ms p99, and 11.718 ms maximum.
3. Corrected the rotor-drag conversion. Gym's numeric motor action uses its
   stock thrust coefficient and is not a physical RPM; drag now uses physical
   angular speed reconstructed with the firmware thrust coefficient.
4. Marked bank-model scheduling as experimental and level matrices as current
   firmware parity.
5. Added native-process retries to the racing suite so an intermittent
   PyBullet crash cannot be reported as a controller collision.

## Validation results

The one-step nominal plant audit covers hover, collective, roll, pitch, yaw,
single-motor saturation, tilted collective, and coupled-rate cases. The maximum
error against the generated wrench/rigid-body equations was
`2.84e-14 m/s2` linear and `3.16e-13 rad/s2` angular (floating-point noise).
This proves nominal equation parity, not hardware parity.

The six current-firmware level-controller racing cases all remained bounded,
reached their endpoints, and had no collision. None met the deliberately strict
tracking criterion of staying within 0.30 m for 95% of the run:

| Maneuver | Ideal RMSE (m) | `realistic-v1` RMSE (m) | Realistic peak error (m) |
| --- | ---: | ---: | ---: |
| 90-degree sweeper | 0.418 | 0.482 | 0.739 |
| 180-degree hairpin | 0.237 | 0.274 | 0.510 |
| Chicane | 0.424 | 0.504 | 0.768 |
| Slalom | 0.387 | 0.473 | 0.653 |
| Elevation chicane | 0.450 | 0.538 | 0.827 |
| High-speed 180-degree sweeper | 0.592 | 0.657 | 1.384 |

`realistic-v1` is a repeatable robustness profile, not an identified hardware
parameter set. Its 10 ms motor lag, 2 ms command delay, 5% mass error, 10%
inertia error, 5% thrust loss, per-motor mismatch, generic rotor drag, 4 ms
state delay, and white measurement noise are plausible stress injections.

## Remaining hardware gap

The largest remaining gaps are the actual estimator (bias, drift, optical-flow
surface and lighting failures), battery/ESC/PWM behavior, motor-lag variation,
propeller inflow and blade flapping, ground and wall effects, wind, flexible
structure, contact geometry for the installed decks/guards, timing jitter, and
unmodeled lateral forces and torques. These dominate the uncertainty for flips
and high-rate racing maneuvers.

The next fidelity milestone should be log replay and residual identification:
record estimator state, normalized motor commands, battery voltage, and motion
capture ground truth from several maneuvers; replay identical references in the
simulator; then fit actuator, drag, delay, and process-noise parameters against
the residuals. Until that is done, successful simulation is a prerequisite for
flight testing, not proof that flight testing will succeed.

## Reproduce

```bash
python3 tools/pybullet_simulation/validate_plant_fidelity.py \
  --out sim_runs/final_parity_audit/plant_fidelity.json

python3 tools/pybullet_simulation/run_racing_maneuver_suite.py \
  --out sim_runs/racing_maneuver_suite_realistic_v1 --overwrite

python3 tools/pybullet_simulation/run_racing_maneuver_suite.py \
  --out sim_runs/racing_maneuver_suite_ideal --profile ideal --overwrite
```
