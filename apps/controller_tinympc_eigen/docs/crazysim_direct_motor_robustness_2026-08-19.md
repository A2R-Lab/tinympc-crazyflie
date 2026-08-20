# Direct-motor TinyMPC CrazySim robustness audit

Date: 2026-08-19

This audit executes the actual 50 Hz, 20-knot, five-iteration firmware
controller in CrazySim/MuJoCo. It uses horizon-wise matrices and Riccati data
generated entirely offline, stock `cf21B_500` mass and dynamics, zero RPM
pre-spin, and direct motor commands. No stabilizing cascade is inserted between
TinyMPC and the motors.

## Problems found and corrected

The original backflip reference demanded roughly 0.18 N per motor for 0.75 s,
rose to nearly 8 m, and left very little differential motor authority. The
stationary flip is now a compact 0.60 s boost, 0.60 s rotation, and 0.60 s
recovery sequence. Its reference spans approximately 1.09--3.90 m, commands a
1122 deg/s peak body rate, and preserves enough motor headroom for attitude
correction. All trajectory and stored-LTV artifacts were regenerated from that
definition.

The more fundamental defect was clock skew. CrazySim advances fixed 1 ms physics
steps but the firmware's POSIX FreeRTOS port originally ticked from wall time.
On this host the simulator achieved about 0.07x wall speed even when capped at
0.1x, so the nominal 50 Hz controller advanced substantially faster than the
plant. Launch times and outcomes changed with host load. Those failures were not
valid controller evidence.

The harness now:

- binds the simulator before booting the firmware;
- scales the POSIX FreeRTOS tick to the measured physics/wall-time rate; and
- rejects any run whose motor launch differs from the requested simulation time
  by more than 0.5 s.

The default calibration uses `--realtime-factor 0.1` and
`--firmware-time-factor 0.07`. It must be rechecked on a substantially different
host.

## Clock-valid results

All cases use a 4 s requested launch and a 10 s simulation. Nominal, noise, and
Flow-deck cases enter at 1.5 m.

| Case | Contact | Rotation | Altitude range | Maximum horizontal displacement | Maximum rate | Motor saturation |
|---|---:|---:|---:|---:|---:|---:|
| Clean repeat 1 | none | -356.69 deg | 1.30--3.74 m | 0.40 m | 21.69 rad/s | 0.015% |
| Clean repeat 2 | none | -360.98 deg | 1.18--3.77 m | 0.20 m | 21.42 rad/s | 0.068% |
| Clean repeat 3 | none | -362.88 deg | 1.29--3.76 m | 0.24 m | 21.40 rad/s | 0.050% |
| Seeded IMU/barometer noise | none | -358.85 deg | 1.21--3.76 m | 0.45 m | 21.92 rad/s | 0.053% |
| Flow deck only | none | -363.74 deg | 1.28--3.76 m | 0.14 m | 23.45 rad/s | 0.180% |
| Fixed hover-matrix negative control | **ground contact** | -1.97 deg | -0.02--4.67 m | 0.60 m | 41.06 rad/s | 1.00% |
| Combined stress, 1.5 m entry | **ground contact** | -359.35 deg | 0.02--2.26 m | 1.48 m | 24.25 rad/s | 5.01% |
| Combined stress, 2.0 m entry | none | -358.98 deg | 0.32--2.68 m | 1.79 m | 24.57 rad/s | 6.01% |

The combined stress applies all of the following at once: seeded sensor noise,
Flow-deck estimation, 0.75 m/s wind, light turbulence, 0.047718 kg mass, 1.15x
diagonal inertia, 1.25x motor time constant, and 0.90x realized thrust. The
1.5 m trial completed the flip before contacting the ground. The 2.0 m trial
completed without contact, but its 1.79 m drift and 0.52 m final altitude make
it a bounded-survival result rather than acceptable racing accuracy.

An attempted increase of the vertical-position and vertical-velocity costs was
rejected: it raised saturation to 19.7%, completed only about half a rotation,
and crashed. The final generated controller retains the original balanced cost
weights.

## Interpretation

With clocks coupled correctly, the revised direct-motor controller is repeatable
in the nominal model and remains bounded under the individual sensor-noise and
Flow-deck tests. The earlier catastrophic noise/Flow failures were dominated by
the invalid clock relationship, not by a demonstrated estimator failure.

The fixed hover-matrix negative control uses the same acrobatic reference and
motor feedforward but disables the stored horizon-wise model sequence. It
produced essentially no net flip, exceeded 41 rad/s, and contacted the ground.
This confirms that the successful runs depend on the acrobatic LTV controller,
not merely on the reference or permissive simulator dynamics.

The combined test defines a real limitation. Large simultaneous thrust, mass,
inertia, motor-lag, estimator, and weather errors consume vertical recovery
height and produce substantial lateral drift. More aggressive vertical weights
are not a principled cure. A hardware program should enforce an entry-altitude
envelope and separately identify mass, thrust scale, and motor response before
attempting the maneuver.

## Reproduction

Outputs are under `apps/controller_tinympc_eigen/sim_runs/crazysim/`. Each run
contains `summary.json`, `state.csv`, firmware and simulator logs, and
`validation.png`.

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 --stored-ltv 1 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/backflip_ltv \
  --overwrite
```

The combined 2 m test is:

```sh
tools/crazysim_mujoco/run.sh \
  --trajectory backflip_360 --stored-ltv 1 --duration 10 --launch-time 4 \
  --spawn-z 2.0 --sensor-noise --flowdeck --wind-speed 0.75 \
  --turbulence light --mass 0.047718 --inertia-scale 1.15 \
  --motor-tau-scale 1.25 --thrust-scale 0.90 --random-seed 7 \
  --out apps/controller_tinympc_eigen/sim_runs/crazysim/combined_2m \
  --overwrite
```

## Decision

The simulator/controller integration issue is fixed, and the direct-motor
architecture now passes a meaningful clock-valid simulation gate. It is still
**not ready for unrestricted real-drone testing**. The next safe step is a
restrained, progressively expanded hardware test beginning with motor-map and
timing validation, then sub-flip attitude steps, with a 2 m minimum flip entry
height until the plant uncertainty is measured more tightly.
