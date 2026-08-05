# 15 Hz GAP8/STM32 equivalence replay

This harness checks the alternating 15 Hz optical-flow path at two compiled
firmware boundaries:

1. GAP8 camera frames to nine filtered flow sectors.
2. STM32 flow sectors plus vehicle state to the persistent obstacle cylinder.

The GAP8 host executable is generated from verbatim named functions in
`tinympc-nanocockpit/src/gap/examples/pulp-frontnet/main.c`. Platform timing,
camera-health queries, and memory-placement annotations are stubbed; the
feature selection, pyramidal LK, aggregation, and persistent filter functions
are compiled from production source.

Recorded native Himax captures can be replayed through the same compiled
boundaries with:

```sh
python3 tools/flow_gap8_validation/equivalence/replay_himax_capture.py /path/to/capture \
  --nanocockpit /path/to/tinympc-nanocockpit
```

The replay uses native PNG or raw PGM grayscale samples unchanged and applies
the production accumulated 66,667 us selection deadline. Its coverage section
must be checked: Wi-Fi capture losses mean that a saved stream with non-nominal
image gaps is diagnostic rather than an exact reconstruction of the on-board
frame sequence.

The STM32 executable directly includes
`apps/controller_tinympc_eigen/src/flowdeck_obstacle_link.c`. Only the RTOS
clock and log-registration macros are mocked.

The deterministic corpus contains 204 160x160 frames in 17 independent
sequences at 15 Hz:

- starting distances: 0.75, 1.5, and 2.5 m;
- relative obstacle/view orientations: -45, -20, 0, 20, and 45 degrees;
- lateral offsets: -0.3 and 0.3 m.

An angled view of the fixed world box is geometrically equivalent to rotating
the box by the opposite yaw relative to the camera. Every compiled and Python
path receives the identical uint8 frame bytes and state rows.

Run:

```sh
python3 apps/controller_tinympc_eigen/tools/flow_gap8_validation/equivalence/run_15hz_equivalence.py \
  --nanocockpit /home/cchen/tinympc-nanocockpit \
  --out apps/controller_tinympc_eigen/tools/results/flow_15hz_equivalence.json
```

The command exits nonzero for any validity mismatch or any numeric difference
greater than `2e-5`.

## Pinned passing result

The checked-in result pins:

- `tinympc-crazyflie`: `c12edaf5d8edf459c4a4b9e81f7fb208d45493a8`
- `tinympc-nanocockpit`: `1e5cf0dc08e306f1849f20d15875c676d7e11e22`

All 7,344 GAP8 sector fields matched. Maximum error was `4.72e-7`.
All STM32 validity flags and compared estimator values matched. Maximum error
was `7.19e-7`.

This proves numeric equivalence of the 15 Hz perception and obstacle-estimator
pipeline for the covered inputs. It does not prove camera calibration,
real-camera image fidelity, flight dynamics, TinyMPC control equivalence, or
safe physical flight.

## Detection-quality finding

Equivalence does not imply adequate detection. The compiled pipeline detected
16 of 17 corpus cases. It missed the 2.5 m, -20-degree case during the
12-frame/0.8-second observation window. That case produced a qualifying
two-sector cluster only once, so the STM32 map never reached its required two
accepted observations.

The safest firmware improvement to evaluate next is a bounded temporal
`N-of-M` candidate accumulator (for example, two qualifying clusters in three
new samples) instead of requiring effectively consecutive cluster evidence.
It should retain the map's two-observation validity requirement. Evaluate this
against negative/no-obstacle, repeated-texture, pure-yaw, and low-texture
corpora before deployment. Increasing GAP8 features or accepting a
single-sector cluster may improve recall, but has larger timing or false-positive
risks and should not be the first change.

The pre-existing seven-scene frontend suite also passes only two scenes under
the now source-identical model. Off-axis, two-depth, narrow, and far scenes
have errors above 0.35 m, and the nominally non-detectable low-texture scene
produces a false cylinder. This is stronger evidence that firmware changes
need a precision/false-positive benchmark, not only a recall improvement.

Recommended change sequence:

1. Add `N-of-M` temporal cluster persistence and explicit negative-scene tests.
2. Add motion-observability gating using minimum aggregate track displacement
   and consistency between parallax and looming estimates.
3. Replace the single best contiguous cluster with a small robust
   bearing/range consensus so foreground and background sectors do not merge.
4. Only then evaluate 36 features (four per sector) against the measured 15 Hz
   GAP8 timing budget.

## Exact per-track image simulation

The current planner path is validated with:

```sh
python3 tools/flow_gap8_validation/equivalence/run_exact_track_image_sim.py \
  --nanocockpit /home/cchen/tinympc-nanocockpit
```

This is the authoritative software regression for the new path. Python only
renders deterministic calibrated images and orchestrates the executables. The
GAP8 executable is built from verbatim function bodies extracted from
`pulp-frontnet/main.c`; the STM32 executable directly includes the production
`flowdeck_obstacle_link.c`.

The suite covers nominal and insufficient peering, no motion, sensor
degradation, timestamp bias, mixed foreground/background depth, low texture,
looming without lateral baseline, centered obstacles at 0.8 m and 1.4 m, and
obstacles offset 0.2 m left and right. The offset cases exercise both peering
directions; the 1.4 m case uses a larger 0.40 m sweep because its parallax is
smaller. See
`../../docs/track_perception_validation_2026-07-23.md` for the pinned findings
and limitations.

`run_15hz_equivalence.py` remains useful for exposing drift in the older Python
sector mirror. It is not the validation authority for the per-track planner
path and is expected to fail while that mirror differs from production.

## Exact 1 m/s race simulation

Run the no-peering race corpus with:

```sh
python3 tools/flow_gap8_validation/equivalence/run_exact_track_race_sim.py \
  --nanocockpit /home/cchen/tinympc-nanocockpit
```

Like the image regression above, this compiles the GAP8 functions extracted
verbatim from production `main.c` and directly includes the production STM32
estimator. It does not use the older Python estimator mirror.

The 14 deterministic cases use 160x160 calibrated uint8 images at the deployed
15 Hz cadence. Every obstacle approach travels at 1 m/s with zero lateral
peering. Coverage includes centered, narrow, left/right offset, cluttered,
three texture/noise/exposure/forward-blur variants, both gentle-yaw directions,
and a 5 ms timing-jitter sequence with one doubled capture interval. Empty
clean/degraded backgrounds and pure yaw are required negatives.

An obstacle case passes only if:

- the first cylinder is published with at least 1.0 m of true forward
  clearance;
- range is not overestimated by more than 0.35 m;
- lateral error is at most 0.35 m; and
- the estimate remains at least 0.30 m in front of the vehicle.

Conservative range underestimation is reported but is not a failure because it
causes earlier avoidance. The current degraded cases underestimate range by as
much as 0.91 m, so this suite supports obstacle-detection timing, not precise
localization. It also does not model flight-controller stopping distance,
real HM01B0 statistics, UART scheduling, or physical hardware.

## Continuous 4 m x 4 m course

Run the persistent multi-obstacle course with:

```sh
python3 tools/flow_gap8_validation/equivalence/run_exact_track_course_sim.py \
  --nanocockpit /home/cchen/tinympc-nanocockpit
```

This suite retains GAP8 and STM32 estimator state for an entire lap. It renders
solid perimeter walls and four solid obstacles in a 4 m x 4 m arena. The
production GAP8 and STM32 sources are used at the same compiled boundaries as
the other exact suites. Runs include the firmware `circuitPoint` circular
geometry translated into the arena and commanded at 1 m/s, safe clockwise and
counterclockwise continuous slaloms, and a degraded-camera clockwise lap.

Every frame is checked for obstacle contact and boundary violation. The suite
reports first detection clearance, false cylinders, emergency cues, accepted
depth tracks, cluster support, and yaw-gated frames. The complete reference is
replayed after collision to measure later perception, but
`physical_lap_completed` becomes false at first contact.

The controller source is audited at runtime for `obsEnable=0`, `obsUseFlow=0`,
and `obsLogOnly=1`. Cylinders therefore do not alter the circuit reference.
The course assumes perfect reference tracking until collision—an optimistic
upper bound, not an exact TinyMPC/PID or flight-dynamics simulation.

The existing firmware does **not** pass this suite:

- the 1 m/s firmware circle detects all four obstacles at 0.61-0.77 m
  clearance with no false cylinders, but produces no emergency cue and
  intersects obstacles because avoidance constraints are disabled;
- the safe slalom detects 0/4 clockwise, 1/4 counterclockwise, and 0/4 under
  degradation; and
- almost every frame exceeds the 0.20 rad/s yaw gate (120/121 circle frames
  and 199/201 slalom frames).

This invalidates extrapolating the straight-line 1 m/s result to a confined
racetrack. The next meaningful step is to enable and validate the production
flow-cylinder constraints, followed by exact controller/dynamics or
hardware-in-the-loop testing rather than weakening the course.
