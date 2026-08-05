# Per-track obstacle perception software validation

Date: 2026-07-23

The Crazyflie and AI-deck were unavailable. All results below are host tests or
cross-build results; none are hardware, UART scheduling, flight, or safety
validation.

## Simulation versus live-test gap

The earlier analytic/sector simulations were substantially more optimistic
than the poor live tests:

- Analytic flow and ground-truth rays omitted feature detection, LK failures,
  repeated texture, exposure changes, blur, distortion, timing error, and
  quantization. Live estimates therefore had far less support and much larger
  uncertainty.
- The Python GAP8 approximation had drifted from production feature count,
  selection order, calibration, full-resolution refinement, forward/backward
  validation, and float behavior. Numeric agreement with that approximation
  was not evidence about the deployed frontend.
- Sector aggregation mixed foreground and background before STM32 estimation.
  It discarded individual residuals and made a few mutually inconsistent
  tracks look like one confident observation.
- Simulations assumed commanded peering motion was achieved. The live result
  improved from about 0.23 m to 0.45 m only when approximately +/-8 cm of real
  lateral peering supplied observable parallax.
- The old confidence score measured sector support, not calibrated range
  uncertainty. Lowering a confidence threshold could admit a biased estimate
  without fixing observability.
- A 16-bit microsecond frame interval saturates at 65,535 us below the nominal
  66,667 us 15 Hz period. Version 1 is retained, but STM32 now reconstructs
  consecutive intervals from the 32-bit capture timestamps.

## Firmware changes driven by those gaps

- STM32 reconstructs Brown-Conrady-undistorted start/end rays for every
  accepted version-1 track and uses state interpolated at the echoed exposure
  tick.
- It removes yaw flow, applies camera yaw and lever-arm velocity, estimates
  per-track inverse depth, and propagates FB residual, weak photometric LK
  penalty, velocity, gyro, calibration, and timestamp uncertainty.
- LK photometric error is no longer treated as geometric pixel error. Exact
  image replay showed that dimensional mistake rejected all clean 1 m peering
  tracks.
- A fixed 32-element union-find separates spatial/range components. Components
  require at least three tracks and are combined with inverse-variance weights.
- A bounded 2-of-9 temporal window tolerates intermittent real-image tracking.
  Only persistent components vote into the fixed 16-cell map; sectors and the
  TinyMPC cylinder are formed after this validation.
- Cylinder validity additionally requires measured lateral baseline >=0.04 m,
  recent support, cluster sigma <=0.20 m, and repeated map votes. The flight
  script independently checks baseline, support, track sigma, and cylinder
  covariance before accepting a frozen obstacle.
- A three-track, FB-validated looming cue with time-to-contact <=1 s remains
  independent of the parallax cylinder. It commands a current-pose PID hold
  and expires after 200 ms.
- GAP8 routine sector transmission is disabled by default, recovering UART
  margin. `FLOW_LEGACY_SECTORS=1` plus `flowObsCtl.mode=0` is the explicit
  compatibility configuration.

## Exact-source image simulation

`tools/flow_gap8_validation/equivalence/run_exact_track_image_sim.py` does not reimplement the
algorithm in Python. Python renders deterministic calibrated grayscale inputs
and orchestrates the run. The GAP8 executable contains verbatim function bodies
extracted from `pulp-frontnet/main.c`; the STM32 executable directly includes
`flowdeck_obstacle_link.c`.

The initial eight cases and four additional position cases passed all expected
behaviors:

| Case | Expected result | Exact-source result |
| --- | --- | --- |
| 0.20 m nominal peering, 1 m obstacle | Detect | Detect, 0.030 m center error |
| No motion | Reject as unobservable | No cylinder |
| 0.025 m peering | Reject for baseline/motion | No cylinder |
| Severe noise/exposure/blur | Detect or reject safely | Detect, 0.189 m center error |
| 25 ms timestamp bias | Detect with degradation | Detect, 0.036 m error |
| Mixed 1.0/1.8 m depths | Select robust near component | Detect, 0.070 m error |
| Low-texture obstacle | Avoid false positive | No cylinder |
| Forward looming without lateral baseline | Detect and emergency cue | Detect, 0.085 m error; brake cue on 2 frames |
| Centered obstacle at 0.8 m, 0.30 m sweep | Detect | Detect, 0.047 m center error |
| Centered obstacle at 1.4 m, 0.40 m sweep | Detect | Detect, 0.062 m center error |
| Obstacle 0.2 m left at 1.0 m, 0.30 m sweep | Detect | Detect, 0.065 m center error |
| Obstacle 0.2 m right at 1.0 m, reversed 0.30 m sweep | Detect | Detect, 0.204 m center error |

The position sweep exposed an observability limitation: all four added cases
were rejected under the first, uniform 0.20 m left-to-right trajectory. The
far case required more baseline, and reversing the sweep restored sufficient
track persistence for the right-offset case. This is evidence that flight
tests must exercise both peering directions and record achieved baseline;
detectability at one bearing does not imply symmetric coverage.

## 1 m/s race approach without peering

The follow-up race corpus runs the same compiled production boundaries at
1 m/s with no lateral peering. The baseline firmware passed only the
empty-background case (1/8): forward-flow tracks were mostly rejected and the
final cylinder required a lateral-only baseline.

Firmware changes driven by the failures:

- forward optical expansion now supplies per-track axial depth when its
  propagated range uncertainty passes the same absolute and relative gates as
  lateral parallax;
- yaw flow is removed with the fixed-world image-motion sign, and pure yaw is
  an explicit negative;
- the baseline gate now measures achieved camera translation rather than only
  the component perpendicular to the obstacle bearing;
- two consistent tracks can form a candidate, while the 2-of-9 temporal,
  spatial, uncertainty, baseline, and map gates remain;
- redundant post-map far-cylinder confirmations were removed to avoid adding
  several 15 Hz periods after temporal validation;
- the evaluated 36-feature GAP8 configuration is now the default; the
  32-track wire selection retains bounded spatial coverage; and
- the renderer integrates forward blur along the approach axis instead of
  incorrectly applying it as lateral smear.

The final 14-case result was 14/14:

| Scenario | First true clearance | Range overestimate | Lateral error |
| --- | ---: | ---: | ---: |
| Centered clean | 1.635 m | -0.004 m | 0.119 m |
| Left offset | 1.570 m | 0.089 m | 0.166 m |
| Right offset | 1.245 m | 0.028 m | 0.003 m |
| Narrow centered | 1.375 m | 0.118 m | 0.047 m |
| Cluttered | 1.500 m | -0.020 m | 0.025 m |
| Degraded texture 1 | 1.375 m | -0.907 m | 0.152 m |
| Degraded texture 2 | 1.310 m | -0.407 m | 0.001 m |
| Degraded texture 3 | 1.310 m | -0.441 m | 0.033 m |
| Gentle yaw | 1.440 m | 0.182 m | 0.098 m |
| Opposite gentle yaw | 1.505 m | -0.037 m | 0.280 m |
| Timing jitter and doubled interval | 1.505 m | 0.060 m | 0.174 m |
| Clean background | No cylinder | n/a | n/a |
| Degraded background | No cylinder | n/a | n/a |
| Pure yaw | No cylinder | n/a | n/a |

Negative range-overestimate values are conservative: the estimate is closer
than truth. The largest conservative error is 0.91 m. This is acceptable for
the avoidance timing gate but not evidence of accurate obstacle localization.
All positive overestimates remained below 0.35 m.

There were no real Himax captures in either repository, so real-image replay
could not be added. These results do not establish physical stopping distance
or reliable racing. The next evidence gate remains capture replay followed by
restrained 1 m/s hardware approaches with measured detection and stopping
clearance. The new `flowObsRx.fwdDepth` log identifies how many accepted
tracks used forward expansion during those tests.

## Continuous confined-course result

A later exact-perception replay replaced independent approaches with
continuous laps in a 4 m x 4 m arena. Four solid obstacles and four solid
perimeter walls remain in the scene for the full run, and GAP8/STM32 estimator
state is never reset between encounters.

The existing firmware fails in this confined-course setting:

| Course | Detections | False cylinders | Emergency frames | Collision contacts | Physical lap |
| --- | ---: | ---: | ---: | ---: | --- |
| Firmware circuit geometry at 1 m/s | 4/4 | 0 | 0 | 31 | Failed |
| Safe clockwise slalom | 0/4 | 0 | 0 | 0 | Completed |
| Safe counterclockwise slalom | 1/4 | 0 | 0 | 0 | Completed |
| Safe degraded clockwise slalom | 0/4 | 0 | 0 | 0 | Completed |

On the firmware-circle replay, first detection clearances were 0.61-0.77 m.
The full reference continues after contact so later perception can be
measured; it is not counted as a physical lap.

The course exposes a geometry mismatch: 120/121 firmware-circle frames and
199/201 slalom frames exceed the forward-depth/looming yaw gate of 0.20 rad/s.
At 1 m/s this gate requires a turn radius above 5 m, which cannot fit inside a
4 m x 4 m arena. Lateral parallax still detects the four circle obstacles, but
the looming emergency path remains suppressed.

The simulator audits the controller defaults (`obsEnable=0`, `obsUseFlow=0`,
`obsLogOnly=1`). Detected cylinders therefore do not affect the reference, and
the only unconditional response is the looming current-position hold—which
never activates here. Perfect reference tracking is used until collision,
making this an optimistic upper bound rather than an exact TinyMPC/PID
dynamics simulation.

This continuous result supersedes any broad claim that the 14/14 straight
approach corpus proves confined-track readiness. It shows that the compiled
perception path can detect straight and some circular encounters, but the
existing firmware configuration cannot safely complete a path-blocked 1 m/s
course and has poor recall on the preplanned slalom.

These deterministic cases support software regression claims only. The
renderer is not a substitute for curated real-camera captures, and the
hardware checklist in `OBSTACLE_PERCEPTION_HANDOFF.md` remains mandatory.
