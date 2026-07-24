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

`tools/equivalence/run_exact_track_image_sim.py` does not reimplement the
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
| Severe noise/exposure/blur | Safe degradation | No cylinder; insufficient persistence |
| 25 ms timestamp bias | Detect with degradation | Detect, 0.036 m error |
| Mixed 1.0/1.8 m depths | Select robust near component | Detect, 0.070 m error |
| Low-texture obstacle | Avoid false positive | No cylinder |
| Forward looming without baseline | Emergency cue only | No cylinder; brake cue on 3 frames |
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

These deterministic cases support software regression claims only. The
renderer is not a substitute for curated real-camera captures, and the
hardware checklist in `OBSTACLE_PERCEPTION_HANDOFF.md` remains mandatory.
