# Sequential Vision Obstacle Test

This bring-up path connects the Tiny Racer sequential network to TinyMPC without
using the dense perception map. GAP8 sends packet type `0x38` after each inference:

```text
header(90 19 08 38), STM32 capture tick, sequence, gate-valid flag,
4 x clearance metres, 4 x confidence score, CRC32
```

The four body-frame directions are fixed at `-40`, `-13.333`, `+13.333`, and
`+40` degrees. STM32 rejects malformed packets. Although the network sends both
a metric score and confidence, the sequential controller uses only the metric
score as a binary classifier: a direction is open when its raw clearance is at
least `seqAvoid.safeMin` (0.22 m by default), otherwise it is blocked. Confidence,
radius, tracking, latency, and perception margins are not applied in this mode.

Each fresh classification contributes its number of blocked slices (0 through 4)
to a bounded moving window. Once `avgN` samples have filled the window, evasion
starts when `dangerAvg` exceeds `trigAvg`; it ends when `dangerAvg` falls below
`clearAvg`. Defaults are a five-frame window, trigger above 3.5 dangerous slices,
and clear below 1.5. The separate thresholds provide hysteresis. Stale input still
produces a position hold. The CSV logs the raw distance/confidence outputs and the
moving-average state, although confidence does not affect either calculation.

The route chooses its lateral direction from the outer slices: if only one outer
slice is open it selects that side, if both are open it selects the one with more
clearance, and if both are blocked it deterministically selects slice 0. The side
is then locked for the maneuver. After each `refShift`-metre lateral waypoint, it
checks the updated moving average and continues stepping laterally until it falls
below `clearAvg`. Lateral displacement is capped by `maxLat`; reaching that
limit produces a hold. Route altitude remains fixed at the commanded flight level.
On release, the achieved lateral displacement is retained as `pathOffX/Y` and
added to subsequent commander references, translating the remaining streamed path
instead of pulling the vehicle back onto the obstructed centerline.

The avoidance waypoint is applied before `updateHorizonReference()`, so
`tiny_SetGoalState()` regenerates the complete MPC horizon around the displaced
waypoint. The persistent path offset likewise applies to every subsequent
commander reference, so there is no separate spline object to edit in this mode.

When `seqAvoid.enable=1`, this mode replaces the dense-map corridor. Sequential
vision creates no TinyMPC state half-spaces; it only chooses piecewise lateral
waypoint references. This avoids treating directional classifier outputs as
geometrically exact world planes. Direction scoring remains available for
diagnostics outside an active maneuver, but it does not control the moving-average
trigger or clear decisions.

Gate-aperture relaxation is intentionally disabled for this first straight-line
test. A later gate-passage mode must validate the complete gate geometry before it
relaxes a corridor plane; this obstacle test treats every accepted plane as solid.

## Build and run

Build and flash the STM32 app and the non-streaming Tiny Racer GAP8 image. With the
vehicle restrained or propellers removed, first verify that `seqRx.rxOk` increments,
`seqRx.crcErr` stays zero, and `seqAvoid.ageMs` remains below 250 ms.

The flight script repeats that check before arming, takes off with PID passthrough,
then hands the straight leg to TinyMPC:

```bash
python3 tinympc_sequential_obstacle_flight.py --log-only
python3 tinympc_sequential_obstacle_flight.py
python3 tinympc_sequential_obstacle_flight.py --safe-min 0.22
```

Use `--log-only` for the first restrained/clear-air validation. The normal run aborts
and lands on stale telemetry, stale vision, low battery, excessive excursion, or a
firmware hold. The CSV includes link, raw classifier
scores, open mask, selected side, route phase, active waypoint, replan count, and
solver diagnostics. Active route phases are 0=idle, 1=sidestep, and 3=hold; phase
2 is retained only for log compatibility. `seqAvoid.clearVotes` logs the current
number of open slices while a route is active; `dangerAvg` and `avgCount` expose
the temporal filter directly. `--confidence-min`,
`--vision-grace-ms`, `--side-min`, `--side-votes`, `--forward-min`,
`--forward-step`, `--pass-distance`, `--clear-votes`, `--probe-progress`,
`--probe-votes`, `--zero-margins`, and `--log-only` remain accepted for
command-line compatibility but do not change the classifier waypoint policy.

The solver-health guard is intentionally compatible with the bounded five-iteration
ADMM solve: its default limits are primal residual `0.50`, dual residual `300`, and
solve time `20000` microseconds. Non-finite and non-convex solves still enter an
immediate position hold. The three limits remain configurable with `--max-pri-res`,
`--max-dua-res`, and `--max-solve-us`; zero disables only that individual limit.

During PID passthrough, vision classification continues but the route state machine
remains disarmed. On the PID-to-MPC edge, firmware discards the background MPC output,
resets its warm start, holds the current pose, and creates the first avoidance waypoint
from the post-takeoff position. This prevents a low-altitude waypoint observed during
takeoff from becoming active at the handoff.
