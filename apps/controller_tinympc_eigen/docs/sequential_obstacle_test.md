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
least `seqAvoid.safeMin` (0.30 m by default), otherwise it is blocked. Confidence,
radius, tracking, latency, and perception margins are not applied in this mode.

Each fresh classification contributes its number of blocked slices (0 through 4)
to a bounded moving window. Once `avgN` samples have filled the window, evasion
starts when `dangerAvg` exceeds `trigAvg`; it ends when `dangerAvg` falls below
`clearAvg`. Defaults are a five-frame window, trigger above 3.5 dangerous slices,
and clear below 1.0. The separate thresholds provide hysteresis. Stale input still
produces a position hold. The CSV logs the raw distance/confidence outputs and the
moving-average state, although confidence does not affect either calculation.

The route chooses its lateral direction from four binary open-frequency moving
averages. Each fresh slice classification contributes 1 when open and 0 when
blocked, so `openAvg0` through `openAvg3` remain in [0, 1]. The side decision
compares only the two outer frequencies: `openAvg0` for right and `openAvg3`
for left. The side with a `sideBias` lead (0.03 by default) is selected.
Instantaneous metric distance and the inner slices do not affect side selection.
A genuine binary-history tie alternates from the last evasion side instead of
always choosing direction 0. The side is then
locked for the maneuver.
After each `refShift`-metre lateral waypoint, it
checks the updated moving average and continues stepping laterally until it falls
below `clearAvg`. An early clear cannot release the route until the first lateral
waypoint has been reached; after that first completed step, a clear may release
during any additional step. Lateral displacement is capped by `maxLat`; reaching that
limit produces a hold. Route altitude remains fixed at the commanded flight level.
On release, the achieved lateral displacement is retained as `pathOffX/Y` and
added to subsequent commander references. The displaced path is held for
`returnDelay` seconds (1.0 by default), then the drone holds position and yaws
50 degrees toward the avoided obstacle at a bounded 45 degrees/second. The scan
yaw is injected only into the downstream stock PID; TinyMPC
keeps its forward attitude-reference frame and suppresses the deliberate scan
yaw in its state. After 0.5 seconds of yaw settling, a separate five-frame
dangerous-slice average checks lateral clearance. A clear
scan permits blending back toward the original streamed path at `returnRate`
metres/second (0.20 by default). A blocked scan keeps the displaced path and
half-space, restores and settles at forward yaw, then advances in the displaced
lane for an adaptive interval and scans again. The interval starts from
`scanRetry` (3.0 seconds), is clamped by `scanRetryMin`/`scanRetryMax` (1.5/6.0 s),
and is shortened when the peer profile opens toward forward travel or its mean
clearance improved; it is lengthened when the obstacle-side outer ray remains
closer. Setting `returnRate=0` retains the displaced path
indefinitely and skips scanning.

After the first lateral waypoint is reached, the controller also anchors a soft
position half-space at that point. The permitted side is the side of the evasion:
a right evasion prevents planned horizon positions from moving left of the anchor,
and a left evasion prevents them from moving right. The plane is independent of
the classifier's metric distance; it only preserves achieved lateral separation.
It remains active through the return delay, every side scan, and blocked-scan
retry. It is removed only after a clear scan authorizes the gradual centerline
return. `constrain=0` or `logOnly=1` disables its application.

The avoidance waypoint is applied before `updateHorizonReference()`, so
`tiny_SetGoalState()` regenerates the complete MPC horizon around the displaced
waypoint. The held-and-decaying path offset likewise applies to each subsequent
firmware-generated reference, so there is no separate spline object to edit in
this mode.

The host sends one fixed final goal. Firmware advances an internal centerline
reference at 50 Hz and pauses it during lateral avoidance or return scanning.
Forward clearance is `min(d1, d2)`, using the two center camera slices so side
walls do not unnecessarily limit longitudinal speed. The stopping-distance bound
is `sqrt(2 * brakeAcc * max(fwdClear - safeMin, 0))`, capped by `cruise` and
rate-limited by `cruiseAcc` and `brakeAcc`. Defaults are 0.40 m/s cruise,
0.40 m/s^2 acceleration, and 0.60 m/s^2 braking. `speedCmd`, `fwdClear`,
`goalDist`, and `goalReached` expose the governor. The script's `--timeout-s` is
only a safety deadline; it does not determine flight speed.

## Circular obstacle course

`tinympc_sequential_obstacle_flight.py --circle-radius R` changes the fixed
goal into a firmware-generated circle. The vehicle begins at `startX/startY` on
the near circumference point, facing +x; the circle center is one radius to the
left for the default counterclockwise direction, or to the right with
`--circle-clockwise`. `--circle-speed` is the requested tangential speed and is
still capped by the center-slice stopping-distance governor. `--circle-laps`
ends the host run after that many completed revolutions.

The circle phase freezes throughout a lateral sidestep and PID-only peer scan.
Once a sidestep clears, its retained displacement is represented as a signed
radial offset that rotates with the circular centerline. This keeps the bypass
arc lateral to the path instead of leaving a fixed world-frame offset as the
vehicle turns. `visGate.phase` and `visGate.laps` are logged in the CSV.

For the first props-on course, use one lap at radius 1.5 m and 0.35--0.40 m/s,
with room for the 0.75 m maximum lateral bypass on both sides of the nominal
circle. Place the two obstacles on the nominal circumference, separated by at
least a quarter lap; the controller does not need their surveyed positions.

When `seqAvoid.enable=1`, this mode replaces the dense-map corridor. Sequential
vision chooses piecewise lateral waypoint references and, after the first step,
adds one world-frame lane-retention half-space. It never converts classifier
distance outputs into obstacle planes, avoiding the assumption that those scores
are geometrically exact world distances. Direction scoring remains available for
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
python3 tinympc_sequential_obstacle_flight.py --safe-min 0.30 --cruise-speed 0.40
python3 tinympc_sequential_obstacle_flight.py --circle-radius 1.5 --circle-speed 0.40 --circle-laps 1
python3 tinympc_sequential_obstacle_flight.py --monitor-only
```

`--monitor-only` selects controller 6 so the UART receiver runs, but keeps PID
passthrough active and never arms or sends a position setpoint. With props off,
carry or point the drone around the room and the console prints each fresh mask,
dangerous-slice count, moving-average fill, average, and `WARMUP`/`CLEAR`/`MID`/
`EVADE` state. It runs until Ctrl-C by default; use `--monitor-duration SECONDS`
for a bounded capture. Monitor samples are also written to the normal CSV output.

Use `--log-only` for the first restrained/clear-air validation. The normal run aborts
and lands on stale telemetry, stale vision, low battery, excessive excursion, or a
firmware hold. The CSV includes link counters, raw directional distances, open mask,
selected side, route phase, active waypoint, replan count, and solver diagnostics.
The speed-governor fields are `fwdClear`, `speedCmd`, `goalDist`, and
`goalReached`. `openAvg0` through `openAvg3` and `sideScore0`/`sideScore3` show
the binary open-frequency profile used for evade-side selection. `scanClr0`
through `scanClr3`, `scanSlope`, and `scanRetryWait` show the peer profile and
the resulting adaptive retry interval.
Obsolete confidence, effective-margin, grace, side-vote, and clear-vote columns are
not streamed. Active route phases are 0=idle, 1=sidestep, 3=hold, and
4=return scan; phase 2 is retained only for log compatibility. `scanAvg`,
`scanCount`, `scanRetries`, `scanYaw`, `scanYawGoal`, `scanSettle`, and
`scanDecision` and `scanPidYaw` expose the side-check state. `scanPidYaw=1`
confirms that `scanYaw` is being routed only to the PID. `scanDecision` is `0` while sampling, `1` after a clear
decision, and `-1` after a blocked decision; phase 4 remains active while yaw
returns forward. `dangerAvg` and `avgCount` expose the temporal filter directly.
`obs.solveStart` and `obs.solveDone` bracket each solve, while `obs.hbAge` and
`obs.stallHold` expose the stabilizer-side MPC heartbeat. A heartbeat older than
250 ms makes the stock PID hold the current pose. `obs.warmResets` counts full ADMM
resets and `obs.resetWhy=1` identifies controller activation;
`obs.solvePhase` records the route phase at solver entry and `obs.stackFree` is the
MPC task's minimum remaining stack in words. During phase 4, constrained ADMM is
paused and the held position plus scan yaw are sent directly to the stock PID;
`obs.scanBypass` counts these PID-only scan cycles. Since TinyMPC never observes
the peer yaw, its position/constraint warm start is preserved when longitudinal
planning resumes.
`--confidence-min`,
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
remains disarmed. Only the lightweight sequential-link classifier poll runs during
takeoff and landing; dense perception, reference generation, ADMM solves, and MPC
debug output remain dormant. `obs.pidBypass` counts these cycles. On the PID-to-MPC
edge, firmware resets its warm start, holds the current pose, and creates the first
avoidance waypoint from the post-takeoff position. This prevents both background
solver stalls and low-altitude waypoints from affecting the handoff.
