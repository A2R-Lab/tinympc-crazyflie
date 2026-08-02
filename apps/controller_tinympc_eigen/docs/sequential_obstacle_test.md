# Sequential Vision Obstacle Test

This bring-up path connects the Tiny Racer sequential network to TinyMPC without
using the dense perception map. GAP8 sends packet type `0x38` after each inference:

```text
header(90 19 08 38), STM32 capture tick, sequence, gate-valid flag,
4 x clearance metres, 4 x confidence score, CRC32
```

The four body-frame directions are fixed at `-40`, `-13.333`, `+13.333`, and
`+40` degrees. STM32 rejects malformed packets and never treats confidence below
`seqAvoid.confMin` as free space. An unreliable direction receives the conservative
`seqAvoid.defOff` boundary (0.25 m by default); no reliable direction produces a
position hold. Effective valid offsets include drone radius, tracking, latency, and
confidence-dependent perception margins.

When `seqAvoid.enable=1`, this mode replaces the dense-map corridor. It rotates
each accepted plane into the world frame, activates it only near an approached
horizon boundary, and reserves TinyMPC half-space slots 1 through 4. Slot 0 remains
available for the modeled obstacle. A smoothly increasing reference shift selects
a usable direction using distance, goal alignment, hysteresis, and velocity cost.
No fresh reliable direction causes a current-position hold.

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
```

Use `--log-only` for the first restrained/clear-air validation. The normal run aborts
and lands on stale telemetry, stale vision, low battery, excessive excursion, or the
firmware's no-reliable-direction hold. The CSV includes link, planner, active-plane,
effective-offset, and slack diagnostics.
