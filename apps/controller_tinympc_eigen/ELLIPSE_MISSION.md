# Ellipse, gate alignment, and obstacle braking

Enable this mission explicitly with `--ellipse`; the script otherwise selects
the existing straight mission. Supply the **full axis lengths**, not radii.
For example, from this application directory:

```sh
python3 tools/run_brake_csv.py --fly --ellipse --major 4 --minor 2 --speed 2 --laps 1 --height 0.5
```

The dimensions above are an example, not a measured fit to the flight area.
The major axis must be at least the minor axis and at most 20 m; the minor axis
must be at least 0.5 m. The speed ceiling is greater than zero and at most 2 m/s.
Choose 1–10 laps and a mission timeout of 5–600 seconds with
`--mission-timeout` (default 120). Omitting `--fly` only records telemetry and
does not configure, arm, or fly the drone.

## Starting pose and trajectory

The mission captures its origin and heading when the run starts. Position the
drone at a minor-axis endpoint, with its camera pointing along the desired
initial tangent. The ellipse center lies to the drone's **left**. Travel is
counter-clockwise viewed from above. The major axis is initially parallel to
the camera heading; the center is half the minor-axis length to the left.

The controller generates the ellipse position, velocity, and tangent yaw in
the mission frame and converts them into the controller's local frame.
The drone yaws with the tangent instead of keeping its initial yaw throughout
the ellipse. The requested speed is a ceiling: curve speed is reduced to
respect a 0.5 rad/s reference yaw-rate limit and a 1 m/s² lateral-acceleration
limit.

## Gate maneuver

Fresh matching rail/corner observations initiate the gate maneuver. Existing
gate confidence, edge checks, and confirmation requirements still apply.
The drone slows, then uses lateral and vertical visual servoing to bring the
gate to the image center. Alignment and passage retain the 0.5 m/s speed limit.

Collision danger does not trigger a brake during gate slowing and centering,
so the gate's rails do not compete with the visual alignment command. Collision
detection is active again during the 1 m forward passage and the return to the
ellipse. One meter is measured travel from the aligned pose; it is not a
measurement of distance to the gate plane.

After passage, the drone rejoins a nearby forward point on the original
ellipse and restores the ellipse tangent heading before continuing the lap.
It does not move the ellipse to the gate exit or continue indefinitely on a
parallel line.

## Dynamic obstacle

Outside gate slowing/centering, a single fresh frame with center danger
**strictly greater than 0.95** triggers braking. The drone stops and waits.
Resume requires stationary conditions and three qualifying clear frames;
the path reference pauses while stopped instead of advancing around the
ellipse without the drone. Passage and rejoin also retain obstacle braking.

## Configuration and logs

Before arming, the flight script writes `ellCfg.major`, `ellCfg.minor`,
`ellCfg.speed`, `ellCfg.laps`, and `ellCfg.timeout`, then enables
`espTest.ellipse`. Firmware snapshots these values for the run. The initial
firmware axis values are zero, requiring explicit dimensions before an ellipse
can start. Starting the script without `--ellipse` writes `espTest.ellipse=0`.

Ellipse mode enables the existing gate CSV groups and requires fresh valid
gate packets before takeoff and again before starting the mission. It also
adds `ellipse.csv` at 10 Hz with:

| Variable | Meaning |
| --- | --- |
| `ellipse.phase` | Ellipse mission phase |
| `ellipse.theta` | Ellipse angular progress |
| `ellipse.yawRef` | Reference yaw |
| `ellipse.refX`, `ellipse.refY` | Ellipse reference position |
| `ellipse.vRef` | Reference speed after curvature limiting |

The recording directory also contains the existing vision, position, attitude,
controller, gate, console, and event records. `metadata.json` records the
requested dimensions, speed, lap count, and timeout. This script change does
not alter the existing Control-C landing behavior.
