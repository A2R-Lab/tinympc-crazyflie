# Straight-line DDND obstacle test

The test flies forward at a requested speed, lets TinyMPC's half-space constraints
limit/redirect the motion, stops at the requested forward displacement, holds,
and lands. Distance means displacement along the heading captured at handoff,
not accumulated path length. An obstacle may prevent reaching the distance;
the timeout or a controller fault then ends the test and initiates landing.

Build the updated CF21BL app and use the recovered NanoCockpit DDND app's compact
115200-baud output. The added `dgPlane0/1` logs and `dgAvoid.distance/timeout`
parameters are required; the script checks the firmware interface before arming.
It does not flash firmware. See [DEPTHGATE_AVOIDANCE.md](DEPTHGATE_AVOIDANCE.md)
for the controller model, limitations and build instructions.

From the repository root:

```sh
.venv/bin/python -m pip install -r apps/controller_tinympc_eigen/tools/requirements-obstacle.txt

# Preview options; no radio connection or flight commands.
.venv/bin/python apps/controller_tinympc_eigen/tools/run_obstacle_avoidance.py --speed .1 --distance .5

# Log only, with no parameter writes, arming or setpoints.
.venv/bin/python apps/controller_tinympc_eigen/tools/run_obstacle_avoidance.py --record --seconds 30 --output obstacle-recordings/bench

# Take off, test with obstacle constraints enabled, hold, and land.
.venv/bin/python apps/controller_tinympc_eigen/tools/run_obstacle_avoidance.py --fly --speed .1 --distance .5 --output obstacle-recordings/run-01

# Render the recorded constraints and trajectory.
.venv/bin/python apps/controller_tinympc_eigen/tools/plot_obstacle_avoidance.py obstacle-recordings/run-01 --output obstacle-recordings/run-01/constraints.mp4
```

`--uri` selects the radio (default `radio://0/80/2M/E7E7E7E7E7`). Speed must be
positive and no more than 0.2 m/s; default 0.1 m/s reflects the approximately
1.45-second DDND inference. Distance defaults to 0.5 m. `--timeout` defaults to
`distance/speed + 10` seconds and must be no more than 120 seconds and greater
than `distance/speed`. The allowance accommodates acceleration and avoidance;
it does not guarantee reaching an obstructed target. `--height` is climb above
current estimated Z, default 0.5 m. Output directories must be new.

## Takeoff and landing

The takeoff matches the [cfclient FlightTab Take Off implementation](https://github.com/bitcraze/crazyflie-clients-python/blob/8396d2854575d9c679fabb1f936e24679d553ce2/src/cfclient/ui/tabs/FlightTab.py):
enable `commander.enHighLevel`, then call `takeoff(current_z + climb, climb / 0.5)`.
The default is a 0.5 m climb over one second, with yaw left unspecified as in
cfclient. The script selects PID and arms using the supervisor interface before
this call. It sends no low-level setpoints during the HLC takeoff trajectory.

After stable hover, a position heartbeat starts and the script selects TinyMPC
(controller 6). RUN remains released until the handoff is acknowledged and the
capture-pose history yields fresh constraints. Both the firmware distance limit
and host monitor can end forward motion. Firmware completion is reason 9;
other faults are recorded as test failures. RUN release commands a TinyMPC hold.
The high-level takeoff controller does not drive the forward test.

Landing releases RUN, selects PID, releases low-level commander priority, and
requests a high-level descent to the original estimated ground Z. The script
then stops and disarms. Exceptions and Ctrl-C after arming also attempt this
cleanup while linked. A lost radio connection cannot deliver a landing command;
the firmware's own supervisor/watchdog applies. Tests do not establish physical
flight safety or guaranteed braking clearance.

## Logs and video

Each run contains:

- `metadata.json`: arguments, schema version and pinned cfclient source.
- `telemetry.csv`: host elapsed seconds, raw firmware timestamp, block name and
  JSON values for each received packet. Blocks retain their own timestamps.
- `events.csv`: takeoff, handoff/run, stop/landing and errors.
- `console.txt`: firmware console output.
- `summary.json`: completion/error, forward displacement and mission heading.

The plotter displays world XY position, heading, trajectory and the actual
controller planes `nx*x + ny*y <= b`. Shading marks the excluded side, already
including clearance and delay padding. These are constraint boundaries, not
measured obstacle surfaces. It pairs plane blocks by the exact controller sample
number and uses only data received by each video time. Missing/mismatched plane
blocks are labeled unavailable. The `active` flag distinguishes TinyMPC control from PID takeoff/landing.
It shows stale data and faults; retained planes
remain visible during a hold because the firmware still applies them. Separate
radio log blocks are not an atomic state-and-constraint snapshot.

MP4 uses FFmpeg, automatically locating the `imageio-ffmpeg` binary if needed.
A `.gif` output uses Pillow. `--fps` and `--playback-speed` control video output.
For an explicitly synthetic example, requiring no drone:

```sh
.venv/bin/python apps/controller_tinympc_eigen/tools/plot_obstacle_avoidance.py --demo build/obstacle-demo
```

## Verification

Fake-radio tests exercise successful takeoff/test/landing, command ordering,
parameter checks and failure cleanup without opening a radio. Plotter tests
cover constraint geometry, sample matching and demo rendering. The firmware's
straight-run helper has host tests for distance/time limits, reset, invalid
inputs and latched completion. The updated CF21BL build passed (354,312 bytes flash, 106,332 bytes RAM,
58,096 bytes CCM). No physical flight was performed for these tools.

```sh
.venv/bin/python -m unittest discover -s apps/controller_tinympc_eigen/tests -p 'test_*obstacle_avoidance.py'
```
