## Active vision target: 2 m/s with gate servo

During braking, horizontal position follows the measured position only until horizontal speed first reaches <=0.15 m/s. The latched XY target is then placed 0.5 m behind that position along the original travel heading and remains fixed while MPC settles. Height and zero-velocity braking references are preserved. Every new brake resets the latch. Console `BRAKE HOLD: XY locked 500mm behind` and `espTest.holdLocked` in test.csv expose the transition; a simultaneous brake-start event takes console priority.

Desired velocity and position-reference progression are 2 m/s. Vision brakes immediately when one fresh inference packet has center strictly above 0.95. After measured speed remains below 0.10 m/s for 0.20 s, three distinct consecutive fresh packets at or below 0.95 resume a new 2 m/s leg from the stopped position on the original heading. Duplicate packets do not count; dangerous, stale, skipped, or reordered packets reset the clear count. Persistent danger or stale input holds position until the 15 s mission timeout. Start checks conservatively reject a currently blocked frame. `espTest.brakeFrames` and `espTest.clearFrames` expose the trigger and resume state in test.csv. Pitch 40 degrees, five iterations, and rho 25,000 remain unchanged. Earlier configurations below are historical.

Gate detections can now interrupt the straight leg for alignment and a one-meter passage at up to 0.5 m/s, then resume from the exit on the original heading. See [GATE_SERVO.md](GATE_SERVO.md) for acquisition criteria, limitations, phase logs and CSV recording. Both STM32 and GAP8 sender updates are required.

## Active vision braking demo

Target 5 m/s, pitch 40 degrees, five ADMM iterations, rho 25,000, fixed hover offsets. Immediate zero-velocity braking when at least two sectors are strictly above 0.9 danger. Three blocked sectors also stop. No turn or automatic resume. Fresh valid vision is required at start; packets older than 400 ms or invalid probabilities stop a running test. The old 1 m distance and target-speed triggers are disabled; 15 s timeout and manual cancellation remain. This timeout is not a stopping-distance safeguard. Keep adequate clear recovery space; 2 m obstacle separation is not validated at this speed.

The Python runner logs vision.csv with all three probabilities, validity, packet age and sequence and checks vision before takeoff and the run request. Start with the usual --fly invocation only after verifying camera input. Earlier configurations below are historical.

## Active 5 m/s, pitch 40, immediate braking

Target 5 m/s, pitch bound 40 degrees, five iterations, rho 25,000, 1 m measured travel trigger, vision off. Immediate zero-velocity braking; planned braking inactive. Earlier configurations below are historical.

## Active 6 m/s, pitch 40, immediate braking

Target 6 m/s, pitch bound 40 degrees, immediate zero-velocity braking. Five iterations, rho 25,000, 1 m measured travel trigger, vision off. Planned braking is inactive. Build and brake-trigger tests pass. Earlier settings below are historical.

## Restored 5 m/s immediate braking

Active source/build: 5 m/s target, immediate zero-velocity braking, 50 degree pitch bound, five iterations, rho 25,000, 1 m measured travel trigger, vision off. Planned braking remains inactive. Build and brake-trigger tests pass. Earlier configurations below are historical.

## Restored 6 m/s immediate braking

Active firmware again uses the earlier 6 m/s target, 50 degree pitch bound, five iterations, rho 25,000, and 1 m measured travel braking trigger. STOP commands zero velocity throughout the horizon, follows measured horizontal position until settled, then holds position. Planned braking is disconnected from the controller. Its standalone helper/test remain only as inactive experimental files. Build and trigger tests passed. Earlier flight showed recovery with reverse overshoot; that limitation remains. Earlier configurations below are historical.

## Planned braking trajectory

Target reduced to 5 m/s for this test; 1 m measured travel trigger, pitch 40 degrees, rho 25,000, five iterations, vision off. On entering STOP, capture measured horizontal velocity and position once, preserving altitude. Advance a persistent scalar speed/displacement profile along that horizontal velocity direction. Deceleration is 6 m/s² above 1 m/s, and 6 sqrt(v/1 m/s) below it, reaching zero in finite time without a reference reversal. Each MPC horizon previews this same profile with model timestep DT. Position and velocity use exact consistent integration. The endpoint becomes the fixed hover target. Measured speed must remain below 0.1 m/s for 0.2 seconds after reference completion to report settled. This shapes references, not a hard bound on actual deceleration or pitch; it does not resolve finite-iteration constraint violations by itself. Earlier settings below are historical.

## Flashed target 12 m/s experiment

Source/build target is 12 m/s, pitch bound 50 degrees, five iterations, rho 25,000, 1 m measured travel brake trigger, vision off. Build and brake state tests pass. Numerical acceleration rollout fails both bounds: 50.16 degrees tilt, 171.89 degrees/s body rate. Flashed after explicit user request; passive startup verification confirmed target 12 m/s and zero motor outputs. Backward excursions from the 8 m/s run remain unresolved. This test cannot establish a hard speed ceiling. Earlier configurations below are historical.

## Target 8 m/s experiment

Target and forward position reference advance at 8 m/s. Brake on 1 m measured travel, reaching 8 m/s, cancellation, or 15 s timeout. Pitch bound 50 degrees, five iterations, rho 25,000, fixed hover offsets, vision off. Build and braking state tests pass. Nominal acceleration rollout reaches 42.73 degrees tilt and 163 degrees/s body rate; body-rate regression remains failing. The preceding 6 m/s flight showed substantial reverse overshoot. Earlier configurations below are historical.

## Target 6 m/s experiment

Target and forward position reference advance at 6 m/s. Brake on 1 m measured travel, reaching 6 m/s, cancellation, or 15 s timeout. Pitch bound 50 degrees, five iterations, rho 25,000, fixed hover offsets, vision off. Build and braking state tests pass. Nominal acceleration rollout reaches 36.55 degrees tilt and 153.18 degrees/s body rate; body-rate regression remains failing. Earlier configurations below are historical.

## Pitch 50 experiment

Pitch Rodrigues bound is tan(25 degrees) = 0.466307658. Roll and body-rate bounds remain unchanged. Target 5 m/s, five ADMM iterations, rho 25,000, 1 m braking trigger, vision off. Build passed. Nominal acceleration prediction is unchanged from pitch 40 (32.79 degrees tilt, 142.55 degrees/s body rate); the body-rate regression still fails. This is an approximate predicted-state constraint, not a guaranteed physical attitude limit. Earlier notes below are historical.

## Flashed pitch 40 experiment

Pitch Rodrigues bound is tan(20 degrees) = 0.363970234; roll and body-rate bounds are unchanged. Target 5 m/s, five ADMM iterations, rho 25,000, 1 m braking trigger, vision off. Build and radio flash passed; passive postflash recording completed without flight commands. Nominal rollout predicts 32.79 degrees tilt and 142.55 degrees/s body rate: the body-rate regression still fails, so constraints are approximate at this iteration count. Earlier candidate notes below are historical.

## Unflashed candidate: 5 m/s, five iterations

Source now requests 5 m/s with five ADMM iterations and regenerated fixed rho = 25,000 and a 1 m measured travel braking trigger. Vision remains off. The numerical rollout regression fails: nominal acceleration predicts 32.39 degrees tilt and 140.84 degrees/s body rate; nominal braking also exceeds the intended 120 degrees/s limit. This candidate has NOT been flashed. Installed firmware remains the previous 3 m/s, two-iteration version. The following notes describe earlier configurations.

# Active: 1 m brake trigger

The firmware requests braking at 1 m cumulative horizontal travel or 3 m/s
forward speed, whichever comes first. Two iterations, pitch30, immediate start
after OOT handoff, and hover offsets remain. Braking can extend beyond 1 m.
Older configuration notes below are historical.

# Active: two iterations, pitch 30 degrees

Pitch Rodrigues component is bounded at +/-tan(15 degrees)=0.267949192.
Roll bound and body-rate bounds are unchanged. This is a predicted component
constraint, not a guaranteed physical attitude bound with an unconverged solve.
Two ADMM iterations, immediate RUN after OOT handoff, 3 m/s target, 3 m travel
brake trigger, vision off, and fixed hover offsets. Older configuration notes
below are historical.

# Active: eight iterations, immediate start after OOT handoff

The script still establishes steady PID hover, then selects OOT and requests RUN
as soon as the firmware handoff counter acknowledges initialization (normally
one telemetry interval). There is no extra OOT hover/stability dwell. The 3 m/s
speed target and 3 m cumulative horizontal travel brake trigger remain. Travel
while braking can extend beyond that trigger. Eight iterations previously
slowed the stabilizer in the sustained disarmed timing test; flight timing
is experimental and logged. Historical build notes below are retained.

# Five-iteration experimental build

User-requested five-iteration version, normal OOT enabled. The optimized solver
measured about 4.0 ms in the isolated disarmed timing sweep. Sustained flight-loop
timing at five iterations is not validated. The nominal numerical envelope test
fails: diagonal acceleration/braking reaches about 21.25 degrees versus the
intended 20-degree envelope. This is an unconverged-solve limitation, not a passed
constraint guarantee. The 3 m/s target, 3 m travel limit, body-rate bounds, and
fixed hover offsets are retained.

# Predicted attitude/rate constrained build

MPC now has state box constraints at prediction knots 1–24. The measured
initial state is never clamped. Roll/pitch Rodrigues components are bounded
by +/-0.105979703: a conservative 20 degree combined tilt box tightened
by 15% for finite-iteration error (pure-axis box angle about 12.1 degrees).
Body x/y/z rates are bounded by +/-1.780235837 rad/s (102 degrees/s),
also 15% inside the intended 120 degree/s envelope. Yaw attitude,
position and velocity receive no new maneuver bounds.

State/input ADMM cache is regenerated by generate_attitude_constraint_cache.py.
Five iterations are budgeted; these are optimization constraints, not a
hard guarantee on physical flight or an unconverged solve. CSV mpcLimit
telemetry reports maximum projected-input rollout bound excess (tiltErr in
Rodrigues units, rateErr in rad/s), primal residual, and solveUs. Zero excess
means the logged model rollout meets the boxes; it does not validate flight.
Input slacks/duals and state slacks/duals restart together each solve;
partially retained warm starts failed the high-rate numerical regression.
State initialization uses the measured state rollout in the current frame. State ADMM
penalty weights are 10000 for roll/pitch Rodrigues, 100 for body rates, and
1 otherwise; the augmented cache uses these same weights. This changes
constraint convergence, not the underlying tracking cost.

Braking reference and fixed hover motor offsets are retained. No powered
flight has validated this configuration.

# Active build: single 3 m/s brake test

Vision is disabled for motion decisions. Select OOT from stable hover, then set
`espTest.run=1`. Position and velocity references advance at 3 m/s. Measured
forward speed >=3 m/s immediately requests zero velocity. The 3 m cumulative
horizontal travel and 15 s limits still stop the run if speed is not reached.
It brakes, settles, and holds; no yaw or automatic restart. `espTest.run=0`
cancels. Console reports the brake reason, measured speed, peak and settling.
STOP stays latched until a new OOT handoff.

The vision-mode description below is retained for restoring the previous mode;
it does not describe this active brake-test build.

# Straight-line vision stop and yaw

Build from this app directory with `make -j4`. Target CF21BL STM32:
`build/cf21bl.bin`, on mpc-hover. Pure MPC, fixed hover calibration, 100 Hz,
two iterations. Stock PID remains available.

Select OOT from stable hover, then set `espTest.run=1`. Start requires valid
collision probabilities no older than 400 ms, fewer than two dangerous sectors,
and total estimated speed <=0.15 m/s. Camera-forward position and velocity
references advance at 4 m/s from the accepted start; lag retains position error.
Reaching 4 m/s no longer triggers a stop.

A sector is dangerous only when its probability is strictly >0.9:

| Dangerous sectors | Response |
| --- | --- |
| Left + center | Brake, then yaw right 45 degrees |
| Center + right | Brake, then yaw left 45 degrees |
| Left + right (center safe) | Brake and hold heading |
| All three | Brake and hold heading |
| Zero or one | Continue forward |

Braking requests zero horizontal velocity across the horizon. After total
estimated speed remains below 0.1 m/s for 0.2 s, capture a fixed horizontal hold.
Only then advance yaw toward the safe side, with a 30 degree/s reference bound.
Keep altitude fixed. The turn completes when measured yaw is within 3 degrees
for 0.2 s and the yaw reference has reached its goal. Rotate both the position
and velocity trajectory by the same 45 degrees. Automatically start a new leg
from the stopped position after all three sectors remain <=0.9 with fresh valid
vision and total speed <=0.15 m/s for 0.3 s. Recheck this condition on the actual
restart tick. Each resumed leg targets 4 m/s and retains the mission altitude.

A pending/active yaw is cancelled if vision becomes invalid/stale, all three
sectors become dangerous, or its chosen side exceeds 0.9. Cancellation holds the
current measured heading. A stop caused by distance, timeout, cancellation or
stale vision never initiates a turn. Rejected starts do not initiate turns.

Cumulative horizontal travel of 5 m or elapsed mission time 15 s also requests
stop. Both include braking and turning and persist across resumed legs. These
are stop-request limits, not guaranteed physical stopping bounds.
`espTest.run` stays 1 during turn/resume; setting it to 0 cancels the sequence.
All-three danger, left+right danger, stale vision, cancelled turns and mission
limits produce a terminal hold until another OOT handoff.

Console reports VISION RUN, BRAKE START and dangerous-sector count/direction,
BRAKE SETTLED (ms and signed forward mm), VISION YAW START/COMPLETE/CANCELLED,
and VISION RESUME.
Periodic measured/peak/target speeds remain in mm/s at 2 Hz. Printing stays in
appMain. Phase 2 includes braking, yaw and final hold. Reasons:1 danger,2 stale,
3 distance,4 cancel,5 timeout,6 invalid state,7 moving too fast to start.

After landing/disarming and closing cfclient, warm-boot flash with:

```sh
/Users/char_chen/miniconda3/bin/python -m cfloader \
  -w radio://0/80/2M/E7E7E7E7E8 flash build/cf21bl.bin stm32-fw
```

## Hover equilibrium

Fixed motor offsets [-6,+10,+6,-10] mN are restored, as confirmed by the user
for this aircraft. They are applied once to generated hover, with input bounds
shifted to retain physical actuator limits. Dynamics and solver caches remain
unchanged. PID trim acquisition remains disabled; stock PID remains available.
