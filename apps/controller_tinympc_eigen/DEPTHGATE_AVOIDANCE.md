# DepthGate sector halfspaces

Updated 2026-09-10 for the recovered DDND depth/gate network. Network weights,
checkpoint, integer runtime and golden fixtures live in the NanoCockpit repository
at `src/gap/examples/ddnd`. See its README and recovery manifest for provenance.
The source is `final-11174-liveled-20260909`; weights SHA256:
`2be0691fb77bc2109933e3f0b1ec1fa1f7e5b0b82e15028cd944bbbcaf61aa76`.

The GAP8 app reduces signed dense logits to three inverse depths using
`0.01 + 9.99 * sigmoid(q * 2^exponent)` and takes the maximum inverse depth
(nearest prediction) in each full-height image third. It sends the existing
80-byte CRC32 DepthGate packet at 115200 baud. Gate corners and visibility
remain telemetry. Raw DDN1 output is optional diagnostics and is not accepted
by the STM32 controller. No dense tensor is buffered on STM32.

## Geometry

Convert positive inverse depths to optical-axis metres. Project the previous
actuator rollout into the image captured by GAP8, then extend it to the forward
mission goal, bounded by `dgAvoid.range` and the remaining mission distance.
This spatial lookahead does not change obstacle depth or add a speed margin.

The packet supplies three full-height image strips: [0,53), [53,106), [106,160).
For approximately level flight, horizontal projection is
`u = 80 - fx * camera_left / camera_forward`, with
`fx = (160/3) / raySlope` (default about 89.16 pixels). Clip each trajectory
segment to the image strip and test it against that strip's depth minus fixed
clearance. Segment clipping catches crossings between prediction knots. Points
behind the camera and outside the horizontal field of view do not create planes.
This is sector-level horizontal projection, not a pixelwise 3D collision test;
the packet cannot tell where vertically the nearest obstacle occurs.

At a detected intersection, place a vertical plane at the observed depth and
subtract `dgAvoid.clearance` (default 0.5 m) along its unit normal. Angle the
normal 30 degrees toward the blocked side so movement toward the clearer side
relaxes the inequality. A side must have at least 0.15 m more depth clearance to
select it; equal clearance gives a front-facing stop plane. The solver accepts
at most two planes, so the first two intersected sectors along the path are used.
A nearby sector that the trajectory does not enter supplies no constraint.

The permitted side is `n dot p <= b`. Transform the plane from the image capture
pose into world coordinates and then into the current MPC frame. A 96-entry
STM32 history, sampled every 50 ms, estimates capture pose from arrival age,
reported inference time and 7 ms UART time. Keep this delay correction even
though no velocity-scaled obstacle inflation is applied. Geometry is recomputed
as the predicted path changes; `dgAvoid.sample` identifies a plane update, while
`dgAvoid.seq` identifies the GAP8 image. Missing history rejects the observation.

This uses static obstacles, a level-camera approximation, symmetric principal
point, and full-height sector minima that can include floor pixels. It does not
provide a full image depth map or resolve occluded obstacles. No new GAP8 firmware
or packet format is needed.

## Controller and parameters

`dgAvoid.enable=1` selects this mode; default0 preserves the existing mode.
Set it before controller handoff with `espTest.run=0`. Changing modes while RUN
is asserted requires releasing RUN. Controller selection remains the existing
OOT handoff procedure; no automatic arming is added.

`dgAvoid.speed=0.1` m/s (accepted range0..2), `clearance=0.5` m,
`raySlope=0.598203`, `range=2.0` m. `espTest.run=1` requests forward motion
along the handoff heading at the handoff altitude; speed ramps at0.5m/s².
Reference positions are projected onto the same plane intersection, and
reference velocities follow the projected horizon. Gate corner navigation
does not own references in this mode. The default run limit is 10 seconds. `dgAvoid.timeout` accepts 1–120 seconds;
`dgAvoid.distance` accepts 0–20m, where zero disables the distance stop. The test
script sets both. Signed forward distance is measured from the RUN rising pose
along the handoff heading; distance completion (reason 9) latches hold until RUN
is released. Releasing RUN resets distance/time tracking.

The recovered network was measured at approximately 1.45s per frame. Arrival
age must be <=1800ms and reported inference in (0,2000]ms. This permits nearly
3.81s capture age at the limits; it is unsuitable for fast avoidance. Wait for
`dgAvoid.fresh=1` with RUN released before requesting motion so capture history
is populated. A stale/invalid observation or unavailable capture pose latches
hold until RUN is released. Set geometry parameters while RUN is released;
valid changes take effect on the next geometry update. Invalid configuration
is rejected each cycle. Other hold faults:
2=roll/pitch exceeds45degrees, 3=current pose violates clearance,
4=configured time limit, 5=reference projection failed, 6=solver projection failure,
nonfinite rollout or >5cm predicted violation, 7=mode change with RUN asserted,
8=horizontal measured speed exceeds2m/s or position/velocity is nonfinite,
9=requested forward distance reached. The first stop reason remains latched.
For fault6 hold is latched for the next100Hz update; there is no second solve
in the same cycle. User explicitly chose five iterations and accepts some violation.
Previously observed planes remain active when data becomes stale. A hold request cannot
guarantee immediate stopping or restoration of already-violated clearance.

The core supports exactly two XY halfspaces plus the existing tilt/rate box.
The DG-only generated cache uses XY constraint weight10000 and five ADMM iterations;
the original cache/five iterations are restored outside DG mode. A compile-time
assertion preserves the100Hz update. The existing20ms prediction grid is
unchanged: the solver replans every10ms, as in the original firmware. These are finite
iteration constraints, not a proof of physical collision avoidance.

## Current recovery validation

The recovered model passes host integer-kernel parity and GVSOC full-vector
parity (42,888 bytes over two fixtures); compact packet values/CRC also pass
host tests. STM32 transport, geometry and delayed-pose tests cover invalid
packets, wraps, stale data and moving/turning capture interpolation. The new
compact UART integration has not been flashed or flight-tested.

Current CF21BL build: flash 352,984 bytes; RAM 106,272 / 131,072 bytes;
CCM 58,096 / 65,536 bytes. Production-cache host rollout checks passed for
clear, wall, angled and two-plane cases at 0.1m/s initial/target speed,
five iterations, and 0.1m/0.3m plane bounds (<=1cm rollout violation,
<=15 degree tilt, <=120 degree/s rate acceptance). Reproduce with:

```sh
CXX=g++ DG_CACHE=tinympc_depthgate_cache.h DG_ACCEPT=1 DG_INITIAL_SPEED=.1 DG_TARGET_SPEED=.1 python3 tests/test_depthgate_rollout.py
python3 tests/test_depthgate_transport.py
python3 tests/test_espnet_collision_link.py
cc -std=c99 -Wall -Wextra -Werror -Isrc tests/test_tinympc_depthgate_timing.c -lm -o /tmp/dg-timing && /tmp/dg-timing
cc -std=c99 -Wall -Wextra -Werror -Isrc tests/test_tinympc_depthgate_planes.c -lm -o /tmp/dg-planes && /tmp/dg-planes
```

## Telemetry and historical validation

`dgAvoid` logs enabled, active, fresh, count, mode, fault, seq, sample, travel, ageMs, left/center/right
in metres, local bound0/bound1, actual motor-input rollout violation, cmdSpeed.
Existing `dg`, corner, `dgRx`, and `mpcCstr` telemetry is preserved. The `dgPlane0/1` groups report actual world nx/ny/b and matching sample IDs.
`active` distinguishes applied TinyMPC constraints from enabled configuration
while PID is selected. The packet
receiver validates CRC/finiteness and provides a coherent critical-section copy.

Host geometry and packet tests pass. Production-core two-plane projection tests
pass, including preserved tilt/rate limits and infeasible intersections.
DG cache single-horizon tests at0.5m/s, with0.1m/0.3m free plane boundaries, pass
1cm rollout /15degree tilt /120degree-per-second rate acceptance. The prior1000-weight/ten-iteration test had6.64mm violation. The final
10000-weight/five-iteration result is2.24mm, below the1cm model-test threshold.
Hardware build passes. An isolated CrazySim firmware build passed; final V5 sources match the successful SITL build. Five-iteration hardware
timing passes: maximum4.397ms across one/two-plane and hold cases (20samples
each), preserving the10ms scheduling budget. The benchmark was motor-inhibited. No closed-loop physical flight has been validated.

No physical flight or motor command has been issued by this implementation task.

Hardware timing history: rejected ten-iteration version reached9.331ms normal
and17.777ms with a second hold solve. This violates the intended10ms scheduling
budget. Final design uses one five-iteration solve per update and an optimized
one/two-plane projection with an exact box fallback.

Historical September 7 record (not the recovered DDND integration):
Final V5 normal firmware flashed through the VM and verified2026-09-07.
dgAvoid.enable=1, speed=.5, clearance=.5; PID selected, espTest.run=0, motors0.
Live GAP8 inference packets were received throughout the disarmed check.
User confirmed launch area ready for a0.5m altitude,3second forward test.
Flight execution/results are pending.

The measured-attitude RUN abort threshold is 45 degrees on either roll or pitch
(previously 15 degrees). This is an abort threshold, not a requested tilt or a
change to the MPC attitude bounds. The flight supervisor remains unchanged.
