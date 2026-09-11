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

Convert the three positive inverse depths to optical-axis metres with `z=1/d`.
Approximate each sector statistic at its center ray:
`L=(zL,+s*zL)`, `C=(zC,0)`, `R=(zR,-s*zR)` in body XY (+X forward, +Y left).
The calibrated HM01B0 `fx=89.1558392549` at width160 gives symmetric
`s=(160/3)/fx=0.598203`. This approximation omits principal-point asymmetry,
distortion, and the actual bearing of the nearest region within a sector.

Each adjacent pair defines a vertical plane with unit horizontal normal pointing
forward. The permitted side is `n dot p <= b`; subtract the requested **0.5m**
clearance from b, measured perpendicular to the plane. Clearance is never
silently clipped when the current pose is already outside it.

- Center closest: use the pair toward the more open side, with a 0.15m tie band.
- One side closest: use that side-to-center pair.
- Center deeper than both nearby sides: use both pairs (0.15m enter, 0.05m retain).
- All three depths beyond activation range (default2m): no planes.

The selected allowed regions are intersected. Plane positions are held in world
coordinates between new accepted frames, then transformed into each MPC local
frame. Capture pose is interpolated from a 96-entry STM32 pose history sampled every
50ms, using local arrival age plus reported inference and 7ms UART time.
Yaw interpolation follows the short arc across wrap. Missing history or a
control gap rejects the observation. The GAP8 camera is synchronous, with no
queued next frame. A further 0.37m margin covers 0.2m/s travel over the receive
budget plus 50ms. Geometry assumes static obstacles and approximately level
flight. Nearest pixels can include the floor, and a sector center is not their
true bearing; this remains an approximation, not a monocular safety bound.

## Controller and parameters

`dgAvoid.enable=1` selects this mode; default0 preserves the existing mode.
Set it before controller handoff with `espTest.run=0`. Changing modes while RUN
is asserted requires releasing RUN. Controller selection remains the existing
OOT handoff procedure; no automatic arming is added.

`dgAvoid.speed=0.1` m/s (accepted range0..0.2), `clearance=0.5` m,
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
valid changes take effect on the next accepted frame. Invalid configuration
is rejected each cycle. Other hold faults:
2=roll/pitch exceeds15degrees, 3=current pose violates clearance,
4=configured time limit, 5=reference projection failed, 6=solver projection failure,
nonfinite rollout or >5cm predicted violation, 7=mode change with RUN asserted,
8=horizontal measured speed exceeds0.2m/s or position/velocity is nonfinite,
9=requested forward distance reached. The first stop reason remains latched.
For fault6 hold is latched for the next100Hz update; there is no second solve
in the same cycle. User explicitly chose five iterations and accepts some violation.
Previously observed planes remain active during hold. A hold request cannot
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
