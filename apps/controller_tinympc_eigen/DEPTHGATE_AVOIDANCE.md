# DepthGate sector halfspaces

Implementation date: 2026-09-07. Active physical firmware lives in this
`tinympc-crazyflie-old-mpc-hover` checkout, matching the installed controller.

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
frame. Capture pose is approximated from current velocity/yaw rate and arrival
age plus inference and UART time. One additional inference interval of
translation padding accommodates the camera double buffer. This is not exact
timestamp synchronization or a calibrated monocular-error bound.

## Controller and parameters

`dgAvoid.enable=1` selects this mode; default0 preserves the existing mode.
Set it before controller handoff with `espTest.run=0`. Changing modes while RUN
is asserted requires releasing RUN. Controller selection remains the existing
OOT handoff procedure; no automatic arming is added.

`dgAvoid.speed=0.5` m/s (accepted range0..0.5), `clearance=0.5` m,
`raySlope=0.598203`, `range=2.0` m. `espTest.run=1` requests forward motion
along the handoff heading at the handoff altitude; speed ramps at0.5m/s².
Reference positions are projected onto the same plane intersection, and
reference velocities follow the projected horizon. Gate corner navigation
does not own references in this mode. The run is bounded to10seconds.

Arrival age must be <=200ms and reported inference <=150ms. A stale or invalid
observation latches hold until RUN is released. Other hold faults:
2=roll/pitch exceeds15degrees, 3=current pose violates clearance,
4=10second limit, 5=reference projection failed, 6=solver projection failure,
nonfinite rollout or >5cm predicted violation, 7=mode change with RUN asserted,
8=horizontal measured speed exceeds0.7m/s or is nonfinite.
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

## Telemetry and validation

`dgAvoid` logs enabled, fresh, count, mode, fault, seq, ageMs, left/center/right
in metres, local bound0/bound1, actual motor-input rollout violation, cmdSpeed.
Existing `dg`, corner, `dgRx`, and `mpcCstr` telemetry is preserved. The packet
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

Final V5 normal firmware flashed through the VM and verified2026-09-07.
dgAvoid.enable=1, speed=.5, clearance=.5; PID selected, espTest.run=0, motors0.
Live GAP8 inference packets were received throughout the disarmed check.
User confirmed launch area ready for a0.5m altitude,3second forward test.
Flight execution/results are pending.
