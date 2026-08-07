# TinyRacer perception, racing, and control boundary

TinyRacer uses three small layers. Each layer owns one kind of state.

```text
GAP8 CNN -> perception observation -> race intent -> world geometry -> TinyMPC
 raw measurements                  policy          half-spaces      actuator input
```

## Perception observation

The GAP8 sends measurements, never controller actions or TinyMPC plane numbers.
The existing CRC-protected 48-byte packet remains unchanged and contains a source
timestamp, sequence, gate-valid bit, four metric clearances, and four confidence
values. `sequential_obstacle_link` validates it and exposes one atomic
`TinyRacerPerceptionObservation` snapshot. Packet age is computed on the STM32.

When gate localization is ready, extend the packet with the smallest geometric
measurement the racing layer needs (normally normalized corners plus confidence),
not a desired roll/yaw command. Version the packet when its wire layout changes.

## Racing layer

`tinyracer_racing` is deterministic policy code. It consumes a complete perception
snapshot and emits a `TinyRacerRaceIntent`; it does not know about Eigen, the local
MPC frame, motors, or ADMM storage. Its policy is:

- a sector is blocked only when its model confidence is at least zero and its
  clearance is below 0.30 m, matching the deployed output contract;
- four blocked sectors enter `BLOCKED`, choose the more open left/right side, and
  freeze one measured-boundary half-space per sector plus a forward stop plane;
- mixed classifications retain the current state;
- four open sectors clear the constraints and enter `RECOVER` while the lateral
  offset smoothly returns to the nominal route;
- stale observations are marked stale and cannot silently remove active constraints.

While blocked, nominal trajectory time is paused and the horizon receives a smooth
0.35 m lateral bypass offset toward the selected side. Pausing is reference time
scaling, not a substitute for collision constraints: the hard half-spaces are what
prevent the optimized state trajectory from crossing the measured boundaries.
Individual directional planes are removed when that direction becomes confidently
open so they do not trap the lateral bypass, while the forward stop plane remains
until all four directions are open.

The gate-racing state machine should later live in this same layer as
`TAKEOFF -> ACQUIRE -> ALIGN -> APPROACH -> COMMIT -> EXIT`. Its output should be a
world-frame reference intent (target position/velocity/yaw and gate ID). During
`COMMIT`, freeze the accepted gate geometry in the world frame for a bounded time so
one dropped frame cannot redirect the vehicle.

## Geometry and controller

`controller_tinympc.cpp` owns the vehicle state and coordinate transforms. On a
constraint activation edge it converts each camera bearing and clearance into a
world-frame separating plane, subtracts a 0.10 m safety margin, and freezes the
planes. At every solve it transforms both the reference and the frozen planes into
the same active local frame, then gives them to TinyMPC at every constrained knot.

This is a local reactive bypass, not a global path-planning guarantee. Four scalar
forward clearances do not describe obstacle topology outside the camera field of
view; a globally certified collision-free route would require a map or a richer
free-space representation.

The MPC task consumes the latest complete state, intent, and constraint snapshot at
its fixed solve rate. It never waits for inference and it contains no temporal
classification logic. A stale motor command still falls back to model-derived hover;
perception loss is not a reason to stop the motors.

## Experimental modes

For controlled comparisons, keep perception, race intent, reference generation,
state estimation, and MPC tuning identical. Change only whether the emitted
half-space is installed in TinyMPC. Log packet sequence/age, race mode, constraint
activation edges, plane coefficients, reference, state, solve time, constraint
violation, and consensus error. This produces a defensible constrained-versus-
unconstrained ablation without maintaining two navigation stacks.
