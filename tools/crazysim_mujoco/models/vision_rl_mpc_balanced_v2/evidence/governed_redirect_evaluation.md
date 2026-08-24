# Corridor-aware governed redirect evaluation

Date: 2026-08-24

## Change

The TinyRacer dodge state now stores lateral-reference velocity and generates
sidesteps with bounded speed, acceleration, and braking distance. Opposite-side
redirects remain latched until the new lane is reached and no longer use the
implicit doubled sidestep speed. Same-side evidence during REJOIN does not
restart SIDESTEP.

For the 2 m PULP-DroNet U-course:

- bypass reference offset: `0.35 m`
- maximum reference offset: `0.45 m`
- shifted tracking-tunnel half-width: `0.30 m`
- maximum combined shifted tube: `0.75 m`
- eroded corridor bound checked at compile time: `0.80 m`
- sidestep/redirect speed: `0.35 m/s`
- lateral-reference acceleration: `1.00 m/s^2`

The tunnel remains centered on the shifted avoidance reference. The compile-time
envelope check ensures the complete shifted tube remains inside the U-course
corridor allowance.

## Validation

The strict-warning host test covers bounded offset, redirect speed,
acceleration, monotonic target approach, no target overshoot, terminal zero
rate, target clamping, opposite-side redirects, and same-side REJOIN rejection.
The CrazySim build also compiled the integrated C++ controller for the U-course.

The same policy and seeds `1001`, `1102`, and `1203` were rerun with all other
simulation settings unchanged.

| Metric | Before | Governed redirect |
|---|---:|---:|
| Full-course successes | 0/3 | 0/3 |
| Seeds reaching S1 | not all | 3/3 |
| Seeds reaching S2 / completing hairpin | not all | 3/3 |
| Mean contact time after launch | 8.640 s | 15.708 s |
| Mean cross-track RMSE | 0.489 m | 0.293 m |
| Mean cross-track p95 | 0.824 m | 0.458 m |
| Mean horizontal speed | 0.748 m/s | 0.638 m/s |

The original early slalom failure is materially improved: every governed run
cleared the initial alternating obstacles and completed the hairpin. The
remaining failures are later attitude/turn instabilities followed by wall or
ground contact. They are not evidence that the redirect is fully accepted, so
the result remains `0/3` course success and no end-to-end promotion claim is
made.
