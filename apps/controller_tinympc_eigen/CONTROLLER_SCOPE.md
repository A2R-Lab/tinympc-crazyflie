# Retained controller

The local controller is the hardware-only `hover_lqr_test` implementation:
100 Hz, 12-state rigid-body MPC followed by 500 Hz LQR feedback. The default
Makefile profile selects it; removed profiles fail explicitly.

Retained behavior includes the fixed hover frame, coherent latest-snapshot
mailbox, projected-input nominal rollout, 60 ms plan deadline, finite checks,
supervisor gating, PID handoff/fallback and pilot-command guard, disarmed-only
benchmark, diagnostics, and hardware thrust-to-command conversion.

The separate A2R copy retains the same controller pipeline with its existing
CrazySim adaptations: direct per-motor SI thrust packets, taskless binary
horizon diagnostics, and hover/height-step/pitch-step profiles. The simulator
retains motor lag and physical limits. Do not copy its SITL-specialized source
over the hardware source.

Racing, perception, stored trajectories, maneuvers, actuator-LTI and rate-cascade
paths were removed from the active controller. Racing and perception objects
are no longer linked. Historical helper files, generated models, and simulation
artifacts elsewhere in the repository were not deleted.

This is a scope cleanup, not a controller retune: model matrices, gains, input
bounds, and motor dynamics were not changed. The earlier simulation instability
has not been fixed by this work. No firmware was flashed.

Pre-cleanup uncommitted originals are preserved locally under
`.codex-build/controller_cleanup_backups/` and on A2R under
`.codex-build/controller_cleanup_backups/20260905/`.

Validation: hardware firmware build; strict C tests for LQR feedback, nominal
rollout, handoff, fallback diagnostics, and BrushJAX conversion; production
frame/reset tests; mailbox scheduling tests; CrazySim build and short
height-step integration run with binary horizon output.
