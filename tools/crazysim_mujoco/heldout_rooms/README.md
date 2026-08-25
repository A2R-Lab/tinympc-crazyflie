# Held-out fake-room matrix

This is an opt-in, simulator-only generalization suite.  It evaluates the
existing `straight_9m`, `circle`, `oval`, and smoothed `figure8` references in
separate bounded rooms. Each gate uses the physical four-piece NewBee mesh and
collision envelope at a disclosed 2x in-plane scale, giving a real simulated
0.90 m clear opening. This is the deliberately easier-course concession for
the first cross-trajectory robustness matrix, not an analysis-only tolerance.
Every gate is tangent-aligned and centered on a checked-in reference-header
sample. Each
course records strict header provenance for a later obstacle source sample and
a still-later completion sample. The obstacle is 2.2 m tall, has 0.12 m by
0.18 m horizontal half-size, and is offset 0.25 m from that tangent centerline:
the inflated nominal vehicle footprint intrudes, so a contact-free completion
requires a horizontal transition rather than an overflight. Completion uses a
0.30 m sphere and is searched only after the ordered physical gate crossing.

The analyzer evaluates obstacle and bounded-room clearance from that crossing
through completion. It reports the physical in-opening gate margin and rejects
course completion before the gate. `minimum_dodge_encounters` is intentionally
absent: no acceptance claim is made from an uninstrumented controller phase.

`run_heldout_rooms_matrix.sh` creates exactly five consecutive fresh seeds per
course from its explicit `--seed-start` in a new output root and never
overwrites evidence. The terminal matrix uses seeds 5301--5305. Seeds
5001--5005, 5101--5105, and 5201--5205 were reserved for dependent jobs that were canceled
before they started while their preceding exact matrices finalized the easier
course's terminal contract; the earlier
4801--4805 block informed a controller correction and is also excluded from
final acceptance. `evaluate_heldout_rooms.py`
rejects missing/invalid trials and writes both JSON and Markdown reports.  It
checks manifest/scene hashes, the header-derived gate and obstacle poses, and
the scene gate-body pose to prevent a run from being evaluated against different
geometry. Every course uses the same explicit 0.5 m/s progress
speed, 1.0 m/s^2 entry acceleration, 1.5 m/s^2 terminal deceleration, 0.40
progress reward, and the calibrated 0.2/0.1047352488 simulator/firmware
time-factor pair measured by the same-scene preflight on `a2r-tiger`.
Acceptance requires at least 70% contact-free completion in aggregate and at
least 2/5 on every individual trajectory; three strong courses therefore
cannot hide complete failure on the fourth.

Because the POSIX FreeRTOS tick is wall-clock
driven while MuJoCo advances simulation time, this factor is not portable to a
different host or rendering workload. `heldout_rooms_cpu.sbatch` therefore pins
the sequential CPU matrix to that node and fails closed elsewhere. A nonzero
runner code aborts immediately, while a behaviorally rejected but structurally
complete matrix still writes its report and returns nonzero. Every trial
must launch within 0.10 s of the requested 1.0 s; the outer runner also stops
grossly mistimed trials before analysis. The job uses the shared
Apptainer image and never invokes Docker.

## Terminal result

Bounded Slurm job 5990 ran all 20 trials on seeds 5301--5305. It failed the
behavioral acceptance gate: straight completed 3/5 contact-free runs, while
circle, oval, and smoothed figure-eight completed 0/5 each, for 3/20 (15%)
aggregate. Physical gate passage was 4/5 on straight and 5/5 on every curved
course. The recurring curved-course failure was post-gate altitude/attitude
loss before terminal rejoin, not a missing trial or camera-port conflict.
Two trials also lacked a valid ordered gate-to-completion interval, so the
no-invalid/missing check failed; zero run directories were missing. The
evaluator's exit code `1` is therefore expected and preserved as negative
evidence under
`../models/vision_rl_gate_joint/evidence/heldout_rooms_final_job5990/`.
