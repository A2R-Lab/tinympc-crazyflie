# Compact acceptance evidence

This directory keeps small, reviewable terminal reports from the bounded Slurm
jobs. Raw videos, telemetry, READFS corpora, generated firmware trees, and
checkpoints remain in shared-home job directories and are addressed by the
hashes inside these reports; they are intentionally not duplicated in Git.

`gate_obstacle_job5939/` is the sealed, accepted **pre-final-controller**
30-run transition matrix on seeds
3121--3130. Its evaluator accepted the candidate with 9/10 strict ordered
gate-plus-obstacle completions, 10/10 physical gate passages, and zero contacts
in every arm. The copied reports have SHA-256 values:

```text
evaluation.json  416a02555262a2df76b5b010d0a2e9cca6a53991e86babf5b8676cc129ea836b
evaluation.md    3e451203834caf4184f34fb51da3233ef9ffa3061d6863bef1c1c355ebcba98d
```

`gvsoc_job5968/` is the terminal eight-core GVSOC parity result over 200
ordered calibration clips. It passed all six clip-zero generated-C layer
checks, 0.990 action agreement, +3.246 px visible gate-center degradation,
14.822 ms p95 at 100 MHz, and the L1/L2/flash limits. Because terminal-affine
and NeMO calibration used this corpus, it is not called a post-quantization
held-out set. The authoritative quantized decode contract and supplemental
confidence/corner/admission audit are copied with the result. `SHA256SUMS`
seals every compact file; raw generated applications, READFS files, and
simulator logs remain in the ignored shared-home job directory.

The held-out-room report below was recorded only after its terminal evaluator
finished. Held-out job 5971 was canceled after a downstream
gate-reacquisition failure, and concurrent job 5973 never started a flight
because its camera bridge port was already in use. Both remain documented in
`DESIGN_AND_EVALUATION.md`; neither is substituted for acceptance evidence.

`gate_obstacle_final_job5989/` is the terminal final-controller matrix on fresh
seeds 4001--4010. It passed 10/10 ordered gate-plus-obstacle completions with
zero gate-frame or obstacle contacts. Candidate and frozen-baseline
obstacle-only arms each completed 10/10 with zero collisions. Its
`SHA256SUMS` seals the evaluator JSON/Markdown and their compact common/model
provenance records. The report, not the earlier 5939 matrix, is authoritative
for final-controller claims.

`heldout_rooms_final_job5990/` is the complete, negative 20-run canonical-room
matrix on seeds 5301--5305. It recorded 19/20 physical gate passages but only
3/20 contact-free completions: straight 3/5 and circle/oval/figure-eight 0/5.
The evaluator intentionally returned nonzero because aggregate, per-course,
and no-invalid/missing acceptance checks failed. `exit_status.txt` preserves
that `1` status; `SHA256SUMS` seals the full machine report, paper table,
candidate bundle, and status. The failure is evidence and is not omitted or
replaced by a post-hoc easier matrix.

`gate_frame_fix_smoke_job5993/` records the serialized post-job-5990 diagnostic
after correcting gate-bearing projection, per-knot path-normal displacement,
and shifted-tunnel reconstruction. Circle, oval, and figure-eight each crossed
the physical gate but later contacted the scene. On every course, the first
accepted gate observation arrived after physical crossing, so the corrected
servo could not guide the approach. This compact negative result narrows the
remaining defect to low/late curved-view gate admission and the associated
dodge/recovery handoff; it is not a replacement acceptance matrix.
