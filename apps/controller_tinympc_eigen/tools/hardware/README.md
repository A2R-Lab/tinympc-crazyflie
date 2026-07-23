# Physical obstacle corpus

These tools collect motors-disabled bench evidence for
`obstacle_corpus.json`. They do not authorize flight.

Before each run, verify the resident GAP8 feature count/rate, remove
propellers, measure the obstacle pose, and keep ADMM obstacle application
disabled. Positive cases require measured world-frame obstacle coordinates;
the launcher refuses to infer them from nominal manifest values. Ground truth
may instead be supplied as measured range/bearing from the initial body/camera
pose; the analyzer anchors that measurement to the first finite logged
x/y/yaw sample.

Preview a case:

```sh
/home/charchen/cf-venv/bin/python run_obstacle_case.py f27-p-001 \
  --truth-world-x 0.50 --truth-world-y -0.30 \
  --truth-obstacle-width-m 0.15 --truth-orientation-deg -45 --dry-run
```

Equivalent initial-pose measurement:

```sh
/home/charchen/cf-venv/bin/python run_obstacle_case.py f27-p-001 \
  --truth-start-range-m 0.58 --truth-start-bearing-deg -31 \
  --truth-obstacle-width-m 0.15 --truth-orientation-deg -45 --dry-run
```

The width/diameter and orientation values are measurements of the actual
object, not values inferred from the manifest's qualitative width label.
Positive coverage also checks the measured initial pose against the manifest:
forward distance and lateral offset must each be within 0.075 m, orientation
within 5 degrees, and measured width/diameter must be positive.

Remove `--dry-run` after arranging the exact displayed scene. Existing case
logs are never overwritten. A five-second countdown begins after radio and
controller setup; start the requested hand motion when it reaches zero.
Analyze completed sidecars with:

```sh
python3 analyze_obstacle_corpus.py logs/*.csv.json \
  --out logs/corpus-summary.json
```

The summary includes detection/false-positive rates, first detection
time/distance, localization errors, continuity dropout, transport counter
deltas, violations of the invariant that a persistent-map vote requires a new
UART sample, and measured speed/yaw evidence that flags gross mismatches
between the requested and performed motion. It also reports cross-block
timestamp completeness and p95/p99/maximum skew. The validated logger defaults
to 30 ms log blocks and 30 Hz snapshots; a 20 ms trial starved parameter
initialization and is not supported.

Bias reporting retains signed bearing and body-lateral errors. The bias gate is
evaluated only with all 180 positive cases present and requires paired
orientation and lateral-offset detection-rate differences no greater than
0.10, absolute mean signed bearing bias no greater than 5 degrees, and
absolute mean body-lateral bias no greater than 0.10 m.

Audit exact manifest coverage separately:

```sh
python3 corpus_progress.py --out logs/corpus-progress.json
```

Ad-hoc logs, duplicate case IDs, unreadable sidecars, manifest-field
mismatches, and runs whose measured motion is unsupported or unverified are
reported but never counted as completed manifest cases. Coverage also requires
at least 95% complete timestamp sets with p99 cross-block skew no greater than
one 15 Hz flow period, and positive measured setup geometry within the
predeclared tolerances above.

Audit the complete saved validation bundle from the Crazyflie repository root:

```sh
python3 apps/controller_tinympc_eigen/tools/hardware/audit_validation_bundle.py \
  --allow-incomplete \
  --out apps/controller_tinympc_eigen/tools/results/flow_obstacle_completion_audit.json
```

Omit `--allow-incomplete` for an acceptance gate: the command exits nonzero
until source state, hashed builds, equivalence, timing, full corpus coverage,
detection, bias, localization, transport, and new-sample-only voting
requirements all pass. Corpus gates are recomputed from qualified raw sidecars;
the verifier does not trust manually toggled acceptance booleans.
