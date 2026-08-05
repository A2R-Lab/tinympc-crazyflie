# Optical-flow obstacle validation status — 2026-07-23

## Recommendation

Retain the 27-feature configuration for bench testing only. Do not proceed to
free flight. Both GAP8 configurations and the updated STM32 image clean-build
and were flashed. Running logs confirm the embedded GAP8 build identifier and
both feature counts. Nominal 15+15 Hz timing is stable for both configurations,
but only two negative cases and one positive case have been rerun with the
updated estimator. The first admissible positive case was missed by the
detector; the remaining physical corpus is still missing. The 36-feature image therefore
has adequate timing headroom but not enough detection-quality evidence to
replace 27 features.

## Source state

- STM32 outer repository baseline: `c12edaf5d8edf459c4a4b9e81f7fb208d45493a8`
  (`vision`), dirty with the changes described below.
- Crazyflie firmware submodule: `de14ff9f08ae80cbb36ffd37859efc2bedfbf6bf`,
  dirty with UART receive-queue drop accounting.
- GAP8 repository baseline: `1e5cf0dc08e306f1849f20d15875c676d7e11e22`
  (`gate8-dory-package`), dirty with feature-count selection and diagnostics.

No changes were committed, pushed, or submitted as a pull request.

## Implemented

- A current-inclusive two-of-three window, updated only for a new UART sample.
- A 0.55 m body-frame consistency gate; one frame can never vote or publish a
  cylinder.
- Fixed-capacity union-find body-point grouping that does not require adjacent
  sectors and produces disjoint foreground/background components. Groups are
  scored by support, confidence, proximity, and temporal agreement; excessive
  within-group range dispersion is rejected.
- Aggregate displacement, yaw-explained motion, parallax/looming disagreement,
  and range-dispersion gates.
- STM32 logs and counters for each rejection reason and its intermediate values.
- UART duplicates, invalid packets, CRC/bad frames, receive-queue drops, stale
  age, producer sequence gaps/resets, GAP8 timestamps, new-sample processing,
  and persistent-map vote counts are exposed by logs.
- GAP8 builds select 27 or 36 features with `FLOW_FEATURE_COUNT=27|36`; the
  heartbeat reports the selected count. Bounded on-device histograms report
  conservative 250-us-bin p95/p99 upper bounds without per-frame JTAG prints.
- The bench logger now records all nine sectors, per-log-block Crazyflie
  timestamps, ground-truth metadata, exact source state, transport health,
  rejection intermediates, and evidence-vote counters.
- `tools/flow_gap8_validation/hardware/obstacle_corpus.json` defines 90 positive and 48 negative
  cases per feature configuration. It covers every requested
  distance/orientation/lateral-offset triple while balancing shape, width,
  depth configuration, texture, lighting, and motion. Negative cases cover
  stationary, translation, translation+yaw, and pure-yaw motion across every
  texture/lighting combination. The companion analyzer reports aggregate and
  stratified detection/error/dropout/transport metrics.

## Reproducible checks

From `apps/controller_tinympc_eigen/tools`:

```sh
python3 -m unittest -v test_sim_flow_obstacle_sectors.py
```

All 13 tests pass. They cover N-of-M intermittency, spatial gating,
parallax/looming disagreement, noncontiguous foreground/background grouping,
duplicate controller reads, stationary input and estimator drift, pure yaw,
and low texture.

Clean STM32 build:

```sh
cd apps/controller_tinympc_eigen
make clean
make -j2
```

Result: Flash 336,884/1,032,192 B; RAM 107,936/131,072 B; CCM
62,392/65,536 B. The generated `build/cf21bl.bin` is 337,340 B; this raw
address-span file size includes padding and is distinct from the linker's
336,884 B occupied-flash total.

Clean GAP8 build commands:

```sh
cd src/gap
./gap8.sh examples/pulp-frontnet FLOW_FEATURE_COUNT=27 clean build
./gap8.sh examples/pulp-frontnet FLOW_FEATURE_COUNT=36 clean build
```

For build-only verification, use `clean build` for both configurations; in GAP
SDK 3.8.1, `all` also runs `image flash_fs` and therefore touches JTAG. The
separately recorded flash operations used `clean all` at the unmodified
SDK-default 1500 kHz. Both final configurations link successfully:

| Features | L2 | FC TCDM | FC TCDM aliased | L1 | L1 aliased |
|---:|---:|---:|---:|---:|---:|
| 27 | 148,208 B | 7,660 B | 992 B | 28 B | 24 B |
| 36 | 148,328 B | 7,740 B | 992 B | 28 B | 24 B |

Raw clean-build logs and preserved ELF/size outputs are stored under
`tinympc-nanocockpit/src/gap/examples/pulp-frontnet/tools/{build_logs,build_artifacts}`.
The STM32 clean-build log and preserved `.bin`/`.hex` outputs are under
`tools/flow_gap8_validation/hardware/{build_logs,build_artifacts}`. SHA-256 hashes for these
artifacts are recorded in the machine-readable results JSON.

Both HyperFlash writes reached `flasher is done` and subsequent JTAG runs
mounted readfs and executed `build=1e5cf0dc08e3-dirty`. The earlier zero-word
readback occurred while the Crazyflie STM side was powered off; after a normal
power/reset sequence, the same flash became readable. The currently resident
image is the 27-feature configuration.

## Updated hardware timing and transport

Raw logs, profile/rate CSVs, quantile CSVs, and summary JSON are saved as
`tinympc-nanocockpit/src/gap/examples/pulp-frontnet/tools/hardware_f{27,36}_timing*`.
Quantiles below are conservative 250 us histogram-bin upper bounds.

| Features | Duration | Completed pairs | Flow rate | Flow min/mean/max | Flow p95/p99 | CNN min/mean/max | CNN p95/p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 27 | 90 s | 1,328 | 14.93 Hz | 16.083/16.95/17.303 ms | ≤17.25/17.25 ms | 50.385/51.26/51.605 ms | ≤51.75/51.75 ms |
| 36 | 240 s | 3,568 | 14.92 Hz | 18.982/20.97/21.424 ms | ≤21.50/21.50 ms | 40.924/54.61/55.725 ms | ≤55.75/55.75 ms |

Every captured window reports zero flow snapshot/TX drops, UART errors, camera
recoveries, and I2C errors. Invocation and completion counts match for both
flow and CNN. The 27-feature JTAG console stopped after 90 s with an OpenOCD
`Burst read timed out`; simultaneous CrazyRadio telemetry continued through
60 s with clean increasing producer counters, so this is recorded as a probe
console failure rather than a firmware transport failure.

## Updated stationary negative

The updated STM32 raw image (reported by the radio loader as 337,339 bytes,
330 pages for that flash operation) was flashed over
`radio://0/80/2M/E7E7E7E7E8`. The prior 0.002-rad gate produced a stationary
false cylinder, motivating a 0.004-rad aggregate-displacement threshold and a
regression test. The 60 s rerun
`tools/flow_gap8_validation/hardware/logs/f27-stationary-threshold-0p004.csv` contains 1,200 rows:
919 received samples, all 919 new, zero CRC/bad/duplicate/invalid frames,
sequence gaps/resets, UART queue drops, persistent-map votes, obstacle outputs,
or cylinder outputs. All 919 samples were rejected by the low-motion gate.
Its machine-readable summary reports false-positive rate 0.0 for this case.

## Acceptance audit

- Compiled/Python equivalence: **passed**. The restored harness compiled the
  production STM32 source and compared 204 frames across 17 cases at 15 Hz.
  Maximum absolute error was `2.6630047500475484e-07`; every case passed.
- Sustained alternating nominal 15+15 Hz: **passed on this bench scene for
  both configurations** (14.92–14.93 measured Hz, no scheduler/transport
  drops in the captured windows).
- No false cylinders in stationary, pure-yaw, low-texture hardware tests:
  **one low-texture stationary case and one low-texture pure-yaw case pass;
  the remaining texture, lighting, motion, and 36-feature strata are missing**.
- At least 95% detection over the defined physical corpus: **not demonstrated;
  the first admissible positive case was missed**.
- Orientation/left-right bias and ≤0.35 m localization: **missing**.
- Controller stability/memory safety/UART recovery: **clean builds pass and
  updated bench counters are clean; fault-injection recovery remains untested**.

The next stage remains motors-disabled bench work: collect the remaining
translation, translation+yaw, texture, and lighting negatives, then the defined
measured-ground-truth positive corpus.
The user also authorized incremental processing-rate experiments after the
15+15 Hz baseline; those must retain the same deadline/drop/error gates and do
not substitute for corpus validation. Do not infer flight authorization from
this report.

The manifest-coverage audit currently reports 3/276 exact manifest cases
complete. `f27-n-010` (stationary, low texture, nominal light, no obstacle)
was recollected at a 30 ms block period and passed 30 s with 900 logged rows,
460 new flow samples, no cylinder or map vote, zero transport faults, and zero
map-vote-without-new-sample violations. All 900 rows contained all 16 log-block
timestamps; cross-block skew was p95/p99/max 35/36/37 ms. `f27-n-046` (pure
yaw with the same scene conditions) passed 30 s with 600 rows and 460
received/new samples, no cylinder or map vote, and the same zero-fault
transport result. Its logged yaw rate spanned -3.21 to +2.43 rad/s and its
measured heading spanned about 92 degrees, confirming that the negative case
contained substantial yaw motion. Its timestamp completeness was 100% and
p95/p99 skew was 30/31 ms. The useful
smoke/diagnostic logs are intentionally classified as outside-manifest evidence
rather than silently counted toward coverage. An earlier stationary recording
accidentally labeled `f27-n-046` is preserved under `logs/invalid_mislabeled/`
and is excluded from coverage.

`f27-p-007` is the first admissible positive case: a measured 0.6096 m-wide
face at 0.523 m range and -29.05 degrees bearing, under nominal light and
medium texture, with translation motion. The run had 900 complete rows, 461
new samples, p95/p99/max timestamp skew of 35/36/38 ms, and zero transport
faults. Mean body speed was 0.109 m/s and mean absolute yaw rate was
0.118 rad/s, satisfying the translation evidence gate. The detector did not
assert a valid obstacle or cast a map vote, so this case is retained as a real
positive miss (current measured positive detection rate 0/1), not excluded.
The rejection telemetry explains where evidence was lost: across the run the
low-motion counter increased by 326, yaw-explained by 79, inverse-depth
disagreement by 75, and no-consistent-group by 53; dispersion never rejected a
sample. Across camera-sample snapshots, aggregate image displacement averaged
0.00276 rad versus the 0.004 rad gate and exceeded that gate on 135/460
samples. Only 57/460 snapshots simultaneously exceeded the motion gate and
had yaw-explained ratio at or below 0.8. A nonempty candidate group appeared
on four snapshots, but each produced only one temporal hit, so the required
two-of-three publication rule correctly prevented a single-frame detection.
This is evidence of insufficient persistent usable observations, not a UART
or logger failure.

A subsequent physically protected, log-only flight diagnostic explicitly armed
the Crazyflie, took off under PID to a commanded 0.30 m, switched to TinyMPC,
commanded six seconds of +/-0.10 m lateral peering, returned to the origin
setpoint, and landed under PID. The 634-row log is
`tools/flow_gap8_validation/hardware/logs/f27-protected-hover-positive-001.csv`. During the peer
phase, estimated lateral position ranged from -0.130 to +0.036 m and estimated
altitude averaged 0.271 m. Flow receive count advanced from 6098 to 6315, but
`mapPeak`, `cylValid`, `cylConf`, and the frozen-obstacle flag all remained
zero. Battery voltage sagged to 3.368 V, so no repeat flight should be attempted
before charging. This flight is retained as a diagnostic miss, not counted as
a manifest case: its flight logger does not capture the complete synchronized
raw-sector/rejection schema required by the corpus.

Early protected-flight attempts exposed an independent controller-handoff
hazard. In `f27-protected-hover-positive-002.csv`, switching from controller 1
to controller 6 in flight was followed by estimated excursions of x
-0.568...+0.502 m, y -0.550...+1.082 m, and z 0.009...0.586 m. A later
bounded retry aborted when altitude fell to 0.112 m. These are retained as
failed diagnostics. The flight harness now verifies takeoff and bounds the
flight envelope, but the important functional correction is `obs.pidPass`: the
OOT controller continues running its perception task while passing the
commander's position setpoint directly to stock PID. The harness selects this
mode while disarmed and no longer changes controller instances in flight.

With that no-handoff sequence, `f27-protected-hover-positive-008.csv` completed
a stable protected flight: during peering, x remained -0.014...+0.038 m, y
-0.080...+0.055 m, and z 0.317...0.351 m. It accumulated map evidence to
0.144 but did not reach the 0.20 cylinder-valid threshold. A longer/faster
10-second pass, `f27-protected-hover-positive-009.csv`, was similarly stable
(peer x -0.013...+0.029 m, y -0.092...+0.078 m, z
0.305...0.353 m) but accumulated no map evidence. Across 287 received flow
samples, its rejection counters increased by 198 low-motion, 45 inverse-depth
disagreement, one dispersion, and 84 no-consistent-group events. These flights
show that stable commanded motion alone does not make this scene reliably
detectable; neither diagnostic is promoted to synchronized corpus evidence.

After the obstacle was moved directly ahead, a first centered attempt
(`f27-protected-hover-centered-010.csv`) was rejected after takeoff because the
estimated horizontal position diverged to approximately (-0.77,+0.46) m. A
repeat after a five-second estimator convergence interval,
`f27-protected-hover-centered-011.csv`, was controlled: during peering x stayed
-0.013...+0.019 m, y -0.038...+0.029 m, and z 0.216...0.245 m. It
received 202 flow samples but produced zero map evidence. Rejection-counter
deltas were 207 low-motion, 7 yaw-explained, 100 inverse-depth disagreement,
one dispersion, and 31 no-consistent-group events. Thus centering the textured
face did not resolve the 27-feature miss. Battery ended at 3.447 V; further
flight on that pack was stopped.

The operator noted that the preceding centered command barely moved laterally,
so `f27-protected-hover-centered-012.csv` increased the command to +/-0.12 m.
The protected PID-passthrough flight remained controlled (peer x
-0.011...+0.024 m, y -0.096...+0.102 m, z 0.254...0.296 m) and
produced the first valid flight detection. Map evidence peaked at 0.927 and
the cylinder remained valid through landing; the frozen center was
(0.459,0.110) m. Against the nominal centered 0.5 m pose this is approximately
0.117 m position error, below the provisional 0.35 m threshold. The cylinder
first became valid during takeoff motion and the flow freeze latched before
the peer phase. This remains a diagnostic rather than a manifest result
because the flight logger does not contain the full synchronized sector schema.
Battery sagged to 3.405 V under load; the operator elected to continue testing
with the pack while retaining the existing pre-arm voltage gate.

Three synchronized, motors-disabled centered diagnostics then isolated
perception from flight-state instability. The first had clean 15 Hz transport
but unintentionally included substantial yaw and was invalid as pure
translation. The second preserved yaw but was too slow and the GAP8 stream
stalled after 49 new samples; measured delivery was 1.64 Hz with a 26.80 s
no-new-sample interval. The analyzer and coverage auditor now explicitly
require at least 12 Hz measured delivery and no stall longer than 0.5 s, with a
unit test for silent stream failure. This preserves the clean synchronized
corpus cases while rejecting the stalled diagnostic.

The third run, `f27-centered-manual-translation-diagnostic-3.csv`, was valid:
mean body speed 0.130 m/s, mean absolute yaw rate 0.082 rad/s, 461 new samples
at 15.38 Hz, maximum delivery stall 0.105 s, complete timestamps with p99
37 ms skew, and no transport faults. It detected after 2.00 s and had no
subsequent cylinder dropout. Mean range error was 0.164 m, bearing error
23.9 degrees, and world-position error 0.329 m. The latter is within the
provisional 0.35 m bound only narrowly; signed body-lateral error was
-0.265 m, so this run is evidence of material bearing/lateral bias. It remains
outside the exact manifest because the current centered planar/broad/medium/
nominal combination is not one of the manifest's assigned pose/attribute
combinations.

A matched left/right diagnostic pair used the same face at 0.5 m forward and
0.3 m lateral. The left case was admissible and detected at 17.67 s with no
dropout, 0.109 m range error, +16.5 degree signed bearing error, 0.178 m world
error, and +0.177 m signed body-lateral error. The right case was also
admissible and transport-clean but missed: two map votes peaked near 0.143,
below the 0.20 validity threshold. Thus this one-pair detection-rate difference
is 1.0, far outside the eventual <=0.10 symmetry gate, although a full
systematic-bias claim still requires the declared corpus. Right-side
inverse-depth disagreement rejections increased by 943 versus 596 on the left.
The two right-side votes were approximately 0.56 m apart, just outside the
0.55 m map merge radius; blindly widening that radius would combine estimates
near 0.70 m and 1.25 m and is not justified by localization quality.

The matched 36-feature manifest case `f36-p-007` was then clean-built and
flashed over JTAG at the unchanged 1500 kHz speed (L2 148328 B, FC TCDM
7740 B, L1 28 B). A first attempt was preserved under `logs/invalid_motion/`:
estimated mean/p95 speed was an implausible 1.59/2.40 m/s and a cylinder was
already valid at the first recorded row. The logger now activates OOT only
after the countdown, and translation evidence is bounded at <=0.75 m/s mean
and <=1.5 m/s p95.

The corrected `f36-p-007` run is admissible: 452 new samples, 15.08 Hz,
0.634 s startup latency, 0.104 s maximum sustained stall, complete synchronized
timestamps, and zero transport faults. It detected at 13.44 s without later
dropout. However, range error was 0.607 m, bearing error 47.9 degrees, and
world-position error 0.724 m. Thus 36 features changed the matched right-offset
case from a 27-feature miss to a detection, but the localization is more than
twice the provisional 0.35 m limit and is not suitable for avoidance.

The earlier `f27-n-022` translation run had valid motion and clean transport,
but its 100 ms logger produced p99 cross-block skew of 101 ms. It is preserved
under `logs/superseded_unsynchronized/` and no longer counts toward acceptance;
the case must be recollected. The corpus analyzer now records measured
speed/yaw and synchronization evidence. The coverage auditor requires exact
scene metadata, supported measured motion, at least 95% complete timestamp
sets, and p99 cross-block skew no greater than one 15 Hz flow period before a
sidecar can increase the completed-case count.

A synchronization sweep retained 30 ms blocks and 30 Hz CSV snapshots:
50/40/30 ms settings were transport-clean with p99 skew 55/40/36 ms,
respectively. A 20 ms setting starved parameter initialization after the 16 log
blocks started and was rejected. These synchronization runs are explicitly
outside the obstacle corpus.

`tools/flow_gap8_validation/hardware/audit_validation_bundle.py` verifies current commit/dirty state,
saved build hashes, compiled/Python equivalence, both timing gates, strict
corpus coverage, and the remaining quality gates. Its saved
`flow_obstacle_completion_audit.json` currently reports `incomplete`, as
intended: source/build/equivalence/timing pass, while only 3/276 synchronized
corpus cases are complete and the sole positive case was missed. Without
`--allow-incomplete`, the verifier exits nonzero.

Positive-case ground truth may now be entered as measured initial range and
bearing instead of requiring the operator to know estimator-world x/y before a
run. The analyzer anchors that polar measurement to the first finite logged
x/y/yaw pose. Positive launches also require the actual obstacle width/diameter
and orientation to be recorded; qualitative manifest labels are not silently
treated as physical dimensions.

The corpus analyzer now retains signed bearing, world-axis, and body-lateral
errors so left/right cancellation cannot hide bias behind absolute-error
metrics. The predeclared full-corpus bias gate requires symmetric orientation
and lateral-offset detection-rate differences <=0.10, absolute mean signed
bearing bias <=5 degrees, and absolute mean body-lateral bias <=0.10 m. It is
not evaluable until all 180 positive cases are present.

## Higher-rate experiment

A parameterized 27-feature image requested a 32 fps Himax rate and 61.5 ms
pipeline period (nominal 16+16 Hz). The sensor accepted and reported 32.00 fps,
but the first 30 s window completed only 876 camera captures and 445 flow/CNN
jobs, versus the requested 960 captures and 480 jobs. The effective pipeline
rate was therefore about 14.8 Hz, not 16 Hz. That window still had zero
flow/UART drops, errors, or camera recoveries; flow p95/p99 were ≤17.25 ms and
CNN p95/p99 were ≤51.5/51.75 ms.

The run then lost the Olimex USB device (`LIBUSB_ERROR_NO_DEVICE`). A subsequent
15 s CrazyRadio check received 224 clean new flow packets (~14.9 Hz), confirming
that GAP8 continued running but the end-to-end rate had not increased. This
experiment is rejected as a throughput improvement.

The default 30 fps, 65 ms, 27-feature image was subsequently restored. Two live
heartbeat windows explicitly reported `target_fps=30 period_us=65000 feat=27`,
the expected dirty build ID, matching flow/CNN invocation and completion
counts, and zero flow/TX drops, UART errors, camera recoveries, or I2C errors.
The parsed restoration log is saved as
`hardware_f27_restored_baseline{_raw.log,.summary.json,...}`.

## Foreground/background edge fix and protected detour

The clean, explicitly reset 36-feature centered diagnostic
`f36-centered-manual-translation-reset-diagnostic.csv` detected the 0.5 m
cube with 0.063 m range error, 13.8 degree bearing error, and 0.162 m world
error. Delivery was 15.08 Hz with a 0.103 s maximum sustained stall and zero
CRC, framing, sequence, or queue faults.

The matched right-offset foreground/background diagnostic initially remained
a clean miss: zero map votes despite 452 new samples at 15.08 Hz. Raw-sector
inspection showed that the near cube commonly occupied only one sector while
the distant wall occupied other sectors. The estimator formerly discarded
that singleton before its two-of-three temporal gate. The revised estimator
retains a one-sector spatial component as tentative evidence, but it still
cannot vote or publish from one camera sample. A spatially consistent
observation in another new frame remains mandatory.

Follow-up hardware runs exposed and corrected three additional stacked-map
effects:

- One temporally valid vote now satisfies the cylinder's near-range map
  confirmation because that vote already represents multiple camera samples.
  Tracks beyond 1.0 m instead require four persistent-map votes.
- Map merge and extraction radii are 0.30 m rather than 0.55 m, preventing
  0.5--0.6 m foreground fixes from blending with approximately 1.0 m
  background tracks.
- Published-cylinder association is retained through modest component jitter.
  A new validated component replaces it only when at least 0.50 m closer,
  allowing a cube to supersede a far wall without lateral component hopping.

The final motors-disabled right-offset run,
`f36-right-offset-track-association-retest.csv`, passed the provisional
localization gate: detection at 1.17 s, 0.122 m range error, 21.1 degree
bearing error, 0.286 m world error, and no cylinder dropout. It delivered
452 samples at 15.08 Hz with zero transport faults. The complete six-run
progression is machine-readable in
`tools/results/flow_obstacle_foreground_fix_2026-07-23.json`.

The estimator mirror has matching singleton, mixed foreground/wall, map
separation, nearest-hazard, far-confirmation, and association tests. All 18
estimator unit tests and all 13 hardware-analysis tests pass. The compiled
production C/Python equivalence suite now includes an explicit single-edge
foreground/far-wall case and passes 216 frames across 18 cases with maximum
absolute error 3.11e-7. The final STM32 build used 337800 B flash, 107940 B
RAM, and 62392 B CCM before the flight-planner-only edit.

Protected flight `f36-protected-pid-detour-001.csv` retained the stable
OOT-plus-PID-passthrough controller, detected and froze a flow obstacle at
(0.559, 0.061) m, and executed a detection-derived three-segment detour before
landing and disarming. During the avoidance phases, altitude stayed
0.295--0.312 m. Actual minimum distance to the frozen center was 0.538 m
against a 0.55 m target. The run received 420 flow packets and battery voltage
fell to 3.336 V during landing.

This flight also found a harness geometry defect: the requested (1.0, 0.0) m
goal was only 0.445 m from the frozen center, so the final commanded segment
entered the clearance circle even though tracking lag kept the actual vehicle
outside it. The planner now rejects start or goal poses inside the requested
clearance and retains the lateral offset until the goal's along-track
coordinate. A replay with the captured frozen center rejects the unsafe
1.0 m goal and produces a 0.5500000002 m minimum commanded clearance for a
1.2 m goal. No repeat flight is authorized on the depleted pack.

## Approach-flight collision and flight-test stop

Protected approach run `f36-protected-approach-detour-002.csv` collided with
the cube. The operator measured the cube at approximately 1.25 m ahead. During
the approach, the estimator froze a low-confidence obstacle at
(1.627, 0.554) m with confidence 0.118 while the estimated vehicle pose was
(0.731, 0.113, 0.306) m. The resulting host-planned detour treated the
obstacle as substantially left of the real corridor and therefore did not
protect the true obstacle position.

After impact, the logged state estimate diverged to physically impossible
values (up to x=7.85 m, y=14.28 m, and z outside -3.76--1.28 m). The telemetry
freshness guard subsequently aborted command streaming, but it cannot prevent
an impact caused by an incorrect obstacle position. The vehicle was stopped
and the operator confirmed it was safe.

The failure exposed an unsafe acceptance boundary: a single persistent-map
vote could publish at ranges up to 1.0 m, including this 0.118-confidence
estimate. The fast one-vote range is now limited to 0.75 m; estimates beyond
0.75 m require four persistent-map votes. The flight harness also rejects PID
detours with frozen confidence below 0.25, which would have rejected this run.
The new explicit 0.9 m boundary regression, all 19 estimator tests, all 13
hardware-analysis tests, and the 216-frame production-C/Python equivalence
suite pass. Maximum C/Python error remains 3.11e-7.

These changes are offline hardening only. The collision invalidates the
current system for autonomous avoidance: no further obstacle-avoidance flight
is authorized until the revised estimator has been flashed and the 1.25 m
centered and offset cases pass stationary, motors-disabled bench validation
with independently measured range and bearing.
