# Joint temporal policy: packed GAP8 GVSOC validation

This is an opt-in deployment-validation branch. It does not alter the frozen
runtime ONNX bundle, TinyMPC Q/R, cached banks, or the default policy. Final
deployment validation is **GVSOC-only**: this workflow never requires a
physical GAP8 board or a flash operation on hardware.

## Packed export

`deploy_model.py` re-expresses an accepted `JointTemporalPolicy` checkpoint as
one NEMO/DORY-compatible graph with input `uint8[1,2,160,160]` in
`[previous,current]` order. Its terminal tensor is `uint8[1,12,1,1]`, with one
packed value for each of:

1. action logits: `TRACK`, `LEFT`, `RIGHT` (indices 0--2);
2. gate-corner logits: `tl_x`, `tl_y`, `tr_x`, `tr_y`, `br_x`, `br_y`,
   `bl_x`, `bl_y` (indices 3--10); and
3. gate-confidence logit (index 11).

The terminal affine decode and corner sigmoid stay outside DORY. The float
export first emits `deployment_contract.json`, explicitly identified as a
pre-quantization contract. After NeMO integerization, the exporter emits the
authoritative `quantized_deployment_contract.json`, including the terminal
epsilon and byte decode `raw=(encoded*epsilon-shift)/scale`. The GVSOC checker
requires that contract and rejects an epsilon mismatch. Consequently the graph
supplied to DORY contains only the supported packed `Conv`, `Relu`, and
`AveragePool` path--not runtime `Slice`, `Concat`, `Reshape`, or multiple ONNX
outputs.

`export_joint_dory.py` seals 200 deterministic clips and their source hashes.
They were excluded from network training, but terminal-affine fitting and NeMO
calibration also consume them. They are therefore named the
**calibration/parity corpus**, not a post-quantization generalization set.
`nemo_joint_integer_export.py` proves pre-GVSOC integer parity for that same
packed representation. These are prerequisite checks; they are not a
substitute for GVSOC acceptance or evidence of unseen-scene accuracy.

### Signed-weight export contract

GAP8 PULP-NN convolution kernels consume weights as signed `int8`. NeMO's
nominal 8-bit PACT integerization can emit a small number of coefficients above
`+127` or below `-128`; allowing DORY to cast those bytes would wrap them and
change the network. The exporter therefore applies signed saturation to the
integerized convolution weights *before* both golden inference and ONNX export,
records the number changed, and audits every ONNX weight initializer. This
keeps 8-bit activations and nominal 8-bit weight quantization while making the
goldens, ONNX graph, DORY payload, and kernel interpretation one explicit
deployment model. Non-integral or still-out-of-range weights fail closed.

### Exact wide BN/ReLU lowering

Some NeMO calibration graphs use non-power-of-two post-BN multipliers.  Their
coefficient products can exceed signed int32 even though the deployed GAP8
PULP-NN backend already supports signed int64 `k` and `lambda` constants.
`dory_wide_bnrelu.py` selects that existing `pulp-nn/64bit` backend and lowers
the expression exactly: powers of two are folded into the power-of-two
divisor; other multipliers are applied to both signed coefficients before the
unchanged divisor. It rejects non-integral constants, invalid divisors, and
any value outside signed int64 before packing, including a conservative check
that `k * int32_accumulator + lambda` fits the deployed kernel's signed-int64
intermediate. It never clips or wraps a coefficient. `dory_frontend.json`
records this lowering choice.

## Bounded 200-clip GVSOC harness

`instrument_gvsoc_harness.py` packages the sealed corpus in READFS as exactly
200 files, `joint_gvsoc_clip_000.hex` through
`joint_gvsoc_clip_199.hex`, before the generated `READFS_FILES :=` snapshot.
The generated C derives those names in numerical order and, for every clip,
loads one 51,200-byte input into a single reusable HyperRAM staging buffer,
runs the network, and emits one `JOINT_GVSOC` output/cycle row. The complete
10,240,000-byte corpus is therefore never resident in HyperRAM. The staging
allocation is freed after clip 199. For clip zero, the harness also compares
the generated-C checksum after every layer with DORY's sealed golden; the
parser fails at any intermediate mismatch rather than trusting terminal
metrics alone.

The host test `test_instrument_gvsoc_harness.py` verifies the filename order
and payload bytes, placement before the READFS snapshot, repeat-install
idempotency, unchanged `JOINT_L2_BYTES`, and the bounded C allocation/load/
free pattern. Instrumentation also patches generated src/mem.c so
load_file_to_ram() closes its READFS descriptor after a complete transfer.
This applies to both the 200 input clips and generated weight loads; omitting
that close exhausts the finite descriptor table at clip 150 (GVSOC exit 252).
The patch fails closed if the expected generated loader is absent and is
idempotent.

## GVSOC acceptance and memory accounting

`run_joint_dory_gvsoc.sbatch` is a reproducible CPU-only conversion and GVSOC
job script. `make ... platform=gvsoc` exercises the simulator only; it does
not authorize physical flashing. A final claim requires
`check_gvsoc_acceptance.py` to consume all 200 ordered generated-C rows and
their per-inference simulated cycles. It checks at the declared clock:

- action agreement >= 0.95;
- visible gate-center error increase <= 5 px; and
- p95 simulated inference latency < 33 ms.

The post-audit checker also decodes confidence and individual corners at the
deployed threshold and reproduces the static bridge/controller gate-admission
decision. Those supplemental checks catch a conversion that preserves actor
argmax while materially damaging gate use. Their thresholds are labeled
post-audit in the JSON rather than presented as preregistered criteria.

The build uses all eight GAP8 cluster processing elements. This is not merely
a throughput setting: the copied PULP-NN kernels use power-of-two row
partitioning (`log2(NUM_CORES)` and `NUM_CORES-1` masks). A seven-core build
can overlap or omit output rows and is therefore rejected by the mandatory
per-layer checksum gate.

`parse_gvsoc_harness.py` combines linker-reported static L1/L2 usage with the
literal dynamic allocations in generated C (`pmsis_l1_malloc`,
`JOINT_L2_BYTES`, and the terminal output). Flash accounting starts from the
complete generated `target.board.devices.flash.img` and subtracts only the
exact 200 raw evaluation clips. Code, DORY weights, filesystem overhead,
alignment, boot payload, and the generated default input remain, making the
reported deployable footprint a conservative upper bound rather than a
weights-only estimate. The final acceptance gate checks those totals against
64,000 B L1, 512,000 B L2, and 8 MiB flash capacities. It also binds the
exported checkpoint to both artifact hashes in the accepted runtime bundle.
DORY's frontend tile estimate remains useful diagnostic evidence, but it is
not substituted for the generated build and runtime allocations.

Terminal Slurm job 5968 passed this gate on GVSOC with all 200 outputs in
order: 0.990 action agreement, +3.246 px visible gate-center degradation,
14.822 ms p95 at 100 MHz, 36,728 B L1, 466,152 B L2, and a 372,256 B
deployment-flash upper bound. All six generated-C layer checks matched their
goldens exactly. This establishes generated-code parity on the calibration
corpus; it does not establish perception generalization or real-chip timing.
The audited static admission recall is 23/71 for float and 16/71 after
quantization, so the result should not be interpreted as a high-recall gate
detector claim.
Compact reports are under `../evidence/gvsoc_job5968/`.

## Preserved historical attempts

Previous attempts and their artifacts are retained for provenance, not as
descriptions of the current pipeline. In particular,
`evidence/5913_nemo_dory_blocker.json` records an earlier terminal-affine
investigation; the current export-only terminal-margin configuration has
already passed DORY frontend, GAP8 tiling, and C generation. The canceled
monolithic-corpus GVSOC attempt is retained under `runs_gvsoc_retry/5930/`.
Its repetitive simulator log was losslessly compressed to `gvsoc.log.gz`; its
original and compressed hashes, byte counts, and reclaimed space are in
`gvsoc_log_provenance.json`. The streaming harness above replaces that
monolithic HyperRAM design. No historical evidence is deleted by this branch.
