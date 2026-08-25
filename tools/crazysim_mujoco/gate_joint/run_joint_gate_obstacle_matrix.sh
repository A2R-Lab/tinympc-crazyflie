#!/usr/bin/env bash
# Reproducible, intentionally sequential held-out matrix for the isolated
# joint gate + obstacle experiment.  This runner never changes controller
# costs: every control setting is passed explicitly to run.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRAZYSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$CRAZYSIM_DIR/../.." && pwd)"
RUNNER="$CRAZYSIM_DIR/run.sh"
EVALUATOR="$SCRIPT_DIR/evaluate_joint_gate_obstacle.py"
MATRIX_RUNNER="$SCRIPT_DIR/run_joint_gate_obstacle_matrix.sh"
SIM_ROOT="$REPO_DIR/apps/controller_tinympc_eigen/sim_runs/crazysim"
BASELINE_POLICY="$CRAZYSIM_DIR/models/vision_rl_mpc_balanced_v2/policy.onnx"
BASELINE_POLICY_SHA256="291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e"
SEED_START=3101
# This is deliberately a new course/matrix namespace.  Do not point this
# runner at the historical POC paths: its evidence remains immutable.
SEED_COUNT=10
CANDIDATE_INPUT=""
OUT_ROOT=""
DRY_RUN=0
CONTAINER_ENGINE=docker
APPTAINER_IMAGE=""

usage() {
  cat <<'EOF'
Usage: run_joint_gate_obstacle_matrix.sh --candidate-bundle PATH --out-root DIRECTORY [options]

Run exactly ten sequential, paired held-out seeds through:
  candidate joint_gate_rl / gate_obstacle_easy_transition
  candidate joint_gate_rl / gate_obstacle_easy_transition_obstacle_only
  frozen hybrid_rl / gate_obstacle_easy_transition_obstacle_only

Options:
  --candidate-bundle PATH  Candidate bundle.json (or its containing directory).
  --out-root DIRECTORY     New matrix directory below sim_runs/crazysim.
  --seed-start INTEGER     First of ten paired seeds (default: 3101).
  --dry-run                Validate and print the 30 planned runs; launch nothing.
  --container-engine NAME  docker (default) or apptainer.
  --apptainer-image PATH   Required shared .sif for the apptainer engine.
  -h, --help               Show this help.

The runner refuses an existing output root, takes no --overwrite option, runs
strictly sequentially, and writes matrix_status.tsv as three tab-separated
fields: group, seed, run.sh exit code.  It intentionally requests no videos.
EOF
}

die() { echo "error: $*" >&2; exit 2; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --candidate-bundle) CANDIDATE_INPUT="${2:-}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:-}"; shift 2 ;;
    --seed-start) SEED_START="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --container-engine) CONTAINER_ENGINE="${2:-}"; shift 2 ;;
    --apptainer-image) APPTAINER_IMAGE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ -n "$CANDIDATE_INPUT" ]] || die "--candidate-bundle is required"
[[ -n "$OUT_ROOT" ]] || die "--out-root is required"
[[ "$SEED_START" =~ ^[0-9]+$ ]] || die "--seed-start must be a nonnegative integer"
[[ "$CONTAINER_ENGINE" == docker || "$CONTAINER_ENGINE" == apptainer ]] || die "invalid --container-engine"
[[ "$CONTAINER_ENGINE" != apptainer || -f "$APPTAINER_IMAGE" ]] || die "apptainer requires --apptainer-image"
[[ -x "$RUNNER" ]] || die "missing executable runner: $RUNNER"
[[ -f "$EVALUATOR" ]] || die "missing evaluator: $EVALUATOR"

CANDIDATE_INPUT="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$CANDIDATE_INPUT")"
if [[ -d "$CANDIDATE_INPUT" ]]; then
  if [[ -f "$CANDIDATE_INPUT/bundle.json" ]]; then
    CANDIDATE_MANIFEST="$CANDIDATE_INPUT/bundle.json"
  elif [[ -f "$CANDIDATE_INPUT/bundle_manifest.json" ]]; then
    CANDIDATE_MANIFEST="$CANDIDATE_INPUT/bundle_manifest.json"
  else
    die "candidate directory has neither bundle.json nor bundle_manifest.json"
  fi
elif [[ -f "$CANDIDATE_INPUT" ]]; then
  CANDIDATE_MANIFEST="$CANDIDATE_INPUT"
else
  die "candidate bundle does not exist: $CANDIDATE_INPUT"
fi

# Fail before an expensive matrix if either frozen identity or the candidate's
# declared policy path/hash is not precisely usable.  The evaluator repeats
# the candidate validation against the immutable provenance snapshot below.
python3 - "$CANDIDATE_MANIFEST" "$BASELINE_POLICY" "$BASELINE_POLICY_SHA256" <<'PY'
import hashlib, json, sys
from pathlib import Path

manifest = Path(sys.argv[1])
baseline = Path(sys.argv[2])
expected_baseline = sys.argv[3]
def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()
try:
    data = json.loads(manifest.read_text())
    policy = data["artifacts"]["policy_onnx"]
    relative = policy["path"]
    expected = policy["sha256"]
    path = (manifest.parent / relative).resolve()
    if path.parent != manifest.parent.resolve() or not path.is_file():
        raise ValueError("policy must be a direct child of the candidate bundle")
    if digest(path) != expected:
        raise ValueError("candidate policy hash mismatch")
    if data.get("format") != "tinympc-joint-gate-obstacle-student-v1":
        raise ValueError("unexpected candidate format")
    if data.get("runtime_adapter") != "joint_gate_rl":
        raise ValueError("candidate runtime_adapter is not joint_gate_rl")
    if digest(baseline) != expected_baseline:
        raise ValueError("frozen hybrid policy hash mismatch")
except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
    raise SystemExit(f"invalid model provenance: {exc}")
PY

if [[ "$OUT_ROOT" != /* ]]; then OUT_ROOT="$REPO_DIR/$OUT_ROOT"; fi
OUT_ROOT="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$OUT_ROOT")"
case "$OUT_ROOT" in "$SIM_ROOT"/*) ;; *) die "--out-root must be below $SIM_ROOT" ;; esac
[[ ! -e "$OUT_ROOT" ]] || die "output root already exists (refusing overwrite): $OUT_ROOT"

declare -a SEEDS=()
for ((index = 0; index < SEED_COUNT; ++index)); do SEEDS+=("$((SEED_START + index))"); done

if [[ "$DRY_RUN" == 1 ]]; then
  printf 'candidate_manifest=%s\n' "$CANDIDATE_MANIFEST"
  printf 'out_root=%s\n' "$OUT_ROOT"
  for group in candidate_gate_obstacle candidate_obstacle_only baseline_obstacle_only; do
    for seed in "${SEEDS[@]}"; do printf '%s\t%s\n' "$group" "$seed"; done
  done
  exit 0
fi

mkdir -p "$OUT_ROOT/provenance/candidate_bundle"
MATRIX_TOOL_DIR="$OUT_ROOT/provenance/matrix_tools"
mkdir -p "$MATRIX_TOOL_DIR"
MATRIX_RUNNER_SNAPSHOT="$MATRIX_TOOL_DIR/$(basename "$MATRIX_RUNNER")"
EVALUATOR_SNAPSHOT="$MATRIX_TOOL_DIR/$(basename "$EVALUATOR")"
cp --preserve=mode,timestamps "$MATRIX_RUNNER" "$MATRIX_RUNNER_SNAPSHOT"
cp --preserve=mode,timestamps "$EVALUATOR" "$EVALUATOR_SNAPSHOT"
CANDIDATE_SNAPSHOT_DIR="$OUT_ROOT/provenance/candidate_bundle"
CANDIDATE_SNAPSHOT_MANIFEST="$CANDIDATE_SNAPSHOT_DIR/$(basename "$CANDIDATE_MANIFEST")"
candidate_policy_relative="$(python3 - "$CANDIDATE_MANIFEST" <<'PY'
import json, sys
print(json.load(open(sys.argv[1]))["artifacts"]["policy_onnx"]["path"])
PY
)"
cp --preserve=mode,timestamps "$CANDIDATE_MANIFEST" "$CANDIDATE_SNAPSHOT_MANIFEST"
while IFS= read -r artifact_relative; do
  mkdir -p "$CANDIDATE_SNAPSHOT_DIR/$(dirname "$artifact_relative")"
  cp --preserve=mode,timestamps "$(dirname "$CANDIDATE_MANIFEST")/$artifact_relative" \
    "$CANDIDATE_SNAPSHOT_DIR/$artifact_relative"
done < <(python3 - "$CANDIDATE_MANIFEST" <<'PY'
import json, sys
manifest = json.load(open(sys.argv[1]))
for name in ("checkpoint", "policy_onnx"):
    print(manifest["artifacts"][name]["path"])
PY
)
cp --preserve=mode,timestamps "$BASELINE_POLICY" "$OUT_ROOT/provenance/frozen_hybrid_policy.onnx"

python3 - "$CANDIDATE_SNAPSHOT_MANIFEST" "$OUT_ROOT/provenance/frozen_hybrid_policy.onnx" \
  "$OUT_ROOT/provenance/model_provenance.json" <<'PY'
import hashlib, json, sys
from pathlib import Path
def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""): h.update(block)
    return h.hexdigest()
candidate, baseline, output = map(Path, sys.argv[1:])
policy_rel = json.loads(candidate.read_text())["artifacts"]["policy_onnx"]["path"]
output.write_text(json.dumps({
    "candidate_bundle": str(candidate), "candidate_bundle_sha256": digest(candidate),
    "candidate_policy_sha256": digest(candidate.parent / policy_rel),
    "baseline_policy": str(baseline), "baseline_policy_sha256": digest(baseline),
}, indent=2) + "\n")
PY

status_file="$OUT_ROOT/matrix_status.tsv"
: > "$status_file"

run_one() {
  local group="$1" seed="$2" course="$3" model="$4" adapter="$5"
  local output="$OUT_ROOT/${group}_seed${seed}"
  local container_args=(--container-engine "$CONTAINER_ENGINE")
  [[ "$CONTAINER_ENGINE" != apptainer ]] || container_args+=(--apptainer-image "$APPTAINER_IMAGE")
  set +e
  "$RUNNER" \
    --trajectory straight_9m --duration 30 --launch-time 1 --launch-prespin 1 \
    --actuator-lti 1 --rate-cascade 0 --progress-speed-mps 0.5 \
    --progress-entry-acceleration-mps2 1.0 --progress-terminal-deceleration-mps2 1.5 \
    --progress-reward-weight 0.40 --firmware-time-factor 0.1347944 --realtime-factor 0.2 \
    --no-flowdeck --stop-on-contact --vision-latency-frames 1 --course "$course" \
    --vision-model "$model" --vision-adapter "$adapter" --random-seed "$seed" --out "$output" \
    "${container_args[@]}"
  local result=$?
  set -e
  printf '%s\t%s\t%s\n' "$group" "$seed" "$result" >> "$status_file"
  if [[ "$result" != 0 ]]; then
    printf 'aborting matrix after failed run: group=%s seed=%s exit=%s\n' \
      "$group" "$seed" "$result" >&2
    exit "$result"
  fi
}

# Each run must finish before the next begins: CPU calibration, filesystem
# artifacts, and one shared CrazySim build are deliberately not concurrent.
set -e
for seed in "${SEEDS[@]}"; do
  run_one candidate_gate_obstacle "$seed" gate_obstacle_easy_transition "$CANDIDATE_SNAPSHOT_DIR" joint_gate_rl
  run_one candidate_obstacle_only "$seed" gate_obstacle_easy_transition_obstacle_only "$CANDIDATE_SNAPSHOT_DIR" joint_gate_rl
  run_one baseline_obstacle_only "$seed" gate_obstacle_easy_transition_obstacle_only "$OUT_ROOT/provenance/frozen_hybrid_policy.onnx" hybrid_rl
done

# Build the evaluator's common provenance from the generated run_config files,
# rather than duplicating our requested flags.  This makes a timing/build drift
# a first-class failure instead of silently evaluating a mixed matrix.
common="$OUT_ROOT/common_provenance.json"
set +e
python3 - "$OUT_ROOT" "$common" "${SEEDS[@]}" <<'PY'
import json, sys
from pathlib import Path

root, output, *seeds = map(Path, sys.argv[1:])
groups = ("candidate_gate_obstacle", "candidate_obstacle_only", "baseline_obstacle_only")
paths = [root / f"{group}_seed{seed}" / "run_config.json" for group in groups for seed in seeds]
if any(not path.is_file() for path in paths):
    missing = [str(path) for path in paths if not path.is_file()]
    raise SystemExit("missing run configs: " + ", ".join(missing))
configs = [json.loads(path.read_text()) for path in paths]
keys = (
    "format", "repository", "repository_commit", "repository_dirty", "runner_sha256",
    "controller_source_sha256", "trajectory_header_sha256", "trajectory", "actuator_lti",
    "rate_cascade", "reference_mode", "duration_s", "launch_time_s", "launch_prespin",
    "spawn_z_m", "model", "mass", "pwm_thrust_full_n", "inertia_scale", "motor_tau_scale",
    "thrust_scale", "realtime_factor", "firmware_time_factor", "vision_latency_frames",
    "camera_fps", "stop_on_contact", "level_cost_mode", "progress_sample_limit",
    "progress_speed_mps", "progress_reference_limits", "progress_laps",
    "progress_entry_acceleration_mps2", "progress_terminal_deceleration_mps2",
    "progress_reward_weight", "flowdeck_enabled", "direct_plan_replay", "mpc_diagnostic",
    "acceptance_contract.controller_sha256", "acceptance_contract.bank_header_sha256",
    "acceptance_contract.vision_bridge_sha256", "acceptance_contract.analyzer_sha256",
    "acceptance_contract.gate_corner_span_m",
    "acceptance_contract.bank_provenance_sha256", "acceptance_contract.progress_path_sha256",
    "acceptance_contract.frenet_error_sha256", "acceptance_contract.generated_model_sha256",
    "acceptance_contract.plant_model_sha256",
)
def dotted(config, key):
    value = config
    for part in key.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value
common = {}
for key in keys:
    values = [dotted(config, key) for config in configs]
    if any(value is None for value in values):
        raise SystemExit(f"missing common provenance field: {key}")
    if any(value != values[0] for value in values[1:]):
        raise SystemExit(f"non-common provenance field: {key}")
    common[key] = values[0]
output.write_text(json.dumps(common, indent=2, sort_keys=True) + "\n")
PY
provenance_status=$?
set -e
if [[ "$provenance_status" != 0 ]]; then
  printf 'evaluation skipped: common provenance could not be verified\n' > "$OUT_ROOT/evaluation_skipped.txt"
  exit "$provenance_status"
fi

cmp -s "$MATRIX_RUNNER" "$MATRIX_RUNNER_SNAPSHOT" || die "matrix runner changed during execution"
cmp -s "$EVALUATOR" "$EVALUATOR_SNAPSHOT" || die "evaluator changed during execution"
eval_args=(--candidate-bundle "$CANDIDATE_SNAPSHOT_MANIFEST" --common-provenance "$common" --out-dir "$OUT_ROOT/evaluation" --seed-start "$SEED_START" --matrix-runner-snapshot "$MATRIX_RUNNER_SNAPSHOT" --evaluator-snapshot "$EVALUATOR_SNAPSHOT")
for seed in "${SEEDS[@]}"; do
  eval_args+=(--candidate-gate-obstacle-run "$OUT_ROOT/candidate_gate_obstacle_seed${seed}")
  eval_args+=(--candidate-obstacle-only-run "$OUT_ROOT/candidate_obstacle_only_seed${seed}")
  eval_args+=(--baseline-obstacle-only-run "$OUT_ROOT/baseline_obstacle_only_seed${seed}")
done
python3 "$EVALUATOR" "${eval_args[@]}"
