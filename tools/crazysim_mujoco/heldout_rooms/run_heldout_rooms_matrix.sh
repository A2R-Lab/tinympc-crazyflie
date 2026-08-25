#!/usr/bin/env bash
# Sequential opt-in held-out room matrix.  It never overwrites evidence.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; sim="$(cd "$here/.." && pwd)"; repo="$(cd "$sim/../.." && pwd)"
runner="$sim/run.sh"; evaluator="$here/evaluate_heldout_rooms.py"
matrix_runner="$here/run_heldout_rooms_matrix.sh"
bundle=""; out=""; engine=docker; image=""; dry=0; seed_start=4801
usage(){ echo "Usage: $0 --candidate-bundle PATH --out-root PATH [--seed-start INTEGER] [--container-engine docker|apptainer --apptainer-image PATH] [--dry-run]"; }
while (($#)); do case "$1" in --candidate-bundle) bundle="$2"; shift 2;; --out-root) out="$2"; shift 2;; --seed-start) seed_start="$2"; shift 2;; --container-engine) engine="$2"; shift 2;; --apptainer-image) image="$2"; shift 2;; --dry-run) dry=1; shift;; -h|--help) usage; exit 0;; *) echo "unknown argument: $1" >&2; exit 2;; esac; done
[[ -f "$runner" && -f "$evaluator" && -n "$bundle" && -n "$out" ]] || { usage >&2; exit 2; }
[[ "$seed_start" =~ ^[0-9]+$ ]] || { echo "seed start must be a nonnegative integer" >&2; exit 2; }
seeds=(); for ((offset=0; offset<5; ++offset)); do seeds+=("$((seed_start + offset))"); done
[[ "$out" = /* ]] || out="$repo/$out"; [[ ! -e "$out" ]] || { echo "refusing existing output root" >&2; exit 2; }
[[ "$engine" != apptainer || -f "$image" ]] || { echo "Apptainer requires --apptainer-image" >&2; exit 2; }
if ((dry)); then for c in straight circle oval figure8; do for s in "${seeds[@]}"; do echo "heldout_room_${c} seed=$s"; done; done; exit 0; fi
bundle="$(realpath "$bundle")"
policy_rel="$(python3 - "$bundle" <<'PY'
import hashlib, json, sys
from pathlib import Path
bundle = Path(sys.argv[1]); data = json.loads(bundle.read_text())
if data.get("format") != "tinympc-joint-gate-obstacle-student-v1": raise SystemExit("unexpected candidate bundle format")
if data.get("runtime_adapter") != "joint_gate_rl": raise SystemExit("candidate bundle runtime_adapter is not joint_gate_rl")
policy = data.get("artifacts", {}).get("policy_onnx", {}); rel, expected = policy.get("path"), policy.get("sha256")
if not isinstance(rel, str) or Path(rel).name != rel or not isinstance(expected, str) or len(expected) != 64: raise SystemExit("invalid candidate policy provenance")
path = bundle.parent / rel
h = hashlib.sha256(path.read_bytes()).hexdigest()
if h != expected: raise SystemExit("candidate policy hash mismatch")
print(rel)
PY
)"
mkdir -p "$out/provenance/candidate_bundle"; cp --preserve=mode,timestamps "$bundle" "$out/provenance/candidate_bundle/bundle.json"; sha256sum "$bundle" > "$out/provenance/candidate_bundle.sha256"
matrix_tool_dir="$out/provenance/matrix_tools"; mkdir -p "$matrix_tool_dir"
matrix_runner_snapshot="$matrix_tool_dir/$(basename "$matrix_runner")"
evaluator_snapshot="$matrix_tool_dir/$(basename "$evaluator")"
cp --preserve=mode,timestamps "$matrix_runner" "$matrix_runner_snapshot"
cp --preserve=mode,timestamps "$evaluator" "$evaluator_snapshot"
policy_src="$(dirname "$bundle")/$policy_rel"
[[ -f "$policy_src" ]] || { echo "candidate policy missing: $policy_src" >&2; exit 2; }
mkdir -p "$out/provenance/candidate_bundle/$(dirname "$policy_rel")"
cp --preserve=mode,timestamps "$policy_src" "$out/provenance/candidate_bundle/$policy_rel"
sha256sum "$out/provenance/candidate_bundle/bundle.json" "$out/provenance/candidate_bundle/$policy_rel" > "$out/provenance/model_snapshot.sha256"
frozen_bundle="$out/provenance/candidate_bundle"
status="$out/matrix_status.tsv"; : > "$status"
for c in straight circle oval figure8; do for s in "${seeds[@]}"; do
  name="heldout_room_${c}"; dest="$out/${name}_seed${s}"; set +e
  trajectory="$c"; [[ "$c" != straight ]] || trajectory=straight_9m
  args=(--trajectory "$trajectory" --course "$name" --duration 30 --launch-time 1 --launch-prespin 1
    --actuator-lti 1 --rate-cascade 0 --progress-speed-mps 0.5
    --progress-entry-acceleration-mps2 1.0 --progress-terminal-deceleration-mps2 1.5
    --progress-reward-weight 0.40 --realtime-factor 0.2 --firmware-time-factor 0.1047352488
    --no-flowdeck --stop-on-contact --vision-latency-frames 1
    --vision-model "$frozen_bundle" --vision-adapter joint_gate_rl
    --random-seed "$s" --out "$dest" --container-engine "$engine")
  [[ "$engine" != apptainer ]] || args+=(--apptainer-image "$image")
  "$runner" "${args[@]}"
  rc=$?; set -e; printf '%s\t%s\t%s\n' "$name" "$s" "$rc" >> "$status"
  if [[ "$rc" != 0 ]]; then
    printf 'aborting matrix after failed run: course=%s seed=%s exit=%s\n' \
      "$name" "$s" "$rc" >&2
    exit "$rc"
  fi
done; done
cmp -s "$matrix_runner" "$matrix_runner_snapshot" || { echo "matrix runner changed during execution" >&2; exit 2; }
cmp -s "$evaluator" "$evaluator_snapshot" || { echo "evaluator changed during execution" >&2; exit 2; }
python3 "$evaluator" --matrix-root "$out" --out-dir "$out/evaluation" \
  --seed-start "$seed_start" --matrix-runner-snapshot "$matrix_runner_snapshot" \
  --evaluator-snapshot "$evaluator_snapshot"
