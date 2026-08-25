#!/usr/bin/env bash
# Submit only after checking live sinfo and squeue; partition/model intentionally omitted.
#SBATCH --job-name=joint_gate_obstacle_v3
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --gres=gpu:1
#SBATCH --time=08:00:00
#SBATCH --output=/home/cchen/isaacsim-workspace/workspace/logs/%x-%j.out
#SBATCH --error=/home/cchen/isaacsim-workspace/workspace/logs/%x-%j.err

set -euo pipefail
REPO=/home/cchen/isaacsim-workspace
PYTHON=/home/cchen/isaacsim-env/bin/python
RUN_ROOT="$REPO/workspace/joint_gate_obstacle_runs"
mkdir -p "$REPO/workspace/logs" "$RUN_ROOT"
RUN_DIR="$RUN_ROOT/${SLURM_JOB_ID}"
mkdir -p "$RUN_DIR"
on_exit() {
  status=$?
  printf '%s\n' "$status" > "$RUN_DIR/exit_status.txt"
  date --iso-8601=seconds >> "$RUN_DIR/finished_at.txt"
  exit "$status"
}
trap on_exit EXIT
cd "$REPO"
sinfo > "$RUN_DIR/sinfo_at_start.txt"
squeue -u "$USER" > "$RUN_DIR/squeue_at_start.txt"
git rev-parse HEAD > "$RUN_DIR/git_commit.txt"
git status --short > "$RUN_DIR/git_status.txt"
printf 'job_id=%s node=%s cuda_visible_devices=%s\n' "$SLURM_JOB_ID" "$(hostname)" "${CUDA_VISIBLE_DEVICES:-unset}" | tee "$RUN_DIR/metadata.txt"
command -v "$PYTHON" | tee "$RUN_DIR/python_path.txt"
"$PYTHON" --version | tee "$RUN_DIR/python_version.txt"
nvidia-smi | tee "$RUN_DIR/nvidia_smi.txt"
EPISODES="${EPISODES:-90}"
LAYOUT_START_INDEX="${LAYOUT_START_INDEX:-29000}"
SEED="${SEED:-20260826}"
FRAMES="${FRAMES:-210}"
RT_SUBFRAMES="${RT_SUBFRAMES:-1}"
CONFIG="$RUN_DIR/config.json"
printf '{"episodes":%s,"layout_start_index":%s,"seed":%s,"frames":%s,"rt_subframes":%s}\n' "$EPISODES" "$LAYOUT_START_INDEX" "$SEED" "$FRAMES" "$RT_SUBFRAMES" > "$CONFIG"
OUTPUT="${OUTPUT:-$REPO/workspace/joint_gate_obstacle_rollouts_v3_job_${SLURM_JOB_ID}}"
if [[ -e "$OUTPUT" ]]; then echo "refusing to overwrite $OUTPUT" >&2; exit 2; fi
PYTHONPATH=. "$PYTHON" user_workflows/export_joint_gate_obstacle_rollouts.py --output-dir "$OUTPUT" --episodes "$EPISODES" --layout-start-index "$LAYOUT_START_INDEX" --seed "$SEED" --frames "$FRAMES" --rt-subframes "$RT_SUBFRAMES"
for artifact in _SUCCESS manifest.json dataset_manifest.json transitions.jsonl train.npz validation.npz test.npz; do
  if [[ ! -f "$OUTPUT/$artifact" ]]; then
    echo "dataset export did not produce required artifact: $OUTPUT/$artifact" >&2
    exit 1
  fi
done
cp "$CONFIG" "$OUTPUT/run_config.json"
"$PYTHON" - "$OUTPUT" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
manifest = json.loads((root / "manifest.json").read_text())
success = json.loads((root / "_SUCCESS").read_text())
assert manifest["schema_version"] == "joint_gate_obstacle_rollout_v3_easy_transition"
assert manifest["families"] == {"gate_only": 30, "obstacle_only": 30, "gate_nearby_obstacle": 30}
assert manifest["easy_transition_course"]["gate_to_box_leading_face_gap_m"] == 2.05
assert len(manifest["source_sha256"]) == 4
assert success["episodes"] == 90
PY
