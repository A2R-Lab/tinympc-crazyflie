#!/usr/bin/env bash
#SBATCH --job-name=joint_gate_compact_v3_recovery
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --output=/home/cchen/isaacsim-workspace/workspace/logs/%x-%j.out
#SBATCH --error=/home/cchen/isaacsim-workspace/workspace/logs/%x-%j.err

set -euo pipefail
: "${RAW_DATASET:?Set RAW_DATASET to a complete raw rollout directory}"
REPO=/home/cchen/isaacsim-workspace
PYTHON=/home/cchen/isaacsim-env/bin/python
RUN_DIR="$REPO/workspace/joint_gate_obstacle_runs/${SLURM_JOB_ID}"
mkdir -p "$RUN_DIR" "$REPO/workspace/logs"
on_exit() {
  status=$?
  printf '%s\n' "$status" > "$RUN_DIR/exit_status.txt"
  date --iso-8601=seconds >> "$RUN_DIR/finished_at.txt"
  exit "$status"
}
trap on_exit EXIT
cd "$REPO"
git rev-parse HEAD > "$RUN_DIR/git_commit.txt"
git status --short > "$RUN_DIR/git_status.txt"
printf 'job_id=%s node=%s raw_dataset=%s recovery=compact_only_no_rerender\n' "$SLURM_JOB_ID" "$(hostname)" "$RAW_DATASET" > "$RUN_DIR/metadata.txt"
test -d "$RAW_DATASET/frames" && test -s "$RAW_DATASET/transitions.jsonl"
test ! -e "$RAW_DATASET/_SUCCESS"
for artifact in manifest.json dataset_manifest.json train.npz validation.npz test.npz; do
  test ! -e "$RAW_DATASET/$artifact"
done
sha256sum "$RAW_DATASET/transitions.jsonl" > "$RUN_DIR/raw_transition_sha256_before.txt"
PYTHONPATH=. "$PYTHON" user_workflows/export_joint_gate_obstacle_rollouts.py \
  --output-dir "$RAW_DATASET" --compact-only
test -f "$RAW_DATASET/_SUCCESS" && test -f "$RAW_DATASET/manifest.json" \
  && test -f "$RAW_DATASET/train.npz" && test -f "$RAW_DATASET/validation.npz" \
  && test -f "$RAW_DATASET/test.npz"
sha256sum "$RAW_DATASET/manifest.json" "$RAW_DATASET/train.npz" \
  "$RAW_DATASET/validation.npz" "$RAW_DATASET/test.npz" > "$RUN_DIR/artifact_sha256.txt"
sha256sum "$RAW_DATASET/transitions.jsonl" > "$RUN_DIR/raw_transition_sha256_after.txt"
cmp "$RUN_DIR/raw_transition_sha256_before.txt" "$RUN_DIR/raw_transition_sha256_after.txt"
