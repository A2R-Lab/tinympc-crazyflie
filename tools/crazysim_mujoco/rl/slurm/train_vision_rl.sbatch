#!/bin/bash
#SBATCH --job-name=tinympc-vision-rl
#SBATCH --partition=debug
#SBATCH --gres=gpu:rtx5090:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=06:00:00
#SBATCH --output=slurm-%j.out

set -euo pipefail

: "${RUN_DIRS:?Set RUN_DIRS to colon-separated retained CrazySim run directories}"
: "${OUTPUT_DIR:?Set OUTPUT_DIR to a fresh training artifact directory}"

REPOSITORY_DIR="${REPOSITORY_DIR:-/home/cchen/tinympc-crazyflie}"
COURSE_FILE="${COURSE_FILE:-$REPOSITORY_DIR/tools/crazysim_mujoco/courses/dronet_u.json}"
CONFIG_FILE="${CONFIG_FILE:-$REPOSITORY_DIR/tools/crazysim_mujoco/rl/configs/dronet_u_poc.json}"
PYTHON_BIN="${PYTHON_BIN:-/home/cchen/isaacsim-env/bin/python}"

mkdir -p "$OUTPUT_DIR"
on_exit() {
  status=$?
  printf '%s\n' "$status" >"$OUTPUT_DIR/exit_status.txt"
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >>"$OUTPUT_DIR/scheduler_metadata.txt"
}
trap on_exit EXIT

{
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'job_id=%s\n' "$SLURM_JOB_ID"
  printf 'node=%s\n' "$(hostname)"
  printf 'cuda_visible_devices=%s\n' "${CUDA_VISIBLE_DEVICES:-unset}"
  printf 'repository_commit=%s\n' "$(git -C "$REPOSITORY_DIR" rev-parse HEAD)"
  printf 'repository_dirty=%s\n' "$(test -n "$(git -C "$REPOSITORY_DIR" status --short)" && echo true || echo false)"
  printf 'python=%s\n' "$($PYTHON_BIN --version 2>&1)"
} >"$OUTPUT_DIR/scheduler_metadata.txt"

nvidia-smi | tee "$OUTPUT_DIR/nvidia-smi.txt"
IFS=: read -r -a run_array <<<"$RUN_DIRS"
expert_args=()
if [[ -n "${EXPERT_DATASET:-}" ]]; then
  expert_args+=(--expert-dataset "$EXPERT_DATASET")
fi
if [[ -n "${VALIDATION_DATASET:-}" ]]; then
  expert_args+=(--validation-dataset "$VALIDATION_DATASET")
fi
cd "$REPOSITORY_DIR"
srun "$PYTHON_BIN" -m tools.crazysim_mujoco.rl.train_mbpo \
  --runs "${run_array[@]}" \
  --course "$COURSE_FILE" \
  --config "$CONFIG_FILE" \
  "${expert_args[@]}" \
  --output "$OUTPUT_DIR"
