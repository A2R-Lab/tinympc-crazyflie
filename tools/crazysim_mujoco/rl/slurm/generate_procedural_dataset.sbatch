#!/bin/bash
#SBATCH --job-name=tinympc-rl-data
#SBATCH --partition=debug
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --output=slurm-%j.out

set -euo pipefail

: "${DATASET_DIR:?Set DATASET_DIR to a fresh shared-home output directory}"

REPOSITORY_DIR="${REPOSITORY_DIR:-/home/cchen/tinympc-crazyflie}"
PYTHON_BIN="${PYTHON_BIN:-/home/cchen/isaacsim-env/bin/python3.12}"
DATASET_SEED="${DATASET_SEED:-20260824}"
mkdir -p "$DATASET_DIR"

on_exit() {
  status=$?
  printf '%s\n' "$status" >"$DATASET_DIR/exit_status.txt"
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >>"$DATASET_DIR/scheduler_metadata.txt"
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
} >"$DATASET_DIR/scheduler_metadata.txt"
nvidia-smi >"$DATASET_DIR/nvidia-smi.txt"

cd "$REPOSITORY_DIR"
export MUJOCO_GL=egl
"$PYTHON_BIN" -m tools.crazysim_mujoco.rl.generate_procedural_avoidance_dataset \
  --output "$DATASET_DIR" \
  --seed "$DATASET_SEED" \
  --episodes 60 \
  --samples-per-episode 720 \
  --validation-fraction 0.20 \
  --layout-variants 12 \
  --appearance-variants 6 \
  --noise-variants 3
"$PYTHON_BIN" -m tools.crazysim_mujoco.rl.validate_procedural_dataset \
  --manifest "$DATASET_DIR/manifest.json" \
  --out "$DATASET_DIR/validation_report.json"
