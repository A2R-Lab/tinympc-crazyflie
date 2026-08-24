#!/bin/bash
#SBATCH --job-name=tinympc-u-expert
#SBATCH --partition=debug
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=00:15:00
#SBATCH --output=slurm-%j.out

set -euo pipefail

: "${RUN_DIRS:?Set RUN_DIRS to the colon-separated seed 11/22/33 runs}"
: "${OUTPUT_DIR:?Set OUTPUT_DIR to a fresh shared-home directory}"

REPOSITORY_DIR="${REPOSITORY_DIR:-/home/cchen/tinympc-crazyflie}"
PYTHON_BIN="${PYTHON_BIN:-/home/cchen/isaacsim-env/bin/python3.12}"
COURSE_FILE="${COURSE_FILE:-$REPOSITORY_DIR/tools/crazysim_mujoco/courses/dronet_u.json}"
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
  printf 'repository_commit=%s\n' "$(git -C "$REPOSITORY_DIR" rev-parse HEAD)"
  printf 'repository_dirty=%s\n' "$(test -n "$(git -C "$REPOSITORY_DIR" status --short)" && echo true || echo false)"
  printf 'python=%s\n' "$($PYTHON_BIN --version 2>&1)"
} >"$OUTPUT_DIR/scheduler_metadata.txt"

IFS=: read -r -a run_array <<<"$RUN_DIRS"
cd "$REPOSITORY_DIR"
"$PYTHON_BIN" -m tools.crazysim_mujoco.rl.generate_oracle_expert \
  --runs "${run_array[@]}" \
  --course "$COURSE_FILE" \
  --output "$OUTPUT_DIR/u_course_oracle_expert.npz" \
  --manifest "$OUTPUT_DIR/manifest.json"
