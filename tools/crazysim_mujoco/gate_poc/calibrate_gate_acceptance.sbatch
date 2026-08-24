#!/bin/bash
# Bounded, reproducible confidence-calibration job. Submit from repository root.
#SBATCH --job-name=gate_poc_cal
#SBATCH --partition=debug
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:10:00
#SBATCH --output=apps/controller_tinympc_eigen/sim_runs/crazysim/gate_obstacle_poc/training/logs/%x-%j.out
#SBATCH --error=apps/controller_tinympc_eigen/sim_runs/crazysim/gate_obstacle_poc/training/logs/%x-%j.err

set -euo pipefail

repo_root="${SLURM_SUBMIT_DIR}"
base_dir="$repo_root/apps/controller_tinympc_eigen/sim_runs/crazysim/gate_obstacle_poc/training"
run_dir="$base_dir/job_${SLURM_JOB_ID}"
mkdir -p "$run_dir"
cp "$repo_root/tools/crazysim_mujoco/gate_poc/calibrate_gate_acceptance.sbatch" \
  "$run_dir/submission.sbatch"
on_exit() {
  status=$?
  printf '%s\n' "$status" > "$run_dir/exit_status.txt"
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >> "$run_dir/metadata.txt"
}
trap on_exit EXIT

{
  echo "started_at=$(date --iso-8601=seconds)"
  echo "job_id=$SLURM_JOB_ID node=$(hostname)"
  echo "submit_dir=$SLURM_SUBMIT_DIR"
  git rev-parse HEAD
  git status --short
  /home/cchen/isaacsim-env/bin/python --version
} | tee "$run_dir/metadata.txt"

/home/cchen/isaacsim-env/bin/python tools/crazysim_mujoco/gate_poc/calibrate_gate_acceptance.py \
  --features-dir /home/cchen/isaacsim-workspace/workspace/newbeedrone_multi_course_confidence_v2 \
  --output-dir "$run_dir/calibration" \
  --repo-root "$repo_root" \
  --minimum-validation-precision 0.96 | tee "$run_dir/result.json"
