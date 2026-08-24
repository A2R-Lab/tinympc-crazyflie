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

nvidia-smi
IFS=: read -r -a run_array <<<"$RUN_DIRS"
cd "$REPOSITORY_DIR"
srun "$PYTHON_BIN" -m tools.crazysim_mujoco.rl.train_mbpo \
  --runs "${run_array[@]}" \
  --course "$COURSE_FILE" \
  --config "$CONFIG_FILE" \
  --output "$OUTPUT_DIR"
