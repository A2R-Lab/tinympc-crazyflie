#!/bin/bash
# Verify partition/GPU names with sinfo before submission; do not submit this file unchanged.
#SBATCH --job-name=joint-gate-obstacle
#SBATCH --partition=debug
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --gres=gpu:1
#SBATCH --time=04:00:00
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/models/vision_rl_gate_joint/slurm_logs/joint-gate-obstacle-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/models/vision_rl_gate_joint/slurm_logs/joint-gate-obstacle-%j.err
set -euo pipefail
: "${PYTHON_BIN:?Set PYTHON_BIN to an existing shared-home Python environment}"
: "${EXTERNAL_DATASET:?Set EXTERNAL_DATASET to one physical Isaac/MuJoCo exporter output directory}"
: "${RUNTIME_TRACK_RETENTION:?Set RUNTIME_TRACK_RETENTION to the validated TRACK-only runtime NPZ}"
: "${RUNTIME_OBSTACLE_RETENTION:?Set RUNTIME_OBSTACLE_RETENTION to the validated frozen-teacher obstacle runtime NPZ}"
repo=/home/cchen/tinympc-crazyflie
root="$repo/tools/crazysim_mujoco/models/vision_rl_gate_joint"
retention="$repo/apps/controller_tinympc_eigen/sim_runs/crazysim/hybrid_rl_balanced_v2/dataset_seed20260824_v2/train.npz"
run="$root/runs/${SLURM_JOB_ID}"
mkdir -p "$run"
trap 's=$?; printf "%s\n" "$s" > "$run/exit_status.txt"; date --iso-8601=seconds >> "$run/finished_at.txt"; exit "$s"' EXIT
cd "$repo"
{ date --iso-8601=seconds; hostname; echo "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-unset}"; git -C "$repo" rev-parse HEAD; git -C "$repo" status --short; "$PYTHON_BIN" --version; } > "$run/metadata.txt"
sinfo -o '%P %a %l %D %t %G' > "$run/sinfo.txt"
squeue -u "$USER" -o '%.18i %.9P %.18j %.2t %.10M %R' > "$run/squeue.txt"
nvidia-smi > "$run/nvidia-smi.txt"
for artifact in _SUCCESS train.npz validation.npz test.npz manifest.json; do
  if [[ ! -f "$EXTERNAL_DATASET/$artifact" ]]; then
    echo "external dataset is incomplete: $EXTERNAL_DATASET/$artifact" >&2
    exit 1
  fi
done
if [[ ! -f "$RUNTIME_TRACK_RETENTION" ]]; then
  echo "runtime TRACK retention NPZ not found: $RUNTIME_TRACK_RETENTION" >&2
  exit 1
fi
if [[ ! -f "$RUNTIME_OBSTACLE_RETENTION" ]]; then
  echo "runtime obstacle retention NPZ not found: $RUNTIME_OBSTACLE_RETENTION" >&2
  exit 1
fi
sha256sum "$EXTERNAL_DATASET/_SUCCESS" "$EXTERNAL_DATASET/train.npz" \
  "$EXTERNAL_DATASET/validation.npz" "$EXTERNAL_DATASET/test.npz" \
  "$EXTERNAL_DATASET/manifest.json" "$retention" \
  "$RUNTIME_TRACK_RETENTION" "$RUNTIME_OBSTACLE_RETENTION" \
  > "$run/input_sha256.txt"
"$PYTHON_BIN" -m tools.crazysim_mujoco.models.vision_rl_gate_joint.train --config "$root/config.json" --dataset "$root" --external-train "$EXTERNAL_DATASET/train.npz" --external-validation "$EXTERNAL_DATASET/validation.npz" --external-test "$EXTERNAL_DATASET/test.npz" --external-manifest "$EXTERNAL_DATASET/manifest.json" --retention-dataset "$retention" --runtime-track-retention "$RUNTIME_TRACK_RETENTION" --runtime-obstacle-retention "$RUNTIME_OBSTACLE_RETENTION" --output "$run/model" --device cuda
