#!/bin/bash
# Submit only after checking live `sinfo` and `squeue -u "$USER"`.
#SBATCH --job-name=heldout-rooms
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=04:00:00
# The 0.2/0.1047352488 CrazySim/FreeRTOS timing pair below was measured on this
# host. Do not silently reuse it on a different node.
#SBATCH --nodelist=a2r-tiger
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.err
set -euo pipefail
repo=/home/cchen/tinympc-crazyflie; image=/home/cchen/containers/tinympc-crazysim-local.sif
[[ "$(hostname -s)" == a2r-tiger ]] || {
  echo "held-out timing profile is calibrated only for a2r-tiger" >&2
  exit 2
}
run_root="${1:?pass a fresh absolute output root}"; bundle="${2:?pass candidate bundle.json}"; seed_start="${3:?pass an explicit fresh seed start}"
[[ "$seed_start" =~ ^[0-9]+$ ]] || { echo "seed start must be a nonnegative integer" >&2; exit 2; }
run_parent="$(dirname "$run_root")"
mkdir -p "$run_parent" "$repo/tools/crazysim_mujoco/heldout_rooms/slurm_logs"
trap 'rc=$?; target="$run_root/exit_status.txt"; [[ -d "$run_root" ]] || target="$run_root.exit_status.txt"; printf "%s\n" "$rc" > "$target"' EXIT
cd "$repo"; { git rev-parse HEAD; git status --short; hostname; command -v apptainer; printf 'seed_start=%s\n' "$seed_start"; } > "$run_root.submit_provenance.txt"
bash tools/crazysim_mujoco/heldout_rooms/run_heldout_rooms_matrix.sh --candidate-bundle "$bundle" --out-root "$run_root" --seed-start "$seed_start" --container-engine apptainer --apptainer-image "$image"
