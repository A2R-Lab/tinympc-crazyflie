#!/bin/bash
# Submit only after live `sinfo` and `squeue -u "$USER"` inspection.
#SBATCH --job-name=joint-gate-matrix
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=06:00:00
# The 0.2/0.1347944 timing pair in the matrix runner is the accepted tiger
# profile; the POSIX SITL clock is host/workload dependent.
#SBATCH --nodelist=a2r-tiger
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/gate_joint/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/gate_joint/slurm_logs/%x-%j.err
set -euo pipefail
repo=/home/cchen/tinympc-crazyflie; image=/home/cchen/containers/tinympc-crazysim-local.sif
[[ "$(hostname -s)" == a2r-tiger ]] || {
  echo "transition timing profile is calibrated only for a2r-tiger" >&2
  exit 2
}
run_root="${1:?pass fresh absolute matrix root}"; bundle="${2:?pass candidate bundle.json}"; seed_start="${3:?pass an explicit fresh seed start}"; parent="$(dirname "$run_root")"
[[ "$seed_start" =~ ^[0-9]+$ ]] || { echo "seed start must be a nonnegative integer" >&2; exit 2; }
mkdir -p "$parent" "$repo/tools/crazysim_mujoco/gate_joint/slurm_logs"
trap 'rc=$?; target="$run_root/exit_status.txt"; [[ -d "$run_root" ]] || target="$run_root.exit_status.txt"; printf "%s\n" "$rc" > "$target"' EXIT
cd "$repo"; { git rev-parse HEAD; git status --short; hostname; command -v apptainer; printf 'seed_start=%s\n' "$seed_start"; } > "$run_root.submit_provenance.txt"
bash tools/crazysim_mujoco/gate_joint/run_joint_gate_obstacle_matrix.sh --candidate-bundle "$bundle" --out-root "$run_root" --seed-start "$seed_start" --container-engine apptainer --apptainer-image "$image"
