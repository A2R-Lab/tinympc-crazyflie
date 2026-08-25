#!/bin/bash
# One development-only smoke; not an acceptance matrix.
#SBATCH --job-name=joint-gate-smoke
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=00:45:00
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/gate_joint/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/gate_joint/slurm_logs/%x-%j.err
set -euo pipefail
repo=/home/cchen/tinympc-crazyflie; image=/home/cchen/containers/tinympc-crazysim-local.sif
out="${1:?pass fresh absolute smoke output}"; bundle="${2:?pass candidate bundle.json}"; parent="$(dirname "$out")"
mkdir -p "$parent" "$repo/tools/crazysim_mujoco/gate_joint/slurm_logs"
trap 'rc=$?; target="$out/exit_status.txt"; [[ -d "$out" ]] || target="$out.exit_status.txt"; printf "%s\n" "$rc" > "$target"' EXIT
cd "$repo"; { git rev-parse HEAD; git status --short; hostname; command -v apptainer; } > "$out.submit_provenance.txt"
bash tools/crazysim_mujoco/run.sh --trajectory straight_9m --course gate_obstacle_easy_transition --duration 30 --launch-time 1 --launch-prespin 1 --actuator-lti 1 --rate-cascade 0 --progress-speed-mps .5 --progress-entry-acceleration-mps2 1.0 --progress-terminal-deceleration-mps2 1.5 --progress-reward-weight .40 --realtime-factor .2 --firmware-time-factor .1347944 --no-flowdeck --stop-on-contact --vision-latency-frames 1 --vision-model "$(dirname "$bundle")" --vision-adapter joint_gate_rl --random-seed 3199 --out "$out" --container-engine apptainer --apptainer-image "$image"
