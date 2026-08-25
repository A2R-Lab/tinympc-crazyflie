#!/bin/bash
# One fresh calibrated oval diagnostic after softening the ellipse and moving
# an offset gate onto its minimum-curvature segment.
#SBATCH --job-name=oval-smooth-gate
#SBATCH --partition=debug
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=00:30:00
#SBATCH --nodelist=a2r-tiger
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.err
set -euo pipefail

repo=/home/cchen/tinympc-crazyflie
image=/home/cchen/containers/tinympc-crazysim-local.sif
run_root="${1:?pass a fresh absolute output root}"
bundle="${2:?pass candidate bundle directory or bundle.json}"
seed="${3:?pass one fresh integer seed}"

[[ "$(hostname -s)" == a2r-tiger ]] || {
  echo "oval smoke timing profile is calibrated only for a2r-tiger" >&2
  exit 2
}
[[ "$seed" =~ ^[0-9]+$ ]] || {
  echo "seed must be a nonnegative integer" >&2
  exit 2
}
if [[ -f "$bundle" && "$(basename "$bundle")" == bundle.json ]]; then
  bundle="$(dirname "$bundle")"
fi
[[ -f "$bundle/bundle.json" ]] || {
  echo "joint policy bundle is missing bundle.json: $bundle" >&2
  exit 2
}
[[ ! -e "$run_root" ]] || {
  echo "refusing existing output root: $run_root" >&2
  exit 2
}

run_dir="$run_root/oval_seed${seed}"
mkdir -p "$run_root" "$repo/tools/crazysim_mujoco/heldout_rooms/slurm_logs"
trap 'rc=$?; printf "%s\n" "$rc" > "$run_root/exit_status.txt"' EXIT
cd "$repo"
{
  date --iso-8601=seconds
  git rev-parse HEAD
  git status --short
  hostname
  command -v apptainer
  sha256sum \
    apps/controller_tinympc_eigen/src/trajectories/50hz/traj_oval_50hz.h \
    tools/crazysim_mujoco/courses/heldout_room_oval.json \
    tools/crazysim_mujoco/scenes/vision_heldout_room_oval.xml \
    "$bundle/bundle.json"
  printf 'seed=%s\n' "$seed"
} > "$run_root/submit_provenance.txt"

tools/crazysim_mujoco/run.sh \
  --trajectory oval \
  --course heldout_room_oval \
  --duration 30 \
  --launch-time 1 \
  --launch-prespin 1 \
  --actuator-lti 1 \
  --rate-cascade 0 \
  --progress-speed-mps 0.5 \
  --progress-entry-acceleration-mps2 1.0 \
  --progress-terminal-deceleration-mps2 1.5 \
  --progress-reward-weight 0.40 \
  --realtime-factor 0.2 \
  --firmware-time-factor 0.1047352488 \
  --no-flowdeck \
  --stop-on-contact \
  --vision-latency-frames 1 \
  --vision-model "$bundle" \
  --vision-adapter joint_gate_rl \
  --random-seed "$seed" \
  --video \
  --video-speed 1 \
  --out "$run_dir" \
  --container-engine apptainer \
  --apptainer-image "$image"
