#!/bin/bash
# Render one already-completed CrazySim telemetry file through the shared
# Apptainer image. This does not rerun or alter the simulation.
#SBATCH --job-name=render-flight
#SBATCH --partition=debug
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:10:00
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/heldout_rooms/slurm_logs/%x-%j.err
set -euo pipefail

repo=/home/cchen/tinympc-crazyflie
image=/home/cchen/containers/tinympc-crazysim-local.sif
run_dir="${1:?pass an absolute completed run directory}"
course="${2:?pass a repository-relative course manifest}"
[[ "$run_dir" == "$repo"/* ]] || {
  echo "run directory must be inside the repository" >&2
  exit 2
}
[[ -f "$run_dir/state.csv" && -f "$repo/$course" ]] || {
  echo "state.csv or course manifest is missing" >&2
  exit 2
}
[[ ! -e "$run_dir/flight.mp4" ]] || {
  echo "refusing to overwrite existing flight.mp4" >&2
  exit 2
}

status="$run_dir/render_exit_status.txt"
trap 'rc=$?; printf "%s\n" "$rc" > "$status"' EXIT
cd "$repo"
{
  date --iso-8601=seconds
  hostname
  git rev-parse HEAD
  git status --short
  command -v apptainer
  sha256sum "$run_dir/state.csv" "$repo/$course"
} > "$run_dir/render_provenance.txt"

relative_run="${run_dir#"$repo/"}"
apptainer exec --bind "$repo:/workspace" --pwd /workspace "$image" \
  python3 tools/crazysim_mujoco/render_flight_video.py \
    --csv "/workspace/$relative_run/state.csv" \
    --out "/workspace/$relative_run/flight.mp4" \
    --course "/workspace/$course" \
    --fps 24 --playback-speed 1 --launch-time 1
