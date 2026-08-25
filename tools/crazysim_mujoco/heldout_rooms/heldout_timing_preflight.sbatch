#!/bin/bash
# Same-host, same-rendering-workload calibration gate for the held-out matrix.
#SBATCH --job-name=heldout-preflight
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
out=${1:?pass a fresh absolute output directory}
bundle=${2:?pass candidate bundle.json or its containing directory}
factor=${3:?pass firmware time factor}
[[ "$(hostname -s)" == a2r-tiger ]] || {
  echo "held-out timing preflight is calibrated only for a2r-tiger" >&2
  exit 2
}
[[ ! -e "$out" ]] || { echo "refusing existing preflight output: $out" >&2; exit 2; }
[[ ! -f "$bundle" ]] || bundle=$(dirname "$bundle")
[[ -f "$bundle/bundle.json" ]] || { echo "candidate bundle is missing bundle.json" >&2; exit 2; }
cd "$repo"
set +e
tools/crazysim_mujoco/run.sh \
  --trajectory straight_9m --course heldout_room_straight --duration 30 \
  --launch-time 1 --launch-prespin 1 --actuator-lti 1 --rate-cascade 0 \
  --progress-speed-mps 0.5 --progress-entry-acceleration-mps2 1.0 \
  --progress-terminal-deceleration-mps2 1.5 --progress-reward-weight 0.40 \
  --realtime-factor 0.2 --firmware-time-factor "$factor" --no-flowdeck \
  --stop-on-contact --vision-latency-frames 1 --vision-model "$bundle" \
  --vision-adapter joint_gate_rl --random-seed 4101 --out "$out" \
  --container-engine apptainer --apptainer-image "$image"
runner_rc=$?
set -e
python3 - "$out" "$factor" "$runner_rc" <<'PY'
import json, math, sys
from pathlib import Path
out, factor, runner_rc = Path(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
summary_path = out / "summary.json"
if not summary_path.is_file():
    raise SystemExit("preflight did not produce summary.json")
summary = json.loads(summary_path.read_text())
actual = summary.get("launch_time_s")
valid = isinstance(actual, (int, float)) and math.isfinite(actual)
error = abs(float(actual) - 1.0) if valid else None
report = {
    "format": "heldout-room-timing-preflight-v1",
    "host": "a2r-tiger",
    "realtime_factor": 0.2,
    "firmware_time_factor": factor,
    "requested_launch_s": 1.0,
    "actual_launch_s": actual,
    "absolute_error_s": error,
    "required_tolerance_s": 0.10,
    "runner_exit_code": runner_rc,
    "passed": error is not None and error <= 0.10 + 1e-12,
}
(out / "timing_preflight.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
if not report["passed"]:
    raise SystemExit("held-out timing preflight failed")
PY
