#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
IMAGE="tinympc-crazysim:local"
TRAJECTORY="backflip_360"
STORED_LTV=1
ACTUATOR_LTI=1
REFERENCE_MODE="waypoint"
LEVEL_COST_MODE="baseline"
PROGRESS_SAMPLE_LIMIT=0
PROGRESS_SPEED_MPS=""
PROGRESS_REFERENCE_LIMITS=""
DURATION=10
LAUNCH_TIME=4
SPAWN_Z=1.5
MODEL="cf21B_500"
MASS="stock"
PWM_THRUST_FULL="0.312852"
LAUNCH_PRESPIN=0
RANDOM_SEED=1
INERTIA_SCALE="1.0"
MOTOR_TAU_SCALE="1.0"
THRUST_SCALE="1.0"
REALTIME_FACTOR="0.1"
FIRMWARE_TIME_FACTOR="0.07"
OUT=""
OVERWRITE=0
STOP_ON_CONTACT=0
VISION_MODEL=""
VISION_ADAPTER="auto"
VISION_SCENE="obstacle"
VISION_LATENCY_FRAMES=1
COURSE=""
COURSE_ACRO="none"
RENDER_VIDEO=0
VIDEO_FPS=24
VIDEO_SPEED=2
EXTRA_SIM_ARGS=(__none__)

usage() {
  cat <<'EOF'
Usage: tools/crazysim_mujoco/run.sh [options]
  --trajectory NAME       straight, straight_long, figure8, oval, circle, chicane,
                          hairpin_180, backflip_360,
                          front_flip_360, roll_flip_360, or barrel_roll_forward_360
  --stored-ltv 0|1        Use horizon-wise stored matrices (default: 1)
  --actuator-lti 0|1      Include fixed motor-lag states in level flight (default: 1)
  --reference-mode MODE   waypoint, trajectory, or progress (default: waypoint)
  --level-cost-mode MODE  baseline, yaw_angle_4x, yaw_rate_4x, or
                          yaw_diff_r_quarter (default: baseline)
  --progress-sample-limit N  Progress route sample count; 0 uses full route (default: 0)
  --progress-speed-mps MPS  Use one positive constant speed in progress mode
                            (default: curvature schedule 0.05..0.15 m/s)
  --progress-reference-limits MODE  default or uncapped; progress mode only
                                    (default: default)
  --duration SECONDS      Simulation duration (default: 10)
  --launch-time SECONDS   Airborne handoff/controller start (default: 4)
  --spawn-z METERS        Handoff altitude (default: 1.5)
  --model NAME            CrazySim model (default: cf21B_500)
  --mass KG|stock         Override vehicle mass (default: stock model value)
  --pwm-thrust-full N     Firmware PWM calibration thrust (default: 0.312852)
  --launch-prespin 0|1    Skip or apply RPM pre-spin at handoff (default: 0)
  --random-seed INTEGER   Seed noise and turbulence (default: 1)
  --inertia-scale SCALE   Scale plant diagonal inertia (default: 1.0)
  --motor-tau-scale SCALE Scale plant motor lag (default: 1.0)
  --thrust-scale SCALE    Scale realized plant thrust (default: 1.0)
  --realtime-factor RATE  Simulator/wall rate cap (default: 0.1)
  --firmware-time-factor RATE  Measured simulator/wall rate (default: 0.07)
  --out DIRECTORY         Output directory
  --vision-model PATH     Model path, or bundled dronet, sequential, or stdc
  --vision-adapter NAME   auto, espnet, sequential, stdc, or dronet (default: auto)
  --vision-scene NAME     obstacle, gate, or none (default: obstacle)
  --vision-latency-frames N  Fixed camera-frame delivery delay (default: 1)
  --course NAME           Select a manifest-backed obstacle/gate course
  --video                 Render flight.mp4 after simulation (does not affect timing)
  --video-fps FPS         Encoded video frame rate (default: 24)
  --video-speed FACTOR    Replay speed multiplier (default: 2)
  --sensor-noise          Enable CrazySim IMU/barometer noise
  --flowdeck              Use simulated Flow deck instead of pose
  --ground-effect         Enable ground effect
  --wind-speed MPS        Add constant wind
  --turbulence LEVEL      none, light, moderate, or severe
  --overwrite             Replace an existing output directory
  --stop-on-contact       End simulation after the first logged contact
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --trajectory) TRAJECTORY="$2"; shift 2 ;;
    --stored-ltv) STORED_LTV="$2"; shift 2 ;;
    --actuator-lti) ACTUATOR_LTI="$2"; shift 2 ;;
    --reference-mode) REFERENCE_MODE="$2"; shift 2 ;;
    --level-cost-mode) LEVEL_COST_MODE="$2"; shift 2 ;;
    --progress-sample-limit) PROGRESS_SAMPLE_LIMIT="$2"; shift 2 ;;
    --progress-speed-mps) PROGRESS_SPEED_MPS="$2"; shift 2 ;;
    --progress-reference-limits) PROGRESS_REFERENCE_LIMITS="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --launch-time) LAUNCH_TIME="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --mass) MASS="$2"; shift 2 ;;
    --pwm-thrust-full) PWM_THRUST_FULL="$2"; shift 2 ;;
    --launch-prespin) LAUNCH_PRESPIN="$2"; shift 2 ;;
    --random-seed) RANDOM_SEED="$2"; shift 2 ;;
    --inertia-scale) INERTIA_SCALE="$2"; shift 2 ;;
    --motor-tau-scale) MOTOR_TAU_SCALE="$2"; shift 2 ;;
    --thrust-scale) THRUST_SCALE="$2"; shift 2 ;;
    --realtime-factor) REALTIME_FACTOR="$2"; shift 2 ;;
    --firmware-time-factor) FIRMWARE_TIME_FACTOR="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --vision-model) VISION_MODEL="$2"; shift 2 ;;
    --vision-adapter) VISION_ADAPTER="$2"; shift 2 ;;
    --vision-scene) VISION_SCENE="$2"; shift 2 ;;
    --vision-latency-frames) VISION_LATENCY_FRAMES="$2"; shift 2 ;;
    --course) COURSE="$2"; shift 2 ;;
    --video) RENDER_VIDEO=1; shift ;;
    --video-fps) VIDEO_FPS="$2"; shift 2 ;;
    --video-speed) VIDEO_SPEED="$2"; shift 2 ;;
    --sensor-noise|--flowdeck|--ground-effect)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1"); shift ;;
    --wind-speed|--turbulence)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1" "$2"); shift 2 ;;
    --overwrite) OVERWRITE=1; shift ;;
    --stop-on-contact) STOP_ON_CONTACT=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$TRAJECTORY" in
  straight|straight_long|canonical_corridor|canonical_circle|figure8|oval|canonical_figure8|circle|chicane|canonical_chicane|hairpin_180|canonical_hairpin|backflip_360|front_flip_360|roll_flip_360|barrel_roll_forward_360) ;;
  *) echo "Unsupported trajectory: $TRAJECTORY" >&2; exit 2 ;;
esac
if [[ "$STORED_LTV" != 0 && "$STORED_LTV" != 1 ]]; then
  echo "--stored-ltv must be 0 or 1" >&2
  exit 2
fi
if [[ "$ACTUATOR_LTI" != 0 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--actuator-lti must be 0 or 1" >&2
  exit 2
fi
case "$REFERENCE_MODE" in
  waypoint|trajectory|progress) ;;
  *) echo "--reference-mode must be waypoint, trajectory, or progress" >&2; exit 2 ;;
esac
case "$LEVEL_COST_MODE" in
  baseline|yaw_angle_4x|yaw_rate_4x|yaw_diff_r_quarter) ;;
  *) echo "--level-cost-mode must be baseline, yaw_angle_4x, yaw_rate_4x, or yaw_diff_r_quarter" >&2; exit 2 ;;
esac
[[ "$PROGRESS_SAMPLE_LIMIT" =~ ^[0-9]+$ ]] || {
  echo "--progress-sample-limit must be a nonnegative integer" >&2; exit 2;
}
if [[ "$PROGRESS_SAMPLE_LIMIT" == 1 ]]; then
  echo "--progress-sample-limit must be 0 (full route) or at least 2" >&2
  exit 2
fi
if [[ "$PROGRESS_SAMPLE_LIMIT" != 0 && "$REFERENCE_MODE" != progress ]]; then
  echo "nonzero --progress-sample-limit requires --reference-mode progress" >&2
  exit 2
fi
if [[ -n "$PROGRESS_SPEED_MPS" ]]; then
  if ! python3 - "$PROGRESS_SPEED_MPS" <<'PY'
import math
import sys
try:
    speed = float(sys.argv[1])
except ValueError:
    raise SystemExit("--progress-speed-mps must be a positive finite number")
if not math.isfinite(speed) or speed <= 0.0:
    raise SystemExit("--progress-speed-mps must be a positive finite number")
PY
  then
    exit 2
  fi
  if [[ "$REFERENCE_MODE" != progress ]]; then
    echo "--progress-speed-mps requires --reference-mode progress" >&2
    exit 2
  fi
fi
if [[ -n "$PROGRESS_REFERENCE_LIMITS" ]]; then
  case "$PROGRESS_REFERENCE_LIMITS" in
    default|uncapped) ;;
    *) echo "--progress-reference-limits must be default or uncapped" >&2; exit 2 ;;
  esac
  if [[ "$REFERENCE_MODE" != progress ]]; then
    echo "--progress-reference-limits requires --reference-mode progress" >&2
    exit 2
  fi
fi
PROGRESS_REFERENCE_LIMITS_EFFECTIVE="${PROGRESS_REFERENCE_LIMITS:-default}"
if [[ "$LEVEL_COST_MODE" != baseline && "$ACTUATOR_LTI" != 1 ]]; then
  echo "non-baseline --level-cost-mode requires --actuator-lti 1" >&2
  exit 2
fi
case "$VISION_ADAPTER" in auto|espnet|sequential|stdc|dronet) ;; *) echo "Invalid --vision-adapter" >&2; exit 2 ;; esac
case "$VISION_SCENE" in obstacle|gate|none|straight_offset|straight_slalom|turn_left|canonical_corridor|canonical_circle|canonical_figure8|canonical_chicane|canonical_hairpin) ;; *) echo "Invalid --vision-scene" >&2; exit 2 ;; esac
[[ "$VISION_LATENCY_FRAMES" =~ ^[0-9]+$ ]] || { echo "--vision-latency-frames must be a nonnegative integer" >&2; exit 2; }
if [[ "$RENDER_VIDEO" == 1 ]]; then
  python3 - "$VIDEO_FPS" "$VIDEO_SPEED" <<'PY'
import sys
if any(float(value) <= 0.0 for value in sys.argv[1:]):
    raise SystemExit("--video-fps and --video-speed must be positive")
PY
fi
COURSE_MANIFEST=""
if [[ -n "$COURSE" ]]; then
  COURSE_MANIFEST="$SCRIPT_DIR/courses/${COURSE}.json"
  [[ -f "$COURSE_MANIFEST" ]] || { echo "Unknown course: $COURSE" >&2; exit 2; }
  read -r course_trajectory course_scene course_acro < <(python3 - "$COURSE_MANIFEST" <<'PY'
import json, sys
course = json.load(open(sys.argv[1]))
print(course["trajectory"], course["scene"], course.get("acro_maneuver", "none"))
PY
)
  [[ "$TRAJECTORY" == "$course_trajectory" ]] || {
    echo "Course $COURSE requires --trajectory $course_trajectory" >&2; exit 2;
  }
  VISION_SCENE="${course_scene#vision_}"
  VISION_SCENE="${VISION_SCENE%.xml}"
  COURSE_ACRO="$course_acro"
  if [[ "$COURSE_ACRO" != none ]]; then
    [[ "$STORED_LTV" == 1 ]] || { echo "Course $COURSE requires --stored-ltv 1" >&2; exit 2; }
    [[ "$ACTUATOR_LTI" == 1 ]] || { echo "Course $COURSE requires --actuator-lti 1" >&2; exit 2; }
  fi
fi
VISION_ENABLED=0
VISION_MODEL_CONTAINER=__none__
if [[ -n "$VISION_MODEL" ]]; then
  VISION_ENABLED=1
  case "$VISION_MODEL" in
    dronet)
      VISION_MODEL="$SCRIPT_DIR/models/dronet/dronet.onnx"
      VISION_ADAPTER=dronet ;;
    tinyracer|espnet)
      VISION_MODEL="$SCRIPT_DIR/models/espnet_dronet_gate_v1/espnet_dronet_gate_seed2027_float.onnx"
      VISION_ADAPTER=espnet ;;
    tinyracer-candidate|espnet-candidate)
      VISION_MODEL="$SCRIPT_DIR/models/tinyracer_espnet"
      VISION_ADAPTER=espnet ;;
    sequential|tinyracer-sequential)
      VISION_MODEL="$SCRIPT_DIR/models/tinyracer_sequential/sequential_int.onnx"
      VISION_ADAPTER=sequential ;;
    stdc)
      VISION_MODEL="$SCRIPT_DIR/models/tinyracer_stdc"
      VISION_ADAPTER=stdc ;;
  esac
  VISION_MODEL="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$VISION_MODEL")"
  [[ -e "$VISION_MODEL" ]] || { echo "Vision model not found: $VISION_MODEL" >&2; exit 2; }
  case "$TRAJECTORY" in
    straight|straight_long|canonical_corridor|canonical_circle|figure8|oval|canonical_figure8|circle|chicane|canonical_chicane|hairpin_180|canonical_hairpin) ;;
    *) echo "Vision is intentionally disabled for flip/roll trajectories; use an ordinary approach trajectory." >&2; exit 2 ;;
  esac
  [[ "$STORED_LTV" == 0 || "$COURSE_ACRO" != none ]] || { echo "Vision approach trajectories require --stored-ltv 0 unless a course schedules a stored primitive" >&2; exit 2; }
  if [[ -d "$VISION_MODEL" ]]; then
    VISION_MODEL_MOUNT="$VISION_MODEL"
    VISION_MODEL_CONTAINER=/vision_model
  else
    # Mount the bundle directory, not only the ONNX file: both deployed
    # adapters require the adjacent quantization/decoder manifest.
    VISION_MODEL_MOUNT="$(dirname "$VISION_MODEL")"
    VISION_MODEL_CONTAINER="/vision_bundle/$(basename "$VISION_MODEL")"
  fi
fi
if [[ -n "$COURSE" && "$VISION_ENABLED" != 1 ]]; then
  echo "--course requires --vision-model so the obstacle-avoidance loop is active" >&2
  exit 2
fi
if [[ "$LAUNCH_PRESPIN" != 0 && "$LAUNCH_PRESPIN" != 1 ]]; then
  echo "--launch-prespin must be 0 or 1" >&2
  exit 2
fi

MODE="fixed"
[[ "$STORED_LTV" == 1 ]] && MODE="stored_ltv"
[[ -n "$FIRMWARE_TIME_FACTOR" ]] || FIRMWARE_TIME_FACTOR="$REALTIME_FACTOR"
TICK_US="$(python3 -c 'import sys; print(round(1000.0 / float(sys.argv[1])))' "$FIRMWARE_TIME_FACTOR")"
if [[ -z "$OUT" ]]; then
  OUT="$REPO_DIR/apps/controller_tinympc_eigen/sim_runs/crazysim/${TRAJECTORY}_${REFERENCE_MODE}_${MODE}"
elif [[ "$OUT" != /* ]]; then
  OUT="$REPO_DIR/$OUT"
fi
OUT="$(python3 -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' "$OUT")"
allowed_root="$REPO_DIR/apps/controller_tinympc_eigen/sim_runs/crazysim"
case "$OUT" in
  "$allowed_root"/*) ;;
  *) echo "--out must be a child of $allowed_root" >&2; exit 2 ;;
esac
if [[ -e "$OUT" ]]; then
  if [[ "$OVERWRITE" != 1 ]]; then
    echo "$OUT already exists; pass --overwrite to replace it." >&2
    exit 2
  fi
  rm -rf -- "$OUT"
fi
mkdir -p "$OUT"

repo_commit="$(git -C "$REPO_DIR" rev-parse HEAD)"
repo_dirty=0
[[ -z "$(git -C "$REPO_DIR" status --porcelain)" ]] || repo_dirty=1
python3 - "$OUT/run_config.json" "$REPO_DIR" "$SCRIPT_DIR/run.sh" \
  "$TRAJECTORY" "$STORED_LTV" "$ACTUATOR_LTI" "$REFERENCE_MODE" "$DURATION" \
  "$LAUNCH_TIME" "$SPAWN_Z" "$MODEL" "$MASS" "$PWM_THRUST_FULL" \
  "$LAUNCH_PRESPIN" "$RANDOM_SEED" "$INERTIA_SCALE" "$MOTOR_TAU_SCALE" \
  "$THRUST_SCALE" "$REALTIME_FACTOR" "$FIRMWARE_TIME_FACTOR" \
  "$VISION_ENABLED" "$VISION_MODEL" "$VISION_ADAPTER" "$VISION_SCENE" \
  "$VISION_LATENCY_FRAMES" "$COURSE" "$COURSE_MANIFEST" "$COURSE_ACRO" \
  "$RENDER_VIDEO" "$VIDEO_FPS" "$VIDEO_SPEED" "$repo_commit" "$repo_dirty" \
  "$STOP_ON_CONTACT" "$LEVEL_COST_MODE" "$PROGRESS_SAMPLE_LIMIT" "$PROGRESS_SPEED_MPS" \
  "$PROGRESS_REFERENCE_LIMITS_EFFECTIVE" \
  "${EXTRA_SIM_ARGS[@]}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

(output, repository, runner, trajectory, stored_ltv, actuator_lti, reference_mode, duration,
 launch_time, spawn_z, model, mass, pwm_thrust_full, launch_prespin,
 random_seed, inertia_scale, motor_tau_scale, thrust_scale, realtime_factor,
 firmware_time_factor, vision_enabled, vision_model, vision_adapter,
 vision_scene, vision_latency_frames, course_name, course_manifest, course_acro,
 render_video, video_fps, video_speed, repository_commit, repository_dirty,
 stop_on_contact, level_cost_mode, progress_sample_limit, progress_speed_mps,
 progress_reference_limits,
 *extra) = sys.argv[1:]

def sha256(path):
    path = Path(path)
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

vision_identity = None
if vision_model:
    path = Path(vision_model)
    if path.is_dir():
        manifest = path / "bundle_manifest.json"
        vision_identity = {
            "path": str(path),
            "bundle_manifest_sha256": sha256(manifest),
        }
    else:
        vision_identity = {"path": str(path), "sha256": sha256(path)}

config = {
    "format": "tinympc-crazysim-run-config-v1",
    "repository": str(Path(repository).resolve()),
    "repository_commit": repository_commit,
    "repository_dirty": bool(int(repository_dirty)),
    "runner_sha256": sha256(runner),
    "controller_source_sha256": {
        relative: sha256(Path(repository) / relative)
        for relative in (
            "apps/controller_tinympc_eigen/src/controller_tinympc.cpp",
            "apps/controller_tinympc_eigen/src/tinyracer_racing.c",
            "apps/controller_tinympc_eigen/src/tinyracer_racing.h",
            "apps/controller_tinympc_eigen/src/tinyracer_interface.h",
            "apps/controller_tinympc_eigen/src/tinympc_generated_params.h",
            "apps/controller_tinympc_eigen/src/tinympc_level_actuator_lti.h",
            "apps/controller_tinympc_eigen/src/tinympc_progress_path.h",
            "apps/controller_tinympc_eigen/src/tinympc_waypoint_nav.h",
        )
    },
    "trajectory_header_sha256": sha256(
        Path(repository) / "apps/controller_tinympc_eigen/src/trajectories/50hz"
        / f"traj_{trajectory}_50hz.h"
    ),
    "course_acro_reference_sha256": (
        None if course_acro == "none" else sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/trajectories/50hz"
            / f"course_acro_{course_acro}_50hz.h"
        )
    ),
    "course_acro_ltv_sha256": (
        None if course_acro == "none" else sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/trajectories/50hz/ltv"
            / f"stored_ltv_{course_acro}_50hz.h"
        )
    ),
    "trajectory": trajectory,
    "stored_ltv": bool(int(stored_ltv)),
    "actuator_lti": bool(int(actuator_lti)),
    "reference_mode": reference_mode,
    "level_cost_mode": level_cost_mode,
    "progress_sample_limit": int(progress_sample_limit),
    "progress_speed_mps": None if not progress_speed_mps else float(progress_speed_mps),
    "progress_reference_limits": progress_reference_limits,
    "duration_s": float(duration),
    "launch_time_s": float(launch_time),
    "spawn_z_m": float(spawn_z),
    "model": model,
    "mass": mass if mass == "stock" else float(mass),
    "pwm_thrust_full_n": float(pwm_thrust_full),
    "launch_prespin": bool(int(launch_prespin)),
    "random_seed": int(random_seed),
    "inertia_scale": float(inertia_scale),
    "motor_tau_scale": float(motor_tau_scale),
    "thrust_scale": float(thrust_scale),
    "realtime_factor": float(realtime_factor),
    "firmware_time_factor": float(firmware_time_factor),
    "vision_enabled": bool(int(vision_enabled)),
    "vision_model": vision_identity,
    "vision_adapter": vision_adapter,
    "vision_scene": vision_scene,
    "vision_latency_frames": int(vision_latency_frames),
    "course": course_name or None,
    "course_acro_maneuver": None if course_acro == "none" else course_acro,
    "course_manifest_sha256": sha256(course_manifest) if course_manifest else None,
    "flight_video_requested": bool(int(render_video)),
    "flight_video_fps": float(video_fps),
    "flight_video_playback_speed": float(video_speed),
    "stop_on_contact": bool(int(stop_on_contact)),
    "extra_simulator_arguments": [item for item in extra if item != "__none__"],
}
Path(output).write_text(json.dumps(config, indent=2) + "\n")
PY

"$SCRIPT_DIR/setup.sh"
docker build -q -t "$IMAGE" "$SCRIPT_DIR" >/dev/null

repo_rel_out="${OUT#"$REPO_DIR"/}"

docker_args=(--rm --interactive \
  --volume "$REPO_DIR:/workspace" \
  --workdir /workspace)
if [[ "$VISION_ENABLED" == 1 ]]; then
  if [[ -d "$VISION_MODEL" ]]; then
    docker_args+=(--volume "$VISION_MODEL_MOUNT:$VISION_MODEL_CONTAINER:ro")
  else
    docker_args+=(--volume "$VISION_MODEL_MOUNT:/vision_bundle:ro")
  fi
fi

set +e
docker run "${docker_args[@]}" \
  "$IMAGE" \
  bash -s -- \
    "$TRAJECTORY" "$STORED_LTV" "$DURATION" "$LAUNCH_TIME" "$SPAWN_Z" \
    "$MODEL" "$MASS" "$PWM_THRUST_FULL" "/workspace/$repo_rel_out" \
    "$LAUNCH_PRESPIN" "$RANDOM_SEED" "$INERTIA_SCALE" \
    "$MOTOR_TAU_SCALE" "$THRUST_SCALE" "$TICK_US" \
    "$VISION_ENABLED" "$VISION_MODEL_CONTAINER" "$VISION_ADAPTER" "$VISION_SCENE" \
    "$ACTUATOR_LTI" "$COURSE" "$COURSE_ACRO" "$VISION_LATENCY_FRAMES" "$REFERENCE_MODE" "$STOP_ON_CONTACT" \
    "$LEVEL_COST_MODE" "$PROGRESS_SAMPLE_LIMIT" "${PROGRESS_SPEED_MPS:-0}" \
    "$PROGRESS_REFERENCE_LIMITS_EFFECTIVE" \
    --realtime-factor "$REALTIME_FACTOR" \
    "${EXTRA_SIM_ARGS[@]}" <<'CONTAINER_SCRIPT'
set -euo pipefail
trajectory="$1"; stored_ltv="$2"; duration="$3"; launch_time="$4"
spawn_z="$5"; model="$6"; mass="$7"; pwm_thrust_full="$8"
out="$9"; launch_prespin="${10}"; random_seed="${11}"
inertia_scale="${12}"; motor_tau_scale="${13}"; thrust_scale="${14}"
tick_us="${15}"
vision_enabled="${16}"; vision_model="${17}"; vision_adapter="${18}"
vision_scene="${19}"
actuator_lti="${20}"
course="${21}"
course_acro="${22}"
vision_latency_frames="${23}"
reference_mode="${24}"
stop_on_contact="${25}"
level_cost_mode="${26}"
progress_sample_limit="${27}"
progress_speed_mps="${28}"
progress_reference_limits="${29}"
shift 29
course_build="${course:-none}"

crazysim=/workspace/tools/crazysim_mujoco/.deps/CrazySim
firmware="$crazysim/crazyflie-firmware"
simulator="$firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"
app=/workspace/apps/controller_tinympc_eigen
support=/workspace/tools/crazysim_mujoco/sitl
build="$firmware/sitl_make/build-tinympc-${trajectory}-${reference_mode}-${course_acro}-${stored_ltv}-${actuator_lti}-${level_cost_mode}-${progress_sample_limit}-speed${progress_speed_mps}-${progress_reference_limits}"
ltv_flag=OFF
[[ "$stored_ltv" == 1 ]] && ltv_flag=ON
actuator_lti_flag=OFF
[[ "$actuator_lti" == 1 ]] && actuator_lti_flag=ON
cost_compile_flag=""
case "$level_cost_mode" in
  baseline) ;;
  yaw_angle_4x) cost_compile_flag="-DTINYMPC_LEVEL_COST_YAW_ANGLE_4X=1" ;;
  yaw_rate_4x) cost_compile_flag="-DTINYMPC_LEVEL_COST_YAW_RATE_4X=1" ;;
  yaw_diff_r_quarter) cost_compile_flag="-DTINYMPC_LEVEL_COST_YAW_DIFF_R_QUARTER=1" ;;
  *) echo "Unsupported level cost mode: $level_cost_mode" >&2; exit 2 ;;
esac
reference_limits_compile_flag=""
case "$progress_reference_limits" in
  default) ;;
  uncapped) reference_limits_compile_flag="-DTINYMPC_PROGRESS_REFERENCE_UNCAPPED=1" ;;
  *) echo "Unsupported progress reference limits: $progress_reference_limits" >&2; exit 2 ;;
esac
delay_ms="$(python3 -c 'import sys; print(round(float(sys.argv[1]) * 1000))' "$launch_time")"

case "$trajectory" in
  backflip_360|front_flip_360|roll_flip_360|barrel_roll_forward_360)
    python3 "$app/tools/pybullet_simulation/verify_acrobatic_artifacts.py" \
      --maneuver "$trajectory" </dev/null >"$out/artifact_check.json" ;;
esac
if [[ "$course_acro" != none ]]; then
  python3 "$app/tools/pybullet_simulation/verify_acrobatic_artifacts.py" \
    --maneuver "$course_acro" </dev/null >"$out/course_acro_artifact_check.json"
fi

cmake -S "$firmware/sitl_make" -B "$build" \
  -DTINYMPC_APP_DIR="$app" \
  -DTINYMPC_SITL_SUPPORT_DIR="$support" \
  -DTINYMPC_TRAJECTORY="$trajectory" \
  -DTINYMPC_REFERENCE_MODE="$reference_mode" \
  -DTINYMPC_COURSE="$course_build" \
  -DTINYMPC_COURSE_ACRO="$course_acro" \
  -DTINYMPC_STORED_LTV="$ltv_flag" \
  -DTINYMPC_ACTUATOR_LTI="$actuator_lti_flag" \
  -DCMAKE_CXX_FLAGS="$cost_compile_flag $reference_limits_compile_flag -DTINYMPC_PROGRESS_SAMPLE_LIMIT=$progress_sample_limit -DTINYMPC_PROGRESS_SPEED_MPS=$progress_speed_mps" \
  -DTINYMPC_SITL_START_DELAY_MS="$delay_ms" \
  -DTINYMPC_SITL_TICK_US="$tick_us" \
  >"$out/configure.log" 2>&1
cmake --build "$build" --target cf2 -j2 >"$out/build.log" 2>&1

cleanup() {
  if [[ -n "${contact_monitor_pid:-}" ]]; then
    kill "$contact_monitor_pid" 2>/dev/null || true
    wait "$contact_monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "${vision_pid:-}" ]]; then
    kill "$vision_pid" 2>/dev/null || true
    wait "$vision_pid" 2>/dev/null || true
  fi
  if [[ -n "${firmware_pid:-}" ]]; then
    kill "$firmware_pid" 2>/dev/null || true
    wait "$firmware_pid" 2>/dev/null || true
  fi
  if [[ -n "${simulator_pid:-}" ]]; then
    kill "$simulator_pid" 2>/dev/null || true
    wait "$simulator_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

sim_command=(python3 -u "$simulator" \
  --port 19950 \
  --dt 0.001 \
  --duration "$duration" \
  --launch-time "$launch_time" \
  --launch-on-motor-command \
  --spawn-z "$spawn_z" \
  --model-type "$model" \
  --pwm-thrust-full "$pwm_thrust_full" \
  --random-seed "$random_seed" \
  --inertia-scale "$inertia_scale" \
  --motor-tau-scale "$motor_tau_scale" \
  --thrust-scale "$thrust_scale" \
  --state-log "$out/state.csv")
if [[ "$vision_enabled" == 1 ]]; then
  scene=/workspace/tools/crazysim_mujoco/scenes/vision_${vision_scene}.xml
  camera_width=160; camera_height=120
  camera_fovy=""
  if [[ "$vision_adapter" == dronet ]]; then
    # Match the AI-deck Himax path: capture QVGA-ish 324x244, then let the
    # bridge take the deployed bottom-centered 200x200 crop.
    camera_width=324; camera_height=244
  elif [[ "$vision_adapter" == espnet ]]; then
    # The August 19 two-frame release consumes the full HM01B0 image; unlike
    # the previous deployment it must not receive the 160x120 center crop.
    camera_width=160; camera_height=160
    camera_fovy=47.168554
  fi
  sim_command+=(--camera --cam-width "$camera_width" --cam-height "$camera_height" --cam-fps 20 --cam-port 5200)
  [[ -z "$camera_fovy" ]] || sim_command+=(--cam-fovy "$camera_fovy")
  [[ "$vision_scene" == none ]] || sim_command+=(--scene "$scene")
  python3 -u /workspace/tools/crazysim_mujoco/vision_bridge.py \
    --model "$vision_model" --adapter "$vision_adapter" \
    --camera-port 5200 --camera-fps 20 --firmware-port 19960 --log "$out/vision.csv" \
    --delivery-latency-frames "$vision_latency_frames" \
    --frames-dir "$out/vision_frames" \
    >"$out/vision_bridge.log" 2>&1 &
  vision_pid=$!
  vision_ready=0
  for _ in $(seq 1 100); do
    if grep -q "vision bridge ready" "$out/vision_bridge.log" 2>/dev/null; then
      vision_ready=1
      break
    fi
    if ! kill -0 "$vision_pid" 2>/dev/null; then
      break
    fi
    sleep 0.05
  done
  if [[ "$vision_ready" != 1 ]]; then
    echo "Vision bridge did not start; see $out/vision_bridge.log" >&2
    exit 1
  fi
fi
[[ "$mass" == stock ]] || sim_command+=(--mass "$mass")
[[ "$launch_prespin" == 0 ]] || sim_command+=(--launch-prespin)
for sim_arg in "$@"; do
  [[ "$sim_arg" == __none__ ]] || sim_command+=("$sim_arg")
done

# Bind the simulator socket before booting firmware. Starting firmware first
# let its wall-clock start delay expire while Docker/Python was still loading,
# which changed estimator warm-up and release time from run to run.
"${sim_command[@]}" >"$out/simulator.log" 2>&1 &
simulator_pid=$!
simulator_ready=0
for _ in $(seq 1 100); do
  if grep -q "waiting for firmware" "$out/simulator.log" 2>/dev/null; then
    simulator_ready=1
    break
  fi
  sleep 0.05
done
if [[ "$simulator_ready" != 1 ]]; then
  echo "CrazySim did not become ready for firmware" >&2
  exit 1
fi

stdbuf -oL -eL "$build/cf2" 19950 >"$out/firmware.log" 2>&1 &
firmware_pid=$!
if [[ "$stop_on_contact" == 1 ]]; then
  python3 - "$out/state.csv" "$simulator_pid" "$out/contact_stop.txt" <<'PY' &
import csv
import os
from pathlib import Path
import signal
import sys
import time

state_path = Path(sys.argv[1])
simulator_pid = int(sys.argv[2])
marker = Path(sys.argv[3])
offset = 0
contact_index = None
while True:
    try:
        os.kill(simulator_pid, 0)
    except ProcessLookupError:
        break
    if state_path.is_file():
        with state_path.open() as stream:
            stream.seek(offset)
            for line in stream:
                row = next(csv.reader([line]))
                if contact_index is None:
                    if "contacts" in row:
                        contact_index = row.index("contacts")
                    continue
                if len(row) > contact_index and float(row[contact_index]) > 0.0:
                    marker.write_text(f"first contact row: {line}")
                    os.kill(simulator_pid, signal.SIGTERM)
                    raise SystemExit(0)
            offset = stream.tell()
    time.sleep(0.02)
PY
  contact_monitor_pid=$!
fi
simulator_status=0
wait "$simulator_pid" || simulator_status=$?
simulator_pid=""
if [[ -n "${contact_monitor_pid:-}" ]]; then
  kill "$contact_monitor_pid" 2>/dev/null || true
  wait "$contact_monitor_pid" 2>/dev/null || true
  contact_monitor_pid=""
fi
if [[ "$simulator_status" != 0 && ! -f "$out/contact_stop.txt" ]]; then
  exit "$simulator_status"
fi
cleanup
firmware_pid=""

case "$trajectory" in
  backflip_360|front_flip_360|roll_flip_360|barrel_roll_forward_360)
    reference="$app/sim/trajectories/acrobatics/${trajectory}.csv" ;;
  straight) reference="$app/sim/trajectories/straight.csv" ;;
  straight_long) reference="$app/sim/trajectories/straight_long.csv" ;;
  chicane|hairpin_180) reference="$app/sim/trajectories/racing/${trajectory}.csv" ;;
  *) reference="" ;;
esac
reference_args=()
[[ -z "$reference" ]] || reference_args=(--reference "$reference")
vision_args=()
if [[ "$vision_enabled" != 0 ]]; then
  analysis_scene_kind="$vision_scene"
  [[ -z "$course" ]] || analysis_scene_kind=none
  vision_args=(--vision-csv "$out/vision.csv" --scene-kind "$analysis_scene_kind")
fi
if [[ -n "$course" ]]; then
  vision_args=(--vision-csv "$out/vision.csv" --scene-kind none \
    --course "/workspace/tools/crazysim_mujoco/courses/${course}.json")
fi
python3 /workspace/tools/crazysim_mujoco/analyze_run.py \
  --csv "$out/state.csv" \
  "${reference_args[@]}" \
  "${vision_args[@]}" \
  --launch-time "$launch_time" \
  --out "$out" | tee "$out/summary.txt"
python3 - "$out/summary.json" "$launch_time" <<'PY'
import json
import sys
summary = json.load(open(sys.argv[1]))
expected = float(sys.argv[2])
actual = summary.get("launch_time_s")
if actual is None or abs(float(actual) - expected) > 0.5:
    raise SystemExit(
        f"invalid timing: motor launch {actual} s, expected {expected:.3f} +/- 0.5 s; "
        "calibrate --firmware-time-factor for this host"
    )
if "acrobatics_success" in summary and not summary["acrobatics_success"]:
    raise SystemExit(
        "acrobatic validation failed: require no contact, a full rotation, "
        "upright recovery, and <=0.50 m terminal position error"
    )
PY
CONTAINER_SCRIPT
simulation_status=$?
set -e

if [[ "$RENDER_VIDEO" == 1 && -f "$OUT/state.csv" ]]; then
  video_args=(--csv "$OUT/state.csv" --out "$OUT/flight.mp4" \
    --fps "$VIDEO_FPS" --playback-speed "$VIDEO_SPEED" --launch-time "$LAUNCH_TIME")
  [[ -z "$COURSE_MANIFEST" ]] || video_args+=(--course "$COURSE_MANIFEST")
  python3 "$SCRIPT_DIR/render_flight_video.py" "${video_args[@]}"
fi

echo "Results: $OUT"
exit "$simulation_status"
