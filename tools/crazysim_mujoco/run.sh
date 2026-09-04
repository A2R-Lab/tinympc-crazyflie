#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
IMAGE="tinympc-crazysim:local"
CONTAINER_ENGINE="docker"
APPTAINER_IMAGE=""
TRAJECTORY="circle"
ACTUATOR_LTI=1
RATE_CASCADE=0
BRAKING_CACHE="auto"
DIRECT_PLAN_REPLAY=0
RATE_IDENTIFICATION=0
MPC_DIAG_MODE="off"
REFERENCE_MODE="progress"
LEVEL_COST_MODE="baseline"
PROGRESS_SAMPLE_LIMIT=0
PROGRESS_SPEED_MPS=""
PROGRESS_LAPS=1
PROGRESS_ENTRY_ACCELERATION_MPS2="1.0"
PROGRESS_TERMINAL_DECELERATION_MPS2="6.0"
PROGRESS_REWARD_WEIGHT="0.40"
FLIP_ENABLE=0
FLIP_TRIGGER_S_M="1.0"
FLIP_TRIGGER_WINDOW_M="0.25"
FLIP_DURATION_S="0.8"
FLIP_PITCH_DIRECTION=1
POWER_LOOP_ENABLE=0
POWER_LOOP_TRIGGER_S_M="1.0"
POWER_LOOP_TRIGGER_WINDOW_M="0.25"
POWER_LOOP_RADIUS_M="1.2"
POWER_LOOP_BOTTOM_SPEED_MPS="2.2"
POWER_LOOP_TOP_SPEED_MPS="4.0"
REACTIVE_POWER_LOOP=0
PITCH_THROUGH_BRAKE=0
VERTICAL_ACTIVE_SENSING=0
YAW_SPIN_TEST=0
YAW_SPIN_RATE_RAD_S="2.0"
YAW_SPIN_REVOLUTIONS="4.0"
YAW_SPIN_STRAIGHT_SPEED_MPS="0.0"
YAW_SPIN_STRAIGHT_DURATION_S="2.0"
PROGRESS_REFERENCE_LIMITS_EFFECTIVE="uncapped"
DURATION=10
LAUNCH_TIME=1
SPAWN_Z=1.5
MODEL="cf21B_500"
MASS="stock"
PWM_THRUST_FULL="0.20"
LAUNCH_PRESPIN=0
RANDOM_SEED=1
INERTIA_SCALE="1.0"
MOTOR_TAU_SCALE="1.0"
THRUST_SCALE="1.0"
REALTIME_FACTOR="1.0"
FIRMWARE_TIME_FACTOR="0.8"
REALTIME_FACTOR_EXPLICIT=0
FIRMWARE_TIME_FACTOR_EXPLICIT=0
OUT=""
OVERWRITE=0
STOP_ON_CONTACT=1
VISION_MODEL=""
VISION_ADAPTER="auto"
VISION_SCENE="obstacle"
VISION_SCENE_EXPLICIT=0
VISION_LATENCY_FRAMES=1
CAMERA_FPS=30
VISION_CONTROL_ENABLE_AFTER_S=0
VISION_CONTROL_ENABLE_AT_X_M=""
VISION_PASSIVE=0
CAMERA_ONLY=0
IMAV22_MODE=""
IMAV22_RELOCATION_PERIOD_S="30.0"
IMAV22_RELOCATION_CLEARANCE_M="1.5"
IMAV22_RELOCATION_DURATION_S="2.0"
IMAV22_OBJECT_SET="official"
FLOWDECK_ENABLED=0
COURSE=""
RENDER_VIDEO=0
VIDEO_FPS=24
VIDEO_SPEED=2
EXTRA_SIM_ARGS=(__none__)

usage() {
  cat <<'EOF'
Usage: tools/crazysim_mujoco/run.sh [options]
  --trajectory NAME       straight, straight_long, straight_9m, straight_20m, figure8, oval, circle, imav22_circle, chicane,
                          hairpin_180, dronet_u, or a canonical level-flight route
  --actuator-lti 0|1      Include fixed motor-lag states in level flight (default: 1)
  --rate-cascade 0|1      Use TinyMPC collective/body-rate output with the 500 Hz
                          firmware rate PID and legacy mixer. This bypasses the
                          direct actuator-LTI braking cache (default: 0)
  --braking-cache MODE    auto, 0, or 1. Explicit 0/1 creates matched direct-
                          actuator TinyMPC cache A/B builds (default: auto)
  --direct-plan-replay 0|1  Replay successive direct-MPC horizon inputs while a solve
                          is delayed (requires direct mode, actuator LTI; default: 0)
  --rate-identification   Run bounded hover rate-response excitation (diagnostic only)
  --mpc-diag-mode MODE    off or taskless; enable mmap MPC diagnostics (default: off)
  --level-cost-mode MODE  baseline, yaw_angle_4x, yaw_rate_4x, or
                          yaw_diff_r_quarter (default: baseline)
  --progress-sample-limit N  Progress route sample count; 0 uses full route (default: 0)
  --progress-speed-mps MPS  Use one positive constant speed in progress mode
                            (default: curvature schedule 0.05..0.15 m/s)
  --progress-laps N      Repeat a closed circle for 1..8 laps (default: 1)
  --progress-entry-acceleration-mps2 MPS2  Maximum command-speed ramp rate
                            in progress mode (default: 1.0)
  --progress-terminal-deceleration-mps2 MPS2  Maximum cooldown braking rate
                            in progress mode (default: 6.0)
  --progress-reward-weight LAMBDA  Weight for -lambda*tangent_dot_velocity
                            in progress mode (default: 0.40)
  --flip 0|1              Enable the one-shot pitch-flip primitive (default: 0)
  --flip-trigger-s-m M    Measured course distance that opens the trigger window
                            (default: 1.0)
  --flip-trigger-window-m M  Width of the measured-distance trigger window
                            (default: 0.25)
  --flip-duration-s S     Maneuver phase duration (default: 0.8)
  --flip-pitch-direction -1|1  Signed full-rotation direction (default: 1)
  --power-loop 0|1        Enable the one-shot vertical spatial loop (default: 0)
  --power-loop-trigger-s-m M  Measured course distance for loop entry (default: 1.0)
  --power-loop-trigger-window-m M  Width of loop trigger window (default: 0.25)
  --power-loop-radius-m M Vertical loop radius (default: 1.2)
  --power-loop-bottom-speed-mps MPS  Entry/exit speed (default: 2.2)
  --power-loop-top-speed-mps MPS  Inverted-top speed (default: 4.0)
  --reactive-power-loop 0|1  Enable the one-shot cached vertical loop when
                            route-free center risk stays <=0.20 (default: 0)
  --pitch-through-brake 0|1  Use the cache-free moving-entry minimum-distance
                            brake for emergency TinyMPC arms (default: 0)
  --vertical-active-sensing 0|1  Add the opt-in +/-0.10 m, 2.0 s altitude
                            square wave to the nominal reference (default: 0)
  --yaw-spin-test 0|1     Command an in-place fixed-rate yaw diagnostic (default: 0)
  --yaw-spin-rate-rad-s RATE  Signed yaw rate for the spin diagnostic (default: 2.0)
  --yaw-spin-revolutions N  Positive number of commanded turns (default: 4.0)
  --yaw-spin-straight-speed-mps MPS  Insert a straight segment halfway through the turns (default: 0)
  --yaw-spin-straight-duration-s S  Duration of the optional straight segment (default: 2.0)
  --duration SECONDS      Simulation duration (default: 10)
  --launch-time SECONDS   Airborne handoff/controller start (default: 1)
  --spawn-z METERS        Handoff altitude (default: 1.5)
  --spawn-yaw-deg DEGREES Initial heading applied at airborne handoff (default: 0)
  --model NAME            CrazySim model (default: cf21B_500)
  --mass KG|stock         Override vehicle mass (default: stock model value)
  --pwm-thrust-full N     Full normalized-command thrust (default: 0.20)
  --launch-prespin 0|1    Skip or apply RPM pre-spin at handoff (default: 0)
  --random-seed INTEGER   Seed noise and turbulence (default: 1)
  --inertia-scale SCALE   Scale plant diagonal inertia (default: 1.0)
  --motor-tau-scale SCALE Scale plant motor lag (default: 1.0)
  --thrust-scale SCALE    Scale realized plant thrust (default: 1.0)
  --realtime-factor RATE  Simulator/wall rate cap (default: 1.0)
  --firmware-time-factor RATE  Measured simulator/wall rate (default: 0.8)
  --out DIRECTORY         Output directory
  --vision-model PATH     Model path, or bundled tinyracer/espnet, dronet-v3,
                          sequential, stdc, vision-rl, combined-gate-rl, joint-gate-rl,
                          gate-frontnet-olgmd (160x96 bring-up), olgmd-obstacle (canonical),
                          the paper-ablation aliases espnet-v7-{pid,emergency},
                          dronet-v{2,3}-{pid,emergency}, and
                          nanoflow-paper-{pid,emergency-yaw}, oracle-brake-pid,
                          espnet-v7 (current float four-corner controller),
                          espnet-v7-gap8 (unreleased INT8 evaluation model),
                          espnet-v7-gap8-reactive (current v8 collision/v9 rail INT8 model with route-free reactive control),
                          espnet-v7-legacy-residual (historical only), or an explicit bundle path;
                          frame-ablation bundles are selected from their manifest
  --vision-adapter NAME   auto, espnet, sequential, stdc, dronet, hybrid_rl,
                          combined_gate_rl, joint_gate_rl, joint_residual_rl, or
                          espnet_v7_dronet, espnet_frame_ablation,
                          espnet_v7_dronet_gap8,
                          espnet_v7_reactive_gap8, gate_frontnet_olgmd,
                          tinyvpc_square_opening, or legacy
                          espnet_v7_residual (default: auto)
  --vision-scene NAME     obstacle, gate, corridor, corridor_obstacles,
                          circle_obstacles, figure8_obstacles, imav22, or none
                          (default: obstacle; gate-frontnet-olgmd defaults to gate)
  --vision-latency-frames N  Fixed camera-frame delivery delay (default: 1)
  --camera-fps FPS        Camera/inference cadence (default: 30)
  --vision-control-enable-after-s S  Suppress paper-adapter control until the
                            obstacle reveal time (default: 0)
  --vision-control-enable-at-x-m M  Reveal paper obstacle at this CrazySim x
  --vision-passive        Run/log inference without sending it to firmware
  --camera-only          Capture AI-deck-style frames without inference or firmware I/O
  --imav22-mode MODE     Official gates, static, or dynamic environment complexity
  --imav22-relocation-period S  Dynamic-mode interval between relocation attempts (default: 30)
  --imav22-relocation-clearance M  Base drone clearance before relocation (default: 1.5;
                            object footprint radius is added)
  --imav22-relocation-duration S  Smooth relocation duration (default: 2)
  --imav22-object-set SET  official or extended textured robustness objects
  --course NAME           Select a manifest-backed obstacle/gate course
  --container-engine NAME docker (default) or apptainer; the latter requires --apptainer-image
  --apptainer-image PATH  Shared .sif used only with --container-engine apptainer
  --video                 Render flight.mp4 after simulation (does not affect timing)
  --video-fps FPS         Encoded video frame rate (default: 24)
  --video-speed FACTOR    Replay speed multiplier (default: 2)
  --sensor-noise          Enable CrazySim IMU/barometer noise
  --flowdeck              Opt in to CrazySim's simulated Flow deck
  --no-flowdeck           Use simulator-provided external pose (default)
  --ground-effect         Enable ground effect
  --wind-speed MPS        Add constant wind
  --turbulence LEVEL      none, light, moderate, or severe
  --overwrite             Replace an existing output directory
  --stop-on-contact       End simulation after the first logged contact (default)
  --no-stop-on-contact    Continue after contact for post-crash diagnostics
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --trajectory) TRAJECTORY="$2"; shift 2 ;;
    --actuator-lti) ACTUATOR_LTI="$2"; shift 2 ;;
    --rate-cascade) RATE_CASCADE="$2"; shift 2 ;;
    --braking-cache) BRAKING_CACHE="$2"; shift 2 ;;
    --direct-plan-replay) DIRECT_PLAN_REPLAY="$2"; shift 2 ;;
    --rate-identification) RATE_IDENTIFICATION=1; shift ;;
    --mpc-diag-mode) MPC_DIAG_MODE="$2"; shift 2 ;;
    --level-cost-mode) LEVEL_COST_MODE="$2"; shift 2 ;;
    --progress-sample-limit) PROGRESS_SAMPLE_LIMIT="$2"; shift 2 ;;
    --progress-speed-mps) PROGRESS_SPEED_MPS="$2"; shift 2 ;;
    --progress-laps) PROGRESS_LAPS="$2"; shift 2 ;;
    --progress-entry-acceleration-mps2) PROGRESS_ENTRY_ACCELERATION_MPS2="$2"; shift 2 ;;
    --progress-terminal-deceleration-mps2) PROGRESS_TERMINAL_DECELERATION_MPS2="$2"; shift 2 ;;
    --progress-reward-weight) PROGRESS_REWARD_WEIGHT="$2"; shift 2 ;;
    --flip) FLIP_ENABLE="$2"; shift 2 ;;
    --flip-trigger-s-m) FLIP_TRIGGER_S_M="$2"; shift 2 ;;
    --flip-trigger-window-m) FLIP_TRIGGER_WINDOW_M="$2"; shift 2 ;;
    --flip-duration-s) FLIP_DURATION_S="$2"; shift 2 ;;
    --flip-pitch-direction) FLIP_PITCH_DIRECTION="$2"; shift 2 ;;
    --power-loop) POWER_LOOP_ENABLE="$2"; shift 2 ;;
    --power-loop-trigger-s-m) POWER_LOOP_TRIGGER_S_M="$2"; shift 2 ;;
    --power-loop-trigger-window-m) POWER_LOOP_TRIGGER_WINDOW_M="$2"; shift 2 ;;
    --power-loop-radius-m) POWER_LOOP_RADIUS_M="$2"; shift 2 ;;
    --power-loop-bottom-speed-mps) POWER_LOOP_BOTTOM_SPEED_MPS="$2"; shift 2 ;;
    --power-loop-top-speed-mps) POWER_LOOP_TOP_SPEED_MPS="$2"; shift 2 ;;
    --reactive-power-loop) REACTIVE_POWER_LOOP="$2"; shift 2 ;;
    --pitch-through-brake) PITCH_THROUGH_BRAKE="$2"; shift 2 ;;
    --vertical-active-sensing) VERTICAL_ACTIVE_SENSING="$2"; shift 2 ;;
    --yaw-spin-test) YAW_SPIN_TEST="$2"; shift 2 ;;
    --yaw-spin-rate-rad-s) YAW_SPIN_RATE_RAD_S="$2"; shift 2 ;;
    --yaw-spin-revolutions) YAW_SPIN_REVOLUTIONS="$2"; shift 2 ;;
    --yaw-spin-straight-speed-mps) YAW_SPIN_STRAIGHT_SPEED_MPS="$2"; shift 2 ;;
    --yaw-spin-straight-duration-s) YAW_SPIN_STRAIGHT_DURATION_S="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --launch-time) LAUNCH_TIME="$2"; shift 2 ;;
    --spawn-z) SPAWN_Z="$2"; shift 2 ;;
    --spawn-yaw-deg)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=(--spawn-yaw-deg "$2"); shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --mass) MASS="$2"; shift 2 ;;
    --pwm-thrust-full) PWM_THRUST_FULL="$2"; shift 2 ;;
    --launch-prespin) LAUNCH_PRESPIN="$2"; shift 2 ;;
    --random-seed) RANDOM_SEED="$2"; shift 2 ;;
    --inertia-scale) INERTIA_SCALE="$2"; shift 2 ;;
    --motor-tau-scale) MOTOR_TAU_SCALE="$2"; shift 2 ;;
    --thrust-scale) THRUST_SCALE="$2"; shift 2 ;;
    --realtime-factor)
      REALTIME_FACTOR="$2"; REALTIME_FACTOR_EXPLICIT=1; shift 2 ;;
    --firmware-time-factor)
      FIRMWARE_TIME_FACTOR="$2"; FIRMWARE_TIME_FACTOR_EXPLICIT=1; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --vision-model) VISION_MODEL="$2"; shift 2 ;;
    --vision-adapter) VISION_ADAPTER="$2"; shift 2 ;;
    --vision-scene) VISION_SCENE="$2"; VISION_SCENE_EXPLICIT=1; shift 2 ;;
    --vision-latency-frames) VISION_LATENCY_FRAMES="$2"; shift 2 ;;
    --camera-fps) CAMERA_FPS="$2"; shift 2 ;;
    --vision-control-enable-after-s) VISION_CONTROL_ENABLE_AFTER_S="$2"; shift 2 ;;
    --vision-control-enable-at-x-m) VISION_CONTROL_ENABLE_AT_X_M="$2"; shift 2 ;;
    --vision-passive) VISION_PASSIVE=1; shift ;;
    --camera-only) CAMERA_ONLY=1; shift ;;
    --imav22-mode) IMAV22_MODE="$2"; shift 2 ;;
    --imav22-relocation-period) IMAV22_RELOCATION_PERIOD_S="$2"; shift 2 ;;
    --imav22-relocation-clearance) IMAV22_RELOCATION_CLEARANCE_M="$2"; shift 2 ;;
    --imav22-relocation-duration) IMAV22_RELOCATION_DURATION_S="$2"; shift 2 ;;
    --imav22-object-set) IMAV22_OBJECT_SET="$2"; shift 2 ;;
    --course) COURSE="$2"; shift 2 ;;
    --container-engine) CONTAINER_ENGINE="$2"; shift 2 ;;
    --apptainer-image) APPTAINER_IMAGE="$2"; shift 2 ;;
    --video) RENDER_VIDEO=1; shift ;;
    --video-fps) VIDEO_FPS="$2"; shift 2 ;;
    --video-speed) VIDEO_SPEED="$2"; shift 2 ;;
    --flowdeck)
      FLOWDECK_ENABLED=1; shift ;;
    --no-flowdeck)
      FLOWDECK_ENABLED=0; shift ;;
    --sensor-noise|--ground-effect)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1"); shift ;;
    --wind-speed|--turbulence)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1" "$2"); shift 2 ;;
    --overwrite) OVERWRITE=1; shift ;;
    --stop-on-contact) STOP_ON_CONTACT=1; shift ;;
    --no-stop-on-contact) STOP_ON_CONTACT=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$FLOWDECK_ENABLED" == 1 ]]; then
  [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
  EXTRA_SIM_ARGS+=(--flowdeck)
fi

case "$TRAJECTORY" in
  straight|straight_long|straight_9m|straight_20m|canonical_corridor|canonical_circle|figure8|oval|canonical_figure8|circle|imav22_circle|chicane|canonical_chicane|hairpin_180|canonical_hairpin|dronet_u) ;;
  *) echo "Unsupported trajectory: $TRAJECTORY" >&2; exit 2 ;;
esac
if [[ "$ACTUATOR_LTI" != 0 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--actuator-lti must be 0 or 1" >&2
  exit 2
fi
if [[ "$RATE_CASCADE" != 0 && "$RATE_CASCADE" != 1 ]]; then
  echo "--rate-cascade must be 0 or 1" >&2
  exit 2
fi
if [[ "$RATE_CASCADE" == 1 && "$ACTUATOR_LTI" == 1 ]]; then
  echo "WARNING: --rate-cascade 1 bypasses the direct actuator-LTI braking cache; use --rate-cascade 0 to exercise BRAKE model switching." >&2
fi
case "$BRAKING_CACHE" in
  auto)
    BRAKING_CACHE_EFFECTIVE=0
    [[ "$ACTUATOR_LTI" == 1 && "$RATE_CASCADE" == 0 ]] && \
      BRAKING_CACHE_EFFECTIVE=1 ;;
  0) BRAKING_CACHE_EFFECTIVE=0 ;;
  1)
    [[ "$ACTUATOR_LTI" == 1 && "$RATE_CASCADE" == 0 ]] || {
      echo "--braking-cache 1 requires --actuator-lti 1 --rate-cascade 0" >&2
      exit 2
    }
    BRAKING_CACHE_EFFECTIVE=1 ;;
  *) echo "--braking-cache must be auto, 0, or 1" >&2; exit 2 ;;
esac
if [[ "$DIRECT_PLAN_REPLAY" != 0 && "$DIRECT_PLAN_REPLAY" != 1 ]]; then
  echo "--direct-plan-replay must be 0 or 1" >&2
  exit 2
fi
if [[ "$DIRECT_PLAN_REPLAY" == 1 && "$RATE_CASCADE" != 0 ]]; then
  echo "--direct-plan-replay 1 requires --rate-cascade 0" >&2
  exit 2
fi
if [[ "$DIRECT_PLAN_REPLAY" == 1 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--direct-plan-replay 1 requires --actuator-lti 1" >&2
  exit 2
fi
if [[ "$RATE_IDENTIFICATION" == 1 && "$RATE_CASCADE" != 1 ]]; then
  echo "--rate-identification requires --rate-cascade 1" >&2
  exit 2
fi
case "$MPC_DIAG_MODE" in
  off|taskless) ;;
  *) echo "--mpc-diag-mode must be off or taskless" >&2; exit 2 ;;
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
fi
if [[ ! "$PROGRESS_LAPS" =~ ^[1-8]$ ]]; then
  echo "--progress-laps must be an integer from 1 through 8" >&2
  exit 2
fi
if [[ "$PROGRESS_LAPS" != 1 ]]; then
  if [[ "$TRAJECTORY" != circle && "$TRAJECTORY" != imav22_circle ]]; then
    echo "--progress-laps greater than 1 requires a closed circle trajectory" >&2
    exit 2
  fi
fi
if ! python3 - "$PROGRESS_ENTRY_ACCELERATION_MPS2" <<'PY'
import math
import sys
try:
    acceleration = float(sys.argv[1])
except ValueError:
    raise SystemExit("--progress-entry-acceleration-mps2 must be positive and finite")
if not math.isfinite(acceleration) or acceleration <= 0.0:
    raise SystemExit("--progress-entry-acceleration-mps2 must be positive and finite")
PY
then
  exit 2
fi
if ! python3 - "$PROGRESS_TERMINAL_DECELERATION_MPS2" <<'PY'
import math
import sys
try:
    deceleration = float(sys.argv[1])
except ValueError:
    raise SystemExit("--progress-terminal-deceleration-mps2 must be positive and finite")
if not math.isfinite(deceleration) or deceleration <= 0.0:
    raise SystemExit("--progress-terminal-deceleration-mps2 must be positive and finite")
PY
then
  exit 2
fi
if ! python3 - "$PROGRESS_REWARD_WEIGHT" <<'PY'
import math
import sys
try:
    weight = float(sys.argv[1])
except ValueError:
    raise SystemExit("--progress-reward-weight must be nonnegative and finite")
if not math.isfinite(weight) or weight < 0.0:
    raise SystemExit("--progress-reward-weight must be nonnegative and finite")
PY
then
  exit 2
fi
if [[ "$FLIP_ENABLE" != 0 && "$FLIP_ENABLE" != 1 ]]; then
  echo "--flip must be 0 or 1" >&2
  exit 2
fi
if [[ "$FLIP_PITCH_DIRECTION" != -1 && "$FLIP_PITCH_DIRECTION" != 1 ]]; then
  echo "--flip-pitch-direction must be -1 or 1" >&2
  exit 2
fi
if [[ "$FLIP_ENABLE" == 1 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--flip 1 requires --actuator-lti 1" >&2
  exit 2
fi
if [[ "$POWER_LOOP_ENABLE" != 0 && "$POWER_LOOP_ENABLE" != 1 ]]; then
  echo "--power-loop must be 0 or 1" >&2
  exit 2
fi
if [[ "$POWER_LOOP_ENABLE" == 1 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--power-loop 1 requires --actuator-lti 1" >&2
  exit 2
fi
if [[ "$POWER_LOOP_ENABLE" == 1 && "$FLIP_ENABLE" == 1 ]]; then
  echo "--power-loop and --flip are mutually exclusive" >&2
  exit 2
fi
if [[ "$REACTIVE_POWER_LOOP" != 0 && "$REACTIVE_POWER_LOOP" != 1 ]]; then
  echo "--reactive-power-loop must be 0 or 1" >&2
  exit 2
fi
if [[ "$REACTIVE_POWER_LOOP" == 1 && "$ACTUATOR_LTI" != 1 ]]; then
  echo "--reactive-power-loop 1 requires --actuator-lti 1" >&2
  exit 2
fi
if [[ "$REACTIVE_POWER_LOOP" == 1 && "$RATE_CASCADE" != 0 ]]; then
  echo "--reactive-power-loop 1 requires direct motors (--rate-cascade 0)" >&2
  exit 2
fi
if [[ "$REACTIVE_POWER_LOOP" == 1 && "$POWER_LOOP_ENABLE" == 1 ]]; then
  echo "--reactive-power-loop and --power-loop are mutually exclusive" >&2
  exit 2
fi
if [[ "$REACTIVE_POWER_LOOP" == 1 && "$FLIP_ENABLE" == 1 ]]; then
  echo "--reactive-power-loop and --flip are mutually exclusive" >&2
  exit 2
fi
if [[ "$PITCH_THROUGH_BRAKE" != 0 && "$PITCH_THROUGH_BRAKE" != 1 ]]; then
  echo "--pitch-through-brake must be 0 or 1" >&2
  exit 2
fi
if [[ "$PITCH_THROUGH_BRAKE" == 1 &&
      ( "$ACTUATOR_LTI" != 1 || "$RATE_CASCADE" != 0 ) ]]; then
  echo "--pitch-through-brake 1 requires --actuator-lti 1 --rate-cascade 0" >&2
  exit 2
fi
if [[ "$PITCH_THROUGH_BRAKE" == 1 &&
      ( "$POWER_LOOP_ENABLE" == 1 || "$REACTIVE_POWER_LOOP" == 1 ||
        "$FLIP_ENABLE" == 1 ) ]]; then
  echo "--pitch-through-brake is mutually exclusive with loop/flip primitives" >&2
  exit 2
fi
if [[ "$VERTICAL_ACTIVE_SENSING" != 0 && "$VERTICAL_ACTIVE_SENSING" != 1 ]]; then
  echo "--vertical-active-sensing must be 0 or 1" >&2
  exit 2
fi
if [[ "$YAW_SPIN_TEST" != 0 && "$YAW_SPIN_TEST" != 1 ]]; then
  echo "--yaw-spin-test must be 0 or 1" >&2
  exit 2
fi
if ! python3 - "$YAW_SPIN_RATE_RAD_S" "$YAW_SPIN_REVOLUTIONS" \
    "$YAW_SPIN_STRAIGHT_SPEED_MPS" "$YAW_SPIN_STRAIGHT_DURATION_S" <<'PY'
import math
import sys
try:
    rate, revolutions, straight_speed, straight_duration = map(float, sys.argv[1:])
except ValueError:
    raise SystemExit("yaw-spin parameters must be finite numbers")
if not all(map(math.isfinite, (rate, revolutions, straight_speed, straight_duration))):
    raise SystemExit("yaw-spin parameters must be finite numbers")
if rate == 0.0 or revolutions <= 0.0:
    raise SystemExit("yaw-spin rate must be nonzero and revolutions must be positive")
if straight_speed < 0.0 or straight_duration < 0.0:
    raise SystemExit("yaw-spin straight speed and duration must be nonnegative")
PY
then
  exit 2
fi
if [[ "$YAW_SPIN_TEST" == 1 && ( "$ACTUATOR_LTI" != 1 || "$RATE_CASCADE" != 0 ) ]]; then
  echo "--yaw-spin-test 1 requires --actuator-lti 1 --rate-cascade 0" >&2
  exit 2
fi
if ! python3 - "$FLIP_TRIGGER_S_M" "$FLIP_TRIGGER_WINDOW_M" "$FLIP_DURATION_S" <<'PY'
import math
import sys
try:
    trigger, window, duration = map(float, sys.argv[1:])
except ValueError:
    raise SystemExit("flip trigger, window, and duration must be finite numbers")
if not all(map(math.isfinite, (trigger, window, duration))):
    raise SystemExit("flip trigger, window, and duration must be finite numbers")
if trigger < 0.0 or window <= 0.0 or duration <= 0.0:
    raise SystemExit("flip trigger must be nonnegative; window and duration must be positive")
PY
then
  exit 2
fi
if ! python3 - "$POWER_LOOP_TRIGGER_S_M" "$POWER_LOOP_TRIGGER_WINDOW_M" \
    "$POWER_LOOP_RADIUS_M" "$POWER_LOOP_BOTTOM_SPEED_MPS" \
    "$POWER_LOOP_TOP_SPEED_MPS" <<'PY'
import math
import sys
try:
    trigger, window, radius, bottom, top = map(float, sys.argv[1:])
except ValueError:
    raise SystemExit("power-loop parameters must be finite numbers")
if not all(map(math.isfinite, (trigger, window, radius, bottom, top))):
    raise SystemExit("power-loop parameters must be finite numbers")
if trigger < 0.0 or window <= 0.0 or radius <= 0.0 or bottom <= 0.0:
    raise SystemExit("power-loop trigger must be nonnegative and other parameters positive")
if top <= bottom:
    raise SystemExit("power-loop top speed must exceed bottom speed")
PY
then
  exit 2
fi
if [[ "$LEVEL_COST_MODE" != baseline && "$ACTUATOR_LTI" != 1 ]]; then
  echo "non-baseline --level-cost-mode requires --actuator-lti 1" >&2
  exit 2
fi
case "$VISION_ADAPTER" in auto|espnet|sequential|stdc|dronet|rl|hybrid_rl|combined_gate_rl|joint_gate_rl|joint_residual_rl|gate_frontnet_olgmd|gate_frontnet_olgmd_obstacle_only|espnet_v7_residual|espnet_v7_dronet|espnet_v7_pid|espnet_frame_ablation|espnet_v7_reactive|espnet_v7_dronet_gap8|espnet_v7_reactive_gap8|tinyvpc_square_opening|tinyvpc_navigation|tinyvpc_pid|tinyvpc_emergency|oracle_brake|oracle_brake_pid|espnet_v7_emergency|espnet_v7_straight_brake|tiny_dronet_v3_direct|pulp_dronet_v3_paper|pulp_dronet_v3_pid|pulp_dronet_v3_emergency|pulp_dronet_v3_straight_brake|dronet_v2_paper|dronet_v2_pid|dronet_v2_emergency|nanoflow|nanoflow_paper|nanoflow_paper_pid|nanoflow_paper_emergency|nanoflow_paper_emergency_yaw|oracle_action) ;; *) echo "Invalid --vision-adapter" >&2; exit 2 ;; esac
case "$CONTAINER_ENGINE" in docker|apptainer) ;; *) echo "Invalid --container-engine" >&2; exit 2 ;; esac
if [[ "$CONTAINER_ENGINE" == apptainer ]]; then
  [[ -n "$APPTAINER_IMAGE" && -f "$APPTAINER_IMAGE" ]] || { echo "--container-engine apptainer requires a readable --apptainer-image" >&2; exit 2; }
  command -v apptainer >/dev/null || { echo "apptainer is unavailable" >&2; exit 2; }
fi
[[ "$VISION_ADAPTER" != rl ]] || VISION_ADAPTER=hybrid_rl
case "$IMAV22_MODE" in ""|gates|static|dynamic) ;; *) echo "Invalid --imav22-mode (expected gates, static, or dynamic)" >&2; exit 2 ;; esac
case "$IMAV22_OBJECT_SET" in official|extended) ;; *) echo "Invalid --imav22-object-set (expected official or extended)" >&2; exit 2 ;; esac
if [[ "$IMAV22_OBJECT_SET" == extended && ( -z "$IMAV22_MODE" || "$IMAV22_MODE" == gates ) ]]; then
  echo "--imav22-object-set extended requires --imav22-mode static or dynamic" >&2
  exit 2
fi
if [[ -n "$IMAV22_MODE" ]]; then
  VISION_SCENE=imav22
  python3 - "$IMAV22_RELOCATION_PERIOD_S" "$IMAV22_RELOCATION_CLEARANCE_M" "$IMAV22_RELOCATION_DURATION_S" <<'PY'
import math, sys
period, clearance, duration = map(float, sys.argv[1:])
if not all(math.isfinite(value) and value > 0.0 for value in (period, clearance, duration)):
    raise SystemExit("IMAV22 relocation period, clearance, and duration must be finite and positive")
if duration >= period:
    raise SystemExit("IMAV22 relocation duration must be shorter than its period")
PY
fi
case "$VISION_SCENE" in obstacle|gate|corridor|corridor_obstacles|circle_obstacles|figure8_obstacles|imav22|none|straight_offset|straight_slalom|turn_left|canonical_corridor|canonical_circle|canonical_figure8|canonical_chicane|canonical_hairpin|dronet_u|gate_obstacle_poc|gate_obstacle_poc_obstacle_only|gate_obstacle_easy_transition|gate_obstacle_easy_transition_obstacle_only|heldout_room_straight|heldout_room_circle|heldout_room_oval|heldout_room_figure8|straight_approach_9m|straight_approach_9m_dronet|paper_headon_9m|hallway_textured_square_9m) ;; *) echo "Invalid --vision-scene" >&2; exit 2 ;; esac
[[ "$VISION_LATENCY_FRAMES" =~ ^[0-9]+$ ]] || { echo "--vision-latency-frames must be a nonnegative integer" >&2; exit 2; }
if [[ "$CAMERA_ONLY" == 1 && -n "$VISION_MODEL" ]]; then
  echo "--camera-only and --vision-model are mutually exclusive" >&2
  exit 2
fi
if [[ "$VISION_PASSIVE" == 1 && -z "$VISION_MODEL" ]]; then
  echo "--vision-passive requires --vision-model" >&2
  exit 2
fi
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
  read -r course_trajectory course_scene course_realtime_factor \
      course_firmware_time_factor < <(python3 - "$COURSE_MANIFEST" <<'PY'
import json, sys
course = json.load(open(sys.argv[1]))
timing = course.get("sitl_timing_profile", {})
print(course["trajectory"], course["scene"],
      timing.get("realtime_factor", "-"),
      timing.get("firmware_time_factor", "-"))
PY
)
  [[ "$TRAJECTORY" == "$course_trajectory" ]] || {
    echo "Course $COURSE requires --trajectory $course_trajectory" >&2; exit 2;
  }
  VISION_SCENE="${course_scene#vision_}"
  VISION_SCENE="${VISION_SCENE%.xml}"
  # Camera rendering changes the simulator/wall-time ratio. Apply the
  # measured course pair only when neither half was explicitly supplied, so
  # an intentional host-specific calibration always wins as a complete pair.
  if [[ "$REALTIME_FACTOR_EXPLICIT" == 0 \
      && "$FIRMWARE_TIME_FACTOR_EXPLICIT" == 0 \
      && "$course_realtime_factor" != - \
      && "$course_firmware_time_factor" != - ]]; then
    REALTIME_FACTOR="$course_realtime_factor"
    FIRMWARE_TIME_FACTOR="$course_firmware_time_factor"
  fi
fi
VISION_ENABLED=0
VISION_MODEL_CONTAINER=__none__
if [[ -n "$VISION_MODEL" ]]; then
  VISION_ENABLED=1
  case "$VISION_MODEL" in
    gate-frontnet-olgmd|gate_frontnet_olgmd)
      VISION_MODEL="/home/cchen/pulp-frontnet/PyTorch"
      VISION_ADAPTER=gate_frontnet_olgmd ;;
    olgmd-obstacle|olgmd_obstacle|gate-frontnet-olgmd-obstacle-only|gate_frontnet_olgmd_obstacle_only)
      VISION_MODEL="$SCRIPT_DIR/models/olgmd_obstacle_v1"
      VISION_ADAPTER=gate_frontnet_olgmd_obstacle_only ;;
    dronet-v3-pid)
      VISION_MODEL="$SCRIPT_DIR/models/pulp_dronet_v3/pulp_dronet_v3.onnx"
      VISION_ADAPTER=pulp_dronet_v3_pid ;;
    dronet-v3-emergency)
      VISION_MODEL="$SCRIPT_DIR/models/pulp_dronet_v3/pulp_dronet_v3.onnx"
      VISION_ADAPTER=pulp_dronet_v3_emergency ;;
    dronet|dronet-v3)
      VISION_MODEL="$SCRIPT_DIR/models/pulp_dronet_v3/pulp_dronet_v3.onnx"
      VISION_ADAPTER=dronet ;;
    pulp-dronet-v3-straight-brake|dronet-v3-straight-brake)
      VISION_MODEL="$SCRIPT_DIR/models/pulp_dronet_v3/pulp_dronet_v3.onnx"
      VISION_ADAPTER=pulp_dronet_v3_straight_brake ;;
    tiny-dronet-v3|tiny_dronet_v3)
      VISION_MODEL="$SCRIPT_DIR/models/tiny_pulp_dronet_v3/tiny_pulp_dronet_v3.onnx"
      VISION_ADAPTER=tiny_dronet_v3_direct ;;
    dronet-v2-pid)
      VISION_MODEL="/home/cchen/pulp-dronet/pulp-dronet-v2/gapflow/nntool_input/models_onnx/model_original_himax.onnx"
      VISION_ADAPTER=dronet_v2_pid ;;
    dronet-v2|dronetv2|pulp-dronet-v2)
      VISION_MODEL="/home/cchen/pulp-dronet/pulp-dronet-v2/gapflow/nntool_input/models_onnx/model_original_himax.onnx"
      VISION_ADAPTER=dronet_v2_paper ;;
    dronet-v2-emergency|dronetv2-emergency)
      VISION_MODEL="/home/cchen/pulp-dronet/pulp-dronet-v2/gapflow/nntool_input/models_onnx/model_original_himax.onnx"
      VISION_ADAPTER=dronet_v2_emergency ;;
    nanoflow|nanoflownet)
      VISION_MODEL="/home/cchen/drone_rl/outputs/nanoflow_rl/detail_guidance_ablation_20260829/run_6951/selected/export/nanoflownet_fp32.tflite"
      VISION_ADAPTER=nanoflow ;;
    nanoflow-paper-pid)
      VISION_MODEL="/home/cchen/waft-gap8/thirdparty/nanoflownet-cnns/pretrained_models/nanoflownet/nanoflownet_unquantized.tflite"
      VISION_ADAPTER=nanoflow_paper_pid
      VERTICAL_ACTIVE_SENSING=1 ;;
    nanoflow-paper|nanoflownet-paper)
      VISION_MODEL="/home/cchen/waft-gap8/thirdparty/nanoflownet-cnns/pretrained_models/nanoflownet/nanoflownet_unquantized.tflite"
      VISION_ADAPTER=nanoflow_paper ;;
    nanoflow-paper-emergency-yaw)
      VISION_MODEL="/home/cchen/waft-gap8/thirdparty/nanoflownet-cnns/pretrained_models/nanoflownet/nanoflownet_unquantized.tflite"
      VISION_ADAPTER=nanoflow_paper_emergency_yaw
      VERTICAL_ACTIVE_SENSING=1 ;;
    nanoflow-paper-emergency|nanoflownet-paper-emergency)
      VISION_MODEL="/home/cchen/waft-gap8/thirdparty/nanoflownet-cnns/pretrained_models/nanoflownet/nanoflownet_unquantized.tflite"
      VISION_ADAPTER=nanoflow_paper_emergency ;;
    oracle-action|oracle_action)
      VISION_MODEL="$SCRIPT_DIR/models/privileged_oracle_straight_approach"
      VISION_ADAPTER=oracle_action ;;
    vision-rl|rl-poc)
      VISION_MODEL="$SCRIPT_DIR/models/vision_rl_mpc_poc/policy.onnx"
      VISION_ADAPTER=hybrid_rl ;;
    combined-gate-rl|combined_gate_rl)
      # Manifest bundle: the bridge resolves its two frozen teachers relative
      # to this directory, preserving the existing obstacle policy exactly.
      VISION_MODEL="$SCRIPT_DIR/models/vision_rl_gate_poc"
      VISION_ADAPTER=combined_gate_rl ;;
    joint-gate-rl|joint_gate_rl)
      # A single, explicitly selected joint-policy bundle.  Auto detection is
      # deliberately unchanged so a new experimental model cannot replace an
      # established adapter by merely being present on disk.
      VISION_MODEL="$SCRIPT_DIR/models/vision_rl_gate_joint"
      VISION_ADAPTER=joint_gate_rl ;;
    espnet-v7-legacy-residual|espnetv2-imav22-residual)
      echo "WARNING: ESPNet v7 PPO residual actor is legacy and excluded from current flight/acceptance testing." >&2
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_residual_v7"
      VISION_ADAPTER=espnet_v7_residual ;;
    espnet-v7-pid)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_pid ;;
    tinyvpc-pid)
      VISION_MODEL="$SCRIPT_DIR/models/tinyvpc_navigation_v1"
      VISION_ADAPTER=tinyvpc_pid ;;
    tinyvpc-emergency)
      VISION_MODEL="$SCRIPT_DIR/models/tinyvpc_navigation_v1"
      VISION_ADAPTER=tinyvpc_emergency ;;
    oracle-brake)
      VISION_MODEL="$SCRIPT_DIR/models/oracle_brake"
      VISION_ADAPTER=oracle_brake ;;
    oracle-brake-pid)
      VISION_MODEL="$SCRIPT_DIR/models/oracle_brake"
      VISION_ADAPTER=oracle_brake_pid ;;
    espnet-v7|espnet-v7-dronet|espnetv2-imav22-dronet)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_dronet ;;
    espnet-v7-reactive|espnetv2-imav22-reactive)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_reactive ;;
    espnet-v7-emergency|espnetv2-imav22-emergency)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_emergency ;;
    espnet-v7-straight-brake|espnetv2-imav22-straight-brake)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_straight_brake ;;
    espnet-v7-gap8|espnet-v7-gap8-dronet)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gap8_v12"
      VISION_ADAPTER=espnet_v7_dronet_gap8 ;;
    espnet-v7-gap8-reactive)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gap8_close_rail_v9"
      VISION_ADAPTER=espnet_v7_reactive_gap8 ;;
    espnet-v7-float|espnet-v7-float-dronet)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_gate_rail_v2"
      VISION_ADAPTER=espnet_v7_dronet ;;
    espnet-v7-float-legacy|espnet-v7-float-collision-only)
      VISION_MODEL="$SCRIPT_DIR/models/espnetv2_imav22_dronet_v7"
      VISION_ADAPTER=espnet_v7_dronet ;;
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
  if [[ "$VISION_ADAPTER" == auto && -d "$VISION_MODEL" && \
        -f "$VISION_MODEL/bundle.json" ]]; then
    detected_runtime_adapter="$(python3 - "$VISION_MODEL/bundle.json" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1])).get("runtime_adapter", ""))
PY
)"
    [[ "$detected_runtime_adapter" != espnet_frame_ablation ]] || \
      VISION_ADAPTER=espnet_frame_ablation
    [[ "$detected_runtime_adapter" != espnet_v7_reactive_gap8 ]] || \
      VISION_ADAPTER=espnet_v7_reactive_gap8
    [[ "$detected_runtime_adapter" != tinyvpc_square_opening ]] || \
      VISION_ADAPTER=tinyvpc_square_opening
    [[ "$detected_runtime_adapter" != tinyvpc_navigation ]] || \
      VISION_ADAPTER=tinyvpc_navigation
    [[ "$detected_runtime_adapter" != gate_frontnet_olgmd ]] || \
      VISION_ADAPTER=gate_frontnet_olgmd
    [[ "$detected_runtime_adapter" != gate_frontnet_olgmd_obstacle_only ]] || \
      VISION_ADAPTER=gate_frontnet_olgmd_obstacle_only
  fi
  if [[ "$VISION_ADAPTER" == gate_frontnet_olgmd &&
        "$VISION_SCENE_EXPLICIT" == 0 ]]; then
    VISION_SCENE=gate
  elif [[ "$VISION_ADAPTER" == gate_frontnet_olgmd_obstacle_only &&
          "$VISION_SCENE_EXPLICIT" == 0 ]]; then
    VISION_SCENE=obstacle
  fi
  if [[ "$VISION_ADAPTER" == gate_frontnet_olgmd_obstacle_only ]]; then
    [[ "$BRAKING_CACHE" != 1 ]] || {
      echo "olgmd-obstacle uses the cache-free 10 m/s^2 emergency formulation; use --braking-cache auto or 0" >&2
      exit 2
    }
    BRAKING_CACHE_EFFECTIVE=0
  fi
  case "$TRAJECTORY" in
    straight|straight_long|straight_9m|straight_20m|canonical_corridor|canonical_circle|figure8|oval|canonical_figure8|circle|imav22_circle|chicane|canonical_chicane|hairpin_180|canonical_hairpin|dronet_u) ;;
    *) echo "Vision requires a supported level-flight trajectory." >&2; exit 2 ;;
  esac
  if [[ "$PITCH_THROUGH_BRAKE" == 1 ]]; then
    case "$VISION_ADAPTER" in
      oracle_brake|tinyvpc_emergency|espnet_v7_emergency|dronet_v2_emergency|pulp_dronet_v3_emergency|nanoflow_paper_emergency_yaw) ;;
      *) echo "--pitch-through-brake requires an emergency TinyMPC vision adapter" >&2; exit 2 ;;
    esac
    [[ "$BRAKING_CACHE_EFFECTIVE" == 0 ]] || {
      echo "--pitch-through-brake is cache-free; pass --braking-cache 0" >&2
      exit 2
    }
  fi
  if [[ "$VISION_ADAPTER" == espnet_v7_emergency ||
        "$VISION_ADAPTER" == dronet_v2_emergency ||
        "$VISION_ADAPTER" == pulp_dronet_v3_emergency ||
        "$VISION_ADAPTER" == nanoflow_paper_emergency_yaw ]] &&
      [[ "$PITCH_THROUGH_BRAKE" != 1 ]]; then
    [[ "$BRAKING_CACHE_EFFECTIVE" == 1 ]] || {
      echo "Emergency frontend modes require direct TinyMPC with --braking-cache 1 (or auto)." >&2
      exit 2
    }
  fi
  if [[ "$VISION_ADAPTER" == combined_gate_rl ||
        "$VISION_ADAPTER" == espnet_v7_dronet ||
        "$VISION_ADAPTER" == espnet_v7_pid ||
        "$VISION_ADAPTER" == espnet_v7_dronet_gap8 ||
        "$VISION_ADAPTER" == espnet_v7_reactive_gap8 ||
        "$VISION_ADAPTER" == espnet_v7_emergency ||
        "$VISION_ADAPTER" == espnet_v7_straight_brake ||
        "$VISION_ADAPTER" == espnet_v7_reactive ]]; then
    # The manifest intentionally points to sibling model bundles. Mount the
    # common models root so those relative teacher paths remain valid inside
    # the container instead of exposing only the manifest leaf directory.
    VISION_MODEL_MOUNT="$SCRIPT_DIR/models"
    VISION_MODEL_CONTAINER="/vision_model/$(basename "$VISION_MODEL")"
  elif [[ -d "$VISION_MODEL" ]]; then
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
  "$TRAJECTORY" "$ACTUATOR_LTI" "$RATE_CASCADE" "$REFERENCE_MODE" "$DURATION" \
  "$LAUNCH_TIME" "$SPAWN_Z" "$MODEL" "$MASS" "$PWM_THRUST_FULL" \
  "$LAUNCH_PRESPIN" "$RANDOM_SEED" "$INERTIA_SCALE" "$MOTOR_TAU_SCALE" \
  "$THRUST_SCALE" "$REALTIME_FACTOR" "$FIRMWARE_TIME_FACTOR" \
  "$VISION_ENABLED" "$VISION_MODEL" "$VISION_ADAPTER" "$VISION_SCENE" \
  "$VISION_LATENCY_FRAMES" "$COURSE" "$COURSE_MANIFEST" \
  "$RENDER_VIDEO" "$VIDEO_FPS" "$VIDEO_SPEED" "$repo_commit" "$repo_dirty" \
  "$STOP_ON_CONTACT" "$LEVEL_COST_MODE" "$PROGRESS_SAMPLE_LIMIT" "$PROGRESS_SPEED_MPS" \
  "$PROGRESS_REFERENCE_LIMITS_EFFECTIVE" "$PROGRESS_LAPS" "$PROGRESS_ENTRY_ACCELERATION_MPS2" "$PROGRESS_TERMINAL_DECELERATION_MPS2" "$PROGRESS_REWARD_WEIGHT" \
  "$FLIP_ENABLE" "$FLIP_TRIGGER_S_M" "$FLIP_TRIGGER_WINDOW_M" "$FLIP_DURATION_S" "$FLIP_PITCH_DIRECTION" \
  "$POWER_LOOP_ENABLE" "$POWER_LOOP_TRIGGER_S_M" "$POWER_LOOP_TRIGGER_WINDOW_M" "$POWER_LOOP_RADIUS_M" "$POWER_LOOP_BOTTOM_SPEED_MPS" "$POWER_LOOP_TOP_SPEED_MPS" \
  "$VERTICAL_ACTIVE_SENSING" \
  "$CAMERA_ONLY" "$VISION_PASSIVE" "$FLOWDECK_ENABLED" "$RATE_IDENTIFICATION" "$DIRECT_PLAN_REPLAY" "$MPC_DIAG_MODE" "$TICK_US" \
  "$IMAV22_MODE" "$IMAV22_RELOCATION_PERIOD_S" "$IMAV22_RELOCATION_CLEARANCE_M" "$IMAV22_RELOCATION_DURATION_S" "$IMAV22_OBJECT_SET" \
  "$CAMERA_FPS" "$VISION_CONTROL_ENABLE_AFTER_S" "${VISION_CONTROL_ENABLE_AT_X_M:-null}" \
  "$BRAKING_CACHE" "$BRAKING_CACHE_EFFECTIVE" \
  "$YAW_SPIN_TEST" "$YAW_SPIN_RATE_RAD_S" "$YAW_SPIN_REVOLUTIONS" \
  "$YAW_SPIN_STRAIGHT_SPEED_MPS" "$YAW_SPIN_STRAIGHT_DURATION_S" \
  "$REACTIVE_POWER_LOOP" "$PITCH_THROUGH_BRAKE" \
  "${EXTRA_SIM_ARGS[@]}" <<'PY'
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

(output, repository, runner, trajectory, actuator_lti, rate_cascade, reference_mode, duration,
 launch_time, spawn_z, model, mass, pwm_thrust_full, launch_prespin,
 random_seed, inertia_scale, motor_tau_scale, thrust_scale, realtime_factor,
 firmware_time_factor, vision_enabled, vision_model, vision_adapter,
 vision_scene, vision_latency_frames, course_name, course_manifest,
 render_video, video_fps, video_speed, repository_commit, repository_dirty,
 stop_on_contact, level_cost_mode, progress_sample_limit, progress_speed_mps,
 progress_reference_limits, progress_laps, progress_entry_acceleration_mps2,
 progress_terminal_deceleration_mps2, progress_reward_weight,
 flip_enable, flip_trigger_s_m, flip_trigger_window_m, flip_duration_s,
 flip_pitch_direction,
 power_loop_enable, power_loop_trigger_s_m, power_loop_trigger_window_m,
 power_loop_radius_m, power_loop_bottom_speed_mps, power_loop_top_speed_mps,
 vertical_active_sensing,
 camera_only, vision_passive, flowdeck_enabled, rate_identification, direct_plan_replay, mpc_diag_mode, tick_us,
 imav22_mode, imav22_relocation_period_s, imav22_relocation_clearance_m,
 imav22_relocation_duration_s, imav22_object_set,
 camera_fps, vision_control_enable_after_s, vision_control_enable_at_x_m,
 braking_cache_mode_requested, braking_cache_enabled,
 yaw_spin_test, yaw_spin_rate_rad_s, yaw_spin_revolutions,
 yaw_spin_straight_speed_mps, yaw_spin_straight_duration_s,
 reactive_power_loop, pitch_through_brake,
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

plant_profile_path = (Path(repository) /
    "apps/controller_tinympc_eigen/tools/crazysim_runtime_profile.py")
plant_profile_spec = importlib.util.spec_from_file_location(
    "crazysim_runtime_profile", plant_profile_path)
if plant_profile_spec is None or plant_profile_spec.loader is None:
    raise RuntimeError(f"cannot load plant profile {plant_profile_path}")
plant_profile = importlib.util.module_from_spec(plant_profile_spec)
plant_profile_spec.loader.exec_module(plant_profile)
drag_diagonal = plant_profile.BODY_LINEAR_DRAG_DIAGONAL_N_PER_MPS
resolved_plant_profile = {
    "name": plant_profile.PROFILE_NAME,
    "mass_kg": plant_profile.MASS_KG,
    "diagonal_inertia_kg_m2": list(plant_profile.INERTIA_DIAGONAL_KGM2),
    "drag_matrix_n_s_per_m": [
        [drag_diagonal[0], 0.0, 0.0],
        [0.0, drag_diagonal[1], 0.0],
        [0.0, 0.0, drag_diagonal[2]],
    ],
    "rpm_to_thrust": list(plant_profile.RPM_TO_THRUST),
    "normalized_command_full_thrust_n":
        plant_profile.NORMALIZED_COMMAND_FULL_THRUST_N,
    "actuator_conversion_provenance": {
        "source_class": plant_profile.SOURCE_CLASS,
        "source_revisions": dict(plant_profile.SOURCE_REVISIONS),
        "source_sha256": dict(plant_profile.SOURCE_SHA256),
        "profile_sha256": sha256(plant_profile_path),
    },
}

vision_identity = None
if vision_model:
    path = Path(vision_model)
    if path.is_dir():
        manifest = path / "bundle_manifest.json"
        if not manifest.is_file():
            manifest = path / "bundle.json"
        vision_identity = {
            "path": str(path),
            "bundle_manifest": manifest.name,
            "bundle_manifest_sha256": sha256(manifest),
        }
    else:
        vision_identity = {"path": str(path), "sha256": sha256(path)}

course_scene_sha256 = None
if course_manifest:
    manifest_data = json.loads(Path(course_manifest).read_text())
    scene_path = (Path(repository) / "tools/crazysim_mujoco/scenes"
                  / manifest_data["scene"])
    course_scene_sha256 = sha256(scene_path)

gate_camera_courses = {
    "gate_obstacle_poc", "gate_obstacle_poc_obstacle_only",
    "gate_obstacle_easy_transition",
    "gate_obstacle_easy_transition_obstacle_only",
    "heldout_room_straight", "heldout_room_circle",
    "heldout_room_oval", "heldout_room_figure8",
}
one_meter_gate_courses = {
    "gate_obstacle_easy_transition",
    "gate_obstacle_easy_transition_obstacle_only",
}
scaled_gate_courses = {
    "heldout_room_straight", "heldout_room_circle",
    "heldout_room_oval", "heldout_room_figure8",
}

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
            "apps/controller_tinympc_eigen/src/tinympc_bank_selector.h",
            "apps/controller_tinympc_eigen/src/tinympc_braking_selector.h",
            "apps/controller_tinympc_eigen/src/tinympc_progress_path.h",
            "apps/controller_tinympc_eigen/src/tinympc_flip_primitive.h",
            "apps/controller_tinympc_eigen/src/tinympc_power_loop.h",
            "apps/controller_tinympc_eigen/src/tinympc_reactive_power_loop.h",
            "apps/controller_tinympc_eigen/src/tinympc_pitch_through_brake.h",
            "apps/controller_tinympc_eigen/src/tinympc_vertical_active_sensing.h",
            "apps/controller_tinympc_eigen/src/tiny_pulp_dronet_v3_servo.h",
            "apps/controller_tinympc_eigen/src/pulp_dronet_v2_brake.h",
            "apps/controller_tinympc_eigen/src/trajectories/50hz/traj_reactive_power_loop_360_50hz.h",
            "apps/controller_tinympc_eigen/src/trajectories/50hz/ltv/stored_ltv_reactive_power_loop_360_50hz.h",
        )
    },
    "trajectory_header_sha256": sha256(
        Path(repository) / "apps/controller_tinympc_eigen/src/trajectories/50hz"
        / f"traj_{trajectory}_50hz.h"
    ),
    "trajectory": trajectory,
    "actuator_lti": bool(int(actuator_lti)),
    "rate_cascade": bool(int(rate_cascade)),
    "braking_cache_enabled": bool(int(braking_cache_enabled)),
    "braking_cache_mode_requested": braking_cache_mode_requested,
    "direct_plan_replay": bool(int(direct_plan_replay)),
    "rate_identification": bool(int(rate_identification)),
    "reference_mode": reference_mode,
    "level_cost_mode": level_cost_mode,
    "progress_sample_limit": int(progress_sample_limit),
    "progress_speed_mps": None if not progress_speed_mps else float(progress_speed_mps),
    "progress_reference_limits": progress_reference_limits,
    "progress_laps": int(progress_laps),
    "progress_entry_acceleration_mps2": float(progress_entry_acceleration_mps2),
    "progress_terminal_deceleration_mps2": float(progress_terminal_deceleration_mps2),
    "progress_reward_weight": float(progress_reward_weight),
    "flip_enabled": bool(int(flip_enable)),
    "flip_trigger_s_m": float(flip_trigger_s_m),
    "flip_trigger_window_m": float(flip_trigger_window_m),
    "flip_duration_s": float(flip_duration_s),
    "flip_pitch_direction": int(flip_pitch_direction),
    "power_loop_enabled": bool(int(power_loop_enable)),
    "power_loop_trigger_s_m": float(power_loop_trigger_s_m),
    "power_loop_trigger_window_m": float(power_loop_trigger_window_m),
    "power_loop_radius_m": float(power_loop_radius_m),
    "power_loop_bottom_speed_mps": float(power_loop_bottom_speed_mps),
    "power_loop_top_speed_mps": float(power_loop_top_speed_mps),
    "reactive_power_loop": {
        "enabled": bool(int(reactive_power_loop)),
        "maximum_center_risk": 0.20,
        "clear_samples_required": 1,
        "cache": "power_loop_vertical_360_phase_ltv_current_cf21b",
        "one_shot": True,
        "direct_motors": True,
    },
    "pitch_through_brake": {
        "enabled": bool(int(pitch_through_brake)),
        "cache": None,
        "direct_motors": True,
        "speed_indexed_pitch_deg": [50.0, 65.0],
        "speed_index_range_mps": [1.0, 4.0],
        "maximum_pitch_rate_rad_s": 4.5,
        "maximum_pitch_acceleration_rad_s2": 60.0,
        "maximum_altitude_loss_m": 0.20,
        "reverse_recovery_allowed": True,
        "vertical_state_source_during_maneuver": "internal_thrust_attitude_propagation",
        "flow_and_downward_range_aiding_during_maneuver": False,
    },
    "vertical_active_sensing": {
        "enabled": bool(int(vertical_active_sensing)),
        "waveform": "square",
        "amplitude_m": 0.10,
        "period_s": 2.0,
        "initial_phase": "low",
        "clock": "elapsed_controller_flight_time",
    },
    "yaw_spin_test": {
        "enabled": bool(int(yaw_spin_test)),
        "rate_rad_s": float(yaw_spin_rate_rad_s),
        "revolutions": float(yaw_spin_revolutions),
        "straight_speed_mps": float(yaw_spin_straight_speed_mps),
        "straight_duration_s": float(yaw_spin_straight_duration_s),
    },
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
    "imav22_complexity": ({
        "mode": imav22_mode,
        "environment_factor": {"gates": 1, "static": 5, "dynamic": 10}[imav22_mode],
        "relocation_period_s": float(imav22_relocation_period_s),
        "relocation_clearance_m": float(imav22_relocation_clearance_m),
        "relocation_duration_s": float(imav22_relocation_duration_s),
        "object_set": imav22_object_set,
    } if imav22_mode else None),
    "vision_latency_frames": int(vision_latency_frames),
    "camera_fps": float(camera_fps),
    "vision_control_enable_after_s": float(vision_control_enable_after_s),
    "vision_control_enable_at_x_m": (
        None if vision_control_enable_at_x_m == "null"
        else float(vision_control_enable_at_x_m)),
    "camera_calibration": ({
        "model": "Himax HM01B0",
        "resolution": [160, 160],
        "fx_px": 89.1558392549,
        "fy_px": 89.4608171623,
        "cx_px": 81.1038105230,
        "cy_px": 73.3473030288,
        "distortion_model": "opencv_plumb_bob",
        "distortion_coefficients": [-0.0176448766, 0.0994132451,
                                    0.0054432154, -0.0060400120,
                                    -0.1900189875],
        "simulation_distortion_coefficients": [-0.0176448766, 0.0994132451,
                                                0.0054432154, -0.0060400120,
                                                -0.055],
        "acquisition": {
            "sensor_readout": "HM01B0 HALF 162x162, crop one pixel on every edge to 160x160",
            "frame_rate_hz": 30.0,
            "integration_time_us": 10000,
            "auto_exposure": False,
            "analog_gain_x": 2.0,
            "digital_gain_x": 1.0,
            "image_orientation_register": "0x03",
            "ai_deck_optical_center_body_m": [0.0, 0.0, 0.030],
        },
        "mujoco_vertical_fov_deg": (94.0387229678 if course_name in gate_camera_courses
                                      and vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl")
                                      else 83.6091095),
        "distortion_applied_in_mujoco": False,
        "bridge_calibration_transform_enabled": course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl"),
        "poc_overscan_render_resolution": ([192, 192] if course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl") else None),
        "poc_source_focal_px": (89.4608171623 if course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl") else None),
        "poc_transform": ("principal_point+plumb_bob+isaac_gray_response+seeded_noise"
            if course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl") else None),
        "poc_sensor_seed": (int(random_seed) if course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl") else None),
        "poc_sensor_response_note": (
            "Isaac's lightweight HM01B0 gray response/noise proxy; it is not a measured proof of exact physical sensor response"
            if course_name in gate_camera_courses and
            vision_adapter in ("hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl") else None),
        "source_repository": "isaacsim-workspace shared-home working tree",
        "source_commit": "045ca8b59622b99a408092124377c66346e8d9c2",
        "source_note": (
            "The relevant IsaacSim files are working-tree artifacts outside that upstream commit; their SHA256 identities below are authoritative"
        ),
        "source_files": {
            "calibration": {
                "path": "gap8_perception/configs/hm01b0_calibration.json",
                "sha256": "f34b7b1eaf25c7e6f5b0b6f08a5343abaf0c6b4e07635aae38efe3c6a4fdcef2",
            },
            "isaac_sensor_pipeline": {
                "path": "user_workflows/generate_course_shard.py",
                "sha256": "c8e0094adb079cdc477d2cdbb874be903c980ab1ea02532d39e34bd8c47bfb27",
            },
            "isaac_gate_example": {
                "path": "workspace/course_gate_smoke/shard_000000000/inspection_contact_sheet.jpg",
                "sha256": "08c25690e2df64e7670958ae6b489e81d56e872bbe284242f4f92318b3cc5940",
            },
            "hardware_acquisition_config": {
                "repository": "tinympc-nanocockpit",
                "commit": "1cdd562c0db99f41a2638a21543d5438f0a257ac",
                "path": "src/gap/examples/tiny-racer/config.h",
                "sha256": "b931ab0e6b4726e8af731ecfbcb449d99529df4746ac9a7a7bfdb9d0256d094b",
            },
        },
    } if (vision_adapter in ("dronet", "hybrid_rl", "combined_gate_rl", "joint_gate_rl", "joint_residual_rl")
          or imav22_mode) else None),
    "camera_only_enabled": bool(int(camera_only)),
    "camera_capture_enabled": bool(int(camera_only)) or bool(int(vision_enabled)),
    "camera_inference_enabled": bool(int(vision_enabled)),
    "vision_control_enabled": bool(int(vision_enabled)) and not bool(int(vision_passive)),
    "flowdeck_enabled": bool(int(flowdeck_enabled)),
    "course": course_name or None,
    "course_manifest_sha256": sha256(course_manifest) if course_manifest else None,
    "course_scene_sha256": course_scene_sha256,
    "flight_video_requested": bool(int(render_video)),
    "flight_video_fps": float(video_fps),
    "flight_video_playback_speed": float(video_speed),
    "stop_on_contact": bool(int(stop_on_contact)),
    "extra_simulator_arguments": [item for item in extra if item != "__none__"],
    "mpc_diagnostic": {
        "mode": mpc_diag_mode,
        "enabled": mpc_diag_mode == "taskless",
        "schema": "tinympc-sitl-diag-v2" if mpc_diag_mode == "taskless" else None,
        "path": "mpc_diag.bin" if mpc_diag_mode == "taskless" else None,
        "container": ({
            "format": "tinympc-sitl-diag-mmap-v1",
            "path": "mpc_diag.mmap",
            "magic": "0x544d444d",
            "version": 1,
            "header_size_bytes": 4096,
            "capacity": 16384,
            "slot_size_bytes": 320,
            "expected_size_bytes": 5246976,
            "checksum": "fnv1a32-slot-bytes-8-through-311",
            "size_bytes": None,
            "sha256": None,
            "validation": None,
        } if mpc_diag_mode == "taskless" else None),
        "firmware_tick_us": int(tick_us),
        "drain_grace_s": 0.0,
        "size_bytes": None,
        "sha256": None,
    },
    "acceptance_contract": {
        "controller_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/controller_tinympc.cpp"),
        "vision_bridge_sha256": sha256(
            Path(repository) / "tools/crazysim_mujoco/vision_bridge.py"),
        "analyzer_sha256": sha256(
            Path(repository) / "tools/crazysim_mujoco/analyze_run.py"),
        "bank_header_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_banked_model_bank.h"),
        "bank_provenance_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_banked_model_bank.provenance.json"),
        "braking_selector_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_braking_selector.h"),
        "progress_path_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_progress_path.h"),
        "frenet_error_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_frenet_error.h"),
        "generated_model_sha256": sha256(
            Path(repository) / "apps/controller_tinympc_eigen/src/tinympc_generated_params.h"),
        "plant_model_sha256": sha256(plant_profile_path),
        # Replaced immediately after the firmware build, before launch.
        "firmware_binary_sha256": "0" * 64,
        "circle_radius_m": 0.75,
        "gate_corner_span_m": (1.2333333333333334 if course_name in one_meter_gate_courses
                               else 1.11 if course_name in scaled_gate_courses else 0.555),
        "flowdeck_enabled": bool(int(flowdeck_enabled)),
        "passive_camera_capture_enabled": bool(int(camera_only)),
        "camera_inference_enabled": bool(int(vision_enabled)),
        "vision_control_enabled": (
            bool(int(vision_enabled)) and not bool(int(vision_passive))),
        "direct_plan_replay": bool(int(direct_plan_replay)),
        "resolved_plant_profile": resolved_plant_profile,
    },
}
Path(output).write_text(json.dumps(config, indent=2) + "\n")
PY

# Shared-home Slurm tasks see one pinned dependency checkout. Serialize the
# idempotent submodule/patch probe so concurrent nodes cannot contend on Git's
# module config locks.
flock "$SCRIPT_DIR/.setup.lock" "$SCRIPT_DIR/setup.sh"
repo_rel_out="${OUT#"$REPO_DIR"/}"
if [[ "$CONTAINER_ENGINE" == docker ]]; then
  docker build -q -t "$IMAGE" "$SCRIPT_DIR" >/dev/null
  docker_args=(--rm --interactive --volume "$REPO_DIR:/workspace" --workdir /workspace)
  if [[ "$VISION_ENABLED" == 1 ]]; then
    if [[ "$VISION_ADAPTER" == combined_gate_rl || "$VISION_ADAPTER" == espnet_v7_dronet || "$VISION_ADAPTER" == espnet_v7_pid || "$VISION_ADAPTER" == espnet_v7_reactive || "$VISION_ADAPTER" == espnet_v7_dronet_gap8 || "$VISION_ADAPTER" == espnet_v7_reactive_gap8 || "$VISION_ADAPTER" == espnet_v7_emergency || "$VISION_ADAPTER" == espnet_v7_straight_brake ]]; then docker_args+=(--volume "$VISION_MODEL_MOUNT:/vision_model:ro")
    elif [[ -d "$VISION_MODEL" ]]; then docker_args+=(--volume "$VISION_MODEL_MOUNT:$VISION_MODEL_CONTAINER:ro")
    else docker_args+=(--volume "$VISION_MODEL_MOUNT:/vision_bundle:ro"); fi
  fi
  container_command=(docker run "${docker_args[@]}" "$IMAGE")
else
  # The repository and model live under shared home; bind them identically to
  # Docker.  No Docker executable is consulted on a Slurm compute node.
  container_command=(apptainer exec --bind "$REPO_DIR:/workspace" --pwd /workspace)
  if [[ "$VISION_ENABLED" == 1 ]]; then
    if [[ "$VISION_ADAPTER" == combined_gate_rl || "$VISION_ADAPTER" == espnet_v7_dronet || "$VISION_ADAPTER" == espnet_v7_pid || "$VISION_ADAPTER" == espnet_v7_reactive || "$VISION_ADAPTER" == espnet_v7_dronet_gap8 || "$VISION_ADAPTER" == espnet_v7_reactive_gap8 || "$VISION_ADAPTER" == espnet_v7_emergency || "$VISION_ADAPTER" == espnet_v7_straight_brake ]]; then container_command+=(--bind "$VISION_MODEL_MOUNT:/vision_model:ro")
    elif [[ -d "$VISION_MODEL" ]]; then container_command+=(--bind "$VISION_MODEL_MOUNT:$VISION_MODEL_CONTAINER:ro")
    else container_command+=(--bind "$VISION_MODEL_MOUNT:/vision_bundle:ro"); fi
  fi
  container_command+=("$APPTAINER_IMAGE")
fi

set +e
"${container_command[@]}" bash -s -- \
    "$TRAJECTORY" "$DURATION" "$LAUNCH_TIME" "$SPAWN_Z" \
    "$MODEL" "$MASS" "$PWM_THRUST_FULL" "/workspace/$repo_rel_out" \
    "$LAUNCH_PRESPIN" "$RANDOM_SEED" "$INERTIA_SCALE" \
    "$MOTOR_TAU_SCALE" "$THRUST_SCALE" "$TICK_US" \
    "$VISION_ENABLED" "$VISION_MODEL_CONTAINER" "$VISION_ADAPTER" "$VISION_SCENE" \
    "$ACTUATOR_LTI" "$RATE_CASCADE" "$COURSE" "$VISION_LATENCY_FRAMES" "$REFERENCE_MODE" "$STOP_ON_CONTACT" \
    "$LEVEL_COST_MODE" "$PROGRESS_SAMPLE_LIMIT" "${PROGRESS_SPEED_MPS:-0}" \
    "$PROGRESS_REFERENCE_LIMITS_EFFECTIVE" "$CAMERA_ONLY" "$VISION_PASSIVE" "$RATE_IDENTIFICATION" \
    "$PROGRESS_LAPS" "$PROGRESS_ENTRY_ACCELERATION_MPS2" "$PROGRESS_TERMINAL_DECELERATION_MPS2" "$PROGRESS_REWARD_WEIGHT" \
    "$FLIP_ENABLE" "$FLIP_TRIGGER_S_M" "$FLIP_TRIGGER_WINDOW_M" "$FLIP_DURATION_S" "$FLIP_PITCH_DIRECTION" \
    "$POWER_LOOP_ENABLE" "$POWER_LOOP_TRIGGER_S_M" "$POWER_LOOP_TRIGGER_WINDOW_M" "$POWER_LOOP_RADIUS_M" "$POWER_LOOP_BOTTOM_SPEED_MPS" "$POWER_LOOP_TOP_SPEED_MPS" \
    "$VERTICAL_ACTIVE_SENSING" \
    "$DIRECT_PLAN_REPLAY" "$MPC_DIAG_MODE" \
    "$IMAV22_MODE" "$IMAV22_RELOCATION_PERIOD_S" "$IMAV22_RELOCATION_CLEARANCE_M" "$IMAV22_RELOCATION_DURATION_S" "$IMAV22_OBJECT_SET" \
    "$CAMERA_FPS" "$VISION_CONTROL_ENABLE_AFTER_S" "${VISION_CONTROL_ENABLE_AT_X_M:-none}" \
    "$BRAKING_CACHE_EFFECTIVE" \
    "$YAW_SPIN_TEST" "$YAW_SPIN_RATE_RAD_S" "$YAW_SPIN_REVOLUTIONS" \
    "$YAW_SPIN_STRAIGHT_SPEED_MPS" "$YAW_SPIN_STRAIGHT_DURATION_S" \
    "$REACTIVE_POWER_LOOP" "$PITCH_THROUGH_BRAKE" \
    --realtime-factor "$REALTIME_FACTOR" \
    "${EXTRA_SIM_ARGS[@]}" <<'CONTAINER_SCRIPT'
set -euo pipefail
trajectory="$1"; duration="$2"; launch_time="$3"; spawn_z="$4"
model="$5"; mass="$6"; pwm_thrust_full="$7"; out="$8"
launch_prespin="$9"; random_seed="${10}"
inertia_scale="${11}"; motor_tau_scale="${12}"; thrust_scale="${13}"
tick_us="${14}"
vision_enabled="${15}"; vision_model="${16}"; vision_adapter="${17}"
vision_scene="${18}"; actuator_lti="${19}"; rate_cascade="${20}"
course="${21}"; vision_latency_frames="${22}"; reference_mode="${23}"
stop_on_contact="${24}"; level_cost_mode="${25}"
progress_sample_limit="${26}"; progress_speed_mps="${27}"
progress_reference_limits="${28}"; camera_only="${29}"
vision_passive="${30}"; rate_identification="${31}"; progress_laps="${32}"
progress_entry_acceleration_mps2="${33}"
progress_terminal_deceleration_mps2="${34}"
progress_reward_weight="${35}"; flip_enable="${36}"
flip_trigger_s_m="${37}"; flip_trigger_window_m="${38}"
flip_duration_s="${39}"; flip_pitch_direction="${40}"
power_loop_enable="${41}"; power_loop_trigger_s_m="${42}"
power_loop_trigger_window_m="${43}"; power_loop_radius_m="${44}"
power_loop_bottom_speed_mps="${45}"; power_loop_top_speed_mps="${46}"
vertical_active_sensing="${47}"
direct_plan_replay="${48}"; mpc_diag_mode="${49}"
imav22_mode="${50}"; imav22_relocation_period_s="${51}"
imav22_relocation_clearance_m="${52}"; imav22_relocation_duration_s="${53}"
imav22_object_set="${54}"
camera_fps="${55}"
vision_control_enable_after_s="${56}"
vision_control_enable_at_x_m="${57}"
braking_cache_enabled="${58}"
yaw_spin_test="${59}"
yaw_spin_rate_rad_s="${60}"
yaw_spin_revolutions="${61}"
yaw_spin_straight_speed_mps="${62}"
yaw_spin_straight_duration_s="${63}"
reactive_power_loop="${64}"
pitch_through_brake="${65}"
shift 65
export PYTHONPATH="/workspace/tools/crazysim_mujoco/.runtime_deps${PYTHONPATH:+:$PYTHONPATH}"
course_build="${course:-none}"
hm01b0_poc_camera=0
[[ "$vision_adapter" != gate_frontnet_olgmd &&
    "$vision_adapter" != gate_frontnet_olgmd_obstacle_only ]] || hm01b0_poc_camera=1
if [[ "$course" == gate_obstacle_poc || "$course" == gate_obstacle_poc_obstacle_only || "$course" == gate_obstacle_easy_transition || "$course" == gate_obstacle_easy_transition_obstacle_only || "$course" == heldout_room_straight || "$course" == heldout_room_circle || "$course" == heldout_room_oval || "$course" == heldout_room_figure8 ]]; then
  if [[ "$vision_enabled" != 1 || ( "$vision_adapter" != hybrid_rl && "$vision_adapter" != combined_gate_rl && "$vision_adapter" != joint_gate_rl && "$vision_adapter" != joint_residual_rl && "$vision_adapter" != gate_frontnet_olgmd ) ]]; then
    echo "The gate-obstacle POC requires a supported gate/obstacle vision adapter." >&2
    exit 2
  fi
  # Keep the normal camera path unchanged.  The POC alone renders a compact
  # overscan image so the bridge can apply the Isaac-calibrated K/distortion
  # transform before either frozen teacher sees a 160x160 frame.
  hm01b0_poc_camera=1
fi

crazysim=/workspace/tools/crazysim_mujoco/.deps/CrazySim
firmware="$crazysim/crazyflie-firmware"
simulator="$firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"
app=/workspace/apps/controller_tinympc_eigen
support=/workspace/tools/crazysim_mujoco/sitl
diagnostic_path=""
[[ "$mpc_diag_mode" != taskless ]] || diagnostic_path="$out/mpc_diag.mmap"
gate_controller_mode=off
[[ "$vision_adapter" != combined_gate_rl ]] || gate_controller_mode=poc
[[ "$vision_adapter" != joint_gate_rl ]] || gate_controller_mode=joint
[[ "$vision_adapter" != joint_residual_rl ]] || gate_controller_mode=residual
[[ "$vision_adapter" != espnet_v7_residual ]] || gate_controller_mode=residual
[[ "$vision_adapter" != espnet_v7_dronet ]] || gate_controller_mode=dronetv7
[[ "$vision_adapter" != espnet_v7_pid ]] || gate_controller_mode=espnetv7pid
[[ "$vision_adapter" != espnet_frame_ablation ]] || gate_controller_mode=dronetv7
[[ "$vision_adapter" != espnet_v7_reactive ]] || gate_controller_mode=reactive
[[ "$vision_adapter" != espnet_v7_dronet_gap8 ]] || gate_controller_mode=dronetv7
[[ "$vision_adapter" != espnet_v7_reactive_gap8 ]] || gate_controller_mode=reactive
[[ "$vision_adapter" != tinyvpc_square_opening ]] || gate_controller_mode=tinyvpcopen
[[ "$vision_adapter" != tinyvpc_navigation ]] || gate_controller_mode=tinyvpc
[[ "$vision_adapter" != tinyvpc_pid ]] || gate_controller_mode=tinyvpcpid
[[ "$vision_adapter" != tinyvpc_emergency ]] || gate_controller_mode=tinyvpcemergency
[[ "$vision_adapter" != oracle_brake ]] || gate_controller_mode=oraclebrake
[[ "$vision_adapter" != oracle_brake_pid ]] || gate_controller_mode=oraclebrakepid
[[ "$vision_adapter" != espnet_v7_emergency ]] || gate_controller_mode=espnetemergency
[[ "$vision_adapter" != espnet_v7_straight_brake ]] || gate_controller_mode=espnetstraightbrake
[[ "$vision_adapter" != tiny_dronet_v3_direct ]] || gate_controller_mode=tinydronetv3
[[ "$vision_adapter" != pulp_dronet_v3_pid ]] || gate_controller_mode=dronetv3pid
[[ "$vision_adapter" != pulp_dronet_v3_emergency ]] || gate_controller_mode=dronetv3emergency
[[ "$vision_adapter" != dronet_v2_paper ]] || gate_controller_mode=dronetv2
[[ "$vision_adapter" != dronet_v2_pid ]] || gate_controller_mode=dronetv2pid
[[ "$vision_adapter" != dronet_v2_emergency ]] || gate_controller_mode=dronetv2emergency
[[ "$vision_adapter" != pulp_dronet_v3_straight_brake ]] || gate_controller_mode=dronetv3straightbrake
[[ "$vision_adapter" != nanoflow ]] || gate_controller_mode=nanoflow
[[ "$vision_adapter" != nanoflow_paper ]] || gate_controller_mode=nanoflowpaper
[[ "$vision_adapter" != nanoflow_paper_pid ]] || gate_controller_mode=nanoflowpaperpid
[[ "$vision_adapter" != nanoflow_paper_emergency ]] || gate_controller_mode=nanoflowpaperemergency
[[ "$vision_adapter" != nanoflow_paper_emergency_yaw ]] || gate_controller_mode=nanoflowpaperyawemergency
[[ "$vision_adapter" != oracle_action ]] || gate_controller_mode=oracle
[[ "$vision_adapter" != gate_frontnet_olgmd ]] || gate_controller_mode=gateolgmd
[[ "$vision_adapter" != gate_frontnet_olgmd_obstacle_only ]] || gate_controller_mode=olgmdonly
square_opening_threshold=disabled
square_opening_compile_flag=""
if [[ "$vision_enabled" == 1 && ( "$vision_adapter" == espnet_v7_reactive_gap8 ||
      "$vision_adapter" == tinyvpc_square_opening ) &&
      -f "$vision_model/bundle.json" ]]; then
  square_opening_threshold="$(python3 - "$vision_model/bundle.json" <<'PY'
import json
import sys

manifest = json.load(open(sys.argv[1]))
formats = {
    "tinympc-espnetv2-gap8-square-opening-v11":
        "espnetv2_collision3_square_opening_probability1_v1",
    "tinympc-tinyvpc-square-opening-v1":
        "tinyvpc_normalized_yaw_collision_square_opening_probability_v1",
}
if manifest.get("format") in formats:
    expected = formats[manifest["format"]]
    if manifest.get("semantic_abi") != expected:
        raise SystemExit("v11 square-opening semantic ABI mismatch")
    opening = manifest.get("square_opening", {})
    threshold = opening.get("operating_threshold")
    if type(threshold) not in (int, float) or not 0.0 < float(threshold) < 1.0:
        raise SystemExit("v11 square-opening operating threshold is invalid")
    if opening.get("confirmation_frames") != 2:
        raise SystemExit("v11 square-opening confirmation_frames must be 2")
    print(repr(float(threshold)))
PY
)"
  if [[ -n "$square_opening_threshold" ]]; then
    square_opening_compile_flag="-DTINYMPC_SQUARE_OPENING_THRESHOLD=${square_opening_threshold}f"
  else
    square_opening_threshold=disabled
  fi
fi
build_signature="${trajectory}-course${course_build}-gatemode${gate_controller_mode}-${reference_mode}-${actuator_lti}-cascade${rate_cascade}-brakecache${braking_cache_enabled}-pitchthrough${pitch_through_brake}-replay${direct_plan_replay}-rateid${rate_identification}-${level_cost_mode}-${progress_sample_limit}-speed${progress_speed_mps}-${progress_reference_limits}-laps${progress_laps}-accel${progress_entry_acceleration_mps2}-decel${progress_terminal_deceleration_mps2}-reward${progress_reward_weight}-flip${flip_enable}-powerloop${power_loop_enable}-at${power_loop_trigger_s_m}-r${power_loop_radius_m}-vb${power_loop_bottom_speed_mps}-vt${power_loop_top_speed_mps}-vas${vertical_active_sensing}-reactiveloop${reactive_power_loop}-sqopen${square_opening_threshold}-yawspin${yaw_spin_test}-yawrate${yaw_spin_rate_rad_s}-yawrev${yaw_spin_revolutions}-yawstraight${yaw_spin_straight_speed_mps}x${yaw_spin_straight_duration_s}"
build_hash="$(printf '%s' "$build_signature" | sha256sum | cut -c1-16)"
build="$firmware/sitl_make/build-tinympc-${trajectory}-${build_hash}"
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
rate_identification_compile_flag=""
[[ "$rate_identification" == 1 ]] && \
  rate_identification_compile_flag="-DTINYMPC_RATE_IDENTIFICATION=1"
direct_plan_replay_compile_flag=""
[[ "$direct_plan_replay" == 1 ]] && \
  direct_plan_replay_compile_flag="-DTINYMPC_DIRECT_PLAN_REPLAY=1"
residual_reference_compile_flag=""
[[ "$vision_enabled" == 1 && ( "$vision_adapter" == joint_residual_rl || \
    "$vision_adapter" == espnet_v7_residual || "$vision_adapter" == nanoflow || \
    "$vision_adapter" == nanoflow_paper || \
    "$vision_adapter" == oracle_action ) ]] && \
  residual_reference_compile_flag="-DTINYMPC_VISION_RL_RESIDUAL_ENABLE=1"
direct_dronet_compile_flag=""
if [[ "$vision_enabled" == 1 && ( "$vision_adapter" == tiny_dronet_v3_direct || \
    "$vision_adapter" == oracle_brake_pid || \
    "$vision_adapter" == espnet_v7_pid || \
    "$vision_adapter" == pulp_dronet_v3_pid ) ]]; then
  direct_dronet_compile_flag="-DTINYMPC_DIRECT_DRONET_SERVO=1 -DTINYMPC_PID_VISION_MODE=1 -DTINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M=0.50f"
elif [[ "$vision_enabled" == 1 && "$vision_adapter" == tinyvpc_pid ]]; then
  direct_dronet_compile_flag="-DTINYMPC_DIRECT_DRONET_SERVO=1 -DTINYMPC_PID_VISION_MODE=4 -DTINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M=0.50f"
elif [[ "$vision_enabled" == 1 && "$vision_adapter" == dronet_v2_pid ]]; then
  direct_dronet_compile_flag="-DTINYMPC_DIRECT_DRONET_SERVO=1 -DTINYMPC_PID_VISION_MODE=2 -DTINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M=0.50f"
elif [[ "$vision_enabled" == 1 && "$vision_adapter" == nanoflow_paper_pid ]]; then
  direct_dronet_compile_flag="-DTINYMPC_DIRECT_DRONET_SERVO=1 -DTINYMPC_PID_VISION_MODE=3 -DTINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M=0.50f"
elif [[ "$vision_enabled" == 1 && ( \
    "$vision_adapter" == espnet_v7_straight_brake || \
    "$vision_adapter" == pulp_dronet_v3_straight_brake ) ]]; then
  direct_dronet_compile_flag="-DTINYMPC_DIRECT_DRONET_SERVO=1 -DTINYMPC_DIRECT_DRONET_STEERING_SILENCED=1 -DTINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M=0.50f"
fi
paper_ablation_compile_flag=""
if [[ "$vision_enabled" == 1 && ( \
    "$vision_adapter" == oracle_brake || \
    "$vision_adapter" == oracle_brake_pid || \
    "$vision_adapter" == tinyvpc_pid || \
    "$vision_adapter" == tinyvpc_emergency || \
    "$vision_adapter" == espnet_v7_pid || \
    "$vision_adapter" == espnet_v7_emergency || \
    "$vision_adapter" == dronet_v2_pid || \
    "$vision_adapter" == dronet_v2_emergency || \
    "$vision_adapter" == pulp_dronet_v3_pid || \
    "$vision_adapter" == pulp_dronet_v3_emergency || \
    "$vision_adapter" == nanoflow_paper_pid || \
    "$vision_adapter" == nanoflow_paper_emergency_yaw ) ]]; then
  # Obstacle-free calibration isolates the speed map from visual false stops.
  # Scored and smoke courses retain the preregistered x=3 m enable boundary.
  paper_control_enable_x_m=3.0
  [[ "$vision_scene" != none ]] || paper_control_enable_x_m=9.0
  paper_ablation_compile_flag="-DTINYMPC_PAPER_ABLATION_ENABLE_GATE=1 -DTINYMPC_PAPER_ABLATION_CONTROL_ENABLE_X_M=${paper_control_enable_x_m}f"
fi
paper_emergency_terminal_stop_compile_flag=""
if [[ "$vision_enabled" == 1 && ( \
    "$vision_adapter" == gate_frontnet_olgmd || \
    "$vision_adapter" == gate_frontnet_olgmd_obstacle_only || \
    "$vision_adapter" == oracle_brake || \
    "$vision_adapter" == tinyvpc_emergency || \
    "$vision_adapter" == espnet_v7_emergency || \
    "$vision_adapter" == dronet_v2_emergency || \
    "$vision_adapter" == pulp_dronet_v3_emergency || \
    "$vision_adapter" == nanoflow_paper_emergency_yaw ) ]]; then
  paper_emergency_terminal_stop_compile_flag="-DTINYMPC_PAPER_EMERGENCY_TERMINAL_STOP=1"
fi
reactive_reference_compile_flag=""
[[ "$vision_enabled" == 1 && ( "$vision_adapter" == espnet_v7_reactive || \
    "$vision_adapter" == espnet_v7_reactive_gap8 || \
    "$vision_adapter" == tinyvpc_square_opening ) ]] && \
  reactive_reference_compile_flag="-DTINYMPC_REACTIVE_REFERENCE_FREE=1"
dronet_v2_compile_flag=""
[[ "$vision_enabled" == 1 && "$vision_adapter" == dronet_v2_paper ]] && \
  dronet_v2_compile_flag="-DTINYMPC_VISION_DRONETV2_BRAKE_ENABLE=1"
oracle_action_compile_flag=""
[[ "$vision_enabled" == 1 && "$vision_adapter" == oracle_action ]] && \
  oracle_action_compile_flag="-DTINYMPC_ORACLE_ACTION_ENABLE=1"
espnet_dronet_compile_flag=""
[[ "$vision_enabled" == 1 && ( "$vision_adapter" == espnet_v7_dronet || \
    "$vision_adapter" == espnet_frame_ablation || \
    "$vision_adapter" == espnet_v7_dronet_gap8 ) ]] && \
  espnet_dronet_compile_flag="-DTINYMPC_VISION_ESPNET_DRONET_ENABLE=1"
gate_position_compile_flag=""
if [[ "$vision_enabled" == 1 && "$vision_adapter" == espnet && \
      "$vision_scene" == corridor ]]; then
  gate_position_compile_flag="-DTINYMPC_GATE_POSITION_FUSION_ENABLE=1 -DTINYMPC_GATE_CENTER_BEARING_FUSION_ENABLE=1 -DTINYMPC_GATE_CAMERA_FOCAL_NORMALIZED=1.14531138f -DTINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED=0.5f -DTINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED=0.5f -DTINYMPC_GATE_WORLD_X_M=4.0f -DTINYMPC_GATE_WORLD_Y_M=0.0f -DTINYMPC_GATE_WORLD_Z_M=1.5f"
fi
gate_olgmd_compile_flag=""
if [[ "$vision_enabled" == 1 && (
      "$vision_adapter" == gate_frontnet_olgmd ||
      "$vision_adapter" == gate_frontnet_olgmd_obstacle_only ) ]]; then
  gate_olgmd_compile_flag="-DTINYMPC_GATE_OLGMD_ENABLE=1 -DTINYMPC_GATE_CAMERA_FOCAL_NORMALIZED=0.5572239953f -DTINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED=0.5068988158f -DTINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED=0.4584206439f"
fi
if [[ "$vision_enabled" == 1 &&
      "$vision_adapter" == gate_frontnet_olgmd_obstacle_only ]]; then
  gate_olgmd_compile_flag+=" -DTINYMPC_PAPER_EMERGENCY_DECELERATION_MPS2=10.0f -DTINYMPC_OLGMD_CLEAR_RESUME_ENABLE=1 -DTINYMPC_OLGMD_MOVING_SPEED_MPS=0.15f -DTINYMPC_OLGMD_STOP_HOLD_S=3.0f -DTINYMPC_OLGMD_RESUME_REFRACTORY_S=2.0f"
fi
course_compile_flag=""
if [[ "$course" == gate_obstacle_poc || \
      "$course" == gate_obstacle_poc_obstacle_only || \
      "$course" == gate_obstacle_easy_transition || \
      "$course" == gate_obstacle_easy_transition_obstacle_only || \
      "$course" == heldout_room_straight || \
      "$course" == heldout_room_circle || \
      "$course" == heldout_room_oval || \
      "$course" == heldout_room_figure8 ]]; then
  # The matched obstacle-only control keeps the visual-association code active
  # so false gates can be measured as a real regression. Every normal course
  # remains on the unchanged controller path.
  if [[ "$vision_adapter" == joint_residual_rl ]]; then
    course_compile_flag=""
  elif [[ "$vision_adapter" == joint_gate_rl ]]; then
    course_compile_flag="-DTINYMPC_JOINT_GATE_RL_ENABLE=1"
  else
    course_compile_flag="-DTINYMPC_GATE_OBSTACLE_POC_ENABLE=1"
  fi
fi
if [[ "$vision_scene" == imav22 && ( "$vision_adapter" == espnet_v7_dronet || \
      "$vision_adapter" == espnet_v7_dronet_gap8 ) ]]; then
  course_compile_flag="-DTINYMPC_GATE_OBSTACLE_POC_ENABLE=1 -DTINYMPC_GATE_CORNER_SPAN_M=0.494f"
fi
case "$course" in
  gate_obstacle_easy_transition|gate_obstacle_easy_transition_obstacle_only)
    course_compile_flag+=" -DTINYMPC_GATE_CORNER_SPAN_M=1.233333333f" ;;
  heldout_room_straight|heldout_room_circle|heldout_room_oval|heldout_room_figure8)
    course_compile_flag+=" -DTINYMPC_GATE_CORNER_SPAN_M=1.11f" ;;
esac
trajectory_compile_flag=""
[[ "$trajectory" == circle ]] && \
  trajectory_compile_flag="-DTINYMPC_TRAJECTORY_CIRCLE=1"
delay_ms="$(python3 -c 'import sys; print(round(float(sys.argv[1]) * 1000))' "$launch_time")"

build_lock_key="$(printf '%s' "$build" | sha256sum | cut -c1-16)"
build_lock="/workspace/tools/crazysim_mujoco/.build-${build_lock_key}.lock"
exec 9>"$build_lock"
flock 9
cmake -S "$firmware/sitl_make" -B "$build" \
  -DTINYMPC_APP_DIR="$app" \
  -DTINYMPC_SITL_SUPPORT_DIR="$support" \
  -DTINYMPC_TRAJECTORY="$trajectory" \
  -DTINYMPC_COURSE="$course_build" \
  -DTINYMPC_ACTUATOR_LTI="$actuator_lti_flag" \
  -DCMAKE_CXX_FLAGS="$cost_compile_flag $rate_identification_compile_flag $direct_plan_replay_compile_flag $residual_reference_compile_flag $direct_dronet_compile_flag $paper_ablation_compile_flag $paper_emergency_terminal_stop_compile_flag $reactive_reference_compile_flag $square_opening_compile_flag $dronet_v2_compile_flag $oracle_action_compile_flag $espnet_dronet_compile_flag $gate_position_compile_flag $gate_olgmd_compile_flag $course_compile_flag $trajectory_compile_flag -DTINYMPC_REACTIVE_POWER_LOOP_ENABLE=$reactive_power_loop -DTINYMPC_PITCH_THROUGH_BRAKE_ENABLE=$pitch_through_brake -DTINYMPC_BRAKING_CACHE_ENABLE=$braking_cache_enabled -DTINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE=$vertical_active_sensing -DTINYMPC_YAW_SPIN_TEST_ENABLE=$yaw_spin_test -DTINYMPC_YAW_SPIN_TEST_RATE_RAD_S=${yaw_spin_rate_rad_s}f -DTINYMPC_YAW_SPIN_TEST_REVOLUTIONS=${yaw_spin_revolutions}f -DTINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS=${yaw_spin_straight_speed_mps}f -DTINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S=${yaw_spin_straight_duration_s}f -DTINYMPC_RATE_CASCADE=$rate_cascade -DTINYMPC_PROGRESS_SAMPLE_LIMIT=$progress_sample_limit -DTINYMPC_PROGRESS_SPEED_MPS=$progress_speed_mps -DTINYMPC_PROGRESS_LAPS=$progress_laps -DTINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2=$progress_entry_acceleration_mps2 -DTINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2=$progress_terminal_deceleration_mps2 -DTINYMPC_PROGRESS_REWARD_WEIGHT=$progress_reward_weight -DTINYMPC_FLIP_ENABLE=$flip_enable -DTINYMPC_FLIP_TRIGGER_S_M=$flip_trigger_s_m -DTINYMPC_FLIP_TRIGGER_WINDOW_M=$flip_trigger_window_m -DTINYMPC_FLIP_DURATION_S=$flip_duration_s -DTINYMPC_FLIP_PITCH_DIRECTION=$flip_pitch_direction -DTINYMPC_POWER_LOOP_ENABLE=$power_loop_enable -DTINYMPC_POWER_LOOP_TRIGGER_S_M=$power_loop_trigger_s_m -DTINYMPC_POWER_LOOP_TRIGGER_WINDOW_M=$power_loop_trigger_window_m -DTINYMPC_POWER_LOOP_RADIUS_M=$power_loop_radius_m -DTINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS=$power_loop_bottom_speed_mps -DTINYMPC_POWER_LOOP_TOP_SPEED_MPS=$power_loop_top_speed_mps" \
  -DTINYMPC_SITL_START_DELAY_MS="$delay_ms" \
  -DTINYMPC_SITL_TICK_US="$tick_us" \
  >"$out/configure.log" 2>&1
cmake --build "$build" --target cf2 -j2 >"$out/build.log" 2>&1
flock -u 9
python3 - "$out/run_config.json" "$build/cf2" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

config_path = Path(sys.argv[1])
firmware_path = Path(sys.argv[2])
digest = hashlib.sha256()
with firmware_path.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
config = json.loads(config_path.read_text())
config["acceptance_contract"]["firmware_binary_sha256"] = digest.hexdigest()
config_path.write_text(json.dumps(config, indent=2) + "\n")
PY

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
if [[ -n "$imav22_mode" ]]; then
  scene=/workspace/tools/crazysim_mujoco/scenes/vision_imav22.xml
  sim_command+=(--scene "$scene" --imav22-mode "$imav22_mode"
    --imav22-relocation-period "$imav22_relocation_period_s"
    --imav22-relocation-clearance "$imav22_relocation_clearance_m"
    --imav22-relocation-duration "$imav22_relocation_duration_s"
    --imav22-object-set "$imav22_object_set"
    --imav22-event-log "$out/imav22_events.csv")
fi
if [[ "$vision_enabled" == 1 || "$camera_only" == 1 ]]; then
  scene=/workspace/tools/crazysim_mujoco/scenes/vision_${vision_scene}.xml
  camera_width=160; camera_height=120
  camera_fovy=""
  if [[ "$hm01b0_poc_camera" == 1 ]]; then
    camera_width=192; camera_height=192
    # 192/2 divided by the calibrated fy gives this source FOV.  It is under
    # one UDP datagram (36,864 bytes) and leaves a bounded overscan margin.
    camera_fovy=94.0387229678
  elif [[ "$vision_adapter" == tiny_dronet_v3_direct ||
          "$vision_adapter" == tinyvpc_square_opening ||
          "$vision_adapter" == tinyvpc_navigation ||
          "$vision_adapter" == tinyvpc_pid ||
          "$vision_adapter" == tinyvpc_emergency ||
          "$vision_adapter" == pulp_dronet_v3_pid ||
          "$vision_adapter" == pulp_dronet_v3_emergency ||
          "$vision_adapter" == pulp_dronet_v3_paper ||
          "$vision_adapter" == pulp_dronet_v3_straight_brake ||
          "$vision_adapter" == dronet_v2_paper ||
          "$vision_adapter" == dronet_v2_pid ||
          "$vision_adapter" == dronet_v2_emergency ]]; then
    # Match the 200x200 TinyVPC/PULP-DroNet standard from an upstream QVGA
    # capture (centered for TinyVPC/v3; center-bottom in the v2 adapter).
    camera_width=324; camera_height=244
    camera_fovy=83.6091095
  elif [[ "$vision_adapter" == dronet || "$vision_adapter" == hybrid_rl ||
        "$vision_adapter" == combined_gate_rl || "$vision_adapter" == joint_gate_rl ||
        "$vision_adapter" == joint_residual_rl ||
        "$vision_adapter" == espnet_v7_residual ||
        "$vision_adapter" == espnet_v7_dronet ||
        "$vision_adapter" == espnet_v7_pid ||
        "$vision_adapter" == espnet_frame_ablation ||
        "$vision_adapter" == espnet_v7_dronet_gap8 ||
        "$vision_adapter" == espnet_v7_reactive_gap8 ||
        "$vision_adapter" == espnet_v7_emergency ||
        "$vision_adapter" == espnet_v7_straight_brake ||
        "$vision_adapter" == espnet_v7_reactive ]]; then
    # Use one raw HM01B0/Isaac-calibrated camera stream for the baseline and
    # learned policy. DroNet is resized to its legacy 200x200 network input;
    # the RL policy consumes two consecutive raw 160x160 frames.
    camera_width=160; camera_height=160
    camera_fovy=83.6091095
  elif [[ "$camera_only" == 1 || "$vision_adapter" == espnet ]]; then
    # The August 19 two-frame release consumes the full HM01B0 image; unlike
    # the previous deployment it must not receive the 160x120 center crop.
    camera_width=160; camera_height=160
    camera_fovy=47.168554
  fi
  if [[ -n "$imav22_mode" ]]; then
    # Render an overscan pinhole source, then apply Isaac's measured K,
    # plumb-bob distortion, principal point, gray response, and seeded noise.
    camera_width=192; camera_height=192
    camera_fovy=94.0387229678
  fi
  sim_command+=(--camera --cam-width "$camera_width" --cam-height "$camera_height" --cam-fps "$camera_fps" --cam-port 5200)
  [[ -z "$camera_fovy" ]] || sim_command+=(--cam-fovy "$camera_fovy")
  [[ "$vision_scene" == none || -n "$imav22_mode" ]] || sim_command+=(--scene "$scene")
  if [[ "$camera_only" == 1 ]]; then
    imav22_camera_args=()
    [[ -z "$imav22_mode" ]] || imav22_camera_args=(--hm01b0-poc --sensor-seed "$random_seed")
    python3 -u /workspace/tools/crazysim_mujoco/vision_bridge.py \
      --camera-only --camera-port 5200 --camera-fps "$camera_fps" \
      "${imav22_camera_args[@]}" \
      --log "$out/camera.csv" --frames-dir "$out/camera_frames" \
      --camera-video "$out/fpv_camera.mp4" \
      >"$out/camera_bridge.log" 2>&1 &
    bridge_log="$out/camera_bridge.log"
  else
    passive_arg=()
    [[ "$vision_passive" != 1 ]] || passive_arg=(--passive)
    hm01b0_poc_args=()
    if [[ "$hm01b0_poc_camera" == 1 || -n "$imav22_mode" ]]; then
      hm01b0_poc_args=(--hm01b0-poc --sensor-seed "$random_seed")
    fi
    reveal_position_args=()
    [[ "$vision_control_enable_at_x_m" == none ]] || \
      reveal_position_args=(--control-enable-at-x-m "$vision_control_enable_at_x_m")
    python3 -u /workspace/tools/crazysim_mujoco/vision_bridge.py \
      --model "$vision_model" --adapter "$vision_adapter" \
      --target-speed-mps "$progress_speed_mps" \
      --control-enable-after-s "$vision_control_enable_after_s" \
      "${reveal_position_args[@]}" \
      --camera-port 5200 --camera-fps "$camera_fps" --firmware-port 19960 --log "$out/vision.csv" \
      --delivery-latency-frames "$vision_latency_frames" \
      "${hm01b0_poc_args[@]}" \
      "${passive_arg[@]}" \
      --frames-dir "$out/vision_frames" \
      --camera-video "$out/fpv_camera.mp4" \
      >"$out/vision_bridge.log" 2>&1 &
    bridge_log="$out/vision_bridge.log"
  fi
  vision_pid=$!
  vision_ready=0
  for _ in $(seq 1 100); do
    if grep -q "bridge ready" "$bridge_log" 2>/dev/null; then
      vision_ready=1
      break
    fi
    if ! kill -0 "$vision_pid" 2>/dev/null; then
      break
    fi
    sleep 0.05
  done
  if [[ "$vision_ready" != 1 ]]; then
    echo "Camera bridge did not start; see $bridge_log" >&2
    exit 1
  fi
fi
[[ "$mass" == stock ]] || sim_command+=(--mass "$mass")
[[ "$launch_prespin" == 0 ]] || sim_command+=(--launch-prespin)
for sim_arg in "$@"; do
  [[ "$sim_arg" == __none__ ]] || sim_command+=("$sim_arg")
done
if [[ "$trajectory" == imav22_circle ]]; then
  # The replica is centered at world (0,0).  The training route is local to
  # the arena entrance, so spawn at its west edge and keep the firmware path
  # local: its +3.6 m center then coincides with the replica's origin.  The
  # option terminator is required because the positional X coordinate is
  # negative, and this is deliberately appended after every simulator option.
  sim_command+=(-- "-3.6,0")
fi

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

actor_state_environment=()
[[ "$vision_adapter" != espnet_v7_residual ]] || \
  actor_state_environment=(TINYMPC_ACTOR_STATE_PORT=19961)
if [[ "$mpc_diag_mode" == taskless ]]; then
  env "${actor_state_environment[@]}" \
    TINYMPC_DIAG_MODE=taskless TINYMPC_DIAG_PATH="$diagnostic_path" \
    stdbuf -oL -eL "$build/cf2" 19950 >"$out/firmware.log" 2>&1 &
else
  if [[ ${#actor_state_environment[@]} -gt 0 ]]; then
    env "${actor_state_environment[@]}" stdbuf -oL -eL "$build/cf2" 19950 >"$out/firmware.log" 2>&1 &
  else
    stdbuf -oL -eL "$build/cf2" 19950 >"$out/firmware.log" 2>&1 &
  fi
fi
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
column_count = None
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
                        column_count = len(row)
                    continue
                # Ignore a partial row if the reader resumes while the simulator
                # is still appending it.  Otherwise shifted columns can make an
                # RPM value look like a contact.
                if len(row) != column_count:
                    continue
                if float(row[contact_index]) > 0.0:
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
# Terminate and wait for cf2 before the outer runner reads the MAP_SHARED
# diagnostic container.  No diagnostic drain delay is needed in taskless mode.
cleanup
firmware_pid=""

case "$trajectory" in
  straight) reference="$app/sim/trajectories/straight.csv" ;;
  straight_long) reference="$app/sim/trajectories/straight_long.csv" ;;
  straight_9m) reference="$app/sim/trajectories/straight_9m.csv" ;;
  straight_20m) reference="$app/sim/trajectories/straight_20m.csv" ;;
  chicane|hairpin_180) reference="$app/sim/trajectories/racing/${trajectory}.csv" ;;
  *) reference="" ;;
esac
reference_args=()
[[ -z "$reference" || -f "$reference" ]] || reference=""
[[ -z "$reference" ]] || reference_args=(--reference "$reference")
vision_args=()
if [[ "$vision_enabled" != 0 ]]; then
  analysis_scene_kind="$vision_scene"
  [[ -z "$course" ]] || analysis_scene_kind=none
  case "$analysis_scene_kind" in
    none|obstacle|gate) ;;
    *) analysis_scene_kind=none ;;
  esac
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
PY
CONTAINER_SCRIPT
simulation_status=$?
set -e

if [[ "$MPC_DIAG_MODE" == taskless ]]; then
  python3 - "$OUT/run_config.json" "$OUT/mpc_diag.mmap" "$OUT/mpc_diag.bin" <<'PY'
import hashlib
import json
from pathlib import Path
import struct
import sys

config_path = Path(sys.argv[1])
container_path = Path(sys.argv[2])
compact_path = Path(sys.argv[3])
config = json.loads(config_path.read_text())
diagnostic = config["mpc_diagnostic"]
container = diagnostic["container"]

MMAP_MAGIC = 0x544D444D
MMAP_VERSION = 1
HEADER_SIZE = 4096
SLOT_SIZE = 320
RECORD_SIZE = 296
CAPACITY = 16384
FILE_SIZE = 5246976
PAYLOAD_MAGIC = 0x544D5043
PAYLOAD_VERSION = 2
FNV_OFFSET_BASIS = 2166136261
FNV_PRIME = 16777619

def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

def fnv1a32(payload):
    value = FNV_OFFSET_BASIS
    for byte in payload:
        value ^= byte
        value = (value * FNV_PRIME) & 0xFFFFFFFF
    return value

validation = {
    "safe_to_extract": False,
    "errors": [],
    "header": None,
    "reserved_count": None,
    "valid_committed_slots": 0,
    "extracted_records": 0,
}
if container_path.is_file():
    container["size_bytes"] = container_path.stat().st_size
    container["sha256"] = sha256(container_path)
    raw = container_path.read_bytes()
    if len(raw) != FILE_SIZE:
        validation["errors"].append(
            f"container_size={len(raw)},expected={FILE_SIZE}")
    else:
        (magic, version, header_size, slot_size, record_size, capacity,
         firmware_tick_us, payload_magic, payload_version, reserved0) = \
            struct.unpack_from("<IHHIIIIIHH", raw, 0)
        next_sequence, overflow_count, committed_count = struct.unpack_from(
            "<QQQ", raw, 64)
        header = {
            "magic": f"0x{magic:08x}",
            "version": version,
            "header_size_bytes": header_size,
            "slot_size_bytes": slot_size,
            "record_size_bytes": record_size,
            "capacity": capacity,
            "firmware_tick_us": firmware_tick_us,
            "payload_magic": f"0x{payload_magic:08x}",
            "payload_version": payload_version,
            "next_sequence": next_sequence,
            "overflow_count": overflow_count,
            "committed_count": committed_count,
        }
        validation["header"] = header
        expected_header = (
            ("magic", magic, MMAP_MAGIC),
            ("version", version, MMAP_VERSION),
            ("header_size", header_size, HEADER_SIZE),
            ("slot_size", slot_size, SLOT_SIZE),
            ("record_size", record_size, RECORD_SIZE),
            ("capacity", capacity, CAPACITY),
            ("firmware_tick_us", firmware_tick_us,
             int(diagnostic["firmware_tick_us"])),
            ("payload_magic", payload_magic, PAYLOAD_MAGIC),
            ("payload_version", payload_version, PAYLOAD_VERSION),
            ("reserved0", reserved0, 0),
        )
        for name, actual, expected in expected_header:
            if actual != expected:
                validation["errors"].append(
                    f"header_{name}={actual},expected={expected}")
        if any(raw[32:64]) or any(raw[88:HEADER_SIZE]):
            validation["errors"].append("header_reserved_bytes_nonzero")
        reserved_count = next_sequence - 1 if next_sequence >= 1 else None
        validation["reserved_count"] = reserved_count
        if reserved_count is None:
            validation["errors"].append("header_next_sequence_is_zero")
            reserved_in_capacity = 0
        else:
            reserved_in_capacity = min(reserved_count, CAPACITY)
            expected_overflow = max(0, reserved_count - CAPACITY)
            if overflow_count != expected_overflow:
                validation["errors"].append(
                    f"overflow_count={overflow_count},expected={expected_overflow}")
            if overflow_count != 0:
                validation["errors"].append(
                    f"diagnostic_capacity_overflow={overflow_count}")

        payloads = []
        for index in range(CAPACITY):
            slot_offset = HEADER_SIZE + index * SLOT_SIZE
            commit_sequence, reservation_sequence = struct.unpack_from(
                "<QQ", raw, slot_offset)
            payload = raw[slot_offset + 16:slot_offset + 312]
            checksum, slot_reserved = struct.unpack_from(
                "<II", raw, slot_offset + 312)
            expected_sequence = index + 1
            if index >= reserved_in_capacity:
                if any(raw[slot_offset:slot_offset + SLOT_SIZE]):
                    validation["errors"].append(
                        f"slot[{index}].nonzero_after_reserved_range")
                continue
            slot_errors = []
            if commit_sequence != expected_sequence:
                slot_errors.append(
                    f"commit={commit_sequence},expected={expected_sequence}")
            if reservation_sequence != expected_sequence:
                slot_errors.append(
                    f"reservation={reservation_sequence},expected={expected_sequence}")
            if slot_reserved != 0:
                slot_errors.append(f"reserved={slot_reserved}")
            calculated_checksum = fnv1a32(
                raw[slot_offset + 8:slot_offset + 312])
            if checksum != calculated_checksum:
                slot_errors.append(
                    f"checksum=0x{checksum:08x},expected=0x{calculated_checksum:08x}")
            (record_magic, record_version, declared_record_size, event,
             event_sequence) = struct.unpack_from("<IHHII", payload, 0)
            if record_magic != PAYLOAD_MAGIC:
                slot_errors.append(f"payload_magic=0x{record_magic:08x}")
            if record_version != PAYLOAD_VERSION:
                slot_errors.append(f"payload_version={record_version}")
            if declared_record_size != RECORD_SIZE:
                slot_errors.append(f"payload_record_size={declared_record_size}")
            if event_sequence != expected_sequence:
                slot_errors.append(
                    f"payload_event_sequence={event_sequence},expected={expected_sequence}")
            if index == 0 and event != 0:
                slot_errors.append(f"first_payload_event={event},expected=0")
            if slot_errors:
                validation["errors"].append(
                    f"slot[{index}]:" + ";".join(slot_errors))
            else:
                validation["valid_committed_slots"] += 1
                payloads.append(payload)
        if committed_count != validation["valid_committed_slots"]:
            validation["errors"].append(
                f"committed_count={committed_count},"
                f"valid_slots={validation['valid_committed_slots']}")
        if validation["valid_committed_slots"] != reserved_in_capacity:
            validation["errors"].append(
                f"reserved_slots={reserved_in_capacity},"
                f"valid_slots={validation['valid_committed_slots']}")
        validation["safe_to_extract"] = not validation["errors"]
        if validation["safe_to_extract"]:
            with compact_path.open("xb") as stream:
                for payload in payloads:
                    stream.write(payload)
            diagnostic["size_bytes"] = compact_path.stat().st_size
            diagnostic["sha256"] = sha256(compact_path)
            validation["extracted_records"] = len(payloads)
else:
    validation["errors"].append("container_missing")
container["validation"] = validation
config_path.write_text(json.dumps(config, indent=2) + "\n")
PY
fi

if [[ "$RENDER_VIDEO" == 1 && -f "$OUT/state.csv" ]]; then
  video_args=(--csv "/workspace/$repo_rel_out/state.csv" \
    --out "/workspace/$repo_rel_out/flight.mp4" \
    --fps "$VIDEO_FPS" --playback-speed "$VIDEO_SPEED" --launch-time "$LAUNCH_TIME")
  [[ -z "$COURSE_MANIFEST" ]] || \
    video_args+=(--course "/workspace/${COURSE_MANIFEST#"$REPO_DIR"/}")
  # Reuse the selected simulator container engine. Calling Docker here made an
  # otherwise valid Apptainer/Slurm flight fail only during post-processing.
  "${container_command[@]}" \
    python3 tools/crazysim_mujoco/render_flight_video.py "${video_args[@]}"
fi

echo "Results: $OUT"
exit "$simulation_status"
