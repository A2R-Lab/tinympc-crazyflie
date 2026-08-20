#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
IMAGE="tinympc-crazysim:local"
TRAJECTORY="backflip_360"
STORED_LTV=1
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
EXTRA_SIM_ARGS=(__none__)

usage() {
  cat <<'EOF'
Usage: tools/crazysim_mujoco/run.sh [options]
  --trajectory NAME       backflip_360, front_flip_360, roll_flip_360,
                          or barrel_roll_forward_360
  --stored-ltv 0|1        Use horizon-wise stored matrices (default: 1)
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
  --sensor-noise          Enable CrazySim IMU/barometer noise
  --flowdeck              Use simulated Flow deck instead of pose
  --ground-effect         Enable ground effect
  --wind-speed MPS        Add constant wind
  --turbulence LEVEL      none, light, moderate, or severe
  --overwrite             Replace an existing output directory
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --trajectory) TRAJECTORY="$2"; shift 2 ;;
    --stored-ltv) STORED_LTV="$2"; shift 2 ;;
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
    --sensor-noise|--flowdeck|--ground-effect)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1"); shift ;;
    --wind-speed|--turbulence)
      [[ "${EXTRA_SIM_ARGS[0]}" == __none__ ]] && EXTRA_SIM_ARGS=()
      EXTRA_SIM_ARGS+=("$1" "$2"); shift 2 ;;
    --overwrite) OVERWRITE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$TRAJECTORY" in
  backflip_360|front_flip_360|roll_flip_360|barrel_roll_forward_360) ;;
  *) echo "Unsupported trajectory: $TRAJECTORY" >&2; exit 2 ;;
esac
if [[ "$STORED_LTV" != 0 && "$STORED_LTV" != 1 ]]; then
  echo "--stored-ltv must be 0 or 1" >&2
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
  OUT="$REPO_DIR/apps/controller_tinympc_eigen/sim_runs/crazysim/${TRAJECTORY}_${MODE}"
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

"$SCRIPT_DIR/setup.sh"
docker build -q -t "$IMAGE" "$SCRIPT_DIR" >/dev/null

repo_rel_out="${OUT#"$REPO_DIR"/}"

docker run --rm --interactive \
  --volume "$REPO_DIR:/workspace" \
  --workdir /workspace \
  "$IMAGE" \
  bash -s -- \
    "$TRAJECTORY" "$STORED_LTV" "$DURATION" "$LAUNCH_TIME" "$SPAWN_Z" \
    "$MODEL" "$MASS" "$PWM_THRUST_FULL" "/workspace/$repo_rel_out" \
    "$LAUNCH_PRESPIN" "$RANDOM_SEED" "$INERTIA_SCALE" \
    "$MOTOR_TAU_SCALE" "$THRUST_SCALE" "$TICK_US" \
    --realtime-factor "$REALTIME_FACTOR" \
    "${EXTRA_SIM_ARGS[@]}" <<'CONTAINER_SCRIPT'
set -euo pipefail
trajectory="$1"; stored_ltv="$2"; duration="$3"; launch_time="$4"
spawn_z="$5"; model="$6"; mass="$7"; pwm_thrust_full="$8"
out="$9"; launch_prespin="${10}"; random_seed="${11}"
inertia_scale="${12}"; motor_tau_scale="${13}"; thrust_scale="${14}"
tick_us="${15}"
shift 15

crazysim=/workspace/tools/crazysim_mujoco/.deps/CrazySim
firmware="$crazysim/crazyflie-firmware"
simulator="$firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py"
app=/workspace/apps/controller_tinympc_eigen
support=/workspace/tools/crazysim_mujoco/sitl
build="$firmware/sitl_make/build-tinympc-${trajectory}-${stored_ltv}"
ltv_flag=OFF
[[ "$stored_ltv" == 1 ]] && ltv_flag=ON
delay_ms="$(python3 -c 'import sys; print(round(float(sys.argv[1]) * 1000))' "$launch_time")"

cmake -S "$firmware/sitl_make" -B "$build" \
  -DTINYMPC_APP_DIR="$app" \
  -DTINYMPC_SITL_SUPPORT_DIR="$support" \
  -DTINYMPC_TRAJECTORY="$trajectory" \
  -DTINYMPC_STORED_LTV="$ltv_flag" \
  -DTINYMPC_SITL_START_DELAY_MS="$delay_ms" \
  -DTINYMPC_SITL_TICK_US="$tick_us" \
  >"$out/configure.log" 2>&1
cmake --build "$build" --target cf2 -j2 >"$out/build.log" 2>&1

cleanup() {
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
wait "$simulator_pid"
simulator_pid=""
cleanup
firmware_pid=""

reference="$app/sim/trajectories/acrobatics/${trajectory}.csv"
python3 /workspace/tools/crazysim_mujoco/analyze_run.py \
  --csv "$out/state.csv" \
  --reference "$reference" \
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

echo "Results: $OUT"
