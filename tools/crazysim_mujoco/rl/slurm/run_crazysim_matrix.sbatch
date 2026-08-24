#!/bin/bash
#SBATCH --job-name=tinympc-rl-runs
#SBATCH --partition=debug
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=slurm-%j.out

set -euo pipefail

: "${PURPOSE:?Set PURPOSE to calibration, training, baseline, or hybrid}"
: "${OUTPUT_ROOT:?Set OUTPUT_ROOT to a fresh retained-artifact directory}"
: "${FIRMWARE_TIME_FACTOR:?Set FIRMWARE_TIME_FACTOR from the shared calibration}"

REPOSITORY_DIR="${REPOSITORY_DIR:-/home/cchen/tinympc-crazyflie}"
HYBRID_MODEL="${HYBRID_MODEL:-$REPOSITORY_DIR/tools/crazysim_mujoco/models/vision_rl_mpc_poc/policy.onnx}"

case "$PURPOSE" in
  calibration)
    seeds=(11)
    duration=6
    model=(--vision-model dronet-v3)
    ;;
  training)
    seeds=(11 22 33)
    duration=35
    model=(--vision-model dronet-v3)
    ;;
  baseline)
    seeds=(101 202 303)
    duration=35
    model=(--vision-model dronet-v3)
    ;;
  hybrid)
    seeds=(101 202 303)
    duration=35
    model=(--vision-model "$HYBRID_MODEL" --vision-adapter hybrid_rl)
    ;;
  *)
    echo "Unsupported PURPOSE=$PURPOSE" >&2
    exit 2
    ;;
esac

echo "job_id=$SLURM_JOB_ID node=$(hostname) purpose=$PURPOSE"
echo "repository_commit=$(git -C "$REPOSITORY_DIR" rev-parse HEAD)"
echo "firmware_time_factor=$FIRMWARE_TIME_FACTOR"

for seed in "${seeds[@]}"; do
  output="$OUTPUT_ROOT/seed_$seed"
  "$REPOSITORY_DIR/tools/crazysim_mujoco/run.sh" \
    --trajectory dronet_u \
    --course dronet_u \
    --duration "$duration" \
    --launch-time 1 \
    --spawn-z 0.5 \
    --actuator-lti 1 \
    --rate-cascade 1 \
    --progress-speed-mps 0.5 \
    --progress-reward-weight 0.0 \
    --launch-prespin 1 \
    --vision-latency-frames 1 \
    --no-flowdeck \
    --sensor-noise \
    --turbulence light \
    --realtime-factor 0.2 \
    --firmware-time-factor "$FIRMWARE_TIME_FACTOR" \
    --random-seed "$seed" \
    --stop-on-contact \
    "${model[@]}" \
    --out "$output"
done
