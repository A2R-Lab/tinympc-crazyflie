#!/usr/bin/env bash
#SBATCH --job-name=joint_dory_gvsoc_stream
#SBATCH --partition=debug
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=02:00:00
#SBATCH --output=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/models/vision_rl_gate_joint/gap8/slurm_logs/%x-%j.out
#SBATCH --error=/home/cchen/tinympc-crazyflie/tools/crazysim_mujoco/models/vision_rl_gate_joint/gap8/slurm_logs/%x-%j.err
# Reproducible CPU-only NEMO/DORY conversion plus GVSOC-only validation.
# This script never targets a physical board or authorizes hardware flashing.
set -euo pipefail
ROOT=/home/cchen/tinympc-crazyflie
GAP8="$ROOT/tools/crazysim_mujoco/models/vision_rl_gate_joint/gap8"
CANDIDATE_DIR=${CANDIDATE_DIR:?set accepted candidate bundle directory}
MODEL_BUNDLE=${MODEL_BUNDLE:-$CANDIDATE_DIR/bundle.json}
CHECKPOINT=${CHECKPOINT:-$CANDIDATE_DIR/checkpoint.pt}
CALIBRATION_SOURCE=${CALIBRATION_SOURCE:?set training-excluded calibration test.npz}
CHECKPOINT_SHA256=${CHECKPOINT_SHA256:?set accepted checkpoint SHA-256}
CALIBRATION_SOURCE_SHA256=${CALIBRATION_SOURCE_SHA256:?set calibration source SHA-256}
OUTPUT_ROOT=${OUTPUT_ROOT:-$GAP8/runs}
GAP_TOOLCHAIN=${GAP_TOOLCHAIN:-/home/cchen/gap_riscv_toolchain_ubuntu_18}
RUN_ROOT="$OUTPUT_ROOT/${SLURM_JOB_ID}"
test -f "$CHECKPOINT"
test -f "$MODEL_BUNDLE"
test -f "$CALIBRATION_SOURCE"
test "$(sha256sum "$CALIBRATION_SOURCE" | awk '{print $1}')" = "$CALIBRATION_SOURCE_SHA256"
mkdir -p "$RUN_ROOT" "$GAP8/slurm_logs"
trap 'status=$?; printf "%s\n" "$status" > "$RUN_ROOT/exit_status.txt"; date --iso-8601=seconds > "$RUN_ROOT/finished_at.txt"' EXIT
export PYTHONPATH="$ROOT"
/home/cchen/isaacsim-env/bin/python -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.validate_candidate_bundle \
  --bundle "$MODEL_BUNDLE" --checkpoint "$CHECKPOINT" \
  --expected-checkpoint-sha256 "$CHECKPOINT_SHA256" \
  --output "$RUN_ROOT/model_provenance.json"
{ date --iso-8601=seconds; hostname; command -v python3; python3 --version; git -C "$ROOT" rev-parse HEAD; git -C "$ROOT" status --short; } > "$RUN_ROOT/metadata.txt"
/home/cchen/isaacsim-env/bin/python -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.export_joint_dory \
  --checkpoint "$CHECKPOINT" --calibration-source "$CALIBRATION_SOURCE" --output "$RUN_ROOT/export" \
  --terminal-scale "${TERMINAL_SCALE:-0.03125}" --terminal-margin "${TERMINAL_MARGIN:-0.000001}"
CUDA_VISIBLE_DEVICES='' PYTHONPATH="$ROOT" /home/cchen/miniconda3/envs/frontnet/bin/python \
  -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.nemo_joint_integer_export \
  --bridge "$RUN_ROOT/export/nemo_bridge_state.npz" --float-export "$RUN_ROOT/export" \
  --output "$RUN_ROOT/integer"
PYTHONPATH="$ROOT:/home/cchen/dory:/home/cchen/miniconda3/envs/dory/lib/python3.7/site-packages" \
  /home/cchen/miniconda3/envs/dory/bin/python "$GAP8/validate_joint_dory_frontend.py" \
  --onnx "$RUN_ROOT/integer/joint_dory_packed_int.onnx" --output "$RUN_ROOT/dory_frontend.json" \
  --app-dir "$RUN_ROOT/gap8_application"
/home/cchen/isaacsim-env/bin/python -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.instrument_gvsoc_harness \
  --app-dir "$RUN_ROOT/gap8_application" --corpus "$RUN_ROOT/export/calibration_200_clips.npz"
set +u
source /home/cchen/gap_sdk_dory/configs/gap8_v2.sh
set -u
export PATH="/home/cchen/miniconda3/envs/dory/bin:$PATH"
export PATH="$GAP_TOOLCHAIN/bin:$PATH"
export GAP_RISCV_GCC_TOOLCHAIN="$GAP_TOOLCHAIN"
export GAP_RISCV_GCC_TOOLCHAIN_BASE=""
{ command -v python3; python3 --version; python3 -c 'import argcomplete; print(argcomplete.__file__)';
  command -v riscv32-unknown-elf-gcc; riscv32-unknown-elf-gcc --version | head -n 1;
  command -v gapy; gapy --help >/dev/null; } >> "$RUN_ROOT/metadata.txt"
cd "$RUN_ROOT/gap8_application"
make clean
# `platform=gvsoc` runs the generated packed 12-value model in the simulator.
# The harness streams 200 ordered READFS clips through one 51,200-byte staging
# allocation; the complete corpus is never placed in HyperRAM at once.
# The copied PULP-NN kernels partition rows with log2(NUM_CORES) and
# NUM_CORES-1 bit masks, so NUM_CORES must be a power of two. GAP8 exposes all
# eight cluster PEs; CORE=7 silently overlaps/skips work and corrupts outputs.
make all run platform=gvsoc CORE=8 2>&1 | tee "$RUN_ROOT/gvsoc.log"
grep -q "JOINT_GVSOC 199 " "$RUN_ROOT/gvsoc.log"
TERMINAL_EPSILON=$( /home/cchen/isaacsim-env/bin/python -c 'import json,sys; print(json.load(open(sys.argv[1]))["terminal_epsilon"])' "$RUN_ROOT/integer/nemo_joint_report.json" )
/home/cchen/isaacsim-env/bin/python -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.parse_gvsoc_harness \
  --log "$RUN_ROOT/gvsoc.log" --terminal-epsilon "$TERMINAL_EPSILON" \
  --app-dir "$RUN_ROOT/gap8_application" --output "$RUN_ROOT/gvsoc_200_outputs.npz" --memory-output "$RUN_ROOT/memory.json"
/home/cchen/isaacsim-env/bin/python -m tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8.check_gvsoc_acceptance \
  --gvsoc-corpus "$RUN_ROOT/gvsoc_200_outputs.npz" \
  --semantic-corpus "$RUN_ROOT/export/calibration_200_clips.npz" \
  --quantized-contract "$RUN_ROOT/integer/quantized_deployment_contract.json" \
  --memory-report "$RUN_ROOT/memory.json" --model-provenance "$RUN_ROOT/model_provenance.json" \
  --clock-hz "${GAP8_CLOCK_HZ:-100000000}" \
  --output "$RUN_ROOT/gvsoc_acceptance.json"
