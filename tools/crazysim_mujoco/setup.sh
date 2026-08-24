#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="$SCRIPT_DIR/.deps"
CRAZYSIM_DIR="$DEPS_DIR/CrazySim"
CRAZYSIM_URL="https://github.com/gtfactslab/CrazySim.git"
CRAZYSIM_COMMIT="3ec8b55da4bff887da542a9f314da825460e65be"

apply_once() {
  local repo="$1"
  local patch="$2"
  if git -C "$repo" apply --reverse --check "$patch" >/dev/null 2>&1; then
    return
  fi
  if ! git -C "$repo" apply --check "$patch"; then
    echo "Cannot apply $patch. The pinned dependency has unexpected local changes." >&2
    exit 1
  fi
  git -C "$repo" apply "$patch"
}

mkdir -p "$DEPS_DIR"
if [[ ! -d "$CRAZYSIM_DIR/.git" ]]; then
  git clone "$CRAZYSIM_URL" "$CRAZYSIM_DIR"
  git -C "$CRAZYSIM_DIR" checkout --detach "$CRAZYSIM_COMMIT"
fi

actual_commit="$(git -C "$CRAZYSIM_DIR" rev-parse HEAD)"
if [[ "$actual_commit" != "$CRAZYSIM_COMMIT" ]]; then
  echo "CrazySim dependency is at $actual_commit, expected $CRAZYSIM_COMMIT." >&2
  echo "Remove $CRAZYSIM_DIR and rerun setup to recreate the pinned dependency." >&2
  exit 1
fi

git -C "$CRAZYSIM_DIR" submodule update --init crazyflie-firmware
git -C "$CRAZYSIM_DIR/crazyflie-firmware" submodule update --init \
  vendor/FreeRTOS tools/crazyflie-simulation
git -C "$CRAZYSIM_DIR/crazyflie-firmware/tools/crazyflie-simulation" \
  submodule update --init simulator_files/mujoco/drone-models
apply_once "$CRAZYSIM_DIR/crazyflie-firmware" \
  "$SCRIPT_DIR/patches/crazysim-firmware-tinympc.patch"
apply_once "$CRAZYSIM_DIR/crazyflie-firmware/vendor/FreeRTOS" \
  "$SCRIPT_DIR/patches/freertos-posix-sim-time.patch"
apply_once "$CRAZYSIM_DIR/crazyflie-firmware/tools/crazyflie-simulation" \
  "$SCRIPT_DIR/patches/crazysim-mujoco-logging.patch"
apply_once "$CRAZYSIM_DIR/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/drone-models" \
  "$SCRIPT_DIR/patches/crazysim-drone-models-hm01b0-camera.patch"

echo "CrazySim is ready at $CRAZYSIM_DIR"
