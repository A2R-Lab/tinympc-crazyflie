#!/usr/bin/env bash
# Run on the flashing VM; USB permissions and cflib come from the existing image.
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm --init --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$project_dir":/workspace -w /workspace \
  -e PYTHONPATH=/workspace/releases/depthgate_thin_20260907/stm32_logging/python310 \
  registry.gitlab.com/eliacereda/gapsdk:22.04-3.8.1 \
  python3 /workspace/tools/run_depthgate_flight.py "$@"
