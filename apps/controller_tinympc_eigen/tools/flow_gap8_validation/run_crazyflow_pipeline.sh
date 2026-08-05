#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
exec "$ROOT/.venv-crazyflow/bin/python" \
  "$ROOT/apps/controller_tinympc_eigen/tools/flow_gap8_validation/sim_crazyflow_racing_pipeline.py" "$@"
