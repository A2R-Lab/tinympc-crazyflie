#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runner="$script_dir/run_joint_gate_obstacle_matrix.sh"

bash -n "$runner"
"$runner" --help | grep -Fq 'exactly ten sequential, paired held-out seeds'
"$runner" --help | grep -Fq 'refuses an existing output root'
! rg -q -- '--video' "$runner"
rg -q -- 'candidate_gate_obstacle.*gate_obstacle_easy_transition' "$runner"
rg -q -- 'candidate_obstacle_only.*gate_obstacle_easy_transition_obstacle_only' "$runner"
rg -q -- 'baseline_obstacle_only.*gate_obstacle_easy_transition_obstacle_only' "$runner"
rg -q -- 'exit "\$result"' "$runner"
rg -Fq -- 'acceptance_contract.vision_bridge_sha256' "$runner"
rg -Fq -- 'acceptance_contract.analyzer_sha256' "$runner"
rg -Fq -- 'provenance/matrix_tools' "$runner"
rg -Fq -- '--matrix-runner-snapshot "$MATRIX_RUNNER_SNAPSHOT"' "$runner"
rg -Fq -- '--evaluator-snapshot "$EVALUATOR_SNAPSHOT"' "$runner"
rg -Fq -- 'cmp -s "$MATRIX_RUNNER" "$MATRIX_RUNNER_SNAPSHOT"' "$runner"
rg -Fq -- 'cmp -s "$EVALUATOR" "$EVALUATOR_SNAPSHOT"' "$runner"
rg -Fq -- '${3:?pass an explicit fresh seed start}' "$script_dir/joint_gate_obstacle_cpu.sbatch"
