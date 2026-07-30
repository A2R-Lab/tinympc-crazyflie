#include "tinympc/tinympc.h"

#include <cmath>
#include <cstdio>

int main() {
  tiny_Model model = {};
  model.nhorizon = NHORIZON;

  tiny_AdmmSettings settings = {};
  settings.en_cstr_states = 1;

  tiny_AdmmData data = {};
  data.model = &model;

  Eigen::VectorNf states[NHORIZON];
  Eigen::VectorNf state_duals[NHORIZON];
  Eigen::VectorNf projected_states[NHORIZON];
  for (int k = 0; k < NHORIZON; ++k) {
    states[k].setZero();
    state_duals[k].setZero();
    projected_states[k].setZero();
  }

  tiny_AdmmSolution solution = {};
  solution.X = states;
  solution.YX = state_duals;

  tiny_AdmmWorkspace workspace = {};
  workspace.data = &data;
  workspace.stgs = &settings;
  workspace.soln = &solution;
  workspace.ZX_new = projected_states;

  const int knot = 4;
  states[knot](0) = 1.0f;
  states[knot](1) = -0.2f;
  const Eigen::Vector3f normal(1.0f, 0.0f, 0.0f);
  const float boundary = 0.25f;
  if (tiny_SetPositionHalfspace(
          &workspace, knot, 0, &normal, boundary, 1) != TINY_NO_ERROR) {
    std::fprintf(stderr, "could not install half-space\n");
    return 1;
  }
  if (UpdateSlackDual(&workspace) != TINY_NO_ERROR) {
    std::fprintf(stderr, "ADMM projection failed\n");
    return 1;
  }

  const float projected_violation =
      normal.dot(projected_states[knot].head(3)) - boundary;
  if (std::fabs(projected_violation) > 1.0e-6f) {
    std::fprintf(stderr, "hard plane violation %.9f\n",
                 (double)projected_violation);
    return 1;
  }
  if (std::fabs(projected_states[knot](1) + 0.2f) > 1.0e-6f) {
    std::fprintf(stderr, "projection changed a tangential coordinate\n");
    return 1;
  }
  if (data.slack_used_hs[knot][0] != 0.0f) {
    std::fprintf(stderr, "position half-space unexpectedly used slack\n");
    return 1;
  }

  // Mirror the firmware's first-step guard: unfinished primal states beyond
  // the tolerance must be replaced by the hard-projected ADMM state.
  const float max_raw_violation = 0.01f;
  const Eigen::VectorNf& executed_state =
      normal.dot(states[knot].head(3)) - boundary <= max_raw_violation
          ? states[knot]
          : projected_states[knot];
  const float executed_violation =
      normal.dot(executed_state.head(3)) - boundary;
  if (executed_violation > 1.0e-6f) {
    std::fprintf(stderr, "guard emitted infeasible state %.9f\n",
                 (double)executed_violation);
    return 1;
  }

  std::printf(
      "hard half-space projection and first-step guard passed: "
      "violation=%.9f\n",
      (double)executed_violation);
  return 0;
}
