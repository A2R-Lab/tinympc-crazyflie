#include "tinympc_cf_adapter.hpp"
#include "tinympc/psd_support.hpp"

// You need the generated solver object.
// Typically codegen provides something like:
//   extern TinySolver tiny_solver;
// in "tinympc/tiny_data.hpp".
//
// And the solve entrypoint:
//   int tiny_solve(TinySolver* solver);

static inline TinySolver* cf_solver() {
  return &tiny_solver;
}

void tinympc_cf_init_defaults(void) {
  // Set defaults that match your old embedded behavior.
  // Old controller used ~2 ADMM iters and disabled termination checks.
  tinympc_cf_set_settings(2, 0);
}

void tinympc_cf_set_settings(int max_iter, int check_termination) {
  TinySolver* s = cf_solver();

  s->settings->max_iter = max_iter;
  s->settings->check_termination = check_termination;
}

void tinympc_cf_set_x0(const float* x0, int nstates) {
  TinySolver* s = cf_solver();

  for (int i = 0; i < nstates; ++i) {
    s->work->x(i, 0) = (tinytype)x0[i];
  }
}

void tinympc_cf_set_reference(const float* Xref, int nstates, int nhorizon,
                              const float* Uref, int ninputs) {
  TinySolver* s = cf_solver();

  // Layout assumption:
  // Xref is packed as [k0_state0..stateN-1, k1_state0.., ...] (time-major)
  // Uref is packed as [k0_input0..inputM-1, k1_input0.., ...] for nhorizon-1
  //
  // Change indexing if your arrays are state-major instead of time-major.
  for (int k = 0; k < nhorizon; ++k) {
    for (int i = 0; i < nstates; ++i) {
      const float v = Xref[k * nstates + i];
      s->work->Xref(i, k) = (tinytype)v;
    }
  }

  for (int k = 0; k < nhorizon - 1; ++k) {
    for (int j = 0; j < ninputs; ++j) {
      const float v = Uref[k * ninputs + j];
      s->work->Uref(j, k) = (tinytype)v;
    }
  }

  tinympc::update_linear_cost(s);
}

int tinympc_cf_solve(void) {
  TinySolver* s = cf_solver();
  return tiny_solve(s);
}

void tinympc_cf_get_x_pred_1(float* out_x1, int nstates) {
  TinySolver* s = cf_solver();

  // Predicted x at k=1. Assumes col-major time indexing (col = timestep).
  for (int i = 0; i < nstates; ++i) {
    out_x1[i] = (float)s->work->x(i, 1);
  }
}

void tinympc_cf_get_x_pred_0(float* out_x0, int nstates) {
  TinySolver* s = cf_solver();
  if (!s || !out_x0) {
    return;
  }
  for (int i = 0; i < nstates; ++i) {
    out_x0[i] = (float)s->solution->x(i, 0);
  }
}

void tinympc_cf_get_solution_base_states(float* out_states, int nx0, int horizon) {
  TinySolver* s = cf_solver();
  if (!s || !out_states) {
    return;
  }
  const int N = s->work->N;
  const int stages = (horizon > 0 && horizon < N) ? horizon : N;
  for (int k = 0; k < stages; ++k) {
    for (int i = 0; i < nx0; ++i) {
      out_states[k * nx0 + i] = (float)s->solution->x(i, k);
    }
  }
}

void tinympc_cf_get_solution_base_inputs(float* out_inputs, int nu0, int horizon) {
  TinySolver* s = cf_solver();
  if (!s || !out_inputs) {
    return;
  }
  const int N = s->work->N;
  const int stages = (horizon > 1 && horizon < N) ? horizon : N;
  for (int k = 0; k < stages - 1; ++k) {
    for (int j = 0; j < nu0; ++j) {
      out_inputs[k * nu0 + j] = (float)s->solution->u(j, k);
    }
  }
}

void tinympc_cf_get_u0(float* out_u0, int ninputs, TinympcControlOutputKind kind) {
  TinySolver* s = cf_solver();

  for (int j = 0; j < ninputs; ++j) {
    // Choose which trajectory to expose for the old controller's "ZU_new[0]".
    // If your generated work has znew and u, this mapping is straightforward.
    if (kind == TinympcControlOutputKind::ZNEW) {
      out_u0[j] = (float)s->work->znew(j, 0);
    } else {
      out_u0[j] = (float)s->work->u(j, 0);
    }
  }
}

// ========== PSD (Obstacle Avoidance) Implementation ==========

int tinympc_cf_enable_psd(int nx0, int nu0, float rho_psd) {
  TinySolver* s = cf_solver();
  if (!s) return -1;

  const int nx_req = nx0 + nx0 * nx0;
  const int nu_req = nu0 + nx0 * nu0 + nu0 * nx0 + nu0 * nu0;
  if (s->work->nx < nx_req || s->work->nu < nu_req) {
    // Solver not configured for lifted PSD dimensions
    s->settings->en_psd = 0;
    return -2;
  }
  return tiny_enable_psd(s, nx0, nu0, (tinytype)rho_psd);
}

int tinympc_cf_set_psd_disks(const float* disks, int count) {
  TinySolver* s = cf_solver();
  if (!s) return -1;
  if (!disks || count <= 0) {
    tinympc_cf_clear_psd_disks();
    return 0;
  }
  std::vector<std::array<tinytype,3>> vec;
  vec.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const int base = 3 * i;
    vec.push_back({{(tinytype)disks[base + 0],
                    (tinytype)disks[base + 1],
                    (tinytype)disks[base + 2]}});
  }
  return tiny_set_lifted_disks(s, vec);
}

int tinympc_cf_set_psd_disks_tv(const float* disks, const int* counts,
                               int stages, int max_per_stage) {
  TinySolver* s = cf_solver();
  if (!s || !disks || !counts || stages <= 0 || max_per_stage <= 0) {
    return -1;
  }
  std::vector<std::vector<std::array<tinytype,3>>> per_stage;
  per_stage.reserve(static_cast<size_t>(stages));
  for (int k = 0; k < stages; ++k) {
    int count = counts[k];
    if (count < 0) count = 0;
    if (count > max_per_stage) count = max_per_stage;
    std::vector<std::array<tinytype,3>> stage;
    stage.reserve(static_cast<size_t>(count));
    for (int j = 0; j < count; ++j) {
      const int base = (k * max_per_stage + j) * 3;
      stage.push_back({{(tinytype)disks[base + 0],
                        (tinytype)disks[base + 1],
                        (tinytype)disks[base + 2]}});
    }
    per_stage.push_back(std::move(stage));
  }
  return tiny_set_lifted_disks_tv(s, per_stage);
}

void tinympc_cf_set_psd_enabled(int enabled) {
  TinySolver* s = cf_solver();
  if (!s) return;
  s->settings->en_psd = enabled ? 1 : 0;
}

void tinympc_cf_clear_psd_disks(void) {
  TinySolver* s = cf_solver();
  if (!s) return;
  s->settings->en_state_linear = 0;
  s->work->numStateLinear = 0;
  s->work->Alin_x = tinyMatrix::Zero(0, s->work->nx);
  s->work->blin_x = tinyVector::Zero(0);
}

