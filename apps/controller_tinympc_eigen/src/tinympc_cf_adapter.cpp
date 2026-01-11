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

  update_linear_cost(s);
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

int tinympc_cf_enable_psd(int nx0, float rho_psd) {
  TinySolver* s = cf_solver();
  return tinympc::tiny_enable_psd(s, nx0, (tinytype)rho_psd);
}

int tinympc_cf_add_psd_disk(float cx, float cy, float radius) {
  TinySolver* s = cf_solver();
  return tinympc::tiny_add_psd_disk(s, (tinytype)cx, (tinytype)cy, (tinytype)radius);
}

int tinympc_cf_update_psd_disk(int idx, float cx, float cy, float radius) {
  TinySolver* s = cf_solver();
  return tinympc::tiny_update_psd_disk(s, idx, (tinytype)cx, (tinytype)cy, (tinytype)radius);
}

void tinympc_cf_clear_psd_disks(void) {
  TinySolver* s = cf_solver();
  tinympc::tiny_clear_psd_disks(s);
}

int tinympc_cf_get_psd_num_disks(void) {
  TinySolver* s = cf_solver();
  return s->work->psd_num_disks;
}

float tinympc_cf_get_psd_residual(void) {
  TinySolver* s = cf_solver();
  return (float)s->work->primal_residual_psd;
}
