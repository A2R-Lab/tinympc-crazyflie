// Simplified TinyMPC adapter for Crazyflie (no PSD)
#include "tinympc_cf_adapter.hpp"

extern "C" {
#define DEBUG_MODULE "TINYMPC"
#include "debug.h"
}

static inline TinySolver* cf_solver() {
  return &tiny_solver;
}

void tinympc_cf_init_defaults(void) {
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

  for (int k = 0; k < nhorizon; ++k) {
    for (int i = 0; i < nstates; ++i) {
      s->work->Xref(i, k) = (tinytype)Xref[k * nstates + i];
    }
  }

  for (int k = 0; k < nhorizon - 1; ++k) {
    for (int j = 0; j < ninputs; ++j) {
      s->work->Uref(j, k) = (tinytype)Uref[k * ninputs + j];
    }
  }

  // Initialize linear cost and backward pass so d is ready for first forward_pass
  tinympc::update_linear_cost(s);
  tinympc::backward_pass_grad(s);
}

int tinympc_cf_solve(void) {
  TinySolver* s = cf_solver();
  int result = tiny_solve(s);
  
  // Debug: check if forward pass populated x
  static int debug_count = 0;
  if (debug_count++ < 3) {
    // Print x0 and x1 after solve
    DEBUG_PRINT("x0=(%.2f,%.2f) x1=(%.2f,%.2f)\n",
                (double)s->work->x(0,0), (double)s->work->x(1,0),
                (double)s->work->x(0,1), (double)s->work->x(1,1));
  }
  return result;
}

void tinympc_cf_get_x_pred_1(float* out_x1, int nstates) {
  TinySolver* s = cf_solver();
  // Get state at k=1 from x trajectory (updated during forward pass)
  for (int i = 0; i < nstates; ++i) {
    out_x1[i] = (float)s->work->x(i, 1);
  }
}

void tinympc_cf_get_u0(float* out_u0, int ninputs, TinympcControlOutputKind kind) {
  TinySolver* s = cf_solver();
  for (int j = 0; j < ninputs; ++j) {
    if (kind == TinympcControlOutputKind::ZNEW) {
      out_u0[j] = (float)s->work->znew(j, 0);
    } else {
      out_u0[j] = (float)s->work->u(j, 0);
    }
  }
}
