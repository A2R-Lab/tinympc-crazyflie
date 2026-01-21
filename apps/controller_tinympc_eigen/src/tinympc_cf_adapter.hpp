#pragma once

// Crazyflie Eigen wrapper
#include <Eigen.h>
using namespace Eigen;

#ifdef __cplusplus
#include "tinympc/tiny_data.hpp"   // extern TinySolver tiny_solver
#include "tinympc/tiny_api.hpp"    // tiny_solve, update_linear_cost
#endif

enum class TinympcControlOutputKind {
  ZNEW,
  UPRIMAL
};

#ifdef __cplusplus
extern "C" {
#endif

void tinympc_cf_init_defaults(void);
void tinympc_cf_set_settings(int max_iter, int check_termination);
void tinympc_cf_set_x0(const float* x0, int nstates);
void tinympc_cf_set_reference(const float* Xref, int nstates, int nhorizon,
                              const float* Uref, int ninputs);
int tinympc_cf_solve(void);
void tinympc_cf_get_x_pred_1(float* out_x1, int nstates);
void tinympc_cf_get_u0(float* out_u0, int ninputs, TinympcControlOutputKind kind);

#ifdef __cplusplus
}
#endif
