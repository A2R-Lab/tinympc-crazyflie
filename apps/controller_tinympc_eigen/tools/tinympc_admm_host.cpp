#include <cmath>
#include <cstring>

#include "tinympc/tinympc.h"

namespace {

static Eigen::MatrixNf A;
static Eigen::MatrixNMf B;
static Eigen::MatrixNf A_model[NHORIZON - 1];
static Eigen::MatrixNMf B_model[NHORIZON - 1];
static Eigen::MatrixMNf Kinf;
static Eigen::MatrixNf Pinf;
static Eigen::MatrixMf Quu_inv;
static Eigen::MatrixNf AmBKt;
static Eigen::MatrixNMf coeff_d2p;
static Eigen::MatrixNf Q;
static Eigen::MatrixMf R;

static Eigen::VectorNf Xhrz[NHORIZON];
static Eigen::VectorMf Uhrz[NHORIZON - 1];
static Eigen::VectorNf YX[NHORIZON];
static Eigen::VectorMf YU[NHORIZON];
static Eigen::VectorNf ZX[NHORIZON];
static Eigen::VectorNf ZX_new[NHORIZON];
static Eigen::VectorMf ZU[NHORIZON - 1];
static Eigen::VectorMf ZU_new[NHORIZON - 1];
static Eigen::VectorMf d[NHORIZON - 1];
static Eigen::VectorNf p[NHORIZON];
static Eigen::VectorNf q[NHORIZON - 1];
static Eigen::VectorMf r[NHORIZON - 1];
static Eigen::VectorMf r_tilde[NHORIZON - 1];
static Eigen::VectorNf Xref[NHORIZON];
static Eigen::VectorMf Uref[NHORIZON - 1];
static Eigen::VectorNf x0;
static Eigen::VectorMf Qu;
static Eigen::MatrixMf Acu;
static Eigen::VectorMf ucu;
static Eigen::VectorMf lcu;
static Eigen::MatrixNf Acx;
static Eigen::VectorNf ucx;
static Eigen::VectorNf lcx;

static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

static bool initialized = false;
static const float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};

void load_params() {
#include "../src/params_100hz.h"
  for (int k = 0; k < NHORIZON - 1; ++k) {
    A_model[k] = A;
    B_model[k] = B;
  }
}

void reset_vectors() {
  x0.setZero();
  for (int k = 0; k < NHORIZON; ++k) {
    Xhrz[k].setZero();
    YX[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    Xref[k].setZero();
    p[k].setZero();
    for (int h = 0; h < MAX_HS; ++h) {
      data.a_hs[k][h].setZero();
      data.b_hs[k][h] = 0.0f;
      data.en_hs[k][h] = 0;
    }
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uhrz[k].setZero();
    YU[k].setZero();
    ZU[k].setZero();
    ZU_new[k].setZero();
    d[k].setZero();
    q[k].setZero();
    r[k].setZero();
    r_tilde[k].setZero();
    Uref[k].setZero();
  }
}

}  // namespace

extern "C" {

bool tinympc_admm_host_init(int max_iter, float rho, bool enable_state_constraints) {
  load_params();
  reset_vectors();

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, 0.002f, A_model, B_model, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = rho;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);
  tiny_SetInitialState(&work, &x0);
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);

  ucu << 1.0f - u_hover[0], 1.0f - u_hover[1], 1.0f - u_hover[2], 1.0f - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  ucx.setConstant(1.0e6f);
  lcx.setConstant(-1.0e6f);
  tiny_SetStateBound(&work, &Acx, &lcx, &ucx);

  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = enable_state_constraints ? 1 : 0;
  stgs.max_iter = max_iter;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2f;
  stgs.tol_abs_prim = 5e-2f;

  tiny_UpdateLinearCost(&work);
  initialized = true;
  return true;
}

void tinympc_admm_host_reset_duals() {
  for (int k = 0; k < NHORIZON; ++k) {
    YX[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    YU[k].setZero();
    ZU[k].setZero();
    ZU_new[k].setZero();
  }
  work.first_run = 1;
}

bool tinympc_admm_host_solve(
    const float* x0_in,
    const float* xref_in,
    const float* uref_in,
    const float* a_hs_in,
    const float* b_hs_in,
    const int* en_hs_in,
    int enable_state_constraints,
    float* x_out,
    float* u_out,
    int* status_out,
    int* iter_out,
    float* pri_res_out,
    float* dua_res_out) {
  if (!initialized) {
    return false;
  }
  for (int i = 0; i < NSTATES; ++i) {
    x0(i) = x0_in[i];
  }
  for (int k = 0; k < NHORIZON; ++k) {
    for (int i = 0; i < NSTATES; ++i) {
      Xref[k](i) = xref_in[k * NSTATES + i];
    }
    for (int h = 0; h < MAX_HS; ++h) {
      const int offset3 = 3 * (k * MAX_HS + h);
      const int offset1 = k * MAX_HS + h;
      data.en_hs[k][h] = en_hs_in[offset1] ? 1 : 0;
      data.b_hs[k][h] = b_hs_in[offset1];
      data.a_hs[k][h] << a_hs_in[offset3 + 0], a_hs_in[offset3 + 1], a_hs_in[offset3 + 2];
      const float norm = data.a_hs[k][h].norm();
      if (norm > 1e-6f) {
        data.a_hs[k][h] /= norm;
        data.b_hs[k][h] /= norm;
      } else {
        data.en_hs[k][h] = 0;
      }
    }
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    for (int i = 0; i < NINPUTS; ++i) {
      Uref[k](i) = uref_in[k * NINPUTS + i];
    }
  }

  stgs.en_cstr_states = enable_state_constraints ? 1 : 0;
  tiny_UpdateLinearCost(&work);
  for (int k = 0; k < NHORIZON - 1; ++k) {
    r_tilde[k] = r[k];
  }
  tiny_SolveAdmm(&work);

  for (int k = 0; k < NHORIZON; ++k) {
    for (int i = 0; i < NSTATES; ++i) {
      x_out[k * NSTATES + i] = Xhrz[k](i);
    }
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    for (int i = 0; i < NINPUTS; ++i) {
      u_out[k * NINPUTS + i] = Uhrz[k](i);
    }
  }
  if (status_out != 0) {
    *status_out = info.status_val;
  }
  if (iter_out != 0) {
    *iter_out = info.iter;
  }
  if (pri_res_out != 0) {
    *pri_res_out = info.pri_res;
  }
  if (dua_res_out != 0) {
    *dua_res_out = info.dua_res;
  }
  return true;
}

int tinympc_admm_host_nstates() {
  return NSTATES;
}

int tinympc_admm_host_ninputs() {
  return NINPUTS;
}

int tinympc_admm_host_horizon() {
  return NHORIZON;
}

}  // extern "C"
