#include <cmath>
#include <cstring>
#include <stdexcept>

#include "tinympc/tinympc.h"
extern "C" {
#include "gotf/dare.h"
#include "gotf/tinympc_model.h"
}

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

bool inverse4(const Eigen::MatrixMf& input, Eigen::MatrixMf* output) {
  float augmented[NINPUTS][2 * NINPUTS];
  for (int row = 0; row < NINPUTS; ++row) {
    for (int col = 0; col < NINPUTS; ++col) {
      augmented[row][col] = input(row, col);
      augmented[row][col + NINPUTS] = row == col ? 1.0f : 0.0f;
    }
  }
  for (int pivot = 0; pivot < NINPUTS; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < NINPUTS; ++row) {
      if (std::fabs(augmented[row][pivot]) > std::fabs(augmented[best][pivot])) best = row;
    }
    if (std::fabs(augmented[best][pivot]) < 1.0e-10f) return false;
    for (int col = 0; col < 2 * NINPUTS; ++col) {
      const float tmp = augmented[pivot][col];
      augmented[pivot][col] = augmented[best][col];
      augmented[best][col] = tmp;
    }
    const float scale = 1.0f / augmented[pivot][pivot];
    for (int col = 0; col < 2 * NINPUTS; ++col) augmented[pivot][col] *= scale;
    for (int row = 0; row < NINPUTS; ++row) {
      if (row == pivot) continue;
      const float factor = augmented[row][pivot];
      for (int col = 0; col < 2 * NINPUTS; ++col) augmented[row][col] -= factor * augmented[pivot][col];
    }
  }
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NINPUTS; ++col)
      (*output)(row, col) = augmented[row][col + NINPUTS];
  return true;
}

void rebuild_lqr_cache(float rho) {
  // Recompute the infinite-horizon LQR terminal cost and the three primal
  // recursion caches from the selected discrete model.  The firmware headers
  // contain these values pre-generated for their fixed sampling time; the
  // host experiment must not reuse them after changing the model timestep.
  Eigen::MatrixNf q_lqr = Q;
  for (int i = 0; i < NSTATES; ++i) q_lqr(i, i) += rho;
  Eigen::MatrixMf r_lqr = R;
  for (int i = 0; i < NINPUTS; ++i) r_lqr(i, i) += rho;
  gotf_float a_data[NSTATES * NSTATES], b_data[NSTATES * NINPUTS];
  gotf_float q_data[NSTATES * NSTATES], r_data[NINPUTS * NINPUTS];
  gotf_float p_data[NSTATES * NSTATES], k_data[NINPUTS * NSTATES];
  for (int row = 0; row < NSTATES; ++row) {
    for (int col = 0; col < NSTATES; ++col) {
      a_data[row * NSTATES + col] = static_cast<gotf_float>(A(row, col));
      q_data[row * NSTATES + col] = static_cast<gotf_float>(q_lqr(row, col));
    }
    for (int col = 0; col < NINPUTS; ++col)
      b_data[row * NINPUTS + col] = static_cast<gotf_float>(B(row, col));
  }
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NINPUTS; ++col)
      r_data[row * NINPUTS + col] = static_cast<gotf_float>(r_lqr(row, col));
  if (gotf_dare_solve(a_data, b_data, q_data, r_data, NSTATES, NINPUTS, p_data, k_data) != 0)
    throw std::runtime_error("DARE regeneration failed");
  for (int row = 0; row < NSTATES; ++row)
    for (int col = 0; col < NSTATES; ++col)
      Pinf(row, col) = static_cast<float>(p_data[row * NSTATES + col]);
  // gotf_dare_solve returns the conventional negative feedback gain u=Kx;
  // this TinyMPC fork stores the positive gain and applies U -= Kinf*X.
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NSTATES; ++col)
      Kinf(row, col) = -static_cast<float>(k_data[row * NSTATES + col]);
  Eigen::MatrixMf s = r_lqr;
  Eigen::MatrixMNf btp;
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NSTATES; ++col) {
      float sum = 0.0f;
      for (int j = 0; j < NSTATES; ++j) sum += B(j, row) * Pinf(j, col);
      btp(row, col) = sum;
    }
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NINPUTS; ++col)
      for (int j = 0; j < NSTATES; ++j) s(row, col) += btp(row, j) * B(j, col);
  if (!inverse4(s, &Quu_inv)) throw std::runtime_error("singular final input Hessian");
  for (int row = 0; row < NINPUTS; ++row)
    for (int col = 0; col < NSTATES; ++col) {
      float sum = 0.0f;
      for (int j = 0; j < NINPUTS; ++j) {
        float gain = 0.0f;
        for (int l = 0; l < NSTATES; ++l) gain += btp(j, l) * A(l, col);
        sum += Quu_inv(row, j) * gain;
      }
      Kinf(row, col) = sum;
    }
  for (int row = 0; row < NSTATES; ++row)
    for (int col = 0; col < NSTATES; ++col) {
      float bk = 0.0f;
      for (int j = 0; j < NINPUTS; ++j) bk += B(row, j) * Kinf(j, col);
      AmBKt(col, row) = A(row, col) - bk;
    }
  for (int row = 0; row < NSTATES; ++row)
    for (int col = 0; col < NINPUTS; ++col) {
      float kr = 0.0f;
      float apb = 0.0f;
      for (int j = 0; j < NINPUTS; ++j) kr += Kinf(j, row) * r_lqr(j, col);
      for (int j = 0; j < NSTATES; ++j)
        for (int l = 0; l < NSTATES; ++l)
          apb += AmBKt(row, j) * Pinf(j, l) * B(l, col);
      coeff_d2p(row, col) = kr - apb;
    }
}

bool load_params(float model_dt_s, float rho) {
#include "../src/params_100hz.h"
  if (!(model_dt_s > 0.0f) || !std::isfinite(model_dt_s)) {
    return false;
  }
  // This is the continuous nonlinear Crazyflie model used to create the
  // deployed params_100hz header.  It integrates RK4 at model_dt_s and
  // linearizes the discrete transition about hover.  In other words, A/B are
  // regenerated for this experiment rather than composed from the 20 ms
  // header.  The implementation currently uses symmetric finite differences;
  // it is the C equivalent of the automatic-differentiation workflow in the
  // TinyMPC model documentation.
  gotf_tinympc_model physical_model = gotf_tinympc_model_cf1();
  physical_model.dt = static_cast<gotf_float>(model_dt_s);
  gotf_float a_generated[GOTF_NX * GOTF_NX];
  gotf_float b_generated[GOTF_NX * GOTF_NU];
  gotf_tinympc_hover_linearize(&physical_model, a_generated, b_generated);
  for (int row = 0; row < NSTATES; ++row) {
    for (int col = 0; col < NSTATES; ++col) {
      A(row, col) = static_cast<float>(a_generated[row * NSTATES + col]);
    }
    for (int col = 0; col < NINPUTS; ++col) {
      B(row, col) = static_cast<float>(b_generated[row * NINPUTS + col]);
    }
  }
  // Preserve the same continuous-time penalty over this new discrete knot.
  const float cost_scale = model_dt_s / 0.020f;
  Q *= cost_scale;
  R *= cost_scale;
  rebuild_lqr_cache(rho);
  for (int k = 0; k < NHORIZON - 1; ++k) {
    A_model[k] = A;
    B_model[k] = B;
  }
  return true;
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
      data.a_pos_hs[k][h].setZero();
      data.a_vel_hs[k][h].setZero();
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

bool tinympc_admm_host_init(int max_iter, float rho, bool enable_state_constraints, float model_dt_s) {
  if (!load_params(model_dt_s, rho)) {
    return false;
  }
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
      data.a_pos_hs[k][h] << a_hs_in[offset3 + 0], a_hs_in[offset3 + 1], a_hs_in[offset3 + 2];
      data.a_vel_hs[k][h].setZero();
      const float norm = data.a_pos_hs[k][h].norm();
      if (norm > 1e-6f) {
        data.a_pos_hs[k][h] /= norm;
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
