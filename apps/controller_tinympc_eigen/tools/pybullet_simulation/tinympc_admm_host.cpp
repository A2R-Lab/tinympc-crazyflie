#include <cmath>
#include <cstring>
#include <stdexcept>

#include "tinympc/tinympc.h"
#include "../../src/tinympc_generated_params.h"
#include "../../src/tinympc_banked_model_bank.h"

namespace {

static Eigen::MatrixNf A;
static Eigen::MatrixNMf B;
static Eigen::MatrixNf A_model[NHORIZON - 1];
static Eigen::MatrixNMf B_model[NHORIZON - 1];
static Eigen::VectorNf f_model[NHORIZON - 1];
static Eigen::MatrixMNf Kinf;
static Eigen::MatrixNf Pinf;
static Eigen::MatrixMf Quu_inv;
static Eigen::MatrixNf AmBKt;
static Eigen::MatrixNMf coeff_d2p;
static Eigen::VectorNf APf;
static Eigen::VectorMf BPf;
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
static int selected_model = 0;
static const float max_motor_thrust = 3.72e-8f * 2900.0f * 2900.0f;

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
  Pinf = q_lqr;
  for (int iteration = 0; iteration < 50000; ++iteration) {
    Eigen::MatrixMf hessian = r_lqr + B.transpose() * Pinf * B;
    Eigen::MatrixMf inverse;
    if (!inverse4(hessian, &inverse)) throw std::runtime_error("DARE input Hessian is singular");
    Eigen::MatrixMNf gain = inverse * B.transpose() * Pinf * A;
    Eigen::MatrixNf next = q_lqr + A.transpose() * Pinf * (A - B * gain);
    const float delta = (next - Pinf).cwiseAbs().maxCoeff();
    Pinf = next;
    if (delta < 1.0e-4f) break;
  }
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

void install_model(
    const float* a, const float* b, const float* affine,
    const float* kinf, const float* pinf, const float* quu_inv,
    const float* ambkt, const float* coeff, const float* apf,
    const float* bpf) {
  for (int row = 0; row < NSTATES; ++row) {
    for (int col = 0; col < NSTATES; ++col) {
      A(row, col) = a[row * NSTATES + col];
      Pinf(row, col) = pinf[row * NSTATES + col];
      AmBKt(row, col) = ambkt[row * NSTATES + col];
    }
    APf(row) = apf[row];
    for (int col = 0; col < NINPUTS; ++col) {
      B(row, col) = b[row * NINPUTS + col];
      coeff_d2p(row, col) = coeff[row * NINPUTS + col];
    }
  }
  for (int row = 0; row < NINPUTS; ++row) {
    for (int col = 0; col < NSTATES; ++col)
      Kinf(row, col) = kinf[row * NSTATES + col];
    for (int col = 0; col < NINPUTS; ++col)
      Quu_inv(row, col) = quu_inv[row * NINPUTS + col];
    BPf(row) = bpf[row];
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    A_model[k] = A;
    B_model[k] = B;
    for (int state = 0; state < NSTATES; ++state) f_model[k](state) = affine[state];
  }
}

bool select_stored_model(int model_id) {
  switch (model_id) {
    case 0:
      install_model(
          tinympc_generated_A, tinympc_generated_B, tinympc_generated_f,
          tinympc_generated_Kinf, tinympc_generated_Pinf,
          tinympc_generated_Quu_inv, tinympc_generated_AmBKt,
          tinympc_generated_coeff_d2p, tinympc_generated_APf,
          tinympc_generated_BPf);
      break;
    case 1:
      install_model(
          tinympc_bank_left_15_A, tinympc_bank_left_15_B,
          tinympc_bank_left_15_f, tinympc_bank_left_15_Kinf,
          tinympc_bank_left_15_Pinf, tinympc_bank_left_15_Quu_inv,
          tinympc_bank_left_15_AmBKt, tinympc_bank_left_15_coeff_d2p,
          tinympc_bank_left_15_APf, tinympc_bank_left_15_BPf);
      break;
    case 2:
      install_model(
          tinympc_bank_right_15_A, tinympc_bank_right_15_B,
          tinympc_bank_right_15_f, tinympc_bank_right_15_Kinf,
          tinympc_bank_right_15_Pinf, tinympc_bank_right_15_Quu_inv,
          tinympc_bank_right_15_AmBKt, tinympc_bank_right_15_coeff_d2p,
          tinympc_bank_right_15_APf, tinympc_bank_right_15_BPf);
      break;
    case 3:
      install_model(
          tinympc_bank_left_30_A, tinympc_bank_left_30_B,
          tinympc_bank_left_30_f, tinympc_bank_left_30_Kinf,
          tinympc_bank_left_30_Pinf, tinympc_bank_left_30_Quu_inv,
          tinympc_bank_left_30_AmBKt, tinympc_bank_left_30_coeff_d2p,
          tinympc_bank_left_30_APf, tinympc_bank_left_30_BPf);
      break;
    case 4:
      install_model(
          tinympc_bank_right_30_A, tinympc_bank_right_30_B,
          tinympc_bank_right_30_f, tinympc_bank_right_30_Kinf,
          tinympc_bank_right_30_Pinf, tinympc_bank_right_30_Quu_inv,
          tinympc_bank_right_30_AmBKt, tinympc_bank_right_30_coeff_d2p,
          tinympc_bank_right_30_APf, tinympc_bank_right_30_BPf);
      break;
    case 5:
      install_model(
          tinympc_bank_left_60_A, tinympc_bank_left_60_B,
          tinympc_bank_left_60_f, tinympc_bank_left_60_Kinf,
          tinympc_bank_left_60_Pinf, tinympc_bank_left_60_Quu_inv,
          tinympc_bank_left_60_AmBKt, tinympc_bank_left_60_coeff_d2p,
          tinympc_bank_left_60_APf, tinympc_bank_left_60_BPf);
      break;
    case 6:
      install_model(
          tinympc_bank_right_60_A, tinympc_bank_right_60_B,
          tinympc_bank_right_60_f, tinympc_bank_right_60_Kinf,
          tinympc_bank_right_60_Pinf, tinympc_bank_right_60_Quu_inv,
          tinympc_bank_right_60_AmBKt, tinympc_bank_right_60_coeff_d2p,
          tinympc_bank_right_60_APf, tinympc_bank_right_60_BPf);
      break;
    default:
      return false;
  }
  selected_model = model_id;
  return true;
}

bool load_params(float model_dt_s, float rho) {
  if (!(model_dt_s > 0.0f) || !std::isfinite(model_dt_s)) {
    return false;
  }
  Q.setZero();
  R.setZero();
  for (int state = 0; state < NSTATES; ++state)
    Q(state, state) = tinympc_generated_Q_diagonal[state];
  for (int input = 0; input < NINPUTS; ++input)
    R(input, input) = tinympc_generated_R_diagonal[input];
  // params_100hz.h is the same generated dynamics/cost source used by the
  // firmware.  Compose its 20 ms discrete transition for integer multiples;
  // this keeps the host build entirely within this repository.
  const float base_dt_s = TINYMPC_GENERATED_MODEL_DT_S;
  const int compositions = static_cast<int>(std::lround(model_dt_s / base_dt_s));
  if (compositions < 1 || std::fabs(model_dt_s - compositions * base_dt_s) > 1.0e-6f) return false;
  if (!select_stored_model(0)) return false;
  const Eigen::MatrixNf base_a = A;
  const Eigen::MatrixNMf base_b = B;
  Eigen::MatrixNf composed_a = Eigen::MatrixNf::Identity();
  Eigen::MatrixNMf composed_b = Eigen::MatrixNMf::Zero();
  for (int i = 0; i < compositions; ++i) {
    composed_b = base_a * composed_b + base_b;
    composed_a = base_a * composed_a;
  }
  A = composed_a;
  B = composed_b;
  // Preserve the same continuous-time penalty over this new discrete knot.
  const float cost_scale = model_dt_s / base_dt_s;
  Q *= cost_scale;
  R *= cost_scale;
  if (compositions != 1 || std::fabs(rho - TINYMPC_GENERATED_ADMM_RHO) > 1.0e-5f) {
    rebuild_lqr_cache(rho);
    APf.setZero();
    BPf.setZero();
    for (int k = 0; k < NHORIZON - 1; ++k) {
      A_model[k] = A;
      B_model[k] = B;
      f_model[k].setZero();
    }
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

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 1, model_dt_s, A_model, B_model, f_model);
  tiny_InitSettings(&stgs);
  stgs.rho_init = rho;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCacheAffine(
      &work, &Quu_inv, &AmBKt, &coeff_d2p, &APf, &BPf);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);
  tiny_SetInitialState(&work, &x0);
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);

  for (int motor = 0; motor < NINPUTS; ++motor) {
    const float hover = tinympc_generated_physical_hover_thrust[motor];
    ucu(motor) = max_motor_thrust - hover;
    lcu(motor) = -hover;
  }
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

bool tinympc_admm_host_select_model(int model_id) {
  if (!initialized || !select_stored_model(model_id)) return false;
  tinympc_admm_host_reset_duals();
  return true;
}

int tinympc_admm_host_selected_model() { return selected_model; }

bool tinympc_admm_host_set_input_baseline(const float* physical_motor_thrust) {
  if (!initialized || physical_motor_thrust == 0) return false;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    const float baseline = physical_motor_thrust[motor];
    if (!std::isfinite(baseline) || baseline < 0.0f || baseline > max_motor_thrust) {
      return false;
    }
    // TinyMPC's decision variable is a correction about this baseline.  Keep
    // the complete physical command inside the same actuator envelope used by
    // firmware and the PyBullet plant, instead of relying on post-solve clips.
    lcu(motor) = -baseline;
    ucu(motor) = max_motor_thrust - baseline;
  }
  return true;
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
