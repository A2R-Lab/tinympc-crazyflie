#include "tinympc/tiny_data.hpp"
#include "tinympc/tiny_api.hpp"

namespace {
constexpr int kNx = 12;
constexpr int kNu = 4;
constexpr int kN = 25;
constexpr tinytype kRho = (tinytype)5.0;

TinySolution g_solution;
TinySettings g_settings;
TinyCache g_cache;
TinyWorkspace g_work;

bool g_initialized = false;
}

TinySolver tiny_solver = {&g_solution, &g_settings, &g_cache, &g_work};

extern "C" void tinympc_init_static_solver_100hz(void) {
  if (g_initialized) {
    return;
  }
  g_initialized = true;

  // Load 100 Hz parameters into float matrices.
  Matrix<float, kNx, kNx> A_f;
  Matrix<float, kNx, kNu> B_f;
  Matrix<float, kNx, kNx> Q_f;
  Matrix<float, kNu, kNu> R_f;
  Matrix<float, kNu, kNx> Kinf_f;
  Matrix<float, kNx, kNx> Pinf_f;
  Matrix<float, kNu, kNu> Quu_inv_f;
  Matrix<float, kNx, kNx> AmBKt_f;
  Matrix<float, kNx, kNu> coeff_d2p_f;
  tinympc_load_params_100hz(A_f, B_f, Q_f, R_f, Kinf_f, Pinf_f, Quu_inv_f, AmBKt_f, coeff_d2p_f);

  // Initialize solution
  g_solution.iter = 0;
  g_solution.solved = 0;
  g_solution.x = tinyMatrix::Zero(kNx, kN);
  g_solution.u = tinyMatrix::Zero(kNu, kN - 1);

  // Initialize settings
  tiny_set_default_settings(&g_settings);

  // Initialize workspace dimensions
  g_work.nx = kNx;
  g_work.nu = kNu;
  g_work.N = kN;

  // Core state/input trajectories
  g_work.x = tinyMatrix::Zero(kNx, kN);
  g_work.u = tinyMatrix::Zero(kNu, kN - 1);

  g_work.q = tinyMatrix::Zero(kNx, kN);
  g_work.r = tinyMatrix::Zero(kNu, kN - 1);

  g_work.p = tinyMatrix::Zero(kNx, kN);
  g_work.d = tinyMatrix::Zero(kNu, kN - 1);

  // Bound constraint variables
  g_work.v = tinyMatrix::Zero(kNx, kN);
  g_work.vnew = tinyMatrix::Zero(kNx, kN);
  g_work.z = tinyMatrix::Zero(kNu, kN - 1);
  g_work.znew = tinyMatrix::Zero(kNu, kN - 1);

  g_work.g = tinyMatrix::Zero(kNx, kN);
  g_work.y = tinyMatrix::Zero(kNu, kN - 1);

  // Cone constraint variables
  g_work.vc = tinyMatrix::Zero(kNx, kN);
  g_work.vcnew = tinyMatrix::Zero(kNx, kN);
  g_work.zc = tinyMatrix::Zero(kNu, kN - 1);
  g_work.zcnew = tinyMatrix::Zero(kNu, kN - 1);

  g_work.gc = tinyMatrix::Zero(kNx, kN);
  g_work.yc = tinyMatrix::Zero(kNu, kN - 1);

  // Linear constraint variables
  g_work.vl = tinyMatrix::Zero(kNx, kN);
  g_work.vlnew = tinyMatrix::Zero(kNx, kN);
  g_work.zl = tinyMatrix::Zero(kNu, kN - 1);
  g_work.zlnew = tinyMatrix::Zero(kNu, kN - 1);

  g_work.gl = tinyMatrix::Zero(kNx, kN);
  g_work.yl = tinyMatrix::Zero(kNu, kN - 1);

  // Time-varying linear constraint variables
  g_work.vl_tv = tinyMatrix::Zero(kNx, kN);
  g_work.vlnew_tv = tinyMatrix::Zero(kNx, kN);
  g_work.zl_tv = tinyMatrix::Zero(kNu, kN - 1);
  g_work.zlnew_tv = tinyMatrix::Zero(kNu, kN - 1);

  g_work.gl_tv = tinyMatrix::Zero(kNx, kN);
  g_work.yl_tv = tinyMatrix::Zero(kNu, kN - 1);

  // Cost and dynamics
  tinyMatrix Q = Q_f.cast<tinytype>();
  tinyMatrix R = R_f.cast<tinytype>();
  g_work.Q = (Q + kRho * tinyMatrix::Identity(kNx, kNx)).diagonal();
  g_work.R = (R + kRho * tinyMatrix::Identity(kNu, kNu)).diagonal();

  g_work.Adyn = A_f.cast<tinytype>();
  g_work.Bdyn = B_f.cast<tinytype>();
  g_work.fdyn = tinyVector::Zero(kNx);

  g_work.Xref = tinyMatrix::Zero(kNx, kN);
  g_work.Uref = tinyMatrix::Zero(kNu, kN - 1);
  g_work.Qu = tinyVector::Zero(kNu);

  g_work.primal_residual_state = 0;
  g_work.primal_residual_input = 0;
  g_work.dual_residual_state = 0;
  g_work.dual_residual_input = 0;
  g_work.status = 0;
  g_work.iter = 0;

  // Bounds default to wide limits (match codegen default)
  g_work.x_min = tinyMatrix::Constant(kNx, kN, (tinytype)-1e9);
  g_work.x_max = tinyMatrix::Constant(kNx, kN, (tinytype)1e9);
  g_work.u_min = tinyMatrix::Constant(kNu, kN - 1, (tinytype)-1e9);
  g_work.u_max = tinyMatrix::Constant(kNu, kN - 1, (tinytype)1e9);

  // Cache (precomputed) from params_100hz.h
  g_cache.rho = kRho;
  g_cache.Kinf = Kinf_f.cast<tinytype>();
  g_cache.Pinf = Pinf_f.cast<tinytype>();
  g_cache.Quu_inv = Quu_inv_f.cast<tinytype>();
  g_cache.AmBKt = AmBKt_f.cast<tinytype>();
  g_cache.APf = tinyVector::Zero(kNx);
  g_cache.BPf = tinyVector::Zero(kNu);
  g_cache.C1 = g_cache.Quu_inv;
  g_cache.C2 = g_cache.AmBKt;

  // Sensitivity matrices (disabled by default)
  g_cache.dKinf_drho = tinyMatrix::Zero(kNu, kNx);
  g_cache.dPinf_drho = tinyMatrix::Zero(kNx, kNx);
  g_cache.dC1_drho = tinyMatrix::Zero(kNu, kNu);
  g_cache.dC2_drho = tinyMatrix::Zero(kNx, kNx);
}
