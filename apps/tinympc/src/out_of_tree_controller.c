// Build this file as C++: it uses TinyMPC/Eigen
#ifdef __cplusplus
extern "C" {
#endif
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller.h"
#include "controller_pid.h"
#include "stabilizer_types.h"
#include "debug.h"
#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// TinyMPC
#include "tinympc/tiny_api.hpp"
#include "tinympc/types.hpp"

// Params (CMU-style headers)
#include "quadrotor_50hz_params_unconstrained.hpp"

#define DEBUG_MODULE "TINY_MPC_OOT"

// Simple MPC state
static TinySolver* g_solver = nullptr;
static bool g_ready = false;

// Minimal app main (idle)
void appMain(void) {
  DEBUG_PRINT("Waiting for activation ...\n");
  while (1) {
    vTaskDelay(M2T(2000));
  }
}

static void mpcSetupOnce() {
  if (g_ready) { return; }

  // Map params into Eigen matrices
  using namespace Eigen;
  const int nx = NSTATES;
  const int nu = NINPUTS;
  const int N = 10; // short horizon for embedded

  Map<const Matrix<tinytype, Dynamic, Dynamic, RowMajor>> A(Adyn_unconstrained_data, nx, nx);
  Map<const Matrix<tinytype, Dynamic, Dynamic, RowMajor>> B(Bdyn_unconstrained_data, nx, nu);
  Matrix<tinytype, Dynamic, 1> f = Matrix<tinytype, Dynamic, 1>::Zero(nx);

  // Use diagonal Q, R from arrays
  Map<const Matrix<tinytype, Dynamic, 1>> Qdiag(Q_unconstrained_data, nx);
  Map<const Matrix<tinytype, Dynamic, 1>> Rdiag(R_unconstrained_data, nu);
  Matrix<tinytype, Dynamic, Dynamic> Q = Qdiag.asDiagonal();
  Matrix<tinytype, Dynamic, Dynamic> R = Rdiag.asDiagonal();

  // Setup solver
  const tinytype rho = rho_unconstrained_value;
  int status = tiny_setup(&g_solver, A, B, f, Q, R, rho, nx, nu, N, 0);
  if (status) {
    DEBUG_PRINT("tiny_setup failed: %d\n", status);
    return;
  }

  // Basic bounds on inputs around hover
  const tinytype u_hover = 0.58; // normalized thrust per motor rough guess
  Matrix<tinytype, Dynamic, Dynamic> umin = Matrix<tinytype, Dynamic, Dynamic>::Zero(nu, N-1);
  Matrix<tinytype, Dynamic, Dynamic> umax = Matrix<tinytype, Dynamic, Dynamic>::Ones(nu, N-1);
  umin.array() = -u_hover;
  umax.array() = 1.0 - u_hover;

  Matrix<tinytype, Dynamic, Dynamic> xmin = Matrix<tinytype, Dynamic, Dynamic>::Constant(nx, N, -1000);
  Matrix<tinytype, Dynamic, Dynamic> xmax = Matrix<tinytype, Dynamic, Dynamic>::Constant(nx, N, 1000);
  tiny_set_bound_constraints(g_solver, xmin, xmax, umin, umax);

  g_ready = true;
}

void controllerOutOfTreeInit(void) {
  controllerPidInit();
  mpcSetupOnce();
}

bool controllerOutOfTreeTest(void) {
  return true;
}

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick) {
  if (!g_ready) {
    controllerPid(control, setpoint, sensors, state, tick);
    return;
  }

  // Build a simple hover reference at current z
  const int nx = NSTATES;
  const int nu = NINPUTS;
  const int N = g_solver->work->N;

  using namespace Eigen;
  Matrix<tinytype, Dynamic, Dynamic> Xref = Matrix<tinytype, Dynamic, Dynamic>::Zero(nx, N);
  Matrix<tinytype, Dynamic, Dynamic> Uref = Matrix<tinytype, Dynamic, Dynamic>::Zero(nu, N-1);

  // Track position.z to current estimate, keep others at 0
  Xref(2, all)? Xref(2,0) = state->position.z : void();
  tiny_set_x_ref(g_solver, Xref);
  tiny_set_u_ref(g_solver, Uref);

  // Initial state x0 from estimate (minimal mapping)
  Matrix<tinytype, Dynamic, 1> x0(nx);
  x0.setZero();
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->position.z;
  x0(3) = state->attitude.roll;
  x0(4) = state->attitude.pitch;
  x0(5) = state->attitude.yaw;
  x0(6) = state->velocity.x;
  x0(7) = state->velocity.y;
  x0(8) = state->velocity.z;
  // leave angular rates 9..11 zero (no easy access here)
  tiny_set_x0(g_solver, x0);

  // Solve
  (void)tiny_solve(g_solver);

  // Map first control to normalized motor forces (simple equal split)
  float u = 0.0f;
  if (g_solver && g_solver->work->u.cols() > 0) {
    // sum of inputs offset by hover
    using Eigen::VectorXd;
    VectorXd u0 = g_solver->work->u.col(0);
    u = static_cast<float>(u0.mean());
  }

  control->controlMode = controlModeForce;
  for (int i = 0; i < 4; i++) {
    control->normalizedForces[i] = 0.58f + u; // crude mapping
  }
}

