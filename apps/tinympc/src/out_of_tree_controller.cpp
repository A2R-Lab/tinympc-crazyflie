/**
 * TinyMPC Out-of-Tree Controller for Crazyflie
 *
 * Basic TinyMPC implementation using the TinyMPC submodule directly
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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

// Include Eigen 
#include <Eigen/Dense>

// TinyMPC dimensions (must be defined before including parameter file)
#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 10

// Use local parameter file
#include "quadrotor_50hz_params.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINY_MPC_OOT"
#include "debug.h"

// Include TinyMPC API directly
#include "tinympc/types.hpp"
#include "tinympc/tiny_api.hpp"

// TinyMPC solver instance
static TinySolver* g_solver = nullptr;
static bool g_solver_initialized = false;
static bool g_use_mpc = true;

// TinyMPC type definitions
typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

// Initialize TinyMPC solver
static void initTinyMPC(void) {
  if (g_solver_initialized) return;

  // Map parameter data to Eigen matrices
  tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
  tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
  tinyVector fdyn = tiny_VectorNx::Zero();
  tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
  tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

  // Set up constraint bounds
  tinyMatrix x_min = tiny_MatrixNxNh::Constant(-2.0);
  tinyMatrix x_max = tiny_MatrixNxNh::Constant(2.0);
  tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(0.0);
  tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(1.0);

  // Initialize TinyMPC solver
  int status = tiny_setup(&g_solver, Adyn, Bdyn, fdyn, Q.asDiagonal(), R.asDiagonal(),
                          rho_value, NSTATES, NINPUTS, NHORIZON, 1);
  if (status != 0) {
    DEBUG_PRINT("TinyMPC setup failed: %d\n", status);
    return;
  }

  status = tiny_set_bound_constraints(g_solver, x_min, x_max, u_min, u_max);
  if (status != 0) {
    DEBUG_PRINT("TinyMPC constraint setup failed: %d\n", status);
    return;
  }

  // Set solver parameters
  g_solver->settings->max_iter = 50;

  // Set default reference (hover at 2m height)
  tiny_VectorNx Xref_hover;
  Xref_hover << 0, 0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  g_solver->work->Xref = Xref_hover.replicate<1, NHORIZON>();

  g_solver_initialized = true;
  DEBUG_PRINT("TinyMPC initialized successfully!\n");
}

// Convert Crazyflie state to TinyMPC state vector
static tiny_VectorNx crazyflieStateToTinyMPC(const state_t* state, const sensorData_t* sensors) {
  tiny_VectorNx x0 = tiny_VectorNx::Zero();

  // Position (x, y, z)
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->position.z;

  // Attitude (using Rodriguez parameters, not Euler angles)
  // For simplicity, approximate with small angle assumption
  x0(3) = state->attitude.roll * M_PI / 180.0;   // Convert to radians
  x0(4) = state->attitude.pitch * M_PI / 180.0;
  x0(5) = state->attitude.yaw * M_PI / 180.0;

  // Velocity
  x0(6) = state->velocity.x;
  x0(7) = state->velocity.y;
  x0(8) = state->velocity.z;

  // Angular rates (convert to rad/s)
  x0(9) = sensors->gyro.x * M_PI / 180.0;
  x0(10) = sensors->gyro.y * M_PI / 180.0;
  x0(11) = sensors->gyro.z * M_PI / 180.0;

  return x0;
}

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
extern "C" void appMain(void) {
  DEBUG_PRINT("TinyMPC OOT Controller active...\n");
  while (1) {
    vTaskDelay(M2T(2000));
  }
}

extern "C" void controllerOutOfTreeInit(void) {
  // Initialize PID controller as fallback
  controllerPidInit();

  // Initialize TinyMPC
  initTinyMPC();

  DEBUG_PRINT("TinyMPC OOT controller initialized!\n");
}

extern "C" bool controllerOutOfTreeTest(void) {
  return g_solver_initialized;
}

extern "C" void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick) {

  if (!g_solver_initialized || !g_use_mpc) {
    // Fallback to PID
    DEBUG_PRINT("Using PID fallback\n");
    controllerPid(control, setpoint, sensors, state, tick);
    return;
  }

  // Convert current state to state vector
  tiny_VectorNx x0 = crazyflieStateToTinyMPC(state, sensors);

  // Set reference trajectory
  tiny_VectorNx Xref_current;
  Xref_current << setpoint->position.x, setpoint->position.y, setpoint->position.z,
                  0, 0, setpoint->attitude.yaw * M_PI / 180.0,
                  0, 0, 0, 0, 0, 0;
  g_solver->work->Xref = Xref_current.replicate<1, NHORIZON>();

  // Set initial state
  tiny_set_x0(g_solver, x0);

  // Solve MPC problem
  tiny_solve(g_solver);

  // Extract first control input
  tiny_VectorNx u0 = g_solver->work->u.col(0);

  // Apply control to motors (convert thrust to normalized forces)
  control->controlMode = controlModeForce;
  for (int i = 0; i < 4; i++) {
    // Clamp thrust between 0 and 1
    float thrust = fmaxf(0.0f, fminf(1.0f, u0(i)));
    control->normalizedForces[i] = 0.5f + thrust * 0.3f;  // Base thrust + MPC output
  }

  DEBUG_PRINT("MPC: pos=%.2f,%.2f,%.2f thrust=%.2f,%.2f,%.2f,%.2f\n",
              x0(0), x0(1), x0(2),
              control->normalizedForces[0],
              control->normalizedForces[1],
              control->normalizedForces[2],
              control->normalizedForces[3]);
}
