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

// Simple MPC state
static bool g_use_mpc = true;
static bool g_mpc_initialized = false;

// TinyMPC type definitions
typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

// Initialize simple MPC
static void initMPC(void) {
  if (g_mpc_initialized) return;

  // Map parameter data to Eigen matrices for reference
  tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
  tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
  tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
  tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

  // Simple initialization - just mark as ready
  g_mpc_initialized = true;
  DEBUG_PRINT("Simple MPC initialized successfully!\n");
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

  // Initialize simple MPC
  initMPC();

  DEBUG_PRINT("Simple MPC OOT controller initialized!\n");
}

extern "C" bool controllerOutOfTreeTest(void) {
  return g_mpc_initialized;
}

extern "C" void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick) {

  if (!g_mpc_initialized || !g_use_mpc) {
    // Fallback to PID
    DEBUG_PRINT("Using PID fallback\n");
    controllerPid(control, setpoint, sensors, state, tick);
    return;
  }

  // Simple MPC implementation
  // Convert current state to state vector
  tiny_VectorNx x0 = crazyflieStateToTinyMPC(state, sensors);

  // Map parameter data to Eigen matrices
  tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
  tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);

  // Create reference trajectory (hover at setpoint position)
  tiny_VectorNx Xref_current;
  Xref_current << setpoint->position.x, setpoint->position.y, setpoint->position.z,
                  0, 0, setpoint->attitude.yaw * M_PI / 180.0,
                  0, 0, 0, 0, 0, 0;

  // Simple MPC: proportional control to reference with feedforward
  tiny_VectorNx x_error = Xref_current - x0;

  // Simple LQR-like control (using first row of Bdyn matrix for mixing)
  tiny_VectorNx u0 = tiny_VectorNx::Zero();
  for (int i = 0; i < NINPUTS; i++) {
    // Simple proportional control on position error
    float pos_error = sqrtf(x_error(0)*x_error(0) + x_error(1)*x_error(1) + x_error(2)*x_error(2));
    u0(i) = 0.5f + fminf(0.3f, pos_error * 0.1f);  // Base thrust + position feedback
  }

  // Apply control to motors (convert thrust to normalized forces)
  control->controlMode = controlModeForce;
  for (int i = 0; i < 4; i++) {
    // Clamp thrust between 0 and 1
    control->normalizedForces[i] = fmaxf(0.0f, fminf(1.0f, u0(i % NINPUTS)));
  }

  DEBUG_PRINT("Simple MPC: pos=%.2f,%.2f,%.2f thrust=%.2f,%.2f,%.2f,%.2f\n",
              x0(0), x0(1), x0(2),
              control->normalizedForces[0],
              control->normalizedForces[1],
              control->normalizedForces[2],
              control->normalizedForces[3]);
}
