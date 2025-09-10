/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_tinympc_basic.cpp - Basic TinyMPC controller implementation for new firmware
 */

#include "Eigen.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "system.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

// TinyMPC and PID controllers
#include "tinympc/types.hpp"
#include "tinympc/admm.hpp"
#include "controller_pid.h"

// Parameter files
#include "quadrotor_50hz_params_unconstrained.hpp"
#include "quadrotor_50hz_params_constrained.hpp"
#include "quadrotor_50hz_line_9s_xyz.hpp"

// Debug module name
#define DEBUG_MODULE "TINYMPC"
#include "debug.h"

// Control rate
#define MPC_RATE RATE_100_HZ
#define LOWLEVEL_RATE RATE_500_HZ

// Task configuration
#define TINYMPC_TASK_STACKSIZE 1000
#define TINYMPC_TASK_PRI 3
#define TINYMPC_TASK_NAME "TINYMPC"

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void *parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

// Structs to keep track of data sent to and received by stabilizer loop
// Stabilizer loop updates/uses these
control_t control_data;
setpoint_t setpoint_data;
sensorData_t sensors_data;
state_t state_data;
tiny_VectorNx mpc_setpoint;
setpoint_t mpc_setpoint_pid;

// Copies that stay constant for duration of MPC loop
setpoint_t setpoint_task;
sensorData_t sensors_task;
state_t state_task;
control_t control_task;
tiny_VectorNx mpc_setpoint_task;

/* Allocate global variables for MPC */
static tinytype u_hover[4] = {.583, .583, .583, .583};
static struct tiny_cache cache;
static struct tiny_params params;
static struct tiny_problem problem;
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_origin; // Start position for trajectory
static tiny_VectorNx current_state;

// Helper variables
static bool enable_traj = false;
static int traj_index = 0;
static int max_traj_index = 0;
static bool isInit = false;
static bool enable_mpc = false;
static uint32_t tick_count = 0;

// Forward declarations
static void resetProblem(void);
static void initializeTinyMPC(void);

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("TinyMPC Basic Controller - Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// The new controller goes here --------------------------------------------

void controllerOutOfTreeInit() {
  DEBUG_PRINT("Initializing TinyMPC Basic Controller\n");

  // Initialize PID controller as fallback
  controllerPidInit();

  // Initialize TinyMPC parameters and structures
  initializeTinyMPC();

  // Create semaphores for task synchronization
  runTaskSemaphore = xSemaphoreCreateBinary();
  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
  
  // Initialize control data
  memset(&control_data, 0, sizeof(control_data));
  memset(&setpoint_data, 0, sizeof(setpoint_data));
  memset(&sensors_data, 0, sizeof(sensors_data));
  memset(&state_data, 0, sizeof(state_data));

  // Create the controller task
  STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;
  DEBUG_PRINT("TinyMPC Basic Controller initialized\n");
}

bool controllerOutOfTreeTest() {
  // Always return true for now
  return isInit;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // For now, fall back to PID controller until TinyMPC is fully implemented
  if (!enable_mpc) {
    controllerPid(control, setpoint, sensors, state, tick);
    return;
  }

  // Copy data for task processing
  if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
    memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
    memcpy(&sensors_data, sensors, sizeof(sensorData_t));
    memcpy(&state_data, state, sizeof(state_t));
    
    tick_count = tick;
    
    // Signal the task to run MPC
    xSemaphoreGive(runTaskSemaphore);
    
    // Copy control output back
    memcpy(control, &control_data, sizeof(control_t));
    
    xSemaphoreGive(dataMutex);
  } else {
    // If we can't get the mutex, fall back to PID
    controllerPid(control, setpoint, sensors, state, tick);
  }
}

static void tinympcControllerTask(void *parameters) {
  systemWaitStart();
  
  DEBUG_PRINT("TinyMPC controller task started\n");
  
  while (1) {
    // Wait for signal from controller
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    
    if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
      // Copy data for processing
      memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
      memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
      memcpy(&state_task, &state_data, sizeof(state_t));
      
      xSemaphoreGive(dataMutex);
      
      // TODO: Implement actual TinyMPC computation here
      // For now, just compute simple hover control
      control_task.thrust = u_hover[0] * UINT16_MAX;
      control_task.roll = 0;
      control_task.pitch = 0;
      control_task.yaw = 0;
      
      // Copy result back
      if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
        memcpy(&control_data, &control_task, sizeof(control_t));
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

// Helper function to reset problem variables
static void resetProblem(void) {
  // Copy problem data
  problem.x = tiny_MatrixNxNh::Zero();
  problem.q = tiny_MatrixNxNh::Zero();
  problem.p = tiny_MatrixNxNh::Zero();
  problem.v = tiny_MatrixNxNh::Zero();
  problem.vnew = tiny_MatrixNxNh::Zero();
  problem.g = tiny_MatrixNxNh::Zero();

  problem.u = tiny_MatrixNuNhm1::Zero();
  problem.r = tiny_MatrixNuNhm1::Zero();
  problem.d = tiny_MatrixNuNhm1::Zero();
  problem.z = tiny_MatrixNuNhm1::Zero();
  problem.znew = tiny_MatrixNuNhm1::Zero();
  problem.y = tiny_MatrixNuNhm1::Zero();
}

// Initialize TinyMPC parameters and cache
static void initializeTinyMPC(void) {
  DEBUG_PRINT("Initializing TinyMPC parameters\n");
  
  // Copy cache data from problem_data/quadrotor*.hpp
  cache.Adyn[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_unconstrained_data);
  cache.Bdyn[0] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_unconstrained_data);
  cache.rho[0] = rho_unconstrained_value;
  cache.Kinf[0] = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_unconstrained_data);
  cache.Pinf[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_unconstrained_data);
  cache.Quu_inv[0] = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_unconstrained_data);
  cache.AmBKt[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_unconstrained_data);
  cache.coeff_d2p[0] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_unconstrained_data);

  cache.Adyn[1] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_constrained_data);
  cache.Bdyn[1] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_constrained_data);
  cache.rho[1] = rho_constrained_value;
  cache.Kinf[1] = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_constrained_data);
  cache.Pinf[1] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_constrained_data);
  cache.Quu_inv[1] = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_constrained_data);
  cache.AmBKt[1] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_constrained_data);
  cache.coeff_d2p[1] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_constrained_data);

  // Copy parameter data
  params.Q[0] = Eigen::Map<tiny_VectorNx>(Q_unconstrained_data);
  params.Qf[0] = Eigen::Map<tiny_VectorNx>(Qf_unconstrained_data);
  params.R[0] = Eigen::Map<tiny_VectorNu>(R_unconstrained_data);
  params.Q[1] = Eigen::Map<tiny_VectorNx>(Q_constrained_data);
  params.Qf[1] = Eigen::Map<tiny_VectorNx>(Qf_constrained_data);
  params.R[1] = Eigen::Map<tiny_VectorNu>(R_constrained_data);
  params.u_min = tiny_VectorNu(-u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3]).replicate<1, NHORIZON - 1>();
  params.u_max = tiny_VectorNu(1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3]).replicate<1, NHORIZON - 1>();
  
  for (int i = 0; i < NHORIZON; i++) {
    params.x_min[i] = tiny_VectorNc::Constant(-1000); // Currently unused
    params.x_max[i] = tiny_VectorNc::Constant(1000);
    params.A_constraints[i] = tiny_MatrixNcNx::Zero();
  }
  
  params.Xref = tiny_MatrixNxNh::Zero();
  params.Uref = tiny_MatrixNuNhm1::Zero();
  params.cache = cache;

  // Initialize problem data to zero
  resetProblem();

  problem.primal_residual_state = 0;
  problem.primal_residual_input = 0;
  problem.dual_residual_state = 0;
  problem.dual_residual_input = 0;
  problem.abs_tol = 0.001;
  problem.status = 0;
  problem.iter = 0;
  problem.max_iter = 5;
  problem.iters_check_rho_update = 10;
  problem.cache_level = 0; // 0 to use rho corresponding to inactive constraints

  // Set up reference trajectory (hover at origin for now)
  Xref_origin << 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Hover at 1m height
  params.Xref = Xref_origin.replicate<1, NHORIZON>();

  enable_traj = false;
  traj_index = 0;
  max_traj_index = NTOTAL - NHORIZON;
  
  DEBUG_PRINT("TinyMPC parameters initialized successfully\n");
}

#ifdef __cplusplus
}
#endif

// TODO: Add parameters and logging after fixing C++ compatibility issues
