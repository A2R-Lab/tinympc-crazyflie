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
#include "watchdog.h"

// TinyMPC and PID controllers
#include "tinympc/constants.hpp"
#include "tinympc/types.hpp"
#include "tinympc/admm.hpp"
#include "controller_pid.h"

// Parameter files
#include "quadrotor_50hz_params_unconstrained.hpp"
#include "quadrotor_50hz_params_constrained.hpp"
// #include "quadrotor_50hz_line_9s_xyz.hpp"  // Commented out to avoid conflict
// Trajectory references - using generated circle instead of large data files
// #include "quadrotor_50hz_ref_circle_2_5s.hpp"

// Debug module name
#define DEBUG_MODULE "TINYMPC"
#include "debug.h"

// Control rate - reduced to prevent watchdog timeout
#define MPC_RATE RATE_50_HZ  // Reduced from 100Hz to 50Hz
#define LOWLEVEL_RATE RATE_500_HZ

// Task configuration
#define TINYMPC_TASK_STACKSIZE 1000
#define TINYMPC_TASK_PRI 0
#define TINYMPC_TASK_NAME "TINYMPC"

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void *parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

// Helper functions from original code
static inline float quat_dot(quaternion_t a, quaternion_t b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline quaternion_t make_quat(float x, float y, float z, float w)
{
  quaternion_t q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

static inline quaternion_t normalize_quat(quaternion_t q)
{
  float s = 1.0f / sqrtf(quat_dot(q, q));
  return make_quat(s * q.x, s * q.y, s * q.z, s * q.w);
}

static inline struct vec quat_2_rp(quaternion_t q)
{
  struct vec v;
  v.x = q.x / q.w;
  v.y = q.y / q.w;
  v.z = q.z / q.w;
  return v;
}

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
static bool isInit = false;
static bool enable_mpc = false; // Disable MPC until we debug the crash
// static uint32_t tick_count = 0; // Removed unused variable

// Trajectory tracking variables
static uint64_t startTimestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters

// Forward declarations
static void resetProblem(void);
static void initializeTinyMPC(void);
static void UpdateHorizonReference(const setpoint_t *setpoint);

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
  
  // Initialize MPC setpoint to safe hover position
  mpc_setpoint_pid.mode.yaw = modeAbs;
  mpc_setpoint_pid.mode.x = modeAbs;
  mpc_setpoint_pid.mode.y = modeAbs;
  mpc_setpoint_pid.mode.z = modeAbs;
  mpc_setpoint_pid.position.x = 0.0f;
  mpc_setpoint_pid.position.y = 0.0f;
  mpc_setpoint_pid.position.z = 0.5f; // Safe hover height
  mpc_setpoint_pid.attitude.yaw = 0.0f;

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
  // Always update data for MPC task (like the old working code)
  if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
    memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
    memcpy(&sensors_data, sensors, sizeof(sensorData_t));
    memcpy(&state_data, state, sizeof(state_t));
    xSemaphoreGive(dataMutex);
  }

  // Enable MPC only after system is stable and in reasonable bounds
  static uint32_t stable_counter = 0;
  bool state_reasonable = (fabs(state->position.x) < 2.0f && 
                          fabs(state->position.y) < 2.0f && 
                          state->position.z > -0.5f && state->position.z < 2.0f);
  
  if (state_reasonable) stable_counter++;
  else stable_counter = 0;
  
  // Enable MPC after 2500 stable PID cycles (~5 seconds at 500Hz)  
  if (stable_counter > 2500 && !enable_mpc) {
    enable_mpc = true;
    DEBUG_PRINT("MPC enabled after stable PID operation\n");
    // Print current state for debugging
    DEBUG_PRINT("Current state: pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n", 
               (double)state->position.x, (double)state->position.y, (double)state->position.z,
               (double)state->velocity.x, (double)state->velocity.y, (double)state->velocity.z);
  }

  // Always run PID at high frequency (like the old working code)
  if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    if (enable_mpc) {
      // Use MPC-generated setpoint (setpoint from MPC task)
      controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);
    } else {  
      // Use original setpoint before MPC is enabled
      controllerPid(control, setpoint, sensors, state, tick);
    }
  }

  // Signal MPC task to compute new setpoint at 50Hz (always, like old code)
  if (RATE_DO_EXECUTE(RATE_50_HZ, tick)) {
    xSemaphoreGive(runTaskSemaphore);
  }
}

static void tinympcControllerTask(void *parameters) {
  systemWaitStart();
  
  startTimestamp = usecTimestamp();
  
  DEBUG_PRINT("TinyMPC controller task started\n");
  
  while (1) {
    // Wait for signal from controller
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    
    // Copy data for processing (always, like old code)
    if (xSemaphoreTake(dataMutex, M2T(2)) == pdTRUE) {
      memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
      memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
      memcpy(&state_task, &state_data, sizeof(state_t));
      xSemaphoreGive(dataMutex);
    }
    
    // Update MPC setpoint (always run like old code)
    if (enable_mpc) {
      
      // Convert state to TinyMPC format with strict bounds checking
      phi = quat_2_rp(normalize_quat(state_task.attitudeQuaternion)); // quaternion to Rodrigues parameters
      
      // Clamp all values to very conservative bounds to prevent solver issues
      float pos_x = fmax(-2.0f, fmin(2.0f, state_task.position.x));
      float pos_y = fmax(-2.0f, fmin(2.0f, state_task.position.y)); 
      float pos_z = fmax(0.0f, fmin(2.0f, state_task.position.z));
      
      // Clamp attitude and rates
      float phi_x = fmax(-0.5f, fmin(0.5f, phi.x));
      float phi_y = fmax(-0.5f, fmin(0.5f, phi.y));
      float phi_z = fmax(-0.5f, fmin(0.5f, phi.z));
      
      float vel_x = fmax(-2.0f, fmin(2.0f, state_task.velocity.x));
      float vel_y = fmax(-2.0f, fmin(2.0f, state_task.velocity.y));
      float vel_z = fmax(-2.0f, fmin(2.0f, state_task.velocity.z));
      
      float gyro_x = fmax(-5.0f, fmin(5.0f, radians(sensors_task.gyro.x)));
      float gyro_y = fmax(-5.0f, fmin(5.0f, radians(sensors_task.gyro.y)));
      float gyro_z = fmax(-5.0f, fmin(5.0f, radians(sensors_task.gyro.z)));
      
      problem.x.col(0) << pos_x, pos_y, pos_z,
          phi_x, phi_y, phi_z,
          vel_x, vel_y, vel_z,
          gyro_x, gyro_y, gyro_z;
      
      // Update horizon reference (for now just hover at origin)
      UpdateHorizonReference(&setpoint_task);
      
      // MPC solve - with timeout protection
      problem.iter = 0;
      mpc_start_timestamp = usecTimestamp();
      
      // First solve
      solve_admm(&problem, &params);
      
      // Check if we're taking too long (watchdog protection)
      uint32_t elapsed = usecTimestamp() - mpc_start_timestamp;
      if (elapsed < 18000) { // Increased timeout to 18ms for more stable MPC
        vTaskDelay(M2T(1));
        solve_admm(&problem, &params);
        mpc_time_us = usecTimestamp() - mpc_start_timestamp - 1000;
      } else {
        mpc_time_us = elapsed;
        // Only print timeout message occasionally to reduce spam
        static int timeout_count = 0;
        if (++timeout_count % 50 == 0) {
          DEBUG_PRINT("MPC solve timeout (%d), using single iteration\n", (int)timeout_count);
        }
      }
      
      // Extract MPC setpoint for PID controller with bounds checking
      mpc_setpoint = problem.x.col(NHORIZON-1);
      
      // Sanity check the MPC setpoint for NaN/Inf values
      for (int i = 0; i < NSTATES; i++) {
        if (isnan(mpc_setpoint(i)) || isinf(mpc_setpoint(i))) {
          DEBUG_PRINT("MPC setpoint contains invalid values, using hover setpoint\n");
          mpc_setpoint(0) = 0.0f; // x position
          mpc_setpoint(1) = 0.0f; // y position  
          mpc_setpoint(2) = 0.5f; // z position (hover)
          mpc_setpoint(3) = 0.0f; // phi_x
          mpc_setpoint(4) = 0.0f; // phi_y
          mpc_setpoint(5) = 0.0f; // phi_z (yaw)
          for (int j = 6; j < NSTATES; j++) {
            mpc_setpoint(j) = 0.0f; // velocities and rates
          }
          break;
        }
      }
      
      // If solver is consistently timing out, use safe hover setpoint
      static int consecutive_timeouts = 0;
      if (mpc_time_us > 18000) {
        consecutive_timeouts++;
      } else {
        consecutive_timeouts = 0;
      }
      
      if (consecutive_timeouts > 10) {
        // Use safe hover setpoint
        mpc_setpoint_pid.position.x = 0.0f;
        mpc_setpoint_pid.position.y = 0.0f;
        mpc_setpoint_pid.position.z = 0.3f; // Safe hover at 30cm
        mpc_setpoint_pid.attitude.yaw = 0.0f;
      } else {
        // Use MPC result with very conservative bounds checking
        mpc_setpoint_pid.position.x = fmax(-1.0f, fmin(1.0f, mpc_setpoint(0)));
        mpc_setpoint_pid.position.y = fmax(-1.0f, fmin(1.0f, mpc_setpoint(1)));
        mpc_setpoint_pid.position.z = fmax(0.2f, fmin(0.8f, mpc_setpoint(2))); // Conservative height range
        
        // Clamp yaw to prevent extreme rotation
        float yaw_setpoint = mpc_setpoint(5);
        while (yaw_setpoint > M_PI_F) yaw_setpoint -= 2*M_PI_F;
        while (yaw_setpoint < -M_PI_F) yaw_setpoint += 2*M_PI_F;
        mpc_setpoint_pid.attitude.yaw = fmax(-M_PI_F/4, fmin(M_PI_F/4, yaw_setpoint)); // Limit to ±45°
        
        // Debug output for first few MPC iterations
        static int mpc_debug_count = 0;
        if (mpc_debug_count < 10) {
          DEBUG_PRINT("MPC setpoint %d: pos=(%.3f,%.3f,%.3f) yaw=%.3f raw=(%.3f,%.3f,%.3f)\n", 
                     mpc_debug_count,
                     (double)mpc_setpoint_pid.position.x, (double)mpc_setpoint_pid.position.y, (double)mpc_setpoint_pid.position.z,
                     (double)mpc_setpoint_pid.attitude.yaw,
                     (double)mpc_setpoint(0), (double)mpc_setpoint(1), (double)mpc_setpoint(2));
          mpc_debug_count++;
        }
      }
    } // end if (enable_mpc)
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

// Helper function to update horizon reference
static void UpdateHorizonReference(const setpoint_t *setpoint) {
  // Simple hover behavior - all horizon steps use the same hover setpoint
  params.Xref = Xref_origin.replicate<1, NHORIZON>();
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
  problem.max_iter = 3; // Reduced from 5 to prevent watchdog timeout
  problem.iters_check_rho_update = 10;
  problem.cache_level = 0; // 0 to use rho corresponding to inactive constraints

  // Set up reference trajectory (hover close to ground initially)
  Xref_origin << 0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Hover at 30cm height (safer)
  params.Xref = Xref_origin.replicate<1, NHORIZON>();

  // Initialization complete
  
  DEBUG_PRINT("TinyMPC parameters initialized successfully\n");
}

#ifdef __cplusplus
#endif

