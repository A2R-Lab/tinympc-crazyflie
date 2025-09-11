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

// Trajectory files
#include "quadrotor_50hz_line_9s_xyz.hpp"

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

// Trajectory variables
static Eigen::Matrix<tinytype, 3, NTOTAL, Eigen::ColMajor> Xref_total; // Full trajectory data
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_end; // End position for trajectory
static bool enable_traj = false;
static int traj_index = 0;
static int max_traj_index = 0;

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
      // Add periodic debug to ensure MPC task is running
      static int task_debug_count = 0;
      if (++task_debug_count % 100 == 1) {
        DEBUG_PRINT("MPC task running: cycle %d, enable_traj=%d, traj_index=%d\n", 
                   task_debug_count, enable_traj, traj_index);
      }
      
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
      
      // Enable trajectory after 7 seconds of stable MPC operation (only once)
      static bool trajectory_started = false;
      if (!enable_traj && !trajectory_started && usecTimestamp() - startTimestamp > 1000000 * 7) {
        DEBUG_PRINT("Enable line trajectory!\n");
        
        // Make trajectory relative to current position
        // Current trajectory goes from Y=-1.5 to Y=+1.5 (3m total movement) at Z=1.0m
        // Modify it to start from current position and height
        float current_x = pos_x; // Current drone X position
        float current_y = pos_y; // Current drone Y position  
        float current_z = pos_z; // Current drone Z position
        
        float trajectory_start_x = Xref_total(0, 0); // Original start X = 0.0
        float trajectory_start_y = Xref_total(1, 0); // Original start Y = -1.5
        float trajectory_start_z = Xref_total(2, 0); // Original start Z = 1.0
        
        float x_offset = current_x - trajectory_start_x; // X offset to apply
        float y_offset = current_y - trajectory_start_y; // Y offset to apply
        float z_offset = current_z - trajectory_start_z; // Z offset to apply
        
        // Shift entire trajectory to start from current position
        for (int i = 0; i < NTOTAL; i++) {
          Xref_total(0, i) += x_offset; // Shift X coordinates
          Xref_total(1, i) += y_offset; // Shift Y coordinates
          Xref_total(2, i) += z_offset; // Shift Z coordinates
        }
        
        DEBUG_PRINT("Trajectory shifted: Y(%.3f→%.3f), Z(%.3f→%.3f), offsets=(%.3f,%.3f,%.3f)\n",
                   (double)trajectory_start_y, (double)current_y, 
                   (double)trajectory_start_z, (double)current_z,
                   (double)x_offset, (double)y_offset, (double)z_offset);
        DEBUG_PRINT("Trajectory limits: NTOTAL=%d, NHORIZON=%d, max_traj_index=%d\n", 
                   NTOTAL, NHORIZON, max_traj_index);
        
        enable_traj = true; 
        trajectory_started = true; // Prevent re-enabling
        traj_index = 0; // Reset to start of trajectory
      }
      
      // Update horizon reference (hover or trajectory) - but don't advance trajectory yet
      UpdateHorizonReference(&setpoint_task);
      
      // MPC solve - with timeout protection
      problem.iter = 0;
      mpc_start_timestamp = usecTimestamp();
      
      // First solve
      solve_admm(&problem, &params);
      
      // Check if we're taking too long (watchdog protection)
      uint32_t elapsed = usecTimestamp() - mpc_start_timestamp;
      bool solve_successful = false;
      if (elapsed < 40000) { // Increased timeout to 40ms to allow MPC solver to complete
        vTaskDelay(M2T(1));
        solve_admm(&problem, &params);
        mpc_time_us = usecTimestamp() - mpc_start_timestamp - 1000;
        solve_successful = true;
      } else {
        mpc_time_us = elapsed;
        solve_successful = false;
        // Print timeout messages to understand solver performance
        static int timeout_count = 0;
        if (++timeout_count % 10 == 0) {
          DEBUG_PRINT("MPC timeout #%d (%.1fms), enable_traj=%d, traj_index=%d\n", 
                     (int)timeout_count, (double)elapsed/1000.0, enable_traj, traj_index);
        }
        // Also print first few timeouts immediately
        if (timeout_count <= 5) {
          DEBUG_PRINT("MPC timeout early #%d (%.1fms) enable_traj=%d\n", 
                     (int)timeout_count, (double)elapsed/1000.0, enable_traj);
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
      if (mpc_time_us > 35000) { // Only count as timeout if >35ms
        consecutive_timeouts++;
      } else {
        consecutive_timeouts = 0;
      }
      
      if (consecutive_timeouts > 20) { // Allow more timeouts before emergency mode
        // Use safe hover setpoint
        mpc_setpoint_pid.position.x = 0.0f;
        mpc_setpoint_pid.position.y = 0.0f;
        mpc_setpoint_pid.position.z = 0.3f; // Safe hover at 30cm
        mpc_setpoint_pid.attitude.yaw = 0.0f;
        
        // Debug output when using emergency hover
        static int emergency_debug_count = 0;
        if (++emergency_debug_count % 50 == 1) {
          DEBUG_PRINT("EMERGENCY HOVER: consecutive_timeouts=%d, mpc_time_us=%d\n", 
                     consecutive_timeouts, (int)mpc_time_us);
        }
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
        
        // Debug output for MPC setpoints (more frequent during trajectory)
        static int mpc_debug_count = 0;
        mpc_debug_count++;
        if ((enable_traj && mpc_debug_count % 25 == 1) || mpc_debug_count <= 10) {
          DEBUG_PRINT("MPC setpoint %d: pos=(%.3f,%.3f,%.3f) solve_time=%.1fms traj=%d\n", 
                     mpc_debug_count,
                     (double)mpc_setpoint_pid.position.x, (double)mpc_setpoint_pid.position.y, (double)mpc_setpoint_pid.position.z,
                     (double)mpc_time_us/1000.0, enable_traj);
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
  if (enable_traj) {
    if (traj_index < max_traj_index) {
      // Bounds check to prevent matrix access out of range
      if (traj_index + NHORIZON <= NTOTAL) {
        // Extract trajectory horizon: copy XYZ positions and zero out attitude/velocities
        params.Xref.block<3, NHORIZON>(0,0) = Xref_total.block<3, NHORIZON>(0, traj_index);
        // Zero out attitude and velocities (rows 3-11)
        params.Xref.block<9, NHORIZON>(3,0).setZero();
      } else {
        // Near end of trajectory - use end position 
        params.Xref = Xref_end.replicate<1, NHORIZON>();
        DEBUG_PRINT("Trajectory near end: traj_index=%d, using end position\n", traj_index);
      }
      
      // Advance trajectory index every cycle (like Ishaan's original)
      traj_index++;
      
      // Debug output for trajectory steps (every 50 steps to reduce spam)
      if (traj_index % 50 == 1 || traj_index <= 10) {
        DEBUG_PRINT("Trajectory advance %d/%d: pos=(%.3f,%.3f,%.3f)\n", 
                   traj_index, max_traj_index,
                   (double)params.Xref(0,0), (double)params.Xref(1,0), (double)params.Xref(2,0));
      }
    } else if (traj_index >= max_traj_index) {
      // Use end position for remaining horizon
      params.Xref = Xref_end.replicate<1, NHORIZON>();
      // Trajectory finished, disable it
      enable_traj = false;
      DEBUG_PRINT("Line trajectory completed: traj_index=%d >= max_traj_index=%d, switching to hover\n", 
                 traj_index, max_traj_index);
    }
  } else {
    // Simple hover behavior - all horizon steps use the same hover setpoint
    params.Xref = Xref_origin.replicate<1, NHORIZON>();
  }
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

  // Set up reference trajectory data
  // Xref_data is [NTOTAL*3] stored row-major: [x0,y0,z0, x1,y1,z1, ...]
  // Map as NTOTAL x 3 matrix, then transpose to get 3 x NTOTAL
  Xref_total = Eigen::Map<Eigen::Matrix<tinytype, NTOTAL, 3, Eigen::RowMajor>>(Xref_data).transpose();
  
  // Set hover position to safe initial hover (not trajectory start)
  Xref_origin << 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Hover at current position (50cm height)
  Xref_end << Xref_total.col(NTOTAL-1).head(3), 0, 0, 0, 0, 0, 0, 0, 0, 0; // Go to xyz end of trajectory
  params.Xref = Xref_origin.replicate<1, NHORIZON>();
  
  // Initialize trajectory parameters
  enable_traj = false;
  traj_index = 0;
  max_traj_index = NTOTAL - NHORIZON;
  
  DEBUG_PRINT("Line trajectory loaded: %d total steps, start=(%.3f,%.3f,%.3f), end=(%.3f,%.3f,%.3f)\n",
             NTOTAL, 
             (double)Xref_total(0,0), (double)Xref_total(1,0), (double)Xref_total(2,0),
             (double)Xref_total(0,NTOTAL-1), (double)Xref_total(1,NTOTAL-1), (double)Xref_total(2,NTOTAL-1));

  // Initialization complete
  
  DEBUG_PRINT("TinyMPC parameters initialized successfully\n");
}

#ifdef __cplusplus
}
#endif

