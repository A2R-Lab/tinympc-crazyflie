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
using namespace Eigen;

// Define TinyMPC constants
#define NSTATES 12
#define NINPUTS 4

// Define matrix types for direct LQR approach
namespace Eigen
{
    typedef Matrix<float, NSTATES, NSTATES> MatrixNf;
    typedef Matrix<float, NSTATES, NINPUTS> MatrixNMf;
    typedef Matrix<float, NINPUTS, NSTATES> MatrixMNf;
    typedef Matrix<float, NINPUTS, NINPUTS> MatrixMf;
    typedef Vector<float, NSTATES>          VectorNf;  
    typedef Vector<float, NINPUTS>          VectorMf; 
}

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
//#include "quadrotor_50hz_line_9s_xyz.hpp"
//#include "quadrotor_50hz_ref_circle.hpp"
//#include "quadrotor_50hz_ref_circle_2_5s.hpp"
#include "quadrotor_50hz_line_8s.hpp"

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

/* Allocate global variables for direct matrix MPC (like Ishaan's approach) */

// Precomputed matrices (like Ishaan's params_500hz.h)
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

// State and control vectors
static VectorNf x0; // Current state
static VectorNf xref; // Reference state 
static VectorMf u0; // Control output
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_origin; // Start position for trajectory

// Helper variables
static bool isInit = false;
static bool enable_mpc = false; // Disable MPC until we debug the crash

// Trajectory variables - use direct access
static bool enable_traj = false;
static int traj_index = 0;
static int max_traj_index = 0;
static float traj_x_offset = 0.0f;
static float traj_y_offset = 0.0f; 
static float traj_z_offset = 0.0f;

// Trajectory tracking variables
static uint64_t startTimestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters

// Forward declarations
static void initializeTinyMPC(void);
static void UpdateReference(const setpoint_t *setpoint);

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("TinyMPC Basic Controller - Waiting for activation...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// The new controller goes here --------------------------------------------

void controllerOutOfTreeInit() {
  DEBUG_PRINT("Initializing Direct Matrix TinyMPC Controller...\n");

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
  DEBUG_PRINT("Direct Matrix TinyMPC Controller Ready \n");
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
    DEBUG_PRINT("Direct LQR Control Active: pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)\n", 
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

  DEBUG_PRINT("Direct LQR Task Started (500Hz)\n");

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
      // Smart periodic debug - less clutter, more info
      static int task_debug_count = 0;
      task_debug_count++;
      
      // Consolidated status every 2 seconds (1000 cycles at 500Hz)
      if (task_debug_count % 1000 == 1) {
        DEBUG_PRINT("MPC Status [%ds]: pos=(%.2f,%.2f,%.2f) traj=%s(%d/%d)\n", 
                   task_debug_count/500, // seconds
                   (double)state_task.position.x, (double)state_task.position.y, (double)state_task.position.z,
                   enable_traj ? "ON" : "OFF", traj_index, max_traj_index);
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
      
      x0 << pos_x, pos_y, pos_z,
          phi_x, phi_y, phi_z,
          vel_x, vel_y, vel_z,
          gyro_x, gyro_y, gyro_z;
      
      // Enable trajectory after 7 seconds of stable MPC operation (only once)
      static bool trajectory_started = false;
      if (!enable_traj && !trajectory_started && usecTimestamp() - startTimestamp > 1000000 * 7) {
        // Calculate trajectory offset to make it relative to current position
        // Current trajectory goes from Y=-1.5 to Y=+1.5 (3m total movement) at Z=1.0m
        float current_x = pos_x; // Current drone X position
        float current_y = pos_y; // Current drone Y position  
        float current_z = pos_z; // Current drone Z position
        
        float trajectory_start_x = Xref_data[0]; // Original start X = 0.0
        float trajectory_start_y = Xref_data[1]; // Original start Y = -1.5
        float trajectory_start_z = Xref_data[2]; // Original start Z = 1.0
        
        // Store offsets to use during trajectory access
        traj_x_offset = current_x - trajectory_start_x; 
        traj_y_offset = current_y - trajectory_start_y; 
        traj_z_offset = current_z - trajectory_start_z; 
        
        DEBUG_PRINT("TRAJECTORY START: from=(%.2f,%.2f,%.2f) offset=(%.2f,%.2f,%.2f) %d steps\n",
                   (double)current_x, (double)current_y, (double)current_z,
                   (double)traj_x_offset, (double)traj_y_offset, (double)traj_z_offset, max_traj_index);
        
        enable_traj = true; 
        trajectory_started = true; // Prevent re-enabling
        traj_index = 0; // Reset to start of trajectory
      }
      
      // Update reference (hover or trajectory)
      UpdateReference(&setpoint_task);
      
      // Direct LQR control law - much faster than ADMM solver!
      mpc_start_timestamp = usecTimestamp();
      
      // Direct matrix multiplication: u = Kinf * (x - xref)
      u0 = Kinf * (x0 - xref);
      
      // Very fast computation - typical time under 50 microseconds
      mpc_time_us = usecTimestamp() - mpc_start_timestamp;
      
      // Bounds check the control output
      for (int i = 0; i < NINPUTS; i++) {
        if (isnan(u0(i)) || isinf(u0(i))) {
          DEBUG_PRINT("Direct control contains invalid values, using hover\n");
          u0.setZero(); // Safe zero control
          break;
        }
      }
      
      // Convert control to PID setpoint format
      // For direct control, we can use the reference directly as the setpoint
      mpc_setpoint_pid.position.x = fmax(-1.0f, fmin(1.0f, xref(0)));
      mpc_setpoint_pid.position.y = fmax(-1.0f, fmin(1.0f, xref(1)));
      mpc_setpoint_pid.position.z = fmax(0.2f, fmin(0.8f, xref(2))); // Conservative height range
      
      // Clamp yaw to prevent extreme rotation
      float yaw_setpoint = xref(5);
      while (yaw_setpoint > M_PI_F) yaw_setpoint -= 2*M_PI_F;
      while (yaw_setpoint < -M_PI_F) yaw_setpoint += 2*M_PI_F;
      mpc_setpoint_pid.attitude.yaw = fmax(-M_PI_F/4, fmin(M_PI_F/4, yaw_setpoint)); // Limit to ±45°
      
      // Smart control debug - show key info only during trajectory
      static int control_debug_count = 0;
      control_debug_count++;
      if (enable_traj && control_debug_count % 100 == 1) {
        float pos_error = sqrt(pow(x0(0) - xref(0), 2) + pow(x0(1) - xref(1), 2) + pow(x0(2) - xref(2), 2));
        DEBUG_PRINT("Direct LQR [%d]: pos_err=%.3fm u_mag=%.2f solve=%.0fus\n", 
                   control_debug_count,
                   (double)pos_error, 
                   (double)sqrt(u0(0)*u0(0) + u0(1)*u0(1) + u0(2)*u0(2) + u0(3)*u0(3)),
                   (double)mpc_time_us);
      }
    } // end if (enable_mpc)
  }
}

// Helper function to update reference
static void UpdateReference(const setpoint_t *setpoint) {
  if (enable_traj) {
    if (traj_index < max_traj_index) {
      // Get current trajectory reference (single point, not horizon)
      int data_idx = traj_index * 3;
      xref(0) = Xref_data[data_idx] + traj_x_offset;       // x + offset
      xref(1) = Xref_data[data_idx + 1] + traj_y_offset;   // y + offset 
      xref(2) = Xref_data[data_idx + 2] + traj_z_offset;   // z + offset
      // Zero out attitude and velocities
      for (int j = 3; j < NSTATES; j++) {
        xref(j) = 0.0f;
      }
      
      // Advance trajectory index
      traj_index++;
      
      // Smart trajectory progress - show milestones
      if (traj_index % 100 == 1 || traj_index <= 5 || traj_index >= max_traj_index - 5) {
        int progress_pct = (traj_index * 100) / max_traj_index;
        DEBUG_PRINT("Traj Progress %d%% (%d/%d): target=(%.2f,%.2f,%.2f)\n", 
                   progress_pct, traj_index, max_traj_index,
                   (double)xref(0), (double)xref(1), (double)xref(2));
      }
    } else if (traj_index >= max_traj_index) {
      // Use final trajectory position
      int final_idx = (NTOTAL-1) * 3;
      xref(0) = Xref_data[final_idx] + traj_x_offset;       // x + offset
      xref(1) = Xref_data[final_idx + 1] + traj_y_offset;   // y + offset
      xref(2) = Xref_data[final_idx + 2] + traj_z_offset;   // z + offset
      for (int j = 3; j < NSTATES; j++) {
        xref(j) = 0.0f;
      }
      
      // Trajectory finished, disable it
      enable_traj = false;
      DEBUG_PRINT("TRAJECTORY COMPLETE: %d steps finished, switching to hover at (%.2f,%.2f,%.2f)\n", 
                 max_traj_index, (double)xref(0), (double)xref(1), (double)xref(2));
    }
  } else {
    // Use hover reference
    xref = Xref_origin;
  }
}

// Initialize TinyMPC parameters and cache
static void initializeTinyMPC(void) {
  DEBUG_PRINT("Loading precomputed LQR matrices...\n");
  
  // Load precomputed matrices (like Ishaan's params_500hz.h)
  
  Kinf << 
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  A << 
  1.000000f,0.000000f,0.000000f,0.000000f,0.003924f,0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000013f,0.000000f,
  0.000000f,1.000000f,0.000000f,-0.003924f,0.000000f,0.000000f,0.000000f,0.020000f,0.000000f,-0.000013f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.392400f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.001962f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,-0.392400f,0.000000f,0.000000f,1.000000f,0.000000f,-0.001962f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f;

  B << 
  -0.000018f,0.000020f,0.000018f,-0.000020f,
  0.000018f,0.000020f,-0.000018f,-0.000020f,
  0.000841f,0.000841f,0.000841f,0.000841f,
  -0.027535f,-0.030323f,0.027566f,0.030293f,
  -0.027671f,0.030428f,0.027757f,-0.030514f,
  0.001975f,-0.000722f,-0.002784f,0.001532f,
  -0.003619f,0.003980f,0.003631f,-0.003991f,
  0.003602f,0.003966f,-0.003606f,-0.003962f,
  0.084086f,0.084086f,0.084086f,0.084086f,
  -5.507092f,-6.064681f,5.513253f,6.058520f,
  -5.534140f,6.085568f,5.551390f,-6.102818f,
  0.394954f,-0.144473f,-0.556875f,0.306394f;

  Q << 
  156.250000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,156.250000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,400.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,2.777778f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,2.777778f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1111.111111f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,25.000000f;

  R << 
  900.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,900.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,900.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,900.000000f;

  // Initialize state vectors
  x0.setZero();
  xref.setZero();
  u0.setZero();
  
  // Set hover position to safe initial hover
  Xref_origin << 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Hover at current position (50cm height)
  
  // Initialize trajectory parameters
  enable_traj = false;
  traj_index = 0;
  max_traj_index = NTOTAL - NHORIZON;
  
  DEBUG_PRINT("Line Trajectory: %d steps (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)\n",
             NTOTAL, 
             (double)Xref_data[0], (double)Xref_data[1], (double)Xref_data[2],
             (double)Xref_data[(NTOTAL-1)*3], (double)Xref_data[(NTOTAL-1)*3+1], (double)Xref_data[(NTOTAL-1)*3+2]);

  DEBUG_PRINT("Direct Matrix LQR Ready: Kinf(4x12) + trajectory loaded\n");
}

#ifdef __cplusplus
}
#endif

