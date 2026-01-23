/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * controller_tinympc.c - App layer application of TinyMPC.
 */

/**
 * Single lap
 */

#include "Eigen.h"

// TinyMPC headers (C++, must be before extern "C")
#include "tinympc/admm.hpp"
#include "tinympc/psd_support.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "config.h"
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
#include "eventtrigger.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h" // needed to compile Cpp to C

// PID controller
#include "controller_pid.h"

// Params
// #include "quadrotor_10hz_params.hpp"
// #include "quadrotor_50hz_params.hpp" // rho = 65
// #include "quadrotor_50hz_params_2.hpp" // rho = 5, passive
// #include "quadrotor_50hz_params_3.hpp" // rho = 5, aggressive
// #include "quadrotor_50hz_params_constraints.hpp"
// #include "quadrotor_250hz_params.hpp"
#include "quadrotor_50hz_params_unconstrained.hpp"
#include "quadrotor_50hz_params_constrained.hpp"

// Trajectory
// #include "quadrotor_100hz_ref_hover.hpp"
// #include "quadrotor_50hz_ref_circle.hpp"
// #include "quadrotor_50hz_ref_circle_2_5s.hpp"
// #include "quadrotor_50hz_line_5s.hpp"
// #include "quadrotor_50hz_line_8s.hpp"
#include "quadrotor_50hz_line_9s_xyz.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MPCTASK"
#include "debug.h"

// #define MPC_RATE RATE_250_HZ  // control frequency
// #define MPC_RATE RATE_50_HZ  // 50Hz gives 20ms period, solve is ~11ms
#define MPC_RATE RATE_25_HZ  // 25Hz gives 40ms period for PSD
// #define MPC_RATE RATE_100_HZ
#define LOWLEVEL_RATE RATE_500_HZ

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void *parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

// // declares eventTrigger_[name] and eventTrigger_[name]_payload
// EVENTTRIGGER(horizon_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
// EVENTTRIGGER(horizon_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
// EVENTTRIGGER(horizon_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
// EVENTTRIGGER(horizon_part3, float, h15, float, h16, float, h17, float, h18, float, h19);
// EVENTTRIGGER(iters_event, int32, iters);
// EVENTTRIGGER(cache_level_event, int32, level);

// declares eventTrigger_[name] and eventTrigger_[name]_payload
EVENTTRIGGER(horizon_x_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_x_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_x_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_x_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(horizon_y_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_y_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_y_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_y_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(horizon_z_part1, float, h0, float, h1, float, h2, float, h3, float, h4);
EVENTTRIGGER(horizon_z_part2, float, h5, float, h6, float, h7, float, h8, float, h9);
EVENTTRIGGER(horizon_z_part3, float, h10, float, h11, float, h12, float, h13, float, h14);
EVENTTRIGGER(horizon_z_part4, float, h15, float, h16, float, h17, float, h18, float, h19);
EVENTTRIGGER(problem_data_event, int32, solvetime_us, int32, iters, int32, cache_level);
EVENTTRIGGER(problem_residuals_event, float, prim_resid_state, float, prim_resid_input, float, dual_resid_state, float, dual_resid_input);



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
// static tinytype u_hover[4] = {.65, .65, .65, .65};
static tinytype u_hover[4] = {.583, .583, .583, .583};
static struct tiny_cache cache;
static struct tiny_params params;
static struct tiny_problem problem;
static tiny_MatrixNxNh problem_x;
// static float horizon_nh_z;
static float init_vel_z;
// static Eigen::Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor> Xref_total;
static Eigen::Matrix<tinytype, 3, NTOTAL, Eigen::ColMajor> Xref_total;
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_origin; // Start position for trajectory
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_end; // End position for trajectory
static tiny_VectorNu u_lqr;
static tiny_VectorNx current_state;

// Helper variables
static bool enable_traj = true;
static bool mpc_has_run = false; // Flag to track if MPC has computed at least once
static int traj_index = 0;
static int max_traj_index = 0;
static float traj_speed = 0.2f; // m/s
static float traj_dist = 1.0f;  // m
static float traj_height = 0.5f;
static float traj_hold_time = 2.0f; // seconds
static uint32_t last_controller_tick = 0;
static uint32_t controller_activate_tick = 0;
// static int mpc_steps_taken = 0;
static uint64_t startTimestamp;
// static uint32_t timestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
static bool isInit = false;
static int prev_cache_level = 0; // Track cache_level changes
static uint8_t enable_obs_constraint = 1; // Static obstacle constraint enable
static uint8_t enable_psd = 1; // PSD enabled (runs every 5 iters)

// Static obstacle (disk) parameters for LTV linear constraints
static Eigen::Matrix<tinytype, 3, 1> obs_center;
static Eigen::Matrix<tinytype, 3, 1> xc;
static Eigen::Matrix<tinytype, 3, 1> a_norm;
static Eigen::Matrix<tinytype, 3, 1> q_c;
static float r_obs = 0.35f;           // Larger radius for more aggressive avoidance
static float obs_activation_margin = 0.15f; // Smaller = later/faster swerve

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

static inline void fill_hold_setpoint(setpoint_t *sp, const state_t *state)
{
  memset(sp, 0, sizeof(setpoint_t));
  sp->mode.yaw = modeAbs;
  sp->mode.x = modeAbs;
  sp->mode.y = modeAbs;
  sp->mode.z = modeAbs;
  sp->position.x = state->position.x;
  sp->position.y = state->position.y;
  sp->position.z = state->position.z;
  sp->attitude.yaw = state->attitude.yaw;
}

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));
  }
}

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


void controllerOutOfTreeInit(void)
{

  controllerPidInit();

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
  for (int i = 0; i < NHORIZON; i++)
  {
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
  problem.cache_level = 0; // 0 to use rho corresponding to inactive constraints (1 to use rho corresponding to active constraints)

  // Initialize straight-line reference (generated, not from table)
  Xref_origin << 0, 0, traj_height, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Xref_end << traj_dist, 0, traj_height, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  params.Xref = Xref_origin.replicate<1, NHORIZON>();

  // Initialize mpc_setpoint to the origin reference to avoid garbage values on first call
  mpc_setpoint = Xref_origin;

  enable_traj = true;
  mpc_has_run = false;
  traj_index = 0;
  max_traj_index = (int)((traj_dist / traj_speed + traj_hold_time) * MPC_RATE);

  // Static obstacle further along path so swerve happens later
  // Offset y=0.1 so drone swerves to negative y (left)
  obs_center << 0.7f, 0.1f, 0.5f;

  // Initialize PSD constraints (disabled by default, enable via enable_psd flag)
  problem.en_psd = enable_psd;
  if (enable_psd) {
    tinytype rho_psd = 10.0f;  // PSD penalty parameter (tune as needed)
    tiny_enable_psd(&problem, &params, rho_psd);
    // Set PSD obstacle (same as LTV obstacle)
    tiny_set_psd_obstacle(&problem, obs_center(0), obs_center(1), r_obs);
    DEBUG_PRINT("PSD enabled with rho_psd=%.1f, obs=(%.2f,%.2f,r=%.2f)\n", 
                (double)rho_psd, (double)obs_center(0), (double)obs_center(1), (double)r_obs);
  }

  /* Begin task initialization */
  runTaskSemaphore = xSemaphoreCreateBinary();
  // ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;
  /* End of task initialization */
}

static void UpdateHorizonReference(const setpoint_t *setpoint)
{
  if (enable_traj)
  {
    const float dt = 1.0f / MPC_RATE;
    const float travel_time = traj_dist / traj_speed;
    const float base_t = traj_index * dt;
    for (int i = 0; i < NHORIZON; ++i) {
      float t = base_t + i * dt;
      float x = (t < travel_time) ? (traj_speed * t) : traj_dist;
      params.Xref(0, i) = x;
      params.Xref(1, i) = 0.0f;
      params.Xref(2, i) = traj_height;
    }

    if (traj_index < max_traj_index) {
      traj_index++;
    } else {
      // Trajectory done - disable trajectory to trigger motor kill
      static bool traj_done_msg = false;
      if (!traj_done_msg) {
        DEBUG_PRINT("TRAJ DONE: idx=%d, max=%d\n", traj_index, max_traj_index);
        traj_done_msg = true;
      }
      enable_traj = false;
      enable_obs_constraint = 0;
      params.Xref = Xref_end.replicate<1, NHORIZON>();
    }
  }
  else
  {
    params.Xref = Xref_origin.replicate<1, NHORIZON>();
  }
}

bool controllerOutOfTreeTest()
{
  // Always return true
  return true;
}

static void tinympcControllerTask(void *parameters)
{
  // systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextMpcMs = nowMs;

  startTimestamp = usecTimestamp();

  static uint32_t task_loop_count = 0;
  while (true)
  {
    // Update task data with most recent stabilizer loop data
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    
    task_loop_count++;
    if (task_loop_count <= 3) {
      DEBUG_PRINT("MPC task loop %lu\n", task_loop_count);
    }

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
    memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
    memcpy(&state_task, &state_data, sizeof(state_t));
    memcpy(&control_task, &control_data, sizeof(control_t));
    xSemaphoreGive(dataMutex);

    nowMs = T2M(xTaskGetTickCount());
    if (nowMs >= nextMpcMs)
    {
      nextMpcMs = nowMs + (1000.0f / MPC_RATE);

      // Comment out when avoiding dynamic obstacle
      // Uncomment if following reference trajectory
      if (usecTimestamp() - startTimestamp > 1000000 * 2 && traj_index == 0 && !enable_traj)
      {
        DEBUG_PRINT("Enable trajectory!\n");
        enable_traj = true;
      }

      // Reset dual variables when switching modes or when in unconstrained mode
      if (problem.cache_level != prev_cache_level) {
        DEBUG_PRINT("Cache level changed: %d -> %d\n", prev_cache_level, problem.cache_level);
        // Reset dual variables when switching modes to avoid instability
        problem.y = tiny_MatrixNuNhm1::Zero();
        problem.g = tiny_MatrixNxNh::Zero();
        problem.v = tiny_MatrixNxNh::Zero();
        problem.vnew = tiny_MatrixNxNh::Zero();
        problem.z = tiny_MatrixNuNhm1::Zero();
        problem.znew = tiny_MatrixNuNhm1::Zero();
        prev_cache_level = problem.cache_level;
      }
      
      if (problem.cache_level == 0) {
        problem.y = tiny_MatrixNuNhm1::Zero();
        problem.g = tiny_MatrixNxNh::Zero();
      }

      // TODO: predict into the future and set initial x to wherever we think we'll be
      //    by the time we're done computing the input for that state. If we just set
      //    initial x to current state then by the time we compute the optimal input for
      //    that state we'll already be at the next state and there will be a mismatch
      //    in the input we're using for our current state.
      // Set initial x to current state
      phi = quat_2_rp(normalize_quat(state_task.attitudeQuaternion)); // quaternion to Rodrigues parameters
      problem.x.col(0) << state_task.position.x, state_task.position.y, state_task.position.z,
          phi.x, phi.y, phi.z,
          state_task.velocity.x, state_task.velocity.y, state_task.velocity.z,
          radians(sensors_task.gyro.x), radians(sensors_task.gyro.y), radians(sensors_task.gyro.z);

      if (task_loop_count <= 3) {
        DEBUG_PRINT("x0: pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)\n",
                    (double)state_task.position.x, (double)state_task.position.y, (double)state_task.position.z,
                    (double)state_task.velocity.x, (double)state_task.velocity.y, (double)state_task.velocity.z);
      }

      // Get command reference
      UpdateHorizonReference(&setpoint_task);
      
      if (task_loop_count <= 3) {
        DEBUG_PRINT("ref: (%.2f,%.2f,%.2f)\n",
                    (double)params.Xref(0,0), (double)params.Xref(1,0), (double)params.Xref(2,0));
      }

      // Static obstacle avoidance via LTV linear constraints (single disk)
      const bool constraint_hold =
          (!mpc_has_run) || ((xTaskGetTickCount() - controller_activate_tick) < M2T(500));
      static uint32_t cstr_log_cnt = 0;
      int cstr_active_count = 0;
      for (int i = 0; i < NHORIZON; i++)
      {
        params.x_min[i] = tiny_VectorNc::Constant(-1000);
        params.x_max[i] = tiny_VectorNc::Constant(1000);
        params.A_constraints[i] = tiny_MatrixNcNx::Zero();

        if (enable_obs_constraint && !constraint_hold) {
          // Use reference position to define the tangent half-space
          Eigen::Matrix<tinytype, 3, 1> ref = params.Xref.col(i).head(3);
          xc = ref - obs_center; // points from obstacle center to reference
          float xc_norm = xc.norm();
          if (xc_norm > 1e-3f && xc_norm < (r_obs + obs_activation_margin)) {
            a_norm = -xc / xc_norm; // inward normal (for A x <= b)
            params.A_constraints[i].head(3) = a_norm.transpose();
            q_c = obs_center - r_obs * a_norm;
            params.x_max[i](0) = a_norm.transpose() * q_c;
            cstr_active_count++;
          }
        }
      }
      if (cstr_active_count > 0 && (cstr_log_cnt++ % 25 == 0)) {
        DEBUG_PRINT("OBS: %d active, pos=(%.2f,%.2f)\n", cstr_active_count,
                    (double)state_task.position.x, (double)state_task.position.y);
      }
      
      // Force cache_level=1 when any constraints are active to prevent oscillation
      // The ADMM solver sets cache_level based on violation, but we want it stable
      if (cstr_active_count > 0) {
        problem.cache_level = 1;
      }


      // // Start predicting the obstacle if the distance between it and the drone is less
      // // than the distance the obstacle would travel over the course of two seconds,
      // // since the drone should be able to move out of the way in less than two seconds.
      // if ((problem.x.col(0).head(3) - obs_center).norm() < obs_velocity.norm()*2) {
      //   obs_offset = (problem.x.col(0).head(3) - obs_center).norm()*.9 * obs_velocity.normalized();
      // }
      // else {
      //   obs_offset << 0.0, 0.0, 0.0;
      // }

      // // When avoiding dynamic obstacle
      // for (int i = 0; i < NHORIZON; i++)
      // {
      //   // obs_predicted_center = obs_center + (obs_velocity/50 * i) * obs_velocity_scale + (problem.x.col(0).head(3) - obs_center).norm() * obs_velocity.normalized() * use_obs_offset;
      //   // obs_predicted_center = obs_center + (obs_velocity/50 * i) * obs_velocity_scale + (problem.x.col(0).head(3) - obs_center).norm() * obs_velocity.normalized();
      //   obs_predicted_center = obs_center + obs_offset + (obs_velocity/50 * i) * obs_velocity_scale;
      //   xc = obs_predicted_center - problem.x.col(i).head(3);
      //   a_norm = xc / xc.norm();
      //   params.A_constraints[i].head(3) = a_norm.transpose();
      //   q_c = obs_center - r_obs * a_norm;
      //   params.x_max[i](0) = a_norm.transpose() * q_c;
      // }

      // MPC solve
      problem.iter = 0;

      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC solve start\n");
      }
      mpc_start_timestamp = usecTimestamp();
      solve_admm(&problem, &params);
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC solve 1 done, iter=%d\n", problem.iter);
      }
      // Skip second solve for now to reduce stack usage
      // vTaskDelay(M2T(1));
      // solve_admm(&problem, &params);
      mpc_time_us = usecTimestamp() - mpc_start_timestamp;
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC time=%lu us\n", mpc_time_us);
      }

      mpc_setpoint_task = problem.x.col(NHORIZON-1);
      
      if (task_loop_count <= 3) {
        DEBUG_PRINT("setpoint: x=%.2f z=%.2f\n", (double)mpc_setpoint_task(0), (double)mpc_setpoint_task(2));
      }

      // Skip event triggers for now to simplify debugging
      // eventTrigger payloads and calls commented out

      // Copy the setpoint calculated by the task loop to the global mpc_setpoint
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&mpc_setpoint, &mpc_setpoint_task, sizeof(tiny_VectorNx));
      memcpy(&init_vel_z, &problem.x.col(0)(8), sizeof(float));
      mpc_has_run = true; // Mark that MPC has computed at least once
      xSemaphoreGive(dataMutex);
    }
  }
}

/**
 * This function is called from the stabilizer loop. It is important that this call returns
 * as quickly as possible. The dataMutex must only be locked short periods by the task.
 */
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
  setpoint_t hold_sp;
  fill_hold_setpoint(&hold_sp, state);

  if (!isInit || (dataMutex == NULL) || (runTaskSemaphore == NULL)) {
    controllerPid(control, &hold_sp, sensors, state, tick);
    return;
  }

  if (xSemaphoreTake(dataMutex, M2T(2)) != pdTRUE) {
    controllerPid(control, &hold_sp, sensors, state, tick);
    return;
  }
  memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
  memcpy(&sensors_data, sensors, sizeof(sensorData_t));
  memcpy(&state_data, state, sizeof(state_t));
  // memcpy(control, &control_data, sizeof(state_t));

  const bool controller_reactivated =
      (last_controller_tick == 0) || ((tick - last_controller_tick) > M2T(200));
  if (controller_reactivated) {
    controller_activate_tick = tick;
    mpc_has_run = false;
    // Initialize to current state to avoid a bad setpoint on first switch
    mpc_setpoint = tiny_VectorNx::Zero();
    mpc_setpoint(0) = state->position.x;
    mpc_setpoint(1) = state->position.y;
    mpc_setpoint(2) = state->position.z;
    DEBUG_PRINT("OOT activated at z=%.2f\n", (double)state->position.z);
  }
  last_controller_tick = tick;

  if (RATE_DO_EXECUTE(LOWLEVEL_RATE, tick))
  {
    mpc_setpoint_pid.mode.yaw = modeAbs;
    mpc_setpoint_pid.mode.x = modeAbs;
    mpc_setpoint_pid.mode.y = modeAbs;
    mpc_setpoint_pid.mode.z = modeAbs;
    
    // Use current position as fallback if MPC hasn't computed yet to avoid diving
    const bool hold_output =
        (!mpc_has_run) || ((tick - controller_activate_tick) < M2T(200));
    if (!hold_output) {
      mpc_setpoint_pid.position.x = mpc_setpoint(0);
      mpc_setpoint_pid.position.y = mpc_setpoint(1);
      mpc_setpoint_pid.position.z = mpc_setpoint(2);
      mpc_setpoint_pid.attitude.yaw = mpc_setpoint(5);
    } else {
      // Hold current position until MPC is ready
      mpc_setpoint_pid.position.x = state->position.x;
      mpc_setpoint_pid.position.y = state->position.y;
      mpc_setpoint_pid.position.z = state->position.z;
      mpc_setpoint_pid.attitude.yaw = state->attitude.yaw;
    }

    // if (RATE_DO_EXECUTE(RATE_25_HZ, tick)) {
    //   // DEBUG_PRINT("z: %.4f\n", mpc_setpoint(2));
    //   DEBUG_PRINT("h: %.4f\n", mpc_setpoint(4));
    //   // DEBUG_PRINT("x: %.4f\n", setpoint->position.x);
    // }

    // Kill motors if trajectory finished (enable_traj goes false)
    if (!enable_traj && mpc_has_run) {
      static bool landed_msg = false;
      if (!landed_msg) {
        DEBUG_PRINT("LANDING: traj done, killing motors\n");
        landed_msg = true;
      }
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
    } else {
      controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);
    }
  }

  // if (RATE_DO_EXECUTE(LQR_RATE, tick)) {

  //   phi = quat_2_rp(normalize_quat(state->attitudeQuaternion));  // quaternion to Rodrigues parameters
  //   current_state << state->position.x, state->position.y, state->position.z,
  //                     phi.x, phi.y, phi.z,
  //                     state->velocity.x, state->velocity.y, state->velocity.z,
  //                     radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z);

  //   // u_lqr = -params.cache.Kinf * (current_state - mpc_setpoint);
  //   u_lqr = -params.cache.Kinf * (current_state - Xref_origin);
  //   // u_lqr = -params.cache.Kinf * (current_state - params.Xref.col(0));

  //   if (setpoint->mode.z == modeDisable) {
  //     control->normalizedForces[0] = 0.0f;
  //     control->normalizedForces[1] = 0.0f;
  //     control->normalizedForces[2] = 0.0f;
  //     control->normalizedForces[3] = 0.0f;
  //   } else {
  //     control->normalizedForces[0] = u_lqr(0) + u_hover[0];  // PWM 0..1
  //     control->normalizedForces[1] = u_lqr(1) + u_hover[1];
  //     control->normalizedForces[2] = u_lqr(2) + u_hover[2];
  //     control->normalizedForces[3] = u_lqr(3) + u_hover[3];
  //   }
  //   control->controlMode = controlModePWM;
  // }

  xSemaphoreGive(dataMutex);

  // Allows mpc task to run again
  xSemaphoreGive(runTaskSemaphore);
  
  static uint32_t oot_loop_count = 0;
  oot_loop_count++;
  if (oot_loop_count <= 3) {
    DEBUG_PRINT("OOT loop %lu done\n", oot_loop_count);
  }
}

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

LOG_GROUP_START(tinympc)

LOG_ADD(LOG_FLOAT, initial_velocity, &init_vel_z)

LOG_GROUP_STOP(tinympc)

#ifdef __cplusplus
} /* extern "C" */
#endif