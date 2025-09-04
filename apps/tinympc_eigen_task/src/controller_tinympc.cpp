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
#include "eventtrigger.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h" // needed to compile Cpp to C

// TinyMPC new API
#include "tinympc/tiny_api.hpp"
#include "controller_pid.h"

// Constants for TinyMPC problem dimensions
#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 20
#define NTOTAL 451

// Task configuration constants  
#define TINYMPC_TASK_STACKSIZE 2000
#define TINYMPC_TASK_NAME "TINYMPC"
#define TINYMPC_TASK_PRI 3

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
// #define MPC_RATE RATE_50_HZ
#define MPC_RATE RATE_100_HZ
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
tinyVector mpc_setpoint(NSTATES);
setpoint_t mpc_setpoint_pid;
// Copies that stay constant for duration of MPC loop
setpoint_t setpoint_task;
sensorData_t sensors_task;
state_t state_task;
control_t control_task;
tinyVector mpc_setpoint_task(NSTATES);

/* Allocate global variables for MPC - New TinyMPC API */
// static tinytype u_hover[4] = {.65, .65, .65, .65};
static tinytype u_hover[4] = {.583, .583, .583, .583};

// New API objects
static TinySolver *solver = nullptr;
static TinyCache cache;
static TinyWorkspace workspace;
static TinySettings settings;

static float init_vel_z;
static tinyMatrix Xref_total(3, NTOTAL);
static tinyVector Xref_origin(NSTATES); // Start position for trajectory
static tinyVector Xref_end(NSTATES); // End position for trajectory
static tinyVector u_lqr(NINPUTS);
static tinyVector current_state(NSTATES);

// Helper variables
static bool enable_traj = false;
static int traj_index = 0;
static int max_traj_index = 0;
static uint64_t startTimestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
static bool isInit = false;

// Obstacle constraint variables
static Eigen::Matrix<tinytype, 3, 1> obs_center;
static Eigen::Matrix<tinytype, 3, 1> obs_predicted_center;
static Eigen::Matrix<tinytype, 3, 1> obs_velocity;
static Eigen::Matrix<tinytype, 3, 1> obs_offset;
static float r_obs = .5;

static Eigen::Matrix<tinytype, 3, 1> xc;
static Eigen::Matrix<tinytype, 3, 1> a_norm;
static Eigen::Matrix<tinytype, 3, 1> q_c;

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

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));
  }
}

static void initializeProblem(void) {
  // Problem will be initialized through the C API, no need for manual matrix allocation
}


void controllerOutOfTreeInit(void)
{
  controllerPidInit();

  // Initialize problem matrices
  initializeProblem();
  
  // Resize global matrices
  Xref_total.resize(3, NTOTAL);
  Xref_origin.resize(NSTATES);
  Xref_end.resize(NSTATES);
  mpc_setpoint.resize(NSTATES);
  mpc_setpoint_task.resize(NSTATES);
  u_lqr.resize(NINPUTS);
  current_state.resize(NSTATES);
  
  // Set up dynamics matrices from parameter files
  tinyMatrix Adyn = Eigen::Map<tinyMatrix>(Adyn_unconstrained_data, NSTATES, NSTATES);
  tinyMatrix Bdyn = Eigen::Map<tinyMatrix>(Bdyn_unconstrained_data, NSTATES, NINPUTS);
  tinyVector fdyn = tinyVector::Zero(NSTATES);  // Assuming no affine term
  
  // Set up cost matrices
  tinyVector Q = Eigen::Map<tinyVector>(Q_unconstrained_data, NSTATES);
  tinyVector R = Eigen::Map<tinyVector>(R_unconstrained_data, NINPUTS);
  
  // Setup solver using C API
  int result = tiny_setup(&solver, Adyn, Bdyn, fdyn, Q, R, rho_unconstrained_value, 
                         NSTATES, NINPUTS, NHORIZON, 0);
  if (result != 0) {
    DEBUG_PRINT("TinyMPC setup failed!\n");
    return;
  }
  
  // Configure settings
  tiny_set_default_settings(&settings);
  tiny_update_settings(&settings, 0.001, 0.001, 5, 1, 0, 1, 0, 0, 0, 0);
  
  // Set input bounds
  tinyMatrix u_min_matrix = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  tinyMatrix u_max_matrix = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  for (int i = 0; i < NHORIZON-1; i++) {
    u_min_matrix.col(i) << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
    u_max_matrix.col(i) << 1-u_hover[0], 1-u_hover[1], 1-u_hover[2], 1-u_hover[3];
  }
  
  // We'll handle constraints dynamically in the control loop
  tinyMatrix x_min_matrix = tinyMatrix::Constant(NSTATES, NHORIZON, -1000.0);
  tinyMatrix x_max_matrix = tinyMatrix::Constant(NSTATES, NHORIZON, 1000.0);
  tiny_set_bound_constraints(solver, x_min_matrix, x_max_matrix, u_min_matrix, u_max_matrix);
  
  // Set reference trajectory to hover at origin
  Xref_origin << 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  tinyMatrix Xref = Xref_origin.replicate(1, NHORIZON);
  tinyMatrix Uref = tinyMatrix::Zero(NINPUTS, NHORIZON-1);
  tiny_set_x_ref(solver, Xref);
  tiny_set_u_ref(solver, Uref);
  
  // Copy reference trajectory data
  Xref_total = Eigen::Map<tinyMatrix>(Xref_data, 3, NTOTAL);
  Xref_end.head(3) = Xref_total.col(NTOTAL-1);
  Xref_end.tail(NSTATES-3).setZero();

  enable_traj = false;
  traj_index = 0;
  max_traj_index = NTOTAL - NHORIZON;

  /* Begin task initialization */
  runTaskSemaphore = xSemaphoreCreateBinary();
  // ASSERT(runTaskSemaphore); // Commented out due to C++ string literal conversion issue

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;
  /* End of task initialization */
}

static void UpdateHorizonReference(const setpoint_t *setpoint)
{
  tinyMatrix Xref(NSTATES, NHORIZON);
  if (enable_traj)
  {
    if (traj_index < max_traj_index)
    {
      // Set XYZ reference from trajectory data, keep other states from origin
      Xref = Xref_origin.replicate(1, NHORIZON);
      Xref.block(0, 0, 3, NHORIZON) = Xref_total.block(0, traj_index, 3, NHORIZON);
      traj_index++;
    }
    else if (traj_index >= max_traj_index) {
      Xref = Xref_end.replicate(1, NHORIZON);
    }
    else
    {
      enable_traj = false;
    }
  }
  else
  {
    Xref = Xref_origin.replicate(1, NHORIZON);
  }
  
  tiny_set_x_ref(solver, Xref);
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

  while (true)
  {
    // Update task data with most recent stabilizer loop data
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

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
      if (usecTimestamp() - startTimestamp > 1000000 * 2 && traj_index == 0)
      {
        DEBUG_PRINT("Enable trajectory!\n");
        // enable_traj = true; 
        traj_index++;
      }

      // TODO: predict into the future and set initial x to wherever we think we'll be
      //    by the time we're done computing the input for that state. If we just set
      //    initial x to current state then by the time we compute the optimal input for
      //    that state we'll already be at the next state and there will be a mismatch
      //    in the input we're using for our current state.
      // Set initial x to current state
      phi = quat_2_rp(normalize_quat(state_task.attitudeQuaternion)); // quaternion to Rodrigues parameters
      tinyVector x0(NSTATES);
      x0 << state_task.position.x, state_task.position.y, state_task.position.z,
          phi.x, phi.y, phi.z,
          state_task.velocity.x, state_task.velocity.y, state_task.velocity.z,
          radians(sensors_task.gyro.x), radians(sensors_task.gyro.y), radians(sensors_task.gyro.z);
      
      tiny_set_x0(solver, x0);

      // Get command reference
      UpdateHorizonReference(&setpoint_task);

      r_obs = setpoint_task.acceleration.x;
      obs_center(0) = setpoint_task.position.x;
      obs_center(1) = setpoint_task.position.y;
      obs_center(2) = setpoint_task.position.z;
      
      obs_velocity(0) = setpoint_task.velocity.x;
      obs_velocity(1) = setpoint_task.velocity.y;
      obs_velocity(2) = setpoint_task.velocity.z;

      // // When avoiding obstacle while tracking trajectory
      // if (enable_traj) {
      //   // Update constraint parameters
      //   for (int i=0; i<NHORIZON; i++) {
      //     obs_predicted_center = obs_center + (obs_velocity/50 * i) * obs_velocity_scale;
      //     xc = obs_predicted_center - problem.x.col(i).head(3);
      //     a_norm = xc / xc.norm();
      //     params.A_constraints[i].head(3) = a_norm.transpose();
      //     q_c = obs_center - r_obs * a_norm;
      //     params.x_max[i](0) = a_norm.transpose() * q_c;
      //   }
      // } else {
      //   for (int i=0; i<NHORIZON; i++) {
      //       params.x_min[i] = tiny_VectorNc::Constant(-1000); // Currently unused
      //       params.x_max[i] = tiny_VectorNc::Constant(1000);
      //       params.A_constraints[i] = tiny_MatrixNcNx::Zero();
      //   }
      // }

      // For now, we'll skip the dynamic obstacle constraints 
      // as they require dynamic linear constraint updates which aren't 
      // straightforward with the current C API. This can be added later.
      
      // Get current state for constraint calculations (if needed)
      // We could use x0 here for obstacle avoidance calculations


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
      mpc_start_timestamp = usecTimestamp();
      
      // Solve MPC problem
      int result = tiny_solve(solver);
      vTaskDelay(M2T(1));
      result = tiny_solve(solver);
      
      mpc_time_us = usecTimestamp() - mpc_start_timestamp - 1000; // -1000 for each vTaskDelay(M2T(1))

      // Get the final state from the solution
      mpc_setpoint_task = solver->solution->x.col(NHORIZON-1);

      eventTrigger_horizon_x_part1_payload.h0 = solver->solution->x.col(0)(0);
      eventTrigger_horizon_x_part1_payload.h1 = solver->solution->x.col(1)(0);
      eventTrigger_horizon_x_part1_payload.h2 = solver->solution->x.col(2)(0);
      eventTrigger_horizon_x_part1_payload.h3 = solver->solution->x.col(3)(0);
      eventTrigger_horizon_x_part1_payload.h4 = solver->solution->x.col(4)(0);
      eventTrigger_horizon_x_part2_payload.h5 = solver->solution->x.col(5)(0);
      eventTrigger_horizon_x_part2_payload.h6 = solver->solution->x.col(6)(0);
      eventTrigger_horizon_x_part2_payload.h7 = solver->solution->x.col(7)(0);
      eventTrigger_horizon_x_part2_payload.h8 = solver->solution->x.col(8)(0);
      eventTrigger_horizon_x_part2_payload.h9 = solver->solution->x.col(9)(0);
      eventTrigger_horizon_x_part3_payload.h10 = solver->solution->x.col(10)(0);
      eventTrigger_horizon_x_part3_payload.h11 = solver->solution->x.col(11)(0);
      eventTrigger_horizon_x_part3_payload.h12 = solver->solution->x.col(12)(0);
      eventTrigger_horizon_x_part3_payload.h13 = solver->solution->x.col(13)(0);
      eventTrigger_horizon_x_part3_payload.h14 = solver->solution->x.col(14)(0);
      eventTrigger_horizon_x_part4_payload.h15 = solver->solution->x.col(15)(0);
      eventTrigger_horizon_x_part4_payload.h16 = solver->solution->x.col(16)(0);
      eventTrigger_horizon_x_part4_payload.h17 = solver->solution->x.col(17)(0);
      eventTrigger_horizon_x_part4_payload.h18 = solver->solution->x.col(18)(0);
      eventTrigger_horizon_x_part4_payload.h19 = solver->solution->x.col(19)(0);

      eventTrigger_horizon_y_part1_payload.h0 = solver->solution->x.col(0)(1);
      eventTrigger_horizon_y_part1_payload.h1 = solver->solution->x.col(1)(1);
      eventTrigger_horizon_y_part1_payload.h2 = solver->solution->x.col(2)(1);
      eventTrigger_horizon_y_part1_payload.h3 = solver->solution->x.col(3)(1);
      eventTrigger_horizon_y_part1_payload.h4 = solver->solution->x.col(4)(1);
      eventTrigger_horizon_y_part2_payload.h5 = solver->solution->x.col(5)(1);
      eventTrigger_horizon_y_part2_payload.h6 = solver->solution->x.col(6)(1);
      eventTrigger_horizon_y_part2_payload.h7 = solver->solution->x.col(7)(1);
      eventTrigger_horizon_y_part2_payload.h8 = solver->solution->x.col(8)(1);
      eventTrigger_horizon_y_part2_payload.h9 = solver->solution->x.col(9)(1);
      eventTrigger_horizon_y_part3_payload.h10 = solver->solution->x.col(10)(1);
      eventTrigger_horizon_y_part3_payload.h11 = solver->solution->x.col(11)(1);
      eventTrigger_horizon_y_part3_payload.h12 = solver->solution->x.col(12)(1);
      eventTrigger_horizon_y_part3_payload.h13 = solver->solution->x.col(13)(1);
      eventTrigger_horizon_y_part3_payload.h14 = solver->solution->x.col(14)(1);
      eventTrigger_horizon_y_part4_payload.h15 = solver->solution->x.col(15)(1);
      eventTrigger_horizon_y_part4_payload.h16 = solver->solution->x.col(16)(1);
      eventTrigger_horizon_y_part4_payload.h17 = solver->solution->x.col(17)(1);
      eventTrigger_horizon_y_part4_payload.h18 = solver->solution->x.col(18)(1);
      eventTrigger_horizon_y_part4_payload.h19 = solver->solution->x.col(19)(1);

      eventTrigger_horizon_z_part1_payload.h0 = solver->solution->x.col(0)(2);
      eventTrigger_horizon_z_part1_payload.h1 = solver->solution->x.col(1)(2);
      eventTrigger_horizon_z_part1_payload.h2 = solver->solution->x.col(2)(2);
      eventTrigger_horizon_z_part1_payload.h3 = solver->solution->x.col(3)(2);
      eventTrigger_horizon_z_part1_payload.h4 = solver->solution->x.col(4)(2);
      eventTrigger_horizon_z_part2_payload.h5 = solver->solution->x.col(5)(2);
      eventTrigger_horizon_z_part2_payload.h6 = solver->solution->x.col(6)(2);
      eventTrigger_horizon_z_part2_payload.h7 = solver->solution->x.col(7)(2);
      eventTrigger_horizon_z_part2_payload.h8 = solver->solution->x.col(8)(2);
      eventTrigger_horizon_z_part2_payload.h9 = solver->solution->x.col(9)(2);
      eventTrigger_horizon_z_part3_payload.h10 = solver->solution->x.col(10)(2);
      eventTrigger_horizon_z_part3_payload.h11 = solver->solution->x.col(11)(2);
      eventTrigger_horizon_z_part3_payload.h12 = solver->solution->x.col(12)(2);
      eventTrigger_horizon_z_part3_payload.h13 = solver->solution->x.col(13)(2);
      eventTrigger_horizon_z_part3_payload.h14 = solver->solution->x.col(14)(2);
      eventTrigger_horizon_z_part4_payload.h15 = solver->solution->x.col(15)(2);
      eventTrigger_horizon_z_part4_payload.h16 = solver->solution->x.col(16)(2);
      eventTrigger_horizon_z_part4_payload.h17 = solver->solution->x.col(17)(2);
      eventTrigger_horizon_z_part4_payload.h18 = solver->solution->x.col(18)(2);
      eventTrigger_horizon_z_part4_payload.h19 = solver->solution->x.col(19)(2);

      eventTrigger_problem_data_event_payload.solvetime_us = mpc_time_us;
      eventTrigger_problem_data_event_payload.iters = solver->work->iter;
      eventTrigger_problem_data_event_payload.cache_level = 0; // No cache levels in new API
      eventTrigger_problem_residuals_event_payload.prim_resid_state = solver->work->primal_residual_state;
      eventTrigger_problem_residuals_event_payload.prim_resid_input = solver->work->primal_residual_input;
      eventTrigger_problem_residuals_event_payload.dual_resid_state = solver->work->dual_residual_state;
      eventTrigger_problem_residuals_event_payload.dual_resid_input = solver->work->dual_residual_input;

      eventTrigger(&eventTrigger_horizon_x_part1);
      eventTrigger(&eventTrigger_horizon_x_part2);
      eventTrigger(&eventTrigger_horizon_x_part3);
      eventTrigger(&eventTrigger_horizon_x_part4);
      eventTrigger(&eventTrigger_horizon_y_part1);
      eventTrigger(&eventTrigger_horizon_y_part2);
      eventTrigger(&eventTrigger_horizon_y_part3);
      eventTrigger(&eventTrigger_horizon_y_part4);
      eventTrigger(&eventTrigger_horizon_z_part1);
      eventTrigger(&eventTrigger_horizon_z_part2);
      eventTrigger(&eventTrigger_horizon_z_part3);
      eventTrigger(&eventTrigger_horizon_z_part4);
      eventTrigger(&eventTrigger_problem_data_event);
      eventTrigger(&eventTrigger_problem_residuals_event);

      // Copy the setpoint calculated by the task loop to the global mpc_setpoint
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      mpc_setpoint = mpc_setpoint_task;
      init_vel_z = solver->solution->x.col(0)(8);
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
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
  memcpy(&sensors_data, sensors, sizeof(sensorData_t));
  memcpy(&state_data, state, sizeof(state_t));
  // memcpy(control, &control_data, sizeof(state_t));

  if (RATE_DO_EXECUTE(LOWLEVEL_RATE, tick))
  {
    mpc_setpoint_pid.mode.yaw = modeAbs;
    mpc_setpoint_pid.mode.x = modeAbs;
    mpc_setpoint_pid.mode.y = modeAbs;
    mpc_setpoint_pid.mode.z = modeAbs;
    mpc_setpoint_pid.position.x = mpc_setpoint(0);
    mpc_setpoint_pid.position.y = mpc_setpoint(1);
    mpc_setpoint_pid.position.z = mpc_setpoint(2);
    mpc_setpoint_pid.attitude.yaw = mpc_setpoint(5);

    // if (RATE_DO_EXECUTE(RATE_25_HZ, tick)) {
    //   // DEBUG_PRINT("z: %.4f\n", mpc_setpoint(2));
    //   DEBUG_PRINT("h: %.4f\n", mpc_setpoint(4));
    //   // DEBUG_PRINT("x: %.4f\n", setpoint->position.x);
    // }

    controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);
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
}

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

// LOG_GROUP_START(tinympc)  // Commented out due to C++ string literal conversion issue
// 
// LOG_ADD(LOG_FLOAT, initial_velocity, &init_vel_z)
// 
// LOG_GROUP_STOP(tinympc)

#ifdef __cplusplus
} /* extern "C" */
#endif