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
 * =============================================================================
 * PLANNER + TRACKER ARCHITECTURE (TinySDP)
 * =============================================================================
 * 
 * This implementation uses a hierarchical architecture matching the simulation:
 * 
 * PLANNER (runs every REPLAN_STRIDE steps):
 *   - Has PSD constraints enabled
 *   - Computes collision-free trajectory
 *   - Stores plan in cache for tracker to follow
 * 
 * TRACKER (runs every step):
 *   - NO obstacle constraints
 *   - Just tracks the planner's reference trajectory
 *   - Lightweight, fast solve
 * 
 * This matches tiny_psd_dynamic_demo.cpp in simulation.
 * =============================================================================
 */

// Use planner+tracker architecture (comment out to use old single-solver)
// #define USE_PLANNER_TRACKER_ARCH 1  // DISABLED - using single-solver arch

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
//#define MPC_RATE 10
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
static uint8_t enable_psd = 1; // PSD enabled (runs every 5 ADMM iters)

// Dynamic obstacle (disk) parameters for LTV linear constraints
// XZ PLANE: Two arms closing gap - one from bottom, one from top
static Eigen::Matrix<tinytype, 3, 1> obs_center_bottom;   // Arm from below
static Eigen::Matrix<tinytype, 3, 1> obs_center_top;      // Arm from above
static Eigen::Matrix<tinytype, 3, 1> obs_start_bottom;    // Initial position (bottom arm)
static Eigen::Matrix<tinytype, 3, 1> obs_start_top;       // Initial position (top arm)
static Eigen::Matrix<tinytype, 3, 1> obs_velocity_bottom; // Bottom arm sweeps upward
static Eigen::Matrix<tinytype, 3, 1> obs_velocity_top;    // Top arm sweeps downward
static Eigen::Matrix<tinytype, 3, 1> xc;
static Eigen::Matrix<tinytype, 3, 1> a_norm;
static Eigen::Matrix<tinytype, 3, 1> q_c;
static float r_obs = 0.25f;           // Obstacle radius (slightly smaller for XZ)
static float obs_activation_margin = 0.15f; // Constraint activation distance
static uint64_t obs_start_time = 0;   // Time when obstacle motion started

// =============================================================================
// PLANNER + TRACKER ARCHITECTURE VARIABLES
// =============================================================================
#ifdef USE_PLANNER_TRACKER_ARCH

// Planner runs every REPLAN_STRIDE MPC steps
#define REPLAN_STRIDE 5

// Separate problem/params for tracker (planner uses the main problem/params)
static struct tiny_cache cache_tracker;
static struct tiny_params params_tracker;
static struct tiny_problem problem_tracker;

// Plan cache: stores the trajectory from the planner
static tiny_MatrixNxNh plan_Xref_cache;      // Cached reference trajectory
static tiny_MatrixNuNhm1 plan_Uref_cache;    // Cached input trajectory
static int plan_start_step = 0;               // Step when plan was computed
static int plan_iters = 0;                    // Iterations planner took
static bool plan_valid = false;               // Is there a valid plan?

// Distance-based PSD activation with hysteresis
static const float psd_on_thresh = 0.5f;      // Distance to activate PSD
static const float psd_off_thresh = 0.7f;     // Distance to deactivate PSD
static bool psd_constraints_active = false;   // Current PSD state

// Step counter for replan scheduling
static int mpc_step_count = 0;

// Helper: clamp index to valid range
static inline int clamp_idx(int idx, int lo, int hi) {
    if (idx < lo) return lo;
    if (idx > hi) return hi;
    return idx;
}

// Helper: compute signed distance from position to obstacle in XZ plane
static inline float signed_distance_to_obs_xz(float px, float pz, float ox, float oz, float r) {
    float dx = px - ox;
    float dz = pz - oz;
    return sqrtf(dx*dx + dz*dz) - r;
}

#endif // USE_PLANNER_TRACKER_ARCH

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

  // XZ PLANE: Two dynamic obstacles - arms closing gap vertically
  // Bottom arm starts at z=0.2, sweeps upward at 0.03 m/s
  // Top arm starts at z=0.8, sweeps downward at 0.03 m/s
  // Goal: drone must lift up into shrinking gap before it closes
  obs_start_bottom << 0.7f, 0.0f, 0.2f;       // Bottom arm (low z)
  obs_velocity_bottom << 0.0f, 0.0f, 0.03f;   // Sweeps upward
  obs_center_bottom = obs_start_bottom;
  
  obs_start_top << 0.7f, 0.0f, 0.8f;          // Top arm (high z)
  obs_velocity_top << 0.0f, 0.0f, -0.03f;     // Sweeps downward
  obs_center_top = obs_start_top;
  
  obs_start_time = 0;                          // Will be set on first MPC solve

  // Initialize PSD constraints for XZ plane (disabled by default, enable via enable_psd flag)
  // Note: psd_obs_y is actually psd_obs_z in XZ mode - we use bottom arm initially
  problem.en_psd = enable_psd;
  if (enable_psd) {
    tinytype rho_psd = 10.0f;  // PSD penalty parameter (tune as needed)
    tiny_enable_psd(&problem, &params, rho_psd);
    // Set PSD obstacle using (x, z) coordinates - using bottom arm
    // Note: tiny_set_psd_obstacle takes (ox, oy) but in XZ mode oy = oz
    tiny_set_psd_obstacle(&problem, obs_center_bottom(0), obs_center_bottom(2), r_obs);
    DEBUG_PRINT("PSD XZ enabled with rho_psd=%.1f, bottom=(%.2f,z=%.2f), top=(%.2f,z=%.2f), r=%.2f\n", 
                (double)rho_psd, 
                (double)obs_center_bottom(0), (double)obs_center_bottom(2),
                (double)obs_center_top(0), (double)obs_center_top(2),
                (double)r_obs);
  }

#ifdef USE_PLANNER_TRACKER_ARCH
  // ==========================================================================
  // TRACKER SOLVER INITIALIZATION
  // Tracker has NO PSD, NO obstacle constraints - just tracks the plan
  // ==========================================================================
  DEBUG_PRINT("Initializing PLANNER+TRACKER architecture...\n");
  
  // Copy cache data for tracker (same dynamics, different rho)
  cache_tracker.Adyn[0] = cache.Adyn[0];
  cache_tracker.Bdyn[0] = cache.Bdyn[0];
  cache_tracker.rho[0] = rho_unconstrained_value;
  cache_tracker.Kinf[0] = cache.Kinf[0];
  cache_tracker.Pinf[0] = cache.Pinf[0];
  cache_tracker.Quu_inv[0] = cache.Quu_inv[0];
  cache_tracker.AmBKt[0] = cache.AmBKt[0];
  cache_tracker.coeff_d2p[0] = cache.coeff_d2p[0];
  
  cache_tracker.Adyn[1] = cache.Adyn[1];
  cache_tracker.Bdyn[1] = cache.Bdyn[1];
  cache_tracker.rho[1] = rho_constrained_value;
  cache_tracker.Kinf[1] = cache.Kinf[1];
  cache_tracker.Pinf[1] = cache.Pinf[1];
  cache_tracker.Quu_inv[1] = cache.Quu_inv[1];
  cache_tracker.AmBKt[1] = cache.AmBKt[1];
  cache_tracker.coeff_d2p[1] = cache.coeff_d2p[1];
  
  // Tracker params - same as main but NO obstacle constraints
  params_tracker.Q[0] = params.Q[0];
  params_tracker.Qf[0] = params.Qf[0];
  params_tracker.R[0] = params.R[0];
  params_tracker.Q[1] = params.Q[1];
  params_tracker.Qf[1] = params.Qf[1];
  params_tracker.R[1] = params.R[1];
  params_tracker.u_min = params.u_min;
  params_tracker.u_max = params.u_max;
  for (int i = 0; i < NHORIZON; i++) {
    params_tracker.x_min[i] = tiny_VectorNc::Constant(-1000);
    params_tracker.x_max[i] = tiny_VectorNc::Constant(1000);
    params_tracker.A_constraints[i] = tiny_MatrixNcNx::Zero();
  }
  params_tracker.Xref = Xref_origin.replicate<1, NHORIZON>();
  params_tracker.Uref = tiny_MatrixNuNhm1::Zero();
  params_tracker.cache = cache_tracker;
  
  // Tracker problem - NO PSD
  problem_tracker.x = tiny_MatrixNxNh::Zero();
  problem_tracker.q = tiny_MatrixNxNh::Zero();
  problem_tracker.p = tiny_MatrixNxNh::Zero();
  problem_tracker.v = tiny_MatrixNxNh::Zero();
  problem_tracker.vnew = tiny_MatrixNxNh::Zero();
  problem_tracker.g = tiny_MatrixNxNh::Zero();
  problem_tracker.u = tiny_MatrixNuNhm1::Zero();
  problem_tracker.r = tiny_MatrixNuNhm1::Zero();
  problem_tracker.d = tiny_MatrixNuNhm1::Zero();
  problem_tracker.z = tiny_MatrixNuNhm1::Zero();
  problem_tracker.znew = tiny_MatrixNuNhm1::Zero();
  problem_tracker.y = tiny_MatrixNuNhm1::Zero();
  
  problem_tracker.primal_residual_state = 0;
  problem_tracker.primal_residual_input = 0;
  problem_tracker.dual_residual_state = 0;
  problem_tracker.dual_residual_input = 0;
  problem_tracker.abs_tol = 0.001;
  problem_tracker.status = 0;
  problem_tracker.iter = 0;
  problem_tracker.max_iter = 3;  // Tracker can be faster
  problem_tracker.iters_check_rho_update = 10;
  problem_tracker.cache_level = 0;
  problem_tracker.en_psd = 0;  // NO PSD for tracker!
  
  // Initialize plan cache
  plan_Xref_cache = Xref_origin.replicate<1, NHORIZON>();
  plan_Uref_cache = tiny_MatrixNuNhm1::Zero();
  plan_valid = false;
  psd_constraints_active = false;
  mpc_step_count = 0;
  
  DEBUG_PRINT("PLANNER+TRACKER init done. Replan every %d steps.\n", REPLAN_STRIDE);
#endif // USE_PLANNER_TRACKER_ARCH

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

#ifdef USE_PLANNER_TRACKER_ARCH
// =============================================================================
// PLANNER + TRACKER TASK (New Architecture)
// =============================================================================
static void tinympcControllerTask(void *parameters)
{
  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextMpcMs = nowMs;
  startTimestamp = usecTimestamp();

  static uint32_t task_loop_count = 0;
  static uint32_t log_cnt = 0;
  
  while (true)
  {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    
    task_loop_count++;
    if (task_loop_count <= 3) {
      DEBUG_PRINT("PLANNER+TRACKER task loop %lu\n", task_loop_count);
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

      // Skip after landing
      if (!enable_traj && traj_index >= max_traj_index) {
        continue;
      }

      // Enable trajectory after 2 second delay
      if (usecTimestamp() - startTimestamp > 1000000 * 2 && traj_index == 0 && !enable_traj)
      {
        DEBUG_PRINT("Enable trajectory!\n");
        enable_traj = true;
      }

      // Get current state
      phi = quat_2_rp(normalize_quat(state_task.attitudeQuaternion));
      tiny_VectorNx current_x;
      current_x << state_task.position.x, state_task.position.y, state_task.position.z,
          phi.x, phi.y, phi.z,
          state_task.velocity.x, state_task.velocity.y, state_task.velocity.z,
          radians(sensors_task.gyro.x), radians(sensors_task.gyro.y), radians(sensors_task.gyro.z);

      // Update nominal reference trajectory
      UpdateHorizonReference(&setpoint_task);
      
      // XZ PLANE: Update obstacle positions (bottom sweeps up, top sweeps down)
      if (obs_start_time == 0) {
        obs_start_time = usecTimestamp();
      }
      float obs_elapsed = (usecTimestamp() - obs_start_time) / 1e6f;
      
      obs_center_bottom = obs_start_bottom + obs_velocity_bottom * obs_elapsed;
      if (obs_center_bottom(2) > 0.5f) obs_center_bottom(2) = 0.5f;
      if (obs_center_bottom(2) < 0.15f) obs_center_bottom(2) = 0.15f;
      
      obs_center_top = obs_start_top + obs_velocity_top * obs_elapsed;
      if (obs_center_top(2) < 0.5f) obs_center_top(2) = 0.5f;
      if (obs_center_top(2) > 0.85f) obs_center_top(2) = 0.85f;

      // Compute signed distance to closest obstacle (XZ plane)
      float sd_bottom = signed_distance_to_obs_xz(current_x(0), current_x(2), 
                                                   obs_center_bottom(0), obs_center_bottom(2), r_obs);
      float sd_top = signed_distance_to_obs_xz(current_x(0), current_x(2), 
                                                obs_center_top(0), obs_center_top(2), r_obs);
      float sd = (sd_bottom < sd_top) ? sd_bottom : sd_top;

      // ================================================================
      // PLANNER: Run every REPLAN_STRIDE steps with PSD constraints
      // ================================================================
      bool need_replan = (mpc_step_count == 0) ||
                         (mpc_step_count - plan_start_step >= REPLAN_STRIDE) ||
                         (!plan_valid);
      
      const bool constraint_hold = 
          (!mpc_has_run) || ((xTaskGetTickCount() - controller_activate_tick) < M2T(500));
      
      if (need_replan && !constraint_hold) {
        // Distance-based PSD activation with hysteresis
        if (!psd_constraints_active && sd < psd_on_thresh) {
          psd_constraints_active = true;
          DEBUG_PRINT("PSD XZ ON: sd=%.2f < %.2f\n", (double)sd, (double)psd_on_thresh);
        } else if (psd_constraints_active && sd > psd_off_thresh) {
          psd_constraints_active = false;
          DEBUG_PRINT("PSD XZ OFF: sd=%.2f > %.2f\n", (double)sd, (double)psd_off_thresh);
        }

        // Configure planner with PSD constraints (use closest obstacle)
        if (psd_constraints_active) {
          problem.en_psd = 1;
          if (sd_bottom < sd_top) {
            problem.psd_obs_x = obs_center_bottom(0);
            problem.psd_obs_y = obs_center_bottom(2);  // z coordinate for XZ
          } else {
            problem.psd_obs_x = obs_center_top(0);
            problem.psd_obs_y = obs_center_top(2);  // z coordinate for XZ
          }
          problem.cache_level = 1;
        } else {
          problem.en_psd = 0;
          problem.cache_level = 0;
        }

        // Set initial state for planner
        problem.x.col(0) = current_x;
        
        // Solve planner
        problem.iter = 0;
        mpc_start_timestamp = usecTimestamp();
        solve_admm(&problem, &params);
        mpc_time_us = usecTimestamp() - mpc_start_timestamp;
        plan_iters = problem.iter;

        // Cache the plan trajectory
        plan_Xref_cache = problem.x;
        plan_Uref_cache = problem.u;
        plan_start_step = mpc_step_count;
        plan_valid = true;

        if (log_cnt++ % 25 == 0) {
          DEBUG_PRINT("PLAN: step=%d sd=%.2f psd=%d iter=%d t=%luus\n",
                      mpc_step_count, (double)sd, psd_constraints_active ? 1 : 0,
                      plan_iters, mpc_time_us);
        }
      }

      // ================================================================
      // TRACKER: Run every step, follows cached plan (NO obstacle constraints)
      // ================================================================
      
      // Set tracker reference from cached plan
      int offset = mpc_step_count - plan_start_step;
      for (int i = 0; i < NHORIZON; ++i) {
        int idx = clamp_idx(offset + i, 0, NHORIZON - 1);
        params_tracker.Xref.col(i) = plan_Xref_cache.col(idx);
      }
      for (int i = 0; i < NHORIZON - 1; ++i) {
        int idx = clamp_idx(offset + i, 0, NHORIZON - 2);
        params_tracker.Uref.col(i) = plan_Uref_cache.col(idx);
      }

      // Set initial state for tracker
      problem_tracker.x.col(0) = current_x;
      
      // Tracker has NO obstacle constraints - just tracks the (safe) plan
      problem_tracker.cache_level = 0;  // Use unconstrained cache
      problem_tracker.iter = 0;
      
      // Solve tracker (fast, no PSD)
      solve_admm(&problem_tracker, &params_tracker);

      // Output: use tracker's result
      mpc_setpoint_task = problem_tracker.x.col(NHORIZON-1);
      
      if (task_loop_count <= 5) {
        DEBUG_PRINT("TRACK: step=%d sp=(%.2f,%.2f,%.2f) iter=%d\n",
                    mpc_step_count, 
                    (double)mpc_setpoint_task(0), (double)mpc_setpoint_task(1), (double)mpc_setpoint_task(2),
                    problem_tracker.iter);
      }

      // Increment step counter
      mpc_step_count++;

      // Copy setpoint to shared variable
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&mpc_setpoint, &mpc_setpoint_task, sizeof(tiny_VectorNx));
      memcpy(&init_vel_z, &problem_tracker.x.col(0)(8), sizeof(float));
      mpc_has_run = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

#else // USE_PLANNER_TRACKER_ARCH
// =============================================================================
// ORIGINAL SINGLE-SOLVER TASK (Commented out old version)
// =============================================================================
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

      // Skip MPC solve after landing (trajectory done)
      if (!enable_traj && traj_index >= max_traj_index) {
        continue;  // Don't solve, just wait
      }

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

      // XZ PLANE: Update positions of both arms (bottom sweeps up, top sweeps down)
      if (obs_start_time == 0) {
        obs_start_time = usecTimestamp();  // Start timer on first solve
      }
      float obs_elapsed = (usecTimestamp() - obs_start_time) / 1e6f;
      
      // Update bottom arm (sweeps upward)
      obs_center_bottom = obs_start_bottom + obs_velocity_bottom * obs_elapsed;
      if (obs_center_bottom(2) > 0.5f) obs_center_bottom(2) = 0.5f;  // Clamp max z
      if (obs_center_bottom(2) < 0.15f) obs_center_bottom(2) = 0.15f;
      
      // Update top arm (sweeps downward)
      obs_center_top = obs_start_top + obs_velocity_top * obs_elapsed;
      if (obs_center_top(2) < 0.5f) obs_center_top(2) = 0.5f;  // Clamp min z
      if (obs_center_top(2) > 0.85f) obs_center_top(2) = 0.85f;
      
      // For PSD: use the closer obstacle (XZ plane: psd_obs_y = z coordinate)
      float drone_x = state_task.position.x;
      float drone_z = state_task.position.z;
      float dist_bottom = sqrtf((drone_x - obs_center_bottom(0))*(drone_x - obs_center_bottom(0)) +
                                (drone_z - obs_center_bottom(2))*(drone_z - obs_center_bottom(2)));
      float dist_top = sqrtf((drone_x - obs_center_top(0))*(drone_x - obs_center_top(0)) +
                             (drone_z - obs_center_top(2))*(drone_z - obs_center_top(2)));
      
      if (enable_psd) {
        // Use closest obstacle for PSD constraint (psd_obs_y is actually z in XZ mode)
        if (dist_bottom < dist_top) {
          problem.psd_obs_x = obs_center_bottom(0);
          problem.psd_obs_y = obs_center_bottom(2);  // z coordinate
        } else {
          problem.psd_obs_x = obs_center_top(0);
          problem.psd_obs_y = obs_center_top(2);  // z coordinate
        }
      }

      // XZ PLANE: LTV linear constraints for both obstacles
      const bool constraint_hold =
          (!mpc_has_run) || ((xTaskGetTickCount() - controller_activate_tick) < M2T(500));
      static uint32_t cstr_log_cnt = 0;
      int cstr_active_count = 0;
      const float dt_horizon = 1.0f / MPC_RATE;  // Time step per horizon
      
      for (int i = 0; i < NHORIZON; i++)
      {
        params.x_min[i] = tiny_VectorNc::Constant(-1000);
        params.x_max[i] = tiny_VectorNc::Constant(1000);
        params.A_constraints[i] = tiny_MatrixNcNx::Zero();

        if (enable_obs_constraint && !constraint_hold) {
          float future_t = obs_elapsed + i * dt_horizon;
          
          // Reference position for this horizon step
          Eigen::Matrix<tinytype, 3, 1> ref = params.Xref.col(i).head(3);
          
          // Check BOTTOM arm constraint (in XZ plane)
          Eigen::Matrix<tinytype, 3, 1> obs_pred_b = obs_start_bottom + obs_velocity_bottom * future_t;
          if (obs_pred_b(2) > 0.5f) obs_pred_b(2) = 0.5f;
          if (obs_pred_b(2) < 0.15f) obs_pred_b(2) = 0.15f;
          
          // XZ distance from ref to bottom obstacle
          Eigen::Matrix<tinytype, 2, 1> ref_xz(ref(0), ref(2));
          Eigen::Matrix<tinytype, 2, 1> obs_xz_b(obs_pred_b(0), obs_pred_b(2));
          Eigen::Matrix<tinytype, 2, 1> diff_b = ref_xz - obs_xz_b;
          float dist_b = diff_b.norm();
          
          if (dist_b > 1e-3f && dist_b < (r_obs + obs_activation_margin)) {
            // Half-space constraint in XZ plane: A_constraints uses indices 0 (x) and 2 (z)
            Eigen::Matrix<tinytype, 2, 1> a_xz = -diff_b / dist_b;
            params.A_constraints[i](0) = a_xz(0);   // x coefficient
            params.A_constraints[i](2) = a_xz(1);   // z coefficient (not y!)
            Eigen::Matrix<tinytype, 2, 1> q_xz = obs_xz_b - r_obs * a_xz;
            params.x_max[i](0) = a_xz.dot(q_xz);
            cstr_active_count++;
          }
          
          // Check TOP arm constraint (in XZ plane) - only if bottom didn't activate
          // (single constraint per step for simplicity)
          if (params.A_constraints[i].norm() < 1e-6f) {
            Eigen::Matrix<tinytype, 3, 1> obs_pred_t = obs_start_top + obs_velocity_top * future_t;
            if (obs_pred_t(2) < 0.5f) obs_pred_t(2) = 0.5f;
            if (obs_pred_t(2) > 0.85f) obs_pred_t(2) = 0.85f;
            
            Eigen::Matrix<tinytype, 2, 1> obs_xz_t(obs_pred_t(0), obs_pred_t(2));
            Eigen::Matrix<tinytype, 2, 1> diff_t = ref_xz - obs_xz_t;
            float dist_t = diff_t.norm();
            
            if (dist_t > 1e-3f && dist_t < (r_obs + obs_activation_margin)) {
              Eigen::Matrix<tinytype, 2, 1> a_xz = -diff_t / dist_t;
              params.A_constraints[i](0) = a_xz(0);   // x coefficient
              params.A_constraints[i](2) = a_xz(1);   // z coefficient
              Eigen::Matrix<tinytype, 2, 1> q_xz = obs_xz_t - r_obs * a_xz;
              params.x_max[i](0) = a_xz.dot(q_xz);
              cstr_active_count++;
            }
          }
        }
      }
      if (cstr_active_count > 0 && (cstr_log_cnt++ % 25 == 0)) {
        DEBUG_PRINT("OBS XZ: %d active, bot_z=%.2f, top_z=%.2f, drone=(%.2f,%.2f)\n", cstr_active_count,
                    (double)obs_center_bottom(2), (double)obs_center_top(2), 
                    (double)state_task.position.x, (double)state_task.position.z);
      }
      
      // Force cache_level=1 permanently once constraints have been activated
      // This prevents oscillation when constraint count goes to 0 temporarily
      static bool constraints_ever_active = false;
      if (cstr_active_count > 0) {
        constraints_ever_active = true;
      }
      if (constraints_ever_active) {
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

      // MPC solve (PSD runs every 5 ADMM iterations inside solve_admm)
      problem.iter = 0;

      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC solve start\n");
      }
      mpc_start_timestamp = usecTimestamp();
      solve_admm(&problem, &params);
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC solve done, iter=%d\n", problem.iter);
      }
      mpc_time_us = usecTimestamp() - mpc_start_timestamp;
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC time=%lu us\n", mpc_time_us);
      }

      // ================================================================
      // Safety Certificate (Section 3.4 of paper) - XZ PLANE
      // Trace gap: Δ = trace(X^(p)) - ||p||² = S(1,1) + S(2,2) - (px² + pz²)
      // Lifted margin: η = S(1,1) + S(2,2) - 2*ox*S(0,1) - 2*oz*S(0,2) + ox² + oz² - r²
      // Certified if: η ≥ 0 AND |Δ| ≤ η
      // Check certificate for BOTH obstacles, certified only if both pass
      // ================================================================
      static uint32_t cert_log_cnt = 0;
      bool certified_k0 = true;
      float trace_gap_k0 = 0.0f;
      float eta_min_k0 = 1000.0f;
      
      if (enable_psd && enable_obs_constraint) {
        // Check certificate for k=0 (current step) in XZ plane
        float px = problem.x(0, 0);
        float pz = problem.x(2, 0);  // XZ plane: use z instead of y
        
        // Get projected slack S from svec representation
        tiny_MatrixPsd Snew = smat_3x3(problem.Spsd_new.col(0));
        
        // Trace gap: Δ = trace(X^(p)) - ||p||²
        // trace(X^(p)) = S(1,1) + S(2,2)
        // ||p||² = px² + pz²
        trace_gap_k0 = (Snew(1,1) + Snew(2,2)) - (px*px + pz*pz);
        
        // Lifted margin for BOTTOM obstacle (XZ coordinates)
        float ox_b = obs_center_bottom(0);
        float oz_b = obs_center_bottom(2);
        float r = r_obs;
        float eta_bottom = Snew(1,1) + Snew(2,2) - 2.0f*ox_b*Snew(0,1) - 2.0f*oz_b*Snew(0,2) + ox_b*ox_b + oz_b*oz_b - r*r;
        
        // Lifted margin for TOP obstacle (XZ coordinates)
        float ox_t = obs_center_top(0);
        float oz_t = obs_center_top(2);
        float eta_top = Snew(1,1) + Snew(2,2) - 2.0f*ox_t*Snew(0,1) - 2.0f*oz_t*Snew(0,2) + ox_t*ox_t + oz_t*oz_t - r*r;
        
        // Use minimum eta (closest/most constraining obstacle)
        eta_min_k0 = (eta_bottom < eta_top) ? eta_bottom : eta_top;
        
        // Certificate check: certified if both obstacles pass
        bool cert_bottom = (eta_bottom >= 0.0f) && (fabsf(trace_gap_k0) <= eta_bottom);
        bool cert_top = (eta_top >= 0.0f) && (fabsf(trace_gap_k0) <= eta_top);
        certified_k0 = cert_bottom && cert_top;
        
        // Log periodically
        if (cert_log_cnt++ % 50 == 0) {
          DEBUG_PRINT("CERT XZ: %s Δ=%.3f η_bot=%.3f η_top=%.3f\n", 
                      certified_k0 ? "OK" : "FAIL",
                      (double)trace_gap_k0, (double)eta_bottom, (double)eta_top);
        }
        
        // Optional: emergency stop if uncertified (commented out for now)
        // if (!certified_k0) {
        //   DEBUG_PRINT("CERT FAIL: Emergency stop!\n");
        //   enable_traj = false;
        // }
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
#endif // USE_PLANNER_TRACKER_ARCH

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