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

// Select exactly one controller for this firmware image:
//   0 = nominal TinyMPC
//   1 = MPC-CBF (analytic linear constraints)
//   2 = LIMO posthoc
//   3 = LIMO embedded
// Change only the number below, run `make -j8`, then run `make cload`.
#define TINYMPC_MODE_NOMINAL       0
#define TINYMPC_MODE_MPC_CBF       1
#define TINYMPC_MODE_LIMO_POSTHOC  2
#define TINYMPC_MODE_LIMO_EMBEDDED 3


#ifndef TINYMPC_FIRMWARE_MODE
#define TINYMPC_FIRMWARE_MODE 3
#endif
#if TINYMPC_FIRMWARE_MODE < TINYMPC_MODE_NOMINAL || \
    TINYMPC_FIRMWARE_MODE > TINYMPC_MODE_LIMO_EMBEDDED
#error "TINYMPC_FIRMWARE_MODE must be 0 (nominal), 1 (MPC-CBF), 2 (LIMO posthoc), or 3 (LIMO embedded)"
#endif

// Select exactly one trajectory for this firmware image:
//   0 = hover
//   1 = X-axis line
//   2 = Y-axis line
//   3 = circle
//   4 = figure eight
// Change only the number below, run `make -j8`, then run `make cload`.
#define TINYMPC_TRAJECTORY_HOVER   0
#define TINYMPC_TRAJECTORY_LINE_X  1
#define TINYMPC_TRAJECTORY_LINE_Y  2
#define TINYMPC_TRAJECTORY_CIRCLE  3
#define TINYMPC_TRAJECTORY_FIGURE8 4


#ifndef TINYMPC_TRAJECTORY
#define TINYMPC_TRAJECTORY 0
#endif
#if TINYMPC_TRAJECTORY < TINYMPC_TRAJECTORY_HOVER || \
    TINYMPC_TRAJECTORY > TINYMPC_TRAJECTORY_FIGURE8
#error "TINYMPC_TRAJECTORY must be 0 (hover), 1 (X-line), 2 (Y-line), 3 (circle), or 4 (figure eight)"
#endif

// Every maneuver finishes with the same PID-controlled descent. The landing
// target is intentionally below the estimated floor so the vehicle settles
// before the motors are disabled.
#define TINYMPC_LANDING_DESCENT_MPS       0.15f
#define TINYMPC_LANDING_TARGET_Z_M       (-0.05f)
#define TINYMPC_LANDING_TOUCHDOWN_Z_M     0.08f
#define TINYMPC_LANDING_TOUCHDOWN_VZ_MPS  0.10f

// Compact XYZ console samples: 0=off, 1=20 Hz, 2=10 Hz, 4=5 Hz.
// Ten hertz is sufficient for hover RMSE without recreating the CRTP
// starvation caused by printing a full diagnostic block every MPC update.
#define TINYMPC_XYZ_CONSOLE_DECIMATION 2
#if TINYMPC_XYZ_CONSOLE_DECIMATION < 0
#error "TINYMPC_XYZ_CONSOLE_DECIMATION must be zero or positive"
#endif

#include "Eigen.h"

// TinyMPC headers (C++, must be before extern "C")
#include "tinympc/admm.hpp"
#include "tinympc/psd_support.hpp"
#include "limo_shared_baseline.hpp"
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_POSTHOC || \
    TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
#include "limo_barrier.hpp"
#endif
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
#include "limo_embedded.hpp"
#endif

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

// Every benchmark arm uses the exact 20 Hz model/cache timing from LIMO.
#define MPC_RATE 20
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
enum FlightPhase : uint8_t {
  FLIGHT_PHASE_TRACKING = 0,
  FLIGHT_PHASE_LANDING = 1,
  FLIGHT_PHASE_COMPLETE = 2,
};

static bool enable_traj = true;
static bool mpc_has_run = false; // Flag to track if MPC has computed at least once
static int traj_index = 0;
static int max_traj_index = 0;
static float traj_speed = 0.2f; // m/s
static float traj_dist = 1.0f;  // m
static float traj_height = 0.5f;
static float traj_duration = 12.0f;
static float traj_radius = 0.75f;
static float traj_omega = 0.45f;
static uint32_t last_controller_tick = 0;
static uint32_t controller_activate_tick = 0;
static uint8_t flight_phase = FLIGHT_PHASE_TRACKING;
static uint32_t landing_start_tick = 0;
static float landing_hold_x = 0.0f;
static float landing_hold_y = 0.0f;
static float landing_hold_yaw = 0.0f;
static float landing_start_z = 0.0f;
static float landing_reference_z = 0.0f;
// static int mpc_steps_taken = 0;
// static uint32_t timestamp;
static uint32_t mpc_start_timestamp;
static uint32_t mpc_time_us;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
static bool isInit = false;
static int prev_cache_level = 0; // Track cache_level changes
// Kept as bytes for logging. They are deliberately not runtime parameters:
// changing the controller or trajectory requires building/flashing an image.
static uint8_t benchmark_mode = TINYMPC_FIRMWARE_MODE;
static uint8_t benchmark_maneuver = TINYMPC_TRAJECTORY;
static uint8_t previous_benchmark_mode = 255;
static uint8_t previous_benchmark_maneuver = 255;
// One barrier evaluation plus five ADMM iterations fits the embedded LIMO
// controller's 50 ms period with margin; six ran it at ~49 of 50 ms on
// hardware. Larger counts must be revalidated on hardware.
static uint8_t benchmark_max_iter = 5;
static uint8_t enable_obs_constraint = 0; // Obstacle LTV constraints disabled for LIMO deploy
static uint8_t enable_psd = 0; // PSD disabled for LIMO deploy

static tinytype limo_margin = tinytype(0.01f);
static tinytype limo_margin_scale = tinytype(1.5f);
static tinytype limo_h_deadband = tinytype(0.30f);
static tinytype limo_act_slack = tinytype(0.20f);
static tinytype limo_demand_cap = tinytype(0.05f);
static tinytype limo_az_coeff = tinytype(8.0f);
static tinytype limo_gravity_comp = tinytype(0.0f);
static tinytype limo_fail_roll_deg = tinytype(50.0f);
static tinytype limo_fail_pitch_deg = tinytype(50.0f);
static tinytype limo_structural_guard_relax = tinytype(0.70f);
static tinytype limo_skip_z = tinytype(0.75f);
static tinytype limo_skip_vz = tinytype(-0.20f);
static uint8_t limo_active_horizon = 3;
static tinytype limo_dist_budget = tinytype(1.6f);
static tinytype limo_dist_margin_coeff = tinytype(0.018f);
static tinytype limo_dist_high_cut = tinytype(1.35f);
static tinytype limo_dist_high_bias = tinytype(0.030f);
static tinytype limo_dist_floor_band = tinytype(0.24f);
static tinytype limo_dist_floor_gain = tinytype(0.10f);
static tinytype limo_dist_desc_band = tinytype(0.20f);
static tinytype limo_dist_desc_gain = tinytype(0.02f);
static tinytype limo_horizon_margin_rate = tinytype(0.0008f);
static float limo_h = 0.0f;
static float limo_raw = 0.0f;
static float limo_grad_norm = 0.0f;
static float limo_margin_eff = 0.0f;
static float limo_threshold = 0.0f;
static uint32_t limo_eval_us = 0;
static uint8_t limo_active = 0;
static uint8_t limo_active_count = 0;
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
static limo_embedded::Runtime limo_embedded_runtime = {};
#endif
static float limo_authority_w = 0.0f;
static float limo_authority_w_requested = 0.0f;
static float limo_qz = 1.0f;
static uint8_t limo_w_index = 0;
static uint8_t limo_qz_index = 0;
static uint8_t limo_cache_ok = 0;
static float limo_no_oracle_score = 0.0f;
static float limo_no_oracle_score_raw = 0.0f;
static uint8_t limo_no_oracle_active = 0;
static uint8_t posthoc_active = 0;
static uint8_t posthoc_failed = 0;
static float posthoc_du_norm = 0.0f;
static uint32_t benchmark_step = 0;

// Raw state logging for offline wind-response analysis. Hover references are
// known (x=0, y=0, z=traj_height), so RMSE/peaks/recovery are reconstructed
// from the timestamped samples instead of spending firmware work on metrics.
static float tracking_pos_x = 0.0f;
static float tracking_pos_y = 0.0f;
static float tracking_pos_z = 0.0f;
static float tracking_vel_x = 0.0f;
static float tracking_vel_y = 0.0f;
static float tracking_vel_z = 0.0f;
static float tracking_cmd_z = 0.0f;
static float tracking_preview_z = 0.0f;
static uint32_t controller_total_us = 0;

// Dynamic obstacle (disk) parameters for LTV linear constraints
static Eigen::Matrix<tinytype, 3, 1> obs_center;
static Eigen::Matrix<tinytype, 3, 1> obs_start;     // Initial obstacle position
static Eigen::Matrix<tinytype, 3, 1> obs_velocity;  // Obstacle velocity (m/s)
static Eigen::Matrix<tinytype, 3, 1> xc;
static Eigen::Matrix<tinytype, 3, 1> a_norm;
static Eigen::Matrix<tinytype, 3, 1> q_c;
static float r_obs = 0.35f;           // Obstacle radius
static float obs_activation_margin = 0.15f; // Constraint activation distance
static uint64_t obs_start_time = 0;   // Time when obstacle motion started

static inline tinytype positive_part(tinytype value)
{
  return value > tinytype(0.0f) ? value : tinytype(0.0f);
}

static inline tinytype clamp_tiny(tinytype value,
                                  tinytype lower,
                                  tinytype upper)
{
  return value < lower ? lower : (value > upper ? upper : value);
}

#if TINYMPC_FIRMWARE_MODE != TINYMPC_MODE_NOMINAL
static tinytype limo_effective_margin(const tiny_VectorNx &xbar, int stage)
{
  const tinytype budget = positive_part(limo_dist_budget);
  const tinytype high_budget = positive_part(budget - positive_part(limo_dist_high_cut));
  tinytype margin = positive_part(limo_margin);

  margin += positive_part(limo_dist_margin_coeff) * budget;
  margin += high_budget * positive_part(limo_dist_floor_gain) *
            positive_part(positive_part(limo_dist_floor_band) - xbar(2));
  margin += high_budget * positive_part(limo_dist_desc_gain) *
            positive_part(-xbar(8) - positive_part(limo_dist_desc_band));
  margin += positive_part(limo_horizon_margin_rate) * static_cast<tinytype>(stage);
  margin += positive_part(limo_dist_high_bias) * high_budget;
  margin *= positive_part(limo_margin_scale);

  return margin;
}
#endif

static inline tinytype structural_floor_h(const tiny_VectorNx &x)
{
  return x(2) - tinytype(0.20f) * positive_part(-x(8));
}

static inline tinytype guarded_limo_h(const tiny_VectorNx &x, tinytype learned_h)
{
  const tinytype guarded_structural =
      structural_floor_h(x) - positive_part(limo_structural_guard_relax);
  return learned_h > guarded_structural ? learned_h : guarded_structural;
}

static inline bool barrier_skipped_high_altitude(const tiny_VectorNx &x)
{
  return x(2) > limo_skip_z && x(8) > limo_skip_vz;
}

static void reset_solver_warm_start()
{
  problem.y.setZero();
  problem.g.setZero();
  problem.v.setZero();
  problem.vnew.setZero();
  problem.z.setZero();
  problem.znew.setZero();
  problem.cache_level = 0;
  prev_cache_level = 0;
}

static void restore_shared_baseline()
{
  // Frozen w=0, qz=1 entry of LIMO's canonical 20 Hz cache bank.
  // All non-adaptive benchmark arms remain on this exact baseline.
  limo_shared_baseline::install(&params);
}

#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_POSTHOC
// Firmware port of safe-reachability's solve_posthoc_cbf_qp. The nominal
// action is the box-projected ADMM iterate z, exactly as in posthoc_learned.
static tiny_VectorNu project_posthoc_limo(const tiny_VectorNu &nominal,
                                          tinytype h,
                                          tinytype margin,
                                          tinytype dh_dz,
                                          tinytype dh_dvz,
                                          bool *active,
                                          bool *failed,
                                          tinytype *du_norm)
{
  tiny_VectorNu command =
      nominal.cwiseMax(params.u_min.col(0)).cwiseMin(params.u_max.col(0));
  *active = false;
  *failed = false;
  *du_norm = 0.0f;

  const tinytype deficit = margin - h;
  if (deficit <= 0.0f) {
    return command;
  }
  *active = true;

  tiny_VectorNu input_gradient = tiny_VectorNu::Zero();
  for (int j = 0; j < NINPUTS; ++j) {
    input_gradient(j) =
        dh_dz * params.cache.Bdyn[problem.cache_level](2, j) +
        dh_dvz * params.cache.Bdyn[problem.cache_level](8, j);
  }
  if (input_gradient.squaredNorm() <= 1e-12f) {
    *failed = true;
    return command;
  }
  const tiny_VectorNu nominal_clamped = command;
  const tinytype target = input_gradient.dot(nominal_clamped) + deficit;
  tinytype maximum = 0.0f;
  tiny_VectorNu maximum_command;
  for (int j = 0; j < NINPUTS; ++j) {
    maximum_command(j) = input_gradient(j) >= 0.0f
        ? params.u_max(j, 0) : params.u_min(j, 0);
    maximum += input_gradient(j) * maximum_command(j);
  }
  if (maximum < target - 1e-6f) {
    *du_norm = (maximum_command - command).norm();
    *failed = true;
    return maximum_command;
  }
  tinytype low = 0.0f;
  tinytype high = 1.0f;
  const auto affine_at = [&](tinytype lambda, tiny_VectorNu *result) {
    tiny_VectorNu candidate =
        (nominal_clamped + lambda * input_gradient)
            .cwiseMax(params.u_min.col(0))
            .cwiseMin(params.u_max.col(0));
    if (result) {
      *result = candidate;
    }
    return input_gradient.dot(candidate);
  };

  tinytype phi_high = affine_at(high, nullptr);
  int grow = 0;
  while (phi_high < target && grow < 80) {
    high *= 2.0f;
    phi_high = affine_at(high, nullptr);
    ++grow;
  }
  if (phi_high < target - 1e-6f) {
    *du_norm = (maximum_command - command).norm();
    *failed = true;
    return maximum_command;
  }

  for (int iteration = 0; iteration < 80; ++iteration) {
    const tinytype middle = 0.5f * (low + high);
    tiny_VectorNu candidate;
    if (affine_at(middle, &candidate) >= target) {
      high = middle;
      command = candidate;
    } else {
      low = middle;
    }
  }
  *du_norm = (command - nominal_clamped).norm();
  const tinytype h_linearized =
      h + input_gradient.dot(command - nominal_clamped);
  *failed = h_linearized < margin - 1e-4f;
  return command;
}
#endif

#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_POSTHOC
static void reroll_horizon_from_first_input()
{
  for (int i = 0; i < NHORIZON - 1; ++i) {
    problem.x.col(i + 1).noalias() =
        params.cache.Adyn[problem.cache_level] * problem.x.col(i) +
        params.cache.Bdyn[problem.cache_level] * problem.u.col(i);
  }
}
#endif

static void reset_benchmark_run()
{
  traj_index = 0;
  benchmark_step = 0;
  enable_traj = true;
  mpc_has_run = false;
  flight_phase = FLIGHT_PHASE_TRACKING;
  landing_start_tick = 0;
  landing_reference_z = traj_height;
  max_traj_index =
      static_cast<int>(positive_part(traj_duration) * MPC_RATE);
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
  limo_embedded_runtime = {};
#endif
  limo_cache_ok = 0;
  limo_no_oracle_score = 0.0f;
  limo_no_oracle_score_raw = 0.0f;
  limo_no_oracle_active = 0;
  posthoc_active = 0;
  posthoc_failed = 0;
  posthoc_du_norm = 0.0f;
  reset_solver_warm_start();
}

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

static inline void stop_motors(control_t *control)
{
  memset(control, 0, sizeof(control_t));
  control->controlMode = controlModeLegacy;
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

  restore_shared_baseline();
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

  // Initialize problem data to zero
  resetProblem();

  problem.primal_residual_state = 0;
  problem.primal_residual_input = 0;
  problem.dual_residual_state = 0;
  problem.dual_residual_input = 0;
  problem.abs_tol = 0.001;
  problem.status = 0;
  problem.iter = 0;
  problem.max_iter = benchmark_max_iter;
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
  max_traj_index = static_cast<int>(traj_duration * MPC_RATE);
  flight_phase = FLIGHT_PHASE_TRACKING;
  landing_start_tick = 0;
  landing_reference_z = traj_height;

  // Dynamic obstacle - arm sweeps from left (y+) to right (y-)
  // Arm starts at y=+0.3, sweeps down to y=-0.3 at 0.1 m/s
  obs_start << 0.7f, 0.3f, 0.5f;      // Start position (left side, in drone path)
  obs_velocity << 0.0f, -0.1f, 0.0f; // Sweeps left-to-right at 0.1 m/s
  obs_center = obs_start;             // Initial position
  obs_start_time = 0;                 // Will be set on first MPC solve

  // The benchmark isolates the four requested controllers. Legacy obstacle
  // and PSD paths stay disabled so every mode sees the same plant/reference.
  enable_obs_constraint = 0;
  enable_psd = 0;
  problem.en_psd = 0;
  DEBUG_PRINT("Benchmark mode %u, maneuver %u, max_iter %u, task_pri %u\n",
              (unsigned int)benchmark_mode,
              (unsigned int)benchmark_maneuver,
              (unsigned int)benchmark_max_iter,
              (unsigned int)TINYMPC_TASK_PRI);

  DEBUG_PRINT("Common baseline: mpc_hz=20 model_hz=20 rho=5 base_w=0 base_qz=1 Qz=100 R=4 bridge=x4 guards=z[ref,ref+0.20],xy+-0.10 cap=0.05\n");
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
  DEBUG_PRINT("LIMO adaptive: cache=16x4 evals=1\n");
#else
  DEBUG_PRINT("Fixed baseline: w=0 qz=1 adaptive_cache=off\n");
#endif
#if TINYMPC_XYZ_CONSOLE_DECIMATION > 0
  DEBUG_PRINT("XYZ console: every=%u MPC steps; fields=mode,step,x,y,z\n",
              (unsigned int)TINYMPC_XYZ_CONSOLE_DECIMATION);
#endif
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
  (void)setpoint;
  if (enable_traj)
  {
    const float dt = 1.0f / MPC_RATE;
    const float base_t = traj_index * dt;
    for (int i = 0; i < NHORIZON; ++i) {
      const float t = base_t + i * dt;
      float x = 0.0f;
      float y = 0.0f;
      float vx = 0.0f;
      float vy = 0.0f;
#if TINYMPC_TRAJECTORY == TINYMPC_TRAJECTORY_HOVER
      // Position and velocity stay at zero in X/Y.
      (void)t;
#elif TINYMPC_TRAJECTORY == TINYMPC_TRAJECTORY_LINE_X
      x = fminf(traj_speed * t, traj_dist);
      vx = traj_speed * t < traj_dist ? traj_speed : 0.0f;
#elif TINYMPC_TRAJECTORY == TINYMPC_TRAJECTORY_LINE_Y
      y = fminf(traj_speed * t, traj_dist);
      vy = traj_speed * t < traj_dist ? traj_speed : 0.0f;
#elif TINYMPC_TRAJECTORY == TINYMPC_TRAJECTORY_CIRCLE
      // Starts at the origin with continuous position.
      x = traj_radius * sinf(traj_omega * t);
      y = traj_radius * (1.0f - cosf(traj_omega * t));
      vx = traj_radius * traj_omega * cosf(traj_omega * t);
      vy = traj_radius * traj_omega * sinf(traj_omega * t);
#elif TINYMPC_TRAJECTORY == TINYMPC_TRAJECTORY_FIGURE8
      x = traj_radius * sinf(traj_omega * t);
      y = 0.5f * traj_radius * sinf(2.0f * traj_omega * t);
      vx = traj_radius * traj_omega * cosf(traj_omega * t);
      vy = traj_radius * traj_omega * cosf(2.0f * traj_omega * t);
#endif
      params.Xref.col(i).setZero();
      params.Xref(0, i) = x;
      params.Xref(1, i) = y;
      params.Xref(2, i) = traj_height;
      params.Xref(6, i) = vx;
      params.Xref(7, i) = vy;
    }
    Xref_end = params.Xref.col(NHORIZON - 1);

    if (traj_index < max_traj_index) {
      traj_index++;
    } else {
      // Stop the benchmark controller before landing so a floor constraint
      // cannot oppose the commanded descent.
      DEBUG_PRINT("TRAJ DONE: idx=%d, max=%d; requesting landing\n",
                  traj_index, max_traj_index);
      enable_traj = false;
      enable_obs_constraint = 0;
      params.Xref = Xref_end.replicate<1, NHORIZON>();
      landing_start_tick = 0;
      flight_phase = FLIGHT_PHASE_LANDING;
    }
  }
  else
  {
    params.Xref = Xref_end.replicate<1, NHORIZON>();
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

      if (benchmark_mode != previous_benchmark_mode ||
          benchmark_maneuver != previous_benchmark_maneuver) {
        previous_benchmark_mode = benchmark_mode;
        previous_benchmark_maneuver = benchmark_maneuver;
        restore_shared_baseline();
        reset_benchmark_run();
        DEBUG_PRINT("Benchmark reset: mode=%u maneuver=%u\n",
                    (unsigned int)benchmark_mode,
                    (unsigned int)benchmark_maneuver);
      }
      problem.max_iter = benchmark_max_iter > 0 ? benchmark_max_iter : 1;

      // Landing uses the stock position PID. Stop TinyMPC/LIMO so the
      // benchmark's floor-safety constraint cannot fight the descent.
      if (flight_phase != FLIGHT_PHASE_TRACKING) {
        continue;
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

      tracking_pos_x = state_task.position.x;
      tracking_pos_y = state_task.position.y;
      tracking_pos_z = state_task.position.z;
      tracking_vel_x = state_task.velocity.x;
      tracking_vel_y = state_task.velocity.y;
      tracking_vel_z = state_task.velocity.z;
      
      if (task_loop_count <= 3) {
        DEBUG_PRINT("ref: (%.2f,%.2f,%.2f)\n",
                    (double)params.Xref(0,0), (double)params.Xref(1,0), (double)params.Xref(2,0));
      }

      limo_cache_ok = 0;
      limo_authority_w = 0.0f;
      limo_authority_w_requested = 0.0f;
      limo_qz = 1.0f;
      limo_w_index = 0;
      limo_qz_index = 0;
      limo_no_oracle_score = 0.0f;
      limo_no_oracle_score_raw = 0.0f;
      limo_no_oracle_active = 0;
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
      // One MLP forward/backward pass per MPC update. The same local
      // linearization is reused over the short active horizon below.
      LimoBarrierEval embedded_eval_x0;
      uint32_t embedded_eval_us = 0;
      {
        const uint32_t embedded_eval_start_us = usecTimestamp();
        limo_eval_barrier(
            problem.x.col(0), limo_az_coeff, limo_gravity_comp,
            radians(limo_fail_roll_deg), radians(limo_fail_pitch_deg),
            &embedded_eval_x0);
        embedded_eval_us = usecTimestamp() - embedded_eval_start_us;
        const tinytype embedded_h =
            guarded_limo_h(problem.x.col(0), embedded_eval_x0.h);
        const tinytype embedded_margin =
            limo_effective_margin(problem.x.col(0), 0);
        limo_embedded::observe_no_oracle(
            &limo_embedded_runtime, problem.x.col(0), embedded_h,
            embedded_margin, radians(limo_fail_roll_deg),
            radians(limo_fail_pitch_deg));
        limo_embedded::update(
            &limo_embedded_runtime, problem.x.col(0), params.Xref.col(0),
            embedded_h, embedded_margin, radians(limo_fail_roll_deg),
            radians(limo_fail_pitch_deg));
        limo_cache_ok =
            limo_embedded::install_cache(limo_embedded_runtime, &params) ? 1 : 0;
        limo_authority_w = limo_embedded_runtime.applied_w;
        limo_authority_w_requested = limo_embedded_runtime.requested_w;
        limo_qz = limo_embedded_runtime.applied_qz;
        limo_w_index =
            static_cast<uint8_t>(limo_embedded_runtime.w_index);
        limo_qz_index =
            static_cast<uint8_t>(limo_embedded_runtime.qz_index);
        limo_no_oracle_score = limo_embedded_runtime.no_oracle_score;
        limo_no_oracle_score_raw =
            limo_embedded_runtime.no_oracle_score_raw;
        limo_no_oracle_active =
            limo_embedded_runtime.no_oracle_active ? 1 : 0;
        if (!limo_cache_ok) {
          DEBUG_PRINT("LIMO embedded cache validation failed\n");
          // A compiled embedded-LIMO image cannot silently switch to another
          // benchmark arm. End the run with the same controlled landing.
          enable_traj = false;
          mpc_has_run = true;
          landing_start_tick = 0;
          flight_phase = FLIGHT_PHASE_LANDING;
          restore_shared_baseline();
          reset_solver_warm_start();
        }
      }
#endif

      float obs_elapsed = 0.0f;
      if (enable_obs_constraint) {
        // Dynamic obstacle - update position based on elapsed time
        // Arm sweeps from left (y+) to right (y-) starting when OOT activates
        if (obs_start_time == 0) {
          obs_start_time = usecTimestamp();  // Start timer on first solve
        }
        obs_elapsed = (usecTimestamp() - obs_start_time) / 1e6f;
        obs_center = obs_start + obs_velocity * obs_elapsed;
        // Clamp obstacle position to reasonable range
        if (obs_center(1) < -0.4f) obs_center(1) = -0.4f;
        if (obs_center(1) > 0.4f) obs_center(1) = 0.4f;
      }
      
      // Update PSD obstacle position
      if (enable_psd) {
        problem.psd_obs_x = obs_center(0);
        problem.psd_obs_y = obs_center(1);
      }

      // Dynamic obstacle avoidance via LTV linear constraints
      const bool constraint_hold =
          (!mpc_has_run) || ((xTaskGetTickCount() - controller_activate_tick) < M2T(500));
      static uint32_t cstr_log_cnt = 0;
      int cstr_active_count = 0;
      const float dt_horizon = 1.0f / MPC_RATE;  // Time step per horizon
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
      limo_eval_us = embedded_eval_us;
#else
      limo_eval_us = 0;
#endif
      limo_h = 0.0f;
      limo_raw = 0.0f;
      limo_active = 0;
      limo_active_count = 0;
      limo_grad_norm = 0.0f;
      limo_margin_eff = 0.0f;
      limo_threshold = 0.0f;
      posthoc_active = 0;
      posthoc_failed = 0;
      posthoc_du_norm = 0.0f;
      
      for (int i = 0; i < NHORIZON; i++)
      {
        params.x_min[i] = tiny_VectorNc::Constant(-1000);
        params.x_max[i] = tiny_VectorNc::Constant(1000);
        params.A_constraints[i] = tiny_MatrixNcNx::Zero();

#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_MPC_CBF || \
    TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
        if (!constraint_hold && i < limo_active_horizon) {
          const tiny_VectorNx xbar = problem.x.col(i);
          tiny_VectorNx grad = tiny_VectorNx::Zero();
          tinytype barrier_h = 0.0f;
          tinytype constraint_h = 0.0f;
          tinytype raw_h = 0.0f;
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_MPC_CBF
          {
            barrier_h = structural_floor_h(xbar);
            constraint_h = barrier_h;
            raw_h = barrier_h;
            grad(2) = 1.0f;
            grad(8) = xbar(8) < 0.0f ? 0.20f : 0.0f;
          }
#else
          {
            // Frozen first-order model about the measured x0. This preserves
            // the LTV constraint form without paying for three additional
            // 128x128 MLP backward passes on the F405.
            grad = embedded_eval_x0.grad;
            const tinytype linearized_delta =
                grad.dot(xbar - problem.x.col(0));
            raw_h = embedded_eval_x0.raw + linearized_delta;
            constraint_h = embedded_eval_x0.h + linearized_delta;
            barrier_h = guarded_limo_h(xbar, constraint_h);
          }
#endif
          for (int j = 0; j < NSTATES; ++j) {
            grad(j) = clamp_tiny(grad(j), tinytype(-2.0f), tinytype(2.0f));
          }
          const tinytype grad_norm = grad.norm();
          const tinytype margin_eff = limo_effective_margin(xbar, i);
          const tinytype activation_threshold =
              limo_h_deadband > (margin_eff + limo_act_slack) ? limo_h_deadband : (margin_eff + limo_act_slack);
          const bool skipped = barrier_skipped_high_altitude(xbar);
          const bool active = !skipped && (grad_norm > tinytype(1e-6f)) &&
                              (barrier_h < activation_threshold);

          if (i == 0) {
            limo_h = barrier_h;
            limo_raw = raw_h;
            limo_grad_norm = grad_norm;
            limo_margin_eff = margin_eff;
            limo_threshold = activation_threshold;
            limo_active = active ? 1 : 0;
          }

          if (active) {
            params.A_constraints[i] = -grad.transpose();
            // Ask for at most a per-step-reachable h recovery. The full
            // deficit (margin - h, ~0.27 at benign hover) cannot be closed
            // within one 50 ms step, and demanding it winds up the ADMM
            // duals until the truncated solve outputs a descent transient.
            tinytype demand = margin_eff - constraint_h;
            if (demand > limo_demand_cap) {
              demand = limo_demand_cap;
            }
            params.x_max[i](0) = -(grad.dot(xbar) + demand);
            cstr_active_count++;
            if (limo_active_count < 255) {
              limo_active_count++;
            }
          }
        }
#endif

        if (enable_obs_constraint && !constraint_hold) {
          // Predict obstacle position for this horizon step
          float future_t = obs_elapsed + i * dt_horizon;
          Eigen::Matrix<tinytype, 3, 1> obs_pred = obs_start + obs_velocity * future_t;
          if (obs_pred(1) < -0.4f) obs_pred(1) = -0.4f;
          if (obs_pred(1) > 0.4f) obs_pred(1) = 0.4f;
          
          // Use reference position to define the tangent half-space
          Eigen::Matrix<tinytype, 3, 1> ref = params.Xref.col(i).head(3);
          xc = ref - obs_pred; // points from predicted obstacle to reference
          float xc_norm = xc.norm();
          if (xc_norm > 1e-3f && xc_norm < (r_obs + obs_activation_margin)) {
            a_norm = -xc / xc_norm; // inward normal (for A x <= b)
            params.A_constraints[i].head(3) = a_norm.transpose();
            q_c = obs_pred - r_obs * a_norm;
            params.x_max[i](0) = a_norm.transpose() * q_c;
            cstr_active_count++;
          }
        }
      }
      if (enable_obs_constraint && cstr_active_count > 0 &&
          (cstr_log_cnt++ % 25 == 0)) {
        DEBUG_PRINT("OBS: %d active, obs_y=%.2f, drone=(%.2f,%.2f)\n", cstr_active_count,
                    (double)obs_center(1), (double)state_task.position.x, (double)state_task.position.y);
      }
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_MPC_CBF || \
    TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
      if (task_loop_count <= 3) {
        DEBUG_PRINT("SAFE TV mode=%u: h=%.3f raw=%.3f grad=%.3f margin=%.3f thr=%.3f active0=%u active_count=%u eval=%lu us\n",
                    (unsigned int)benchmark_mode,
                    (double)limo_h, (double)limo_raw, (double)limo_grad_norm,
                    (double)limo_margin_eff, (double)limo_threshold,
                    (unsigned int)limo_active, (unsigned int)limo_active_count, limo_eval_us);
      }
#endif
      
      const int requested_cache_level = cstr_active_count > 0 ? 1 : 0;
      if (requested_cache_level != prev_cache_level) {
        DEBUG_PRINT("Cache level changed: %d -> %d\n",
                    prev_cache_level, requested_cache_level);
        reset_solver_warm_start();
        problem.cache_level = requested_cache_level;
        prev_cache_level = requested_cache_level;
      } else {
        problem.cache_level = requested_cache_level;
      }
      if (problem.cache_level == 0) {
        problem.y.setZero();
        problem.g.setZero();
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
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_POSTHOC
      {
        LimoBarrierEval posthoc_eval;
        const uint32_t eval_start_us = usecTimestamp();
        limo_eval_barrier(
            problem.x.col(0), limo_az_coeff, limo_gravity_comp,
            radians(limo_fail_roll_deg), radians(limo_fail_pitch_deg),
            &posthoc_eval);
        limo_eval_us += usecTimestamp() - eval_start_us;

        const tinytype posthoc_h =
            guarded_limo_h(problem.x.col(0), posthoc_eval.h);
        const tinytype posthoc_margin =
            limo_effective_margin(problem.x.col(0), 0);
        // The canonical posthoc arm consumes z.col(0), not the unconstrained
        // primal u.col(0). Keeping that distinction is important at a
        // truncated ADMM iteration count.
        const tiny_VectorNu nominal_command = problem.z.col(0);
        tiny_VectorNu filtered_command = nominal_command;
        bool projection_active = false;
        bool projection_failed = false;
        tinytype projection_du_norm = 0.0f;
        if (!barrier_skipped_high_altitude(problem.x.col(0))) {
          filtered_command = project_posthoc_limo(
              nominal_command, posthoc_h, posthoc_margin,
              posthoc_eval.grad(2), posthoc_eval.grad(8),
              &projection_active, &projection_failed,
              &projection_du_norm);
        }
        posthoc_du_norm = projection_du_norm;
        posthoc_active = projection_active ? 1 : 0;
        posthoc_failed = projection_failed ? 1 : 0;

        // This task-controller drives the stock Crazyflie PID with the MPC
        // terminal state. Apply the exact canonical delta-u to the primal
        // horizon and reroll it so the posthoc action reaches that bridge
        // without changing the nominal no-intervention path.
        if (posthoc_du_norm > 1e-5f) {
          problem.u.col(0) =
              (problem.u.col(0) + filtered_command - nominal_command)
                  .cwiseMax(params.u_min.col(0))
                  .cwiseMin(params.u_max.col(0));
          reroll_horizon_from_first_input();
        }

        limo_h = posthoc_h;
        limo_raw = posthoc_eval.raw;
        limo_grad_norm = sqrtf(
            posthoc_eval.grad(2) * posthoc_eval.grad(2) +
            posthoc_eval.grad(8) * posthoc_eval.grad(8));
        limo_margin_eff = posthoc_margin;
        limo_threshold = posthoc_margin;
        limo_active = posthoc_active;
        limo_active_count = posthoc_active;
      }
#endif
#if TINYMPC_FIRMWARE_MODE == TINYMPC_MODE_LIMO_EMBEDDED
      {
        limo_embedded::update_saturation(
            &limo_embedded_runtime, problem.u.col(0),
            params.u_min.col(0), params.u_max.col(0));
        limo_embedded::save_prediction(
            &limo_embedded_runtime, problem.x.col(1));
      }
#endif
      ++benchmark_step;
#if TINYMPC_XYZ_CONSOLE_DECIMATION > 0
      if ((benchmark_step % TINYMPC_XYZ_CONSOLE_DECIMATION) == 0) {
        DEBUG_PRINT("XYZ mode=%u step=%lu x=%.3f y=%.3f z=%.3f\n",
                    (unsigned int)benchmark_mode,
                    (unsigned long)benchmark_step,
                    (double)tracking_pos_x, (double)tracking_pos_y,
                    (double)tracking_pos_z);
      }
#endif
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC solve done, iter=%d\n", problem.iter);
      }
      mpc_time_us = usecTimestamp() - mpc_start_timestamp;
      controller_total_us = mpc_time_us + limo_eval_us;
      if (task_loop_count <= 3) {
        DEBUG_PRINT("MPC time=%lu us\n", mpc_time_us);
      }
      if (task_loop_count == 1) {
        DEBUG_PRINT("MPC stack free=%lu words\n",
                    (unsigned long)uxTaskGetStackHighWaterMark(NULL));
      }

      // ================================================================
      // Safety Certificate (Section 3.4 of paper)
      // Trace gap: Δ = trace(X^(p)) - ||p||² = S(1,1) + S(2,2) - (px² + py²)
      // Lifted margin: η = S(1,1) + S(2,2) - 2*ox*S(0,1) - 2*oy*S(0,2) + ox² + oy² - r²
      // Certified if: η ≥ 0 AND |Δ| ≤ η
      // ================================================================
      static uint32_t cert_log_cnt = 0;
      bool certified_k0 = true;
      float trace_gap_k0 = 0.0f;
      float eta_min_k0 = 1000.0f;
      
      if (enable_psd && enable_obs_constraint) {
        // Check certificate for k=0 (current step)
        float px = problem.x(0, 0);
        float py = problem.x(1, 0);
        
        // Get projected slack S from svec representation
        tiny_MatrixPsd Snew = smat_3x3(problem.Spsd_new.col(0));
        
        // Trace gap: Δ = trace(X^(p)) - ||p||²
        // trace(X^(p)) = S(1,1) + S(2,2)
        // ||p||² = px² + py²
        trace_gap_k0 = (Snew(1,1) + Snew(2,2)) - (px*px + py*py);
        
        // Lifted margin for obstacle
        float ox = obs_center(0);
        float oy = obs_center(1);
        float r = r_obs;
        eta_min_k0 = Snew(1,1) + Snew(2,2) - 2.0f*ox*Snew(0,1) - 2.0f*oy*Snew(0,2) + ox*ox + oy*oy - r*r;
        
        // Certificate check
        certified_k0 = (eta_min_k0 >= 0.0f) && (fabsf(trace_gap_k0) <= eta_min_k0);
        
        // Log periodically
        if (cert_log_cnt++ % 50 == 0) {
          DEBUG_PRINT("CERT: %s Δ=%.3f η=%.3f\n", 
                      certified_k0 ? "OK" : "FAIL",
                      (double)trace_gap_k0, (double)eta_min_k0);
        }
        
        // Optional: emergency stop if uncertified (commented out for now)
        // if (!certified_k0) {
        //   DEBUG_PRINT("CERT FAIL: Emergency stop!\n");
        //   enable_traj = false;
        // }
      }

      // Keep the stock 500 Hz PID as the hardware inner loop. A 200 ms MPC
      // preview gives it a meaningful outer-loop target without exposing the
      // brushless vehicle to the unstable 20 Hz direct-motor path.
      constexpr int kPidBridgeIndex = 4;
      const tinytype preview_z = problem.x(2, kPidBridgeIndex);
      const tinytype reference_z = params.Xref(2, 0);
      mpc_setpoint_task = problem.x.col(kPidBridgeIndex);
      tracking_preview_z = preview_z;
      // Tracking/LIMO may request extra altitude for safety, but the partial
      // solve is never allowed to pull the vehicle below the nominal path.
      mpc_setpoint_task(2) =
          clamp_tiny(preview_z, reference_z, reference_z + tinytype(0.20f));
      tracking_cmd_z = mpc_setpoint_task(2);
      // Under slow drift the preview follows current velocity, which bypasses
      // the PID's position stiffness; anchor the lateral target to the
      // reference so wind cannot walk the vehicle off the path.
      mpc_setpoint_task(0) = clamp_tiny(mpc_setpoint_task(0),
                                        params.Xref(0, 0) - tinytype(0.10f),
                                        params.Xref(0, 0) + tinytype(0.10f));
      mpc_setpoint_task(1) = clamp_tiny(mpc_setpoint_task(1),
                                        params.Xref(1, 0) - tinytype(0.10f),
                                        params.Xref(1, 0) + tinytype(0.10f));
      
      if (task_loop_count <= 3) {
        DEBUG_PRINT("setpoint: x=%.2f z=%.2f preview_z=%.2f\n",
                    (double)mpc_setpoint_task(0), (double)mpc_setpoint_task(2),
                    (double)preview_z);
      }

      // Skip event triggers for now to simplify debugging
      // eventTrigger payloads and calls commented out

      // Copy the setpoint calculated by the task loop to the global mpc_setpoint
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      mpc_setpoint = mpc_setpoint_task;
      init_vel_z = problem.x(8, 0);
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
    if (flight_phase == FLIGHT_PHASE_LANDING) {
      if (landing_start_tick == 0) {
        landing_start_tick = tick;
        landing_hold_x = state->position.x;
        landing_hold_y = state->position.y;
        landing_hold_yaw = state->attitude.yaw;
        landing_start_z = state->position.z;
        landing_reference_z = landing_start_z;
        DEBUG_PRINT("LANDING: start z=%.2f, hold=(%.2f,%.2f)\n",
                    (double)landing_start_z,
                    (double)landing_hold_x,
                    (double)landing_hold_y);
      }

      const float landing_elapsed_s =
          0.001f * static_cast<float>(T2M(tick - landing_start_tick));
      landing_reference_z =
          fmaxf(TINYMPC_LANDING_TARGET_Z_M,
                landing_start_z -
                    TINYMPC_LANDING_DESCENT_MPS * landing_elapsed_s);

      memset(&mpc_setpoint_pid, 0, sizeof(mpc_setpoint_pid));
      mpc_setpoint_pid.mode.yaw = modeAbs;
      mpc_setpoint_pid.mode.x = modeAbs;
      mpc_setpoint_pid.mode.y = modeAbs;
      mpc_setpoint_pid.mode.z = modeAbs;
      mpc_setpoint_pid.position.x = landing_hold_x;
      mpc_setpoint_pid.position.y = landing_hold_y;
      mpc_setpoint_pid.position.z = landing_reference_z;
      mpc_setpoint_pid.attitude.yaw = landing_hold_yaw;
      controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);

      const bool landing_reference_complete =
          landing_reference_z <= TINYMPC_LANDING_TARGET_Z_M + 1e-4f;
      const bool near_floor =
          state->position.z <= TINYMPC_LANDING_TOUCHDOWN_Z_M;
      const bool vertically_settled =
          fabsf(state->velocity.z) <= TINYMPC_LANDING_TOUCHDOWN_VZ_MPS;
      if (landing_reference_complete && near_floor && vertically_settled) {
        flight_phase = FLIGHT_PHASE_COMPLETE;
        stop_motors(control);
        DEBUG_PRINT("LANDING COMPLETE: z=%.2f, vz=%.2f; motors off\n",
                    (double)state->position.z,
                    (double)state->velocity.z);
      }
    } else if (flight_phase == FLIGHT_PHASE_COMPLETE) {
      stop_motors(control);
    } else {
      memset(&mpc_setpoint_pid, 0, sizeof(mpc_setpoint_pid));
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
LOG_ADD(LOG_UINT8, mode, &benchmark_mode)
LOG_ADD(LOG_UINT8, maneuver, &benchmark_maneuver)
LOG_ADD(LOG_UINT8, phase, &flight_phase)
LOG_ADD(LOG_FLOAT, land_zref, &landing_reference_z)
LOG_ADD(LOG_UINT32, step, &benchmark_step)
LOG_ADD(LOG_FLOAT, limo_h, &limo_h)
LOG_ADD(LOG_FLOAT, limo_raw, &limo_raw)
LOG_ADD(LOG_FLOAT, limo_grad, &limo_grad_norm)
LOG_ADD(LOG_FLOAT, limo_margin, &limo_margin_eff)
LOG_ADD(LOG_FLOAT, limo_thresh, &limo_threshold)
LOG_ADD(LOG_UINT32, limo_eval_us, &limo_eval_us)
LOG_ADD(LOG_UINT8, limo_active, &limo_active)
LOG_ADD(LOG_UINT8, limo_active_count, &limo_active_count)
LOG_ADD(LOG_FLOAT, limo_w, &limo_authority_w)
LOG_ADD(LOG_FLOAT, limo_w_req, &limo_authority_w_requested)
LOG_ADD(LOG_FLOAT, limo_qz, &limo_qz)
LOG_ADD(LOG_UINT8, limo_w_idx, &limo_w_index)
LOG_ADD(LOG_UINT8, limo_qz_idx, &limo_qz_index)
LOG_ADD(LOG_UINT8, limo_cache, &limo_cache_ok)
LOG_ADD(LOG_FLOAT, no_or_score, &limo_no_oracle_score)
LOG_ADD(LOG_FLOAT, no_or_raw, &limo_no_oracle_score_raw)
LOG_ADD(LOG_UINT8, no_or_act, &limo_no_oracle_active)
LOG_ADD(LOG_UINT8, post_active, &posthoc_active)
LOG_ADD(LOG_UINT8, post_failed, &posthoc_failed)
LOG_ADD(LOG_FLOAT, post_du, &posthoc_du_norm)

LOG_ADD(LOG_FLOAT, posX, &tracking_pos_x)
LOG_ADD(LOG_FLOAT, posY, &tracking_pos_y)
LOG_ADD(LOG_FLOAT, posZ, &tracking_pos_z)
LOG_ADD(LOG_FLOAT, velX, &tracking_vel_x)
LOG_ADD(LOG_FLOAT, velY, &tracking_vel_y)
LOG_ADD(LOG_FLOAT, velZ, &tracking_vel_z)
LOG_ADD(LOG_FLOAT, cmdZ, &tracking_cmd_z)
LOG_ADD(LOG_FLOAT, previewZ, &tracking_preview_z)
LOG_ADD(LOG_UINT32, mpc_us, &mpc_time_us)
LOG_ADD(LOG_UINT32, total_us, &controller_total_us)

LOG_GROUP_STOP(tinympc)

#define PARAM_TOC_TYPE(TYPE) \
  static_cast<uint8_t>(((TYPE) <= 0xFF) ? ((TYPE) & 0xFF) : (((TYPE) | PARAM_EXTENDED) & 0xFF))
#define PARAM_TOC_EXT_TYPE(TYPE) \
  static_cast<uint8_t>((((TYPE) & 0xFF00) >> 8))
#define PARAM_GROUP_ENTRY(TYPE, NAME) \
  { .type = static_cast<uint8_t>(TYPE), .extended_type = 0, .name = const_cast<char *>(#NAME), \
    .address = NULL, .callback = NULL, .getter = NULL, },
#define PARAM_VALUE_ENTRY(TYPE, NAME, ADDRESS) \
  { .type = PARAM_TOC_TYPE(TYPE), .extended_type = PARAM_TOC_EXT_TYPE(TYPE), \
    .name = const_cast<char *>(#NAME), .address = static_cast<void *>(ADDRESS), \
    .callback = NULL, .getter = NULL, },

static struct param_s __params_limo[] __attribute__((section(".param.limo"), used)) = {
  PARAM_GROUP_ENTRY(PARAM_GROUP | PARAM_START, limo)
  PARAM_VALUE_ENTRY(PARAM_UINT8, maxIter, &benchmark_max_iter)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, duration, &traj_duration)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, radius, &traj_radius)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, omega, &traj_omega)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, speed, &traj_speed)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, distance, &traj_dist)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, height, &traj_height)
  PARAM_VALUE_ENTRY(PARAM_UINT8, activeH, &limo_active_horizon)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, margin, &limo_margin)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, mScale, &limo_margin_scale)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, hDeadband, &limo_h_deadband)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, actSlack, &limo_act_slack)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, azCoeff, &limo_az_coeff)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, gComp, &limo_gravity_comp)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, failRoll, &limo_fail_roll_deg)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, failPitch, &limo_fail_pitch_deg)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, guard, &limo_structural_guard_relax)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, skipZ, &limo_skip_z)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, skipVz, &limo_skip_vz)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, distBudget, &limo_dist_budget)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, distCoeff, &limo_dist_margin_coeff)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, highCut, &limo_dist_high_cut)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, highBias, &limo_dist_high_bias)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, floorBand, &limo_dist_floor_band)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, floorGain, &limo_dist_floor_gain)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, descBand, &limo_dist_desc_band)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, descGain, &limo_dist_desc_gain)
  PARAM_VALUE_ENTRY(PARAM_FLOAT, hRate, &limo_horizon_margin_rate)
  PARAM_GROUP_ENTRY(PARAM_GROUP | PARAM_STOP, stop_limo)
};

#undef PARAM_VALUE_ENTRY
#undef PARAM_GROUP_ENTRY
#undef PARAM_TOC_EXT_TYPE
#undef PARAM_TOC_TYPE

#ifdef __cplusplus
} /* extern "C" */
#endif
