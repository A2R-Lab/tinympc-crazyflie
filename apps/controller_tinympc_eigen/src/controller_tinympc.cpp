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
#include <cmath>
#include "tinympc_generated_params.h"
#define NHORIZON TINYMPC_GENERATED_HORIZON_KNOTS
using namespace Eigen;

#ifdef __cplusplus
extern "C" {
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
#include "static_mem.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "supervisor.h"
#include "sequential_obstacle_link.h"
#include "tinyracer_debug.h"
#include "tinyracer_racing.h"
#include "tinympc_progress_path.h"
#include "tinympc_waypoint_nav.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (10 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC ADMM"
#define TINYMPC_TASK_PRI              1

// Per-solve frame at the current position and yaw. Its z-axis stays aligned
// with world z, so gravity and the hover linearization are unchanged.
struct MpcLocalFrame {
  float origin_x;
  float origin_y;
  float origin_z;
  float yaw_world;
  float cos_yaw;
  float sin_yaw;
};

static Eigen::Vector3f worldVectorToLocal(
    const MpcLocalFrame& frame, const Eigen::Vector3f& vector_world) {
  return Eigen::Vector3f(
      frame.cos_yaw * vector_world.x() + frame.sin_yaw * vector_world.y(),
      -frame.sin_yaw * vector_world.x() + frame.cos_yaw * vector_world.y(),
      vector_world.z());
}

static Eigen::Vector3f localVectorToWorld(
    const MpcLocalFrame& frame, const Eigen::Vector3f& vector_local) {
  return Eigen::Vector3f(
      frame.cos_yaw * vector_local.x() - frame.sin_yaw * vector_local.y(),
      frame.sin_yaw * vector_local.x() + frame.cos_yaw * vector_local.y(),
      vector_local.z());
}

// Remove the local frame yaw from a body-to-world quaternion.
static struct vec worldQuaternionToLocalRodrigues(
    const MpcLocalFrame& frame, struct quat quaternion_world_body) {
  const float half_yaw = 0.5f * frame.yaw_world;
  const float c = cosf(half_yaw);
  const float s = sinf(half_yaw);
  struct quat quaternion_local_body = mkquat(
      c * quaternion_world_body.x + s * quaternion_world_body.y,
      c * quaternion_world_body.y - s * quaternion_world_body.x,
      c * quaternion_world_body.z - s * quaternion_world_body.w,
      c * quaternion_world_body.w + s * quaternion_world_body.z);
  const float denominator = fabsf(quaternion_local_body.w) > 1e-6f
      ? quaternion_local_body.w : copysignf(1e-6f, quaternion_local_body.w);
  return mkvec(
      quaternion_local_body.x / denominator,
      quaternion_local_body.y / denominator,
      quaternion_local_body.z / denominator);
}

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Model and solve timing come from the generated upstream specialization.
#define DT TINYMPC_GENERATED_MODEL_DT_S
#define MPC_RATE TINYMPC_GENERATED_SOLVE_RATE_HZ
#define LQR_RATE RATE_500_HZ  // control frequency

// A direct command may be held briefly while the next asynchronous solve
// finishes. Past this deadline, command model-derived hover rather than flying
// indefinitely on a stale open-loop command.
#define TINYMPC_DIRECT_COMMAND_MAX_AGE_MS (3U * (1000U / MPC_RATE))

static_assert(NSTATES == TINYMPC_GENERATED_STATE_DIM, "generated state dimension mismatch");
static_assert(NINPUTS == TINYMPC_GENERATED_INPUT_DIM, "generated input dimension mismatch");

/* Include trajectory to track. Acrobatic headers carry motor feedforward. */
#if defined(TINYMPC_TRAJECTORY_ROLL_FLIP_360)
#include "trajectories/50hz/traj_roll_flip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_FRONT_FLIP_360)
#include "trajectories/50hz/traj_front_flip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_BACKFLIP_360)
#include "trajectories/50hz/traj_backflip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_BARREL_ROLL_FORWARD_360)
#include "trajectories/50hz/traj_barrel_roll_forward_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_STRAIGHT)
#include "trajectories/50hz/traj_straight_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_STRAIGHT_LONG)
#include "trajectories/50hz/traj_straight_long_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR)
#include "trajectories/50hz/traj_canonical_corridor_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CIRCLE)
#include "trajectories/50hz/traj_canonical_circle_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_FIGURE8)
#include "trajectories/50hz/traj_figure8_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_OVAL)
#include "trajectories/50hz/traj_oval_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
#include "trajectories/50hz/traj_canonical_figure8_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CHICANE)
#include "trajectories/50hz/traj_chicane_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_HAIRPIN_180)
#include "trajectories/50hz/traj_hairpin_180_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CHICANE)
#include "trajectories/50hz/traj_canonical_chicane_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_HAIRPIN)
#include "trajectories/50hz/traj_canonical_hairpin_50hz.h"
#else
#include "trajectories/50hz/traj_circle_50hz.h"
#endif

/* Canonical courses keep the compact level-flight controller for the track
 * and carry only one stored-LTV Tier-3 primitive. The trigger index is in the
 * level-course reference and vision is latched out for the primitive. */
#if defined(TINYMPC_COURSE_ACRO_BARREL_ROLL_FORWARD_360)
#include "trajectories/50hz/course_acro_barrel_roll_forward_360_50hz.h"
#define TINYMPC_COURSE_ACRO_AVAILABLE 1
#define TINYMPC_COURSE_ACRO_TRIGGER_INDEX 680U
#elif defined(TINYMPC_COURSE_ACRO_BACKFLIP_360)
#include "trajectories/50hz/course_acro_backflip_360_50hz.h"
#define TINYMPC_COURSE_ACRO_AVAILABLE 1
#define TINYMPC_COURSE_ACRO_TRIGGER_INDEX (TRAJECTORY_SAMPLE_COUNT - 3U)
#else
#define TINYMPC_COURSE_ACRO_AVAILABLE 0
#endif

#if defined(TINYMPC_USE_STORED_LTV)
#if defined(TINYMPC_COURSE_ACRO_BARREL_ROLL_FORWARD_360)
#include "trajectories/50hz/ltv/stored_ltv_barrel_roll_forward_360_50hz.h"
#elif defined(TINYMPC_COURSE_ACRO_BACKFLIP_360)
#include "trajectories/50hz/ltv/stored_ltv_backflip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_ROLL_FLIP_360)
#include "trajectories/50hz/ltv/stored_ltv_roll_flip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_FRONT_FLIP_360)
#include "trajectories/50hz/ltv/stored_ltv_front_flip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_BACKFLIP_360)
#include "trajectories/50hz/ltv/stored_ltv_backflip_360_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_BARREL_ROLL_FORWARD_360)
#include "trajectories/50hz/ltv/stored_ltv_barrel_roll_forward_360_50hz.h"
#else
#error "Stored LTV requires a supported acrobatic trajectory"
#endif
static_assert(TINYMPC_STORED_LTV_DT_S == TINYMPC_GENERATED_MODEL_DT_S,
              "stored LTV and firmware model periods differ");
static_assert(TINYMPC_STORED_LTV_RHO == TINYMPC_GENERATED_ADMM_RHO,
              "stored LTV and firmware ADMM rho differ");
static_assert(TINYMPC_STORED_LTV_STATE_DIM == NSTATES + NINPUTS,
              "stored LTV must add one actuator state per motor");
#endif
#define TINYMPC_ACRO_AVAILABLE \
  (TRAJECTORY_HAS_MOTOR_FEEDFORWARD || TINYMPC_COURSE_ACRO_AVAILABLE)
#ifndef TRAJECTORY_HAS_MOTOR_FEEDFORWARD
#define TRAJECTORY_HAS_MOTOR_FEEDFORWARD 0
#endif
#ifndef TRAJECTORY_TURN_DIRECTION
/* +1 is a left/CCW curve, -1 is right/CW, and 0 is not prescribed. */
#define TRAJECTORY_TURN_DIRECTION 0
#endif
#ifndef TRAJECTORY_OBSTACLE_APPROACH_WINDOW
#define TRAJECTORY_OBSTACLE_APPROACH_WINDOW 0
#define TRAJECTORY_OBSTACLE_APPROACH_START_INDEX 0U
#define TRAJECTORY_OBSTACLE_INDEX 0U
#define TRAJECTORY_OBSTACLE_APPROACH_END_INDEX UINT32_MAX
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
#include "tinympc_level_actuator_lti.h"
static_assert(TINYMPC_LEVEL_ACTUATOR_STATE_DIM == NSTATES + NINPUTS,
              "level actuator model must add one state per motor");
static_assert(TINYMPC_LEVEL_ACTUATOR_DT_S == TINYMPC_GENERATED_MODEL_DT_S,
              "level actuator model and firmware periods differ");
static_assert(TINYMPC_LEVEL_ACTUATOR_RHO == TINYMPC_GENERATED_ADMM_RHO,
              "level actuator model and firmware ADMM rho differ");
#endif
// #include "traj_circle_500hz.h"  // Large circle (1m radius)
// #include "traj_circle_small.h"  // Small circle (0.5m radius)
// #include "traj_perching.h"
//#include "traj_straight_line.h"  // Straight line (0,0,0.5) to (1,0,0.5)

// Precomputed data and cache, in params_*.h
static MatrixNf A;
static MatrixNMf B;
static VectorNf f;
static MatrixMNf Kinf;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static VectorNf APf;
static VectorMf BPf;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1];
static VectorMf d[NHORIZON-1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON];

static VectorNf q[NHORIZON-1];
static VectorMf r[NHORIZON-1];
static VectorMf r_tilde[NHORIZON-1];

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;

// For obstacle avoidance
static VectorNf ZX[NHORIZON];
static VectorNf ZX_new[NHORIZON];
static VectorNf YX[NHORIZON];

static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg;
static VectorMf ug;

static const float perception_halfspace_penalty = 0.0f;
static const float perception_pass_distance_m = 0.75f;
static const float perception_bypass_speed_mps = 0.25f;
static const float perception_sector_spacing_rad = 0.4654211f;
static const float perception_drone_radius_m = 0.10f;
static const float perception_uncertainty_gain_m = 0.10f;
static const float perception_sector_bearing_rad[TINYRACER_CLEARANCE_SECTORS] = {
  0.6981317f, 0.2327106f, -0.2327106f, -0.6981317f
};
static const TinyRacerRaceConfig race_config = {
  0.30f, -1.0986123f, 0.10f, 0.05f, 0.35f, 0.35f, 0.25f, 250,
  0.28f, 0.70f, 2, 0.18f, 2, 0.25f, 2
};
static const TinyRacerNavigationConfig navigation_config = {
  250, 0.50f, 2.09439510239f, 0.30f
};
/* Tangent-heading curves revisit scenery and already consume lateral
 * acceleration. Keep their smaller dodge envelope and longer re-arm distance.
 * On a known curve, use the outside passing lane instead of allowing small
 * frame/timing changes in DroNet steering to flip the chosen side. */
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.50f
#define TINYRACER_DODGE_REARM_DISTANCE_M 100.0f
#define TINYRACER_DODGE_PASS_SIDE (-1)
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.20f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.20f
#define TINYRACER_DODGE_REJOIN_RATE_MPS 0.15f
#define TINYRACER_DODGE_REJOIN_SPEED_MPS 0.10f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_HAIRPIN)
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.35f
#define TINYRACER_DODGE_REARM_DISTANCE_M 100.0f
#define TINYRACER_DODGE_PASS_SIDE (-1)
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.25f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.30f
#define TINYRACER_DODGE_REJOIN_RATE_MPS 0.08f
#define TINYRACER_DODGE_REJOIN_SPEED_MPS 0.15f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CIRCLE)
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.50f
#define TINYRACER_DODGE_REARM_DISTANCE_M 100.0f
#define TINYRACER_DODGE_PASS_SIDE (-TRAJECTORY_TURN_DIRECTION)
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.35f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.30f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CHICANE)
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.46f
#define TINYRACER_DODGE_REARM_DISTANCE_M 100.0f
#define TINYRACER_DODGE_PASS_SIDE (-TRAJECTORY_TURN_DIRECTION)
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.35f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.30f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR)
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.70f
#define TINYRACER_DODGE_REARM_DISTANCE_M 100.0f
#define TINYRACER_DODGE_PASS_SIDE 0
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.50f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.45f
#elif TRAJECTORY_TANGENT_HEADING
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.46f
#define TINYRACER_DODGE_REARM_DISTANCE_M 1.00f
#define TINYRACER_DODGE_PASS_SIDE (-TRAJECTORY_TURN_DIRECTION)
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.35f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.30f
#else
#define TINYRACER_DODGE_LATERAL_OFFSET_M 0.70f
#define TINYRACER_DODGE_REARM_DISTANCE_M 0.40f
#define TINYRACER_DODGE_PASS_SIDE 0
#define TINYRACER_DODGE_SIDESTEP_RATE_MPS 0.50f
#define TINYRACER_DODGE_PASS_SPEED_MPS 0.45f
#endif
#ifndef TINYRACER_DODGE_REJOIN_RATE_MPS
#define TINYRACER_DODGE_REJOIN_RATE_MPS 0.30f
#endif
#ifndef TINYRACER_DODGE_REJOIN_SPEED_MPS
#define TINYRACER_DODGE_REJOIN_SPEED_MPS 0.35f
#endif
static const TinyRacerDodgeConfig dodge_config = {
  0.25f, 0.18f, TINYRACER_DODGE_LATERAL_OFFSET_M,
  TINYRACER_DODGE_SIDESTEP_RATE_MPS, TINYRACER_DODGE_REJOIN_RATE_MPS,
  0.12f, TINYRACER_DODGE_PASS_SPEED_MPS,
  TINYRACER_DODGE_REJOIN_SPEED_MPS, perception_pass_distance_m,
  TINYRACER_DODGE_REARM_DISTANCE_M, false,
  TINYRACER_DODGE_PASS_SIDE, 2, 4
};
typedef struct {
  Eigen::Vector3f normal_world;
  float boundary_world;
} PerceptionPlane;
static PerceptionPlane perception_stop_plane;
static bool perception_obstacle_active = false;
static bool perception_halfspace_active = false;
static bool perception_binary_constraint = false;
static Eigen::Vector3f perception_obstacle_center_world = Eigen::Vector3f::Zero();
static float perception_obstacle_radius_m = 0.0f;
static bool perception_recovery_active = false;
static float perception_recovery_distance_m = 0.0f;
static Eigen::Vector3f perception_recovery_last_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f perception_motion_target_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f avoidance_forward_world = Eigen::Vector3f::UnitX();
static Eigen::Vector3f avoidance_left_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f avoidance_start_world = Eigen::Vector3f::Zero();
static TinyRacerRaceState race_state;
static TinyRacerRaceIntent race_intent;
static float perception_clearance_history[TINYRACER_CLEARANCE_SECTORS][5];
static uint8_t perception_filter_index = 0;
static uint32_t perception_filter_sample = UINT32_MAX;
static uint32_t gate_filter_sample = UINT32_MAX;
static float gate_lateral_offset_m = 0.0f;
static float gate_vertical_offset_m = 0.0f;
static TinyRacerNavigationState navigation_state;
static TinyRacerNavigationIntent navigation_intent;
static TinyRacerDodgeState dodge_state;
static TinyRacerDodgeIntent dodge_intent;
static uint16_t navigation_warmup_steps = 0;
static bool gate_priority_latched = false;
static uint8_t gate_priority_clear_samples = 0;
static uint16_t gate_priority_hold_steps = 0;
static bool gate_priority_rearm_ready = true;
static TinyRacerGateProgress course_gate_progress;
static bool course_gate_approach_active = false;
static uint8_t course_acro_clear_steps = 0;
static uint8_t course_acro_stable_steps = 0;
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
static bool figure8_midcourse_dual_reset_complete = false;
#endif
#if defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR)
#define TINYMPC_GATE_PRIORITY_CLEAR_STEPS 20u
#else
#define TINYMPC_GATE_PRIORITY_CLEAR_STEPS 75u
#endif
#define TINYMPC_GATE_PRIORITY_MAX_STEPS 125u

#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
static void resetLevelActuatorDuals(void);
#endif

#if defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR) || \
    defined(TINYMPC_TRAJECTORY_CANONICAL_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8) || \
    defined(TINYMPC_TRAJECTORY_CANONICAL_CHICANE) || \
    defined(TINYMPC_TRAJECTORY_CANONICAL_HAIRPIN)
#define TINYMPC_SINGLE_OBSTACLE_COURSE 1
#else
#define TINYMPC_SINGLE_OBSTACLE_COURSE 0
#endif

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

static uint32_t step = 0;
static bool en_traj = true;   // Track the generated stored trajectory.
static const uint32_t traj_length = T_ARRAY_SIZE(trajectory_reference_data);

#ifndef TINYMPC_PROGRESS_SAMPLE_LIMIT
#define TINYMPC_PROGRESS_SAMPLE_LIMIT 0
#endif
#ifndef TINYMPC_PROGRESS_SPEED_MPS
#define TINYMPC_PROGRESS_SPEED_MPS 0.0f
#endif
#if defined(TINYMPC_REFERENCE_MODE_PROGRESS)
static_assert(TINYMPC_PROGRESS_SAMPLE_LIMIT == 0 ||
              TINYMPC_PROGRESS_SAMPLE_LIMIT >= 2,
              "progress sample limit must be zero (full route) or at least 2");
static_assert(TINYMPC_PROGRESS_SAMPLE_LIMIT == 0 ||
              TINYMPC_PROGRESS_SAMPLE_LIMIT <= TRAJECTORY_SAMPLE_COUNT,
              "progress sample limit exceeds compiled trajectory");
static const uint32_t progress_sample_count =
    TINYMPC_PROGRESS_SAMPLE_LIMIT == 0
    ? TRAJECTORY_SAMPLE_COUNT : TINYMPC_PROGRESS_SAMPLE_LIMIT;
static_assert(TINYMPC_PROGRESS_SPEED_MPS >= 0.0f,
              "progress speed must be zero (curvature schedule) or positive");
static constexpr float progress_minimum_speed_mps =
    TINYMPC_PROGRESS_SPEED_MPS > 0.0f ? TINYMPC_PROGRESS_SPEED_MPS : 0.05f;
static constexpr float progress_maximum_speed_mps =
    TINYMPC_PROGRESS_SPEED_MPS > 0.0f ? TINYMPC_PROGRESS_SPEED_MPS : 0.15f;
#endif

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(
    TRAJECTORY_REFERENCE_DIM == (TRAJECTORY_HAS_MOTOR_FEEDFORWARD ? 17 : 13),
    "trajectory reference dimension mismatch");
#if TINYMPC_COURSE_ACRO_AVAILABLE
static_assert(TINYMPC_COURSE_ACRO_SAMPLE_RATE_HZ == MPC_RATE,
              "course acrobatic primitive and MPC rates must match");
static_assert(TINYMPC_COURSE_ACRO_REFERENCE_DIM == 17,
              "course acrobatic reference dimension mismatch");
#endif

static struct quat attitude;
static MpcLocalFrame active_local_frame;
static Eigen::Vector3f trajectory_origin_world = Eigen::Vector3f::Zero();
static float trajectory_cos_yaw = 1.0f;
static float trajectory_sin_yaw = 0.0f;
static struct quat trajectory_yaw_rotation = qeye();
static uint16_t trajectory_handoff_hold_steps = 0;
static float reference_yaw_unwrapped_rad[NHORIZON];
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
static float reference_yaw_phase_rad = 0.0f;
#if defined(TINYMPC_REFERENCE_MODE_PROGRESS)
static TinyMpcProgressPath progress_path;
static bool progress_completion_reported = false;
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
static uint32_t progress_diag_cycle = 0u;
static uint32_t progress_diag_last_max_log_cycle = 0u;
static float progress_diag_max_curvature_per_m = 0.0f;
static float progress_diag_max_speed_mps = 0.0f;
static float progress_diag_max_yaw_rate_rad_s = 0.0f;
static float progress_diag_max_roll_rad = 0.0f;
static float progress_diag_max_pitch_rad = 0.0f;
static float progress_diag_max_tilt_rad = 0.0f;
static float progress_diag_max_thrust_scale = 0.0f;
static uint8_t progress_diag_threshold_mask = 0u;
static bool progress_diag_uref_clamped = false;
static bool progress_diag_pending_max_log = false;
static bool progress_diag_has_max_log = false;

static void resetUncappedProgressDiagnostics(void) {
  progress_diag_cycle = 0u;
  progress_diag_last_max_log_cycle = 0u;
  progress_diag_max_curvature_per_m = 0.0f;
  progress_diag_max_speed_mps = 0.0f;
  progress_diag_max_yaw_rate_rad_s = 0.0f;
  progress_diag_max_roll_rad = 0.0f;
  progress_diag_max_pitch_rad = 0.0f;
  progress_diag_max_tilt_rad = 0.0f;
  progress_diag_max_thrust_scale = 0.0f;
  progress_diag_threshold_mask = 0u;
  progress_diag_uref_clamped = false;
  progress_diag_pending_max_log = false;
  progress_diag_has_max_log = false;
}
#endif
#endif
#define TINYMPC_MAX_ROUTE_WAYPOINTS 96u
static TinyMpcWaypoint route_waypoints[TINYMPC_MAX_ROUTE_WAYPOINTS];
static uint32_t route_source_knots[TINYMPC_MAX_ROUTE_WAYPOINTS];
static TinyMpcWaypointNavigator waypoint_navigator;
static bool waypoint_yaw_settle_active = false;
#endif
static VectorMf acro_motor_baseline_n;
#if TINYMPC_ACRO_AVAILABLE
static float acro_altitude_estimate_m = 0.0f;
static bool acro_altitude_initialized = false;
static uint16_t acro_reference_index = 0;
static float motor_rotor_state_estimate[NINPUTS] = {0.0f};
static float planner_motor_rotor_state_estimate[NINPUTS] = {0.0f};
static VectorMf acro_motor_rotor_state_estimate;
#endif
#if TINYMPC_COURSE_ACRO_AVAILABLE
typedef enum {
  TINYMPC_MODE_WAYPOINT_TRACKING = 0,
  TINYMPC_MODE_ACROBATIC_MANEUVER = 1,
} TinyMpcReferenceMode;
static TinyMpcReferenceMode reference_mode = TINYMPC_MODE_WAYPOINT_TRACKING;
static bool course_acro_completed = false;
static uint16_t course_acro_step = 0;
static uint8_t course_acro_terminal_hold_steps = 0;
static Eigen::Vector3f course_acro_origin_world = Eigen::Vector3f::Zero();
static float course_acro_cos_yaw = 1.0f;
static float course_acro_sin_yaw = 0.0f;
static struct quat course_acro_yaw_rotation = qeye();
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
static float level_motor_rotor_state_estimate[NINPUTS] = {0.0f};
static float planner_level_motor_rotor_state_estimate[NINPUTS] = {0.0f};
static VectorMf level_motor_rotor_state_snapshot;
static float level_warm_start_yaw_rad = 0.0f;
static bool level_warm_start_yaw_initialized = false;
#endif

static bool mpc_has_run = false;
static uint32_t last_controller_tick = 0;
static uint32_t plan_start_tick = 0;
static bool motors_were_allowed = false;
static float active_motor_commands[NINPUTS];
static SemaphoreHandle_t runTaskSemaphore = NULL;
static SemaphoreHandle_t dataMutex = NULL;
static StaticSemaphore_t dataMutexBuffer;
static setpoint_t planner_setpoint;
static sensorData_t planner_sensors;
static state_t planner_state;
static uint32_t planner_tick = 0;
static bool planner_reset_requested = false;
static void tinympcControllerTask(void *parameters);
STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

#ifndef TINYMPC_ACRO_MAX_MOTOR_THRUST_N
#define TINYMPC_ACRO_MAX_MOTOR_THRUST_N 0.20f
#endif

static void loadGeneratedSolverData(void) {
  for (int row = 0; row < NSTATES; ++row) {
    f(row) = tinympc_generated_f[row];
    APf(row) = tinympc_generated_APf[row];
    for (int column = 0; column < NSTATES; ++column) {
      const int index = row * NSTATES + column;
      A(row, column) = tinympc_generated_A[index];
      Pinf(row, column) = tinympc_generated_Pinf[index];
      AmBKt(row, column) = tinympc_generated_AmBKt[index];
    }
    for (int column = 0; column < NINPUTS; ++column) {
      const int index = row * NINPUTS + column;
      B(row, column) = tinympc_generated_B[index];
      coeff_d2p(row, column) = tinympc_generated_coeff_d2p[index];
    }
  }

  Q.setZero();
  R.setZero();
  for (int input = 0; input < NINPUTS; ++input) {
    BPf(input) = tinympc_generated_BPf[input];
    ug(input) = tinympc_generated_hover_reference[input];
    lcu(input) = tinympc_generated_input_lower[input * (NHORIZON - 1)];
    ucu(input) = tinympc_generated_input_upper[input * (NHORIZON - 1)];
    R(input, input) = tinympc_generated_R_diagonal[input];
    for (int state = 0; state < NSTATES; ++state) {
      Kinf(input, state) = tinympc_generated_Kinf[input * NSTATES + state];
    }
    for (int column = 0; column < NINPUTS; ++column) {
      Quu_inv(input, column) =
          tinympc_generated_Quu_inv[input * NINPUTS + column];
    }
  }
  for (int state = 0; state < NSTATES; ++state) {
    Q(state, state) = tinympc_generated_Q_diagonal[state];
  }
}

static float unwrapNear(float angle, float reference) {
  return reference + remainderf(angle - reference, 6.28318530717958647692f);
}

#if TINYMPC_ACRO_AVAILABLE
static Eigen::Matrix3f quaternionRotation(struct quat quaternion) {
  quaternion = qnormalize(quaternion);
  const float w = quaternion.w;
  const float x = quaternion.x;
  const float y = quaternion.y;
  const float z = quaternion.z;
  Eigen::Matrix3f rotation;
  rotation <<
      1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - z * w), 2.0f * (x * z + y * w),
      2.0f * (x * y + z * w), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - x * w),
      2.0f * (x * z - y * w), 2.0f * (y * z + x * w), 1.0f - 2.0f * (x * x + y * y);
  return rotation;
}

static bool acroControlActive() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return true;
#elif TINYMPC_COURSE_ACRO_AVAILABLE
  return reference_mode == TINYMPC_MODE_ACROBATIC_MANEUVER;
#else
  return false;
#endif
}

static uint32_t acroReferenceLength() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return traj_length;
#else
  return TINYMPC_COURSE_ACRO_SAMPLE_COUNT;
#endif
}

static float acroReferenceDt() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return TRAJECTORY_SAMPLE_DT_S;
#else
  return TINYMPC_COURSE_ACRO_SAMPLE_DT_S;
#endif
}

static float acroReferenceValue(uint32_t index, int field) {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return trajectory_reference_data[index][field];
#else
  return tinympc_course_acro_reference_data[index][field];
#endif
}

static void sampleAcroReference(float sample_position, float reference[17]) {
  const uint32_t length = acroReferenceLength();
  sample_position = fminf(fmaxf(sample_position, 0.0f), (float)(length - 1));
  const uint32_t lower = (uint32_t)floorf(sample_position);
  const uint32_t upper = lower + 1 < length ? lower + 1 : lower;
  const float alpha = sample_position - (float)lower;
  for (int field = 0; field < 17; ++field) {
    reference[field] = (1.0f - alpha) * acroReferenceValue(lower, field)
        + alpha * acroReferenceValue(upper, field);
  }
  float quaternion_norm_sq = 0.0f;
  for (int field = 3; field <= 6; ++field) {
    quaternion_norm_sq += reference[field] * reference[field];
  }
  const float inverse_norm = 1.0f / sqrtf(quaternion_norm_sq);
  for (int field = 3; field <= 6; ++field) {
    reference[field] *= inverse_norm;
  }
}

static Eigen::Vector3f acroOriginWorld() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return trajectory_origin_world;
#else
  return course_acro_origin_world;
#endif
}

static float acroCosYaw() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return trajectory_cos_yaw;
#else
  return course_acro_cos_yaw;
#endif
}

static float acroSinYaw() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return trajectory_sin_yaw;
#else
  return course_acro_sin_yaw;
#endif
}

static struct quat acroYawRotation() {
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  return trajectory_yaw_rotation;
#else
  return course_acro_yaw_rotation;
#endif
}
#endif

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  attitude = qnormalize(mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
      state->attitudeQuaternion.w));
  active_local_frame.origin_x = state->position.x;
  active_local_frame.origin_y = state->position.y;
  active_local_frame.origin_z = state->position.z;
  active_local_frame.yaw_world = quat2rpy(attitude).z;
  active_local_frame.cos_yaw = cosf(active_local_frame.yaw_world);
  active_local_frame.sin_yaw = sinf(active_local_frame.yaw_world);

#if TINYMPC_ACRO_AVAILABLE
  if (acroControlActive()) {
  // Express the measured rigid-body state in the stored maneuver's local
  // tracking-error chart. Recentring the Rodrigues chart on the reference at
  // every solve avoids a singularity when an absolute flip crosses 180 deg.
  const float current_time_s =
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
      trajectory_handoff_hold_steps > 0
          ? 0.0f : (float)step / (float)MPC_RATE;
#else
      (float)course_acro_step * TINYMPC_COURSE_ACRO_SAMPLE_DT_S;
#endif
  acro_reference_index = (uint16_t)T_MIN(
      (uint32_t)lroundf(current_time_s / acroReferenceDt()),
      acroReferenceLength() - 1);
  float reference[17];
  sampleAcroReference(current_time_s / acroReferenceDt(), reference);
  const struct quat reference_attitude = qnormalize(qqmul(
      acroYawRotation(),
      mkquat(reference[4], reference[5], reference[6], reference[3])));
  const float acro_cos_yaw = acroCosYaw();
  const float acro_sin_yaw = acroSinYaw();
  const Eigen::Vector3f reference_position = acroOriginWorld() + Eigen::Vector3f(
      acro_cos_yaw * reference[0] - acro_sin_yaw * reference[1],
      acro_sin_yaw * reference[0] + acro_cos_yaw * reference[1],
      reference[2]);
  const Eigen::Vector3f reference_velocity(
      acro_cos_yaw * reference[7] - acro_sin_yaw * reference[8],
      acro_sin_yaw * reference[7] + acro_cos_yaw * reference[8],
      reference[9]);
  struct quat attitude_error = qnormalize(qqmul(
      mkquat(-reference_attitude.x, -reference_attitude.y,
             -reference_attitude.z, reference_attitude.w),
      attitude));
  if (attitude_error.w < 0.0f) {
    attitude_error = mkquat(
        -attitude_error.x, -attitude_error.y,
        -attitude_error.z, -attitude_error.w);
  }
  const float denominator = T_MAX(attitude_error.w, 1.0e-6f);
  const Eigen::Matrix3f reference_rotation = quaternionRotation(reference_attitude);
  const Eigen::Matrix3f actual_rotation = quaternionRotation(attitude);
  const Eigen::Vector3f actual_velocity(
      state->velocity.x, state->velocity.y, state->velocity.z);
  const Eigen::Vector3f actual_omega(
      radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));
  const Eigen::Vector3f reference_omega(reference[10], reference[11], reference[12]);
  const Eigen::Vector3f reference_omega_in_actual_body =
      actual_rotation.transpose() * reference_rotation * reference_omega;
  // Flow-deck range is a slant distance and becomes unusable while the deck
  // is strongly tilted or inverted. Reject it above 35 degrees, bridge the
  // short outage with vertical-velocity dead reckoning, then blend back to the
  // estimator altitude over 120 ms once the deck faces the floor again.
  const bool downward_range_valid = actual_rotation(2, 2) >= 0.819152044f;
  if (!acro_altitude_initialized) {
    acro_altitude_estimate_m = state->position.z;
    acro_altitude_initialized = true;
  } else if (downward_range_valid) {
    const float reacquire_alpha = 1.0f - expf(-DT / 0.120f);
    acro_altitude_estimate_m += reacquire_alpha * (
        state->position.z - acro_altitude_estimate_m);
  } else {
    acro_altitude_estimate_m += state->velocity.z * DT;
  }
  x0.segment<3>(0) = reference_rotation.transpose() * (
      Eigen::Vector3f(
          state->position.x, state->position.y, acro_altitude_estimate_m)
      - reference_position);
  x0.segment<3>(3) << attitude_error.x / denominator,
      attitude_error.y / denominator, attitude_error.z / denominator;
  x0.segment<3>(6) = reference_rotation.transpose() * (
      actual_velocity - reference_velocity);
  x0.segment<3>(9) = actual_omega - reference_omega_in_actual_body;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    acro_motor_baseline_n(motor) = reference[13 + motor];
  }
  return;
  }
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  const Eigen::Vector3f velocity_local = worldVectorToLocal(
      active_local_frame,
      Eigen::Vector3f(state->velocity.x, state->velocity.y, state->velocity.z));
  const struct vec attitude_local =
      worldQuaternionToLocalRodrigues(active_local_frame, attitude);

  x0.head(3).setZero();
  x0(3) = attitude_local.x;
  x0(4) = attitude_local.y;
  x0(5) = attitude_local.z;
  x0.segment<3>(6) = velocity_local;
  x0.segment<3>(9) << radians(sensors->gyro.x),
      radians(sensors->gyro.y), radians(sensors->gyro.z);
#endif
}

static float cross2d(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return a.x() * b.y() - a.y() * b.x();
}

static void sampleTrajectoryReference(
    float sample_position, float reference[TRAJECTORY_REFERENCE_DIM]) {
  sample_position = T_MIN(T_MAX(sample_position, 0.0f),
                          (float)(traj_length - 1u));
  const uint32_t lower = (uint32_t)floorf(sample_position);
  const uint32_t upper = lower + 1u < traj_length ? lower + 1u : lower;
  const float alpha = sample_position - (float)lower;
  for (int field = 0; field < TRAJECTORY_REFERENCE_DIM; ++field) {
    reference[field] =
        (1.0f - alpha) * trajectory_reference_data[lower][field]
        + alpha * trajectory_reference_data[upper][field];
  }
  float quaternion_norm_sq = 0.0f;
  for (int field = 3; field <= 6; ++field) {
    quaternion_norm_sq += reference[field] * reference[field];
  }
  const float quaternion_inverse_norm = 1.0f / sqrtf(quaternion_norm_sq);
  for (int field = 3; field <= 6; ++field) {
    reference[field] *= quaternion_inverse_norm;
  }
}

static Eigen::Vector3f trajectoryPointWorld(uint32_t knot) {
  const float x = trajectory_reference_data[knot][0];
  const float y = trajectory_reference_data[knot][1];
  return trajectory_origin_world + Eigen::Vector3f(
      trajectory_cos_yaw * x - trajectory_sin_yaw * y,
      trajectory_sin_yaw * x + trajectory_cos_yaw * y,
      trajectory_reference_data[knot][2]);
}

static uint32_t nextWaypointKnot(uint32_t current) {
  constexpr float waypoint_spacing_m = 0.30f;
  const Eigen::Vector3f anchor = trajectoryPointWorld(current);
  for (uint32_t candidate = current + 1u;
       candidate < TRAJECTORY_SAMPLE_COUNT; ++candidate) {
    Eigen::Vector3f displacement = trajectoryPointWorld(candidate) - anchor;
    displacement.z() = 0.0f;
    if (displacement.head(2).norm() >= waypoint_spacing_m) {
      return candidate;
    }
  }
  return TRAJECTORY_SAMPLE_COUNT - 1u;
}

#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
static void buildWaypointRoute(void) {
  uint16_t count = 0;
  uint32_t knot = 0;
  while (count < TINYMPC_MAX_ROUTE_WAYPOINTS) {
    const Eigen::Vector3f point = trajectoryPointWorld(knot);
    route_waypoints[count] = {point.x(), point.y(), point.z(), 0.0f};
    route_source_knots[count] = knot;
    ++count;
    if (knot + 1u >= TRAJECTORY_SAMPLE_COUNT) {
      break;
    }
    const uint32_t next = nextWaypointKnot(knot);
    if (next <= knot) {
      break;
    }
    knot = next;
  }
  /* Preserve the exact terminal point even when the final segment is shorter
   * than the nominal spacing. */
  if (count < TINYMPC_MAX_ROUTE_WAYPOINTS &&
      route_source_knots[count - 1u] != TRAJECTORY_SAMPLE_COUNT - 1u) {
    const uint32_t terminal = TRAJECTORY_SAMPLE_COUNT - 1u;
    const Eigen::Vector3f point = trajectoryPointWorld(terminal);
    route_waypoints[count] = {point.x(), point.y(), point.z(), 0.0f};
    route_source_knots[count] = terminal;
    ++count;
  }
  /* Store the outbound chord tangent at each discrete waypoint. The vehicle
   * settles and turns to this heading before the navigator releases the next
   * segment. This avoids turning while crossing a lobe tip. */
  for (uint16_t index = 0; index < count; ++index) {
    const uint16_t after = index + 1u < count ? index + 1u : index;
    const uint16_t before = index > 0 ? index - 1u : index;
    const uint16_t direction_start = after != index ? index : before;
    const float dx = route_waypoints[after].x - route_waypoints[direction_start].x;
    const float dy = route_waypoints[after].y - route_waypoints[direction_start].y;
    route_waypoints[index].yaw_rad = atan2f(dy, dx);
  }
  tinyMpcWaypointNavigatorInit(
      &waypoint_navigator, route_waypoints, count, 0.10f);
  waypoint_yaw_settle_active = false;
  step = route_source_knots[0];
  DEBUG_PRINT("Waypoint route ready count=%u spacing=0.30m radius=0.10m\n",
              (unsigned int)count);
}
#endif

static int8_t trajectoryOutsidePassSide() {
  const uint32_t first = T_MIN(step + 4u, traj_length - 1u);
  const uint32_t middle = T_MIN(step + 12u, traj_length - 1u);
  const uint32_t last = T_MIN(step + 20u, traj_length - 1u);
  Eigen::Vector3f incoming = trajectoryPointWorld(middle) -
      trajectoryPointWorld(first);
  Eigen::Vector3f outgoing = trajectoryPointWorld(last) -
      trajectoryPointWorld(middle);
  incoming.z() = 0.0f;
  outgoing.z() = 0.0f;
  const float curvature = cross2d(incoming, outgoing);
  if (fabsf(curvature) < 1.0e-5f) {
    return 0;
  }
  /* Positive curvature turns left, whose outside lane is to the right. */
  return curvature > 0.0f ? -1 : 1;
}

static void rejoinTrajectory(
    const Eigen::Vector3f& position_world,
    const Eigen::Vector3f& heading_world) {
  uint32_t nearest = step;
  float nearest_forward_distance = INFINITY;
  /* A figure-eight has a spatially coincident crossover. Searching only a
   * bounded forward window preserves the active lobe instead of jumping to a
   * much later, spatially nearby branch. It is also safer on every track. */
  const uint32_t search_end = T_MIN(
      step + (uint32_t)(4U * MPC_RATE), traj_length - 1u);
  for (uint32_t knot = step; knot < search_end; ++knot) {
    Eigen::Vector3f a = trajectoryPointWorld(knot);
    a.z() = 0.0f;
    Eigen::Vector3f next = trajectoryPointWorld(knot + 1);
    next.z() = 0.0f;
    const Eigen::Vector3f segment = next - a;
    const float denominator = cross2d(heading_world, segment);
    if (fabsf(denominator) < 1e-5f) {
      continue;
    }
    const Eigen::Vector3f offset = a - position_world;
    const float forward_distance = cross2d(offset, segment) / denominator;
    const float segment_fraction = cross2d(offset, heading_world) / denominator;
    if (forward_distance >= 0.0f && segment_fraction >= 0.0f &&
        segment_fraction <= 1.0f && forward_distance < nearest_forward_distance) {
      nearest_forward_distance = forward_distance;
      nearest = knot + 1;
    }
  }
  if (!isfinite(nearest_forward_distance)) {
    for (uint32_t knot = step; knot <= search_end; ++knot) {
      Eigen::Vector3f point = trajectoryPointWorld(knot);
      point.z() = 0.0f;
      const float forward_distance = heading_world.dot(point - position_world);
      if (forward_distance >= 0.0f && forward_distance < nearest_forward_distance) {
        nearest_forward_distance = forward_distance;
        nearest = knot;
      }
    }
  }
  if (!isfinite(nearest_forward_distance)) {
    float nearest_distance_sq = INFINITY;
    for (uint32_t knot = step; knot <= search_end; ++knot) {
      Eigen::Vector3f offset = trajectoryPointWorld(knot) - position_world;
      offset.z() = 0.0f;
      const float distance_sq = offset.squaredNorm();
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest = knot;
      }
    }
    nearest_forward_distance = sqrtf(nearest_distance_sq);
  }
  step = nearest;
  DEBUG_PRINT("Vision trajectory rejoined ahead at knot=%lu distance=%.2f\n",
              (unsigned long)step, (double)nearest_forward_distance);
}

/* Gate perception/control is deliberately detached while the avoidance head
 * is validated. Course scenes and acceptance manifests contain no gates. */
#define TINYMPC_COURSE_GATE_COUNT 0u
static const TinyRacerGateDefinition course_gates[] = {
  {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
};
static const uint8_t course_gate_count = TINYMPC_COURSE_GATE_COUNT;

static void updateCourseGateProgress(const state_t *state) {
  if (course_gate_count == 0u || course_gate_progress.next_gate >= course_gate_count) {
    course_gate_approach_active = false;
    return;
  }
  const float dx = state->position.x - trajectory_origin_world.x();
  const float dy = state->position.y - trajectory_origin_world.y();
  const float local_x = trajectory_cos_yaw * dx + trajectory_sin_yaw * dy;
  const float local_y = -trajectory_sin_yaw * dx + trajectory_cos_yaw * dy;
  const float local_z = state->position.z - trajectory_origin_world.z();
  const TinyRacerGateDefinition *gate =
      &course_gates[course_gate_progress.next_gate];
  const float gate_dx = local_x - gate->center_x_m;
  const float gate_dy = local_y - gate->center_y_m;
  const float signed_distance =
      gate->normal_x * gate_dx + gate->normal_y * gate_dy;
  /* The course planner knows which physical gate comes next. Reserve only its
   * final one-metre approach so a momentary gate-head dropout cannot make the
   * collision head steer around the frame. Vision still supplies centering. */
  if (!course_gate_approach_active && signed_distance <= 0.15f &&
      gate_dx * gate_dx + gate_dy * gate_dy <= 0.75f * 0.75f) {
    course_gate_approach_active = true;
  } else if (course_gate_approach_active && signed_distance > 0.20f) {
    /* The plane was crossed outside the usable opening. Release the latch so
     * the controller can recover, but do not advance ordered gate progress. */
    course_gate_approach_active = false;
  }
  if (!tinyRacerGateProgressUpdate(
      &course_gate_progress, course_gates, course_gate_count,
      local_x, local_y, local_z)) {
    return;
  }
  DEBUG_PRINT("Course gate crossed index=%u/%u\n",
              (unsigned int)course_gate_progress.next_gate,
              (unsigned int)course_gate_count);
  gate_priority_latched = false;
  gate_priority_rearm_ready = false;
  gate_priority_clear_samples = 0;
  gate_priority_hold_steps = 0;
  course_gate_approach_active = false;
}

static void updateObstacleTangent(const Eigen::Vector3f& position_world) {
  Eigen::Vector3f toward_center = perception_obstacle_center_world - position_world;
  toward_center.z() = 0.0f;
  if (toward_center.head(2).norm() < 0.01f) {
    toward_center = avoidance_forward_world;
  } else {
    toward_center.normalize();
  }
  perception_stop_plane.normal_world = toward_center;
  perception_stop_plane.boundary_world = toward_center.dot(
      perception_obstacle_center_world) - perception_obstacle_radius_m;
}

typedef struct {
  Eigen::Vector3f direction_local;
  float depth_m;
  float radius_m;
  float confidence;
  int sectors;
} ObstacleGeometry;

static float perceptionConfidence(float score) {
  return 1.0f / (1.0f + expf(-T_MIN(T_MAX(score, -10.0f), 10.0f)));
}

static bool estimateObstacleGeometry(
    const TinyRacerPerceptionObservation& observation,
    ObstacleGeometry& geometry) {
  if (!observation.has_metric_clearance) {
    return false;
  }
  Eigen::Vector3f direction_sum = Eigen::Vector3f::Zero();
  float weighted_depth = 0.0f;
  float weight_sum = 0.0f;
  float minimum_bearing = INFINITY;
  float maximum_bearing = -INFINITY;
  int sectors = 0;
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    if (observation.confidence[sector] < race_config.confidence_threshold ||
        observation.clearance_m[sector] >= race_config.clearance_threshold_m) {
      continue;
    }
    const float weight = perceptionConfidence(observation.confidence[sector]);
    const float bearing = perception_sector_bearing_rad[sector];
    direction_sum += weight * Eigen::Vector3f(cosf(bearing), sinf(bearing), 0.0f);
    weighted_depth += weight * observation.clearance_m[sector];
    weight_sum += weight;
    minimum_bearing = fminf(minimum_bearing, bearing);
    maximum_bearing = fmaxf(maximum_bearing, bearing);
    ++sectors;
  }
  if (sectors == 0 || weight_sum < 1e-3f ||
      direction_sum.head(2).norm() < 1e-3f) {
    return false;
  }
  geometry.direction_local = direction_sum.normalized();
  geometry.depth_m = weighted_depth / weight_sum;
  geometry.confidence = weight_sum / (float)sectors;
  geometry.sectors = sectors;
  const float angular_support = maximum_bearing - minimum_bearing +
      perception_sector_spacing_rad;
  geometry.radius_m = geometry.depth_m * tanf(0.5f * angular_support) +
      perception_drone_radius_m + race_config.safety_margin_m +
      perception_uncertainty_gain_m * (1.0f - geometry.confidence);
  return true;
}

static bool createObstacleCylinder(
    const Eigen::Vector3f& position_world,
    const TinyRacerPerceptionObservation& observation) {
  ObstacleGeometry geometry;
  if (!estimateObstacleGeometry(observation, geometry)) {
    return false;
  }
  const Eigen::Vector3f direction_world = localVectorToWorld(
      active_local_frame, geometry.direction_local);
  perception_obstacle_radius_m = geometry.radius_m;
  perception_obstacle_center_world = position_world + direction_world *
      (geometry.depth_m + perception_obstacle_radius_m);
  perception_obstacle_active = true;
  perception_halfspace_active = true;
  updateObstacleTangent(position_world);
  DEBUG_PRINT("Vision cylinder sectors=%d depth=%.2f confidence=%.2f radius=%.2f center=(%.2f,%.2f)\n",
              geometry.sectors, (double)geometry.depth_m,
              (double)geometry.confidence, (double)perception_obstacle_radius_m,
              (double)perception_obstacle_center_world.x(),
              (double)perception_obstacle_center_world.y());
  return true;
}

static void expandObstacleCylinder(
    const Eigen::Vector3f& position_world,
    const TinyRacerPerceptionObservation& observation) {
  ObstacleGeometry geometry;
  if (!estimateObstacleGeometry(observation, geometry)) {
    return;
  }
  const Eigen::Vector3f direction_world = localVectorToWorld(
      active_local_frame, geometry.direction_local);
  Eigen::Vector3f surface_point = position_world +
      direction_world * geometry.depth_m;
  surface_point.z() = perception_obstacle_center_world.z();
  const float measured_radius_m = fmaxf(
      geometry.radius_m,
      (surface_point - perception_obstacle_center_world).norm());
  if (measured_radius_m > perception_obstacle_radius_m) {
    perception_obstacle_radius_m +=
        0.1f * (measured_radius_m - perception_obstacle_radius_m);
  }
}

static int8_t trajectorySideOfCylinder() {
  uint32_t nearest = step;
  float nearest_distance_sq = INFINITY;
  for (uint32_t knot = step; knot < traj_length; ++knot) {
    const Eigen::Vector3f point = trajectoryPointWorld(knot);
    const float dx = point.x() - perception_obstacle_center_world.x();
    const float dy = point.y() - perception_obstacle_center_world.y();
    const float distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = knot;
    }
  }
  const Eigen::Vector3f projected = trajectoryPointWorld(nearest);
  return avoidance_left_world.dot(
      projected - perception_obstacle_center_world) < 0.0f ? -1 : 1;
}

static void filterPerceptionClearances(
    TinyRacerPerceptionObservation& observation) {
  if (!observation.has_metric_clearance) {
    return;
  }
  if (observation.valid &&
      observation.received_age_ms <= race_config.maximum_age_ms &&
      observation.sample != perception_filter_sample) {
    for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
      if (observation.confidence[sector] >= race_config.confidence_threshold) {
        perception_clearance_history[sector][perception_filter_index] =
            observation.clearance_m[sector];
      }
    }
    perception_filter_index = (perception_filter_index + 1) % 5;
    perception_filter_sample = observation.sample;
  }
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    float sorted[5];
    memcpy(sorted, perception_clearance_history[sector], sizeof(sorted));
    for (int i = 1; i < 5; ++i) {
      const float value = sorted[i];
      int j = i;
      while (j > 0 && sorted[j - 1] > value) {
        sorted[j] = sorted[j - 1];
        --j;
      }
      sorted[j] = value;
    }
    observation.clearance_m[sector] = fminf(
        observation.clearance_m[sector], sorted[2]);
  }
}

static void resetPerceptionFilter() {
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    for (int sample = 0; sample < 5; ++sample) {
      perception_clearance_history[sector][sample] = 0.35f;
    }
  }
  perception_filter_index = 0;
  perception_filter_sample = UINT32_MAX;
  gate_filter_sample = UINT32_MAX;
  gate_lateral_offset_m = 0.0f;
  gate_vertical_offset_m = 0.0f;
  tinyRacerNavigationReset(&navigation_state);
  memset(&navigation_intent, 0, sizeof(navigation_intent));
  tinyRacerDodgeReset(&dodge_state);
  memset(&dodge_intent, 0, sizeof(dodge_intent));
  navigation_warmup_steps = 0;
  gate_priority_latched = false;
  gate_priority_clear_samples = 0;
  gate_priority_hold_steps = 0;
  gate_priority_rearm_ready = true;
}

static void setLocalReferenceState(
    VectorNf& target, const Eigen::Vector3f& position_world,
    struct quat attitude_world_body, const Eigen::Vector3f& velocity_world,
    const Eigen::Vector3f& angular_velocity_body);

static void applyVisionNavigation(
    const TinyRacerPerceptionObservation& observation,
    float measured_forward_speed_mps,
    const Eigen::Vector3f& position_world,
    const Eigen::Vector3f& heading_world) {
  TinyRacerPerceptionObservation navigation_observation = observation;
  if (navigation_warmup_steps < 175u) {
    ++navigation_warmup_steps;
    navigation_observation.collision_probability = 0.0f;
  }
#if TINYMPC_SINGLE_OBSTACLE_COURSE
  /* A canonical course carries a coarse route-level encounter window. The
   * neural output still decides whether and when to dodge inside the window;
   * this only prevents a background-high logit elsewhere on the route from
   * consuming the course's sole obstacle encounter. Keep an in-progress
   * dodge active if its reference advancement crosses the window boundary. */
#if TRAJECTORY_OBSTACLE_APPROACH_WINDOW
  if (dodge_state.phase == TINYRACER_DODGE_TRACK && dodge_state.armed &&
      (step < TRAJECTORY_OBSTACLE_APPROACH_START_INDEX ||
       step > TRAJECTORY_OBSTACLE_APPROACH_END_INDEX)) {
    navigation_observation.collision_probability = 0.0f;
    navigation_observation.steering_command = 0.0f;
  }
  /* Once vision initiates the declared encounter, hold the established pass
   * lane through the obstacle's route station. Clear logits may begin while
   * the obstacle leaves the camera but must not rejoin the centerline early. */
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
  if (dodge_state.phase == TINYRACER_DODGE_PASS &&
      step <= TRAJECTORY_OBSTACLE_APPROACH_END_INDEX) {
    navigation_observation.collision_probability = 1.0f;
  }
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_HAIRPIN)
  if (dodge_state.phase == TINYRACER_DODGE_PASS &&
      step <= TRAJECTORY_OBSTACLE_INDEX) {
    navigation_observation.collision_probability = 1.0f;
  }
#endif
#endif
  /* These bounded demos declare exactly one physical obstacle. Continue
   * logging perception after the completed encounter, but do not let a
   * background-high collision output launch another dodge. */
  if (dodge_state.phase == TINYRACER_DODGE_TRACK && !dodge_state.armed) {
    return;
  }
#endif
  /* Gate-priority suppression is disabled with gate control: a false gate
   * detection must never mask the collision output during avoidance tests.
   * A calibrated gate candidate is the immediate target, not an obstacle.
   * Use the deployment's structured-confidence threshold even when a corner
   * is temporarily geometrically invalid, so one flickering heatmap cannot
   * launch a dodge through the gate frame. */
  if (false && !gate_priority_latched && gate_priority_rearm_ready &&
      observation.gate_confidence >= 0.6174671283f) {
    gate_priority_latched = true;
    gate_priority_clear_samples = 0;
    gate_priority_hold_steps = 0;
  } else if (gate_priority_latched && observation.gate_confidence < 0.20f) {
    if (gate_priority_clear_samples < UINT8_MAX) {
      ++gate_priority_clear_samples;
    }
    if (gate_priority_clear_samples >= TINYMPC_GATE_PRIORITY_CLEAR_STEPS) {
      gate_priority_latched = false;
      gate_priority_clear_samples = 0;
      gate_priority_hold_steps = 0;
    }
  }
  if (gate_priority_latched) {
    if (gate_priority_hold_steps < UINT16_MAX) {
      ++gate_priority_hold_steps;
    }
    if (gate_priority_hold_steps >= TINYMPC_GATE_PRIORITY_MAX_STEPS) {
      gate_priority_latched = false;
      gate_priority_rearm_ready = false;
      gate_priority_clear_samples = 0;
      gate_priority_hold_steps = 0;
    }
  } else if (!gate_priority_rearm_ready && observation.gate_confidence < 0.20f) {
    gate_priority_rearm_ready = true;
  }
  if (gate_priority_latched || course_gate_approach_active) {
    navigation_observation.collision_probability = 0.0f;
  }
#if TINYMPC_COURSE_ACRO_AVAILABLE
  /* Once the ordered course is complete, reserve the scheduled maneuver
   * staging zone before counting its level-flight settling window. Otherwise
   * a high background collision logit can start a new dodge in the one frame
   * between gate confirmation and the stored-LTV vision latch. */
  if (course_gate_progress.next_gate >= course_gate_count &&
      step >= TINYMPC_COURSE_ACRO_TRIGGER_INDEX) {
    navigation_observation.collision_probability = 0.0f;
  }
#endif
  /* Once the finite course is complete, do not launch a new avoidance
   * encounter from scenery outside the reference. Existing dodge state still
   * receives clear samples and returns to TRACK normally. */
  if (step + 5u >= TRAJECTORY_SAMPLE_COUNT) {
    navigation_observation.collision_probability = 0.0f;
  }
  tinyRacerNavigationUpdate(
      &navigation_state, &navigation_observation, &navigation_config, DT,
      active_local_frame.yaw_world, &navigation_intent);
  const TinyRacerDodgePhase previous_phase = dodge_state.phase;
  TinyRacerDodgeConfig active_dodge_config = dodge_config;
  const int8_t outside_side = trajectoryOutsidePassSide();
  if (active_dodge_config.preferred_pass_side == 0 &&
      fabsf(navigation_intent.yaw_rate_rad_s) < 1.0e-5f && outside_side != 0) {
    active_dodge_config.preferred_pass_side = outside_side;
  }
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &active_dodge_config, DT,
      measured_forward_speed_mps, &dodge_intent);
  if (previous_phase == TINYRACER_DODGE_REJOIN &&
      dodge_state.phase == TINYRACER_DODGE_TRACK) {
#if TRAJECTORY_TANGENT_HEADING || \
    defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
    /* The reference has already advanced by measured navigation speed while
     * sidestepping. A ray/curve intersection can select a distant branch of
     * a circle, figure-eight, or hairpin and create a discontinuous reference.
     * Retain the time-consistent knot on curved tracks; the lateral offset has
     * smoothly returned to zero at this transition. */
    DEBUG_PRINT("Vision curved trajectory rejoin retained knot=%lu\n",
                (unsigned long)step);
#else
    rejoinTrajectory(position_world, heading_world);
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    /* The shifted avoidance horizon is now obsolete. Cold-start the five-step
     * ADMM solve from the nominal track instead of retaining its lateral dual
     * bias for the rest of a long curved course. */
    resetLevelActuatorDuals();
#endif
  }
  if (!navigation_intent.active || race_intent.mode != TINYRACER_RACE_TRACK ||
      perception_halfspace_active || perception_recovery_active) {
    return;
  }
  /* DroNet owns encounter timing and pass-side selection. TinyMPC owns the
   * maneuver: shift the stored racing horizon laterally while preserving its
   * heading. This produces a banked sidestep without accumulating the large
   * yaw angles that make a single level linearization fragile. */
  Eigen::Vector3f path_local =
      Xref[NHORIZON - 1].head(3) - Xref[0].head(3);
  path_local.z() = 0.0f;
  if (path_local.head(2).norm() < 0.01f) {
    path_local = Xref[0].segment(6, 3);
    path_local.z() = 0.0f;
  }
  if (path_local.head(2).norm() < 0.01f) {
    path_local = Eigen::Vector3f::UnitX();
  } else {
    path_local.normalize();
  }
  const float commanded_forward_speed_mps =
      (gate_priority_latched || course_gate_approach_active)
      ? T_MIN(dodge_intent.forward_speed_mps, 0.18f)
      : dodge_intent.forward_speed_mps;
  for (int k = 0; k < NHORIZON; ++k) {
    const Eigen::Vector3f lateral_local(
        -path_local.y(), path_local.x(), 0.0f);
    const float future_offset = T_MIN(T_MAX(
        dodge_intent.lateral_offset_m +
            dodge_intent.lateral_rate_mps * (float)k * DT,
        -dodge_config.lateral_offset_m), dodge_config.lateral_offset_m);
    Xref[k].head(3) += lateral_local * future_offset;
    Xref[k](6) = path_local.x() * commanded_forward_speed_mps +
        lateral_local.x() * dodge_intent.lateral_rate_mps;
    Xref[k](7) = path_local.y() * commanded_forward_speed_mps +
        lateral_local.y() * dodge_intent.lateral_rate_mps;
  }
#if defined(CONFIG_PLATFORM_SITL)
  if (previous_phase != dodge_state.phase) {
    DEBUG_PRINT("DroNet banked dodge phase=%d side=%s offset=%.2f\n",
                (int)dodge_state.phase,
                dodge_state.pass_side > 0 ? "left" : "right",
                (double)dodge_state.lateral_offset_m);
  }
#endif
}

static void applyGateVisualServo(
    const TinyRacerPerceptionObservation& observation) {
  const bool fresh_gate = observation.valid && observation.gate_valid &&
      observation.received_age_ms <= race_config.maximum_age_ms &&
      observation.gate_confidence >= 0.25f;
  if (!fresh_gate || !tinyRacerGateServoAllowed(
      race_intent.mode, dodge_intent.phase,
      perception_halfspace_active, perception_recovery_active)) {
    return;
  }

  const float *corner = observation.gate_corners_xy;
  const float width = 0.5f * ((corner[2] - corner[0]) +
                              (corner[4] - corner[6]));
  const float height = 0.5f * ((corner[7] - corner[1]) +
                               (corner[5] - corner[3]));
  if (width < 0.05f || height < 0.05f || width * height < 0.01f) {
    return;
  }
  if (observation.sample != gate_filter_sample) {
    const float center_x = 0.25f *
        (corner[0] + corner[2] + corner[4] + corner[6]);
    const float center_y = 0.25f *
        (corner[1] + corner[3] + corner[5] + corner[7]);
    const float fx_normalized = observation.gate_fx_normalized;
    const float fy_normalized = observation.gate_fy_normalized;
    const float cx_normalized = observation.gate_cx_normalized;
    const float cy_normalized = observation.gate_cy_normalized;
    if (fx_normalized < 0.05f || fy_normalized < 0.05f ||
        cx_normalized < 0.0f || cx_normalized > 1.0f ||
        cy_normalized < 0.0f || cy_normalized > 1.0f) {
      return;
    }
    const float gate_opening_width_m = 0.45f;
    const float estimated_depth_m = T_MIN(T_MAX(
        gate_opening_width_m * fx_normalized / width, 0.4f), 4.0f);
    const float requested_lateral_m = T_MIN(T_MAX(
        -(center_x - cx_normalized) * estimated_depth_m / fx_normalized,
        -0.35f), 0.35f);
    const float requested_vertical_m = T_MIN(T_MAX(
        -(center_y - cy_normalized) * estimated_depth_m / fy_normalized,
        -0.25f), 0.25f);
    gate_lateral_offset_m +=
        0.25f * (requested_lateral_m - gate_lateral_offset_m);
    gate_vertical_offset_m +=
        0.25f * (requested_vertical_m - gate_vertical_offset_m);
    gate_filter_sample = observation.sample;
    if ((observation.sample % 10u) == 0u) {
      DEBUG_PRINT("Vision gate confidence=%.2f offset=(%.2f,%.2f) depth=%.2f\n",
                  (double)observation.gate_confidence,
                  (double)gate_lateral_offset_m,
                  (double)gate_vertical_offset_m,
                  (double)estimated_depth_m);
    }
  }
  for (int k = 0; k < NHORIZON; ++k) {
    const float ramp = (float)k / (float)(NHORIZON - 1);
    Xref[k](1) += ramp * gate_lateral_offset_m;
    Xref[k](2) += ramp * gate_vertical_offset_m;
  }
}

static void applyCourseGateTransitReference() {
  if (!course_gate_approach_active ||
      course_gate_progress.next_gate >= course_gate_count) {
    return;
  }
  const TinyRacerGateDefinition *gate =
      &course_gates[course_gate_progress.next_gate];
  const Eigen::Vector3f center_world = trajectory_origin_world +
      Eigen::Vector3f(
          trajectory_cos_yaw * gate->center_x_m -
              trajectory_sin_yaw * gate->center_y_m,
          trajectory_sin_yaw * gate->center_x_m +
              trajectory_cos_yaw * gate->center_y_m,
          gate->center_z_m);
  const Eigen::Vector3f normal_world(
      trajectory_cos_yaw * gate->normal_x -
          trajectory_sin_yaw * gate->normal_y,
      trajectory_sin_yaw * gate->normal_x +
          trajectory_cos_yaw * gate->normal_y,
      0.0f);
  const Eigen::Vector3f center_local = worldVectorToLocal(
      active_local_frame,
      center_world - Eigen::Vector3f(
          active_local_frame.origin_x, active_local_frame.origin_y,
          active_local_frame.origin_z));
  const Eigen::Vector3f normal_local = worldVectorToLocal(
      active_local_frame, normal_world).normalized();
  const Eigen::Vector3f lateral_local(
      -normal_local.y(), normal_local.x(), 0.0f);
  for (int k = 0; k < NHORIZON; ++k) {
    const float blend = (float)k / (float)(NHORIZON - 1);
    const Eigen::Vector3f offset = Xref[k].head(3) - center_local;
    Xref[k].head(3) -= blend * lateral_local * lateral_local.dot(offset);
    Xref[k](2) += blend * (center_local.z() - Xref[k](2));
    const float requested_speed = T_MIN(
        Xref[k].segment(6, 2).norm(), 0.18f);
    Xref[k](6) = (1.0f - blend) * Xref[k](6) +
        blend * normal_local.x() * requested_speed;
    Xref[k](7) = (1.0f - blend) * Xref[k](7) +
        blend * normal_local.y() * requested_speed;
  }
}

static void createDangerStopPlane(
    const Eigen::Vector3f& position_world,
    const Eigen::Vector3f& velocity_world,
    const TinyRacerPerceptionObservation& observation) {
  const float forward_speed_mps = T_MAX(
      avoidance_forward_world.dot(velocity_world), 0.0f);
  const float stopping_distance_m = T_MIN(T_MAX(
      0.20f + 0.25f * forward_speed_mps +
          forward_speed_mps * forward_speed_mps / 3.0f,
      0.25f), 0.75f);
  perception_stop_plane.normal_world = avoidance_forward_world;
  perception_stop_plane.boundary_world =
      avoidance_forward_world.dot(position_world) + stopping_distance_m;
  perception_binary_constraint = true;
  perception_obstacle_active = false;
  perception_halfspace_active = true;

  float bearing_sum = 0.0f;
  float weight_sum = 0.0f;
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    const float excess = T_MAX(observation.danger_probability[sector] -
                                   race_config.danger_probability_threshold,
                               0.0f);
    bearing_sum += excess * perception_sector_bearing_rad[sector];
    weight_sum += excess;
  }
  race_state.pass_side = weight_sum > 1e-4f && bearing_sum < 0.0f ? 1 : -1;
  race_intent.pass_side = race_state.pass_side;
  DEBUG_PRINT("Vision danger plane distance=%.2f action=%s probabilities=(%.2f,%.2f,%.2f,%.2f)\n",
              (double)stopping_distance_m,
              race_intent.pass_side > 0 ? "left" : "right",
              (double)observation.danger_probability[0],
              (double)observation.danger_probability[1],
              (double)observation.danger_probability[2],
              (double)observation.danger_probability[3]);
}

static bool horizonApproachesObstacle() {
  const Eigen::Vector3f origin(
      active_local_frame.origin_x, active_local_frame.origin_y,
      active_local_frame.origin_z);
  for (int k = 0; k < NHORIZON; ++k) {
    const Eigen::Vector3f point = origin + localVectorToWorld(
        active_local_frame, Xref[k].head(3));
    if ((point.head(2) - perception_obstacle_center_world.head(2)).norm() <=
        perception_obstacle_radius_m + 0.05f) {
      return true;
    }
  }
  return false;
}

static void setLocalReferenceState(
    VectorNf& target, const Eigen::Vector3f& position_world,
    struct quat attitude_world_body, const Eigen::Vector3f& velocity_world,
    const Eigen::Vector3f& angular_velocity_body) {
  const Eigen::Vector3f position_local =
      worldVectorToLocal(
          active_local_frame,
          position_world - Eigen::Vector3f(
              active_local_frame.origin_x,
              active_local_frame.origin_y,
              active_local_frame.origin_z));
  const Eigen::Vector3f velocity_local =
      worldVectorToLocal(active_local_frame, velocity_world);
  const struct vec attitude_local =
      worldQuaternionToLocalRodrigues(active_local_frame, attitude_world_body);
  target << position_local.x(), position_local.y(), position_local.z(),
      attitude_local.x, attitude_local.y, attitude_local.z,
      velocity_local.x(), velocity_local.y(), velocity_local.z(),
      angular_velocity_body.x(), angular_velocity_body.y(), angular_velocity_body.z();
}

void updateHorizonReference(const setpoint_t *setpoint, bool advance) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
#if TINYMPC_ACRO_AVAILABLE
    if (acroControlActive()) {
    // The state and input variables are errors about the sampled maneuver.
    // Physical motor limits therefore become correction limits around the
    // current feedforward operating point.
    for (int i = 0; i < NHORIZON; ++i) {
      Xref[i].setZero();
      reference_yaw_unwrapped_rad[i] = active_local_frame.yaw_world;
      if (i < NHORIZON - 1) {
        Uref[i].setZero();
      }
    }
    for (int motor = 0; motor < NINPUTS; ++motor) {
      lcu(motor) = -acro_motor_baseline_n(motor);
      const float maximum_motor_thrust = TINYMPC_ACRO_MAX_MOTOR_THRUST_N;
      ucu(motor) = maximum_motor_thrust - acro_motor_baseline_n(motor);
    }
    } else
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
    {
#if defined(TINYMPC_REFERENCE_MODE_TRAJECTORY)
    /* Dense, time-indexed tracking baseline. The world reference is sampled
     * from the same artifact waypoint mode subsamples, then transformed into
     * the freshly recentered vehicle-local chart for every solver iteration. */
    const float current_time_s = (float)step / (float)MPC_RATE;
    float yaw_reference = step == 0u
        ? active_local_frame.yaw_world : reference_yaw_phase_rad;
    for (int i = 0; i < NHORIZON; ++i) {
      const float reference_time_s = trajectory_handoff_hold_steps > 0u
          ? 0.0f : current_time_s + (float)i * DT;
      const float sample_position = reference_time_s / TRAJECTORY_SAMPLE_DT_S;
      float reference[TRAJECTORY_REFERENCE_DIM];
      sampleTrajectoryReference(sample_position, reference);
      const struct quat reference_attitude = qqmul(
          trajectory_yaw_rotation,
          mkquat(reference[4], reference[5], reference[6], reference[3]));
      const Eigen::Vector3f reference_position(
          trajectory_cos_yaw * reference[0] - trajectory_sin_yaw * reference[1],
          trajectory_sin_yaw * reference[0] + trajectory_cos_yaw * reference[1],
          reference[2]);
      const Eigen::Vector3f reference_velocity(
          trajectory_cos_yaw * reference[7] - trajectory_sin_yaw * reference[8],
          trajectory_sin_yaw * reference[7] + trajectory_cos_yaw * reference[8],
          reference[9]);
      setLocalReferenceState(
          Xref[i], trajectory_origin_world + reference_position,
          reference_attitude, reference_velocity,
          Eigen::Vector3f(reference[10], reference[11], reference[12]));
      yaw_reference = unwrapNear(quat2rpy(reference_attitude).z, yaw_reference);
      reference_yaw_unwrapped_rad[i] = yaw_reference;
      if (i < NHORIZON - 1) {
        float next_reference[TRAJECTORY_REFERENCE_DIM];
        sampleTrajectoryReference(
            sample_position + DT / TRAJECTORY_SAMPLE_DT_S, next_reference);
        const Eigen::Vector3f specific_force(
            (next_reference[7] - reference[7]) / DT,
            (next_reference[8] - reference[8]) / DT,
            (next_reference[9] - reference[9]) / DT + 9.81f);
        const float thrust_scale = specific_force.norm() / 9.81f - 1.0f;
        for (int motor = 0; motor < NINPUTS; ++motor) {
          Uref[i](motor) = T_MIN(T_MAX(
              tinympc_generated_physical_hover_thrust[motor] * thrust_scale,
              lcu(motor)), ucu(motor));
        }
      }
    }
    reference_yaw_phase_rad = reference_yaw_unwrapped_rad[0];
#elif defined(TINYMPC_REFERENCE_MODE_PROGRESS)
    /* Project onto a bounded local path window, then construct a fresh smooth
     * arc-length horizon. Progress follows the vehicle instead of wall time,
     * so disturbances never create a global-reference catch-up demand. */
    const Eigen::Vector3f vehicle_world(
        active_local_frame.origin_x,
        active_local_frame.origin_y,
        active_local_frame.origin_z);
    const Eigen::Vector3f vehicle_offset = vehicle_world - trajectory_origin_world;
    const TinyMpcPathPoint vehicle_path = {
        trajectory_cos_yaw * vehicle_offset.x()
            + trajectory_sin_yaw * vehicle_offset.y(),
        -trajectory_sin_yaw * vehicle_offset.x()
            + trajectory_cos_yaw * vehicle_offset.y(),
        vehicle_offset.z(),
    };
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
    progress_diag_cycle++;
#endif
    if (advance && trajectory_handoff_hold_steps == 0u &&
        !progress_path.complete) {
      const float before = progress_path.progress;
      tinyMpcProgressPathProject(&progress_path, vehicle_path);
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
      const float projection_delta = progress_path.progress - before;
      const bool material_projection_jump = projection_delta >= 5.0f;
      if (material_projection_jump ||
          (progress_diag_cycle % (uint32_t)MPC_RATE) == 0u) {
        DEBUG_PRINT(
            "UNCAPPED projection previous=%.3f current=%.3f delta=%.3f material=%u\n",
            (double)before, (double)progress_path.progress,
            (double)projection_delta,
            material_projection_jump ? 1u : 0u);
      }
#endif
      step = (uint32_t)floorf(progress_path.progress);
      if ((uint32_t)floorf(before) / 50u != step / 50u) {
        DEBUG_PRINT("Progress path sample=%.2f/%u\n",
                    (double)progress_path.progress,
                    (unsigned int)(progress_path.count - 1u));
      }
    }
    if (progress_path.complete && !progress_completion_reported) {
      progress_completion_reported = true;
      DEBUG_PRINT("Progress path complete sample=%.2f/%u\n",
                  (double)progress_path.progress,
                  (unsigned int)(progress_path.count - 1u));
    }
    TinyMpcPathSample first_sample = tinyMpcProgressPathSample(
        &progress_path, progress_path.progress);
    const float desired_first_yaw = atan2f(
        trajectory_sin_yaw * first_sample.tangent.x
            + trajectory_cos_yaw * first_sample.tangent.y,
        trajectory_cos_yaw * first_sample.tangent.x
            - trajectory_sin_yaw * first_sample.tangent.y);
    constexpr float maximum_local_yaw_deviation_rad = 0.2617993878f;
    const float desired_yaw_error = tinyMpcWrapAngle(
        desired_first_yaw - active_local_frame.yaw_world);
    const float bounded_yaw = active_local_frame.yaw_world + T_MIN(T_MAX(
        desired_yaw_error, -maximum_local_yaw_deviation_rad),
        maximum_local_yaw_deviation_rad);
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
    reference_yaw_phase_rad += tinyMpcWrapAngle(
        bounded_yaw - reference_yaw_phase_rad);
#else
    constexpr float maximum_yaw_step_rad =
        1.57079632679f / (float)MPC_RATE;
    reference_yaw_phase_rad = tinyMpcMoveAngleToward(
        reference_yaw_phase_rad, bounded_yaw, maximum_yaw_step_rad);
#endif
    float horizon_progress = progress_path.progress;
    float horizon_yaw = reference_yaw_phase_rad;
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
    float diag_curvature_per_m = 0.0f;
    float diag_speed_mps = 0.0f;
    float diag_yaw_rate_rad_s = 0.0f;
    float diag_roll_rad = 0.0f;
    float diag_pitch_rad = 0.0f;
    float diag_tilt_rad = 0.0f;
    float diag_thrust_scale = 0.0f;
    bool diag_uref_clamped = false;
#endif
    for (int i = 0; i < NHORIZON; ++i) {
      const TinyMpcPathSample sample = tinyMpcProgressPathSample(
          &progress_path, horizon_progress);
      const Eigen::Vector3f position_world = trajectory_origin_world
          + Eigen::Vector3f(
              trajectory_cos_yaw * sample.position.x
                  - trajectory_sin_yaw * sample.position.y,
              trajectory_sin_yaw * sample.position.x
                  + trajectory_cos_yaw * sample.position.y,
              sample.position.z);
      const Eigen::Vector3f tangent_world(
          trajectory_cos_yaw * sample.tangent.x
              - trajectory_sin_yaw * sample.tangent.y,
          trajectory_sin_yaw * sample.tangent.x
              + trajectory_cos_yaw * sample.tangent.y,
          sample.tangent.z);
      const float geometric_yaw = atan2f(tangent_world.y(), tangent_world.x());
      const float unwrapped_geometric_yaw = unwrapNear(geometric_yaw, horizon_yaw);
      const float geometric_delta = T_MIN(T_MAX(
          unwrapped_geometric_yaw - reference_yaw_phase_rad,
          -maximum_local_yaw_deviation_rad), maximum_local_yaw_deviation_rad);
      horizon_yaw = reference_yaw_phase_rad + geometric_delta;
      const Eigen::Vector3f normal_world(
          -tangent_world.y(), tangent_world.x(), 0.0f);
      const Eigen::Vector3f reference_acceleration = normal_world
          * (sample.curvature_per_m * sample.speed_mps * sample.speed_mps);
      const float heading_cos = cosf(horizon_yaw);
      const float heading_sin = sinf(horizon_yaw);
      const float forward_acceleration =
          heading_cos * reference_acceleration.x()
          + heading_sin * reference_acceleration.y();
      const float left_acceleration =
          -heading_sin * reference_acceleration.x()
          + heading_cos * reference_acceleration.y();
      const float raw_reference_roll = -atan2f(left_acceleration, 9.81f);
      const float raw_reference_pitch = atan2f(forward_acceleration, 9.81f);
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
      const float reference_roll = raw_reference_roll;
      const float reference_pitch = raw_reference_pitch;
      const float raw_tilt_rad = hypotf(
          raw_reference_roll, raw_reference_pitch);
      if (fabsf(sample.curvature_per_m) > fabsf(diag_curvature_per_m)) {
        diag_curvature_per_m = sample.curvature_per_m;
      }
      diag_speed_mps = T_MAX(diag_speed_mps, sample.speed_mps);
      const float raw_yaw_rate_rad_s =
          sample.curvature_per_m * sample.speed_mps;
      if (fabsf(raw_yaw_rate_rad_s) > fabsf(diag_yaw_rate_rad_s)) {
        diag_yaw_rate_rad_s = raw_yaw_rate_rad_s;
      }
      if (raw_tilt_rad > diag_tilt_rad) {
        diag_roll_rad = raw_reference_roll;
        diag_pitch_rad = raw_reference_pitch;
        diag_tilt_rad = raw_tilt_rad;
      }
#else
      constexpr float maximum_reference_tilt_rad = 0.1745329252f;
      const float reference_roll = T_MIN(T_MAX(
          raw_reference_roll, -maximum_reference_tilt_rad),
          maximum_reference_tilt_rad);
      const float reference_pitch = T_MIN(T_MAX(
          raw_reference_pitch, -maximum_reference_tilt_rad),
          maximum_reference_tilt_rad);
#endif
      const struct quat reference_attitude = rpy2quat(
          mkvec(reference_roll, reference_pitch, horizon_yaw));
      setLocalReferenceState(
          Xref[i], position_world, reference_attitude,
          tangent_world * sample.speed_mps,
          Eigen::Vector3f(0.0f, 0.0f,
                          sample.curvature_per_m * sample.speed_mps));
      reference_yaw_unwrapped_rad[i] = horizon_yaw;
      if (i < NHORIZON - 1) {
        const float thrust_scale = sqrtf(
            1.0f + reference_acceleration.squaredNorm() / (9.81f * 9.81f))
            - 1.0f;
        for (int motor = 0; motor < NINPUTS; ++motor) {
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
          const float raw_uref =
              tinympc_generated_physical_hover_thrust[motor] * thrust_scale;
          Uref[i](motor) = T_MIN(T_MAX(raw_uref, lcu(motor)), ucu(motor));
          diag_uref_clamped = diag_uref_clamped || Uref[i](motor) != raw_uref;
#else
          Uref[i](motor) = T_MIN(T_MAX(
              tinympc_generated_physical_hover_thrust[motor] * thrust_scale,
              lcu(motor)), ucu(motor));
#endif
        }
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
        diag_thrust_scale = T_MAX(diag_thrust_scale, thrust_scale);
#endif
        horizon_progress = tinyMpcProgressPathAdvance(
            &progress_path, horizon_progress, sample.speed_mps * DT);
      }
    }
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
    uint8_t threshold_mask = 0u;
    threshold_mask |= diag_speed_mps > 0.150001f ? 1u : 0u;
    threshold_mask |= fabsf(diag_yaw_rate_rad_s) > 1.57079632679f ? 2u : 0u;
    threshold_mask |= diag_tilt_rad > 0.1745329252f ? 4u : 0u;
    threshold_mask |= diag_thrust_scale > 0.05f ? 8u : 0u;
    threshold_mask |= diag_uref_clamped ? 16u : 0u;
    const uint8_t newly_crossed_thresholds =
        threshold_mask & (uint8_t)~progress_diag_threshold_mask;
    progress_diag_threshold_mask |= threshold_mask;
    bool new_reference_max = false;
    if (fabsf(diag_curvature_per_m) >
        fabsf(progress_diag_max_curvature_per_m)) {
      progress_diag_max_curvature_per_m = diag_curvature_per_m;
      new_reference_max = true;
    }
    if (diag_speed_mps > progress_diag_max_speed_mps) {
      progress_diag_max_speed_mps = diag_speed_mps;
      new_reference_max = true;
    }
    if (fabsf(diag_yaw_rate_rad_s) >
        fabsf(progress_diag_max_yaw_rate_rad_s)) {
      progress_diag_max_yaw_rate_rad_s = diag_yaw_rate_rad_s;
      new_reference_max = true;
    }
    if (diag_tilt_rad > progress_diag_max_tilt_rad) {
      progress_diag_max_roll_rad = diag_roll_rad;
      progress_diag_max_pitch_rad = diag_pitch_rad;
      progress_diag_max_tilt_rad = diag_tilt_rad;
      new_reference_max = true;
    }
    if (diag_thrust_scale > progress_diag_max_thrust_scale) {
      progress_diag_max_thrust_scale = diag_thrust_scale;
      new_reference_max = true;
    }
    if (diag_uref_clamped && !progress_diag_uref_clamped) {
      progress_diag_uref_clamped = true;
      new_reference_max = true;
    }
    progress_diag_pending_max_log =
        progress_diag_pending_max_log || new_reference_max;
    const bool max_log_rate_ready = !progress_diag_has_max_log ||
        progress_diag_cycle - progress_diag_last_max_log_cycle >=
            (uint32_t)(MPC_RATE / 5);
    if (newly_crossed_thresholds != 0u ||
        (progress_diag_pending_max_log && max_log_rate_ready)) {
      DEBUG_PRINT(
          "UNCAPPED refmax progress=%.3f kappa=%.4f/m speed=%.3fm/s yaw_rate=%.4frad/s roll=%.4frad pitch=%.4frad tilt=%.4frad thrust_scale=%.4f uref_clamp=%u threshold_new=0x%02x\n",
          (double)progress_path.progress,
          (double)progress_diag_max_curvature_per_m,
          (double)progress_diag_max_speed_mps,
          (double)progress_diag_max_yaw_rate_rad_s,
          (double)progress_diag_max_roll_rad,
          (double)progress_diag_max_pitch_rad,
          (double)progress_diag_max_tilt_rad,
          (double)progress_diag_max_thrust_scale,
          progress_diag_uref_clamped ? 1u : 0u,
          (unsigned int)newly_crossed_thresholds);
      progress_diag_last_max_log_cycle = progress_diag_cycle;
      progress_diag_pending_max_log = false;
      progress_diag_has_max_log = true;
    }
#endif
#else
    /* Level flight is waypoint-driven, not clock-driven. A discrete endpoint
     * remains fixed until the vehicle reaches it. The dense generated
     * trajectory is only a repository of explicit knots. */
    const Eigen::Vector3f vehicle_world(
        active_local_frame.origin_x,
        active_local_frame.origin_y,
        active_local_frame.origin_z);
    const TinyMpcWaypoint *held_target =
        tinyMpcWaypointNavigatorTarget(&waypoint_navigator);
    constexpr float waypoint_yaw_tolerance_rad = 0.436332313f;  // 25 deg.
    const bool held_yaw_reached = tinyMpcWaypointYawReached(
        held_target, active_local_frame.yaw_world,
        waypoint_yaw_tolerance_rad);
    if (advance && trajectory_handoff_hold_steps == 0 &&
        !waypoint_yaw_settle_active && tinyMpcWaypointPositionReached(
            &waypoint_navigator,
            {vehicle_world.x(), vehicle_world.y(), vehicle_world.z()})) {
      waypoint_yaw_settle_active = true;
    }
    if (advance && trajectory_handoff_hold_steps == 0 &&
        waypoint_yaw_settle_active && held_yaw_reached) {
      const uint16_t previous = waypoint_navigator.current;
      if (tinyMpcWaypointNavigatorAdvance(&waypoint_navigator)) {
        waypoint_yaw_settle_active = false;
        step = route_source_knots[waypoint_navigator.current];
#if defined(TINYMPC_USE_ACTUATOR_LTI)
        /* A held waypoint is a distinct optimization problem. Do not carry
         * consensus duals from the previous fixed endpoint into the new one. */
        resetLevelActuatorDuals();
#endif
        DEBUG_PRINT(
            "Waypoint reached index=%u/%u next_knot=%lu complete=%u\n",
            (unsigned int)previous,
            (unsigned int)waypoint_navigator.count,
            (unsigned long)step,
            waypoint_navigator.complete ? 1u : 0u);
      }
    }
    const TinyMpcWaypoint *target =
        tinyMpcWaypointNavigatorTarget(&waypoint_navigator);
    const Eigen::Vector3f route_target_world(target->x, target->y, target->z);
    Eigen::Vector3f target_world = route_target_world;
    Eigen::Vector3f local_waypoint_deviation = route_target_world - vehicle_world;
    local_waypoint_deviation.z() = 0.0f;
    constexpr float maximum_local_position_deviation_m = 0.18f;
    const float horizontal_deviation_m = local_waypoint_deviation.head(2).norm();
    if (horizontal_deviation_m > maximum_local_position_deviation_m) {
      target_world.head(2) = vehicle_world.head(2)
          + local_waypoint_deviation.head(2)
              * (maximum_local_position_deviation_m / horizontal_deviation_m);
    }
    /* Slew toward the waypoint's route tangent. The directed deviation avoids
     * the +/-pi chart seam; every individual local demand remains bounded. */
    constexpr float maximum_yaw_step_rad =
        1.57079632679f / (float)MPC_RATE;  // 90 degrees/second.
    constexpr float maximum_local_yaw_deviation_rad = 0.2617993878f;  // 15 deg.
    const float route_tangent_error_rad = tinyMpcYawDeviationAvoidingPi(
        active_local_frame.yaw_world, target->yaw_rad);
    const float horizontal_speed_mps = hypotf(x0(6), x0(7));
    const float tangent_error_rad = horizontal_speed_mps > 0.15f
        ? atan2f(x0(7), x0(6))
        : route_tangent_error_rad;
    const float bounded_tangent_error_rad = T_MIN(T_MAX(
        tangent_error_rad, -maximum_local_yaw_deviation_rad),
        maximum_local_yaw_deviation_rad);
    const float locally_bounded_yaw = active_local_frame.yaw_world +
        bounded_tangent_error_rad;
    reference_yaw_phase_rad = tinyMpcMoveAngleToward(
        reference_yaw_phase_rad, locally_bounded_yaw, maximum_yaw_step_rad);
    /* Never let the slew state retain an error outside the local bound if the
     * measured vehicle yaw moves in the opposite direction. */
    const float phase_error_rad = tinyMpcWrapAngle(
        reference_yaw_phase_rad - active_local_frame.yaw_world);
    if (phase_error_rad > maximum_local_yaw_deviation_rad) {
      reference_yaw_phase_rad = active_local_frame.yaw_world +
          maximum_local_yaw_deviation_rad;
    } else if (phase_error_rad < -maximum_local_yaw_deviation_rad) {
      reference_yaw_phase_rad = active_local_frame.yaw_world -
          maximum_local_yaw_deviation_rad;
    }
    const float yaw_reference = reference_yaw_phase_rad;
    const struct quat reference_attitude = rpy2quat(
        mkvec(0.0f, 0.0f, yaw_reference));
    const Eigen::Vector3f reference_velocity = Eigen::Vector3f::Zero();
    for (int i = 0; i < NHORIZON; ++i) {
      setLocalReferenceState(
          Xref[i],
          target_world,
          reference_attitude,
          reference_velocity,
          Eigen::Vector3f::Zero());
      reference_yaw_unwrapped_rad[i] = yaw_reference;
      if (i < NHORIZON - 1) {
        Uref[i].setZero();
      }
    }
    reference_yaw_phase_rad = yaw_reference;
#endif
    }
#else
    {}
#endif
  }
  else {
    const struct quat reference_attitude = rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
                        radians(setpoint->attitude.pitch), 
        radians(setpoint->attitude.yaw)));
    setLocalReferenceState(
        xg,
        Eigen::Vector3f(
            setpoint->position.x, setpoint->position.y, setpoint->position.z),
        reference_attitude,
        Eigen::Vector3f(
            setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z),
        Eigen::Vector3f(
            radians(setpoint->attitudeRate.roll),
            radians(setpoint->attitudeRate.pitch),
            radians(setpoint->attitudeRate.yaw)));
    const float reference_yaw = unwrapNear(
        radians(setpoint->attitude.yaw),
        active_local_frame.yaw_world);
    for (int i = 0; i < NHORIZON; ++i) {
      reference_yaw_unwrapped_rad[i] = reference_yaw;
    }
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
  }
#if TINYMPC_COURSE_ACRO_AVAILABLE
  if (advance && en_traj &&
      reference_mode == TINYMPC_MODE_ACROBATIC_MANEUVER) {
    if (course_acro_step + 1U < TINYMPC_COURSE_ACRO_SAMPLE_COUNT) {
      ++course_acro_step;
    }
    return;
  }
#endif
  if (advance && en_traj) {
    if (trajectory_handoff_hold_steps > 0) {
      --trajectory_handoff_hold_steps;
    } else if ((TRAJECTORY_HAS_MOTOR_FEEDFORWARD ||
#if defined(TINYMPC_REFERENCE_MODE_TRAJECTORY)
                true
#else
                false
#endif
               ) &&
               step + 1u < TRAJECTORY_SAMPLE_COUNT) {
      step += 1;
    }
  }
}

static void __attribute__((unused)) applyRaceIntent(void) {
  const Eigen::Vector3f velocity_normal = Eigen::Vector3f::Zero();
  const Eigen::Vector3f lateral_local = worldVectorToLocal(
      active_local_frame, avoidance_left_world);
  const float motion_speed = perception_recovery_active
      ? perception_bypass_speed_mps : 0.0f;
  const Eigen::Vector3f motion_velocity_local = worldVectorToLocal(
      active_local_frame, avoidance_forward_world * motion_speed);
  for (int k = 0; k < NHORIZON; ++k) {
    if (perception_recovery_active) {
      const Eigen::Vector3f target_world = perception_motion_target_world +
          avoidance_forward_world * motion_speed * ((float)k * DT);
      const Eigen::Vector3f target_local = worldVectorToLocal(
          active_local_frame,
          target_world - Eigen::Vector3f(active_local_frame.origin_x,
                                         active_local_frame.origin_y,
                                         active_local_frame.origin_z));
      Xref[k](0) = target_local.x();
      Xref[k](1) = target_local.y();
      Xref[k](6) = motion_velocity_local.x();
      Xref[k](7) = motion_velocity_local.y();
    } else {
      Xref[k].head(3) += lateral_local * race_intent.lateral_offset_m *
          ((float)k / (float)(NHORIZON - 1));
    }
  }

  const PerceptionPlane *plane = perception_halfspace_active
      ? &perception_stop_plane : NULL;
  if (plane == NULL) {
    return;
  }
  const Eigen::Vector3f normal_local = worldVectorToLocal(
      active_local_frame, plane->normal_world);
  const float boundary_local = plane->boundary_world
      - plane->normal_world.x() * active_local_frame.origin_x
      - plane->normal_world.y() * active_local_frame.origin_y
      - plane->normal_world.z() * active_local_frame.origin_z;
  for (int k = 0; k < NHORIZON; ++k) {
    const float violation = normal_local.dot(Xref[k].head(3)) - boundary_local;
    if (violation > 0.0f) {
      Xref[k].head(3) -= violation * normal_local;
    }
    tiny_SetKinematicHalfspace(
        &work, k, 0, &normal_local, &velocity_normal, boundary_local,
        perception_halfspace_penalty, 1);
  }
}

static void __attribute__((unused)) updateRaceIntent(const state_t *state) {
  TinyRacerPerceptionObservation observation = {};
  sequentialObstacleLinkGetLatest(&observation);
  filterPerceptionClearances(observation);
  const bool was_active = race_intent.constraint_active;
  const bool continuing_encounter = race_intent.mode == TINYRACER_RACE_RECOVER;
  const Eigen::Vector3f position_world(
      state->position.x, state->position.y, state->position.z);
  const Eigen::Vector3f velocity_world(
      state->velocity.x, state->velocity.y, state->velocity.z);
  if (perception_halfspace_active) {
    if (!perception_binary_constraint &&
        (race_intent.constraint_active || perception_recovery_active) &&
        observation.valid &&
        observation.received_age_ms <= race_config.maximum_age_ms) {
      expandObstacleCylinder(position_world, observation);
    }
    if (!perception_binary_constraint) {
      updateObstacleTangent(position_world);
    }
  }
  if (perception_recovery_active) {
    Eigen::Vector3f travel = position_world - perception_recovery_last_world;
    travel.z() = 0.0f;
    perception_recovery_distance_m += travel.norm();
    perception_recovery_last_world = position_world;
    perception_motion_target_world += avoidance_forward_world *
        perception_bypass_speed_mps * DT;
  }
  Eigen::Vector3f path_local = Xref[NHORIZON - 1].head(3) - Xref[0].head(3);
  path_local.z() = 0.0f;
  if (path_local.head(2).norm() < 0.01f) {
    path_local = Xref[0].segment(6, 3);
    path_local.z() = 0.0f;
  }
  if (path_local.head(2).norm() < 0.01f) {
    path_local = Eigen::Vector3f::UnitX();
  } else {
    path_local.normalize();
  }
  const float path_bearing = atan2f(path_local.y(), path_local.x());
  uint8_t relevant_sector_mask = 0;
  float nearest_error = 10.0f;
  float second_error = 10.0f;
  int nearest_sector = 0;
  int second_sector = 1;
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    const float error = fabsf(remainderf(
        path_bearing - perception_sector_bearing_rad[sector],
        6.28318530717958647692f));
    if (error < nearest_error) {
      second_error = nearest_error;
      second_sector = nearest_sector;
      nearest_error = error;
      nearest_sector = sector;
    } else if (error < second_error) {
      second_error = error;
      second_sector = sector;
    }
  }
  relevant_sector_mask = (1u << nearest_sector) | (1u << second_sector);
  const float lateral_displacement = (float)race_intent.pass_side *
      avoidance_left_world.dot(position_world - avoidance_start_world);
  const bool release_ready = was_active &&
      fabsf(avoidance_left_world.dot(velocity_world)) < 0.10f &&
      lateral_displacement >= race_config.bypass_offset_m;
  tinyRacerRaceUpdate(
      &race_state, &observation, &race_config, DT, release_ready,
      perception_recovery_active,
      !perception_recovery_active && race_intent.mode == TINYRACER_RACE_TRACK,
      relevant_sector_mask, &race_intent);
#if defined(CONFIG_PLATFORM_SITL)
  static uint32_t last_vision_debug_sample = UINT32_MAX;
  static uint8_t vision_debug_samples = 0;
  if (observation.sample != last_vision_debug_sample &&
      vision_debug_samples < 5) {
    last_vision_debug_sample = observation.sample;
    ++vision_debug_samples;
    DEBUG_PRINT(
        "Vision sample=%lu valid=%d age=%lums danger=%d mask=0x%x "
        "blocked=%u active=%d mode=%d\n",
        (unsigned long)observation.sample, observation.valid,
        (unsigned long)observation.received_age_ms,
        observation.has_sector_danger, relevant_sector_mask,
        race_state.blocked_samples, race_intent.constraint_active,
        race_intent.mode);
  }
#endif
  const Eigen::Vector3f path_world = localVectorToWorld(
      active_local_frame, path_local);
  const struct vec vehicle_rpy = quat2rpy(qnormalize(attitude));
  const Eigen::Vector3f heading_world(
      cosf(vehicle_rpy.z), sinf(vehicle_rpy.z), 0.0f);
  applyVisionNavigation(
      observation, path_world.dot(velocity_world), position_world,
      heading_world);
  /* Gate transit and visual servo are intentionally detached for the
   * obstacle-only validation phase. */
  if (perception_recovery_active && !race_intent.constraint_active &&
      race_state.clear_samples >= race_config.clear_samples_required &&
      perception_recovery_distance_m >= perception_pass_distance_m) {
    rejoinTrajectory(position_world, heading_world);
    perception_recovery_active = false;
    perception_halfspace_active = false;
    perception_binary_constraint = false;
    tiny_ClearPositionHalfspaces(&work);
    DEBUG_PRINT("Vision avoidance complete; constraint disabled\n");
    return;
  }
  if (perception_obstacle_active && !perception_halfspace_active &&
      !perception_recovery_active && horizonApproachesObstacle()) {
    perception_halfspace_active = true;
    updateObstacleTangent(position_world);
    DEBUG_PRINT("Stored cylinder tangent reactivated\n");
  }
  if (was_active == race_intent.constraint_active) {
    return;
  }

  if (!race_intent.constraint_active) {
    perception_recovery_active = true;
    perception_recovery_distance_m = 0.0f;
    perception_recovery_last_world = position_world;
    perception_motion_target_world = position_world;
    race_state.clear_samples = 0;
    DEBUG_PRINT("Vision cylinder pass continuing lateral=%.2f\n",
                (double)lateral_displacement);
    return;
  }

  perception_recovery_active = false;
  perception_recovery_distance_m = 0.0f;
  if (continuing_encounter) {
    if (!perception_binary_constraint) {
      updateObstacleTangent(position_world);
    }
    DEBUG_PRINT("Vision blocked again; continuing pass=%s\n",
                race_intent.pass_side > 0 ? "left" : "right");
    return;
  }
  avoidance_forward_world = localVectorToWorld(
      active_local_frame, path_local);
  avoidance_left_world = Eigen::Vector3f(
      -avoidance_forward_world.y(), avoidance_forward_world.x(), 0.0f);
  avoidance_start_world = position_world;
  perception_motion_target_world = position_world;
  if (observation.has_metric_clearance &&
      createObstacleCylinder(position_world, observation)) {
    perception_binary_constraint = false;
    race_state.pass_side = trajectorySideOfCylinder();
    race_intent.pass_side = race_state.pass_side;
  } else if (observation.has_sector_danger) {
    createDangerStopPlane(position_world, velocity_world, observation);
  } else {
    race_state.obstacle_constraint_active = false;
    race_intent.constraint_active = false;
    race_intent.mode = TINYRACER_RACE_TRACK;
    return;
  }
  DEBUG_PRINT("Vision blocked action=%s path=%.0fdeg sectors=0x%x clearances=(%.2f,%.2f,%.2f,%.2f)\n",
              race_intent.pass_side > 0 ? "left" : "right",
              (double)(path_bearing * 57.2957795f), relevant_sector_mask,
              (double)observation.clearance_m[0],
              (double)observation.clearance_m[1],
              (double)observation.clearance_m[2],
              (double)observation.clearance_m[3]);
}

#if defined(TINYMPC_USE_STORED_LTV)
typedef Eigen::Matrix<float, TINYMPC_STORED_LTV_STATE_DIM, 1> StoredLtvState;
static StoredLtvState stored_ltv_Xhrz[NHORIZON];
static StoredLtvState stored_ltv_p[NHORIZON];

static float thrustToStoredRotorState(float thrust_n) {
  const float linear = TINYMPC_STORED_LTV_RPM_TO_THRUST_LINEAR;
  const float quadratic = TINYMPC_STORED_LTV_RPM_TO_THRUST_QUADRATIC;
  const float discriminant = T_MAX(
      linear * linear + 4.0f * quadratic * T_MAX(thrust_n, 0.0f), 0.0f);
  const float rpm = (-linear + sqrtf(discriminant)) / (2.0f * quadratic);
  return rpm / TINYMPC_STORED_LTV_ROTOR_STATE_SCALE_RPM;
}

static int storedLtvInterval(int horizon_knot) {
  const int requested = (int)acro_reference_index + horizon_knot;
  return requested < TINYMPC_STORED_LTV_INTERVALS
      ? requested : TINYMPC_STORED_LTV_INTERVALS - 1;
}

static void resetStoredLtvDuals(void) {
  for (int k = 0; k < NHORIZON - 1; ++k) {
    YU[k].setZero();
    ZU[k].setZero();
    ZU_new[k].setZero();
  }
}

static void solveStoredLtv(void) {
  stored_ltv_p[NHORIZON - 1].setZero();
  info.pri_res = 0.0f;
  info.dua_res = 0.0f;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (int k = NHORIZON - 2; k >= 0; --k) {
      const int interval = storedLtvInterval(k);
      const int a_offset = interval * TINYMPC_STORED_LTV_STATE_DIM
          * TINYMPC_STORED_LTV_STATE_DIM;
      const int b_offset = interval * TINYMPC_STORED_LTV_STATE_DIM * NINPUTS;
      const int f_offset = interval * TINYMPC_STORED_LTV_STATE_DIM;
      const int k_offset = interval * NINPUTS * TINYMPC_STORED_LTV_STATE_DIM;
      const int h_offset = interval * NINPUTS * NINPUTS;
      StoredLtvState value_gradient;
      VectorMf rhs;
      for (int row = 0; row < TINYMPC_STORED_LTV_STATE_DIM; ++row) {
        value_gradient(row) = stored_ltv_p[k + 1](row)
            + tinympc_stored_ltv_P_affine[f_offset + row];
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -TINYMPC_STORED_LTV_RHO * (ZU_new[k](motor) - YU[k](motor));
        for (int state = 0; state < TINYMPC_STORED_LTV_STATE_DIM; ++state) {
          value += tinympc_stored_ltv_B[b_offset + state * NINPUTS + motor]
              * value_gradient(state);
        }
        rhs(motor) = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = 0.0f;
        for (int column = 0; column < NINPUTS; ++column) {
          value += tinympc_stored_ltv_Hinv[h_offset + motor * NINPUTS + column]
              * rhs(column);
        }
        d[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_STORED_LTV_STATE_DIM; ++state) {
        float value = 0.0f;
        for (int row = 0; row < TINYMPC_STORED_LTV_STATE_DIM; ++row) {
          value += tinympc_stored_ltv_A[
                  a_offset + row * TINYMPC_STORED_LTV_STATE_DIM + state]
              * value_gradient(row);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value -= tinympc_stored_ltv_K[
                  k_offset + motor * TINYMPC_STORED_LTV_STATE_DIM + state]
              * rhs(motor);
        }
        stored_ltv_p[k](state) = value;
      }
    }

    for (int state = 0; state < NSTATES; ++state) {
      stored_ltv_Xhrz[0](state) = x0(state);
    }
    const int initial_interval = storedLtvInterval(0);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      stored_ltv_Xhrz[0](NSTATES + motor) =
          acro_motor_rotor_state_estimate(motor)
          - tinympc_stored_ltv_motor_state_reference[
              initial_interval * NINPUTS + motor];
    }
    for (int k = 0; k < NHORIZON - 1; ++k) {
      const int interval = storedLtvInterval(k);
      const int a_offset = interval * TINYMPC_STORED_LTV_STATE_DIM
          * TINYMPC_STORED_LTV_STATE_DIM;
      const int b_offset = interval * TINYMPC_STORED_LTV_STATE_DIM * NINPUTS;
      const int f_offset = interval * TINYMPC_STORED_LTV_STATE_DIM;
      const int k_offset = interval * NINPUTS * TINYMPC_STORED_LTV_STATE_DIM;
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -d[k](motor);
        for (int state = 0; state < TINYMPC_STORED_LTV_STATE_DIM; ++state) {
          value -= tinympc_stored_ltv_K[
                  k_offset + motor * TINYMPC_STORED_LTV_STATE_DIM + state]
              * stored_ltv_Xhrz[k](state);
        }
        Uhrz[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_STORED_LTV_STATE_DIM; ++state) {
        float value = tinympc_stored_ltv_affine[f_offset + state];
        for (int column = 0; column < TINYMPC_STORED_LTV_STATE_DIM; ++column) {
          value += tinympc_stored_ltv_A[
                  a_offset + state * TINYMPC_STORED_LTV_STATE_DIM + column]
              * stored_ltv_Xhrz[k](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value += tinympc_stored_ltv_B[b_offset + state * NINPUTS + motor]
              * Uhrz[k](motor);
        }
        stored_ltv_Xhrz[k + 1](state) = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float baseline = tinympc_stored_ltv_motor_command_reference[
            interval * NINPUTS + motor];
        const float lower = -baseline;
        const float upper = TINYMPC_ACRO_MAX_MOTOR_THRUST_N - baseline;
        const float projected = T_MIN(T_MAX(YU[k](motor) + Uhrz[k](motor), lower), upper);
        YU[k](motor) += Uhrz[k](motor) - projected;
        ZU_new[k](motor) = projected;
      }
    }
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    for (int motor = 0; motor < NINPUTS; ++motor) {
      info.pri_res = T_MAX(info.pri_res, fabsf(Uhrz[k](motor) - ZU_new[k](motor)));
    }
  }
  info.iter = 5;
}
#endif

#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
typedef Eigen::Matrix<float, TINYMPC_LEVEL_ACTUATOR_STATE_DIM, 1>
    LevelActuatorState;
static LevelActuatorState level_Xhrz[NHORIZON];
static LevelActuatorState level_p[NHORIZON];
static LevelActuatorState level_ZX_new[NHORIZON];
static LevelActuatorState level_YX[NHORIZON];
static VectorMf level_Uhrz[NHORIZON - 1];
static VectorMf level_d[NHORIZON - 1];
static VectorMf level_ZU_new[NHORIZON - 1];
static VectorMf level_YU[NHORIZON - 1];

static float thrustToLevelRotorState(float thrust_n) {
  const float linear = TINYMPC_LEVEL_RPM_TO_THRUST_LINEAR;
  const float quadratic = TINYMPC_LEVEL_RPM_TO_THRUST_QUADRATIC;
  const float discriminant = T_MAX(
      linear * linear + 4.0f * quadratic * T_MAX(thrust_n, 0.0f), 0.0f);
  const float rpm = (-linear + sqrtf(discriminant)) / (2.0f * quadratic);
  return rpm / TINYMPC_LEVEL_ROTOR_STATE_SCALE_RPM;
}

static float levelReferenceState(int knot, int state) {
  if (state < NSTATES) {
    return Xref[knot](state);
  }
  const int motor = state - NSTATES;
  const int input_knot = knot < NHORIZON - 1 ? knot : NHORIZON - 2;
  const float target_thrust = T_MIN(T_MAX(
      TINYMPC_LEVEL_HOVER_THRUST_N + Uref[input_knot](motor), 0.0f),
      TINYMPC_LEVEL_MAX_MOTOR_THRUST_N);
  return thrustToLevelRotorState(target_thrust)
      - TINYMPC_LEVEL_HOVER_ROTOR_STATE;
}

static void resetLevelActuatorDuals(void) {
  for (int k = 0; k < NHORIZON; ++k) {
    level_Xhrz[k].setZero();
    level_p[k].setZero();
    level_ZX_new[k].setZero();
    level_YX[k].setZero();
    if (k < NHORIZON - 1) {
      level_Uhrz[k].setZero();
      level_d[k].setZero();
      level_ZU_new[k].setZero();
      level_YU[k].setZero();
    }
  }
}

static void solveLevelActuatorLti(void) {
  info.pri_res = 0.0f;
  info.dua_res = 0.0f;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (int row = 0; row < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++row) {
      float terminal = 0.0f;
      for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
        terminal -= tinympc_level_actuator_P[
                row * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
            * levelReferenceState(NHORIZON - 1, column);
      }
      terminal -= TINYMPC_LEVEL_ACTUATOR_RHO
          * (level_ZX_new[NHORIZON - 1](row)
             - level_YX[NHORIZON - 1](row));
      level_p[NHORIZON - 1](row) = terminal;
    }

    for (int k = NHORIZON - 2; k >= 0; --k) {
      float r_tilde[NINPUTS];
      float rhs[NINPUTS];
      for (int motor = 0; motor < NINPUTS; ++motor) {
        r_tilde[motor] = -TINYMPC_LEVEL_ACTUATOR_RHO
            * (level_ZU_new[k](motor) - level_YU[k](motor));
        for (int column = 0; column < NINPUTS; ++column) {
          r_tilde[motor] -= tinympc_level_actuator_R[
              motor * NINPUTS + column] * Uref[k](column);
        }
        float value = r_tilde[motor] + tinympc_level_actuator_BPf[motor];
        for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
          value += tinympc_level_actuator_B[state * NINPUTS + motor]
              * level_p[k + 1](state);
        }
        rhs[motor] = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = 0.0f;
        for (int column = 0; column < NINPUTS; ++column) {
          value += tinympc_level_actuator_Hinv[motor * NINPUTS + column]
              * rhs[column];
        }
        level_d[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
        float value = -tinympc_level_actuator_Q_diagonal[state]
            * levelReferenceState(k, state)
            - TINYMPC_LEVEL_ACTUATOR_RHO
                * (level_ZX_new[k](state) - level_YX[k](state))
            + tinympc_level_actuator_APf[state];
        for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
          value += tinympc_level_actuator_AmBKt[
                  state * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
              * level_p[k + 1](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value -= tinympc_level_actuator_K[
                  motor * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + state]
              * r_tilde[motor];
          value += tinympc_level_actuator_coeff_d2p[state * NINPUTS + motor]
              * level_d[k](motor);
        }
        level_p[k](state) = value;
      }
    }

    for (int state = 0; state < NSTATES; ++state) {
      level_Xhrz[0](state) = x0(state);
    }
    for (int motor = 0; motor < NINPUTS; ++motor) {
      level_Xhrz[0](NSTATES + motor) =
          level_motor_rotor_state_snapshot(motor)
          - TINYMPC_LEVEL_HOVER_ROTOR_STATE;
    }
    for (int k = 0; k < NHORIZON - 1; ++k) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -level_d[k](motor);
        for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
          value -= tinympc_level_actuator_K[
                  motor * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + state]
              * level_Xhrz[k](state);
        }
        level_Uhrz[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
        float value = tinympc_level_actuator_affine[state];
        for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
          value += tinympc_level_actuator_A[
                  state * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
              * level_Xhrz[k](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value += tinympc_level_actuator_B[state * NINPUTS + motor]
              * level_Uhrz[k](motor);
        }
        level_Xhrz[k + 1](state) = value;
      }
    }

    for (int k = 0; k < NHORIZON - 1; ++k) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float lower = -TINYMPC_LEVEL_HOVER_THRUST_N;
        const float upper = TINYMPC_LEVEL_MAX_MOTOR_THRUST_N
            - TINYMPC_LEVEL_HOVER_THRUST_N;
        level_YU[k](motor) += level_Uhrz[k](motor);
        level_ZU_new[k](motor) = T_MIN(
            T_MAX(level_YU[k](motor), lower), upper);
        level_YU[k](motor) -= level_ZU_new[k](motor);
      }
    }
    for (int k = 0; k < NHORIZON; ++k) {
      level_YX[k] += level_Xhrz[k];
      level_ZX_new[k] = level_YX[k];
      for (int h = 0; h < MAX_HS; ++h) {
        if (!data.en_hs[k][h]) {
          continue;
        }
        const float violation =
            data.a_pos_hs[k][h].dot(level_ZX_new[k].head(3))
            + data.a_vel_hs[k][h].dot(level_ZX_new[k].segment(6, 3))
            - data.b_hs[k][h];
        const float positive_violation = T_MAX(violation, 0.0f);
        const float penalty = data.slack_penalty_hs[k][h];
        const float slack = penalty > 0.0f
            ? positive_violation / (1.0f + penalty) : 0.0f;
        data.slack_used_hs[k][h] = slack;
        const float correction = positive_violation - slack;
        if (correction > 0.0f) {
          level_ZX_new[k].head(3) -= correction * data.a_pos_hs[k][h];
          level_ZX_new[k].segment(6, 3) -=
              correction * data.a_vel_hs[k][h];
        }
      }
      level_YX[k] -= level_ZX_new[k];
    }
  }

  for (int k = 0; k < NHORIZON; ++k) {
    for (int state = 0; state < NSTATES; ++state) {
      Xhrz[k](state) = level_Xhrz[k](state);
      ZX_new[k](state) = level_ZX_new[k](state);
    }
    if (k < NHORIZON - 1) {
      Uhrz[k] = level_Uhrz[k];
      ZU_new[k] = level_ZU_new[k];
      for (int motor = 0; motor < NINPUTS; ++motor) {
        info.pri_res = T_MAX(
            info.pri_res,
            fabsf(level_Uhrz[k](motor) - level_ZU_new[k](motor)));
      }
    }
    for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
      info.pri_res = T_MAX(
          info.pri_res,
          fabsf(level_Xhrz[k](state) - level_ZX_new[k](state)));
    }
  }
  info.iter = 5;
}
#endif

#if TINYMPC_COURSE_ACRO_AVAILABLE
static void updateCourseAcroMode(
    const sensorData_t *sensors, const state_t *state) {
  if (reference_mode == TINYMPC_MODE_ACROBATIC_MANEUVER) {
    if (course_acro_step + 1U < TINYMPC_COURSE_ACRO_SAMPLE_COUNT) {
      return;
    }
    if (++course_acro_terminal_hold_steps < 10U) {
      return;
    }
    reference_mode = TINYMPC_MODE_WAYPOINT_TRACKING;
    course_acro_completed = true;
    course_acro_stable_steps = 0;
    const float yaw = quat2rpy(qnormalize(mkquat(
        state->attitudeQuaternion.x, state->attitudeQuaternion.y,
        state->attitudeQuaternion.z, state->attitudeQuaternion.w))).z;
    rejoinTrajectory(
        Eigen::Vector3f(state->position.x, state->position.y, 0.0f),
        Eigen::Vector3f(cosf(yaw), sinf(yaw), 0.0f));
    acro_altitude_initialized = false;
    resetLevelActuatorDuals();
    tinyRacerRaceReset(&race_state);
    tinyRacerDodgeReset(&dodge_state);
    memset(&race_intent, 0, sizeof(race_intent));
    memset(&dodge_intent, 0, sizeof(dodge_intent));
    perception_recovery_active = false;
    perception_obstacle_active = false;
    perception_halfspace_active = false;
    perception_binary_constraint = false;
    perception_recovery_distance_m = 0.0f;
    resetPerceptionFilter();
    tiny_ClearPositionHalfspaces(&work);
    DEBUG_PRINT("Course acro complete name=%s "
                "mode=WAYPOINT_TRACKING next_waypoint=%lu\n",
                TINYMPC_COURSE_ACRO_NAME, (unsigned long)step);
    return;
  }
  const bool clear_recovery_window =
      course_gate_progress.next_gate >= course_gate_count &&
      !gate_priority_latched &&
      race_intent.mode == TINYRACER_RACE_TRACK &&
      dodge_intent.phase == TINYRACER_DODGE_TRACK &&
      !perception_recovery_active && !perception_halfspace_active;
  if (clear_recovery_window) {
    if (course_acro_clear_steps < UINT8_MAX) {
      ++course_acro_clear_steps;
    }
  } else {
    course_acro_clear_steps = 0;
    course_acro_stable_steps = 0;
  }
  if (course_acro_completed || step < TINYMPC_COURSE_ACRO_TRIGGER_INDEX ||
      course_acro_clear_steps < 15u ||
      race_intent.mode != TINYRACER_RACE_TRACK ||
      dodge_intent.phase != TINYRACER_DODGE_TRACK ||
      perception_recovery_active || perception_halfspace_active) {
    return;
  }
  const struct quat current_attitude = qnormalize(mkquat(
      state->attitudeQuaternion.x, state->attitudeQuaternion.y,
      state->attitudeQuaternion.z, state->attitudeQuaternion.w));
  const struct vec rpy = quat2rpy(current_attitude);
  const float maximum_rate_deg_s = fmaxf(
      fabsf(sensors->gyro.x), fmaxf(fabsf(sensors->gyro.y), fabsf(sensors->gyro.z)));
  const float horizontal_speed_mps = hypotf(state->velocity.x, state->velocity.y);
  const bool physically_stable =
      fabsf(rpy.x) <= radians(5.0f) && fabsf(rpy.y) <= radians(5.0f) &&
      maximum_rate_deg_s <= 45.0f && horizontal_speed_mps <= 0.10f &&
      fabsf(state->velocity.z) <= 0.08f && state->position.z >= 0.60f;
  if (!physically_stable) {
    course_acro_stable_steps = 0;
    return;
  }
  if (course_acro_stable_steps < UINT8_MAX) {
    ++course_acro_stable_steps;
  }
  /* Preserve the proven endpoint reference and require 200 ms of genuinely
   * settled flight before anchoring the stored maneuver at the current pose. */
  if (course_acro_stable_steps < 10u) {
    return;
  }
  reference_mode = TINYMPC_MODE_ACROBATIC_MANEUVER;
  course_acro_stable_steps = 0;
  course_acro_step = 0;
  course_acro_terminal_hold_steps = 0;
  const float yaw = rpy.z;
  course_acro_cos_yaw = cosf(yaw);
  course_acro_sin_yaw = sinf(yaw);
  course_acro_yaw_rotation = rpy2quat(mkvec(0.0f, 0.0f, yaw));
  const float first_x = tinympc_course_acro_reference_data[0][0];
  const float first_y = tinympc_course_acro_reference_data[0][1];
  course_acro_origin_world = Eigen::Vector3f(
      state->position.x, state->position.y, state->position.z) - Eigen::Vector3f(
      course_acro_cos_yaw * first_x - course_acro_sin_yaw * first_y,
      course_acro_sin_yaw * first_x + course_acro_cos_yaw * first_y,
      tinympc_course_acro_reference_data[0][2]);
  acro_altitude_estimate_m = state->position.z;
  acro_altitude_initialized = true;
  resetStoredLtvDuals();
  tiny_ClearPositionHalfspaces(&work);
  DEBUG_PRINT("Course acro start name=%s "
              "mode=ACROBATIC_MANEUVER paused_waypoint=%lu\n",
              TINYMPC_COURSE_ACRO_NAME, (unsigned long)step);
}
#endif

static void tinympcControllerTask(void *parameters) {
  (void)parameters;
  uint32_t log_counter = 0;

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    setpoint_t setpoint_task;
    sensorData_t sensors_task;
    state_t state_task;
    uint32_t solve_tick;
    bool reset_requested;
#if TINYMPC_ACRO_AVAILABLE
    float motor_rotor_state_estimate_task[NINPUTS];
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    float level_motor_rotor_state_estimate_task[NINPUTS];
#endif
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &planner_setpoint, sizeof(setpoint_task));
    memcpy(&sensors_task, &planner_sensors, sizeof(sensors_task));
    memcpy(&state_task, &planner_state, sizeof(state_task));
    solve_tick = planner_tick;
    reset_requested = planner_reset_requested;
#if TINYMPC_ACRO_AVAILABLE
    memcpy(motor_rotor_state_estimate_task, planner_motor_rotor_state_estimate,
           sizeof(motor_rotor_state_estimate_task));
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    memcpy(level_motor_rotor_state_estimate_task,
           planner_level_motor_rotor_state_estimate,
           sizeof(level_motor_rotor_state_estimate_task));
#endif
    planner_reset_requested = false;
    xSemaphoreGive(dataMutex);

    if (reset_requested) {
      step = 0;
#if TINYMPC_ACRO_AVAILABLE
      acro_altitude_initialized = false;
#endif
#if TINYMPC_COURSE_ACRO_AVAILABLE
      reference_mode = TINYMPC_MODE_WAYPOINT_TRACKING;
      course_acro_completed = false;
      course_acro_step = 0;
      course_acro_terminal_hold_steps = 0;
      course_acro_clear_steps = 0;
      course_acro_stable_steps = 0;
#endif
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
      figure8_midcourse_dual_reset_complete = false;
#endif
#if defined(TINYMPC_USE_STORED_LTV)
      resetStoredLtvDuals();
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
      resetLevelActuatorDuals();
#endif
      const struct quat handoff_attitude = qnormalize(mkquat(
          state_task.attitudeQuaternion.x, state_task.attitudeQuaternion.y,
          state_task.attitudeQuaternion.z, state_task.attitudeQuaternion.w));
      const float handoff_yaw = quat2rpy(handoff_attitude).z;
      trajectory_cos_yaw = cosf(handoff_yaw);
      trajectory_sin_yaw = sinf(handoff_yaw);
      trajectory_yaw_rotation = rpy2quat(mkvec(0.0f, 0.0f, handoff_yaw));
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
      reference_yaw_phase_rad = handoff_yaw;
#endif
      trajectory_handoff_hold_steps = (uint16_t)(MPC_RATE * 2 / 5);
      const float first_x = trajectory_reference_data[0][0];
      const float first_y = trajectory_reference_data[0][1];
      trajectory_origin_world = Eigen::Vector3f(
          state_task.position.x, state_task.position.y, state_task.position.z) - Eigen::Vector3f(
          trajectory_cos_yaw * first_x - trajectory_sin_yaw * first_y,
          trajectory_sin_yaw * first_x + trajectory_cos_yaw * first_y,
                          trajectory_reference_data[0][2]);
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
#if defined(TINYMPC_REFERENCE_MODE_PROGRESS)
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
      constexpr float maximum_projection_advance_m = 1.0e6f;
#else
      constexpr float maximum_projection_advance_m = 0.02f;
#endif
      tinyMpcProgressPathInit(
          &progress_path, &trajectory_reference_data[0][0],
          TRAJECTORY_REFERENCE_DIM, progress_sample_count,
          4u, 60u, progress_minimum_speed_mps, progress_maximum_speed_mps,
          0.75f, maximum_projection_advance_m, 0.15f);
      progress_completion_reported = false;
#if defined(TINYMPC_PROGRESS_REFERENCE_UNCAPPED)
      resetUncappedProgressDiagnostics();
      DEBUG_PRINT(
          "Progress reference limits=uncapped projection_advance=finite-unbounded yaw_phase_slew=uncapped roll_pitch=uncapped local_yaw=+/-15deg\n");
      DEBUG_PRINT(
          "UNCAPPED diagnostics material_jump>=5knots thresholds speed>0.15m/s yaw_rate>1.5708rad/s tilt>0.1745rad thrust_scale>0.05 or Uref clamp\n");
      if (TINYMPC_PROGRESS_SPEED_MPS > 0.0f) {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=constant %.3fm/s curvature_gain=0.75m projection_step=uncapped\n",
            (unsigned int)progress_sample_count,
            (double)progress_minimum_speed_mps);
      } else {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=0.05..0.15m/s curvature_gain=0.75m projection_step=uncapped\n",
            (unsigned int)progress_sample_count);
      }
#else
      DEBUG_PRINT(
          "Progress reference limits=default projection_advance=0.02m yaw_phase_slew=90deg/s roll_pitch=+/-10deg local_yaw=+/-15deg\n");
      if (TINYMPC_PROGRESS_SPEED_MPS > 0.0f) {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=constant %.3fm/s curvature_gain=0.75m projection_step=0.02m\n",
            (unsigned int)progress_sample_count,
            (double)progress_minimum_speed_mps);
      } else {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=0.05..0.15m/s curvature_gain=0.75m projection_step=0.02m\n",
            (unsigned int)progress_sample_count);
      }
#endif
#elif !defined(TINYMPC_REFERENCE_MODE_TRAJECTORY)
      buildWaypointRoute();
#endif
#endif
      tinyRacerGateProgressReset(&course_gate_progress);
      course_gate_approach_active = false;
      DEBUG_PRINT("Trajectory origin=(%.2f,%.2f,%.2f) yaw=%.1fdeg hold=%.1fs\n",
                  (double)trajectory_origin_world.x(),
                  (double)trajectory_origin_world.y(),
                  (double)trajectory_origin_world.z(),
                  (double)(handoff_yaw * 57.2957795f),
                  (double)trajectory_handoff_hold_steps / MPC_RATE);
      tinyRacerRaceReset(&race_state);
      memset(&race_intent, 0, sizeof(race_intent));
      perception_recovery_active = false;
      perception_obstacle_active = false;
      perception_halfspace_active = false;
      perception_binary_constraint = false;
      perception_recovery_distance_m = 0.0f;
      resetPerceptionFilter();
      tiny_ClearPositionHalfspaces(&work);
    }

#if TINYMPC_COURSE_ACRO_AVAILABLE
    updateCourseAcroMode(&sensors_task, &state_task);
    if (reference_mode == TINYMPC_MODE_WAYPOINT_TRACKING) {
      updateCourseGateProgress(&state_task);
    }
#else
    updateCourseGateProgress(&state_task);
#endif
    updateInitialState(&sensors_task, &state_task);
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    /* The level ADMM auxiliaries live in the per-solve yaw-aligned frame.
     * Reusing them after that frame has rotated materially injects an old
     * coordinate chart into the new optimization problem. Periodically cold
     * start at a small accumulated rotation; motor-input warm starts remain
     * useful between these bounded chart changes. */
    constexpr float maximum_level_warm_start_yaw_change_rad =
        0.0872664626f;  // 5 deg.
    if (!level_warm_start_yaw_initialized) {
      level_warm_start_yaw_rad = active_local_frame.yaw_world;
      level_warm_start_yaw_initialized = true;
    } else if (fabsf(tinyMpcWrapAngle(
            active_local_frame.yaw_world - level_warm_start_yaw_rad)) >=
        maximum_level_warm_start_yaw_change_rad) {
      resetLevelActuatorDuals();
      level_warm_start_yaw_rad = active_local_frame.yaw_world;
    }
#endif
#if TINYMPC_ACRO_AVAILABLE
    for (int motor = 0; motor < NINPUTS; ++motor) {
      acro_motor_rotor_state_estimate(motor) = motor_rotor_state_estimate_task[motor];
    }
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    for (int motor = 0; motor < NINPUTS; ++motor) {
      level_motor_rotor_state_snapshot(motor) =
          level_motor_rotor_state_estimate_task[motor];
    }
#endif
    // Intentionally preserve TinyMPC's state auxiliaries and duals.
    const bool acro_active =
#if TINYMPC_ACRO_AVAILABLE
        acroControlActive();
#else
        false;
#endif
    updateHorizonReference(
        &setpoint_task,
        acro_active || race_intent.mode == TINYRACER_RACE_TRACK);
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8) && \
    !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI) && \
    !defined(TINYMPC_REFERENCE_MODE_TRAJECTORY) && \
    !defined(TINYMPC_REFERENCE_MODE_PROGRESS)
    if (!figure8_midcourse_dual_reset_complete && step >= 2600u) {
      resetLevelActuatorDuals();
      figure8_midcourse_dual_reset_complete = true;
      DEBUG_PRINT("Figure-eight numerical reset knot=%lu\n",
                  (unsigned long)step);
    }
#endif
    if (acro_active) {
      // Vision and level-chart position halfspaces are latched out until the
      // primitive ends and the level solver has been restored.
      tiny_ClearPositionHalfspaces(&work);
    } else {
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD
      updateRaceIntent(&state_task);
      applyRaceIntent();
#endif
    }

    const uint64_t solve_start_us = usecTimestamp();
#if TINYMPC_COURSE_ACRO_AVAILABLE
    if (acro_active) {
      solveStoredLtv();
    } else {
      solveLevelActuatorLti();
    }
#elif defined(TINYMPC_USE_STORED_LTV)
    solveStoredLtv();
#elif !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    solveLevelActuatorLti();
#else
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
#endif
    const uint32_t solve_us = (uint32_t)(usecTimestamp() - solve_start_us);
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float correction_now =
#if TINYMPC_COURSE_ACRO_AVAILABLE
          acro_active ? ZU_new[0](motor) : level_ZU_new[0](motor);
#elif defined(TINYMPC_USE_STORED_LTV)
          ZU_new[0](motor);
#elif !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
          level_ZU_new[0](motor);
#else
          Uhrz[0](motor);
#endif
      const float motor_thrust_now =
#if TINYMPC_COURSE_ACRO_AVAILABLE
          acro_active
              ? tinympc_stored_ltv_motor_command_reference[
                    storedLtvInterval(0) * NINPUTS + motor] + correction_now
              : tinympc_generated_physical_hover_thrust[motor] + correction_now;
#elif TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  #if defined(TINYMPC_USE_STORED_LTV)
          tinympc_stored_ltv_motor_command_reference[
              storedLtvInterval(0) * NINPUTS + motor] + correction_now;
  #else
          trajectory_reference_data[acro_reference_index][13 + motor]
              + correction_now;
  #endif
#else
          tinympc_generated_physical_hover_thrust[motor] + correction_now;
#endif
      float command =
          tinympc_generated_thrust_to_normalized_command(
              motor_thrust_now);
      active_motor_commands[motor] = T_MIN(T_MAX(command, 0.0f), 1.0f);
    }
    plan_start_tick = solve_tick;
    mpc_has_run = true;
    xSemaphoreGive(dataMutex);

#if defined(CONFIG_PLATFORM_SITL) && TINYMPC_ACRO_AVAILABLE
    static uint16_t last_acro_trace_index = UINT16_MAX;
    if (acro_active && acro_reference_index >= 45U && acro_reference_index <= 125U &&
        (acro_reference_index % 5U) == 0U &&
        acro_reference_index != last_acro_trace_index) {
      last_acro_trace_index = acro_reference_index;
      DEBUG_PRINT(
          "ACRO: idx=%u err_ry=%.4f err_wy=%.3f gyro_y=%.1f "
          "base=(%.3f,%.3f) corr=(%.3f,%.3f) cmd=(%.3f,%.3f)\n",
          (unsigned int)acro_reference_index,
          (double)x0(4), (double)x0(10), (double)sensors_task.gyro.y,
          (double)acro_motor_baseline_n(0), (double)acro_motor_baseline_n(1),
          (double)ZU_new[0](0), (double)ZU_new[0](1),
          (double)active_motor_commands[0], (double)active_motor_commands[1]);
    }
#endif

    {
      float max_primal_violation = 0.0f;
      float max_halfspace_slack = 0.0f;
      float max_primal_aux_gap = 0.0f;
      int diagnostic_last_knot =
          (int)ceilf((1.0f / (float)MPC_RATE) / DT);
      const int constrained_knots =
          TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS < NHORIZON
              ? TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS : NHORIZON;
      if (diagnostic_last_knot < 1) {
        diagnostic_last_knot = 1;
      }
      if (diagnostic_last_knot >= constrained_knots) {
        diagnostic_last_knot = constrained_knots - 1;
      }
      for (int k = 0; k <= diagnostic_last_knot; ++k) {
        const float primal_aux_gap =
            (Xhrz[k].head(3) - ZX_new[k].head(3)).cwiseAbs().maxCoeff();
        max_primal_aux_gap = T_MAX(max_primal_aux_gap, primal_aux_gap);
        for (int h = 0; h < MAX_HS; ++h) {
          if (!data.en_hs[k][h]) {
            continue;
          }
          const float violation =
              data.a_pos_hs[k][h].dot(Xhrz[k].head(3)) +
              data.a_vel_hs[k][h].dot(Xhrz[k].segment(6, 3)) - data.b_hs[k][h];
          max_primal_violation = T_MAX(max_primal_violation, violation);
          max_halfspace_slack =
              T_MAX(max_halfspace_slack, data.slack_used_hs[k][h]);
        }
      }
      const Eigen::Vector3f frame_origin(
          active_local_frame.origin_x, active_local_frame.origin_y,
          active_local_frame.origin_z);
      const Eigen::Vector3f ref_world = frame_origin + localVectorToWorld(
          active_local_frame, Xref[0].head(3));
      const Eigen::Vector3f horizon_world = frame_origin + localVectorToWorld(
          active_local_frame, Xref[NHORIZON - 1].head(3));
      tinyRacerDebug.ref_x = ref_world.x();
      tinyRacerDebug.ref_y = ref_world.y();
      tinyRacerDebug.ref_z = ref_world.z();
      tinyRacerDebug.horizon_x = horizon_world.x();
      tinyRacerDebug.horizon_y = horizon_world.y();
      tinyRacerDebug.horizon_z = horizon_world.z();
      tinyRacerDebug.cylinder_x = perception_obstacle_center_world.x();
      tinyRacerDebug.cylinder_y = perception_obstacle_center_world.y();
      tinyRacerDebug.cylinder_radius = perception_obstacle_radius_m;
      tinyRacerDebug.plane_nx = perception_stop_plane.normal_world.x();
      tinyRacerDebug.plane_ny = perception_stop_plane.normal_world.y();
      tinyRacerDebug.plane_boundary = perception_stop_plane.boundary_world;
      tinyRacerDebug.lateral_offset = race_intent.lateral_offset_m;
      tinyRacerDebug.plane_violation = max_primal_violation;
      tinyRacerDebug.consensus_error = max_primal_aux_gap;
      tinyRacerDebug.slack = max_halfspace_slack;
      tinyRacerDebug.solve_us = solve_us;
      tinyRacerDebug.mode = (uint8_t)race_intent.mode;
      tinyRacerDebug.cylinder_active = perception_halfspace_active ? 1 : 0;
      tinyRacerDebug.pass_side = race_intent.pass_side;
      if ((log_counter % MPC_RATE) == 0U) {
        DEBUG_PRINT(
            "MPC: iterations=%d solve_us=%lu ref_local=(%.2f,%.2f,%.2f) plane_violation=%.3f consensus_error=%.3f slack=%.3f\n",
            info.iter, (unsigned long)solve_us,
            (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2),
            (double)max_primal_violation,
            (double)max_primal_aux_gap,
            (double)max_halfspace_slack);
      }
    }
    if (log_counter == 0) {
      DEBUG_PRINT("MPC task stack free=%lu words\n",
                  (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    }
    log_counter++;
  }
}

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/
  loadGeneratedSolverData();

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 1, DT, &A, &B, &f);
  tiny_InitSettings(&stgs);
  stgs.rho_init = TINYMPC_GENERATED_ADMM_RHO;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCacheAffine(
      &work, &Quu_inv, &AmBKt, &coeff_d2p, &APf, &BPf);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalState(&work, Xref, &xg);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uref[k] = ug;
    Uhrz[k] = ug;
    ZU[k] = ug;
    ZU_new[k] = ug;
    YU[k].setZero();
  }
  for (int k = 0; k < NHORIZON; ++k) {
    ZX[k].setZero();
    ZX_new[k].setZero();
    YX[k].setZero();
  }
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
  resetLevelActuatorDuals();
  DEBUG_PRINT(
      "Level MPC cost=%s Qyaw=%.1f Qyaw_rate=%.1f Ryaw=%.6f\n",
      TINYMPC_LEVEL_COST_MODE_NAME,
      (double)TINYMPC_LEVEL_Q_YAW,
      (double)TINYMPC_LEVEL_Q_YAW_RATE,
      (double)TINYMPC_LEVEL_R_YAW_EIGENVALUE);
  for (int motor = 0; motor < NINPUTS; ++motor) {
    level_motor_rotor_state_estimate[motor] = 0.0f;
    planner_level_motor_rotor_state_estimate[motor] = 0.0f;
  }
#endif
#if TINYMPC_ACRO_AVAILABLE
  for (int motor = 0; motor < NINPUTS; ++motor) {
    motor_rotor_state_estimate[motor] = 0.0f;
    planner_motor_rotor_state_estimate[motor] = 0.0f;
  }
#endif
  tiny_ClearPositionHalfspaces(&work);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = TRAJECTORY_HAS_MOTOR_FEEDFORWARD ? 0 : 1;
  stgs.max_iter = 5;
  stgs.iters_check_rho_update = 0;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  /* End of MPC initialization */  
  step = 0;
  tinyRacerRaceReset(&race_state);
  memset(&race_intent, 0, sizeof(race_intent));
  perception_recovery_active = false;
  perception_obstacle_active = false;
  perception_halfspace_active = false;
  perception_binary_constraint = false;
  perception_recovery_distance_m = 0.0f;
  resetPerceptionFilter();
  mpc_has_run = false;
  last_controller_tick = 0;
  plan_start_tick = 0;
  motors_were_allowed = false;

  sequentialObstacleLinkInit();

  static bool task_initialized = false;
  if (!task_initialized) {
    runTaskSemaphore = xSemaphoreCreateBinary();
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask,
                           TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);
    task_initialized = true;
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int motor = 0; motor < NINPUTS; ++motor) {
    active_motor_commands[motor] = 0.0f;
  }
  planner_reset_requested = true;
  xSemaphoreGive(dataMutex);
  
  if (en_traj) {
    DEBUG_PRINT("Stored trajectory enabled\n");
  } else {
    DEBUG_PRINT("Commander/setpoint mode enabled\n");
  }
  DEBUG_PRINT("Exclusive TinyMPC direct motor control enabled\n");
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  const bool controller_reactivated =
      last_controller_tick == 0 || tick - last_controller_tick > M2T(200);
  last_controller_tick = tick;
  // CrazySim's SITL stabilizer intentionally bypasses the hardware supervisor
  // and always permits motor output. Mirror that contract here; hardware still
  // uses the real arming/supervisor state.
#ifdef CONFIG_PLATFORM_SITL
  #ifndef TINYMPC_SITL_START_DELAY_MS
  #define TINYMPC_SITL_START_DELAY_MS 0U
  #endif
  const bool motors_allowed =
      xTaskGetTickCount() >= M2T(TINYMPC_SITL_START_DELAY_MS);
#else
  const bool motors_allowed = supervisorAreMotorsAllowedToRun();
#endif
  bool has_run_snapshot = false;
  uint32_t plan_start_tick_snapshot = 0;
  float motor_command_snapshot[NINPUTS] = {0.0f};
  if (dataMutex != NULL && xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    if (controller_reactivated || (!motors_allowed && motors_were_allowed)) {
      mpc_has_run = false;
      planner_reset_requested = true;
#if TINYMPC_ACRO_AVAILABLE
      for (int motor = 0; motor < NINPUTS; ++motor) {
        motor_rotor_state_estimate[motor] = 0.0f;
      }
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
      for (int motor = 0; motor < NINPUTS; ++motor) {
        level_motor_rotor_state_estimate[motor] = 0.0f;
      }
#endif
    }
    if (motors_allowed && RATE_DO_EXECUTE(MPC_RATE, tick)) {
      memcpy(&planner_setpoint, setpoint, sizeof(planner_setpoint));
      memcpy(&planner_sensors, sensors, sizeof(planner_sensors));
      memcpy(&planner_state, state, sizeof(planner_state));
#if TINYMPC_ACRO_AVAILABLE
      memcpy(planner_motor_rotor_state_estimate, motor_rotor_state_estimate,
             sizeof(planner_motor_rotor_state_estimate));
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
      memcpy(planner_level_motor_rotor_state_estimate,
             level_motor_rotor_state_estimate,
             sizeof(planner_level_motor_rotor_state_estimate));
#endif
      planner_tick = tick;
      xSemaphoreGive(runTaskSemaphore);
    }
    has_run_snapshot = mpc_has_run;
    plan_start_tick_snapshot = plan_start_tick;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      motor_command_snapshot[motor] = active_motor_commands[motor];
    }
    xSemaphoreGive(dataMutex);
  }
  motors_were_allowed = motors_allowed;

  const bool command_is_fresh = has_run_snapshot &&
      (tick - plan_start_tick_snapshot <=
       M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS));
  control->controlMode = controlModePWM;
  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; ++motor) {
    const float hover_command =
        tinympc_generated_thrust_to_normalized_command(
            tinympc_generated_physical_hover_thrust[motor]);
    float command = command_is_fresh ? motor_command_snapshot[motor]
                                     : hover_command;
#ifdef CONFIG_PLATFORM_SITL
    // The CrazySim stabilizer bypasses the firmware motor supervisor. Restore
    // the gate here so the calibration/handoff delay cannot spin the plant.
    if (!motors_allowed) {
      command = 0.0f;
    }
#endif
    if (!std::isfinite(command)) {
      command = hover_command;
    }
    if (command < 0.0f) {
      command = 0.0f;
    } else if (command > 1.0f) {
      command = 1.0f;
    }
    control->normalizedForces[motor] = command;
#if TINYMPC_ACRO_AVAILABLE && defined(TINYMPC_USE_STORED_LTV)
    {
    const float target_thrust_n = motors_allowed
        ? tinympc_generated_normalized_command_to_thrust(command) : 0.0f;
    const float target_rotor_state = thrustToStoredRotorState(target_thrust_n);
    const float estimator_alpha = 1.0f - expf(
        -(1.0f / (float)LQR_RATE) / TINYMPC_STORED_LTV_MOTOR_TIME_CONSTANT_S);
    motor_rotor_state_estimate[motor] += estimator_alpha
        * (target_rotor_state - motor_rotor_state_estimate[motor]);
    }
#endif
#if !TRAJECTORY_HAS_MOTOR_FEEDFORWARD && defined(TINYMPC_USE_ACTUATOR_LTI)
    {
    const float target_thrust_n = motors_allowed
        ? T_MIN(tinympc_generated_normalized_command_to_thrust(command),
                TINYMPC_LEVEL_MAX_MOTOR_THRUST_N)
        : 0.0f;
    const float target_rotor_state = thrustToLevelRotorState(target_thrust_n);
    const float estimator_alpha = 1.0f - expf(
        -(1.0f / (float)LQR_RATE) / TINYMPC_LEVEL_MOTOR_TIME_CONSTANT_S);
    level_motor_rotor_state_estimate[motor] += estimator_alpha
        * (target_rotor_state - level_motor_rotor_state_estimate[motor]);
    }
#endif
  }
}

#ifdef __cplusplus
}
#endif
