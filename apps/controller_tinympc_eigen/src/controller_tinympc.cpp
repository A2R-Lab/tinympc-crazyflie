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
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <cmath>
#include "tinympc_generated_params.h"
#define NHORIZON TINYMPC_GENERATED_HORIZON_KNOTS
using namespace Eigen;

#if defined(__cplusplus)
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#if defined(CONFIG_PLATFORM_SITL)
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include "tinympc_sitl_mmap_diag.h"
#endif

#include "app.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "static_mem.h"

#include "controller.h"
#include "controller_pid.h"
#include "estimator.h"
#include "attitude_controller.h"
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
#include "tinympc_bank_selector.h"
#include "tinympc_braking_selector.h"
#include "tinympc_direct_plan_replay.h"
#include "tinympc_emergency_stop.h"
#include "tinympc_pitch_through_brake.h"
#include "tinympc_flip_primitive.h"
#include "tinympc_frenet_error.h"
#include "tinympc_gate_olgmd.h"
#include "tinympc_progress_path.h"
#include "tinympc_progress_yaw.h"
#include "tinympc_rate_command_guard.h"
#include "tinympc_path_tunnel.h"
#include "tinympc_power_loop.h"
#include "tinympc_reactive_power_loop.h"
#include "tinympc_vertical_active_sensing.h"
#include "tinympc_vision_residual_authority.h"
#include "tinympc_actor_state_link.h"
#include "tiny_pulp_dronet_v3_servo.h"
#include "pulp_dronet_v2_brake.h"
#include "tinyracer_reactive_escape.h"
#include "tinyracer_square_opening.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (10 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC ADMM"
// Run below the stabilizer (5) and sensors (4), but above background tasks so
// the 50 Hz worker consumes releases before its command reaches the age limit.
#define TINYMPC_TASK_PRI              3

#ifndef TINYMPC_DIRECT_DRONET_SERVO
#define TINYMPC_DIRECT_DRONET_SERVO 0
#endif
#ifndef TINYMPC_DIRECT_DRONET_STEERING_SILENCED
#define TINYMPC_DIRECT_DRONET_STEERING_SILENCED 0
#endif
#ifndef TINYMPC_PID_VISION_MODE
#define TINYMPC_PID_VISION_MODE 1
#endif
#define TINYMPC_PID_VISION_DRONETV3 1
#define TINYMPC_PID_VISION_DRONETV2 2
#define TINYMPC_PID_VISION_NANOFLOW 3
#define TINYMPC_PID_VISION_TINYVPC 4
#ifndef TINYMPC_REACTIVE_REFERENCE_FREE
#define TINYMPC_REACTIVE_REFERENCE_FREE 0
#endif
#ifndef TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2
#define TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2 0.50f
#endif
#ifndef TINYMPC_REACTIVE_YAW_ACCELERATION_RAD_S2
#define TINYMPC_REACTIVE_YAW_ACCELERATION_RAD_S2 0.50f
#endif
#ifndef TINYMPC_REACTIVE_POWER_LOOP_ENABLE
#define TINYMPC_REACTIVE_POWER_LOOP_ENABLE 0
#endif
#ifndef TINYMPC_GATE_OLGMD_ENABLE
#define TINYMPC_GATE_OLGMD_ENABLE 0
#endif
#ifndef TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
#define TINYMPC_OLGMD_CLEAR_RESUME_ENABLE 0
#endif
#ifndef TINYMPC_OLGMD_MOVING_SPEED_MPS
#define TINYMPC_OLGMD_MOVING_SPEED_MPS 0.15f
#endif
#ifndef TINYMPC_OLGMD_STOP_HOLD_S
#define TINYMPC_OLGMD_STOP_HOLD_S 3.0f
#endif
#ifndef TINYMPC_OLGMD_RESUME_REFRACTORY_S
#define TINYMPC_OLGMD_RESUME_REFRACTORY_S 2.0f
#endif
#ifndef TINYMPC_YAW_SPIN_TEST_ENABLE
#define TINYMPC_YAW_SPIN_TEST_ENABLE 0
#endif
#ifndef TINYMPC_YAW_SPIN_TEST_RATE_RAD_S
#define TINYMPC_YAW_SPIN_TEST_RATE_RAD_S 2.0f
#endif
#ifndef TINYMPC_YAW_SPIN_TEST_REVOLUTIONS
#define TINYMPC_YAW_SPIN_TEST_REVOLUTIONS 4.0f
#endif
#ifndef TINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS
#define TINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS 0.0f
#endif
#ifndef TINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S
#define TINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S 2.0f
#endif
#ifndef TINYMPC_ORACLE_ACTION_ENABLE
#define TINYMPC_ORACLE_ACTION_ENABLE 0
#endif
#ifndef TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
#define TINYMPC_VISION_DRONETV2_BRAKE_ENABLE 0
#endif
#ifndef TINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M
#define TINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M 0.50f
#endif
#ifndef TINYMPC_PAPER_ABLATION_CONTROL_ENABLE_X_M
#define TINYMPC_PAPER_ABLATION_CONTROL_ENABLE_X_M (-INFINITY)
#endif
#ifndef TINYMPC_PAPER_EMERGENCY_PITCH_RATE_LIMIT_RAD_S
#define TINYMPC_PAPER_EMERGENCY_PITCH_RATE_LIMIT_RAD_S 5.0f
#endif
#ifndef TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
#define TINYMPC_PITCH_THROUGH_BRAKE_ENABLE 0
#endif
#ifndef TINYMPC_PAPER_EMERGENCY_DECELERATION_MPS2
#ifdef TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2
#define TINYMPC_PAPER_EMERGENCY_DECELERATION_MPS2 \
  TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2
#else
#define TINYMPC_PAPER_EMERGENCY_DECELERATION_MPS2 6.0f
#endif
#endif
#ifndef TINYMPC_PROGRESS_SPEED_MPS
#define TINYMPC_PROGRESS_SPEED_MPS 0.0f
#endif
#ifndef TINYMPC_DIRECT_DRONET_SITL_THRUST_BASE_PWM
/* Hover feed-forward for CrazySim cf21B_500:
 * 65535 * sqrt((0.04338 kg * 9.81 m/s^2 / 4) / 0.20 N) = 47798.
 * Hardware retains the platform default; this only aligns the stock PID with
 * the selected simulated airframe before evaluating the published servo. */
#define TINYMPC_DIRECT_DRONET_SITL_THRUST_BASE_PWM 47798
#endif

#if TINYMPC_DIRECT_DRONET_SERVO
static_assert(TINYMPC_PID_VISION_MODE >= TINYMPC_PID_VISION_DRONETV3 &&
                  TINYMPC_PID_VISION_MODE <= TINYMPC_PID_VISION_TINYVPC,
              "invalid stock-PID vision mode");
static TinyPulpDronetV3Servo direct_dronet_servo =
    tinyPulpDronetV3ServoDefault(
        (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
            ? (float)TINYMPC_PROGRESS_SPEED_MPS : 0.5f);
static TinyPulpDronetV3Command direct_dronet_command = {0.0f, 0.0f};
static PulpDronetV2Brake direct_dronet_v2_brake =
    pulpDronetV2BrakeDefault();
static uint32_t direct_dronet_last_sample = 0u;
static bool direct_dronet_takeoff_initialized = false;
static uint32_t direct_dronet_takeoff_start_tick = 0u;
static float direct_dronet_takeoff_x_m = 0.0f;
static float direct_dronet_takeoff_y_m = 0.0f;
static float direct_dronet_takeoff_yaw_deg = 0.0f;
#define TINYMPC_DIRECT_DRONET_TAKEOFF_START_ALTITUDE_M 0.20f
#define TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_M 0.01f
#define TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_MS 50u
#define TINYMPC_DIRECT_DRONET_TAKEOFF_HOLD_MS 5000u
#endif

#if TINYMPC_REACTIVE_REFERENCE_FREE
static TinyRacerReactiveState reactive_reference_state;
static TinyRacerReactiveCommand reactive_reference_command = {
    0.0f, 0.0f, 0.0f, TINYRACER_REACTIVE_CRUISE, 0, false};
static TinyRacerReactiveConfig reactive_reference_config =
    tinyRacerReactiveDefaultConfig(
        (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
            ? (float)TINYMPC_PROGRESS_SPEED_MPS : 1.0f);
static uint32_t reactive_reference_last_sample = 0u;
static float reactive_reference_speed_mps = 0.0f;
static float reactive_reference_lateral_speed_mps = 0.0f;
static float reactive_reference_altitude_world_m = 0.0f;
static float reactive_reference_heading_world_rad = 0.0f;
static float reactive_reference_turn_rate_rad_s = 0.0f;
static Eigen::Vector3f reactive_reference_backtrack_anchor_world =
    Eigen::Vector3f::Zero();
static bool reactive_reference_initialized = false;
#define TINYRACER_REACTIVE_GATE_CONFIDENCE_THRESHOLD 0.90f
#ifndef TINYMPC_SQUARE_OPENING_THRESHOLD
/* Rebuild the promoted firmware with the validation-frozen v11 threshold. */
#define TINYMPC_SQUARE_OPENING_THRESHOLD 0.50f
#endif
static TinyRacerSquareOpeningState reactive_square_opening_state = {};
static TinyRacerSquareOpeningConfig reactive_square_opening_config =
    tinyRacerSquareOpeningDefaultConfig(
        (float)TINYMPC_SQUARE_OPENING_THRESHOLD);
static bool reactive_square_collision_stop_context = false;

static float tinyRacerReactiveSlewSignedSpeed(
    float current_speed_mps, float target_speed_mps,
    float maximum_acceleration_mps2, float maximum_deceleration_mps2,
    float dt_s) {
  if (!isfinite(current_speed_mps) || !isfinite(target_speed_mps) ||
      !isfinite(maximum_acceleration_mps2) ||
      !isfinite(maximum_deceleration_mps2) || !isfinite(dt_s) ||
      maximum_acceleration_mps2 <= 0.0f ||
      maximum_deceleration_mps2 <= 0.0f || dt_s <= 0.0f) {
    return 0.0f;
  }
  const float delta_mps = target_speed_mps - current_speed_mps;
  const bool reversing_direction =
      current_speed_mps * target_speed_mps < 0.0f;
  const bool reducing_magnitude =
      fabsf(target_speed_mps) < fabsf(current_speed_mps);
  const float rate_mps2 = reversing_direction || reducing_magnitude
      ? maximum_deceleration_mps2 : maximum_acceleration_mps2;
  const float maximum_step_mps = rate_mps2 * dt_s;
  return current_speed_mps + fminf(
      fmaxf(delta_mps, -maximum_step_mps), maximum_step_mps);
}
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
static TinyMpcReactivePowerLoopState reactive_power_loop_state;
static TinyMpcReactivePowerLoopConfig reactive_power_loop_config =
    tinyMpcReactivePowerLoopDefaultConfig();
static TinyMpcReactivePowerLoopCommand reactive_power_loop_command = {};
static uint16_t reactive_power_loop_step = 0u;
static Eigen::Vector3f reactive_power_loop_anchor_world =
    Eigen::Vector3f::Zero();
static Eigen::Vector3f reactive_power_loop_recovery_anchor_world =
    Eigen::Vector3f::Zero();
static float reactive_power_loop_yaw_world_rad = 0.0f;
#endif
#endif

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
static struct quat worldQuaternionToLocalQuaternion(
    const MpcLocalFrame& frame, struct quat quaternion_world_body) {
  const float half_yaw = 0.5f * frame.yaw_world;
  const float c = cosf(half_yaw);
  const float s = sinf(half_yaw);
  struct quat quaternion_local_body = mkquat(
      c * quaternion_world_body.x + s * quaternion_world_body.y,
      c * quaternion_world_body.y - s * quaternion_world_body.x,
      c * quaternion_world_body.z - s * quaternion_world_body.w,
      c * quaternion_world_body.w + s * quaternion_world_body.z);
  return qnormalize(quaternion_local_body);
}

static struct vec worldQuaternionToLocalRodrigues(
    const MpcLocalFrame& frame, struct quat quaternion_world_body) {
  const struct quat quaternion_local_body =
      worldQuaternionToLocalQuaternion(frame, quaternion_world_body);
  const float denominator = fabsf(quaternion_local_body.w) > 1e-6f
      ? quaternion_local_body.w : copysignf(1e-6f, quaternion_local_body.w);
  return mkvec(
      quaternion_local_body.x / denominator,
      quaternion_local_body.y / denominator,
      quaternion_local_body.z / denominator);
}

/* math3d's qqmul(q, p) stores the Hamilton product p*q. Put the maneuver
 * first so this returns the conventional world-yaw * maneuver composition:
 * the loop thrust direction then rotates with its world-frame path. */
static struct quat composeWorldYawWithManeuver(
    struct quat yaw_rotation, struct quat maneuver_attitude) {
  return qnormalize(qqmul(maneuver_attitude, yaw_rotation));
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

#ifndef TINYMPC_RATE_CASCADE
#define TINYMPC_RATE_CASCADE 0
#endif
#ifndef TINYMPC_BRAKING_CACHE_ENABLE
#define TINYMPC_BRAKING_CACHE_ENABLE 1
#endif
#if TINYMPC_RATE_CASCADE && defined(TINYMPC_DIRECT_PLAN_REPLAY)
#error "TINYMPC rate cascade and direct plan replay are mutually exclusive"
#endif
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE && !TINYMPC_REACTIVE_REFERENCE_FREE
#error "reactive power loop requires route-free reactive mode"
#endif
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE && !defined(TINYMPC_USE_ACTUATOR_LTI)
#error "reactive power loop requires the direct-motor actuator-LTI solver"
#endif
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE && TINYMPC_RATE_CASCADE
#error "reactive power loop uses direct motors and cannot use the rate cascade"
#endif
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE && TINYMPC_POWER_LOOP_ENABLE
#error "reactive cached power loop and legacy progress-triggered power loop are mutually exclusive"
#endif
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE && !defined(TINYMPC_USE_ACTUATOR_LTI)
#error "pitch-through braking requires the direct-motor actuator-LTI solver"
#endif
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE && TINYMPC_RATE_CASCADE
#error "pitch-through braking uses direct motors and cannot use the rate cascade"
#endif
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE && \
    (TINYMPC_POWER_LOOP_ENABLE || TINYMPC_REACTIVE_POWER_LOOP_ENABLE || \
     TINYMPC_FLIP_ENABLE)
#error "pitch-through braking is mutually exclusive with acrobatic loop/flip primitives"
#endif

// A direct command may be held briefly while the next asynchronous solve
// finishes. Past this deadline, command model-derived hover rather than flying
// indefinitely on a stale open-loop command.
#define TINYMPC_DIRECT_COMMAND_MAX_AGE_MS (3U * (1000U / MPC_RATE))
#define TINYMPC_RATE_COMMAND_MAX_AGE_MS (3U * (1000U / MPC_RATE))

static_assert(NSTATES == TINYMPC_GENERATED_STATE_DIM, "generated state dimension mismatch");
static_assert(NINPUTS == TINYMPC_GENERATED_INPUT_DIM, "generated input dimension mismatch");
static_assert(MAX_HS >= 5, "path tunnel and perception require five halfspaces");

#ifndef TINYMPC_PATH_TUNNEL_ENABLE
#define TINYMPC_PATH_TUNNEL_ENABLE 1
#endif
#ifndef TINYMPC_PATH_TUNNEL_HALF_WIDTH_1_M
#define TINYMPC_PATH_TUNNEL_HALF_WIDTH_1_M 0.30f
#endif
#ifndef TINYMPC_PATH_TUNNEL_HALF_WIDTH_2_M
#define TINYMPC_PATH_TUNNEL_HALF_WIDTH_2_M 0.30f
#endif
#ifndef TINYMPC_GATE_POSITION_FUSION_ENABLE
#define TINYMPC_GATE_POSITION_FUSION_ENABLE 0
#endif
#ifndef TINYMPC_GATE_WORLD_X_M
#define TINYMPC_GATE_WORLD_X_M 4.0f
#endif
#ifndef TINYMPC_GATE_WORLD_Y_M
#define TINYMPC_GATE_WORLD_Y_M 0.0f
#endif
#ifndef TINYMPC_GATE_WORLD_Z_M
#define TINYMPC_GATE_WORLD_Z_M 1.5f
#endif
#ifndef TINYMPC_GATE_PNP_FUSION_ENABLE
#define TINYMPC_GATE_PNP_FUSION_ENABLE 0
#endif
#ifndef TINYMPC_GATE_CENTER_BEARING_FUSION_ENABLE
#define TINYMPC_GATE_CENTER_BEARING_FUSION_ENABLE 0
#endif
/* This is an intentionally narrow combined gate/obstacle POC.  It is off in
 * every normal firmware build.  It has no gate map or pose: a short-lived
 * visual association may make only a bounded center-bearing reference shift.
 * It does not enable pose fusion or give a generic gate detection authority
 * over collision avoidance. */
#ifndef TINYMPC_GATE_OBSTACLE_POC_ENABLE
/* The split gate/oLGMD mode uses the existing bounded visual gate state
 * machine to establish SEARCH/ALIGN/TRANSIT context. */
#define TINYMPC_GATE_OBSTACLE_POC_ENABLE TINYMPC_GATE_OLGMD_ENABLE
#endif
/* The joint gate/obstacle policy is deliberately a distinct experiment from
 * the older two-teacher POC.  It may use the same bounded gate association and
 * servo, but its TRACK/LEFT/RIGHT packet is the only collision authority. */
#ifndef TINYMPC_JOINT_GATE_RL_ENABLE
#define TINYMPC_JOINT_GATE_RL_ENABLE 0
#endif
/* Opt-in continuous reference residuals carried by vision packet V4.  TinyMPC
 * remains the motor controller and retains its model, Q/R, constraints, and
 * cached bank linearizations. */
#ifndef TINYMPC_VISION_RL_RESIDUAL_ENABLE
#define TINYMPC_VISION_RL_RESIDUAL_ENABLE 0
#endif
/* NanoFlowNet-style active sensing is a nominal altitude-reference overlay,
 * not a learned residual action. It is deliberately opt-in. */
#ifndef TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE
#define TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE 0
#endif
#ifndef TINYMPC_VERTICAL_ACTIVE_SENSING_AMPLITUDE_M
#define TINYMPC_VERTICAL_ACTIVE_SENSING_AMPLITUDE_M 0.10f
#endif
#ifndef TINYMPC_VERTICAL_ACTIVE_SENSING_PERIOD_S
#define TINYMPC_VERTICAL_ACTIVE_SENSING_PERIOD_S 2.0f
#endif
#ifndef TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED
#define TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED 1.14531138f
#endif
#ifndef TINYMPC_GATE_CORNER_SPAN_M
#define TINYMPC_GATE_CORNER_SPAN_M 0.555f
#endif
#ifndef TINYMPC_GATE_TRANSIT_ALTITUDE_M
/* Standard IMAV gate opening center; horizontal gate position remains
 * vision-only. Centering vertically preserves rotor clearance to both rims. */
#define TINYMPC_GATE_TRANSIT_ALTITUDE_M 1.00f
#endif
#ifndef TINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED
#define TINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED 0.5f
#endif
#ifndef TINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED
#define TINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED 0.5f
#endif

enum {
  TINYMPC_PATH_TUNNEL_NORMAL_1_POSITIVE_SLOT = 0,
  TINYMPC_PATH_TUNNEL_NORMAL_1_NEGATIVE_SLOT = 1,
  TINYMPC_PATH_TUNNEL_NORMAL_2_POSITIVE_SLOT = 2,
  TINYMPC_PATH_TUNNEL_NORMAL_2_NEGATIVE_SLOT = 3,
  TINYMPC_PERCEPTION_HALFSPACE_SLOT = 4,
};

/* Include the level-flight trajectory to track. */
#if defined(TINYMPC_TRAJECTORY_STRAIGHT)
#include "trajectories/50hz/traj_straight_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_STRAIGHT_LONG)
#include "trajectories/50hz/traj_straight_long_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_STRAIGHT_9M)
#include "trajectories/50hz/traj_straight_9m_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_STRAIGHT_20M)
#include "trajectories/50hz/traj_straight_20m_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR)
#include "trajectories/50hz/traj_canonical_corridor_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CIRCLE)
#include "trajectories/50hz/traj_canonical_circle_50hz.h"
#elif defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
#include "trajectories/50hz/traj_imav22_circle_50hz.h"
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
#elif defined(TINYMPC_TRAJECTORY_DRONET_U)
#include "trajectories/50hz/traj_dronet_u_50hz.h"
#else
#include "trajectories/50hz/traj_circle_50hz.h"
#endif

#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
#include "trajectories/50hz/traj_reactive_power_loop_360_50hz.h"
#include "trajectories/50hz/ltv/stored_ltv_reactive_power_loop_360_50hz.h"
static_assert(TINYMPC_REACTIVE_LOOP_SAMPLE_RATE_HZ == MPC_RATE,
              "power-loop reference and MPC rates must match");
static_assert(TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM == NSTATES + NINPUTS,
              "power-loop cache must include four rotor states");
static_assert(TINYMPC_REACTIVE_LOOP_STORED_LTV_DT_S == DT,
              "power-loop cache and controller periods must match");
static_assert(TINYMPC_REACTIVE_LOOP_STORED_LTV_RHO ==
                  TINYMPC_GENERATED_ADMM_RHO,
              "power-loop cache and controller rho must match");
static_assert(TINYMPC_REACTIVE_LOOP_STORED_LTV_INTERVALS + 1 ==
                  TINYMPC_REACTIVE_LOOP_SAMPLE_COUNT,
              "power-loop cache and reference lengths must match");
#endif

#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
static_assert(1000U % MPC_RATE == 0U,
              "direct plan replay requires an integral knot period");
#define TINYMPC_DIRECT_PLAN_INPUT_KNOTS (NHORIZON - 1U)
#define TINYMPC_DIRECT_PLAN_KNOT_TICKS M2T(1000U / MPC_RATE)
#endif
#if !defined(TRAJECTORY_TURN_DIRECTION)
/* +1 is a left/CCW curve, -1 is right/CW, and 0 is not prescribed. */
#define TRAJECTORY_TURN_DIRECTION 0
#endif
#if !defined(TRAJECTORY_OBSTACLE_APPROACH_WINDOW)
#define TRAJECTORY_OBSTACLE_APPROACH_WINDOW 0
#define TRAJECTORY_OBSTACLE_APPROACH_START_INDEX 0U
#define TRAJECTORY_OBSTACLE_INDEX 0U
#define TRAJECTORY_OBSTACLE_APPROACH_END_INDEX UINT32_MAX
#endif
#include "tinympc_level_actuator_lti.h"
#include "tinympc_banked_model_bank.h"
#if TINYMPC_RATE_CASCADE
#include "tinympc_outer_loop_model_bank.h"
static_assert(TINYMPC_OUTER_LOOP_STATE_DIM == NSTATES,
              "outer-loop model state dimension mismatch");
static_assert(TINYMPC_OUTER_LOOP_INPUT_DIM == NINPUTS,
              "outer-loop model input dimension mismatch");
static_assert(TINYMPC_OUTER_LOOP_MODEL_COUNT >=
                  TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH + 1,
              "outer-loop model bank must cover the 2.5m/s tier");
static_assert(TINYMPC_OUTER_LOOP_IDENTIFIED == 1,
              "outer-loop rate response must be identified");
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI)
static_assert(TINYMPC_LEVEL_ACTUATOR_STATE_DIM == NSTATES + NINPUTS,
              "level actuator model must add one state per motor");
static_assert(TINYMPC_LEVEL_ACTUATOR_DT_S == TINYMPC_GENERATED_MODEL_DT_S,
              "level actuator model and firmware periods differ");
static_assert(TINYMPC_LEVEL_ACTUATOR_RHO == TINYMPC_GENERATED_ADMM_RHO,
              "level actuator model and firmware ADMM rho differ");
static_assert(TINYMPC_BANK_MODEL_DT_S == TINYMPC_LEVEL_ACTUATOR_DT_S
                  && TINYMPC_BANK_MODEL_DT_S
                      == TINYMPC_GENERATED_MODEL_DT_S,
              "bank, level, and firmware model periods differ");
static_assert(TINYMPC_BANK_MODEL_RHO == TINYMPC_LEVEL_ACTUATOR_RHO
                  && TINYMPC_BANK_MODEL_RHO == TINYMPC_GENERATED_ADMM_RHO,
              "bank, level, and firmware ADMM rho differ");
static_assert(TINYMPC_GENERATED_MODEL_DT_S
                  == 1.0f / (float)TINYMPC_GENERATED_SOLVE_RATE_HZ,
              "generated model period and solve frequency differ");
static_assert(TINYMPC_BANK_MODEL_STATE_DIM == TINYMPC_LEVEL_ACTUATOR_STATE_DIM,
              "bank and level actuator state dimensions differ");
static_assert(TINYMPC_BANK_MODEL_INPUT_DIM == NINPUTS,
              "bank model input dimension mismatch");
static_assert(TINYMPC_BANK_MODEL_COUNT == 16,
              "bank selector and generated model count differ");
static_assert(TINYMPC_BANK_BUNDLE_ID_LEVEL == TINYMPC_BANK_MODEL_LEVEL
                  && TINYMPC_BANK_BUNDLE_ID_LEFT_LOW
                      == TINYMPC_BANK_MODEL_LEFT_LOW
                  && TINYMPC_BANK_BUNDLE_ID_RIGHT_LOW
                      == TINYMPC_BANK_MODEL_RIGHT_LOW
                  && TINYMPC_BANK_BUNDLE_ID_LEFT_MEDIUM
                      == TINYMPC_BANK_MODEL_LEFT_MEDIUM
                  && TINYMPC_BANK_BUNDLE_ID_RIGHT_MEDIUM
                      == TINYMPC_BANK_MODEL_RIGHT_MEDIUM
                  && TINYMPC_BANK_BUNDLE_ID_LEFT_HIGH
                      == TINYMPC_BANK_MODEL_LEFT_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_RIGHT_HIGH
                      == TINYMPC_BANK_MODEL_RIGHT_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_LEFT_VERY_HIGH
                      == TINYMPC_BANK_MODEL_LEFT_VERY_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_RIGHT_VERY_HIGH
                      == TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_LEFT_MAXIMUM
                      == TINYMPC_BANK_MODEL_LEFT_MAXIMUM
                  && TINYMPC_BANK_BUNDLE_ID_RIGHT_MAXIMUM
                      == TINYMPC_BANK_MODEL_RIGHT_MAXIMUM
                  && TINYMPC_BANK_BUNDLE_ID_BRAKE_LOW
                      == TINYMPC_BANK_MODEL_BRAKE_LOW
                  && TINYMPC_BANK_BUNDLE_ID_BRAKE_MEDIUM
                      == TINYMPC_BANK_MODEL_BRAKE_MEDIUM
                  && TINYMPC_BANK_BUNDLE_ID_BRAKE_HIGH
                      == TINYMPC_BANK_MODEL_BRAKE_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_BRAKE_VERY_HIGH
                      == TINYMPC_BANK_MODEL_BRAKE_VERY_HIGH
                  && TINYMPC_BANK_BUNDLE_ID_BRAKE_MAXIMUM
                      == TINYMPC_BANK_MODEL_BRAKE_MAXIMUM,
              "bank selector and generated bundle IDs differ");

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
static VectorNf progress_state_linear_cost[NHORIZON];
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
  250,
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
  /* The direct-motor controller repeatedly loses pitch authority on the
   * regenerated course after a rejoin when the reactive cruise ramp passes
   * about 1 m/s. Obstacle-course survival takes priority over the trajectory
   * speed ceiling; keep enough margin for braking and lateral recovery. */
  (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
      ? T_MIN((float)TINYMPC_PROGRESS_SPEED_MPS, 0.75f) : 0.50f,
#else
  (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
      ? (float)TINYMPC_PROGRESS_SPEED_MPS : 0.50f,
#endif
  2.09439510239f,
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
  /* Sector risk can flicker substantially while a post-avoidance vehicle
   * accelerates around the circle. Triggering remains based on fresh sector
   * samples below; filter only the continuous speed command so sub-threshold
   * perception noise cannot alternate hard acceleration and braking. */
  /* At the 30 Hz vision cadence this is a roughly 1.65 s low-pass. Raw risk
   * still triggers dodge/brake immediately; only the continuous TRACK speed
   * is prevented from following arena-texture flicker into pitch oscillation. */
  0.98f
#else
  0.30f
#endif
};
/* Spatial sector risk selects AVOID_LEFT/AVOID_RIGHT. Once selected, the
 * maneuver is committed: move to the route-specific lateral envelope, hold it
 * through measured forward clearance, then return slowly. TRACK resumes only
 * after measured lateral position and velocity have settled. */
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.50f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_HAIRPIN)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.35f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CIRCLE)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.50f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CHICANE)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.55f
#elif defined(TINYMPC_TRAJECTORY_CANONICAL_CORRIDOR)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.70f
#elif defined(TINYMPC_TRAJECTORY_DRONET_U)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.45f
#elif defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.50f
#elif defined(TINYMPC_VISION_ESPNET_DRONET_ENABLE)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 1.00f
#elif defined(TINYMPC_TRAJECTORY_CIRCLE)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.30f
#elif defined(TINYMPC_TRAJECTORY_FIGURE8)
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.35f
#elif TRAJECTORY_TANGENT_HEADING
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 0.30f
#else
#define TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M 1.15f
#endif
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
/* At cruise speed the direct-motor controller cannot safely combine the old
 * 4 m/s^2 lateral step with the simultaneous longitudinal slowdown. Keep the
 * full bypass displacement and early trigger, but stay inside the measured
 * lateral authority envelope. */
#define TINYRACER_DODGE_MINIMUM_LATERAL_RATE_MPS 0.35f
#define TINYRACER_DODGE_MAXIMUM_LATERAL_RATE_MPS 0.50f
#define TINYRACER_DODGE_LATERAL_ACCELERATION_MPS2 1.00f
#else
#define TINYRACER_DODGE_MINIMUM_LATERAL_RATE_MPS 1.05f
#define TINYRACER_DODGE_MAXIMUM_LATERAL_RATE_MPS 1.40f
#define TINYRACER_DODGE_LATERAL_ACCELERATION_MPS2 4.00f
#endif
#define TINYRACER_DODGE_REJOIN_LATERAL_RATE_MPS 0.06f
#define TINYRACER_DODGE_REJOIN_SPLINE_LENGTH_M 2.40f
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
#define TINYRACER_DODGE_TRIGGER_PROBABILITY 0.75f
#define TINYRACER_DODGE_TRIGGER_SAMPLES 2
#else
#define TINYRACER_DODGE_TRIGGER_PROBABILITY 0.80f
#define TINYRACER_DODGE_TRIGGER_SAMPLES 1
#endif
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
#define TINYRACER_DODGE_MINIMUM_LATERAL_OFFSET_M 0.40f
#else
#define TINYRACER_DODGE_MINIMUM_LATERAL_OFFSET_M 0.35f
#endif
#define TINYRACER_EMERGENCY_BRAKE_PROBABILITY 0.90f
#define TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2 \
  ((float)TINYMPC_PAPER_EMERGENCY_DECELERATION_MPS2)
#define TINYRACER_BACKTRACK_RELEASE_PROBABILITY 0.50f
#define TINYRACER_BACKTRACK_SPEED_MPS 0.50f
#define TINYRACER_BACKTRACK_SETTLE_MAXIMUM_TILT_RAD 0.34906585f
#define TINYRACER_BACKTRACK_SETTLE_MAXIMUM_FORWARD_SPEED_MPS 0.20f
#define TINYRACER_BACKTRACK_SETTLE_MAXIMUM_LATERAL_SPEED_MPS 0.20f
#define TINYRACER_BACKTRACK_SETTLE_MAXIMUM_VERTICAL_SPEED_MPS 0.20f
#define TINYRACER_BACKTRACK_SETTLE_MAXIMUM_BODY_RATE_RAD_S 0.75f
#define TINYRACER_RECOVERY_FORWARD_ACCELERATION_MPS2 1.00f
#define TINYRACER_DODGE_AVOID_FORWARD_SPEED_MPS 0.50f
#define TINYRACER_DODGE_HOLD_FORWARD_SPEED_MPS 0.50f
#define TINYRACER_DODGE_REJOIN_FORWARD_SPEED_MPS 0.50f
#define TINYRACER_DODGE_REARM_FORWARD_SPEED_MPS 0.50f
#define TINYRACER_DODGE_REDIRECT_FORWARD_SPEED_MPS 0.50f
#define TINYRACER_DODGE_HOLD_RELEASE_PROBABILITY 0.50f
#define TINYRACER_DODGE_HOLD_CLEAR_SAMPLES 3
#define TINYRACER_DODGE_REJOIN_ALIGNMENT_CLEAR_SAMPLES 3
#define TINYRACER_DODGE_MINIMUM_HOLD_PROGRESS_M 1.20f
#define TINYRACER_DODGE_MINIMUM_REARM_PROGRESS_M 0.60f
#define TINYRACER_DODGE_REJOIN_LATERAL_TOLERANCE_M 0.15f
#define TINYRACER_DODGE_REJOIN_LATERAL_SPEED_TOLERANCE_MPS 0.20f
#define TINYRACER_DODGE_REDIRECT_MAXIMUM_TILT_RAD 0.34906585f
#define TINYRACER_DODGE_REDIRECT_MAXIMUM_FORWARD_SPEED_MPS 0.75f
#define TINYRACER_DODGE_REDIRECT_MAXIMUM_LATERAL_SPEED_MPS 0.25f
#define TINYRACER_DODGE_REDIRECT_MINIMUM_VERTICAL_SPEED_MPS (-0.30f)
#define TINYRACER_DODGE_REJOIN_SETTLE_SAMPLES 5
#define TINYRACER_DODGE_REARM_CLEAR_SAMPLES 5
#define TINYRACER_DODGE_REDIRECT_TRIGGER_SAMPLES 1
#define TINYRACER_DODGE_REDIRECT_SETTLE_SAMPLES 5
#define TINYRACER_BACKTRACK_SETTLE_SAMPLES 5
#define TINYRACER_DODGE_REJOIN_ALIGNMENT_HEADING_TOLERANCE_RAD 0.20943951f
#define TINYRACER_LOOP_SCAN_YAW_RAD 0.78539816f
#define TINYRACER_LOOP_SCAN_SAMPLES 2
#define TINYRACER_LOOP_ESCAPE_LATERAL_STEP_M 0.35f
#define TINYRACER_LOOP_ESCAPE_MAXIMUM_OFFSET_M 0.70f
#define TINYRACER_LOOP_ESCAPE_SPLINE_LENGTH_M 2.40f
#if defined(TINYMPC_USE_ACTUATOR_LTI) && TINYMPC_BRAKING_CACHE_ENABLE
static_assert(
    TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2 ==
        TINYMPC_BRAKING_DECELERATION_MPS2,
    "emergency-brake state and aggressive braking cache must share deceleration");
#endif
#if defined(TINYMPC_TRAJECTORY_DRONET_U) && TINYMPC_PATH_TUNNEL_ENABLE
static_assert(
    (float)TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M +
        (float)TINYMPC_PATH_TUNNEL_HALF_WIDTH_1_M <= 0.80f,
    "shifted U-course tracking tube must remain inside the eroded corridor");
#endif
static const TinyRacerDodgeConfig dodge_config = {
  TINYRACER_DODGE_TRIGGER_PROBABILITY,
  TINYRACER_DODGE_MINIMUM_LATERAL_RATE_MPS,
  TINYRACER_DODGE_MAXIMUM_LATERAL_RATE_MPS,
  TINYRACER_DODGE_REJOIN_LATERAL_RATE_MPS,
  TINYRACER_DODGE_REJOIN_SPLINE_LENGTH_M,
  TINYRACER_DODGE_LATERAL_ACCELERATION_MPS2,
  TINYRACER_DODGE_MAXIMUM_LATERAL_OFFSET_M,
  TINYRACER_DODGE_MINIMUM_LATERAL_OFFSET_M,
  TINYRACER_EMERGENCY_BRAKE_PROBABILITY,
  TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2,
  TINYRACER_BACKTRACK_RELEASE_PROBABILITY,
  TINYRACER_BACKTRACK_SPEED_MPS,
  TINYRACER_BACKTRACK_SETTLE_MAXIMUM_TILT_RAD,
  TINYRACER_BACKTRACK_SETTLE_MAXIMUM_FORWARD_SPEED_MPS,
  TINYRACER_BACKTRACK_SETTLE_MAXIMUM_LATERAL_SPEED_MPS,
  TINYRACER_BACKTRACK_SETTLE_MAXIMUM_VERTICAL_SPEED_MPS,
  TINYRACER_BACKTRACK_SETTLE_MAXIMUM_BODY_RATE_RAD_S,
  TINYRACER_RECOVERY_FORWARD_ACCELERATION_MPS2,
  TINYRACER_DODGE_AVOID_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_HOLD_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_REJOIN_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_REARM_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_REDIRECT_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_HOLD_RELEASE_PROBABILITY,
  TINYRACER_DODGE_MINIMUM_HOLD_PROGRESS_M,
  TINYRACER_DODGE_MINIMUM_REARM_PROGRESS_M,
  TINYRACER_DODGE_REJOIN_LATERAL_TOLERANCE_M,
  TINYRACER_DODGE_REJOIN_LATERAL_SPEED_TOLERANCE_MPS,
  TINYRACER_DODGE_REDIRECT_MAXIMUM_TILT_RAD,
  TINYRACER_DODGE_REDIRECT_MAXIMUM_FORWARD_SPEED_MPS,
  TINYRACER_DODGE_REDIRECT_MAXIMUM_LATERAL_SPEED_MPS,
  TINYRACER_DODGE_REDIRECT_MINIMUM_VERTICAL_SPEED_MPS,
  TINYRACER_DODGE_TRIGGER_SAMPLES,
  TINYRACER_DODGE_REDIRECT_TRIGGER_SAMPLES,
  TINYRACER_DODGE_REDIRECT_SETTLE_SAMPLES,
  TINYRACER_BACKTRACK_SETTLE_SAMPLES,
  TINYRACER_DODGE_HOLD_CLEAR_SAMPLES,
  TINYRACER_DODGE_REJOIN_SETTLE_SAMPLES,
  TINYRACER_DODGE_REARM_CLEAR_SAMPLES,
  TINYRACER_DODGE_REJOIN_ALIGNMENT_HEADING_TOLERANCE_RAD,
  TINYRACER_DODGE_REJOIN_ALIGNMENT_CLEAR_SAMPLES,
  TINYRACER_LOOP_SCAN_YAW_RAD,
  TINYRACER_LOOP_SCAN_SAMPLES,
  TINYRACER_LOOP_ESCAPE_LATERAL_STEP_M,
  TINYRACER_LOOP_ESCAPE_MAXIMUM_OFFSET_M,
  TINYRACER_LOOP_ESCAPE_SPLINE_LENGTH_M
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
#if TINYMPC_GATE_OLGMD_ENABLE
static TinyMpcGateOlgmdState gate_olgmd_state;
static uint32_t gate_olgmd_trigger_count = 0u;
static uint32_t gate_olgmd_release_count = 0u;
static bool gate_olgmd_near_gate_suppression = false;
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
static uint32_t gate_olgmd_stop_hold_cycles = 0u;
static uint32_t gate_olgmd_resume_refractory_cycles = 0u;
#endif
#endif
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
/* The controller-ablation emergency arm is a one-shot safety action, not an
 * obstacle-avoidance encounter.  Once asserted it may brake and settle, but
 * it must never backtrack, scan, or resume the nominal approach. */
static bool paper_emergency_terminal_stop_latched = false;
static float paper_emergency_measured_forward_speed_mps = 0.0f;
static bool paper_emergency_position_velocity_active = false;
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
static bool pitch_through_brake_reference_active = false;
static float pitch_through_brake_entry_altitude_world_m = 0.0f;
static float pitch_through_brake_internal_up_position_m = 0.0f;
static float pitch_through_brake_internal_up_speed_mps = 0.0f;
static uint32_t pitch_through_brake_trigger_count = 0u;
static const TinyMpcPitchThroughBrakeConfig pitch_through_brake_config = {
    0.87266463f,  /* 50 deg at and below 1 m/s. */
    1.13446401f,  /* 65 deg at and above 4 m/s. */
    1.0f,
    4.0f,
    4.5f,         /* 258 deg/s remains below ordinary Crazyflie rate limits. */
    60.0f,
    0.80f,        /* Begin counter-rotation while thrust still brakes. */
    4.0f,
    6.0f,
    18.0f,
    6.0f,
    5.0f,
    0.20f,        /* Spend at most 20 cm of altitude before recovery. */
    0.04f,
    TINYMPC_BANK_MODEL_DRAG_X_N_PER_MPS,
    TINYMPC_BANK_MODEL_MASS_KG,
    9.81f,
    TINYMPC_LEVEL_MAX_MOTOR_THRUST_N,
    28.0e-6f,
    0.035355f,
};
#endif
#endif
static bool dodge_camera_yaw_initialized = false;
static float dodge_camera_yaw_world_rad = 0.0f;
static bool dodge_rejoin_target_yaw_valid = false;
static float dodge_rejoin_target_yaw_world_rad = 0.0f;
static bool dodge_loop_scan_route_yaw_valid = false;
static float dodge_loop_scan_route_yaw_world_rad = 0.0f;
static bool dodge_backtrack_settle_anchor_valid = false;
static Eigen::Vector3f dodge_backtrack_settle_anchor_world =
    Eigen::Vector3f::Zero();
static uint16_t navigation_warmup_steps = 0;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
static bool gate_poc_associated = false;
/* The joint proof-of-concept acceptance courses contain one physical gate.
 * Once that gate's retained association expires, do not let rear views or a
 * later gate-like obstacle steal authority back from obstacle avoidance. */
static bool gate_poc_completed = false;
static uint8_t gate_poc_consecutive_samples = 0u;
static uint8_t gate_poc_dropout_steps = 0u;
static uint8_t gate_poc_geometry_dropout_samples = UINT8_MAX;
static uint16_t gate_poc_association_samples = 0u;
static uint32_t gate_poc_last_sample = UINT32_MAX;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
typedef enum {
  GATE_VISUAL_SEARCH = 0,
  GATE_VISUAL_ALIGN,
  GATE_VISUAL_TRANSIT,
  GATE_VISUAL_REARM,
} GateVisualPhase;
static GateVisualPhase gate_visual_phase = GATE_VISUAL_SEARCH;
static Eigen::Vector3f gate_visual_center_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f gate_visual_forward_world = Eigen::Vector3f::UnitX();
static Eigen::Vector3f gate_visual_velocity_world = Eigen::Vector3f::Zero();
static bool gate_visual_center_valid = false;
static bool gate_visual_velocity_valid = false;
static uint8_t gate_visual_rearm_clear_samples = 0u;
#endif
#if TINYMPC_JOINT_GATE_RL_ENABLE
/* A 20-frame geometry-outage bound bridges only a short near-plane gap. A
 * separate monotonic 300-sample (10 s at 30 Hz) lifetime prevents continuous
 * gate-like geometry from extending either behavior for the whole flight. */
static constexpr uint8_t joint_gate_maximum_geometry_dropout_samples = 20u;
static constexpr uint16_t joint_gate_maximum_association_samples = 300u;
#endif
#endif
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
static bool gate_poc_collision_suppressed = false;
#endif
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
static bool figure8_midcourse_dual_reset_complete = false;
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI)
static void resetLevelActuatorDuals(void);
#endif
#if TINYMPC_RATE_CASCADE
static void resetOuterLoopDuals(void);
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

#if !defined(TINYMPC_PROGRESS_SAMPLE_LIMIT)
#define TINYMPC_PROGRESS_SAMPLE_LIMIT 0
#endif
#if !defined(TINYMPC_PROGRESS_SPEED_MPS)
#define TINYMPC_PROGRESS_SPEED_MPS 0.0f
#endif
#if !defined(TINYMPC_PROGRESS_LAPS)
#define TINYMPC_PROGRESS_LAPS 1
#endif
#if !defined(TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2)
#define TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2 1.0f
#endif
#if !defined(TINYMPC_PROGRESS_REWARD_WEIGHT)
#define TINYMPC_PROGRESS_REWARD_WEIGHT 0.40f
#endif
#if !defined(TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2)
#define TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2 6.0f
#endif
#if !defined(TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2)
#define TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2 6.0f
#endif
#if !defined(TINYMPC_FLIP_ENABLE)
#define TINYMPC_FLIP_ENABLE 0
#endif
#if !defined(TINYMPC_FLIP_TRIGGER_S_M)
#define TINYMPC_FLIP_TRIGGER_S_M 1.0f
#endif
#if !defined(TINYMPC_FLIP_TRIGGER_WINDOW_M)
#define TINYMPC_FLIP_TRIGGER_WINDOW_M 0.25f
#endif
#if !defined(TINYMPC_FLIP_DURATION_S)
#define TINYMPC_FLIP_DURATION_S 0.8f
#endif
#if !defined(TINYMPC_FLIP_PITCH_DIRECTION)
#define TINYMPC_FLIP_PITCH_DIRECTION 1
#endif
#if !defined(TINYMPC_POWER_LOOP_ENABLE)
#define TINYMPC_POWER_LOOP_ENABLE 0
#endif
#if !defined(TINYMPC_POWER_LOOP_TRIGGER_S_M)
#define TINYMPC_POWER_LOOP_TRIGGER_S_M 1.0f
#endif
#if !defined(TINYMPC_POWER_LOOP_TRIGGER_WINDOW_M)
#define TINYMPC_POWER_LOOP_TRIGGER_WINDOW_M 0.25f
#endif
#if !defined(TINYMPC_POWER_LOOP_RADIUS_M)
#define TINYMPC_POWER_LOOP_RADIUS_M 1.2f
#endif
#if !defined(TINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS)
#define TINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS 2.2f
#endif
#if !defined(TINYMPC_POWER_LOOP_TOP_SPEED_MPS)
#define TINYMPC_POWER_LOOP_TOP_SPEED_MPS 4.0f
#endif
static_assert(TINYMPC_PROGRESS_SAMPLE_LIMIT == 0 ||
              TINYMPC_PROGRESS_SAMPLE_LIMIT >= 2,
              "progress sample limit must be zero (full route) or at least 2");
static_assert(TINYMPC_PROGRESS_SAMPLE_LIMIT == 0 ||
              TINYMPC_PROGRESS_SAMPLE_LIMIT <= TRAJECTORY_SAMPLE_COUNT,
              "progress sample limit exceeds compiled trajectory");
static const uint32_t progress_sample_count =
    TINYMPC_PROGRESS_SAMPLE_LIMIT == 0
    ? TRAJECTORY_SAMPLE_COUNT : TINYMPC_PROGRESS_SAMPLE_LIMIT;
static_assert((float)TINYMPC_PROGRESS_SPEED_MPS >= 0.0f,
              "progress speed must be zero (curvature schedule) or positive");
static_assert(TINYMPC_PROGRESS_LAPS >= 1 && TINYMPC_PROGRESS_LAPS <= 8,
              "progress laps must be between one and eight");
static_assert((float)TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2 > 0.0f,
              "progress entry acceleration must be positive");
static_assert((float)TINYMPC_PROGRESS_REWARD_WEIGHT >= 0.0f,
              "progress reward weight must be nonnegative");
static_assert((float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2 > 0.0f,
              "terminal deceleration must be positive");
static_assert(
    (float)TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2 > 0.0f,
    "maximum centripetal acceleration must be positive");
static_assert(TINYMPC_FLIP_ENABLE == 0 || TINYMPC_FLIP_ENABLE == 1,
              "flip enable must be zero or one");
static_assert((float)TINYMPC_FLIP_TRIGGER_S_M >= 0.0f,
              "flip trigger progress must be nonnegative");
static_assert((float)TINYMPC_FLIP_TRIGGER_WINDOW_M > 0.0f,
              "flip trigger window must be positive");
static_assert((float)TINYMPC_FLIP_DURATION_S > 0.0f,
              "flip duration must be positive");
static_assert(TINYMPC_FLIP_PITCH_DIRECTION == -1 ||
              TINYMPC_FLIP_PITCH_DIRECTION == 1,
              "flip pitch direction must be -1 or 1");
#if TINYMPC_FLIP_ENABLE && !defined(TINYMPC_USE_ACTUATOR_LTI)
#error "flip primitive requires the actuator-LTI relative-error solver"
#endif
static_assert(TINYMPC_POWER_LOOP_ENABLE == 0 || TINYMPC_POWER_LOOP_ENABLE == 1,
              "power-loop enable must be zero or one");
static_assert((float)TINYMPC_POWER_LOOP_TRIGGER_S_M >= 0.0f,
              "power-loop trigger progress must be nonnegative");
static_assert((float)TINYMPC_POWER_LOOP_TRIGGER_WINDOW_M > 0.0f,
              "power-loop trigger window must be positive");
static_assert((float)TINYMPC_POWER_LOOP_RADIUS_M > 0.0f,
              "power-loop radius must be positive");
static_assert((float)TINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS > 0.0f,
              "power-loop bottom speed must be positive");
static_assert((float)TINYMPC_POWER_LOOP_TOP_SPEED_MPS >
                  (float)TINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS,
              "power-loop top speed must exceed bottom speed");
static_assert(!(TINYMPC_FLIP_ENABLE && TINYMPC_POWER_LOOP_ENABLE),
              "flip and power loop are mutually exclusive");
#if TINYMPC_POWER_LOOP_ENABLE && !defined(TINYMPC_USE_ACTUATOR_LTI)
#error "power loop requires the actuator-LTI relative-error solver"
#endif
#if TINYMPC_PROGRESS_LAPS > 1 && \
    !defined(TINYMPC_TRAJECTORY_CIRCLE) && \
    !defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
#error "repeated progress laps are only supported for closed circle experiments"
#endif
static constexpr float progress_minimum_speed_mps =
    (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
        ? (float)TINYMPC_PROGRESS_SPEED_MPS : 0.05f;
static constexpr float progress_maximum_speed_mps =
    (float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f
        ? (float)TINYMPC_PROGRESS_SPEED_MPS : 0.15f;
#if defined(TINYMPC_TRAJECTORY_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
/* Preserve racing semantics: report the requested finish at full speed, then
 * use one hidden closed-path lap to unload speed, curvature bank, and thrust
 * continuously before entering the endpoint hold. */
static constexpr uint16_t progress_control_lap_count =
    (uint16_t)TINYMPC_PROGRESS_LAPS + 1u;
#else
static constexpr uint16_t progress_control_lap_count =
    (uint16_t)TINYMPC_PROGRESS_LAPS;
#endif

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(
    TRAJECTORY_REFERENCE_DIM == 13,
    "trajectory reference dimension mismatch");

static struct quat attitude;
static MpcLocalFrame active_local_frame;
static Eigen::Vector3f position_delta_velocity_world =
    Eigen::Vector3f::Zero();
static Eigen::Vector3f previous_solve_position_world =
    Eigen::Vector3f::Zero();
static bool position_delta_velocity_valid = false;
static Eigen::Vector3f trajectory_origin_world = Eigen::Vector3f::Zero();
static float trajectory_cos_yaw = 1.0f;
static float trajectory_sin_yaw = 0.0f;
static struct quat trajectory_yaw_rotation = qeye();
static uint16_t trajectory_handoff_hold_steps = 0;
static uint32_t vertical_active_sensing_start_tick = 0u;
#if TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE
static_assert(
    (float)TINYMPC_VERTICAL_ACTIVE_SENSING_AMPLITUDE_M >= 0.0f,
    "vertical active-sensing amplitude must be nonnegative");
static_assert(
    (float)TINYMPC_VERTICAL_ACTIVE_SENSING_PERIOD_S > 0.0f,
    "vertical active-sensing period must be positive");
static const TinyMpcVerticalActiveSensingConfig
    vertical_active_sensing_config = {
        (float)TINYMPC_VERTICAL_ACTIVE_SENSING_AMPLITUDE_M,
        (float)TINYMPC_VERTICAL_ACTIVE_SENSING_PERIOD_S,
    };
#endif
static float reference_yaw_unwrapped_rad[NHORIZON];
static float reference_yaw_phase_rad = 0.0f;
#if TINYMPC_YAW_SPIN_TEST_ENABLE
static Eigen::Vector3f yaw_spin_test_anchor_world = Eigen::Vector3f::Zero();
static float yaw_spin_test_initial_yaw_rad = 0.0f;
static uint32_t yaw_spin_test_step = 0u;
static uint8_t yaw_spin_test_reported_revolutions = 0u;
static uint8_t yaw_spin_test_last_phase = UINT8_MAX;
static bool yaw_spin_test_complete_reported = false;
#endif
static bool progress_heading_alignment_active = false;
static bool progress_heading_alignment_complete = false;
static TinyMpcProgressPath progress_path;
static TinyMpcFlipState flip_state;
static bool flip_reference_active = false;
static bool flip_recovery_active = false;
static uint16_t flip_recovery_settle_steps = 0u;
static uint32_t flip_recovery_elapsed_steps = 0u;
static TinyMpcFlipInputKnot flip_input_knots[5];
static TinyMpcFlipConfig flip_config = {
    (float)TINYMPC_FLIP_TRIGGER_S_M,
    (float)TINYMPC_FLIP_TRIGGER_S_M + (float)TINYMPC_FLIP_TRIGGER_WINDOW_M,
    (float)TINYMPC_FLIP_DURATION_S,
    (int8_t)TINYMPC_FLIP_PITCH_DIRECTION,
    flip_input_knots,
    5u,
    NULL,
    0u,
};
static TinyMpcPowerLoopState power_loop_state;
static bool power_loop_reference_active = false;
static bool power_loop_recovery_active = false;
static uint16_t power_loop_recovery_settle_steps = 0u;
static uint32_t power_loop_recovery_elapsed_steps = 0u;
static Eigen::Vector3f power_loop_anchor_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f power_loop_recovery_anchor_world =
    Eigen::Vector3f::Zero();
static float power_loop_yaw_world_rad = 0.0f;
static Eigen::Vector3f power_loop_forward_world = Eigen::Vector3f::UnitX();
static TinyMpcPowerLoopConfig power_loop_config = {
    (float)TINYMPC_POWER_LOOP_TRIGGER_S_M,
    (float)TINYMPC_POWER_LOOP_TRIGGER_S_M
        + (float)TINYMPC_POWER_LOOP_TRIGGER_WINDOW_M,
    (float)TINYMPC_POWER_LOOP_RADIUS_M,
    (float)TINYMPC_POWER_LOOP_BOTTOM_SPEED_MPS,
    (float)TINYMPC_POWER_LOOP_TOP_SPEED_MPS,
    TINYMPC_BANK_MODEL_MASS_KG,
    9.81f,
    TINYMPC_LEVEL_MAX_MOTOR_THRUST_N,
    28.0e-6f,
    0.035355f,
};
static bool progress_completion_reported = false;
static float progress_reference_speed_mps = 0.0f;
static float progress_reference_acceleration_mps2 = 0.0f;
static float vision_track_speed_limit_mps = INFINITY;
#if TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
static PulpDronetV2Brake dronet_v2_brake = pulpDronetV2BrakeDefault();
static float dronet_v2_speed_scale = 0.0f;
static uint32_t dronet_v2_last_sample = 0u;
#endif
#if TINYMPC_VISION_RL_RESIDUAL_ENABLE
static float vision_residual_lateral_offset_m = 0.0f;
static float vision_residual_vertical_offset_m = 0.0f;
static float vision_residual_progress_speed_scale = 1.0f;
static const TinyMpcVisionResidualAuthority vision_residual_authority =
    tinyMpcVisionResidualFullAuthorityV1();
#endif
static uint32_t progress_invariant_diag_cycle = 0u;
static uint32_t progress_body_rate_reconstruction_violation_count = 0u;
static uint16_t progress_last_reported_measured_lap = 0u;
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
static float progress_curvature_speed_envelope_mps[TRAJECTORY_SAMPLE_COUNT];
static float levelStateBankRad(const VectorNf& state);
static float levelStatePitchRad(const VectorNf& state);
static float levelStateYawRad(const VectorNf& state);

static void buildProgressCurvatureSpeedEnvelope(void) {
  const float proposed_speed_mps = progress_maximum_speed_mps
      + (float)TINYMPC_PROGRESS_REWARD_WEIGHT
          / tinympc_generated_Q_diagonal[6];
  constexpr float recovery_distance_m = 2.5f;
  for (uint32_t index = 0u; index < progress_path.count; ++index) {
    float limit_mps = proposed_speed_mps;
    float distance_m = 0.0f;
    uint32_t scan = index;
    while (true) {
      const TinyMpcPathSample sample = tinyMpcProgressPathSample(
          &progress_path, (float)scan);
      if (sample.curvature_magnitude_per_m > 1.0e-6f) {
        limit_mps = T_MIN(limit_mps, sqrtf(
            (float)TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2
            / sample.curvature_magnitude_per_m));
      }
      if (scan == 0u || distance_m >= recovery_distance_m) {
        break;
      }
      --scan;
      distance_m += tinyMpcPathSegmentLength(&progress_path, scan);
    }
    progress_curvature_speed_envelope_mps[index] = limit_mps;
  }
  constexpr float braking_deceleration_mps2 =
      (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2;
  for (uint32_t index = progress_path.count - 1u; index > 0u; --index) {
    const float distance_m = tinyMpcPathSegmentLength(
        &progress_path, index - 1u);
    const float next_limit_mps =
        progress_curvature_speed_envelope_mps[index];
    const float braking_limit_mps = sqrtf(
        next_limit_mps * next_limit_mps
        + 2.0f * braking_deceleration_mps2 * distance_m);
    progress_curvature_speed_envelope_mps[index - 1u] = T_MIN(
        progress_curvature_speed_envelope_mps[index - 1u],
        braking_limit_mps);
  }
}

static float progressCurvatureEnvelopeScale(
    float progress, float proposed_speed_mps) {
  if (!isfinite(progress) || !isfinite(proposed_speed_mps)
      || proposed_speed_mps <= 0.0f || progress_path.count < 2u) {
    return 0.0f;
  }
  const float segments_per_lap = (float)(progress_path.count - 1u);
  float source_progress = fmodf(T_MAX(progress, 0.0f), segments_per_lap);
  if (progress >= (float)(progress_path.virtual_count - 1u)) {
    source_progress = segments_per_lap;
  }
  const uint32_t lower = (uint32_t)floorf(source_progress);
  const uint32_t upper = T_MIN(lower + 1u, progress_path.count - 1u);
  const float alpha = source_progress - (float)lower;
  const float limit_mps =
      (1.0f - alpha) * progress_curvature_speed_envelope_mps[lower]
      + alpha * progress_curvature_speed_envelope_mps[upper];
  return tinyMpcPathClamp(limit_mps / proposed_speed_mps, 0.0f, 1.0f);
}

static void configureFlipInputPrimitive(void) {
  constexpr float hover_n = TINYMPC_LEVEL_HOVER_THRUST_N;
  /* The first flight reached 34.6 rad/s against a 14.7 rad/s peak reference.
   * Reduce the open-loop pitch moment; feedback still tracks omega_ref. */
  constexpr float pitch_differential_n = 0.020f;
  constexpr float inverted_collective_n = 0.025f;
  const float direction = (float)TINYMPC_FLIP_PITCH_DIRECTION;
  const float accelerate_14 = hover_n - direction * pitch_differential_n;
  const float accelerate_23 = hover_n + direction * pitch_differential_n;
  const float brake_14 = hover_n + direction * pitch_differential_n;
  const float brake_23 = hover_n - direction * pitch_differential_n;
  flip_input_knots[0] = {0.0f, {hover_n, hover_n, hover_n, hover_n}};
  flip_input_knots[1] = {
      0.20f, {accelerate_14, accelerate_23,
              accelerate_23, accelerate_14}};
  flip_input_knots[2] = {
      0.50f, {inverted_collective_n, inverted_collective_n,
              inverted_collective_n, inverted_collective_n}};
  flip_input_knots[3] = {
      0.80f, {brake_14, brake_23, brake_23, brake_14}};
  flip_input_knots[4] = {1.0f, {hover_n, hover_n, hover_n, hover_n}};
}

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

#if TINYMPC_RATE_CASCADE
static const TinyMpcOuterLoopModelData *active_outer_loop_model =
    &tinympc_outer_loop_models[TINYMPC_BANK_MODEL_LEVEL];
static VectorNf outer_Xhrz[NHORIZON];
static VectorNf outer_p[NHORIZON];
static VectorNf outer_ZX_new[NHORIZON];
static VectorNf outer_YX[NHORIZON];
static VectorMf outer_Uref[NHORIZON - 1];
static VectorMf outer_Uhrz[NHORIZON - 1];
static VectorMf outer_d[NHORIZON - 1];
static VectorMf outer_ZU_new[NHORIZON - 1];
static VectorMf outer_YU[NHORIZON - 1];
static const TinyMpcBankSelectorConfig outer_bank_selector_config = {
    TINYMPC_BANK_MODEL_RIGHT_LOW_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_MEDIUM_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_HIGH_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_MAXIMUM_ROLL_RAD,
    0.067f, 0.045f, 0.020f, 5u,
};
static TinyMpcBankSelector outer_bank_selector;
static TinyMpcBankSelection outer_bank_selection = {
    TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
static float outer_warm_start_yaw_rad = 0.0f;
static bool outer_warm_start_yaw_initialized = false;
static bool outer_frame_reseed_requested = false;
#if defined(CONFIG_PLATFORM_SITL)
/* Emit a short, solve-rate trace around each bundle handoff.  The ordinary
 * 10 Hz rate-loop trace shows the downstream command but cannot distinguish a
 * bad chart state from an ADMM projection transient. */
static uint8_t outer_model_transition_diag_steps = 0u;
#endif
/* First-action slew constraints make model-bundle changes bumpless at 50 Hz.
 * They constrain the actual outer input seen by the identified closed-loop
 * plant, not a post-solve command that the optimizer cannot observe. */
static const float outer_input_slew_per_solve[NINPUTS] = {
  0.025f, 0.25f, 0.25f, 0.30f,
};
/* The identified rate-response data only exercised approximately
 * +/-0.30rad/s roll/pitch and +/-0.61rad/s yaw around its feedforward. Keep
 * ordinary racing inside that measured correction envelope. The separately
 * guarded flip/power-loop path retains its explicit high-rate allowance. */
static const float outer_tracking_trim_limit[NINPUTS] = {
  0.08f, 0.35f, 0.35f, 0.65f,
};
/* The stored flip/power-loop primitive previously needed body rates well above
 * the small-signal racing envelope (the golden direct-MPC loop reached about
 * 16.5 rad/s).  Keep ordinary banked racing inside the identified +/-6 rad/s
 * range, but let the optimizer request the physically necessary maneuver rate
 * through the same 500 Hz stock rate PID. */
static constexpr float outer_maneuver_roll_pitch_rate_limit_rad_s = 18.0f;
static constexpr float outer_maneuver_rate_slew_per_solve_rad_s = 0.75f;
/* A single level linearization cannot supply trustworthy large corrections at
 * the pi-radian chart singularity.  During a stored maneuver primitive, retain
 * its phase-dependent thrust/rate feedforward and allow only local MPC trim.
 * Ordinary racing continues to publish the unrestricted optimized action. */
static const float outer_maneuver_trim_limit[NINPUTS] = {
  0.08f, 1.0f, 1.0f, 1.0f,
};
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI)
static float level_motor_rotor_state_estimate[NINPUTS] = {0.0f};
static float planner_level_motor_rotor_state_estimate[NINPUTS] = {0.0f};
static VectorMf level_motor_rotor_state_snapshot;
static float level_warm_start_yaw_rad = 0.0f;
static bool level_warm_start_yaw_initialized = false;
static bool level_frame_reseed_requested = false;
static const TinyMpcBankSelectorConfig level_bank_selector_config = {
    TINYMPC_BANK_MODEL_RIGHT_LOW_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_MEDIUM_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_HIGH_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH_ROLL_RAD,
    TINYMPC_BANK_MODEL_RIGHT_MAXIMUM_ROLL_RAD,
    0.067f,  // Enter a banked chart above 3.84 deg.
    0.045f,  // Return to level below 2.58 deg.
    0.020f,  // Keep 1.15 deg of hysteresis at tier boundaries.
    5u,      // Hold a selected bundle for at least 100 ms at 50 Hz.
};
static TinyMpcBankSelector level_bank_selector;
static TinyMpcBankSelection level_bank_selection = {
    TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
static const TinyMpcBrakingSelectorConfig level_braking_selector_config = {
    5.5f,  // Enter only the explicit 6 m/s^2 braking schedule.
    0.5f,  // Release once the reference is no longer materially braking.
    0.10f, // Require at least 5.73 deg of negative reference pitch.
    {TINYMPC_BANK_MODEL_BRAKE_LOW_PITCH_RAD,
     TINYMPC_BANK_MODEL_BRAKE_MEDIUM_PITCH_RAD,
     TINYMPC_BANK_MODEL_BRAKE_HIGH_PITCH_RAD,
     TINYMPC_BANK_MODEL_BRAKE_VERY_HIGH_PITCH_RAD,
     TINYMPC_BANK_MODEL_BRAKE_MAXIMUM_PITCH_RAD},
    0.12f, // Pitch-only banks are restricted to near-level roll.
    0.26f, // Cover the complete nearest-neighbor 0.5 m/s speed spacing.
    0.12f, // Enter after measured pitch is within 6.88 deg of reference.
    0.05f, // Avoid chatter at speed-tier midpoints.
    5u,    // Hold each cache for at least 100 ms at 50 Hz.
};
static TinyMpcBrakingSelector level_braking_selector;
static TinyMpcBrakingSelection level_braking_selection = {
    TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
static bool level_model_switched_this_solve = false;
#endif

static bool mpc_has_run = false;
static uint32_t last_controller_tick = 0;
static uint32_t plan_start_tick = 0;
static bool motors_were_allowed = false;
static float active_motor_commands[NINPUTS];
#if TINYMPC_RATE_CASCADE
typedef struct {
  float collective_thrust_n;
  float body_rate_rad_s[3];
} TinyMpcRateCommand;
static TinyMpcRateCommand active_rate_command = {};
static TinyMpcRateCommand cached_rate_command = {};
static bool cached_rate_command_valid = false;
static uint32_t cached_rate_command_tick = 0u;
static bool rate_pid_was_tracking_fresh_command = false;
/* The MPC task raises a pending reset at maneuver authority boundaries.  It is
 * transferred with the next published command and consumed exactly once by
 * the 500 Hz callback under dataMutex. */
static bool planner_rate_pid_reset_pending = false;
static bool active_rate_pid_reset_requested = false;

static float hoverCollectiveThrustN(void) {
  float collective_thrust_n = 0.0f;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    collective_thrust_n += tinympc_generated_physical_hover_thrust[motor];
  }
  return collective_thrust_n;
}

static float collectiveThrustToLegacyCommand(float collective_thrust_n) {
  const float per_motor_thrust_n = T_MAX(
      collective_thrust_n / (float)STABILIZER_NR_OF_MOTORS, 0.0f);
  const float normalized_command = T_MIN(T_MAX(
      tinympc_generated_thrust_to_normalized_command(per_motor_thrust_n),
      0.0f), 1.0f);
  return normalized_command * (float)UINT16_MAX;
}
#endif
#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
static float active_motor_plan[TINYMPC_DIRECT_PLAN_INPUT_KNOTS][NINPUTS];
static bool cached_motor_plan_valid = false;
static uint32_t cached_motor_plan_tick = 0u;
static float cached_motor_commands[NINPUTS] = {0.0f};
#else
/* The 500 Hz callback must not replace a fresh MPC command with hover merely
 * because the worker holds dataMutex during a nonblocking read attempt.  This
 * callback-owned cache is refreshed only after a coherent mutex-protected
 * snapshot and remains subject to the normal command-age safety limit. */
static bool cached_motor_command_valid = false;
static uint32_t cached_motor_command_tick = 0u;
static float cached_motor_command[NINPUTS] = {0.0f};
#endif
static bool solve_tick_timing_initialized = false;
static uint32_t previous_solve_tick = 0u;
static uint32_t solve_tick_gap_min = UINT32_MAX;
static uint32_t solve_tick_gap_max = 0u;
static uint32_t solve_tick_gap_samples = 0u;
static uint32_t solve_tick_skipped_periods = 0u;
static SemaphoreHandle_t runTaskSemaphore = NULL;
static SemaphoreHandle_t dataMutex = NULL;
static StaticSemaphore_t dataMutexBuffer;
static setpoint_t planner_setpoint;
static sensorData_t planner_sensors;
static state_t planner_state;
static uint32_t planner_tick = 0;
static bool planner_reset_requested = false;
#if defined(CONFIG_PLATFORM_SITL)
/*
 * Binary, append-only diagnostic stream. Initialization exclusively creates,
 * sizes, maps, and pre-touches the file before controller tasks start. Runtime
 * producers reserve one no-wrap mmap slot with a lock-free atomic and publish
 * checksum-protected records by release-committing the slot. There is no
 * diagnostic task, critical section, allocation, syscall, or runtime flush.
 *
 * Schema version 2, native little-endian CrazySim host representation.
 * event: 0=stream header, 1=solve, 2=fallback transition.
 * fallback_reason: 0=fresh, 1=motors disabled, 2=no completed plan,
 *                  3=stale plan, 4=non-finite motor command.
 */
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t record_size;
  uint32_t event;
  uint32_t event_sequence;
  uint32_t release_sequence;
  uint32_t solve_sequence;
  uint32_t release_tick;
  uint32_t start_tick;
  uint32_t finish_tick;
  uint32_t plan_tick;
  uint32_t plan_age_ticks;
  uint32_t release_due_count;
  uint32_t release_mutex_miss_count;
  uint32_t semaphore_coalesced_count;
  uint32_t dropped_record_count;
  uint32_t queue_high_water_count;
  uint32_t solve_us;
  uint32_t solve_tick_gap;
  int32_t model_id;
  int32_t solver_iterations;
  uint32_t clamp_mask;
  uint32_t numeric_flags;
  uint32_t fallback_reason;
  uint64_t release_us;
  uint64_t start_us;
  uint64_t finish_us;
  float primal_residual;
  float dual_residual;
  float optimizer_max_abs;
  float estimator[13]; /* world position, velocity, xyzw, body gyro rad/s */
  float reference[12]; /* current local-frame Xref[0] snapshot */
  float rotor_state[NINPUTS];
  float first_action_raw[NINPUTS];
  float first_action_clamped[NINPUTS];
  float progress[4]; /* measured, commanded, command speed, phase lead */
} TinyMpcSitlDiagRecord;
static_assert(sizeof(TinyMpcSitlDiagRecord) == 296u,
              "SITL diagnostic schema layout changed");

static constexpr uint32_t TINYMPC_SITL_DIAG_MAGIC = 0x544d5043u;
static constexpr uint16_t TINYMPC_SITL_DIAG_VERSION = 2u;
static constexpr uint32_t TINYMPC_SITL_DIAG_HEARTBEAT_TICKS = M2T(1000);
static_assert(TINYMPC_SITL_MMAP_RECORD_SIZE ==
                  sizeof(TinyMpcSitlDiagRecord),
              "mmap slot payload and v2 record sizes differ");
static_assert(TINYMPC_SITL_MMAP_FILE_SIZE == 5246976u,
              "SITL mmap diagnostic file size changed");
static_assert(__atomic_always_lock_free(8, 0),
              "SITL diagnostics require lock-free 64-bit atomics");
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "SITL mmap diagnostic format requires a little-endian host"
#endif
static TinyMpcSitlMmapDiagHeader *sitl_diag_mapping = NULL;
static uint32_t sitl_diag_release_sequence = 0u;
static uint32_t sitl_diag_solve_sequence = 0u;
static uint32_t sitl_diag_release_due_count = 0u;
static uint32_t sitl_diag_release_mutex_miss_count = 0u;
static uint32_t sitl_diag_semaphore_coalesced_count = 0u;
static uint32_t sitl_diag_previous_fallback_reason = UINT32_MAX;
static TinyMpcSitlDiagRecord sitl_diag_solve_record;
static TinyMpcSitlDiagRecord sitl_diag_fallback_record;
static uint64_t planner_diag_release_us = 0u;
static uint32_t planner_diag_release_rtos_tick = 0u;
static uint32_t planner_diag_release_sequence = 0u;
static uint32_t planner_diag_release_due_count = 0u;
static uint32_t planner_diag_release_mutex_miss_count = 0u;
static uint32_t planner_diag_semaphore_coalesced_count = 0u;
enum TinyMpcSitlDiagMode {
  TINYMPC_SITL_DIAG_OFF = 0,
  TINYMPC_SITL_DIAG_TASKLESS = 1,
};
static TinyMpcSitlDiagMode sitl_diag_mode = TINYMPC_SITL_DIAG_OFF;
static uint32_t sitl_diag_last_solve_record_tick = 0u;
static uint32_t sitl_diag_previous_solve_anomaly_mask = 0u;
static uint32_t sitl_diag_previous_model_id = UINT32_MAX;
static uint32_t sitl_diag_previous_release_mutex_miss_count = 0u;
static uint32_t sitl_diag_previous_semaphore_coalesced_count = 0u;
static float sitl_diag_previous_first_action[NINPUTS] = {0.0f};
static bool sitl_diag_previous_first_action_valid = false;

static inline bool sitlDiagEnabled(void) {
  return sitl_diag_mode != TINYMPC_SITL_DIAG_OFF;
}

static bool sitlDiagShouldEmitSolve(
    uint32_t finish_tick, uint32_t solve_tick_gap,
    uint32_t expected_solve_tick_gap, uint32_t start_latency_us,
    uint32_t solve_us, uint32_t release_mutex_miss_count,
    uint32_t semaphore_coalesced_count, uint32_t numeric_flags,
    uint32_t scalar_trigger_flags, uint32_t clamp_mask, int32_t model_id,
    const float first_action[NINPUTS]) {
  float maximum_action_step = 0.0f;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    if (sitl_diag_previous_first_action_valid &&
        std::isfinite(first_action[motor])) {
      maximum_action_step = T_MAX(
          maximum_action_step,
          fabsf(first_action[motor] - sitl_diag_previous_first_action[motor]));
    }
  }
  uint32_t anomaly_mask = 0u;
  anomaly_mask |= solve_tick_gap > expected_solve_tick_gap ? 1u : 0u;
  anomaly_mask |= start_latency_us > 1000000u / (uint32_t)MPC_RATE ? 2u : 0u;
  anomaly_mask |= solve_us > 1000000u / (uint32_t)MPC_RATE ? 4u : 0u;
  anomaly_mask |= release_mutex_miss_count !=
      sitl_diag_previous_release_mutex_miss_count ? 8u : 0u;
  anomaly_mask |= semaphore_coalesced_count !=
      sitl_diag_previous_semaphore_coalesced_count ? 16u : 0u;
  anomaly_mask |= numeric_flags != 0u ? 32u : 0u;
  anomaly_mask |= clamp_mask != 0u ? 64u : 0u;
  anomaly_mask |= maximum_action_step > 0.5f ? 128u : 0u;
  anomaly_mask |= sitl_diag_previous_model_id != UINT32_MAX &&
      (int32_t)sitl_diag_previous_model_id != model_id ? 256u : 0u;
  anomaly_mask |= scalar_trigger_flags << 9;
  const bool heartbeat_due = sitl_diag_last_solve_record_tick == 0u ||
      finish_tick - sitl_diag_last_solve_record_tick >=
          TINYMPC_SITL_DIAG_HEARTBEAT_TICKS;
  const bool new_anomaly =
      (anomaly_mask & ~sitl_diag_previous_solve_anomaly_mask) != 0u;
  const bool emit = heartbeat_due || new_anomaly;
  sitl_diag_previous_solve_anomaly_mask = anomaly_mask;
  sitl_diag_previous_model_id = (uint32_t)model_id;
  sitl_diag_previous_release_mutex_miss_count = release_mutex_miss_count;
  sitl_diag_previous_semaphore_coalesced_count = semaphore_coalesced_count;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    sitl_diag_previous_first_action[motor] = first_action[motor];
  }
  sitl_diag_previous_first_action_valid = true;
  if (emit) {
    sitl_diag_last_solve_record_tick = finish_tick;
  }
  return emit;
}

static void sitlDiagPublish(TinyMpcSitlDiagRecord *record) {
  if (sitl_diag_mapping == NULL) {
    return;
  }
  uint64_t reservation_sequence = 0u;
  TinyMpcSitlMmapDiagSlot *slot = tinyMpcSitlMmapDiagReserve(
      sitl_diag_mapping, &reservation_sequence);
  if (slot == NULL) {
    return;
  }
  record->magic = TINYMPC_SITL_DIAG_MAGIC;
  record->version = TINYMPC_SITL_DIAG_VERSION;
  record->record_size = (uint16_t)sizeof(*record);
  record->event_sequence = (uint32_t)reservation_sequence;
  record->dropped_record_count = (uint32_t)__atomic_load_n(
      &sitl_diag_mapping->overflow_count, __ATOMIC_RELAXED);
  record->queue_high_water_count = 0u;
  tinyMpcSitlMmapDiagCommit(
      sitl_diag_mapping, slot, reservation_sequence,
      (const uint8_t *)record);
}

static void sitlDiagInit(void) {
  const char *mode = getenv("TINYMPC_DIAG_MODE");
  const char *path = getenv("TINYMPC_DIAG_PATH");
  sitl_diag_mode = TINYMPC_SITL_DIAG_OFF;
  if (mode == NULL || strcmp(mode, "taskless") != 0) {
    DEBUG_PRINT("MPCDIAG mode=off\n");
    return;
  }
  if (path == NULL || path[0] == '\0') {
    DEBUG_PRINT("MPCDIAG disabled: TINYMPC_DIAG_PATH is unset\n");
    return;
  }
  const int descriptor = open(
      path, O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0644);
  if (descriptor < 0) {
    DEBUG_PRINT("MPCDIAG disabled: could not exclusively create path\n");
    return;
  }
  const int allocation_result = posix_fallocate(
      descriptor, 0, (off_t)TINYMPC_SITL_MMAP_FILE_SIZE);
  if (allocation_result != 0 &&
      ftruncate(descriptor, (off_t)TINYMPC_SITL_MMAP_FILE_SIZE) != 0) {
    close(descriptor);
    DEBUG_PRINT("MPCDIAG disabled: could not allocate mmap file\n");
    return;
  }
  void *mapping = mmap(
      NULL, TINYMPC_SITL_MMAP_FILE_SIZE, PROT_READ | PROT_WRITE,
      MAP_SHARED, descriptor, 0);
  close(descriptor);
  if (mapping == MAP_FAILED) {
    DEBUG_PRINT("MPCDIAG disabled: could not map configured path\n");
    return;
  }
  sitl_diag_mapping = (TinyMpcSitlMmapDiagHeader *)mapping;
  tinyMpcSitlMmapDiagInitialize(
      sitl_diag_mapping, TINYMPC_SITL_TICK_US,
      TINYMPC_SITL_DIAG_MAGIC, TINYMPC_SITL_DIAG_VERSION);
  sitl_diag_mode = TINYMPC_SITL_DIAG_TASKLESS;
  sitl_diag_release_sequence = 0u;
  __atomic_store_n(&sitl_diag_solve_sequence, 0u, __ATOMIC_RELAXED);
  sitl_diag_release_due_count = 0u;
  sitl_diag_release_mutex_miss_count = 0u;
  sitl_diag_semaphore_coalesced_count = 0u;
  sitl_diag_previous_fallback_reason = UINT32_MAX;
  sitl_diag_last_solve_record_tick = 0u;
  sitl_diag_previous_solve_anomaly_mask = 0u;
  sitl_diag_previous_model_id = UINT32_MAX;
  sitl_diag_previous_release_mutex_miss_count = 0u;
  sitl_diag_previous_semaphore_coalesced_count = 0u;
  sitl_diag_previous_first_action_valid = false;
  __atomic_store_n(&planner_diag_release_us, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(&planner_diag_release_rtos_tick, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(&planner_diag_release_sequence, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(&planner_diag_release_due_count, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(
      &planner_diag_release_mutex_miss_count, 0u, __ATOMIC_RELAXED);
  __atomic_store_n(
      &planner_diag_semaphore_coalesced_count, 0u, __ATOMIC_RELAXED);
  TinyMpcSitlDiagRecord header = {};
  header.event = 0u;
  sitlDiagPublish(&header);
  if (msync(mapping, TINYMPC_SITL_MMAP_FILE_SIZE, MS_SYNC) != 0) {
    DEBUG_PRINT("MPCDIAG warning: preflight msync failed\n");
  }
  DEBUG_PRINT(
      "MPCDIAG mmap schema=2 mode=taskless record=%u slot=%u capacity=%u bytes=%u path=%s\n",
      (unsigned int)sizeof(TinyMpcSitlDiagRecord),
      (unsigned int)TINYMPC_SITL_MMAP_SLOT_SIZE,
      (unsigned int)TINYMPC_SITL_MMAP_CAPACITY,
      (unsigned int)TINYMPC_SITL_MMAP_FILE_SIZE, path);
}
#endif
static void tinympcControllerTask(void *parameters);
STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

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


void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  const Eigen::Vector3f solve_position_world(
      state->position.x, state->position.y, state->position.z);
  if (position_delta_velocity_valid) {
    const Eigen::Vector3f candidate =
        (solve_position_world - previous_solve_position_world) / DT;
    if (candidate.allFinite() && candidate.norm() <= 20.0f) {
      position_delta_velocity_world = candidate;
    } else {
      position_delta_velocity_valid = false;
      position_delta_velocity_world.setZero();
    }
  } else {
    position_delta_velocity_valid = true;
    position_delta_velocity_world.setZero();
  }
  previous_solve_position_world = solve_position_world;
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
}

#if defined(CONFIG_PLATFORM_SITL) && TINYMPC_VISION_RL_RESIDUAL_ENABLE
static void publishEspNetV7ActorState(
    const state_t& estimated_state, uint32_t firmware_tick_ms) {
  static const float future_times_s[5] = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f};
  float packed[TINYMPC_ACTOR_STATE_FLOATS] = {0.0f};
  const struct quat world_from_body = qnormalize(attitude);
  const struct quat body_from_world = qinv(world_from_body);
  const Eigen::Vector3f vehicle_world(
      estimated_state.position.x,
      estimated_state.position.y,
      estimated_state.position.z);
  for (int preview = 0; preview < 5; ++preview) {
    const float future_progress = tinyMpcProgressPathAdvance(
        &progress_path, progress_path.progress,
        progress_reference_speed_mps * future_times_s[preview]);
    const TinyMpcPathSample sample = tinyMpcProgressPathSample(
        &progress_path, future_progress);
    const Eigen::Vector3f sample_world = trajectory_origin_world + Eigen::Vector3f(
        trajectory_cos_yaw * sample.position.x
            - trajectory_sin_yaw * sample.position.y,
        trajectory_sin_yaw * sample.position.x
            + trajectory_cos_yaw * sample.position.y,
        sample.position.z);
    const Eigen::Vector3f offset_world = sample_world - vehicle_world;
    const struct vec offset_body = qvrot(
        body_from_world,
        mkvec(offset_world.x(), offset_world.y(), offset_world.z()));
    packed[3 * preview + 0] = offset_body.x;
    packed[3 * preview + 1] = offset_body.y;
    packed[3 * preview + 2] = offset_body.z;
  }
  const struct vec velocity_body = qvrot(
      body_from_world,
      mkvec(estimated_state.velocity.x,
            estimated_state.velocity.y,
            estimated_state.velocity.z));
  packed[15] = velocity_body.x;
  packed[16] = velocity_body.y;
  packed[17] = velocity_body.z;
  packed[18] = x0(9);
  packed[19] = x0(10);
  packed[20] = x0(11);
#if TINYMPC_ORACLE_ACTION_ENABLE
  /* Oracle-only SITL ABI: expose exact estimator position.  Every other
   * adapter retains the deployed gravity-vector fields at these indices. */
  packed[21] = estimated_state.position.x;
  packed[22] = estimated_state.position.y;
  packed[23] = estimated_state.position.z;
#else
  const struct vec gravity_body = qvrot(
      body_from_world, mkvec(0.0f, 0.0f, -1.0f));
  packed[21] = gravity_body.x;
  packed[22] = gravity_body.y;
  packed[23] = gravity_body.z;
#endif
  tinyMpcActorStateLinkPublish(packed, firmware_tick_ms);
}
#endif

static float cross2d(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return a.x() * b.y() - a.y() * b.x();
}

static void __attribute__((unused)) sampleTrajectoryReference(
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
  const float incoming_norm = incoming.head(2).norm();
  const float outgoing_norm = outgoing.head(2).norm();
  if (incoming_norm < 1.0e-7f || outgoing_norm < 1.0e-7f) {
    return 0;
  }
  incoming /= incoming_norm;
  outgoing /= outgoing_norm;
  const float curvature = cross2d(incoming, outgoing);
  if (fabsf(curvature) < 1.0e-4f) {
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
    for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
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
#if TINYMPC_GATE_OLGMD_ENABLE
  tinyMpcGateOlgmdReset(&gate_olgmd_state);
  gate_olgmd_trigger_count = 0u;
  gate_olgmd_release_count = 0u;
  gate_olgmd_near_gate_suppression = false;
#endif
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
  paper_emergency_terminal_stop_latched = false;
  paper_emergency_measured_forward_speed_mps = 0.0f;
  paper_emergency_position_velocity_active = false;
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
  pitch_through_brake_reference_active = false;
  pitch_through_brake_entry_altitude_world_m = 0.0f;
  pitch_through_brake_internal_up_position_m = 0.0f;
  pitch_through_brake_internal_up_speed_mps = 0.0f;
  pitch_through_brake_trigger_count = 0u;
#endif
#endif
  dodge_camera_yaw_initialized = false;
  dodge_camera_yaw_world_rad = 0.0f;
  dodge_rejoin_target_yaw_valid = false;
  dodge_rejoin_target_yaw_world_rad = 0.0f;
  dodge_loop_scan_route_yaw_valid = false;
  dodge_loop_scan_route_yaw_world_rad = 0.0f;
  dodge_backtrack_settle_anchor_valid = false;
  dodge_backtrack_settle_anchor_world.setZero();
  vision_track_speed_limit_mps = INFINITY;
  navigation_warmup_steps = 0;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
  gate_poc_associated = false;
  gate_poc_completed = false;
  gate_poc_consecutive_samples = 0u;
  gate_poc_dropout_steps = 0u;
  gate_poc_geometry_dropout_samples = UINT8_MAX;
  gate_poc_association_samples = 0u;
  gate_poc_last_sample = UINT32_MAX;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  gate_visual_phase = GATE_VISUAL_SEARCH;
  gate_visual_center_world.setZero();
  gate_visual_forward_world = Eigen::Vector3f::UnitX();
  gate_visual_velocity_world.setZero();
  gate_visual_center_valid = false;
  gate_visual_velocity_valid = false;
  gate_visual_rearm_clear_samples = 0u;
#endif
#endif
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
  gate_poc_collision_suppressed = false;
#endif
}

#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
/* The transport layer range-checks normalized corners. This controller also
 * requires a plausible rotation-invariant quadrilateral: convex ordered
 * corners, non-degenerate area, and nearly parallel opposing edges with
 * bounded ratios. */
static bool gateObservationFreshAndGeometric(
    const TinyRacerPerceptionObservation& observation) {
  constexpr float gate_confidence_threshold = 0.6174671283f;
  if (!observation.valid || !observation.gate_valid ||
      observation.received_age_ms > race_config.maximum_age_ms ||
      observation.gate_confidence < gate_confidence_threshold) {
    return false;
  }
  const float *corner = observation.gate_corners_xy;
  float signed_area_twice = 0.0f;
  for (int i = 0; i < 4; ++i) {
    const int next = (i + 1) % 4;
    signed_area_twice += corner[2 * i] * corner[2 * next + 1] -
        corner[2 * i + 1] * corner[2 * next];
  }
  const Eigen::Vector2f top(corner[2] - corner[0], corner[3] - corner[1]);
  const Eigen::Vector2f right(corner[4] - corner[2], corner[5] - corner[3]);
  const Eigen::Vector2f bottom(corner[4] - corner[6], corner[5] - corner[7]);
  const Eigen::Vector2f left(corner[6] - corner[0], corner[7] - corner[1]);
  const Eigen::Vector2f bottom_loop(-bottom.x(), -bottom.y());
  const Eigen::Vector2f left_loop(-left.x(), -left.y());
  const Eigen::Vector2f diagonal_tl_br(
      corner[4] - corner[0], corner[5] - corner[1]);
  const Eigen::Vector2f diagonal_tr_bl(
      corner[6] - corner[2], corner[7] - corner[3]);
  /* Gate corners arrive on a coarse 0.05-normalized image grid. Permit one
   * quantized edge at that nominal size and the corresponding 1:3 ratio. */
  constexpr float minimum_edge = 0.045f;
  /* Validate quadrilateral structure, not image-axis alignment. A real gate
   * remains valid under camera roll and perspective during banking. Opposing
   * edge parallelism, ratios, diagonals, and convexity below still reject
   * self-crossing and implausibly skewed corner sets. */
  if (!isfinite(signed_area_twice) || fabsf(signed_area_twice) < 0.01f ||
      top.norm() < minimum_edge || bottom.norm() < minimum_edge ||
      left.norm() < minimum_edge || right.norm() < minimum_edge ||
      diagonal_tl_br.norm() < minimum_edge ||
      diagonal_tr_bl.norm() < minimum_edge) {
    return false;
  }
  const float parallel_top_bottom = fabsf(top.normalized().dot(bottom.normalized()));
  const float parallel_left_right = fabsf(left.normalized().dot(right.normalized()));
  const float top_bottom_ratio = top.norm() / bottom.norm();
  const float left_right_ratio = left.norm() / right.norm();
  const float diagonal_ratio = diagonal_tl_br.norm() / diagonal_tr_bl.norm();
  const float turn_0 = top.x() * right.y() - top.y() * right.x();
  const float turn_1 = right.x() * bottom_loop.y() - right.y() * bottom_loop.x();
  const float turn_2 = bottom_loop.x() * left_loop.y() -
      bottom_loop.y() * left_loop.x();
  const float turn_3 = left_loop.x() * top.y() - left_loop.y() * top.x();
  const bool consistently_convex =
      ((turn_0 > 0.0f && turn_1 > 0.0f && turn_2 > 0.0f && turn_3 > 0.0f) ||
       (turn_0 < 0.0f && turn_1 < 0.0f && turn_2 < 0.0f && turn_3 < 0.0f));
  return parallel_top_bottom >= 0.75f && parallel_left_right >= 0.75f &&
      top_bottom_ratio >= 0.30f && top_bottom_ratio <= 3.34f &&
      left_right_ratio >= 0.30f && left_right_ratio <= 3.34f &&
      diagonal_ratio >= 0.30f && diagonal_ratio <= 3.34f &&
      consistently_convex;
}

static bool gateObservationFreshPresence(
    const TinyRacerPerceptionObservation& observation) {
  constexpr float gate_confidence_threshold = 0.6174671283f;
  return observation.valid &&
      observation.received_age_ms <= race_config.maximum_age_ms &&
      observation.gate_confidence >= gate_confidence_threshold;
}

#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
static bool gateVisualMeasurement(
    const TinyRacerPerceptionObservation& observation,
    float *depth_m, float *lateral_m, float *vertical_m) {
  if (!gateObservationFreshAndGeometric(observation)) {
    return false;
  }
  const float *corner = observation.gate_corners_xy;
  const float width = 0.5f * (
      hypotf(corner[2] - corner[0], corner[3] - corner[1]) +
      hypotf(corner[4] - corner[6], corner[5] - corner[7]));
  const float height = 0.5f * (
      hypotf(corner[6] - corner[0], corner[7] - corner[1]) +
      hypotf(corner[4] - corner[2], corner[5] - corner[3]));
  const float fx = observation.gate_fx_normalized;
  const float fy = observation.gate_fy_normalized;
  if (width < 0.05f || height < 0.05f || width * height < 0.01f ||
      fx < 0.05f || fy < 0.05f) {
    return false;
  }
  const float center_x = 0.25f *
      (corner[0] + corner[2] + corner[4] + corner[6]);
  const float center_y = 0.25f *
      (corner[1] + corner[3] + corner[5] + corner[7]);
  *depth_m = T_MIN(T_MAX(
      TINYMPC_GATE_CORNER_SPAN_M * fx / width, 0.40f), 4.0f);
  *lateral_m = T_MIN(T_MAX(
      -(center_x - observation.gate_cx_normalized) * *depth_m / fx,
      -0.50f), 0.50f);
  *vertical_m = T_MIN(T_MAX(
      -(center_y - observation.gate_cy_normalized) * *depth_m / fy,
      -0.35f), 0.35f);
  return true;
}

static bool updateGateVisualEstimate(
    const TinyRacerPerceptionObservation& observation,
    bool refine_transit_heading,
    float *depth_m, float *lateral_m, float *vertical_m) {
  if (!gateVisualMeasurement(
          observation, depth_m, lateral_m, vertical_m)) {
    return false;
  }
  const Eigen::Vector3f origin_world(
      active_local_frame.origin_x,
      active_local_frame.origin_y,
      active_local_frame.origin_z);
  Eigen::Vector3f measured_center_world = origin_world +
      localVectorToWorld(active_local_frame, Eigen::Vector3f(
          *depth_m, *lateral_m, *vertical_m));
  /* The corner head's vertical center is much noisier than its horizontal
   * bearing under roll. Use the standardized opening center for altitude;
   * horizontal gate position and approach direction remain vision-only. */
  measured_center_world.z() = TINYMPC_GATE_TRANSIT_ALTITUDE_M;
  if (!gate_visual_center_valid) {
    gate_visual_center_world = measured_center_world;
    gate_visual_center_valid = true;
  } else {
    constexpr float center_filter_alpha = 0.35f;
    gate_visual_center_world += center_filter_alpha *
        (measured_center_world - gate_visual_center_world);
  }
  if (refine_transit_heading && *depth_m >= 0.65f) {
    /* Generated arenas place gates tangent to the route. A direct chord from
     * an early sighting to the opening remains camera-centered but meets the
     * gate plane obliquely on a curved course. Retain the current route
     * tangent as the crossing direction and let vision correct cross-track. */
    Eigen::Vector3f route_forward = localVectorToWorld(
        active_local_frame, Xref[0].segment<3>(6));
    route_forward.z() = 0.0f;
    if (route_forward.head<2>().norm() > 0.10f) {
      const Eigen::Vector3f measured_forward = route_forward.normalized();
      constexpr float heading_filter_alpha = 0.25f;
      Eigen::Vector3f blended_forward =
          (1.0f - heading_filter_alpha) * gate_visual_forward_world +
          heading_filter_alpha * measured_forward;
      blended_forward.z() = 0.0f;
      if (blended_forward.head<2>().norm() > 0.10f) {
        gate_visual_forward_world = blended_forward.normalized();
      }
    }
  }
  return true;
}
#endif

static void updateGatePocAssociation(
    const TinyRacerPerceptionObservation& observation) {
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  /* Three high-confidence presence frames reject isolated rail hallucinations;
   * acquisition below additionally requires a retained geometric bearing. */
  constexpr uint8_t required_consecutive_samples = 3u;
#else
  constexpr uint8_t required_consecutive_samples = 2u;
#endif
  /* Low-presence samples (or stale MPC cycles after transport expiry) release
   * the retained bearing. In joint mode, one counter bounds consecutive
   * geometry loss and a monotonic counter bounds total association lifetime;
   * both govern servoing and collision suppression together. */
#if TINYMPC_JOINT_GATE_RL_ENABLE
  /* Retain the last geometrically accepted bearing for at most 20 consecutive
   * missing-geometry frames and never beyond 300 associated camera samples.
   * Either bound permanently completes the one-gate association. */
  constexpr uint8_t maximum_dropout_steps =
      joint_gate_maximum_geometry_dropout_samples + 1u;
#else
  constexpr uint8_t maximum_dropout_steps = 6u;
#endif
  const bool geometric = gateObservationFreshAndGeometric(observation);
  const bool fresh_presence = gateObservationFreshPresence(observation);
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  const bool association_evidence = fresh_presence;
#else
  const bool association_evidence = geometric;
#endif
  const bool new_sample = observation.sample != gate_poc_last_sample;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  if (gate_visual_phase == GATE_VISUAL_REARM) {
    /* An obstacle dodge temporarily owns Xref. Its stopped or displaced
     * horizon is not evidence that the nominal route has been recaptured. */
    if (dodge_state.phase != TINYRACER_DODGE_TRACK ||
        race_intent.constraint_active) {
      return;
    }
    if (new_sample) {
      if (fresh_presence) {
        gate_visual_rearm_clear_samples = 0u;
      } else if (gate_visual_rearm_clear_samples < UINT8_MAX) {
        ++gate_visual_rearm_clear_samples;
      }
      Eigen::Vector3f route_forward_world = localVectorToWorld(
          active_local_frame, Xref[0].segment<3>(6));
      route_forward_world.z() = 0.0f;
      const bool route_stopped =
          route_forward_world.head<2>().norm() <= 0.05f;
      Eigen::Vector3f route_forward_local = Xref[0].segment<3>(6);
      route_forward_local.z() = 0.0f;
      float route_heading_error_rad = 0.0f;
      float route_cross_track_error_m = 0.0f;
      float route_vertical_error_m = fabsf(Xref[0](2));
      float route_lateral_speed_error_mps = 0.0f;
      float route_tangent_speed_error_mps = 0.0f;
      if (!route_stopped) {
        route_forward_world.normalize();
        route_forward_local.normalize();
        const Eigen::Vector3f route_lateral_local(
            -route_forward_local.y(), route_forward_local.x(), 0.0f);
        route_heading_error_rad = fabsf(remainderf(
            atan2f(route_forward_world.y(), route_forward_world.x()) -
                atan2f(gate_visual_forward_world.y(),
                       gate_visual_forward_world.x()),
            6.28318530717958647692f));
        route_cross_track_error_m = fabsf(
            route_lateral_local.dot(Xref[0].head<3>()));
        const Eigen::Vector3f route_velocity_error_local =
            x0.segment<3>(6) - Xref[0].segment<3>(6);
        route_lateral_speed_error_mps = fabsf(
            route_lateral_local.dot(route_velocity_error_local));
        route_tangent_speed_error_mps = fabsf(
            route_forward_local.dot(route_velocity_error_local));
      }
      constexpr float maximum_rearm_heading_error_rad = 0.0872664626f;
      const struct quat measured_attitude = qnormalize(attitude);
      const float measured_body_z_world_z = T_MIN(T_MAX(
          1.0f - 2.0f *
              (measured_attitude.x * measured_attitude.x +
               measured_attitude.y * measured_attitude.y),
          -1.0f), 1.0f);
      const float measured_tilt_rad = acosf(measured_body_z_world_z);
      const float measured_body_rate_rad_s = x0.segment<3>(9).norm();
      constexpr float maximum_rearm_cross_track_error_m = 0.12f;
      constexpr float maximum_rearm_vertical_error_m = 0.10f;
      constexpr float maximum_rearm_lateral_speed_error_mps = 0.15f;
      constexpr float maximum_rearm_tangent_speed_error_mps = 0.10f;
      constexpr float maximum_rearm_tilt_rad = 0.1745329252f;
      constexpr float maximum_rearm_body_rate_rad_s = 0.75f;
      const bool route_capture_ready = route_stopped ||
          (route_heading_error_rad <= maximum_rearm_heading_error_rad &&
           route_cross_track_error_m <= maximum_rearm_cross_track_error_m &&
           route_vertical_error_m <= maximum_rearm_vertical_error_m &&
           route_lateral_speed_error_mps <=
               maximum_rearm_lateral_speed_error_mps &&
           route_tangent_speed_error_mps <=
               maximum_rearm_tangent_speed_error_mps &&
           measured_tilt_rad <= maximum_rearm_tilt_rad &&
           measured_body_rate_rad_s <= maximum_rearm_body_rate_rad_s);
      if (gate_visual_rearm_clear_samples >= 15u &&
          route_capture_ready) {
        /* Gate flight deliberately runs below the nominal virtual route
         * clock. Do not carry that stale along-track lead back into TRACK:
         * its position catch-up and collision-limited velocity objectives
         * otherwise disagree until the direct-motor model leaves its local
         * region. Rejoin the route at the phase actually reached. */
        const float discarded_route_lead_m = tinyMpcProgressPathDistance(
            &progress_path, progress_path.measured_progress,
            progress_path.progress);
        progress_path.progress = progress_path.measured_progress;
        gate_visual_phase = GATE_VISUAL_SEARCH;
        gate_visual_velocity_valid = false;
        gate_poc_consecutive_samples = 0u;
        gate_poc_dropout_steps = 0u;
#if TINYMPC_RATE_CASCADE
        resetOuterLoopDuals();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
        resetLevelActuatorDuals();
#endif
        DEBUG_PRINT(
            "Gate visual transit rearmed after route capture cross=%.3fm vertical=%.3fm lateral_speed=%.3fm/s tangent_speed_error=%.3fm/s heading_error=%.1fdeg tilt=%.1fdeg body_rate=%.2frad/s discarded_route_lead=%.3fm\n",
            (double)route_cross_track_error_m,
            (double)route_vertical_error_m,
            (double)route_lateral_speed_error_mps,
            (double)route_tangent_speed_error_mps,
            (double)(route_heading_error_rad * 57.2957795131f),
            (double)(measured_tilt_rad * 57.2957795131f),
            (double)measured_body_rate_rad_s,
            (double)discarded_route_lead_m);
      }
      gate_poc_last_sample = observation.sample;
    }
    return;
  }
  if (gate_visual_phase == GATE_VISUAL_TRANSIT) {
    if (new_sample) {
      float depth_m = 0.0f;
      float lateral_m = 0.0f;
      float vertical_m = 0.0f;
      /* A single coarse corner estimate can miss a 0.4 m opening. Continue
       * visual servo refinement while the gate is ahead, then retain the last
       * estimate through the near-plane geometry dropout. */
      (void)updateGateVisualEstimate(
          observation, true, &depth_m, &lateral_m, &vertical_m);
      gate_poc_last_sample = observation.sample;
    }
    return;
  }
#endif
  if (new_sample && gate_poc_associated &&
      gate_poc_association_samples < UINT16_MAX) {
    ++gate_poc_association_samples;
  }
  if (geometric) {
    gate_poc_geometry_dropout_samples = 0u;
  } else if ((new_sample ||
              observation.received_age_ms > race_config.maximum_age_ms) &&
             gate_poc_geometry_dropout_samples < UINT8_MAX) {
    ++gate_poc_geometry_dropout_samples;
  }
  if (association_evidence) {
    gate_poc_dropout_steps = 0u;
    if (new_sample && gate_poc_consecutive_samples < UINT8_MAX) {
      ++gate_poc_consecutive_samples;
    }
    if (!gate_poc_completed &&
        gate_poc_consecutive_samples >= required_consecutive_samples &&
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
        gate_visual_center_valid &&
#endif
        !gate_poc_associated) {
      gate_poc_associated = true;
      gate_poc_association_samples = 0u;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
      gate_visual_phase = GATE_VISUAL_ALIGN;
      Eigen::Vector3f approach_local = Xref[0].segment<3>(6);
      approach_local.z() = 0.0f;
      if (approach_local.head<2>().norm() > 0.05f) {
        approach_local.normalize();
        gate_visual_forward_world = localVectorToWorld(
            active_local_frame, approach_local);
      } else {
        gate_visual_forward_world = Eigen::Vector3f(
            cosf(active_local_frame.yaw_world),
            sinf(active_local_frame.yaw_world), 0.0f);
      }
#endif
      DEBUG_PRINT("Gate POC visual association acquired sample=%lu confidence=%.2f\n",
                  (unsigned long)observation.sample,
                  (double)observation.gate_confidence);
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
      if (gate_visual_phase == GATE_VISUAL_TRANSIT) {
        DEBUG_PRINT("Gate visual TRANSIT committed from retained center\n");
      }
#endif
    }
  } else if (gate_poc_associated && fresh_presence) {
    /* The corner/mask postprocessor can reject a still-visible gate after a
     * successful acquisition. Keep its last bounded bearing correction, but
     * never update that correction or treat this weaker evidence as an open
     * passage for collision suppression. */
    gate_poc_dropout_steps = 0u;
  } else if (new_sample ||
             observation.received_age_ms > race_config.maximum_age_ms) {
    gate_poc_consecutive_samples = 0u;
    if (gate_poc_dropout_steps < UINT8_MAX) {
      ++gate_poc_dropout_steps;
    }
    if (gate_poc_dropout_steps >= maximum_dropout_steps && gate_poc_associated) {
      gate_poc_associated = false;
#if TINYMPC_JOINT_GATE_RL_ENABLE
      gate_poc_completed = true;
#endif
      gate_lateral_offset_m = 0.0f;
      gate_vertical_offset_m = 0.0f;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
      gate_visual_phase = GATE_VISUAL_SEARCH;
      gate_visual_center_valid = false;
#endif
      DEBUG_PRINT("Gate POC visual association released after low presence\n");
    }
  }
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  if (geometric && new_sample &&
      (gate_visual_phase == GATE_VISUAL_SEARCH ||
       gate_visual_phase == GATE_VISUAL_ALIGN)) {
    float depth_m = 0.0f;
    float lateral_m = 0.0f;
    float vertical_m = 0.0f;
    if (updateGateVisualEstimate(
            observation, false, &depth_m, &lateral_m, &vertical_m)) {
      if (gate_poc_associated && gate_visual_phase == GATE_VISUAL_ALIGN &&
          fabsf(lateral_m) <= 0.08f && fabsf(vertical_m) <= 0.08f) {
        Eigen::Vector3f route_forward = localVectorToWorld(
            active_local_frame, Xref[0].segment<3>(6));
        route_forward.z() = 0.0f;
        if (route_forward.head<2>().norm() > 0.10f) {
          route_forward.normalize();
          const float route_heading_rad = atan2f(
              route_forward.y(), route_forward.x());
          const float heading_error_rad = fabsf(remainderf(
              route_heading_rad - active_local_frame.yaw_world,
              6.28318530717958647692f));
          const Eigen::Vector3f route_lateral(
              -route_forward.y(), route_forward.x(), 0.0f);
          const Eigen::Vector3f measured_velocity_world = localVectorToWorld(
              active_local_frame, x0.segment<3>(6));
          const float lateral_speed_mps = fabsf(
              route_lateral.dot(measured_velocity_world));
          const struct quat measured_attitude = qnormalize(attitude);
          const float body_z_world_z = T_MIN(T_MAX(
              1.0f - 2.0f *
                  (measured_attitude.x * measured_attitude.x +
                   measured_attitude.y * measured_attitude.y),
              -1.0f), 1.0f);
          const float tilt_rad = acosf(body_z_world_z);
          const float body_rate_rad_s = x0.segment<3>(9).norm();
          constexpr float maximum_gate_entry_heading_error_rad =
              0.0872664626f;
          constexpr float maximum_gate_entry_lateral_speed_mps = 0.15f;
          constexpr float maximum_gate_entry_tilt_rad = 0.3490658504f;
          constexpr float maximum_gate_entry_body_rate_rad_s = 0.75f;
          if (heading_error_rad <= maximum_gate_entry_heading_error_rad &&
              lateral_speed_mps <= maximum_gate_entry_lateral_speed_mps &&
              tilt_rad <= maximum_gate_entry_tilt_rad &&
              body_rate_rad_s <= maximum_gate_entry_body_rate_rad_s) {
            gate_visual_forward_world = route_forward;
            gate_visual_phase = GATE_VISUAL_TRANSIT;
            DEBUG_PRINT(
                "Gate visual TRANSIT committed depth=%.2f bearing=(%.2f,%.2f) heading_error=%.1fdeg lateral_speed=%.2fm/s\n",
                (double)depth_m, (double)lateral_m, (double)vertical_m,
                (double)(heading_error_rad * 57.2957795131f),
                (double)lateral_speed_mps);
          }
        }
      }
    }
  }
  if (!fresh_presence && !gate_poc_associated &&
      gate_visual_phase == GATE_VISUAL_SEARCH) {
    gate_visual_center_valid = false;
  }
#endif
#if TINYMPC_JOINT_GATE_RL_ENABLE
  if (gate_poc_associated &&
      gate_poc_geometry_dropout_samples >=
          joint_gate_maximum_geometry_dropout_samples + 1u) {
    gate_poc_associated = false;
    gate_poc_completed = true;
    gate_poc_consecutive_samples = 0u;
    gate_lateral_offset_m = 0.0f;
    gate_vertical_offset_m = 0.0f;
    DEBUG_PRINT("Joint gate association completed after geometry timeout\n");
  }
  if (gate_poc_associated &&
      gate_poc_association_samples >=
          joint_gate_maximum_association_samples) {
    gate_poc_associated = false;
    gate_poc_completed = true;
    gate_poc_consecutive_samples = 0u;
    gate_lateral_offset_m = 0.0f;
    gate_vertical_offset_m = 0.0f;
    DEBUG_PRINT("Joint gate association completed after hard lifetime\n");
  }
#endif
  if (new_sample) {
    gate_poc_last_sample = observation.sample;
  }
}
#endif

static void setLocalReferenceState(
    VectorNf& target, const Eigen::Vector3f& position_world,
    struct quat attitude_world_body, const Eigen::Vector3f& velocity_world,
    const Eigen::Vector3f& angular_velocity_body);

#if TINYMPC_PATH_TUNNEL_ENABLE
static void setPathTunnelHalfspaces(
    int knot, const Eigen::Vector3f& center_world,
    const TinyMpcTunnelFrame& frame_world);
#endif

static struct quat referenceStateQuaternion(const VectorNf& state) {
  return qnormalize(mkquat(state(3), state(4), state(5), 1.0f));
}

/* Change camera heading without changing the reference thrust direction.
 * A raw Rodrigues-z edit would also corrupt the commanded bank whenever the
 * vehicle is laterally accelerating. */
static struct quat setReferenceYawPreservingBodyZ(
    VectorNf& state, float yaw_local_rad) {
  const struct quat old_attitude = referenceStateQuaternion(state);
  const float body_z_x = 2.0f *
      (old_attitude.x * old_attitude.z + old_attitude.w * old_attitude.y);
  const float body_z_y = 2.0f *
      (old_attitude.y * old_attitude.z - old_attitude.w * old_attitude.x);
  const float body_z_z = 1.0f - 2.0f *
      (old_attitude.x * old_attitude.x + old_attitude.y * old_attitude.y);
  const float heading_cos = cosf(yaw_local_rad);
  const float heading_sin = sinf(yaw_local_rad);
  const float body_z_forward =
      heading_cos * body_z_x + heading_sin * body_z_y;
  const float body_z_left = fminf(fmaxf(
      -heading_sin * body_z_x + heading_cos * body_z_y, -1.0f), 1.0f);
  const float roll_rad = asinf(-body_z_left);
  const float pitch_rad = atan2f(body_z_forward, body_z_z);
  const struct quat attitude = qnormalize(rpy2quat(mkvec(
      roll_rad, pitch_rad, yaw_local_rad)));
  const float denominator = fabsf(attitude.w) > 1.0e-6f
      ? attitude.w : copysignf(1.0e-6f, attitude.w);
  state(3) = attitude.x / denominator;
  state(4) = attitude.y / denominator;
  state(5) = attitude.z / denominator;
  return attitude;
}

static void applyVisionNavigation(
    const TinyRacerPerceptionObservation& observation) {
#if TINYMPC_GATE_OLGMD_ENABLE
  SequentialObstacleThreatObservation olgmd_threat = {};
  SequentialObstacleGateObservation olgmd_gate = {};
  const bool olgmd_threat_available =
      sequentialObstacleLinkGetLatestThreat(&olgmd_threat);
  const bool olgmd_gate_available =
      sequentialObstacleLinkGetLatestGate(&olgmd_gate);
  TinyRacerPerceptionObservation olgmd_gate_observation = {};
  olgmd_gate_observation.valid = olgmd_gate_available;
  olgmd_gate_observation.gate_valid = olgmd_gate.valid;
  olgmd_gate_observation.gate_confidence = olgmd_gate.valid ? 1.0f : 0.0f;
  olgmd_gate_observation.gate_fx_normalized =
      (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  olgmd_gate_observation.gate_fy_normalized =
      (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  olgmd_gate_observation.gate_cx_normalized =
      (float)TINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED;
  olgmd_gate_observation.gate_cy_normalized =
      (float)TINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED;
  olgmd_gate_observation.received_age_ms = olgmd_gate.received_age_ms;
  olgmd_gate_observation.sample = olgmd_gate.sample;
  memcpy(olgmd_gate_observation.gate_corners_xy, olgmd_gate.corners_xy,
         sizeof(olgmd_gate_observation.gate_corners_xy));
#endif
  TinyRacerPerceptionObservation navigation_observation = observation;
  const auto clear_navigation_risk = [&navigation_observation]() {
    navigation_observation.collision_probability = 0.0f;
    for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
      navigation_observation.danger_probability[sector] = 0.0f;
    }
  };
#if TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
  /* The paper's head-on experiment explicitly silences steering. Update the
   * published integral/quadratic/IIR braking law once per camera result and
   * keep the generic lateral dodge state machine inactive for this mode. */
  if (observation.valid && observation.has_navigation_command &&
      observation.sample != dronet_v2_last_sample) {
    dronet_v2_speed_scale = pulpDronetV2BrakeStep(
        &dronet_v2_brake, observation.collision_probability);
    dronet_v2_last_sample = observation.sample;
    DEBUG_PRINT(
        "PULP-DroNetV2 sample=%lu pcol=%.4f integral=%.4f speed_scale=%.4f\n",
        (unsigned long)observation.sample,
        (double)observation.collision_probability,
        (double)dronet_v2_brake.collision_integral,
        (double)dronet_v2_speed_scale);
  }
  clear_navigation_risk();
#endif
#if defined(TINYMPC_VISION_ESPNET_DRONET_ENABLE)
  /* ESPNet exposes three spatial risks. Any one may be the first valid
   * warning for a side-entering obstacle, so use their maximum for the
   * continuous TRACK slowdown and dodge trigger, and steer toward the safer
   * half-image when the trigger threshold is crossed. */
  tinyRacerNavigationFuseSectorRisk(
      &navigation_observation, dodge_config.trigger_probability,
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
      /* A circular centerline appears toward the inside of the image by
       * roughly lookahead/(2R). Treat moderate half-image imbalance as
       * curvature ambiguity; only a strongly sided obstacle overrides the
       * inside-lane preference. */
      0.15f, -TRAJECTORY_TURN_DIRECTION);
#else
      0.0f, 0);
#endif
  if (progress_heading_alignment_active) {
    /* Camera sectors are route-relative only after yaw is tangent to the
     * circle. Do not latch scenery seen during the launch alignment turn. */
    clear_navigation_risk();
  }
#endif
  constexpr uint16_t navigation_warmup_limit_steps = 25u;
  if (navigation_warmup_steps < navigation_warmup_limit_steps) {
    ++navigation_warmup_steps;
    clear_navigation_risk();
    if (navigation_warmup_steps == navigation_warmup_limit_steps) {
      /* Discard background steering accumulated while collision triggering was
       * inhibited. Side selection must start from the current encounter. */
      tinyRacerNavigationReset(&navigation_state);
    }
  }
  /* Avoidance is deliberately reactive. Each sustained threshold crossing
   * starts one encounter; after a clear pass and short re-arm distance the
   * same state machine can respond to the next obstacle. No course obstacle
   * positions or encounter indices are stored in firmware. */
  /* A gate receives authority only after three consecutive high-confidence
   * presence observations and at least one geometric bearing. No course
   * position, gate ordering, or map window is used. */
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  const bool gate_was_associated = gate_poc_associated;
#if TINYMPC_GATE_OLGMD_ENABLE
  updateGatePocAssociation(olgmd_gate_observation);
#else
  updateGatePocAssociation(observation);
#endif
  if (!gate_was_associated && gate_poc_associated &&
      dodge_state.phase != TINYRACER_DODGE_TRACK) {
    const bool recovery_owns_reference =
        dodge_state.phase == TINYRACER_DODGE_EMERGENCY_BRAKE ||
        dodge_state.phase == TINYRACER_DODGE_BACKTRACK ||
        dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE ||
        dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
        dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT ||
        dodge_state.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
    if (recovery_owns_reference) {
      /* Never replace a committed stop/retreat with a newly recognized gate. */
      gate_poc_associated = false;
      gate_poc_consecutive_samples = 0u;
      gate_poc_dropout_steps = 0u;
      gate_poc_association_samples = 0u;
      gate_visual_phase = GATE_VISUAL_SEARCH;
      gate_visual_center_valid = false;
      gate_visual_velocity_valid = false;
      DEBUG_PRINT("Gate visual acquisition deferred during recovery\n");
    } else {
      /* Gate rails initially look like an obstacle. Once three-frame gate
       * geometry and a retained center confirm an opening, hand the reference
       * to the already-bounded gate servo instead of finishing a lateral dodge
       * into the rim. The first regenerated-course gate uses this same servo. */
      tinyRacerDodgeReset(&dodge_state);
      tinyRacerNavigationReset(&navigation_state);
      dodge_camera_yaw_initialized = false;
      dodge_rejoin_target_yaw_valid = false;
#if TINYMPC_RATE_CASCADE
      resetOuterLoopDuals();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
      resetLevelActuatorDuals();
#endif
      DEBUG_PRINT("Gate visual acquisition superseding ordinary dodge\n");
    }
  }
  const bool visual_gate_authority =
      ((gate_poc_associated && gate_visual_center_valid &&
        (gate_visual_phase == GATE_VISUAL_ALIGN ||
         gate_visual_phase == GATE_VISUAL_TRANSIT))) &&
      !race_intent.constraint_active &&
      tinyRacerGateServoAllowed(
          race_intent.mode, dodge_state.phase, perception_halfspace_active,
          perception_recovery_active);
  if (visual_gate_authority) {
    clear_navigation_risk();
    if (!gate_poc_collision_suppressed) {
      gate_poc_collision_suppressed = true;
      DEBUG_PRINT("Gate visual transit suppressing collision risk\n");
    }
  } else if (gate_poc_collision_suppressed) {
    gate_poc_collision_suppressed = false;
    DEBUG_PRINT("Gate visual transit restoring collision authority\n");
  }
#elif TINYMPC_JOINT_GATE_RL_ENABLE
#if TINYMPC_GATE_OLGMD_ENABLE
  updateGatePocAssociation(olgmd_gate_observation);
#else
  updateGatePocAssociation(observation);
#endif
  /* Gate rails are traversable structure, not an obstacle to dodge around.
   * A short 20-frame geometry grace bridges the near-plane blind interval; a
   * separate 300-sample hard lifetime also ends a continuously geometric
   * association. Either completion is one-shot for this single-gate flight. */
  const bool visual_gate_authority = gate_poc_associated &&
      gate_poc_geometry_dropout_samples <=
          joint_gate_maximum_geometry_dropout_samples &&
      !race_intent.constraint_active &&
      tinyRacerGateServoAllowed(
          race_intent.mode, dodge_state.phase, perception_halfspace_active,
          perception_recovery_active);
  if (visual_gate_authority) {
    clear_navigation_risk();
    if (!gate_poc_collision_suppressed) {
      gate_poc_collision_suppressed = true;
      DEBUG_PRINT("Joint gate suppressing avoidance during opening evidence\n");
    }
  } else if (gate_poc_collision_suppressed) {
    gate_poc_collision_suppressed = false;
    DEBUG_PRINT("Joint gate restoring obstacle authority\n");
  }
#endif
  /* Once the finite course is complete, do not launch a new avoidance
   * encounter from scenery outside the reference. Existing dodge state still
   * receives clear samples and returns to TRACK normally. */
  if (step + 5u >= progress_path.virtual_count) {
    clear_navigation_risk();
  }
#if defined(TINYMPC_TRAJECTORY_DRONET_U)
  /* The published U course deliberately makes the corridor walls fill the
   * camera during S2. They are boundaries already represented by the stored
   * path, not new cross-lane obstacles. Suppress new neural dodge triggers in
   * the locally curved turnaround so the active bypass can rejoin center and
   * TinyMPC can execute the 180-degree reference. Reactive avoidance resumes
   * automatically on the straight S3 segment. */
  if (trajectoryOutsidePassSide() != 0) {
    clear_navigation_risk();
  }
#endif
  tinyRacerNavigationUpdate(
      &navigation_state, &navigation_observation, &navigation_config, DT,
      active_local_frame.yaw_world, &navigation_intent);
  /* Measure the vehicle against the still-unshifted route. These values—not
   * the commanded avoidance offset—govern clearance and TRACK re-entry. */
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
  const Eigen::Vector3f route_lateral_local(
      -path_local.y(), path_local.x(), 0.0f);
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
  const Eigen::Vector3f position_delta_velocity_local = worldVectorToLocal(
      active_local_frame, position_delta_velocity_world);
  const float estimator_forward_speed_mps =
      path_local.dot(x0.segment<3>(6));
  const float emergency_signed_forward_speed_mps =
      position_delta_velocity_valid
      ? path_local.dot(position_delta_velocity_local)
      : estimator_forward_speed_mps;
  const float emergency_forward_speed_mps =
      tinyMpcEmergencyMeasuredForwardSpeed(
          estimator_forward_speed_mps,
          path_local.dot(position_delta_velocity_local),
          position_delta_velocity_valid);
  const float emergency_lateral_speed_mps = position_delta_velocity_valid
      ? route_lateral_local.dot(position_delta_velocity_local)
      : route_lateral_local.dot(x0.segment<3>(6));
  const bool paper_emergency_requested =
      paper_emergency_terminal_stop_latched ||
      (navigation_intent.new_sample &&
       navigation_intent.center_collision_probability >
           dodge_config.emergency_brake_probability);
  if (paper_emergency_requested && position_delta_velocity_valid) {
    /* CrazySim's external-pose estimator can lag the plant velocity by more
     * than 50% during a fast approach.  A 50 Hz position delta is in the same
     * spatial frame as the MPC state and is the authoritative emergency
     * velocity for both state error and cache selection. */
    x0.segment<3>(6) = position_delta_velocity_local;
    paper_emergency_position_velocity_active = true;
  }
  paper_emergency_measured_forward_speed_mps =
      emergency_forward_speed_mps;
#else
  const float emergency_signed_forward_speed_mps =
      path_local.dot(x0.segment<3>(6));
  const float emergency_forward_speed_mps =
      path_local.dot(x0.segment<3>(6));
  const float emergency_lateral_speed_mps =
      route_lateral_local.dot(x0.segment<3>(6));
#endif
  const struct quat measured_attitude = qnormalize(attitude);
  const float body_z_world_z = fminf(fmaxf(
      1.0f - 2.0f * (measured_attitude.x * measured_attitude.x +
                     measured_attitude.y * measured_attitude.y),
      -1.0f), 1.0f);
  const float rejoin_lateral_direction = dodge_state.lateral_offset_m > 0.0f
      ? -1.0f : dodge_state.lateral_offset_m < 0.0f ? 1.0f : 0.0f;
  const Eigen::Vector3f prospective_rejoin_velocity_local =
      path_local * dodge_config.rejoin_forward_speed_mps +
      route_lateral_local *
          (rejoin_lateral_direction * dodge_config.rejoin_lateral_rate_mps);
  float prospective_rejoin_yaw_world_rad = active_local_frame.yaw_world;
  float rejoin_heading_error_rad = 0.0f;
  if (prospective_rejoin_velocity_local.head<2>().norm() > 1.0e-6f) {
    prospective_rejoin_yaw_world_rad = active_local_frame.yaw_world +
        atan2f(prospective_rejoin_velocity_local.y(),
               prospective_rejoin_velocity_local.x());
    const bool rejoin_alignment_active =
        dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
        dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
    const float desired_rejoin_yaw_world_rad =
        rejoin_alignment_active && dodge_rejoin_target_yaw_valid
        ? dodge_rejoin_target_yaw_world_rad
        : prospective_rejoin_yaw_world_rad;
    rejoin_heading_error_rad = remainderf(
        desired_rejoin_yaw_world_rad - active_local_frame.yaw_world,
        6.28318530717958647692f);
  }
  const bool loop_scan_left_requested =
      dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
      dodge_state.loop_scan_pending;
  const bool loop_scan_right_requested =
      dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT;
  const float loop_scan_heading_error_rad =
      loop_scan_left_requested || loop_scan_right_requested
      ? remainderf(
          (dodge_loop_scan_route_yaw_valid
               ? dodge_loop_scan_route_yaw_world_rad -
                   active_local_frame.yaw_world
               : atan2f(path_local.y(), path_local.x())) +
              (loop_scan_left_requested ? 1.0f : -1.0f) *
                  dodge_config.loop_scan_yaw_rad,
          6.28318530717958647692f)
      : 0.0f;
  TinyRacerDodgeFeedback dodge_feedback = {
    true,
    progress_path.cumulative_measured_advance_m,
    -route_lateral_local.dot(Xref[0].head<3>()),
    emergency_lateral_speed_mps -
        route_lateral_local.dot(Xref[0].segment<3>(6)),
    emergency_forward_speed_mps,
    x0(8),
    acosf(body_z_world_z),
    x0.segment<3>(9).norm(),
    rejoin_heading_error_rad,
    loop_scan_heading_error_rad
  };
  dodge_feedback.valid = isfinite(dodge_feedback.forward_progress_m) &&
      isfinite(dodge_feedback.lateral_offset_m) &&
      isfinite(dodge_feedback.lateral_speed_mps) &&
      isfinite(dodge_feedback.forward_speed_mps) &&
      isfinite(dodge_feedback.vertical_speed_mps) &&
      isfinite(dodge_feedback.tilt_rad) &&
      isfinite(dodge_feedback.body_rate_rad_s) &&
      isfinite(dodge_feedback.rejoin_heading_error_rad) &&
      isfinite(dodge_feedback.loop_scan_heading_error_rad);
#if TINYMPC_GATE_OLGMD_ENABLE
  bool olgmd_force_hold_anchor = false;
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
  if (gate_olgmd_resume_refractory_cycles > 0u) {
    --gate_olgmd_resume_refractory_cycles;
  }
#endif
  float olgmd_gate_depth_m = INFINITY;
  const bool olgmd_gate_geometry_valid = olgmd_gate_available &&
      olgmd_gate.valid && tinyMpcGateOlgmdDepthMeters(
          olgmd_gate.corners_xy,
          (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED,
          (float)TINYMPC_GATE_CORNER_SPAN_M, &olgmd_gate_depth_m);
  /*
   * The current corner-regression head has no calibrated presence/objectness
   * output. Until that exists, it must not veto the independent safety sensor:
   * obstacle-only footage is otherwise classified as a nearby gate.
   */
  const bool olgmd_gate_align_or_transit = false;
  const bool olgmd_stationary =
      fabsf(dodge_feedback.forward_speed_mps) <=
      (float)TINYMPC_OLGMD_MOVING_SPEED_MPS;
  const TinyMpcGateOlgmdInput olgmd_input = {
    olgmd_threat_available ||
        (TINYMPC_OLGMD_CLEAR_RESUME_ENABLE && olgmd_stationary),
    (olgmd_threat_available &&
         olgmd_threat.received_age_ms <= navigation_config.maximum_age_ms) ||
        (TINYMPC_OLGMD_CLEAR_RESUME_ENABLE && olgmd_stationary),
    olgmd_threat.sample,
    olgmd_threat.imminent_threat &&
        (!TINYMPC_OLGMD_CLEAR_RESUME_ENABLE ||
         (gate_olgmd_resume_refractory_cycles == 0u &&
          dodge_feedback.forward_speed_mps >
              (float)TINYMPC_OLGMD_MOVING_SPEED_MPS)),
    olgmd_gate_available,
    olgmd_gate_available &&
        olgmd_gate.received_age_ms <= navigation_config.maximum_age_ms,
    olgmd_gate.sample,
    olgmd_gate_geometry_valid,
    olgmd_gate_align_or_transit,
    olgmd_gate_depth_m,
    TINYMPC_OLGMD_CLEAR_RESUME_ENABLE &&
            fabsf(dodge_feedback.forward_speed_mps) < 0.35f
        ? 0.0f
        : dodge_feedback.forward_speed_mps,
  };
  const TinyMpcGateOlgmdOutput olgmd_output =
      tinyMpcGateOlgmdStep(&gate_olgmd_state, &olgmd_input);
  gate_olgmd_near_gate_suppression = olgmd_output.near_gate_suppression;
  if (olgmd_output.triggered) {
    ++gate_olgmd_trigger_count;
    DEBUG_PRINT(
        "oLGMD brake latched threat=%u fresh=%u near_gate=%u depth=%.2fm\n",
        (unsigned)olgmd_threat.imminent_threat,
        (unsigned)olgmd_input.threat_fresh,
        (unsigned)olgmd_output.near_gate_suppression,
        (double)olgmd_gate_depth_m);
  }
  if (olgmd_output.released) {
    ++gate_olgmd_release_count;
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
    if (paper_emergency_terminal_stop_latched) {
      dodge_state.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
      dodge_state.backtrack_settle_samples = 0u;
      dodge_state.emergency_forward_speed_mps = 0.0f;
      dodge_state.recovery_forward_speed_mps = 0.0f;
      dodge_state.recovery_forward_speed_active = false;
      memset(&dodge_intent, 0, sizeof(dodge_intent));
      dodge_intent.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
      olgmd_force_hold_anchor = true;
    }
#else
    tinyRacerDodgeReset(&dodge_state);
    memset(&dodge_intent, 0, sizeof(dodge_intent));
#endif
    DEBUG_PRINT("oLGMD brake released after five fresh clear frames\n");
  }
  if (olgmd_output.brake_latched) {
    /* Present the binary safety decision through the existing emergency path.
     * Reassertion keeps the generic FSM from advancing into backtrack while the
     * safety latch owns the reference. */
    navigation_intent.active = true;
    navigation_intent.new_sample = true;
    navigation_intent.forward_speed_mps = fmaxf(
        navigation_intent.forward_speed_mps,
        fmaxf(dodge_feedback.forward_speed_mps, 0.0f));
    navigation_intent.center_collision_probability = 1.0f;
    navigation_intent.collision_probability = 1.0f;
  }
#endif
  const TinyRacerDodgePhase previous_phase = dodge_state.phase;
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
  bool olgmd_rearmed_this_cycle = false;
  if (paper_emergency_terminal_stop_latched &&
      dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE &&
      !gate_olgmd_state.brake_latched) {
    ++gate_olgmd_stop_hold_cycles;
    if (gate_olgmd_stop_hold_cycles == 1u) {
      DEBUG_PRINT("oLGMD CLEAR_RESUME stationary hold started duration=%.1fs\n",
                  (double)TINYMPC_OLGMD_STOP_HOLD_S);
    }
    const uint32_t required_hold_cycles =
        (uint32_t)ceilf((float)TINYMPC_OLGMD_STOP_HOLD_S / DT);
    if (gate_olgmd_stop_hold_cycles >= required_hold_cycles) {
      paper_emergency_terminal_stop_latched = false;
      gate_olgmd_stop_hold_cycles = 0u;
      gate_olgmd_resume_refractory_cycles = (uint32_t)ceilf(
          (float)TINYMPC_OLGMD_RESUME_REFRACTORY_S / DT);
      tinyRacerDodgeReset(&dodge_state);
      memset(&dodge_intent, 0, sizeof(dodge_intent));
      olgmd_rearmed_this_cycle = true;
      DEBUG_PRINT(
          "oLGMD CLEAR_RESUME rearmed after %.1fs stationary hold refractory=%.1fs\n",
          (double)TINYMPC_OLGMD_STOP_HOLD_S,
          (double)TINYMPC_OLGMD_RESUME_REFRACTORY_S);
    }
  } else if (!paper_emergency_terminal_stop_latched) {
    gate_olgmd_stop_hold_cycles = 0u;
  }
#endif
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
  if (!paper_emergency_terminal_stop_latched &&
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
      !olgmd_rearmed_this_cycle &&
#endif
      navigation_intent.new_sample &&
      navigation_intent.center_collision_probability >
          dodge_config.emergency_brake_probability) {
    paper_emergency_terminal_stop_latched = true;
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
    pitch_through_brake_reference_active = true;
    pitch_through_brake_entry_altitude_world_m =
        active_local_frame.origin_z + x0(2);
    pitch_through_brake_internal_up_position_m = 0.0f;
    pitch_through_brake_internal_up_speed_mps = 0.0f;
    ++pitch_through_brake_trigger_count;
#if defined(TINYMPC_USE_ACTUATOR_LTI)
    resetLevelActuatorDuals();
#endif
    DEBUG_PRINT(
        "PITCH_THROUGH enter speed=%.3fm/s altitude=%.3fm pitch=%.3frad pitch_rate=%.3frad/s target=%.1f..%.1fdeg altitude_budget=%.2fm z_aiding=disabled internal_vertical_prediction=1 trigger=%lu\n",
        (double)emergency_signed_forward_speed_mps,
        (double)(active_local_frame.origin_z + x0(2)),
        (double)levelStatePitchRad(x0),
        (double)x0(10),
        (double)(pitch_through_brake_config.minimum_braking_pitch_rad *
            57.2957795f),
        (double)(pitch_through_brake_config.maximum_braking_pitch_rad *
            57.2957795f),
        (double)pitch_through_brake_config.maximum_altitude_loss_m,
        (unsigned long)pitch_through_brake_trigger_count);
#endif
    DEBUG_PRINT(
        "PAPER_EMERGENCY terminal stop latched center_risk=%.3f speed=%.3fm/s decel=%.1fm/s2 cache=%u pitch_through=%u\n",
        (double)navigation_intent.center_collision_probability,
        (double)dodge_feedback.forward_speed_mps,
        (double)dodge_config.emergency_brake_deceleration_mps2,
        (unsigned)TINYMPC_BRAKING_CACHE_ENABLE,
        (unsigned)TINYMPC_PITCH_THROUGH_BRAKE_ENABLE);
  }
  if (paper_emergency_terminal_stop_latched &&
      dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE) {
    /* The fixed measured-position anchor was captured on entry below.  Keep
     * emitting the stationary recovery profile without running the generic
     * settle/recovery transitions again. */
    memset(&dodge_intent, 0, sizeof(dodge_intent));
    dodge_intent.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
    dodge_intent.lateral_offset_m = dodge_state.lateral_offset_m;
    dodge_intent.avoidance_probability = fmaxf(
        dodge_state.center_collision_probability,
        fmaxf(dodge_state.left_collision_probability,
              dodge_state.right_collision_probability));
  } else {
#endif
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, DT,
      &dodge_intent);
#if TINYMPC_GATE_OLGMD_ENABLE
  if (gate_olgmd_state.brake_latched &&
      dodge_state.phase != TINYRACER_DODGE_EMERGENCY_BRAKE) {
    dodge_state.phase = TINYRACER_DODGE_EMERGENCY_BRAKE;
    dodge_state.emergency_forward_speed_mps = 0.0f;
    dodge_intent.phase = TINYRACER_DODGE_EMERGENCY_BRAKE;
    dodge_intent.forward_speed_mps = 0.0f;
    dodge_intent.lateral_rate_mps = 0.0f;
  }
#endif
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
    if (paper_emergency_terminal_stop_latched &&
        dodge_state.phase == TINYRACER_DODGE_BACKTRACK &&
        !(TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
              ? (fabsf(emergency_signed_forward_speed_mps) < 0.35f &&
                 fabsf(emergency_lateral_speed_mps) < 0.20f)
              : tinyMpcEmergencyStopReady(
                    emergency_signed_forward_speed_mps,
                    emergency_lateral_speed_mps, 0.10f,
                    levelStatePitchRad(x0), x0.segment<3>(9).norm(),
                    0.08726646f, 0.75f))) {
      /* The generic FSM's finite reference schedule has reached zero, but
       * the vehicle has not. Keep rebuilding the 6 m/s^2 horizon from
       * measured motion; otherwise STOP_HOLD captures a coasting pose. */
      dodge_state.phase = TINYRACER_DODGE_EMERGENCY_BRAKE;
      dodge_intent.phase = TINYRACER_DODGE_EMERGENCY_BRAKE;
      dodge_intent.forward_speed_mps = emergency_forward_speed_mps;
      dodge_intent.lateral_rate_mps = 0.0f;
    } else if (paper_emergency_terminal_stop_latched &&
        dodge_state.phase == TINYRACER_DODGE_BACKTRACK) {
      /* The generic FSM requests a retreat after the 6 m/s^2 schedule has
       * reached zero.  The paper emergency arm instead enters a permanent,
       * fixed-position zero-velocity hold at the measured stopping pose. */
      dodge_state.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
      dodge_state.backtrack_settle_samples = 0u;
      dodge_state.recovery_forward_speed_mps = 0.0f;
      dodge_state.recovery_forward_speed_active = false;
      dodge_intent.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
      dodge_intent.forward_speed_mps = 0.0f;
      dodge_intent.lateral_rate_mps = 0.0f;
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
      pitch_through_brake_reference_active = false;
      resetLevelActuatorDuals();
#endif
      DEBUG_PRINT(
          "PAPER_EMERGENCY transition EMERGENCY_BRAKE -> STOP_HOLD measured=(forward=%.3f lateral=%.3f)m/s pitch=%.3frad body_rate=%.3frad/s\n",
          (double)emergency_signed_forward_speed_mps,
          (double)emergency_lateral_speed_mps,
          (double)levelStatePitchRad(x0),
          (double)x0.segment<3>(9).norm());
    }
  }
#endif
  if (dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE &&
      (previous_phase != TINYRACER_DODGE_BACKTRACK_SETTLE
#if TINYMPC_OLGMD_CLEAR_RESUME_ENABLE
       || olgmd_force_hold_anchor
#endif
       )) {
    /* BACKTRACK is receding-horizon anchored at x0. Latch that same position
     * at the zero-speed handoff; returning to the frozen route anchor here
     * creates a multi-metre reference step and rails opposing motors. */
    const Eigen::Vector3f frame_origin_world(
        active_local_frame.origin_x,
        active_local_frame.origin_y,
        active_local_frame.origin_z);
    dodge_backtrack_settle_anchor_world = frame_origin_world +
        localVectorToWorld(active_local_frame, x0.head<3>());
    dodge_backtrack_settle_anchor_valid = true;
  } else if (dodge_state.phase != TINYRACER_DODGE_BACKTRACK_SETTLE) {
    dodge_backtrack_settle_anchor_valid = false;
  }
  /* During a dodge, route station belongs to the vehicle rather than to the
   * nominal speed clock.  Letting the virtual target retain its normal 0.5 m
   * lead while the vehicle slows for avoid/hold/rejoin makes the displaced
   * horizon progressively point ahead of the maneuver.  The resulting
   * position catch-up and low-speed velocity request conflict is the source
   * of the delayed high-bank departure seen after otherwise clean rejoins.
   * Keep the route phase physically anchored throughout every non-TRACK
   * state; the next update can then resume its ordinary bounded preview. */
  if (dodge_state.phase != TINYRACER_DODGE_TRACK) {
    const float discarded_route_lead_m = tinyMpcProgressPathDistance(
        &progress_path, progress_path.measured_progress,
        progress_path.progress);
    progress_path.progress = progress_path.measured_progress;
    if (previous_phase != dodge_state.phase &&
        discarded_route_lead_m > 0.01f) {
      DEBUG_PRINT(
          "DroNet maneuver route rebase phase=%d discarded_route_lead=%.3fm\n",
          (int)dodge_state.phase, (double)discarded_route_lead_m);
    }
  }
  const bool previous_rejoin_alignment =
      previous_phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
      previous_phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
  const bool current_rejoin_alignment =
      dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
      dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
  if (current_rejoin_alignment && !previous_rejoin_alignment) {
    dodge_rejoin_target_yaw_world_rad = prospective_rejoin_yaw_world_rad;
    dodge_rejoin_target_yaw_valid = true;
  } else if (!current_rejoin_alignment) {
    dodge_rejoin_target_yaw_valid = false;
  }
  const bool previous_loop_scan =
      previous_phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
      previous_phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT;
  const bool current_loop_scan =
      dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
      dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT;
  const bool previous_loop_escape =
      previous_phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
  const bool current_loop_escape =
      dodge_state.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
  const float loop_escape_progress_m = current_loop_escape &&
      dodge_state.encounter_progress_valid
      ? fmaxf(dodge_feedback.forward_progress_m -
                    dodge_state.encounter_start_forward_progress_m,
                0.0f)
      : 0.0f;
  const auto loop_escape_frenet_profile = [](
      float station_m, float *offset_m, float *slope) {
    const float length_m = fmaxf(
        dodge_config.loop_escape_spline_length_m, 1.0e-3f);
    const float t = T_MIN(T_MAX(station_m / length_m, 0.0f), 1.0f);
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float start_offset_m = dodge_state.lateral_offset_m;
    const float start_slope =
        (float)dodge_state.loop_escape_yaw_direction *
        tanf(dodge_config.loop_scan_yaw_rad);
    /* Cubic Hermite offset over route station: retain the clear scan tangent
     * at entry, and meet the nominal route with zero offset and zero slope.
     * Both position and direction are continuous at either end. */
    if (offset_m != nullptr) {
      *offset_m = (2.0f * t3 - 3.0f * t2 + 1.0f) * start_offset_m +
          (t3 - 2.0f * t2 + t) * length_m * start_slope;
    }
    if (slope != nullptr) {
      *slope = (6.0f * t2 - 6.0f * t) * start_offset_m / length_m +
          (3.0f * t2 - 4.0f * t + 1.0f) * start_slope;
    }
  };
  if (current_loop_scan && !previous_loop_scan) {
    dodge_loop_scan_route_yaw_world_rad = remainderf(
        active_local_frame.yaw_world + atan2f(path_local.y(), path_local.x()),
        6.28318530717958647692f);
    dodge_loop_scan_route_yaw_valid = true;
  } else if (!current_loop_scan && !current_loop_escape) {
    dodge_loop_scan_route_yaw_valid = false;
  }
  const auto camera_yaw_phase = [](TinyRacerDodgePhase phase) {
    return phase == TINYRACER_DODGE_AVOID_LEFT ||
        phase == TINYRACER_DODGE_AVOID_RIGHT ||
        phase == TINYRACER_DODGE_HOLD_LEFT ||
        phase == TINYRACER_DODGE_HOLD_RIGHT ||
        phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
        phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT ||
        phase == TINYRACER_DODGE_REJOIN_LEFT ||
        phase == TINYRACER_DODGE_REJOIN_RIGHT ||
        phase == TINYRACER_DODGE_REARM_LEFT ||
        phase == TINYRACER_DODGE_REARM_RIGHT ||
        phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
        phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT ||
        phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
        phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT ||
        phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
  };
  if (camera_yaw_phase(dodge_state.phase)) {
    if (!dodge_camera_yaw_initialized) {
      dodge_camera_yaw_world_rad = active_local_frame.yaw_world;
      dodge_camera_yaw_initialized = true;
    }
  } else {
    dodge_camera_yaw_initialized = false;
  }
  /* TRACK slowdown must govern the route generator itself. Limiting only the
   * velocity entries after the 0.5 m-ahead position horizon is built gives
   * TinyMPC contradictory acceleration and braking objectives. */
  vision_track_speed_limit_mps = dodge_state.phase == TINYRACER_DODGE_TRACK
      ? (navigation_intent.active
          ? fmaxf(navigation_intent.forward_speed_mps, 0.0f) : INFINITY)
      : dodge_state.phase == TINYRACER_DODGE_BACKTRACK ||
            dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE
          ? 0.0f : fmaxf(dodge_intent.forward_speed_mps, 0.0f);
  if (current_loop_escape) {
    float current_spline_offset_m = 0.0f;
    float current_spline_slope = 0.0f;
    loop_escape_frenet_profile(
        loop_escape_progress_m, &current_spline_offset_m,
        &current_spline_slope);
    float current_frenet_tangent_metric = 1.0f;
#if defined(TINYMPC_TRAJECTORY_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
    constexpr float loop_escape_circle_radius_m =
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
        3.60f;
#else
        0.75f;
#endif
    current_frenet_tangent_metric = T_MIN(T_MAX(
        (loop_escape_circle_radius_m - current_spline_offset_m) /
            loop_escape_circle_radius_m,
        0.25f), 2.0f);
#endif
    /* The maneuver cap applies to total velocity along the spline, not just
     * its route-tangent component. Feed the matching station rate into the
     * route generator so position and velocity previews remain consistent. */
    vision_track_speed_limit_mps = fminf(
        vision_track_speed_limit_mps,
        fmaxf(dodge_intent.forward_speed_mps, 0.0f) /
            sqrtf(current_frenet_tangent_metric *
                      current_frenet_tangent_metric +
                  current_spline_slope * current_spline_slope));
  }
  if (dodge_state.rejoin_spline_active &&
      (dodge_state.phase == TINYRACER_DODGE_REJOIN_LEFT ||
       dodge_state.phase == TINYRACER_DODGE_REJOIN_RIGHT)) {
    /* The maneuver cap is total speed along the merge, while the progress
     * generator advances in route station. Use the same conversion here that
     * the horizon velocity profile uses below. */
    float current_frenet_tangent_metric = 1.0f;
#if defined(TINYMPC_TRAJECTORY_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
    constexpr float current_circle_radius_m =
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
        3.60f;
#else
        0.75f;
#endif
    current_frenet_tangent_metric = T_MIN(T_MAX(
        (current_circle_radius_m - dodge_intent.lateral_offset_m) /
            current_circle_radius_m,
        0.25f), 2.0f);
#endif
    vision_track_speed_limit_mps = fminf(
        vision_track_speed_limit_mps,
        fmaxf(dodge_intent.forward_speed_mps, 0.0f) /
            sqrtf(current_frenet_tangent_metric *
                      current_frenet_tangent_metric +
                  dodge_state.rejoin_spline_slope *
                      dodge_state.rejoin_spline_slope));
  }
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
  const bool outward_right_phase =
      dodge_state.phase == TINYRACER_DODGE_AVOID_RIGHT ||
      dodge_state.phase == TINYRACER_DODGE_HOLD_RIGHT ||
      dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT ||
      dodge_state.phase == TINYRACER_DODGE_REJOIN_RIGHT;
  if (outward_right_phase && !dodge_state.rejoin_spline_active &&
      isfinite(vision_track_speed_limit_mps)) {
    const float most_outward_offset_m = fminf(
        dodge_intent.lateral_offset_m,
        -fmaxf(dodge_state.avoidance_target_offset_m, 0.0f));
    const float maximum_radius_scale = fmaxf(
        (3.60f - most_outward_offset_m) / 3.60f, 1.0f);
    vision_track_speed_limit_mps /= maximum_radius_scale;
  }
#endif
  const bool previous_recovery_handoff_phase =
      previous_phase == TINYRACER_DODGE_EMERGENCY_BRAKE ||
      previous_phase == TINYRACER_DODGE_BACKTRACK ||
      previous_phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
  const bool current_recovery_handoff_phase =
      dodge_state.phase == TINYRACER_DODGE_EMERGENCY_BRAKE ||
      dodge_state.phase == TINYRACER_DODGE_BACKTRACK ||
      dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
  const bool recovery_handoff_changed = previous_phase != dodge_state.phase &&
      (previous_recovery_handoff_phase || current_recovery_handoff_phase);
  const bool rejoin_alignment_handoff = previous_phase != dodge_state.phase &&
      (previous_phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
       previous_phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT ||
       dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
       dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT);
  const bool redirect_handoff = previous_phase != dodge_state.phase &&
      (previous_phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
       previous_phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT ||
       dodge_state.phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
       dodge_state.phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT);
  const bool loop_scan_handoff = previous_phase != dodge_state.phase &&
      (previous_phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
       previous_phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT ||
       previous_loop_escape ||
       dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
       dodge_state.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT ||
       current_loop_escape);
  if (recovery_handoff_changed || rejoin_alignment_handoff ||
      redirect_handoff || loop_scan_handoff ||
      (previous_phase != TINYRACER_DODGE_TRACK &&
       dodge_state.phase == TINYRACER_DODGE_TRACK)) {
#if TINYMPC_RATE_CASCADE
    /* Emergency recovery changes between braking, retreat, hold, and curved
     * route horizons. Discard dual bias at every such discontinuity. */
    resetOuterLoopDuals();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
    resetLevelActuatorDuals();
#endif
  }
  if ((!navigation_intent.active &&
       dodge_intent.phase == TINYRACER_DODGE_TRACK) ||
      race_intent.mode != TINYRACER_RACE_TRACK ||
      perception_halfspace_active || perception_recovery_active) {
    return;
  }
  /* DroNet owns encounter timing and pass-side selection. TinyMPC owns the
   * maneuver geometry: avoidance yaw follows its commanded motion; rejoin
   * and loop escape use tangent-continuous Frenet splines whose position,
   * velocity, and camera yaw share one derivative. */
#if TINYMPC_PATH_TUNNEL_ENABLE
  TinyMpcTunnelFrame previous_shifted_tunnel_frame = {};
#endif
  const bool recovery_reference_profile =
      dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE ||
      dodge_intent.phase == TINYRACER_DODGE_BACKTRACK ||
      dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
  const bool stationary_rejoin_scan =
      dodge_intent.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
      dodge_intent.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
  const bool stationary_loop_scan =
      dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
      dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT;
  const bool directional_loop_escape =
      dodge_intent.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
  const bool spline_rejoin =
      dodge_state.rejoin_spline_active &&
      (dodge_intent.phase == TINYRACER_DODGE_REJOIN_LEFT ||
       dodge_intent.phase == TINYRACER_DODGE_REJOIN_RIGHT);
  const bool stationary_camera_scan =
      stationary_rejoin_scan || stationary_loop_scan;
  const bool camera_yaw_reference_profile =
      camera_yaw_phase(dodge_intent.phase);
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
  const bool paper_terminal_stop_hold =
      paper_emergency_terminal_stop_latched &&
      dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
#else
  const bool paper_terminal_stop_hold = false;
#endif
  const Eigen::Vector3f recovery_anchor_local = Xref[0].head<3>();
  const float recovery_hold_yaw_rad = levelStateYawRad(Xref[0]);
  const struct quat recovery_level_attitude = rpy2quat(mkvec(
      0.0f, 0.0f, recovery_hold_yaw_rad));
  const float recovery_level_denominator =
      fabsf(recovery_level_attitude.w) > 1.0e-6f
      ? recovery_level_attitude.w
      : copysignf(1.0e-6f, recovery_level_attitude.w);
  TinyMpcProgressQuaternion camera_reference_attitudes[NHORIZON];
  TinyMpcProgressBodyRate camera_reference_body_rates[NHORIZON];
  float camera_reconstruction_residual_rad[NHORIZON - 1];
  float horizon_camera_yaw_world_rad = dodge_camera_yaw_world_rad;
  const bool motion_tangent_dodge =
      dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT ||
      dodge_intent.phase == TINYRACER_DODGE_AVOID_RIGHT ||
      spline_rejoin;
  const float camera_yaw_slew_rate_rad_s = motion_tangent_dodge
      ? 1.20f : directional_loop_escape ? 0.65f : 0.30f;
  float current_spline_slope = 0.0f;
  float current_spline_offset_m = 0.0f;
  if (directional_loop_escape) {
    loop_escape_frenet_profile(
        loop_escape_progress_m, &current_spline_offset_m,
        &current_spline_slope);
  }
  float current_loop_escape_tangent_metric = 1.0f;
#if defined(TINYMPC_TRAJECTORY_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
  constexpr float loop_escape_horizon_circle_radius_m =
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
      3.60f;
#else
      0.75f;
#endif
  current_loop_escape_tangent_metric = T_MIN(T_MAX(
      (loop_escape_horizon_circle_radius_m - current_spline_offset_m) /
          loop_escape_horizon_circle_radius_m,
      0.25f), 2.0f);
#endif
  const float loop_escape_route_speed_mps = directional_loop_escape
      ? fmaxf(dodge_intent.forward_speed_mps, 0.0f) /
          sqrtf(current_loop_escape_tangent_metric *
                    current_loop_escape_tangent_metric +
                current_spline_slope * current_spline_slope)
      : 0.0f;
  float loop_escape_horizon_station_m = loop_escape_progress_m;
  float rejoin_horizon_station_m = dodge_state.rejoin_spline_progress_m;
  float emergency_forward_distance_m = 0.0f;
  float emergency_horizon_pitch_rad = levelStatePitchRad(x0);
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
  const bool pitch_through_horizon_active =
      pitch_through_brake_reference_active &&
      dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE;
  if (pitch_through_horizon_active) {
    /* The solve frame is re-anchored at the vehicle each cycle, so zero is
     * the internally consistent vertical position. Replace deck-aided
     * vertical velocity with the maneuver's propagated short-horizon state. */
    x0(2) = 0.0f;
    x0(8) = pitch_through_brake_internal_up_speed_mps;
  }
  const Eigen::Vector3f pitch_through_anchor_local = x0.head<3>();
  const float pitch_through_horizon_origin_up_m =
      pitch_through_brake_internal_up_position_m;
  TinyMpcPitchThroughBrakeState pitch_through_horizon_state = {
      0.0f,
      pitch_through_brake_internal_up_position_m,
      emergency_signed_forward_speed_mps,
      pitch_through_brake_internal_up_speed_mps,
      levelStatePitchRad(x0),
      x0(10),
      0.0f,
  };
  TinyMpcPitchThroughBrakeState pitch_through_next_internal_state =
      pitch_through_horizon_state;
#else
  const bool pitch_through_horizon_active = false;
#endif
  for (int k = 0; k < NHORIZON; ++k) {
    /* Preserve the trajectory's analytical tangent at every prediction knot.
     * Reusing one horizon chord is exact on a straight, but turns a curved
     * bypass into a succession of locally straight velocity references and
     * removes the centripetal preview that the outer loop needs. */
    Eigen::Vector3f knot_path_local = Xref[k].segment<3>(6);
    knot_path_local.z() = 0.0f;
    const float nominal_forward_speed_mps = knot_path_local.head(2).norm();
    if (nominal_forward_speed_mps < 0.01f) {
      knot_path_local = path_local;
    } else {
      knot_path_local /= nominal_forward_speed_mps;
    }
    const Eigen::Vector3f lateral_local(
        -knot_path_local.y(), knot_path_local.x(), 0.0f);
    const Eigen::Vector3f profile_path_local =
        recovery_reference_profile || stationary_camera_scan
        ? path_local : knot_path_local;
    const Eigen::Vector3f profile_lateral_local =
        recovery_reference_profile || stationary_camera_scan
        ? route_lateral_local : lateral_local;
    float commanded_forward_speed_mps = nominal_forward_speed_mps;
    float route_station_speed_mps = nominal_forward_speed_mps;
    if (stationary_camera_scan) {
      Xref[k].head(3) = recovery_anchor_local;
      commanded_forward_speed_mps = 0.0f;
    } else if (directional_loop_escape &&
               nominal_forward_speed_mps < 0.01f) {
      /* On the scan-to-spline transition the route generator still contains
       * one stationary horizon. Seed that single preview from its route anchor;
       * subsequent updates come from the ordinary progress-path generator. */
      commanded_forward_speed_mps = loop_escape_route_speed_mps;
      route_station_speed_mps = loop_escape_route_speed_mps;
      Xref[k].head(3) = recovery_anchor_local + profile_path_local *
          (loop_escape_route_speed_mps * (float)k * DT);
    }
    if (dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE) {
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
      if (pitch_through_horizon_active) {
        const TinyMpcPitchThroughBrakeSample pitch_through_sample =
            tinyMpcPitchThroughAdvance(
                &pitch_through_brake_config,
                &pitch_through_horizon_state, DT);
        if (pitch_through_sample.valid) {
          pitch_through_horizon_state = pitch_through_sample.state;
          if (k == 0) {
            pitch_through_next_internal_state = pitch_through_sample.state;
          }
          commanded_forward_speed_mps =
              pitch_through_sample.state.forward_speed_mps;
          route_station_speed_mps = commanded_forward_speed_mps;
          Xref[k].head(3) = pitch_through_anchor_local +
              profile_path_local *
                  pitch_through_sample.state.forward_position_m +
              Eigen::Vector3f::UnitZ() *
                  (pitch_through_sample.state.up_position_m -
                      pitch_through_horizon_origin_up_m);
          const struct quat pitch_through_attitude = rpy2quat(mkvec(
              0.0f, pitch_through_sample.state.pitch_rad,
              recovery_hold_yaw_rad));
          const float pitch_through_attitude_denominator =
              fabsf(pitch_through_attitude.w) > 1.0e-6f
              ? pitch_through_attitude.w
              : copysignf(1.0e-6f, pitch_through_attitude.w);
          Xref[k](3) = pitch_through_attitude.x /
              pitch_through_attitude_denominator;
          Xref[k](4) = pitch_through_attitude.y /
              pitch_through_attitude_denominator;
          Xref[k](5) = pitch_through_attitude.z /
              pitch_through_attitude_denominator;
          Xref[k](8) = pitch_through_sample.state.up_speed_mps;
          Xref[k].segment<3>(9) <<
              0.0f, pitch_through_sample.state.pitch_rate_rad_s, 0.0f;
          if (k < NHORIZON - 1) {
            for (int motor = 0; motor < NINPUTS; ++motor) {
              Uref[k](motor) = T_MIN(T_MAX(
                  pitch_through_sample.motor_thrust_n[motor] -
                      TINYMPC_LEVEL_HOVER_THRUST_N,
                  lcu(motor)), ucu(motor));
            }
          }
        }
      } else
#endif
      {
      const float future_brake_speed_mps = fmaxf(
          dodge_intent.forward_speed_mps
              - dodge_config.emergency_brake_deceleration_mps2
                  * (float)k * DT,
          0.0f);
      commanded_forward_speed_mps = future_brake_speed_mps;
      if (k > 0) {
        const float previous_brake_speed_mps = fmaxf(
            dodge_intent.forward_speed_mps
                - dodge_config.emergency_brake_deceleration_mps2
                    * (float)(k - 1) * DT,
            0.0f);
        emergency_forward_distance_m += 0.5f *
            (previous_brake_speed_mps + future_brake_speed_mps) * DT;
      }
      Xref[k].head(3) = recovery_anchor_local + profile_path_local *
          emergency_forward_distance_m;
#if defined(TINYMPC_USE_ACTUATOR_LTI)
      const float measured_forward_speed_mps = fmaxf(
          x0.segment<3>(6).dot(profile_path_local), 0.0f);
      const float brake_schedule_speed_mps = fmaxf(
          future_brake_speed_mps,
          measured_forward_speed_mps
              - dodge_config.emergency_brake_deceleration_mps2
                  * (float)(k + 1) * DT);
      const float target_braking_pitch_rad =
          brake_schedule_speed_mps > 1.0e-4f
          ? tinyMpcEmergencyPitchForNetDeceleration(
              dodge_config.emergency_brake_deceleration_mps2,
              brake_schedule_speed_mps,
              TINYMPC_BANK_MODEL_DRAG_X_N_PER_MPS,
              TINYMPC_BANK_MODEL_MASS_KG, 9.81f)
          : 0.0f;
      /* A frozen braking chart is a steady equilibrium, not a transition
       * model.  The old emergency path stepped directly from level flight to
       * that equilibrium and produced 8--10 rad/s pitch motion plus opposing
       * motor saturation.  Keep this one-shot safety path on the level chart
       * and present it with a dynamically consistent, bounded pitch capture. */
      const float previous_horizon_pitch_rad = emergency_horizon_pitch_rad;
      emergency_horizon_pitch_rad = tinyMpcEmergencySlewAttitude(
          previous_horizon_pitch_rad, target_braking_pitch_rad,
          (float)TINYMPC_PAPER_EMERGENCY_PITCH_RATE_LIMIT_RAD_S, DT);
      const float pitch_step_rad =
          emergency_horizon_pitch_rad - previous_horizon_pitch_rad;
      const struct quat braking_attitude = rpy2quat(mkvec(
          0.0f, emergency_horizon_pitch_rad, recovery_hold_yaw_rad));
      const float attitude_denominator = fabsf(braking_attitude.w) > 1.0e-6f
          ? braking_attitude.w : copysignf(1.0e-6f, braking_attitude.w);
      Xref[k](3) = braking_attitude.x / attitude_denominator;
      Xref[k](4) = braking_attitude.y / attitude_denominator;
      Xref[k](5) = braking_attitude.z / attitude_denominator;
      Xref[k].segment<3>(9) <<
          0.0f, pitch_step_rad / DT, 0.0f;
      if (k < NHORIZON - 1) {
        const float cosine_pitch = fmaxf(
            cosf(emergency_horizon_pitch_rad), 0.10f);
        const float per_motor_thrust_n = T_MIN(
            TINYMPC_BANK_MODEL_MASS_KG * 9.81f /
                ((float)NINPUTS * cosine_pitch),
            TINYMPC_LEVEL_MAX_MOTOR_THRUST_N);
        const float collective_delta_n =
            per_motor_thrust_n - TINYMPC_LEVEL_HOVER_THRUST_N;
        for (int motor = 0; motor < NINPUTS; ++motor) {
          Uref[k](motor) = T_MIN(T_MAX(
              collective_delta_n, lcu(motor)), ucu(motor));
        }
      }
#else
      Xref[k].segment<3>(9).setZero();
#endif
      }
    } else if (dodge_intent.phase == TINYRACER_DODGE_BACKTRACK ||
               dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE) {
      commanded_forward_speed_mps = dodge_intent.forward_speed_mps;
      /* Active retreat is receding-horizon relative to the measured vehicle.
       * Anchoring it at the frozen route reference can leave knot zero ahead,
       * so position cost cancels the negative velocity request and hovers.
       * SETTLE holds the measured position latched at its entry; using moving
       * x0 with zero speed would remove its position restoring term. */
      const Eigen::Vector3f backtrack_anchor_local =
          dodge_intent.phase == TINYRACER_DODGE_BACKTRACK
          ? x0.head<3>()
          : dodge_backtrack_settle_anchor_valid
              ? worldVectorToLocal(
                    active_local_frame,
                    dodge_backtrack_settle_anchor_world - Eigen::Vector3f(
                        active_local_frame.origin_x,
                        active_local_frame.origin_y,
                        active_local_frame.origin_z))
              : x0.head<3>();
      if (paper_terminal_stop_hold) {
        const float signed_anchor_error_m = profile_path_local.dot(
            backtrack_anchor_local - x0.head<3>());
        commanded_forward_speed_mps = tinyMpcEmergencyHoldSpeedCommand(
            signed_anchor_error_m, emergency_signed_forward_speed_mps,
            1.5f, 1.25f, 0.30f);
        route_station_speed_mps = commanded_forward_speed_mps;
        Xref[k].head(3) = backtrack_anchor_local;
      } else {
        Xref[k].head(3) = backtrack_anchor_local + profile_path_local *
            (dodge_intent.forward_speed_mps * (float)k * DT);
      }
      Xref[k](3) = recovery_level_attitude.x / recovery_level_denominator;
      Xref[k](4) = recovery_level_attitude.y / recovery_level_denominator;
      Xref[k](5) = recovery_level_attitude.z / recovery_level_denominator;
      Xref[k].segment<3>(9).setZero();
    }
    const float loop_escape_offset_limit_m = dodge_state.loop_escape_active
        ? fmaxf(dodge_config.loop_escape_maximum_offset_m,
                dodge_config.maximum_lateral_offset_m)
        : dodge_config.maximum_lateral_offset_m;
    float spline_offset_m = 0.0f;
    float spline_slope = 0.0f;
    if (directional_loop_escape) {
      loop_escape_frenet_profile(
          loop_escape_horizon_station_m,
          &spline_offset_m, &spline_slope);
    }
    if (spline_rejoin) {
      tinyRacerFrenetSplineSample(
          rejoin_horizon_station_m,
          dodge_config.rejoin_spline_length_m,
          dodge_state.rejoin_spline_start_offset_m,
          dodge_state.rejoin_spline_start_slope,
          0.0f, 0.0f, &spline_offset_m, &spline_slope);
    }
    const float unconstrained_offset =
        directional_loop_escape || spline_rejoin
        ? spline_offset_m : dodge_intent.lateral_offset_m +
            dodge_intent.lateral_rate_mps * (float)k * DT;
    const float future_offset = T_MIN(T_MAX(
        unconstrained_offset, -loop_escape_offset_limit_m),
        loop_escape_offset_limit_m);
    float future_lateral_rate_mps =
        dodge_intent.phase == TINYRACER_DODGE_TRACK ||
            fabsf(future_offset - unconstrained_offset) > 1.0e-6f
        ? 0.0f : dodge_intent.lateral_rate_mps;
    Xref[k].head(3) += profile_lateral_local * future_offset;
    const Eigen::Vector3f maneuver_tunnel_center_local = Xref[k].head(3);
#if defined(TINYMPC_TRAJECTORY_CIRCLE) || \
    defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
    /* Every knot retains the nominal angular station but is displaced to
     * R-d, where positive left offset is inward on this circle. Therefore
     * both the finite-difference position speed and its centripetal bank scale
     * by (R-d)/R. Keeping nominal velocity/bank with displaced positions made
     * the reference internally inconsistent and over-banked the long inner
     * IMAV bypass until it spiralled inward. */
    constexpr float circle_radius_m =
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
        3.60f;
#else
        0.75f;
#endif
    const float displaced_radius_scale = T_MIN(T_MAX(
        (circle_radius_m - future_offset) / circle_radius_m,
        0.25f), 2.0f);
    float displaced_bank_scale = displaced_radius_scale;
    if (!recovery_reference_profile) {
      if (directional_loop_escape || spline_rejoin) {
        route_station_speed_mps = fminf(
            route_station_speed_mps,
            fmaxf(dodge_intent.forward_speed_mps, 0.0f) /
                sqrtf(displaced_radius_scale * displaced_radius_scale +
                      spline_slope * spline_slope));
        commanded_forward_speed_mps =
            route_station_speed_mps * displaced_radius_scale;
      } else {
        commanded_forward_speed_mps *= displaced_radius_scale;
      }
      if (dodge_intent.phase != TINYRACER_DODGE_TRACK &&
          !directional_loop_escape && !spline_rejoin) {
        commanded_forward_speed_mps = fminf(
            commanded_forward_speed_mps,
            fmaxf(dodge_intent.forward_speed_mps, 0.0f));
        route_station_speed_mps =
            commanded_forward_speed_mps / displaced_radius_scale;
      } else if (!directional_loop_escape && !spline_rejoin) {
        route_station_speed_mps = nominal_forward_speed_mps;
      }
      if (dodge_intent.phase != TINYRACER_DODGE_TRACK) {
        if (nominal_forward_speed_mps > 1.0e-4f) {
          const float speed_ratio =
              commanded_forward_speed_mps / nominal_forward_speed_mps;
          displaced_bank_scale = speed_ratio * speed_ratio /
              displaced_radius_scale;
        } else {
          displaced_bank_scale = 0.0f;
        }
      }
      Xref[k](3) *= displaced_bank_scale;
    }
#endif
#if !defined(TINYMPC_TRAJECTORY_CIRCLE) && \
    !defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
    if (!recovery_reference_profile && !paper_terminal_stop_hold &&
        dodge_intent.phase != TINYRACER_DODGE_TRACK) {
      if (directional_loop_escape || spline_rejoin) {
        route_station_speed_mps = fminf(
            route_station_speed_mps,
            fmaxf(dodge_intent.forward_speed_mps, 0.0f) /
                sqrtf(1.0f + spline_slope * spline_slope));
        commanded_forward_speed_mps = route_station_speed_mps;
      } else {
        commanded_forward_speed_mps = fminf(
            commanded_forward_speed_mps,
            fmaxf(dodge_intent.forward_speed_mps, 0.0f));
        route_station_speed_mps = commanded_forward_speed_mps;
      }
    }
#endif
    if (directional_loop_escape || spline_rejoin) {
      /* For p(s)=route(s)+d(s)n(s), dp/ds is
       * (1-kappa*d)t + d'(s)n. The tangent metric above is one on a straight
       * and (R-d)/R on the circle; d' always multiplies route-station speed. */
      future_lateral_rate_mps =
          route_station_speed_mps * spline_slope;
    }
    Xref[k](6) = profile_path_local.x() * commanded_forward_speed_mps +
        profile_lateral_local.x() * future_lateral_rate_mps;
    Xref[k](7) = profile_path_local.y() * commanded_forward_speed_mps +
        profile_lateral_local.y() * future_lateral_rate_mps;
    if (spline_rejoin && k + 1 < NHORIZON) {
      rejoin_horizon_station_m = fminf(
          rejoin_horizon_station_m + route_station_speed_mps * DT,
          dodge_config.rejoin_spline_length_m);
    }
    if (directional_loop_escape && k + 1 < NHORIZON) {
      loop_escape_horizon_station_m = fminf(
          loop_escape_horizon_station_m + route_station_speed_mps * DT,
          dodge_config.loop_escape_spline_length_m);
    }
    if (recovery_reference_profile && !pitch_through_horizon_active) {
      Xref[k](8) = 0.0f;
      progress_state_linear_cost[k].setZero();
    }
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
    if (pitch_through_horizon_active) {
      progress_state_linear_cost[k].setZero();
    }
#endif
    if (stationary_camera_scan) {
      Xref[k](8) = 0.0f;
      progress_state_linear_cost[k].setZero();
    }
    if (camera_yaw_reference_profile) {
      Eigen::Vector3f camera_direction_local(
          Xref[k](6), Xref[k](7), 0.0f);
      if (camera_direction_local.head<2>().norm() < 1.0e-4f) {
        camera_direction_local = profile_path_local;
      }
      if (dodge_intent.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
          dodge_intent.phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT) {
        const float direction =
            dodge_intent.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT
            ? -1.0f : 1.0f;
        camera_direction_local = profile_path_local *
                dodge_config.rejoin_forward_speed_mps +
            profile_lateral_local *
                (direction * dodge_config.rejoin_lateral_rate_mps);
      }
      float camera_target_yaw_world_rad =
          active_local_frame.yaw_world + atan2f(
              camera_direction_local.y(), camera_direction_local.x());
      if (stationary_rejoin_scan && dodge_rejoin_target_yaw_valid) {
        camera_target_yaw_world_rad = dodge_rejoin_target_yaw_world_rad;
      } else if (stationary_loop_scan) {
        camera_target_yaw_world_rad =
            (dodge_loop_scan_route_yaw_valid
                 ? dodge_loop_scan_route_yaw_world_rad
                 : camera_target_yaw_world_rad) +
            (dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT
                 ? 1.0f : -1.0f) * dodge_config.loop_scan_yaw_rad;
      }
      horizon_camera_yaw_world_rad = tinyMpcProgressSlewYawToward(
          horizon_camera_yaw_world_rad, camera_target_yaw_world_rad,
          camera_yaw_slew_rate_rad_s * DT);
      if (k == 0) {
        dodge_camera_yaw_world_rad = horizon_camera_yaw_world_rad;
      }
      const float camera_yaw_local_rad = remainderf(
          horizon_camera_yaw_world_rad - active_local_frame.yaw_world,
          6.28318530717958647692f);
      struct quat camera_attitude;
      if (stationary_camera_scan) {
        camera_attitude = qnormalize(rpy2quat(mkvec(
            0.0f, 0.0f, camera_yaw_local_rad)));
        const float denominator = fabsf(camera_attitude.w) > 1.0e-6f
            ? camera_attitude.w : copysignf(1.0e-6f, camera_attitude.w);
        Xref[k](3) = camera_attitude.x / denominator;
        Xref[k](4) = camera_attitude.y / denominator;
        Xref[k](5) = camera_attitude.z / denominator;
      } else {
        camera_attitude = setReferenceYawPreservingBodyZ(
            Xref[k], camera_yaw_local_rad);
      }
      camera_reference_attitudes[k] = tinyMpcProgressQuaternionMake(
          camera_attitude.x, camera_attitude.y,
          camera_attitude.z, camera_attitude.w);
      reference_yaw_unwrapped_rad[k] = horizon_camera_yaw_world_rad;
    }
#if TINYMPC_PATH_TUNNEL_ENABLE
    if (k > 0) {
      const Eigen::Vector3f tangent_world = localVectorToWorld(
          active_local_frame, knot_path_local);
      const TinyMpcTunnelVector tunnel_tangent = tinyMpcTunnelVector(
          tangent_world.x(), tangent_world.y(), tangent_world.z());
      const TinyMpcTunnelVector *previous_normal =
          previous_shifted_tunnel_frame.valid
          ? &previous_shifted_tunnel_frame.normal_1 : NULL;
      const TinyMpcTunnelFrame shifted_tunnel_frame = tinyMpcPathTunnelFrame(
          tunnel_tangent, previous_normal);
      if (shifted_tunnel_frame.valid) {
        const Eigen::Vector3f shifted_center_world(
            active_local_frame.origin_x,
            active_local_frame.origin_y,
            active_local_frame.origin_z);
        setPathTunnelHalfspaces(
            k, shifted_center_world + localVectorToWorld(
                active_local_frame, maneuver_tunnel_center_local),
            shifted_tunnel_frame);
        previous_shifted_tunnel_frame = shifted_tunnel_frame;
      }
    }
#endif
  }
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
  if (pitch_through_horizon_active) {
    /* This state is propagated from the commanded thrust/orientation model;
     * no optical-flow or downward-range altitude update enters the maneuver. */
    pitch_through_brake_internal_up_position_m =
        pitch_through_next_internal_state.up_position_m;
    pitch_through_brake_internal_up_speed_mps =
        pitch_through_next_internal_state.up_speed_mps;
  }
#endif
  if (camera_yaw_reference_profile) {
    tinyMpcProgressQuaternionHorizonBodyRates(
        camera_reference_attitudes, NHORIZON, DT,
        camera_reference_body_rates, camera_reconstruction_residual_rad);
    for (int k = 0; k < NHORIZON; ++k) {
      Xref[k].segment<3>(9) << camera_reference_body_rates[k].x,
          camera_reference_body_rates[k].y,
          camera_reference_body_rates[k].z;
    }
  }
#if defined(CONFIG_PLATFORM_SITL)
  static uint8_t rejoin_alignment_debug_steps = 0u;
  if (stationary_rejoin_scan) {
    if (++rejoin_alignment_debug_steps >= 50u) {
      rejoin_alignment_debug_steps = 0u;
      DEBUG_PRINT(
          "DroNet rejoin scan heading_error=%.1fdeg yaw_ref=%.1fdeg "
          "risk=%.3f clear_samples=%u\n",
          (double)(dodge_feedback.rejoin_heading_error_rad * 57.2957795f),
          (double)(dodge_camera_yaw_world_rad * 57.2957795f),
          (double)dodge_intent.avoidance_probability,
          (unsigned)dodge_state.rejoin_alignment_clear_samples);
    }
  } else {
    rejoin_alignment_debug_steps = 0u;
  }
  static uint8_t loop_scan_debug_steps = 0u;
  if (stationary_loop_scan) {
    if (++loop_scan_debug_steps >= 50u) {
      loop_scan_debug_steps = 0u;
      DEBUG_PRINT(
          "DroNet loop scan heading_error=%.1fdeg yaw_ref=%.1fdeg "
          "center_risk=%.3f samples=%u left=%.3f right=%.3f\n",
          (double)(dodge_feedback.loop_scan_heading_error_rad * 57.2957795f),
          (double)(dodge_camera_yaw_world_rad * 57.2957795f),
          (double)dodge_state.center_collision_probability,
          (unsigned)dodge_state.loop_scan_samples,
          (double)dodge_state.loop_scan_left_risk,
          (double)dodge_state.loop_scan_right_risk);
    }
  } else {
    loop_scan_debug_steps = 0u;
  }
  if (previous_phase != dodge_state.phase) {
    const char *phase_name = dodge_state.phase == TINYRACER_DODGE_AVOID_LEFT
        ? "AVOID_LEFT"
        : dodge_state.phase == TINYRACER_DODGE_AVOID_RIGHT
            ? "AVOID_RIGHT"
            : dodge_state.phase == TINYRACER_DODGE_HOLD_LEFT
                ? "HOLD_LEFT"
                : dodge_state.phase == TINYRACER_DODGE_HOLD_RIGHT
                    ? "HOLD_RIGHT"
                    : dodge_state.phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT
                        ? "REJOIN_ALIGN_LEFT"
                        : dodge_state.phase ==
                              TINYRACER_DODGE_REJOIN_ALIGN_RIGHT
                            ? "REJOIN_ALIGN_RIGHT"
                    : dodge_state.phase == TINYRACER_DODGE_REJOIN_LEFT
                        ? "REJOIN_LEFT"
                        : dodge_state.phase == TINYRACER_DODGE_REJOIN_RIGHT
                            ? "REJOIN_RIGHT"
                            : dodge_state.phase == TINYRACER_DODGE_REARM_LEFT
                                ? "REARM_LEFT"
                                : dodge_state.phase ==
                                      TINYRACER_DODGE_REARM_RIGHT
                                    ? "REARM_RIGHT"
                                    : dodge_state.phase ==
                                          TINYRACER_DODGE_REDIRECT_PREP_LEFT
                                        ? "REDIRECT_PREP_LEFT"
                                        : dodge_state.phase ==
                                              TINYRACER_DODGE_REDIRECT_PREP_RIGHT
                                            ? "REDIRECT_PREP_RIGHT"
                                    : dodge_state.phase ==
                                          TINYRACER_DODGE_EMERGENCY_BRAKE
                                        ? "EMERGENCY_BRAKE"
                                        : dodge_state.phase ==
                                              TINYRACER_DODGE_BACKTRACK
                                            ? "BACKTRACK"
                                            : dodge_state.phase ==
                                                  TINYRACER_DODGE_LOOP_SCAN_LEFT
                                                ? "LOOP_SCAN_LEFT"
                                                : dodge_state.phase ==
                                                      TINYRACER_DODGE_LOOP_SCAN_RIGHT
                                                    ? "LOOP_SCAN_RIGHT"
                                            : dodge_state.phase ==
                                                  TINYRACER_DODGE_BACKTRACK_SETTLE
                                                ? "BACKTRACK_SETTLE"
                                                : dodge_state.phase ==
                                                      TINYRACER_DODGE_LOOP_ESCAPE_FORWARD
                                                    ? "LOOP_ESCAPE_FORWARD"
                                                    : "TRACK";
    DEBUG_PRINT(
        "DroNet avoidance state=%s risk=%.3f rate=%.3fm/s offset=%.3fm "
        "target=%.3fm backtrack_avg=%.3f samples=%u cycles=%u "
        "scan_left=%.3f scan_right=%.3f rejoin_heading_error=%.1fdeg\n",
        phase_name, (double)dodge_intent.avoidance_probability,
        (double)dodge_intent.lateral_rate_mps,
        (double)dodge_intent.lateral_offset_m,
        (double)dodge_state.avoidance_target_offset_m,
        (double)dodge_state.backtrack_risk_average,
        (unsigned)dodge_state.backtrack_risk_window_count,
        (unsigned)dodge_state.backtrack_cycle_count,
        (double)dodge_state.loop_scan_left_risk,
        (double)dodge_state.loop_scan_right_risk,
        (double)(dodge_feedback.rejoin_heading_error_rad * 57.2957795f));
  }
#endif
}

static void __attribute__((unused)) applyVisionResidualReference(
    const TinyRacerPerceptionObservation& observation) {
#if TINYMPC_VISION_RL_RESIDUAL_ENABLE
  const bool usable = observation.valid && observation.has_residual_reference &&
      observation.received_age_ms <= race_config.maximum_age_ms;
  if (!usable) {
    vision_residual_lateral_offset_m = 0.0f;
    vision_residual_vertical_offset_m = 0.0f;
    vision_residual_progress_speed_scale = 1.0f;
    return;
  }
  const float lateral_rate_mps = tinyMpcVisionResidualClampSymmetric(
      observation.lateral_reference_rate_mps,
      vision_residual_authority.lateral_rate_limit_mps);
  const float vertical_rate_mps = tinyMpcVisionResidualClampSymmetric(
      observation.vertical_reference_rate_mps,
      vision_residual_authority.vertical_rate_limit_mps);
  /* While a geometrically admitted gate occupies the view, reserve vehicle
   * body/rotor margin inside the 0.90 m NewBee opening. Hold that envelope
   * across the short near-fill detector dropout at the gate plane, then make
   * the full obstacle-bypass envelope available again. */
  static uint32_t last_gate_envelope_sample = UINT32_MAX;
  static uint8_t gate_envelope_dropout_samples = UINT8_MAX;
  static uint16_t gate_envelope_age_samples = 0u;
  static bool gate_envelope_started = false;
  static bool gate_envelope_completed = false;
  if (observation.sample != last_gate_envelope_sample) {
    last_gate_envelope_sample = observation.sample;
    if (!gate_envelope_started && !gate_envelope_completed &&
        observation.gate_valid && observation.gate_confidence >= 0.50f) {
      gate_envelope_started = true;
      gate_envelope_dropout_samples = 0u;
    } else if (gate_envelope_started) {
      if (observation.gate_valid && observation.gate_confidence >= 0.50f) {
        gate_envelope_dropout_samples = 0u;
      } else if (gate_envelope_dropout_samples < UINT8_MAX) {
        ++gate_envelope_dropout_samples;
      }
    }
    if (gate_envelope_started && gate_envelope_age_samples < UINT16_MAX) {
      ++gate_envelope_age_samples;
    }
    if (gate_envelope_started &&
        (gate_envelope_dropout_samples > 45u ||
         gate_envelope_age_samples > 210u)) {
      gate_envelope_started = false;
      gate_envelope_completed = true;
    }
  }
  const bool gate_envelope_active = gate_envelope_started;
  const float lateral_offset_limit_m =
      tinyMpcVisionResidualLateralOffsetLimit(
          &vision_residual_authority, gate_envelope_active);
  vision_residual_progress_speed_scale = T_MIN(T_MAX(
      observation.progress_speed_scale,
      vision_residual_authority.minimum_progress_speed_scale),
      vision_residual_authority.maximum_progress_speed_scale);
  const float proposed_lateral_offset_m =
      vision_residual_lateral_offset_m + lateral_rate_mps * DT;
  const float proposed_vertical_offset_m =
      vision_residual_vertical_offset_m + vertical_rate_mps * DT;
  vision_residual_lateral_offset_m = T_MIN(T_MAX(
      proposed_lateral_offset_m, -lateral_offset_limit_m), lateral_offset_limit_m);
  vision_residual_vertical_offset_m = tinyMpcVisionResidualClampSymmetric(
      proposed_vertical_offset_m,
      vision_residual_authority.vertical_offset_limit_m);
  /* Once an integrated reference reaches its safety envelope, remove only
   * the outward velocity component. Retaining the raw rate at a clamped
   * position asks TinyMPC to keep accelerating through its own bound. */
  const float effective_lateral_rate_mps =
      fabsf(proposed_lateral_offset_m - vision_residual_lateral_offset_m) < 1.0e-6f
      ? lateral_rate_mps : 0.0f;
  const float effective_vertical_rate_mps =
      fabsf(proposed_vertical_offset_m - vision_residual_vertical_offset_m) < 1.0e-6f
      ? vertical_rate_mps : 0.0f;

  Eigen::Vector3f fallback_tangent_local = Xref[0].segment<3>(6);
  if (fallback_tangent_local.norm() < 0.01f) {
    fallback_tangent_local =
        Xref[NHORIZON - 1].head<3>() - Xref[0].head<3>();
  }
  if (fallback_tangent_local.norm() < 0.01f) {
    fallback_tangent_local = Eigen::Vector3f::UnitX();
  } else {
    fallback_tangent_local.normalize();
  }
#if TINYMPC_PATH_TUNNEL_ENABLE
  TinyMpcTunnelFrame previous_residual_tunnel_frame = {};
#endif
  for (int k = 0; k < NHORIZON; ++k) {
    const Eigen::Vector3f nominal_velocity_local = Xref[k].segment<3>(6);
    Eigen::Vector3f tangent_local = nominal_velocity_local;
    if (tangent_local.norm() < 0.01f) {
      tangent_local = fallback_tangent_local;
    } else {
      tangent_local.normalize();
    }
    Eigen::Vector3f horizontal_tangent = tangent_local;
    horizontal_tangent.z() = 0.0f;
    if (horizontal_tangent.head<2>().norm() < 0.01f) {
      horizontal_tangent = fallback_tangent_local;
      horizontal_tangent.z() = 0.0f;
    }
    if (horizontal_tangent.head<2>().norm() < 0.01f) {
      horizontal_tangent = Eigen::Vector3f::UnitX();
    } else {
      horizontal_tangent.normalize();
    }
    const Eigen::Vector3f normal_local(
        -horizontal_tangent.y(), horizontal_tangent.x(), 0.0f);
    const float lateral_offset_m = T_MIN(T_MAX(
        vision_residual_lateral_offset_m + effective_lateral_rate_mps * (float)k * DT,
        -lateral_offset_limit_m), lateral_offset_limit_m);
    const float vertical_offset_m = tinyMpcVisionResidualClampSymmetric(
        vision_residual_vertical_offset_m + effective_vertical_rate_mps * (float)k * DT,
        vision_residual_authority.vertical_offset_limit_m);
    Xref[k].head<3>() += normal_local * lateral_offset_m;
    Xref[k](2) += vertical_offset_m;
    /* The scalar is consumed by updateHorizonReference() on the next 50 Hz
     * solve. The nominal velocity here was already built with the previously
     * latched scalar, so multiplying it again would square the command. */
    Xref[k].segment<3>(6) = nominal_velocity_local
        + normal_local * effective_lateral_rate_mps
        + Eigen::Vector3f::UnitZ() * effective_vertical_rate_mps;
#if TINYMPC_PATH_TUNNEL_ENABLE
    if (k > 0) {
      const Eigen::Vector3f tangent_world = localVectorToWorld(
          active_local_frame, tangent_local);
      const TinyMpcTunnelVector tunnel_tangent = tinyMpcTunnelVector(
          tangent_world.x(), tangent_world.y(), tangent_world.z());
      const TinyMpcTunnelVector *previous_normal =
          previous_residual_tunnel_frame.valid
          ? &previous_residual_tunnel_frame.normal_1 : NULL;
      const TinyMpcTunnelFrame shifted_tunnel_frame = tinyMpcPathTunnelFrame(
          tunnel_tangent, previous_normal);
      if (shifted_tunnel_frame.valid) {
        const Eigen::Vector3f frame_origin_world(
            active_local_frame.origin_x, active_local_frame.origin_y,
            active_local_frame.origin_z);
        setPathTunnelHalfspaces(
            k, frame_origin_world + localVectorToWorld(
                active_local_frame, Xref[k].head<3>()),
            shifted_tunnel_frame);
        previous_residual_tunnel_frame = shifted_tunnel_frame;
      }
    }
#endif
  }
#if defined(CONFIG_PLATFORM_SITL)
  static uint32_t last_residual_sample = UINT32_MAX;
  if (observation.sample != last_residual_sample) {
    last_residual_sample = observation.sample;
    DEBUG_PRINT(
        "Vision residual sample=%lu rate=(%.3f,%.3f)m/s offset=(%.3f,%.3f)m speed_scale=%.3f\n",
        (unsigned long)observation.sample, (double)lateral_rate_mps,
        (double)vertical_rate_mps, (double)vision_residual_lateral_offset_m,
        (double)vision_residual_vertical_offset_m,
        (double)vision_residual_progress_speed_scale);
  }
#endif
#else
  (void)observation;
#endif
}

static void __attribute__((unused)) applyVerticalActiveSensingReference(
    uint32_t flight_tick) {
#if TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE
  const float elapsed_flight_time_s =
      (float)(flight_tick - vertical_active_sensing_start_tick) /
      (float)configTICK_RATE_HZ;
  for (int k = 0; k < NHORIZON; ++k) {
    Xref[k](2) += tinyMpcVerticalActiveSensingOffset(
        &vertical_active_sensing_config,
        elapsed_flight_time_s + (float)k * DT);
  }
#else
  (void)flight_tick;
#endif
}

static void __attribute__((unused)) applyGateVisualServo(
    const TinyRacerPerceptionObservation& observation) {
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  (void)observation;
  bool exit_blend = gate_visual_phase == GATE_VISUAL_REARM;
  const bool approach_active = gate_poc_associated &&
      gate_visual_center_valid &&
      (gate_visual_phase == GATE_VISUAL_ALIGN ||
       gate_visual_phase == GATE_VISUAL_TRANSIT);
  if ((!approach_active && !exit_blend) || !tinyRacerGateServoAllowed(
          race_intent.mode, dodge_intent.phase,
          perception_halfspace_active, perception_recovery_active)) {
    return;
  }

  const Eigen::Vector3f position_world(
      active_local_frame.origin_x,
      active_local_frame.origin_y,
      active_local_frame.origin_z);
  Eigen::Vector3f to_center = gate_visual_center_world - position_world;
  if (gate_visual_phase == GATE_VISUAL_TRANSIT) {
    const Eigen::Vector3f measured_velocity_world = localVectorToWorld(
        active_local_frame, x0.segment<3>(6));
    const float measured_gate_speed_mps =
        gate_visual_forward_world.dot(measured_velocity_world);
    const struct quat measured_attitude = referenceStateQuaternion(x0);
    const float measured_tilt_rad = acosf(T_MIN(T_MAX(
        1.0f - 2.0f * (measured_attitude.x * measured_attitude.x +
                       measured_attitude.y * measured_attitude.y),
        -1.0f), 1.0f));
    const float measured_body_rate_rad_s = x0.segment<3>(9).norm();
    /* Gate traversal is optional. If the vehicle stops being local to the
     * gentle transit model, relinquish the visual overlay and recover onto
     * the route instead of continuing to chase the opening. This guard also
     * restores collision authority while there is still attitude margin. */
    const bool transit_locality_lost =
        measured_gate_speed_mps > 0.60f ||
        measured_tilt_rad > 0.1745329252f ||  // 10 deg.
        measured_body_rate_rad_s > 0.75f;
    if (transit_locality_lost) {
      gate_visual_phase = GATE_VISUAL_REARM;
      exit_blend = true;
      gate_poc_associated = false;
      gate_visual_center_valid = false;
      gate_visual_rearm_clear_samples = 0u;
      gate_poc_consecutive_samples = 0u;
      gate_poc_dropout_steps = 0u;
#if TINYMPC_RATE_CASCADE
      resetOuterLoopDuals();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
      resetLevelActuatorDuals();
#endif
      DEBUG_PRINT(
          "Gate visual transit aborted speed=%.2fm/s tilt=%.1fdeg body_rate=%.2frad/s; returning to route\n",
          (double)measured_gate_speed_mps,
          (double)(measured_tilt_rad * 57.2957795f),
          (double)measured_body_rate_rad_s);
    }
  }
  if (gate_visual_phase == GATE_VISUAL_TRANSIT &&
      gate_visual_forward_world.dot(to_center) < -0.55f) {
    gate_visual_phase = GATE_VISUAL_REARM;
    exit_blend = true;
    gate_poc_associated = false;
    gate_visual_center_valid = false;
    gate_visual_rearm_clear_samples = 0u;
    gate_poc_consecutive_samples = 0u;
    gate_poc_dropout_steps = 0u;
#if TINYMPC_RATE_CASCADE
    resetOuterLoopDuals();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
    resetLevelActuatorDuals();
#endif
    DEBUG_PRINT("Gate visual transit complete; returning to route\n");
  }

  Eigen::Vector3f forward_world = gate_visual_forward_world;
  Eigen::Vector3f desired_velocity_world = Eigen::Vector3f::Zero();
  float forward_speed_mps = 0.0f;
  if (exit_blend) {
    /* Xref still contains the freshly generated nominal route horizon here.
     * Blend back to its velocity and tangent heading before relinquishing the
     * gate overlay. A direct heading snap can leave the level linearization's
     * local region even though both headings are individually valid. */
    desired_velocity_world = localVectorToWorld(
        active_local_frame, Xref[0].segment<3>(6));
    forward_speed_mps = desired_velocity_world.head<2>().norm();
    if (forward_speed_mps > 0.05f) {
      Eigen::Vector3f route_forward_world = desired_velocity_world;
      route_forward_world.z() = 0.0f;
      route_forward_world.normalize();
      const Eigen::Vector3f route_position_error_world = localVectorToWorld(
          active_local_frame, Xref[0].head<3>());
      const float route_along_error_m =
          route_forward_world.dot(route_position_error_world);
      Eigen::Vector3f route_cross_track_error_world =
          route_position_error_world -
          route_forward_world * route_along_error_m;
      route_cross_track_error_world.z() = 0.0f;
      Eigen::Vector3f route_capture_velocity_world =
          1.2f * route_cross_track_error_world;
      constexpr float maximum_gate_rejoin_lateral_speed_mps = 0.20f;
      if (route_capture_velocity_world.head<2>().norm() >
          maximum_gate_rejoin_lateral_speed_mps) {
        route_capture_velocity_world *=
            maximum_gate_rejoin_lateral_speed_mps /
            route_capture_velocity_world.head<2>().norm();
      }
      constexpr float maximum_gate_rejoin_vertical_speed_mps = 0.15f;
      route_capture_velocity_world.z() = T_MIN(T_MAX(
          1.2f * route_position_error_world.z(),
          -maximum_gate_rejoin_vertical_speed_mps),
          maximum_gate_rejoin_vertical_speed_mps);
      desired_velocity_world += route_capture_velocity_world;
      Eigen::Vector3f rejoin_forward_world = desired_velocity_world;
      rejoin_forward_world.z() = 0.0f;
      if (rejoin_forward_world.head<2>().norm() > 0.05f) {
        rejoin_forward_world.normalize();
      } else {
        rejoin_forward_world = route_forward_world;
      }
      const float route_yaw_world_rad = atan2f(
          rejoin_forward_world.y(), rejoin_forward_world.x());
      const float retained_gate_yaw_world_rad = atan2f(
          gate_visual_forward_world.y(), gate_visual_forward_world.x());
      constexpr float gate_rejoin_yaw_rate_rad_s = 0.30f;
      const float blended_yaw_world_rad = tinyMpcProgressSlewYawToward(
          retained_gate_yaw_world_rad, route_yaw_world_rad,
          gate_rejoin_yaw_rate_rad_s * DT);
      forward_world = Eigen::Vector3f(
          cosf(blended_yaw_world_rad), sinf(blended_yaw_world_rad), 0.0f);
      gate_visual_forward_world = forward_world;
    }
  } else {
    /* Keep turning with the nominal course while centering the detected
     * opening. This approaches a tangent gate normal to its plane instead of
     * freezing the chord heading from the first detection. */
    Eigen::Vector3f route_forward_world = localVectorToWorld(
        active_local_frame, Xref[0].segment<3>(6));
    route_forward_world.z() = 0.0f;
    if (route_forward_world.head<2>().norm() > 0.10f) {
      forward_world = route_forward_world.normalized();
      gate_visual_forward_world = forward_world;
    }
    const float along_m = forward_world.dot(to_center);
    Eigen::Vector3f cross_track_world = to_center - forward_world * along_m;
    cross_track_world.z() = 0.0f;
    Eigen::Vector3f lateral_velocity_world = 1.8f * cross_track_world;
    constexpr float maximum_lateral_speed_mps = 0.45f;
    if (lateral_velocity_world.head<2>().norm() > maximum_lateral_speed_mps) {
      lateral_velocity_world *= maximum_lateral_speed_mps /
          lateral_velocity_world.head<2>().norm();
    }
    /* The 0.4 m IMAV opening leaves little margin while the monocular center
     * estimate is still converging. Keep the proven alignment speed through
     * the plane so lateral correction cannot be outrun by the transit. */
    forward_speed_mps = gate_visual_phase == GATE_VISUAL_ALIGN
        ? 0.15f : 0.35f;
    const float vertical_speed_mps = T_MIN(T_MAX(
        1.8f * to_center.z(), -0.30f), 0.30f);
    desired_velocity_world =
        forward_world * forward_speed_mps + lateral_velocity_world +
        Eigen::Vector3f::UnitZ() * vertical_speed_mps;
  }
  if (!gate_visual_velocity_valid) {
    gate_visual_velocity_world = localVectorToWorld(
        active_local_frame, x0.segment<3>(6));
    gate_visual_velocity_valid = true;
  }
  const Eigen::Vector3f velocity_delta =
      desired_velocity_world - gate_visual_velocity_world;
  constexpr float maximum_gate_acceleration_mps2 = 0.50f;
  const float maximum_velocity_step_mps = maximum_gate_acceleration_mps2 * DT;
  if (velocity_delta.norm() > maximum_velocity_step_mps) {
    gate_visual_velocity_world +=
        velocity_delta.normalized() * maximum_velocity_step_mps;
  } else {
    gate_visual_velocity_world = desired_velocity_world;
  }
  const Eigen::Vector3f velocity_world = gate_visual_velocity_world;
  const float yaw_world = forward_world.head<2>().norm() > 0.05f
      ? atan2f(forward_world.y(), forward_world.x())
      : active_local_frame.yaw_world;
  const struct quat attitude_world_body = rpy2quat(
      mkvec(0.0f, 0.0f, yaw_world));

  if (!exit_blend) {
    vision_track_speed_limit_mps = T_MIN(
        vision_track_speed_limit_mps, forward_speed_mps);
  }
#if TINYMPC_PATH_TUNNEL_ENABLE
  TinyMpcTunnelFrame previous_gate_tunnel_frame = {};
#endif
  for (int k = 0; k < NHORIZON; ++k) {
    const Eigen::Vector3f reference_position_world = position_world +
        velocity_world * ((float)(k + 1) * DT);
    setLocalReferenceState(
        Xref[k], reference_position_world, attitude_world_body,
        velocity_world, Eigen::Vector3f::Zero());
    reference_yaw_unwrapped_rad[k] = yaw_world;
    if (k < NHORIZON - 1) {
      Uref[k].setZero();
    }
#if TINYMPC_PATH_TUNNEL_ENABLE
    if (k > 0) {
      const TinyMpcTunnelVector tangent = tinyMpcTunnelVector(
          forward_world.x(), forward_world.y(), forward_world.z());
      const TinyMpcTunnelVector *previous_normal =
          previous_gate_tunnel_frame.valid
          ? &previous_gate_tunnel_frame.normal_1 : NULL;
      const TinyMpcTunnelFrame frame = tinyMpcPathTunnelFrame(
          tangent, previous_normal);
      if (frame.valid) {
        setPathTunnelHalfspaces(k, reference_position_world, frame);
        previous_gate_tunnel_frame = frame;
      }
    }
#endif
  }
  return;
#else
  const bool fresh_gate = observation.valid && observation.gate_valid &&
      observation.received_age_ms <= race_config.maximum_age_ms &&
      observation.gate_confidence >= 0.25f;
  if (!tinyRacerGateServoAllowed(
      race_intent.mode, dodge_intent.phase,
      perception_halfspace_active, perception_recovery_active)) {
    return;
  }
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
  if (!gate_poc_associated) {
    return;
  }
#else
  if (!fresh_gate) {
    return;
  }
#endif
  /* Gate image bearing is lateral in the camera/body frame. The solve-local
   * frame follows current vehicle yaw, so its Y axis is the yaw-level camera
   * lateral approximation at the first knot. Project that vector and the
   * nominal path displacement onto the first Frenet normal before comparing
   * them. Keep the tangent as a fallback for stopped terminal knots. */
  Eigen::Vector3f gate_path_tangent_local = Xref[0].segment<3>(6);
  gate_path_tangent_local.z() = 0.0f;
  if (gate_path_tangent_local.head<2>().norm() < 0.01f) {
    gate_path_tangent_local =
        Xref[NHORIZON - 1].head<3>() - Xref[0].head<3>();
    gate_path_tangent_local.z() = 0.0f;
  }
  if (gate_path_tangent_local.head<2>().norm() < 0.01f) {
    gate_path_tangent_local = Eigen::Vector3f::UnitX();
  } else {
    gate_path_tangent_local.normalize();
  }
  const Eigen::Vector3f gate_path_normal_local(
      -gate_path_tangent_local.y(), gate_path_tangent_local.x(), 0.0f);
  if (fresh_gate) {
    const float *corner = observation.gate_corners_xy;
    const float width = 0.5f * ((corner[2] - corner[0]) +
                                (corner[4] - corner[6]));
    const float height = 0.5f * ((corner[7] - corner[1]) +
                                 (corner[5] - corner[3]));
    const float fx_normalized = observation.gate_fx_normalized;
    const float fy_normalized = observation.gate_fy_normalized;
    const float cx_normalized = observation.gate_cx_normalized;
    const float cy_normalized = observation.gate_cy_normalized;
    if (width >= 0.05f && height >= 0.05f && width * height >= 0.01f &&
        fx_normalized >= 0.05f && fy_normalized >= 0.05f &&
        cx_normalized >= 0.0f && cx_normalized <= 1.0f &&
        cy_normalized >= 0.0f && cy_normalized <= 1.0f &&
        observation.sample != gate_filter_sample) {
      const float center_x = 0.25f *
          (corner[0] + corner[2] + corner[4] + corner[6]);
      const float center_y = 0.25f *
          (corner[1] + corner[3] + corner[5] + corner[7]);
      /* Corners supervise real gate rail centers, not its collision-free
       * opening. Isaac training metadata specifies their 0.555 m span; use
       * that fixed detected-class dimension only to scale image bearing into
       * a bounded reference offset, never as a world gate pose. */
      constexpr float gate_corner_span_m = TINYMPC_GATE_CORNER_SPAN_M;
      const float estimated_depth_m = T_MIN(T_MAX(
          gate_corner_span_m * fx_normalized / width, 0.4f), 4.0f);
      const float requested_lateral_m = T_MIN(T_MAX(
          -(center_x - cx_normalized) * estimated_depth_m / fx_normalized,
          -0.35f), 0.35f);
      const float requested_vertical_m = T_MIN(T_MAX(
          -(center_y - cy_normalized) * estimated_depth_m / fy_normalized,
          -0.25f), 0.25f);
      /* requested_* is a camera-relative displacement from the vehicle to
       * the gate centre. Xref[0] is the nominal path displacement from that
       * same vehicle-centred local-frame origin. Compare their scalar
       * components on the current path normal. Using Xref[0](1) directly is a
       * hidden straight-course assumption and rotates the correction into the
       * wrong world direction on circle/oval/figure-eight references. */
      constexpr float maximum_lateral_path_shift_m = 0.35f;
      const Eigen::Vector3f requested_gate_displacement_local(
          0.0f, requested_lateral_m, 0.0f);
      const float requested_lateral_shift_m = T_MIN(T_MAX(
          gate_path_normal_local.dot(
              requested_gate_displacement_local - Xref[0].head<3>()),
          -maximum_lateral_path_shift_m), maximum_lateral_path_shift_m);
      /* The course path already provides the gate's nominal height.  Preserve
       * a small upward crossing margin for the altitude lost while banking
       * laterally, and let the learned bearing add only a bounded correction.
       * Never allow a near-fill corner outlier to pull the horizon downward
       * into the bottom rail. */
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
      /* The regenerated IMAV route already crosses the 0.40 m opening center.
       * Keep correction symmetric and small; the older forced upward bias
       * consumed nearly all of this gate's vertical envelope. */
      constexpr float minimum_vertical_path_shift_m = -0.04f;
      constexpr float maximum_vertical_path_shift_m = 0.04f;
#elif TINYMPC_JOINT_GATE_RL_ENABLE
      /* Development seeds on the physical 0.45 m NewBee opening showed that
       * banking/vertical lag could consume the entire lower-rail margin.  The
       * joint experiment therefore carries 4 cm more upward feedforward.  It
       * remains bounded and opt-in; ordinary and legacy POC builds retain the
       * original reference behavior below. */
      constexpr float minimum_vertical_path_shift_m = 0.12f;
      constexpr float maximum_vertical_path_shift_m = 0.16f;
#else
      constexpr float minimum_vertical_path_shift_m = 0.08f;
      constexpr float maximum_vertical_path_shift_m = 0.12f;
#endif
      const float requested_vertical_shift_m = T_MIN(T_MAX(
          requested_vertical_m - Xref[0](2),
          minimum_vertical_path_shift_m), maximum_vertical_path_shift_m);
      gate_lateral_offset_m += 0.25f *
          (requested_lateral_shift_m - gate_lateral_offset_m);
      gate_vertical_offset_m += 0.25f *
          (requested_vertical_shift_m - gate_vertical_offset_m);
      gate_filter_sample = observation.sample;
      if ((observation.sample % 10u) == 0u) {
        DEBUG_PRINT("Vision gate confidence=%.2f offset=(%.2f,%.2f) depth=%.2f\n",
                    (double)observation.gate_confidence,
                    (double)gate_lateral_offset_m,
                    (double)gate_vertical_offset_m,
                    (double)estimated_depth_m);
      }
    }
  }
  /* Shift each knot on its own rotating path normal. Rebuild the path-tunnel
   * slots around the same displaced horizon so the optimizer never receives a
   * shifted reference inside an unshifted corridor. Nominal attitude/bank is
   * retained as feedforward; TinyMPC may deviate through its unchanged
   * dynamics and costs. */
#if TINYMPC_PATH_TUNNEL_ENABLE
  TinyMpcTunnelFrame previous_gate_shifted_tunnel_frame = {};
#endif
  for (int k = 0; k < NHORIZON; ++k) {
    const float ramp = (float)k / (float)(NHORIZON - 1);
    Eigen::Vector3f knot_path_tangent_local = Xref[k].segment<3>(6);
    knot_path_tangent_local.z() = 0.0f;
    if (knot_path_tangent_local.head<2>().norm() < 0.01f) {
      knot_path_tangent_local = gate_path_tangent_local;
    } else {
      knot_path_tangent_local.normalize();
    }
    const Eigen::Vector3f knot_path_normal_local(
        -knot_path_tangent_local.y(), knot_path_tangent_local.x(), 0.0f);
    Xref[k].head<3>() +=
        knot_path_normal_local * (ramp * gate_lateral_offset_m);
    Xref[k](2) += ramp * gate_vertical_offset_m;
#if TINYMPC_PATH_TUNNEL_ENABLE
    if (k > 0) {
      const Eigen::Vector3f tangent_world = localVectorToWorld(
          active_local_frame, knot_path_tangent_local);
      const TinyMpcTunnelVector tunnel_tangent = tinyMpcTunnelVector(
          tangent_world.x(), tangent_world.y(), tangent_world.z());
      const TinyMpcTunnelVector *previous_normal =
          previous_gate_shifted_tunnel_frame.valid
          ? &previous_gate_shifted_tunnel_frame.normal_1 : NULL;
      const TinyMpcTunnelFrame shifted_tunnel_frame = tinyMpcPathTunnelFrame(
          tunnel_tangent, previous_normal);
      if (shifted_tunnel_frame.valid) {
        const Eigen::Vector3f local_frame_origin_world(
            active_local_frame.origin_x,
            active_local_frame.origin_y,
            active_local_frame.origin_z);
        setPathTunnelHalfspaces(
            k, local_frame_origin_world + localVectorToWorld(
                active_local_frame, Xref[k].head<3>()),
            shifted_tunnel_frame);
        previous_gate_shifted_tunnel_frame = shifted_tunnel_frame;
      }
    }
#endif
  }
#endif
}

static void maybeFuseGatePose(
    const TinyRacerPerceptionObservation& observation,
    const state_t& estimated_state) {
#if TINYMPC_GATE_POSITION_FUSION_ENABLE && TINYMPC_GATE_PNP_FUSION_ENABLE
  static uint16_t last_sequence;
  static uint16_t last_fused_sequence;
  static bool has_last_fused_sequence;
  if (!observation.valid || !observation.gate_valid ||
      observation.sequence == last_sequence ||
      observation.received_age_ms > race_config.maximum_age_ms ||
      observation.gate_confidence < 0.6174671283f) {
    return;
  }
  last_sequence = observation.sequence;

  const float* corner = observation.gate_corners_xy;
  const float width = 0.5f * (
      hypotf(corner[2] - corner[0], corner[3] - corner[1]) +
      hypotf(corner[4] - corner[6], corner[5] - corner[7]));
  const float height = 0.5f * (
      hypotf(corner[6] - corner[0], corner[7] - corner[1]) +
      hypotf(corner[4] - corner[2], corner[5] - corner[3]));
  /* PnP needs the capture camera calibration, not the training dataset's
   * intrinsics carried in the ONNX adapter metadata. CrazySim's 47.168554-deg
   * square render has this normalized focal length; hardware builds must set
   * these macros from their calibrated AI-deck camera. */
  const float fx = (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  const float fy = (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  if (width < 0.04f || height < 0.04f || fx < 0.05f || fy < 0.05f) {
    return;
  }

  const float cx = (float)TINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED;
  const float cy = (float)TINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED;
  if (!isfinite(cx) || !isfinite(cy)) {
    return;
  }

  /* Isaac training metadata defines network corner supervision on the real
   * rail centers: a 0.555 m square, distinct from the clear opening. */
  constexpr float gate_corner_span_m = 0.555f;
  constexpr float half_span_m = 0.5f * gate_corner_span_m;
  const float object_xy[8] = {
      -half_span_m, -half_span_m,
       half_span_m, -half_span_m,
       half_span_m,  half_span_m,
      -half_span_m,  half_span_m,
  };

  /* Gate axes in world are image-right=-Y, image-down=-Z, normal=+X.
   * The camera-to-body mounting has exactly the same axis permutation. */
  Eigen::Matrix3f rotation_world_gate;
  rotation_world_gate << 0.0f, 0.0f, 1.0f,
                        -1.0f, 0.0f, 0.0f,
                         0.0f, -1.0f, 0.0f;
  const Eigen::Matrix3f rotation_body_camera = rotation_world_gate;
  const Eigen::Vector3f gate_world(
      (float)TINYMPC_GATE_WORLD_X_M,
      (float)TINYMPC_GATE_WORLD_Y_M,
      (float)TINYMPC_GATE_WORLD_Z_M);
  const Eigen::Vector3f estimated_position(
      estimated_state.position.x,
      estimated_state.position.y,
      estimated_state.position.z);
  Eigen::Quaternionf estimated_attitude(
      estimated_state.attitudeQuaternion.w,
      estimated_state.attitudeQuaternion.x,
      estimated_state.attitudeQuaternion.y,
      estimated_state.attitudeQuaternion.z);
  estimated_attitude.normalize();

  /* A direct planar-homography decomposition has two pose branches and is
   * extremely sensitive to a one-cell heatmap error. Solve PnP iteratively
   * instead, initialized at the propagated EKF pose. This selects the nearby
   * physical branch while the image residual still determines the correction. */
  const Eigen::Matrix3f estimated_rotation_world_body =
      estimated_attitude.toRotationMatrix();
  const Eigen::Matrix3f estimated_rotation_world_camera =
      estimated_rotation_world_body * rotation_body_camera;
  Eigen::Matrix3f rotation_camera_gate =
      estimated_rotation_world_camera.transpose() * rotation_world_gate;
  const Eigen::Quaternionf initial_rotation_camera_gate(
      rotation_camera_gate);
  Eigen::Vector3f translation_camera_gate =
      estimated_rotation_world_camera.transpose() *
      (gate_world - estimated_position);

  Eigen::Matrix<float, 8, 1> observed_normalized;
  for (int i = 0; i < 4; ++i) {
    observed_normalized(2 * i) = (corner[2 * i] - cx) / fx;
    observed_normalized(2 * i + 1) =
        (corner[2 * i + 1] - cy) / fy;
  }
  const auto project_gate = [&](const Eigen::Matrix3f& rotation,
                                const Eigen::Vector3f& translation) {
    Eigen::Matrix<float, 8, 1> projection;
    for (int i = 0; i < 4; ++i) {
      const Eigen::Vector3f point_camera =
          rotation * Eigen::Vector3f(
              object_xy[2 * i], object_xy[2 * i + 1], 0.0f) +
          translation;
      const float safe_depth = T_MAX(point_camera.z(), 0.05f);
      projection(2 * i) = point_camera.x() / safe_depth;
      projection(2 * i + 1) = point_camera.y() / safe_depth;
    }
    return projection;
  };
  const auto incremental_rotation = [](
      const Eigen::Vector3f& delta) -> Eigen::Matrix3f {
    const float angle = delta.norm();
    if (angle < 1e-7f) {
      return Eigen::Matrix3f::Identity();
    }
    return Eigen::AngleAxisf(angle, delta / angle).toRotationMatrix();
  };

  constexpr float derivative_step = 1e-4f;
  constexpr float levenberg_damping = 1e-4f;
  for (int iteration = 0; iteration < 8; ++iteration) {
    const Eigen::Matrix<float, 8, 1> residual =
        observed_normalized -
        project_gate(rotation_camera_gate, translation_camera_gate);
    Eigen::Matrix<float, 8, 6> jacobian;
    for (int parameter = 0; parameter < 6; ++parameter) {
      Eigen::Matrix3f perturbed_rotation = rotation_camera_gate;
      Eigen::Vector3f perturbed_translation = translation_camera_gate;
      if (parameter < 3) {
        Eigen::Vector3f delta = Eigen::Vector3f::Zero();
        delta(parameter) = derivative_step;
        perturbed_rotation =
            incremental_rotation(delta) * perturbed_rotation;
      } else {
        perturbed_translation(parameter - 3) += derivative_step;
      }
      jacobian.col(parameter) =
          (observed_normalized -
           project_gate(perturbed_rotation, perturbed_translation) - residual) /
          derivative_step;
    }
    Eigen::Matrix<float, 6, 6> normal =
        jacobian.transpose() * jacobian;
    normal.diagonal().array() += levenberg_damping;
    Eigen::Matrix<float, 6, 1> delta =
        normal.partialPivLu().solve(-jacobian.transpose() * residual);
    if (!delta.allFinite()) {
      return;
    }
    const float rotation_step_norm = delta.head<3>().norm();
    if (rotation_step_norm > 0.15f) {
      delta.head<3>() *= 0.15f / rotation_step_norm;
    }
    const float translation_step_norm = delta.tail<3>().norm();
    if (translation_step_norm > 0.25f) {
      delta.tail<3>() *= 0.25f / translation_step_norm;
    }
    rotation_camera_gate =
        incremental_rotation(delta.head<3>()) * rotation_camera_gate;
    Eigen::Quaternionf candidate_rotation_camera_gate(rotation_camera_gate);
    if (initial_rotation_camera_gate.dot(candidate_rotation_camera_gate) <
        0.0f) {
      candidate_rotation_camera_gate.coeffs() *= -1.0f;
    }
    const float rotation_from_propagation_rad = 2.0f * acosf(T_MIN(T_MAX(
        initial_rotation_camera_gate.dot(candidate_rotation_camera_gate),
        0.0f), 1.0f));
    constexpr float maximum_rotation_from_propagation_rad =
        0.20943951f;  // 12 deg
    if (rotation_from_propagation_rad >
        maximum_rotation_from_propagation_rad) {
      candidate_rotation_camera_gate = initial_rotation_camera_gate.slerp(
          maximum_rotation_from_propagation_rad /
              rotation_from_propagation_rad,
          candidate_rotation_camera_gate);
    }
    rotation_camera_gate =
        candidate_rotation_camera_gate.normalized().toRotationMatrix();
    translation_camera_gate += delta.tail<3>();
    if (delta.norm() < 1e-4f) {
      break;
    }
  }
  const Eigen::Matrix<float, 8, 1> final_residual =
      observed_normalized -
      project_gate(rotation_camera_gate, translation_camera_gate);
  const float reprojection_rms_px =
      160.0f * sqrtf(final_residual.squaredNorm() / 8.0f);
  const float pnp_depth_m = translation_camera_gate.z();
  if (!rotation_camera_gate.allFinite() || !isfinite(pnp_depth_m) ||
      !isfinite(reprojection_rms_px) || pnp_depth_m < 0.30f ||
      pnp_depth_m > 6.0f || reprojection_rms_px > 12.0f) {
    DEBUG_PRINT(
        "Gate PnP fit rejected seq=%u depth=%.3fm reprojection=%.2fpx\n",
        (unsigned int)observation.sequence, (double)pnp_depth_m,
        (double)reprojection_rms_px);
    return;
  }

  const Eigen::Matrix3f rotation_world_camera =
      rotation_world_gate * rotation_camera_gate.transpose();
  const Eigen::Matrix3f rotation_world_body =
      rotation_world_camera * rotation_body_camera.transpose();
  const Eigen::Vector3f raw_position_world =
      gate_world - rotation_world_camera * translation_camera_gate;
  Eigen::Quaternionf raw_attitude_world_body(rotation_world_body);
  raw_attitude_world_body.normalize();
  if (!raw_position_world.allFinite() ||
      !raw_attitude_world_body.coeffs().allFinite() ||
      !estimated_attitude.coeffs().allFinite()) {
    return;
  }
  if (estimated_attitude.dot(raw_attitude_world_body) < 0.0f) {
    raw_attitude_world_body.coeffs() *= -1.0f;
  }
  const Eigen::Vector3f raw_position_innovation =
      raw_position_world - estimated_position;
  const float raw_attitude_angle_rad = 2.0f * acosf(T_MIN(T_MAX(
      estimated_attitude.dot(raw_attitude_world_body), 0.0f), 1.0f));
  constexpr float maximum_raw_position_innovation_m = 1.25f;
  constexpr float maximum_raw_attitude_innovation_rad = 0.610865238f;  // 35 deg
  if (raw_position_innovation.norm() > maximum_raw_position_innovation_m ||
      !isfinite(raw_attitude_angle_rad) ||
      raw_attitude_angle_rad > maximum_raw_attitude_innovation_rad) {
    DEBUG_PRINT(
        "Gate PnP rejected seq=%u position_innovation=%.3fm attitude_innovation=%.1fdeg depth=%.3fm confidence=%.3f\n",
        (unsigned int)observation.sequence,
        (double)raw_position_innovation.norm(),
        (double)(raw_attitude_angle_rad * 57.2957795f),
        (double)pnp_depth_m,
        (double)observation.gate_confidence);
    return;
  }

  /* Bound each accepted correction before it reaches the EKF. Planar PnP from
   * 20x20 heatmaps is informative but noisy, especially in range and tilt.
   * These limits let vertical bearing and attitude participate without one
   * imperfect corner set dominating IMU/flow/range propagation. */
  constexpr float maximum_position_correction_m = 0.03f;
  Eigen::Vector3f bounded_position_innovation = raw_position_innovation;
  if (bounded_position_innovation.norm() > maximum_position_correction_m) {
    bounded_position_innovation *=
        maximum_position_correction_m / bounded_position_innovation.norm();
  }
  constexpr float maximum_attitude_correction_rad =
      0.00872664626f;  // 0.5 deg
  const float attitude_fraction = raw_attitude_angle_rad > 1e-5f
      ? T_MIN(1.0f,
              maximum_attitude_correction_rad / raw_attitude_angle_rad)
      : 1.0f;
  const Eigen::Quaternionf bounded_attitude = estimated_attitude.slerp(
      attitude_fraction, raw_attitude_world_body);

  /* Camera inference is 30 Hz. Fuse at no more than 5 Hz, anchored to the
   * first valid observation. Rejected detections do not consume the interval. */
  const uint16_t frames_since_last_fusion =
      (uint16_t)(observation.sequence - last_fused_sequence);
  if (has_last_fused_sequence && frames_since_last_fusion < 6u) {
    DEBUG_PRINT(
        "Gate PnP rate-limited seq=%u frames_since_fusion=%u\n",
        (unsigned int)observation.sequence,
        (unsigned int)frames_since_last_fusion);
    return;
  }

  poseMeasurement_t measurement = {};
  const Eigen::Vector3f bounded_position =
      estimated_position + bounded_position_innovation;
  measurement.x = bounded_position.x();
  measurement.y = bounded_position.y();
  measurement.z = bounded_position.z();
  measurement.quat.x = bounded_attitude.x();
  measurement.quat.y = bounded_attitude.y();
  measurement.quat.z = bounded_attitude.z();
  measurement.quat.w = bounded_attitude.w();
  measurement.stdDevPos = T_MIN(T_MAX(
      0.24f + 0.04f * pnp_depth_m +
          0.10f * (1.0f - observation.gate_confidence),
      0.35f), 0.50f);
  measurement.stdDevQuat = T_MIN(T_MAX(
      0.20f + 0.03f * pnp_depth_m +
          0.08f * (1.0f - observation.gate_confidence),
      0.30f), 0.45f);
  estimatorEnqueuePose(&measurement);
  last_fused_sequence = observation.sequence;
  has_last_fused_sequence = true;
  DEBUG_PRINT(
      "Gate PnP fused seq=%u raw_position=(%.3f,%.3f,%.3f) bounded_position=(%.3f,%.3f,%.3f) raw_innovation=(%.3f,%.3f,%.3f) attitude_innovation=%.1fdeg depth=%.3fm stddev=(%.3fm,%.3frad) confidence=%.3f\n",
      (unsigned int)observation.sequence,
      (double)raw_position_world.x(), (double)raw_position_world.y(),
      (double)raw_position_world.z(),
      (double)measurement.x, (double)measurement.y, (double)measurement.z,
      (double)raw_position_innovation.x(),
      (double)raw_position_innovation.y(),
      (double)raw_position_innovation.z(),
      (double)(raw_attitude_angle_rad * 57.2957795f),
      (double)pnp_depth_m,
      (double)measurement.stdDevPos, (double)measurement.stdDevQuat,
      (double)observation.gate_confidence);
#else
  (void)observation;
  (void)estimated_state;
#endif
}

static void maybeFuseGateCenterBearing(
    const TinyRacerPerceptionObservation& observation,
    const state_t& estimated_state) {
#if TINYMPC_GATE_POSITION_FUSION_ENABLE && \
    TINYMPC_GATE_CENTER_BEARING_FUSION_ENABLE
  static uint16_t last_sequence;
  static uint16_t last_fused_sequence;
  static bool has_last_fused_sequence;
  if (!observation.valid || !observation.gate_valid ||
      observation.sequence == last_sequence ||
      observation.received_age_ms > race_config.maximum_age_ms ||
      observation.gate_confidence < 0.6174671283f) {
    return;
  }
  last_sequence = observation.sequence;

  const float* corner = observation.gate_corners_xy;
  const float center_x = 0.25f *
      (corner[0] + corner[2] + corner[4] + corner[6]);
  const float center_y = 0.25f *
      (corner[1] + corner[3] + corner[5] + corner[7]);
  if (!isfinite(center_x) || !isfinite(center_y)) {
    return;
  }

  /* The heatmaps associate the known gate and provide only its center ray.
   * IMU/EKF attitude supplies the ray orientation; corner asymmetry is never
   * interpreted as camera roll, pitch, or monocular PnP depth. */
  const struct quat attitude_world_body = qnormalize(mkquat(
      estimated_state.attitudeQuaternion.x,
      estimated_state.attitudeQuaternion.y,
      estimated_state.attitudeQuaternion.z,
      estimated_state.attitudeQuaternion.w));
  const struct vec forward_world = qvrot(
      attitude_world_body, mkvec(1.0f, 0.0f, 0.0f));
  const struct vec camera_right_world = qvrot(
      attitude_world_body, mkvec(0.0f, -1.0f, 0.0f));
  const struct vec camera_up_world = qvrot(
      attitude_world_body, mkvec(0.0f, 0.0f, 1.0f));
  const Eigen::Vector3f gate_from_estimate(
      (float)TINYMPC_GATE_WORLD_X_M - estimated_state.position.x,
      (float)TINYMPC_GATE_WORLD_Y_M - estimated_state.position.y,
      (float)TINYMPC_GATE_WORLD_Z_M - estimated_state.position.z);
  const float forward_range_m = gate_from_estimate.dot(Eigen::Vector3f(
      forward_world.x, forward_world.y, forward_world.z));
  if (!isfinite(forward_range_m) || forward_range_m < 0.30f ||
      forward_range_m > 6.0f) {
    return;
  }

  const float fx = (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  const float fy = (float)TINYMPC_GATE_CAMERA_FOCAL_NORMALIZED;
  const float gate_right_m =
      (center_x - (float)TINYMPC_GATE_CAMERA_CENTER_X_NORMALIZED) *
      forward_range_m / fx;
  const float gate_up_m =
      -(center_y - (float)TINYMPC_GATE_CAMERA_CENTER_Y_NORMALIZED) *
      forward_range_m / fy;
  const Eigen::Vector3f raw_position(
      (float)TINYMPC_GATE_WORLD_X_M
          - forward_range_m * forward_world.x
          - gate_right_m * camera_right_world.x
          - gate_up_m * camera_up_world.x,
      (float)TINYMPC_GATE_WORLD_Y_M
          - forward_range_m * forward_world.y
          - gate_right_m * camera_right_world.y
          - gate_up_m * camera_up_world.y,
      (float)TINYMPC_GATE_WORLD_Z_M
          - forward_range_m * forward_world.z
          - gate_right_m * camera_right_world.z
          - gate_up_m * camera_up_world.z);
  const Eigen::Vector3f estimated_position(
      estimated_state.position.x,
      estimated_state.position.y,
      estimated_state.position.z);
  Eigen::Vector3f transverse_innovation(
      0.0f,
      raw_position.y() - estimated_position.y(),
      raw_position.z() - estimated_position.z());
  if (!raw_position.allFinite() || !transverse_innovation.allFinite() ||
      transverse_innovation.norm() > 0.75f) {
    DEBUG_PRINT(
        "Gate bearing rejected seq=%u innovation_yz=%.3fm range=%.3fm confidence=%.3f\n",
        (unsigned int)observation.sequence,
        (double)transverse_innovation.norm(), (double)forward_range_m,
        (double)observation.gate_confidence);
    return;
  }

  constexpr float maximum_transverse_correction_m = 0.03f;
  if (transverse_innovation.norm() > maximum_transverse_correction_m) {
    transverse_innovation *=
        maximum_transverse_correction_m / transverse_innovation.norm();
  }
  const uint16_t frames_since_last_fusion =
      (uint16_t)(observation.sequence - last_fused_sequence);
  if (has_last_fused_sequence && frames_since_last_fusion < 6u) {
    DEBUG_PRINT(
        "Gate bearing rate-limited seq=%u frames_since_fusion=%u\n",
        (unsigned int)observation.sequence,
        (unsigned int)frames_since_last_fusion);
    return;
  }

  positionMeasurement_t measurement = {};
  measurement.x = estimated_state.position.x;
  measurement.y = estimated_state.position.y + transverse_innovation.y();
  measurement.z = estimated_state.position.z + transverse_innovation.z();
  measurement.stdDev = 0.35f;
  measurement.source = MeasurementSourceLocationService;
  estimatorEnqueuePosition(&measurement);
  last_fused_sequence = observation.sequence;
  has_last_fused_sequence = true;
  DEBUG_PRINT(
      "Gate bearing fused seq=%u center=(%.3f,%.3f) measured_yz=(%.3f,%.3f) correction_yz=(%.3f,%.3f) range=%.3fm stddev=%.3fm confidence=%.3f\n",
      (unsigned int)observation.sequence, (double)center_x, (double)center_y,
      (double)measurement.y, (double)measurement.z,
      (double)transverse_innovation.y(),
      (double)transverse_innovation.z(), (double)forward_range_m,
      (double)measurement.stdDev, (double)observation.gate_confidence);
#else
  (void)observation;
  (void)estimated_state;
#endif
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
  static const float danger_bearing_rad[TINYRACER_DANGER_SECTORS] = {
    0.6981317f, 0.0f, -0.6981317f
  };
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    const float excess = T_MAX(observation.danger_probability[sector] -
                                   race_config.danger_probability_threshold,
                               0.0f);
    bearing_sum += excess * danger_bearing_rad[sector];
    weight_sum += excess;
  }
  race_state.pass_side = weight_sum > 1e-4f && bearing_sum < 0.0f ? 1 : -1;
  race_intent.pass_side = race_state.pass_side;
  DEBUG_PRINT("Vision danger plane distance=%.2f action=%s probabilities=(%.2f,%.2f,%.2f)\n",
              (double)stopping_distance_m,
              race_intent.pass_side > 0 ? "left" : "right",
              (double)observation.danger_probability[0],
              (double)observation.danger_probability[1],
              (double)observation.danger_probability[2]);
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

#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
static bool reactivePowerLoopActive(void) {
  return reactive_power_loop_state.phase == TINYMPC_REACTIVE_LOOP_ACTIVE;
}

static uint16_t reactivePowerLoopReferenceIndex(int horizon_knot) {
  const uint32_t requested = (uint32_t)reactive_power_loop_step +
      (uint32_t)T_MAX(horizon_knot, 0);
  return (uint16_t)T_MIN(
      requested, (uint32_t)TINYMPC_REACTIVE_LOOP_SAMPLE_COUNT - 1u);
}

static int reactivePowerLoopCacheInterval(int horizon_knot) {
  const uint32_t requested = (uint32_t)reactive_power_loop_step +
      (uint32_t)T_MAX(horizon_knot, 0);
  return (int)T_MIN(
      requested, (uint32_t)TINYMPC_REACTIVE_LOOP_STORED_LTV_INTERVALS - 1u);
}

static void setReactivePowerLoopHorizonReference(void) {
  const struct quat yaw_rotation = rpy2quat(
      mkvec(0.0f, 0.0f, reactive_power_loop_yaw_world_rad));
  const float initial_x = tinympc_reactive_loop_reference_data[0][0];
  const float initial_y = tinympc_reactive_loop_reference_data[0][1];
  const float initial_z = tinympc_reactive_loop_reference_data[0][2];
  const float cosine = cosf(reactive_power_loop_yaw_world_rad);
  const float sine = sinf(reactive_power_loop_yaw_world_rad);
  for (int knot = 0; knot < NHORIZON; ++knot) {
    const uint16_t index = reactivePowerLoopReferenceIndex(knot);
    const float *reference = tinympc_reactive_loop_reference_data[index];
    const float relative_x = reference[0] - initial_x;
    const float relative_y = reference[1] - initial_y;
    const Eigen::Vector3f position_world = reactive_power_loop_anchor_world +
        Eigen::Vector3f(
            cosine * relative_x - sine * relative_y,
            sine * relative_x + cosine * relative_y,
            reference[2] - initial_z);
    const Eigen::Vector3f velocity_world(
        cosine * reference[7] - sine * reference[8],
        sine * reference[7] + cosine * reference[8],
        reference[9]);
    const struct quat maneuver_attitude = mkquat(
        reference[4], reference[5], reference[6], reference[3]);
    const struct quat attitude_world_body =
        composeWorldYawWithManeuver(yaw_rotation, maneuver_attitude);
    const Eigen::Vector3f body_rate(
        reference[10], reference[11], reference[12]);
    setLocalReferenceState(
        Xref[knot], position_world, attitude_world_body,
        velocity_world, body_rate);
    reference_yaw_unwrapped_rad[knot] = reactive_power_loop_yaw_world_rad;
    if (knot < NHORIZON - 1) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        Uref[knot](motor) = reference[13 + motor] -
            TINYMPC_LEVEL_HOVER_THRUST_N;
      }
    }
  }
  tiny_ClearPositionHalfspaces(&work);
}

static void setReactivePowerLoopRecoveryHorizonReference(void) {
  const struct quat upright_attitude = rpy2quat(
      mkvec(0.0f, 0.0f, reactive_power_loop_yaw_world_rad));
  for (int knot = 0; knot < NHORIZON; ++knot) {
    setLocalReferenceState(
        Xref[knot], reactive_power_loop_recovery_anchor_world,
        upright_attitude, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    reference_yaw_unwrapped_rad[knot] = reactive_power_loop_yaw_world_rad;
    if (knot < NHORIZON - 1) {
      Uref[knot].setZero();
    }
  }
  tiny_ClearPositionHalfspaces(&work);
}
#endif

#if TINYMPC_PATH_TUNNEL_ENABLE
static void setPathTunnelHalfspaces(
    int knot, const Eigen::Vector3f& center_world,
    const TinyMpcTunnelFrame& frame_world) {
  const Eigen::Vector3f origin_world(
      active_local_frame.origin_x,
      active_local_frame.origin_y,
      active_local_frame.origin_z);
  const Eigen::Vector3f center_local = worldVectorToLocal(
      active_local_frame, center_world - origin_world);
  const Eigen::Vector3f normal_1_local = worldVectorToLocal(
      active_local_frame,
      Eigen::Vector3f(
          frame_world.normal_1.x,
          frame_world.normal_1.y,
          frame_world.normal_1.z));
  const Eigen::Vector3f normal_2_local = worldVectorToLocal(
      active_local_frame,
      Eigen::Vector3f(
          frame_world.normal_2.x,
          frame_world.normal_2.y,
          frame_world.normal_2.z));
  const Eigen::Vector3f zero_velocity = Eigen::Vector3f::Zero();
  const Eigen::Vector3f negative_normal_1 = -normal_1_local;
  const Eigen::Vector3f negative_normal_2 = -normal_2_local;
  const float width_1 = (float)TINYMPC_PATH_TUNNEL_HALF_WIDTH_1_M;
  const float width_2 = (float)TINYMPC_PATH_TUNNEL_HALF_WIDTH_2_M;

  tiny_SetKinematicHalfspace(
      &work, knot, TINYMPC_PATH_TUNNEL_NORMAL_1_POSITIVE_SLOT,
      &normal_1_local, &zero_velocity,
      normal_1_local.dot(center_local) + width_1, 0.0f, 1);
  tiny_SetKinematicHalfspace(
      &work, knot, TINYMPC_PATH_TUNNEL_NORMAL_1_NEGATIVE_SLOT,
      &negative_normal_1, &zero_velocity,
      negative_normal_1.dot(center_local) + width_1, 0.0f, 1);
  tiny_SetKinematicHalfspace(
      &work, knot, TINYMPC_PATH_TUNNEL_NORMAL_2_POSITIVE_SLOT,
      &normal_2_local, &zero_velocity,
      normal_2_local.dot(center_local) + width_2, 0.0f, 1);
  tiny_SetKinematicHalfspace(
      &work, knot, TINYMPC_PATH_TUNNEL_NORMAL_2_NEGATIVE_SLOT,
      &negative_normal_2, &zero_velocity,
      negative_normal_2.dot(center_local) + width_2, 0.0f, 1);
}
#endif

static void setPowerLoopHorizonReference(void) {
  float horizon_sigma = power_loop_state.sigma;
  const struct quat yaw_rotation = rpy2quat(
      mkvec(0.0f, 0.0f, power_loop_yaw_world_rad));
  for (int knot = 0; knot < NHORIZON; ++knot) {
    const bool terminal_hover = horizon_sigma >= 1.0f - 1.0e-6f;
    const TinyMpcPowerLoopSample sample = tinyMpcPowerLoopSample(
        &power_loop_config, horizon_sigma);
    if (!sample.valid) {
      Xref[knot].setZero();
      if (knot < NHORIZON - 1) {
        Uref[knot].setZero();
      }
      continue;
    }
    const Eigen::Vector3f position_world = terminal_hover
        ? power_loop_anchor_world
        : power_loop_anchor_world
            + power_loop_forward_world * sample.position_forward_m
            + Eigen::Vector3f::UnitZ() * sample.position_up_m;
    Eigen::Vector3f velocity_world = Eigen::Vector3f::Zero();
    if (!terminal_hover) {
      velocity_world =
          power_loop_forward_world * sample.velocity_forward_mps
          + Eigen::Vector3f::UnitZ() * sample.velocity_up_mps;
    }
    const struct quat loop_attitude = mkquat(
        sample.attitude_loop.x, sample.attitude_loop.y,
        sample.attitude_loop.z, sample.attitude_loop.w);
    const struct quat attitude_world_body = terminal_hover
        ? yaw_rotation
        : composeWorldYawWithManeuver(yaw_rotation, loop_attitude);
    const Eigen::Vector3f body_rate = terminal_hover
        ? Eigen::Vector3f::Zero()
        : Eigen::Vector3f(
            sample.body_rate_rad_s.x,
            sample.body_rate_rad_s.y,
            sample.body_rate_rad_s.z);
    setLocalReferenceState(
        Xref[knot], position_world, attitude_world_body,
        velocity_world, body_rate);
    reference_yaw_unwrapped_rad[knot] = power_loop_yaw_world_rad;
    if (knot < NHORIZON - 1) {
      if (terminal_hover) {
        Uref[knot].setZero();
      } else {
        for (int motor = 0; motor < NINPUTS; ++motor) {
          const float correction_n = sample.motor_thrust_n[motor]
              - tinympc_generated_physical_hover_thrust[motor];
          Uref[knot](motor) = T_MIN(T_MAX(
              correction_n, lcu(motor)), ucu(motor));
        }
      }
      horizon_sigma = tinyMpcPowerLoopClamp01(
          horizon_sigma + DT * sample.speed_mps
              / (6.28318530717958647692f * power_loop_config.radius_m));
    }
  }
}

static void setPowerLoopRecoveryHorizonReference(void) {
  const struct quat upright_attitude = rpy2quat(
      mkvec(0.0f, 0.0f, power_loop_yaw_world_rad));
  for (int knot = 0; knot < NHORIZON; ++knot) {
    setLocalReferenceState(
        Xref[knot], power_loop_recovery_anchor_world, upright_attitude,
        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    reference_yaw_unwrapped_rad[knot] = power_loop_yaw_world_rad;
    if (knot < NHORIZON - 1) {
      Uref[knot].setZero();
    }
  }
}

#if TINYMPC_REACTIVE_REFERENCE_FREE
static void setReactiveReferenceFreeHorizon(bool advance) {
  /* Route-free mode returns before the progress-path handoff bookkeeping.
   * Own the same one-shot countdown here so readiness gates cannot remain
   * falsely blocked for the entire flight. */
  if (advance && trajectory_handoff_hold_steps > 0u) {
    --trajectory_handoff_hold_steps;
  }
  const Eigen::Vector3f origin_world(
      active_local_frame.origin_x,
      active_local_frame.origin_y,
      active_local_frame.origin_z);
  if (!reactive_reference_initialized) {
    reactive_reference_altitude_world_m = origin_world.z();
    reactive_reference_heading_world_rad = active_local_frame.yaw_world;
    reactive_reference_initialized = true;
  }

  TinyRacerPerceptionObservation observation = {};
  const bool available = sequentialObstacleLinkGetLatest(&observation);
  const bool observation_fresh = available && observation.valid &&
      (observation.has_sector_danger ||
       observation.has_collision_probability) &&
      observation.received_age_ms <= 200u;
  const bool regional_collision = observation_fresh &&
      observation.has_sector_danger;
  float danger_left = regional_collision
      ? observation.danger_probability[TINYRACER_DANGER_LEFT] : 1.0f;
  float danger_center = regional_collision
      ? observation.danger_probability[TINYRACER_DANGER_CENTER]
      : (observation_fresh ? observation.collision_probability : 1.0f);
  float danger_right = regional_collision
      ? observation.danger_probability[TINYRACER_DANGER_RIGHT] : 1.0f;
  if (observation_fresh && !regional_collision) {
    /* TinyVPC provides one collision probability, never invented sectors.
     * Keep side choice ambiguous while the scalar drives center stop/release. */
    danger_left = 0.0f;
    danger_right = 0.0f;
  }
  const float measured_horizontal_speed_mps = x0.segment<2>(6).norm();
  const float measured_forward_speed_mps = T_MAX(x0(6), 0.0f);
  const float measured_tilt_rad = hypotf(
      levelStateBankRad(x0), levelStatePitchRad(x0));
  const float measured_body_rate_rad_s = x0.segment<3>(9).norm();
  const bool vehicle_settled = measured_horizontal_speed_mps <= 0.08f &&
      /* The cached pitch recovery leaves about 0.03-0.04 m/s of harmless
       * cross-axis drift in CrazySim. Keep this far below the encoder-test
       * brake release limit (0.20 m/s), but do not livelock BRAKE at 0.025. */
      fabsf(x0(7)) <= 0.05f && fabsf(x0(8)) <= 0.12f &&
      measured_tilt_rad <= 0.08726646f &&
      fabsf(x0(9)) <= 0.06f && fabsf(x0(10)) <= 0.06f &&
      measured_body_rate_rad_s <= 0.08f;
  const bool rail_opening_direction_valid = observation_fresh &&
      observation.gate_valid &&
      observation.gate_confidence >=
          TINYRACER_REACTIVE_GATE_CONFIDENCE_THRESHOLD &&
      fabsf(observation.steering_command) >= 0.5f;
  const int8_t rail_opening_direction = observation.steering_command > 0.0f
      ? 1 : (observation.steering_command < 0.0f ? -1 : 0);

  const bool new_observation = observation_fresh &&
      observation.sample != reactive_reference_last_sample;
  const Eigen::Vector3f backtrack_forward_world(
      cosf(reactive_reference_heading_world_rad),
      sinf(reactive_reference_heading_world_rad), 0.0f);
  const float reactive_backtrack_progress_m =
      -(origin_world - reactive_reference_backtrack_anchor_world).dot(
          backtrack_forward_world);
  const bool reactive_backtrack_complete =
      reactive_reference_state.phase == TINYRACER_REACTIVE_BACKTRACK &&
      reactive_backtrack_progress_m >=
          reactive_reference_config.backtrack_distance_m;
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
  const TinyMpcReactivePowerLoopPhase previous_reactive_loop_phase =
      reactive_power_loop_state.phase;
  const bool reactive_loop_containment_clear =
      trajectory_handoff_hold_steps == 0u && origin_world.z() >= 0.75f;
  const bool reactive_loop_sequence_complete = reactivePowerLoopActive() &&
      reactive_power_loop_step >= TINYMPC_REACTIVE_LOOP_SAMPLE_COUNT - 1u;
  const bool reactive_loop_recovery_ready =
      measured_horizontal_speed_mps <= 0.20f && fabsf(x0(8)) <= 0.20f &&
      measured_tilt_rad <= 0.13962634f && measured_body_rate_rad_s <= 0.30f;
  const bool reactive_loop_navigation_idle =
      reactive_reference_state.phase == TINYRACER_REACTIVE_CRUISE ||
      reactive_reference_state.scan_clear_latched;
  reactive_power_loop_command = tinyMpcReactivePowerLoopUpdate(
      &reactive_power_loop_state, &reactive_power_loop_config,
      advance && new_observation, danger_center,
      reactive_loop_navigation_idle,
      reactive_loop_containment_clear, advance && vehicle_settled,
      reactive_loop_sequence_complete, advance && reactive_loop_recovery_ready);
  const bool reactive_loop_entered =
      previous_reactive_loop_phase != TINYMPC_REACTIVE_LOOP_ACTIVE &&
      reactive_power_loop_state.phase == TINYMPC_REACTIVE_LOOP_ACTIVE;
  if (reactive_loop_entered) {
    reactive_power_loop_step = 0u;
    reactive_power_loop_anchor_world = origin_world;
    reactive_power_loop_yaw_world_rad = active_local_frame.yaw_world;
    reactive_reference_speed_mps = 0.0f;
    reactive_reference_lateral_speed_mps = 0.0f;
    reactive_reference_turn_rate_rad_s = 0.0f;
    level_frame_reseed_requested = true;
  }
  if (previous_reactive_loop_phase == TINYMPC_REACTIVE_LOOP_ACTIVE &&
      reactive_power_loop_state.phase == TINYMPC_REACTIVE_LOOP_RECOVERY) {
    reactive_power_loop_recovery_anchor_world = origin_world;
    level_frame_reseed_requested = true;
  }
  if (reactive_power_loop_command.changed) {
    DEBUG_PRINT(
        "REACTIVE_POWER_LOOP transition phase=%u center_risk=%.3f clear=%u settled=%u recovery=%u index=%u anchor=(%.2f,%.2f,%.2f) triggers=%u\n",
        (unsigned)reactive_power_loop_state.phase, (double)danger_center,
        (unsigned)reactive_power_loop_state.clear_samples,
        (unsigned)reactive_power_loop_state.settled_samples,
        (unsigned)reactive_power_loop_state.recovery_samples,
        (unsigned)reactive_power_loop_step,
        (double)reactive_power_loop_anchor_world.x(),
        (double)reactive_power_loop_anchor_world.y(),
        (double)reactive_power_loop_anchor_world.z(),
        (unsigned)reactive_power_loop_state.trigger_count);
  }
  const bool reactive_loop_preempts_navigation =
      reactive_power_loop_command.request_stop ||
      reactive_power_loop_command.owns_reference;
#else
  const bool reactive_loop_preempts_navigation = false;
#endif
  if (advance && !reactive_loop_preempts_navigation && new_observation) {
    const TinyRacerReactivePhase previous_reactive_reference_phase =
        reactive_reference_state.phase;
    reactive_reference_command = tinyRacerReactiveStepWithRail(
        &reactive_reference_state, &reactive_reference_config,
        danger_left, danger_center, danger_right,
        rail_opening_direction_valid, rail_opening_direction, vehicle_settled,
        reactive_backtrack_complete);
    if (previous_reactive_reference_phase == TINYRACER_REACTIVE_CRUISE &&
        reactive_reference_state.phase == TINYRACER_REACTIVE_BRAKE) {
      reactive_square_collision_stop_context = true;
    } else if (reactive_reference_state.phase == TINYRACER_REACTIVE_CRUISE) {
      reactive_square_collision_stop_context = false;
    }
    if (new_observation) {
      reactive_reference_last_sample = observation.sample;
    }
    if (reactive_reference_command.changed &&
        reactive_reference_state.phase == TINYRACER_REACTIVE_BRAKE) {
      /* Match the encoder-comparison emergency-brake procedure: start the
       * scheduled ramp from the greater of the current reference and measured
       * forward speeds, so the first brake horizon never asks for acceleration
       * merely because tracking was slightly ahead of its reference. */
      reactive_reference_speed_mps = T_MAX(
          reactive_reference_speed_mps, measured_forward_speed_mps);
    }
    if (previous_reactive_reference_phase == TINYRACER_REACTIVE_BRAKE &&
        reactive_reference_state.phase == TINYRACER_REACTIVE_BACKTRACK) {
      reactive_reference_backtrack_anchor_world = origin_world;
      reactive_reference_heading_world_rad = active_local_frame.yaw_world;
      reactive_reference_speed_mps = 0.0f;
    }
    if (reactive_reference_command.changed &&
        (reactive_reference_state.phase == TINYRACER_REACTIVE_CRUISE ||
         reactive_reference_state.phase ==
             TINYRACER_REACTIVE_TRANSLATE_OPENING ||
         reactive_reference_state.scan_clear_latched)) {
      reactive_reference_heading_world_rad = active_local_frame.yaw_world;
    }
    if (reactive_reference_command.changed) {
#if TINYMPC_RATE_CASCADE
      outer_frame_reseed_requested = true;
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
      level_frame_reseed_requested = true;
#endif
      DEBUG_PRINT(
          "REACTIVE_MPC phase=%u direction=%d command=(forward=%.2f lateral=%.2f yaw=%.1f) brake_entry_speed=%.2f backtrack=(progress=%.3f reached=%u) scan_clear=%u rail=(valid=%u dir=%d confidence=%.3f latched=%u) danger=(%.3f,%.3f,%.3f) measured=(speed=%.2f tilt=%.2f rate=%.2f)\n",
          (unsigned)reactive_reference_command.phase,
          (int)reactive_reference_command.turn_direction,
          (double)reactive_reference_command.forward_speed_mps,
          (double)reactive_reference_command.lateral_speed_mps,
          (double)reactive_reference_command.yaw_rate_deg_s,
          (double)reactive_reference_speed_mps,
          (double)reactive_backtrack_progress_m,
          (unsigned)reactive_reference_state.backtrack_distance_reached,
          (unsigned)reactive_reference_state.scan_clear_latched,
          (unsigned)rail_opening_direction_valid,
          (int)rail_opening_direction,
          (double)observation.gate_confidence,
          (unsigned)reactive_reference_state.rail_direction_latched,
          (double)danger_left,
          (double)danger_center,
          (double)danger_right,
          (double)measured_horizontal_speed_mps,
          (double)measured_tilt_rad,
          (double)measured_body_rate_rad_s);
    }
  }

  const bool reactive_forward_command_active =
      reactive_reference_state.phase == TINYRACER_REACTIVE_CRUISE ||
      reactive_reference_state.phase == TINYRACER_REACTIVE_BACKTRACK;
  const float requested_speed_mps = !reactive_loop_preempts_navigation &&
      observation_fresh && reactive_forward_command_active
      ? reactive_reference_command.forward_speed_mps : 0.0f;
  if (advance) {
    reactive_reference_speed_mps = tinyRacerReactiveSlewSignedSpeed(
        reactive_reference_speed_mps, requested_speed_mps,
        (float)TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2,
        TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2, DT);
  }

  const Eigen::Vector3f forward_world(
      cosf(reactive_reference_heading_world_rad),
      sinf(reactive_reference_heading_world_rad), 0.0f);
  const Eigen::Vector3f left_world(
      -sinf(reactive_reference_heading_world_rad),
      cosf(reactive_reference_heading_world_rad), 0.0f);
  const bool braking =
      reactive_reference_state.phase == TINYRACER_REACTIVE_BRAKE;
  const bool maneuver_attitude_safe = measured_tilt_rad <= 0.26179939f &&
      measured_body_rate_rad_s <= 1.20f;
  const bool yaw_scan_requested =
      reactive_reference_state.phase == TINYRACER_REACTIVE_TURN_SCAN &&
      !reactive_reference_state.scan_clear_latched &&
      measured_horizontal_speed_mps <= 0.15f && maneuver_attitude_safe;
  if (advance) {
    tinyRacerSquareOpeningUpdate(
        &reactive_square_opening_state, &reactive_square_opening_config,
        reactive_reference_state.phase == TINYRACER_REACTIVE_TURN_SCAN,
        reactive_square_collision_stop_context,
        observation_fresh, new_observation,
        observation.has_square_opening,
        observation.square_opening_visible_probability,
        measured_horizontal_speed_mps,
        levelStateBankRad(x0), levelStatePitchRad(x0), x0(11));
    tinyRacerDebug.square_opening_probability =
        observation.has_square_opening
            ? observation.square_opening_visible_probability : 0.0f;
    tinyRacerDebug.square_opening_eligible =
        reactive_square_opening_state.trigger_eligible ? 1u : 0u;
    tinyRacerDebug.square_opening_seen =
        reactive_square_opening_state.square_opening_seen ? 1u : 0u;
  }
  const bool translation_requested = observation_fresh &&
      reactive_reference_state.phase ==
          TINYRACER_REACTIVE_TRANSLATE_OPENING &&
      !reactive_reference_state.scan_clear_latched &&
      maneuver_attitude_safe;
  if (reactive_reference_state.phase == TINYRACER_REACTIVE_TURN_SCAN &&
      !yaw_scan_requested) {
    reactive_reference_heading_world_rad = active_local_frame.yaw_world;
  }
  const float requested_turn_rate_rad_s = yaw_scan_requested
      ? (float)reactive_reference_state.turn_direction *
          radians(reactive_reference_config.turn_rate_deg_s)
      : 0.0f;
  const float requested_lateral_speed_mps = translation_requested
      ? reactive_reference_command.lateral_speed_mps : 0.0f;
  const float maximum_turn_rate_step_rad_s =
      (float)TINYMPC_REACTIVE_YAW_ACCELERATION_RAD_S2 * DT;
  if (advance) {
    const float turn_rate_error_rad_s = requested_turn_rate_rad_s
        - reactive_reference_turn_rate_rad_s;
    reactive_reference_turn_rate_rad_s += T_MIN(T_MAX(
        turn_rate_error_rad_s, -maximum_turn_rate_step_rad_s),
        maximum_turn_rate_step_rad_s);
    const float lateral_speed_error_mps = requested_lateral_speed_mps -
        reactive_reference_lateral_speed_mps;
    const float lateral_slew_mps2 = fabsf(requested_lateral_speed_mps) > 0.0f
        ? (float)TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2
        : TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2;
    const float maximum_lateral_speed_step_mps = lateral_slew_mps2 * DT;
    reactive_reference_lateral_speed_mps += T_MIN(T_MAX(
        lateral_speed_error_mps, -maximum_lateral_speed_step_mps),
        maximum_lateral_speed_step_mps);
  }
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
  if (reactive_power_loop_command.owns_reference) {
    if (reactivePowerLoopActive()) {
      setReactivePowerLoopHorizonReference();
      if (advance && !reactive_loop_entered &&
          reactive_power_loop_step < TINYMPC_REACTIVE_LOOP_SAMPLE_COUNT - 1u) {
        ++reactive_power_loop_step;
      }
    } else {
      setReactivePowerLoopRecoveryHorizonReference();
    }
    return;
  }
#endif
  const float turn_rate_rad_s = reactive_reference_turn_rate_rad_s;
  const bool yaw_maneuver_active = fabsf(turn_rate_rad_s) > 1.0e-4f ||
      fabsf(requested_turn_rate_rad_s) > 1.0e-4f;
  float previous_speed_mps = reactive_reference_speed_mps;
  float previous_lateral_speed_mps = reactive_reference_lateral_speed_mps;
  float forward_distance_m = 0.0f;
  float lateral_distance_m = 0.0f;
  float horizon_turn_rate_rad_s = turn_rate_rad_s;
  float horizon_yaw_world_rad = active_local_frame.yaw_world;

  for (int knot = 0; knot < NHORIZON; ++knot) {
    if (knot > 0 && yaw_maneuver_active) {
      const float previous_horizon_turn_rate_rad_s =
          horizon_turn_rate_rad_s;
      const float horizon_turn_rate_error_rad_s = requested_turn_rate_rad_s
          - horizon_turn_rate_rad_s;
      horizon_turn_rate_rad_s += T_MIN(T_MAX(
          horizon_turn_rate_error_rad_s, -maximum_turn_rate_step_rad_s),
          maximum_turn_rate_step_rad_s);
      horizon_yaw_world_rad += 0.5f * (
          previous_horizon_turn_rate_rad_s + horizon_turn_rate_rad_s) * DT;
    }
    float speed_mps = 0.0f;
    if (braking) {
      /* Keep position, velocity, and cached pitch on the same finite-rate
       * emergency profile used by the sealed vision-encoder comparison. In
       * particular, Xref[0]-Xref[1] must expose the 6 m/s^2 deceleration to
       * updateLevelActuatorModelSelection(); an all-zero horizon cannot. */
      speed_mps = T_MAX(
          reactive_reference_speed_mps -
              TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2 * knot * DT,
          0.0f);
    } else if (!yaw_maneuver_active) {
      speed_mps = knot == 0 ? reactive_reference_speed_mps
          : tinyRacerReactiveSlewSignedSpeed(
              previous_speed_mps, requested_speed_mps,
              (float)TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2,
              TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2, DT);
    }
    if (knot > 0) {
      forward_distance_m +=
          0.5f * (previous_speed_mps + speed_mps) * DT;
    }
    previous_speed_mps = speed_mps;
    float lateral_speed_mps = reactive_reference_lateral_speed_mps;
    if (translation_requested) {
      const float horizon_lateral_step_mps =
          (float)TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2 * knot * DT;
      const float requested_magnitude_mps =
          fabsf(requested_lateral_speed_mps);
      lateral_speed_mps = copysignf(T_MIN(
          fabsf(reactive_reference_lateral_speed_mps) +
              horizon_lateral_step_mps,
          requested_magnitude_mps), requested_lateral_speed_mps);
    }
    if (knot > 0) {
      lateral_distance_m += 0.5f *
          (previous_lateral_speed_mps + lateral_speed_mps) * DT;
    }
    previous_lateral_speed_mps = lateral_speed_mps;

    const float yaw_world_rad = yaw_maneuver_active
        ? horizon_yaw_world_rad
        : reactive_reference_heading_world_rad;
    float pitch_rad = 0.0f;
#if defined(TINYMPC_USE_ACTUATOR_LTI)
    if (braking) {
      const float cache_schedule_speed_mps = T_MAX(
          speed_mps,
          measured_forward_speed_mps -
              TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2 *
                  (knot + 1) * DT);
      const uint8_t braking_tier = tinyMpcNearestBrakingSpeedTier(
          cache_schedule_speed_mps);
      pitch_rad = cache_schedule_speed_mps > 1.0e-4f
          ? tinyMpcBrakingNominalPitchRad(
              braking_tier, &level_braking_selector_config)
          : 0.0f;
    }
#endif
    Eigen::Vector3f position_world = origin_world +
        forward_world * forward_distance_m + left_world * lateral_distance_m;
    position_world.z() = reactive_reference_altitude_world_m;
    const Eigen::Vector3f velocity_world = forward_world * speed_mps +
        left_world * lateral_speed_mps;
    const struct quat attitude_world_body = rpy2quat(
        mkvec(0.0f, pitch_rad, yaw_world_rad));
    setLocalReferenceState(
        Xref[knot], position_world, attitude_world_body,
        velocity_world,
        Eigen::Vector3f(0.0f, 0.0f, horizon_turn_rate_rad_s));
    reference_yaw_unwrapped_rad[knot] = yaw_world_rad;
    if (knot < NHORIZON - 1) {
      Uref[knot].setZero();
    }
  }
}
#endif

#if TINYMPC_YAW_SPIN_TEST_ENABLE
static void setYawSpinTestHorizon(void) {
  constexpr float two_pi_rad = 6.2831853071795864769f;
  constexpr float commanded_rate_rad_s =
      (float)TINYMPC_YAW_SPIN_TEST_RATE_RAD_S;
  constexpr float commanded_revolutions =
      (float)TINYMPC_YAW_SPIN_TEST_REVOLUTIONS;
  constexpr float straight_speed_mps =
      (float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS;
  constexpr float straight_duration_s =
      (float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S;
  static_assert(commanded_rate_rad_s != 0.0f,
                "yaw-spin rate must be nonzero");
  static_assert(commanded_revolutions > 0.0f,
                "yaw-spin revolutions must be positive");
  static_assert(straight_speed_mps >= 0.0f,
                "yaw-spin straight speed must be nonnegative");
  static_assert(straight_duration_s >= 0.0f,
                "yaw-spin straight duration must be nonnegative");
  constexpr float absolute_rate_rad_s = commanded_rate_rad_s < 0.0f
      ? -commanded_rate_rad_s : commanded_rate_rad_s;
  constexpr float total_angle_rad = two_pi_rad * commanded_revolutions;
  constexpr float first_spin_angle_rad = 0.5f * total_angle_rad;
  constexpr float first_spin_duration_s =
      first_spin_angle_rad / absolute_rate_rad_s;
  constexpr bool straight_segment_enabled = straight_speed_mps > 0.0f
      && straight_duration_s > 0.0f;
  constexpr float second_spin_start_s = first_spin_duration_s
      + (straight_segment_enabled ? straight_duration_s : 0.0f);
  constexpr float maneuver_duration_s = second_spin_start_s
      + first_spin_duration_s;
  const float first_spin_yaw_rad = yaw_spin_test_initial_yaw_rad
      + commanded_rate_rad_s * first_spin_duration_s;
  const Eigen::Vector3f straight_direction_world(
      cosf(first_spin_yaw_rad), sinf(first_spin_yaw_rad), 0.0f);
  const Eigen::Vector3f second_spin_anchor_world =
      yaw_spin_test_anchor_world + straight_direction_world
          * (straight_speed_mps * straight_duration_s);

  for (int knot = 0; knot < NHORIZON; ++knot) {
    const float elapsed_s = T_MIN(
        ((float)yaw_spin_test_step + (float)knot) * DT,
        maneuver_duration_s);
    float yaw_world_rad;
    float yaw_rate_rad_s;
    Eigen::Vector3f position_world;
    Eigen::Vector3f velocity_world = Eigen::Vector3f::Zero();
    if (elapsed_s < first_spin_duration_s) {
      yaw_world_rad = yaw_spin_test_initial_yaw_rad
          + commanded_rate_rad_s * elapsed_s;
      yaw_rate_rad_s = commanded_rate_rad_s;
      position_world = yaw_spin_test_anchor_world;
    } else if (elapsed_s < second_spin_start_s) {
      const float straight_elapsed_s = elapsed_s - first_spin_duration_s;
      yaw_world_rad = first_spin_yaw_rad;
      yaw_rate_rad_s = 0.0f;
      position_world = yaw_spin_test_anchor_world
          + straight_direction_world * (straight_speed_mps * straight_elapsed_s);
      velocity_world = straight_direction_world * straight_speed_mps;
    } else if (elapsed_s < maneuver_duration_s) {
      const float second_spin_elapsed_s = elapsed_s - second_spin_start_s;
      yaw_world_rad = first_spin_yaw_rad
          + commanded_rate_rad_s * second_spin_elapsed_s;
      yaw_rate_rad_s = commanded_rate_rad_s;
      position_world = second_spin_anchor_world;
    } else {
      yaw_world_rad = yaw_spin_test_initial_yaw_rad
          + commanded_rate_rad_s * (2.0f * first_spin_duration_s);
      yaw_rate_rad_s = 0.0f;
      position_world = second_spin_anchor_world;
    }
    const struct quat attitude_world_body = rpy2quat(
        mkvec(0.0f, 0.0f, yaw_world_rad));
    setLocalReferenceState(
        Xref[knot], position_world, attitude_world_body, velocity_world,
        Eigen::Vector3f(0.0f, 0.0f, yaw_rate_rad_s));
    reference_yaw_unwrapped_rad[knot] = yaw_world_rad;
    if (knot < NHORIZON - 1) {
      Uref[knot].setZero();
    }
  }

  const float elapsed_s = (float)yaw_spin_test_step * DT;
  const uint8_t phase = elapsed_s < first_spin_duration_s ? 0u
      : elapsed_s < second_spin_start_s ? 1u
      : elapsed_s < maneuver_duration_s ? 2u : 3u;
  if (phase != yaw_spin_test_last_phase) {
    yaw_spin_test_last_phase = phase;
    DEBUG_PRINT(
        "YAW_SPIN_TEST phase=%u elapsed=%.3fs straight_speed=%.3fm/s\n",
        (unsigned int)phase, (double)elapsed_s,
        (double)(phase == 1u ? straight_speed_mps : 0.0f));
  }
  const float completed_angle_rad = phase == 0u
      ? absolute_rate_rad_s * elapsed_s
      : phase == 1u ? first_spin_angle_rad
      : phase == 2u ? first_spin_angle_rad
          + absolute_rate_rad_s * (elapsed_s - second_spin_start_s)
      : total_angle_rad;
  const uint8_t completed_revolutions = (uint8_t)floorf(
      completed_angle_rad / two_pi_rad + 1.0e-4f);
  if (completed_revolutions > yaw_spin_test_reported_revolutions) {
    yaw_spin_test_reported_revolutions = completed_revolutions;
    DEBUG_PRINT(
        "YAW_SPIN_TEST revolution=%u/%u elapsed=%.3fs\n",
        (unsigned int)completed_revolutions,
        (unsigned int)ceilf(commanded_revolutions),
        (double)((float)yaw_spin_test_step * DT));
  }
  if (elapsed_s >= maneuver_duration_s) {
    if (!yaw_spin_test_complete_reported) {
      yaw_spin_test_complete_reported = true;
      DEBUG_PRINT(
          "YAW_SPIN_TEST complete revolutions=%.2f elapsed=%.3fs hold=1\n",
          (double)commanded_revolutions,
          (double)((float)yaw_spin_test_step * DT));
    }
  } else {
    ++yaw_spin_test_step;
  }
}
#endif

void updateHorizonReference(const setpoint_t *setpoint, bool advance) {
  /* Rebuild all horizon-dependent planes every solve.  Path tunnel slots are
   * filled below; applyRaceIntent() may subsequently fill the fifth slot. */
  tiny_ClearPositionHalfspaces(&work);
  for (int k = 0; k < NHORIZON; ++k) {
    progress_state_linear_cost[k].setZero();
  }
#if TINYMPC_YAW_SPIN_TEST_ENABLE
  (void)setpoint;
  (void)advance;
  setYawSpinTestHorizon();
  return;
#endif
#if TINYMPC_REACTIVE_REFERENCE_FREE
  setReactiveReferenceFreeHorizon(advance);
  return;
#endif
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    {
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
    TinyMpcPathPoint progress_vehicle_path = vehicle_path;
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
    if (dodge_state.phase != TINYRACER_DODGE_TRACK) {
      /* Measure route station, not distance along the displaced avoidance
       * lane.  The inner lane has a smaller radius, so its physical arc is
       * shorter than the corresponding nominal-circle arc.  Radially
       * projecting only the progress measurement onto the 3.6 m centerline
       * preserves the 0.5 m target-lead bound without letting route phase
       * fall behind the vehicle during a long pass. */
      constexpr float center_x_m = 3.60f;
      constexpr float center_y_m = 0.0f;
      constexpr float nominal_radius_m = 3.60f;
      const float radial_x_m = vehicle_path.x - center_x_m;
      const float radial_y_m = vehicle_path.y - center_y_m;
      const float measured_radius_m = hypotf(radial_x_m, radial_y_m);
      if (isfinite(measured_radius_m) && measured_radius_m > 0.25f) {
        const float radius_scale = nominal_radius_m / measured_radius_m;
        progress_vehicle_path.x = center_x_m + radius_scale * radial_x_m;
        progress_vehicle_path.y = center_y_m + radius_scale * radial_y_m;
      }
    }
#endif
    const TinyMpcPathSample heading_sample = tinyMpcProgressPathSample(
        &progress_path, progress_path.progress);
    const float desired_heading_yaw_rad = atan2f(
        trajectory_sin_yaw * heading_sample.tangent.x
            + trajectory_cos_yaw * heading_sample.tangent.y,
        trajectory_cos_yaw * heading_sample.tangent.x
            - trajectory_sin_yaw * heading_sample.tangent.y);
    constexpr float heading_alignment_rate_rad_s = 0.60f;
    constexpr float heading_alignment_release_error_rad =
        0.0523598776f;  // 3 deg.
    float remaining_heading_error_rad = 0.0f;
    const bool previous_heading_alignment_active =
        progress_heading_alignment_active;
    progress_heading_alignment_active =
        tinyMpcProgressUpdateInitialHeadingAlignment(
            desired_heading_yaw_rad, heading_alignment_rate_rad_s * DT,
            heading_alignment_release_error_rad, &reference_yaw_phase_rad,
            &progress_heading_alignment_complete,
            &remaining_heading_error_rad);
    if (progress_heading_alignment_active !=
        previous_heading_alignment_active) {
      DEBUG_PRINT(
          "PROGRESS heading alignment active=%u phase=%.2fdeg target=%.2fdeg remaining=%.2fdeg rate=%.2frad/s\n",
          progress_heading_alignment_active ? 1u : 0u,
          (double)(reference_yaw_phase_rad * 57.2957795f),
          (double)(desired_heading_yaw_rad * 57.2957795f),
          (double)(remaining_heading_error_rad * 57.2957795f),
          (double)heading_alignment_rate_rad_s);
    }
    progress_diag_cycle++;
    progress_invariant_diag_cycle++;
    if (advance && trajectory_handoff_hold_steps == 0u &&
        !progress_heading_alignment_active &&
        !progress_path.complete && !power_loop_reference_active) {
      const float before = progress_path.progress;
      const float measured_before = progress_path.measured_progress;
      const TinyMpcPathSample command_sample = tinyMpcProgressPathSample(
          &progress_path, progress_path.progress);
      constexpr float progress_entry_acceleration_mps2 =
          (float)TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2;
      constexpr float progress_terminal_deceleration_mps2 =
          (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2;
      const float terminal_speed_limit_mps =
          tinyMpcProgressTerminalSpeedLimit(
              &progress_path, progress_path.progress,
              progress_terminal_deceleration_mps2);
      const float reward_speed_bias_mps =
          (float)TINYMPC_PROGRESS_REWARD_WEIGHT
          / tinympc_generated_Q_diagonal[6];
      const float curvature_speed_scale =
          progressCurvatureEnvelopeScale(
              progress_path.progress,
              command_sample.speed_mps + reward_speed_bias_mps);
      float command_target_speed_mps = T_MIN(
          command_sample.speed_mps * curvature_speed_scale,
          terminal_speed_limit_mps);
#if defined(TINYMPC_VISION_ESPNET_DRONET_ENABLE)
      command_target_speed_mps = T_MIN(
          command_target_speed_mps, vision_track_speed_limit_mps);
#endif
#if TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
      command_target_speed_mps *= dronet_v2_speed_scale;
#endif
#if TINYMPC_VISION_RL_RESIDUAL_ENABLE
      command_target_speed_mps *= vision_residual_progress_speed_scale;
#endif
      const float previous_reference_speed_mps =
          progress_reference_speed_mps;
      progress_reference_speed_mps = tinyMpcProgressSlewSpeed(
          progress_reference_speed_mps, command_target_speed_mps,
          progress_entry_acceleration_mps2,
          progress_terminal_deceleration_mps2, DT);
      progress_reference_acceleration_mps2 =
          (progress_reference_speed_mps - previous_reference_speed_mps) / DT;
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
      tinyMpcProgressPathSetGeometricCatchupUpdateEnabled(
          &progress_path, true);
#endif
      tinyMpcProgressPathUpdate(
          &progress_path, progress_vehicle_path,
          progress_reference_speed_mps * DT);
      step = (uint32_t)floorf(progress_path.progress);
      if ((uint32_t)floorf(before) / 50u != step / 50u) {
        DEBUG_PRINT("Progress path sample=%.2f/%u\n",
                    (double)progress_path.progress,
                    (unsigned int)(progress_path.virtual_count - 1u));
      }
      const uint16_t completed_measured_laps =
          tinyMpcProgressPathCompletedLaps(
              &progress_path, progress_path.measured_progress);
      while (progress_last_reported_measured_lap < completed_measured_laps
          && progress_last_reported_measured_lap
              < (uint16_t)TINYMPC_PROGRESS_LAPS) {
        ++progress_last_reported_measured_lap;
        DEBUG_PRINT(
            "PROGRESS lap_complete lap=%u/%u measured=%.8f target=%.8f elapsed_s=%.3f\n",
            (unsigned int)progress_last_reported_measured_lap,
            (unsigned int)TINYMPC_PROGRESS_LAPS,
            (double)progress_path.measured_progress,
            (double)progress_path.progress,
            (double)progress_invariant_diag_cycle / (double)MPC_RATE);
      }
      if ((progress_invariant_diag_cycle % (uint32_t)MPC_RATE) == 0u ||
          progress_path.projection_bound_violation ||
          progress_path.command_step_violation ||
          progress_path.lead_bound_violation ||
          progress_path.lag_bound_violation) {
        DEBUG_PRINT(
            "PROGRESS state measured=%.3f target=%.3f measured_index_delta=%.3f target_index_delta=%.3f measured_total_m=%.8f target_total_m=%.8f\n",
            (double)progress_path.measured_progress,
            (double)progress_path.progress,
            (double)(progress_path.measured_progress - measured_before),
            (double)(progress_path.progress - before),
            (double)progress_path.cumulative_measured_advance_m,
            (double)progress_path.cumulative_commanded_advance_m);
        DEBUG_PRINT(
            "PROGRESS step vehicle_m=%.8f forward_m=%.8f projection_candidate_m=%.8f measured_m=%.8f measured_bound_m=%.8f geometric_catchup_m=%.8f geometric_catchup_total_m=%.8f geometric_catchup_active=%u rebase_m=%.8f command_request_m=%.8f command_m=%.8f lead_m=%.8f lead_bound_m=%.8f lag_m=%.8f tolerance_m=%.8f\n",
            (double)progress_path.last_vehicle_displacement_m,
            (double)progress_path.last_forward_displacement_m,
            (double)progress_path.last_projection_candidate_advance_m,
            (double)progress_path.last_measured_advance_m,
            (double)progress_path.last_measured_advance_bound_m,
            (double)progress_path.last_geometric_catchup_m,
            (double)progress_path.cumulative_geometric_catchup_m,
            progress_path.geometric_catchup_active ? 1u : 0u,
            (double)progress_path.last_reference_rebase_m,
            (double)progress_path.last_commanded_request_m,
            (double)progress_path.last_commanded_advance_m,
            (double)progress_path.last_phase_lead_m,
            (double)progress_path.last_phase_lead_bound_m,
            (double)progress_path.last_phase_lag_m,
            (double)progress_path.progress_tolerance_m);
        DEBUG_PRINT(
            "PROGRESS counters vehicle_total_m=%.5f forward_total_m=%.5f reference_rebase_total_m=%.5f projection_limited=%u limited_count=%lu projection_violation=%u projection_violation_count=%lu command_violation=%u command_violation_count=%lu lead_violation=%u lead_violation_count=%lu lag_violation=%u lag_violation_count=%lu\n",
            (double)progress_path.cumulative_vehicle_displacement_m,
            (double)progress_path.cumulative_forward_displacement_m,
            (double)progress_path.cumulative_reference_rebase_m,
            progress_path.projection_limited ? 1u : 0u,
            (unsigned long)progress_path.projection_limited_count,
            progress_path.projection_bound_violation ? 1u : 0u,
            (unsigned long)progress_path.projection_bound_violation_count,
            progress_path.command_step_violation ? 1u : 0u,
            (unsigned long)progress_path.command_step_violation_count,
            progress_path.lead_bound_violation ? 1u : 0u,
            (unsigned long)progress_path.lead_bound_violation_count,
            progress_path.lag_bound_violation ? 1u : 0u,
            (unsigned long)progress_path.lag_bound_violation_count);
        DEBUG_PRINT(
            "PROGRESS multilap measured_lap=%u/%u measured_lap_progress=%.8f target_lap=%u/%u target_lap_progress=%.8f commanded_speed_mps=%.5f entry_acceleration_mps2=%.5f elapsed_s=%.3f\n",
            (unsigned int)tinyMpcProgressPathCompletedLaps(
                &progress_path, progress_path.measured_progress),
            (unsigned int)progress_path.lap_count,
            (double)tinyMpcProgressPathLapProgress(
                &progress_path, progress_path.measured_progress),
            (unsigned int)tinyMpcProgressPathCompletedLaps(
                &progress_path, progress_path.progress),
            (unsigned int)progress_path.lap_count,
            (double)tinyMpcProgressPathLapProgress(
                &progress_path, progress_path.progress),
            (double)progress_reference_speed_mps,
            (double)progress_entry_acceleration_mps2,
            (double)progress_invariant_diag_cycle / (double)MPC_RATE);
      }
    }
#if TINYMPC_FLIP_ENABLE
    {
      const TinyMpcFlipMode previous_flip_mode = flip_state.mode;
      tinyMpcFlipUpdate(
          &flip_state, &flip_config,
          progress_path.cumulative_measured_advance_m, DT,
          advance && trajectory_handoff_hold_steps == 0u
              && !progress_path.complete,
          true);
      if (previous_flip_mode == TINYMPC_FLIP_ACTIVE
          && flip_state.mode == TINYMPC_FLIP_COMPLETE) {
        flip_recovery_active = true;
        flip_recovery_settle_steps = 0u;
        flip_recovery_elapsed_steps = 0u;
#if TINYMPC_RATE_CASCADE
        planner_rate_pid_reset_pending = true;
#endif
      }
      if (flip_recovery_active) {
        const struct quat measured_attitude = qnormalize(attitude);
        const float body_z_world_z = 1.0f - 2.0f * (
            measured_attitude.x * measured_attitude.x
            + measured_attitude.y * measured_attitude.y);
        const float body_rate_norm_rad_s = x0.segment<3>(9).norm();
        constexpr float minimum_recovery_body_z_world_z =
            0.9396926208f;  // Within 20 deg of upright.
        constexpr float maximum_recovery_body_rate_rad_s = 2.0f;
        constexpr uint16_t required_recovery_settle_steps = 10u;  // 0.2 s.
        ++flip_recovery_elapsed_steps;
        if (tinyMpcFlipRecoveryReady(
                body_z_world_z, body_rate_norm_rad_s,
                minimum_recovery_body_z_world_z,
                maximum_recovery_body_rate_rad_s)) {
          ++flip_recovery_settle_steps;
        } else {
          flip_recovery_settle_steps = 0u;
        }
        if (flip_recovery_settle_steps >= required_recovery_settle_steps) {
          flip_recovery_active = false;
#if TINYMPC_RATE_CASCADE
          planner_rate_pid_reset_pending = true;
#endif
          DEBUG_PRINT(
              "FLIP recovery complete elapsed_s=%.3f body_z_dot_world_z=%.4f body_rate=%.3frad/s\n",
              (double)flip_recovery_elapsed_steps * DT,
              (double)body_z_world_z, (double)body_rate_norm_rad_s);
        } else if ((flip_recovery_elapsed_steps % (uint32_t)MPC_RATE) == 0u) {
          DEBUG_PRINT(
              "FLIP recovery active elapsed_s=%.3f body_z_dot_world_z=%.4f body_rate=%.3frad/s settle=%u/%u\n",
              (double)flip_recovery_elapsed_steps * DT,
              (double)body_z_world_z, (double)body_rate_norm_rad_s,
              (unsigned int)flip_recovery_settle_steps,
              (unsigned int)required_recovery_settle_steps);
        }
      }
      flip_reference_active = flip_state.mode == TINYMPC_FLIP_ACTIVE
          || flip_recovery_active;
      if (flip_state.mode != previous_flip_mode) {
        DEBUG_PRINT(
            "FLIP transition mode=%d sigma=%.3f measured_s_m=%.3f duration_s=%.3f triggers=%u\n",
            (int)flip_state.mode, (double)flip_state.sigma,
            (double)progress_path.cumulative_measured_advance_m,
            (double)flip_config.duration_s,
            (unsigned int)flip_state.trigger_count);
      }
    }
#else
    flip_reference_active = false;
    flip_recovery_active = false;
#endif
#if TINYMPC_POWER_LOOP_ENABLE
    {
      const TinyMpcPowerLoopMode previous_mode = power_loop_state.mode;
      const bool previously_armed = power_loop_state.armed;
      const TinyMpcPathSample entry_path_sample = tinyMpcProgressPathSample(
          &progress_path, progress_path.progress);
      Eigen::Vector3f entry_forward_world(
          trajectory_cos_yaw * entry_path_sample.tangent.x
              - trajectory_sin_yaw * entry_path_sample.tangent.y,
          trajectory_sin_yaw * entry_path_sample.tangent.x
              + trajectory_cos_yaw * entry_path_sample.tangent.y,
          0.0f);
      const float entry_forward_norm = entry_forward_world.norm();
      if (entry_forward_norm > 1.0e-5f) {
        entry_forward_world /= entry_forward_norm;
      }
      const Eigen::Vector3f entry_forward_local = worldVectorToLocal(
          active_local_frame, entry_forward_world);
      const float entry_tangent_speed_mps =
          x0.segment<3>(6).dot(entry_forward_local);
      const struct quat entry_attitude = qnormalize(attitude);
      const float entry_body_z_world_z = 1.0f - 2.0f * (
          entry_attitude.x * entry_attitude.x
          + entry_attitude.y * entry_attitude.y);
      const float entry_body_rate_norm_rad_s = x0.segment<3>(9).norm();
      const bool entry_ready =
          entry_tangent_speed_mps
              >= 0.90f * power_loop_config.bottom_speed_mps
          && fabsf(x0(8)) <= 0.50f
          && entry_body_z_world_z >= 0.9396926208f
          && entry_body_rate_norm_rad_s <= 2.0f;
      tinyMpcPowerLoopUpdate(
          &power_loop_state, &power_loop_config,
          progress_path.cumulative_measured_advance_m, DT,
          advance && trajectory_handoff_hold_steps == 0u
              && !progress_path.complete,
          entry_ready, true);
      if (!previously_armed && power_loop_state.armed) {
        DEBUG_PRINT(
            "POWER_LOOP armed measured_s_m=%.3f entry_speed=%.3fm/s required=%.3fm/s body_z_dot_world_z=%.3f body_rate=%.3frad/s\n",
            (double)progress_path.cumulative_measured_advance_m,
            (double)entry_tangent_speed_mps,
            (double)(0.90f * power_loop_config.bottom_speed_mps),
            (double)entry_body_z_world_z,
            (double)entry_body_rate_norm_rad_s);
      }
      if (previous_mode == TINYMPC_POWER_LOOP_WAITING
          && power_loop_state.mode == TINYMPC_POWER_LOOP_ACTIVE) {
        /* Anchor at the measured entry position so enabling the primitive
         * cannot introduce an instantaneous position-reference step. */
        power_loop_anchor_world = vehicle_world;
        power_loop_forward_world = entry_forward_world;
        const float forward_norm = power_loop_forward_world.norm();
        if (forward_norm > 1.0e-5f) {
          power_loop_forward_world /= forward_norm;
        } else {
          power_loop_forward_world = Eigen::Vector3f(
              cosf(reference_yaw_phase_rad),
              sinf(reference_yaw_phase_rad), 0.0f);
        }
        power_loop_yaw_world_rad = atan2f(
            power_loop_forward_world.y(), power_loop_forward_world.x());
      }
      if (previous_mode == TINYMPC_POWER_LOOP_ACTIVE
          && power_loop_state.mode == TINYMPC_POWER_LOOP_COMPLETE) {
        power_loop_recovery_active = true;
        power_loop_recovery_anchor_world = vehicle_world;
        power_loop_recovery_settle_steps = 0u;
        power_loop_recovery_elapsed_steps = 0u;
#if TINYMPC_RATE_CASCADE
        planner_rate_pid_reset_pending = true;
#endif
      }
      if (power_loop_recovery_active) {
        const struct quat measured_attitude = qnormalize(attitude);
        const float body_z_world_z = 1.0f - 2.0f * (
            measured_attitude.x * measured_attitude.x
            + measured_attitude.y * measured_attitude.y);
        const float body_rate_norm_rad_s = x0.segment<3>(9).norm();
        const float position_error_m = (vehicle_world
            - power_loop_recovery_anchor_world).norm();
        const float velocity_error_mps = x0.segment<3>(6).norm();
        const bool recovered = tinyMpcFlipRecoveryReady(
                body_z_world_z, body_rate_norm_rad_s,
                0.9396926208f, 2.0f)
            /* Braking a full loop can displace the vehicle from the phase-end
             * anchor even after attitude and velocity have settled.  The
             * completion path immediately rebases the course at the measured
             * vehicle position, so do not keep exciting the recovery model
             * merely to return to an obsolete inertial anchor. */
            && position_error_m <= 1.25f && velocity_error_mps <= 1.0f;
        ++power_loop_recovery_elapsed_steps;
        if (recovered) {
          ++power_loop_recovery_settle_steps;
        } else {
          power_loop_recovery_settle_steps = 0u;
        }
        if (power_loop_recovery_settle_steps >= 10u) {
          /* Rebase the frozen course at the recovered vehicle state before
           * releasing the maneuver overlay. Otherwise an inertial-only
           * outage can leave a large position offset and the first ordinary
           * solve commands an abrupt catch-up maneuver. */
          const TinyMpcPathSample rejoin_sample = tinyMpcProgressPathSample(
              &progress_path, progress_path.progress);
          const Eigen::Vector3f rejoin_path_offset(
              trajectory_cos_yaw * rejoin_sample.position.x
                  - trajectory_sin_yaw * rejoin_sample.position.y,
              trajectory_sin_yaw * rejoin_sample.position.x
                  + trajectory_cos_yaw * rejoin_sample.position.y,
              rejoin_sample.position.z);
          trajectory_origin_world = vehicle_world - rejoin_path_offset;
          progress_reference_speed_mps = 0.0f;
          progress_reference_acceleration_mps2 = 0.0f;
          trajectory_handoff_hold_steps = (uint16_t)(MPC_RATE / 5);
          power_loop_recovery_active = false;
#if TINYMPC_RATE_CASCADE
          planner_rate_pid_reset_pending = true;
#endif
          DEBUG_PRINT(
              "POWER_LOOP recovery complete elapsed_s=%.3f position_error=%.3fm velocity_error=%.3fm/s body_rate=%.3frad/s\n",
              (double)power_loop_recovery_elapsed_steps * DT,
              (double)position_error_m, (double)velocity_error_mps,
              (double)body_rate_norm_rad_s);
        } else if ((power_loop_recovery_elapsed_steps
                    % (uint32_t)MPC_RATE) == 0u) {
          DEBUG_PRINT(
              "POWER_LOOP recovery active elapsed_s=%.3f position_error=%.3fm velocity_error=%.3fm/s body_z_dot_world_z=%.3f body_rate=%.3frad/s settle=%u/10\n",
              (double)power_loop_recovery_elapsed_steps * DT,
              (double)position_error_m, (double)velocity_error_mps,
              (double)body_z_world_z, (double)body_rate_norm_rad_s,
              (unsigned int)power_loop_recovery_settle_steps);
        }
      }
      power_loop_reference_active =
          power_loop_state.mode == TINYMPC_POWER_LOOP_ACTIVE
          || power_loop_recovery_active;
      if (power_loop_state.mode != previous_mode) {
        DEBUG_PRINT(
            "POWER_LOOP transition mode=%d sigma=%.4f measured_s_m=%.3f anchor=(%.2f,%.2f,%.2f) radius=%.2fm speeds=%.2f..%.2fm/s triggers=%u\n",
            (int)power_loop_state.mode, (double)power_loop_state.sigma,
            (double)progress_path.cumulative_measured_advance_m,
            (double)power_loop_anchor_world.x(),
            (double)power_loop_anchor_world.y(),
            (double)power_loop_anchor_world.z(),
            (double)power_loop_config.radius_m,
            (double)power_loop_config.bottom_speed_mps,
            (double)power_loop_config.top_speed_mps,
            (unsigned int)power_loop_state.trigger_count);
      }
    }
#else
    power_loop_reference_active = false;
    power_loop_recovery_active = false;
#endif
    if (power_loop_reference_active) {
      if (power_loop_recovery_active) {
        setPowerLoopRecoveryHorizonReference();
      } else {
        setPowerLoopHorizonReference();
      }
      return;
    }
    if (progress_path.complete && !progress_completion_reported) {
      progress_completion_reported = true;
      DEBUG_PRINT("Progress path complete sample=%.2f/%u elapsed_s=%.3f\n",
                  (double)progress_path.progress,
                  (unsigned int)(progress_path.virtual_count - 1u),
                  (double)progress_invariant_diag_cycle / (double)MPC_RATE);
    }
    TinyMpcPathSample first_sample = tinyMpcProgressPathSample(
        &progress_path, progress_path.progress);
    /* Closed paths contain a short, zero-distance terminal tail.  Once the
     * commanded target reaches that tail, continuing to apply the last
     * nonzero speed with its now-zero curvature sends the vehicle straight
     * past the finish.  Hold the endpoint at zero speed until the measured
     * vehicle position satisfies the existing physical completion gate. */
    const bool terminal_position_hold = !progress_path.complete
        && progress_path.progress >=
            tinyMpcProgressPathTerminalProgress(&progress_path) - 1.0e-3f;
    const float first_reference_speed_mps =
        terminal_position_hold || progress_heading_alignment_active
        ? 0.0f
        : T_MIN(
            T_MIN(first_sample.speed_mps, progress_reference_speed_mps),
            tinyMpcProgressTerminalSpeedLimit(
                &progress_path, progress_path.progress,
                (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2));
    const Eigen::Vector3f first_tangent_world(
        trajectory_cos_yaw * first_sample.tangent.x
            - trajectory_sin_yaw * first_sample.tangent.y,
        trajectory_sin_yaw * first_sample.tangent.x
            + trajectory_cos_yaw * first_sample.tangent.y,
        first_sample.tangent.z);
    const Eigen::Vector3f first_tangent_local = worldVectorToLocal(
        active_local_frame, first_tangent_world);
    const float measured_tangent_speed_mps =
        x0.segment<3>(6).dot(first_tangent_local);
    const bool progress_feedforward_active =
        advance && trajectory_handoff_hold_steps == 0u
        && !progress_heading_alignment_active
        && !terminal_position_hold && !progress_path.complete;
    const float full_progress_reward_speed_bias_mps =
        progress_feedforward_active
        ? (float)TINYMPC_PROGRESS_REWARD_WEIGHT
            / tinympc_generated_Q_diagonal[6]
        : 0.0f;
    const float first_curvature_speed_scale =
        tinyMpcProgressCentripetalSpeedScale(
            first_sample.speed_mps + full_progress_reward_speed_bias_mps,
            first_sample.curvature_magnitude_per_m,
            (float)TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2);
    const float first_terminal_speed_limit_mps =
        tinyMpcProgressTerminalSpeedLimit(
            &progress_path, progress_path.progress,
            (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2);
    const float first_reward_scale = tinyMpcProgressTerminalRewardScale(
        (first_sample.speed_mps + full_progress_reward_speed_bias_mps)
            * first_curvature_speed_scale,
        first_terminal_speed_limit_mps) * first_curvature_speed_scale;
    const float progress_reward_speed_bias_mps =
        full_progress_reward_speed_bias_mps * first_reward_scale;
    /* The linear reward shifts the unconstrained tangential-velocity optimum
     * by lambda/q_v. Use that proposed speed for attitude and thrust
     * feedforward while leaving the position/velocity reference horizon and
     * progress state unchanged. */
    const float first_feedforward_speed_mps =
        first_reference_speed_mps + progress_reward_speed_bias_mps;
    const float first_speed_tracking_acceleration_mps2 =
        tinyMpcProgressTangentialAcceleration(
            first_feedforward_speed_mps, measured_tangent_speed_mps);
    const float first_scheduled_deceleration_mps2 =
        terminal_position_hold
        ? 0.0f : T_MIN(progress_reference_acceleration_mps2, 0.0f);
    const float first_tangential_acceleration_mps2 = tinyMpcPathClamp(
        first_speed_tracking_acceleration_mps2
            + first_scheduled_deceleration_mps2,
        -(float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2, 0.50f);
    const float steady_drag_compensation_mps2 =
        -TINYMPC_BANK_MODEL_DRAG_X_N_PER_MPS
        * first_feedforward_speed_mps / TINYMPC_BANK_MODEL_MASS_KG;
    constexpr float maximum_local_yaw_deviation_rad = 0.2617993878f;
    float horizon_progress = progress_path.progress;
    float horizon_yaw = reference_yaw_phase_rad;
    float horizon_speed_mps = first_reference_speed_mps;
    float horizon_tangential_acceleration_mps2 =
        first_tangential_acceleration_mps2;
    float previous_horizon_pitch_rad = levelStatePitchRad(x0);
    TinyMpcProgressQuaternion progress_reference_attitudes[NHORIZON];
    TinyMpcProgressBodyRate progress_reference_body_rates[NHORIZON];
    TinyMpcProgressBodyRate flip_reference_body_rates[NHORIZON];
    float flip_reference_motor_thrust_n[NHORIZON][NINPUTS] = {};
    bool flip_reference_knot_active[NHORIZON] = {};
    float progress_reconstruction_residual_rad[NHORIZON - 1];
    float diag_curvature_per_m = 0.0f;
    float diag_speed_mps = 0.0f;
    float diag_yaw_rate_rad_s = 0.0f;
    float diag_roll_rad = 0.0f;
    float diag_pitch_rad = 0.0f;
    float diag_tilt_rad = 0.0f;
    float diag_thrust_scale = 0.0f;
    bool diag_uref_clamped = false;
#if TINYMPC_PATH_TUNNEL_ENABLE
    TinyMpcTunnelFrame previous_tunnel_frame = {};
#endif
    for (int i = 0; i < NHORIZON; ++i) {
      const TinyMpcPathSample sample = tinyMpcProgressPathSample(
          &progress_path, horizon_progress);
      const float curvature_speed_scale =
          tinyMpcProgressCentripetalSpeedScale(
              sample.speed_mps + full_progress_reward_speed_bias_mps,
              sample.curvature_magnitude_per_m,
              (float)TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2);
      constexpr float progress_entry_acceleration_mps2 =
          (float)TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2;
      const float terminal_speed_limit_mps =
          tinyMpcProgressTerminalSpeedLimit(
              &progress_path, horizon_progress,
              (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2);
      horizon_speed_mps = T_MIN(
          T_MIN(sample.speed_mps * curvature_speed_scale, horizon_speed_mps),
          terminal_speed_limit_mps);
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
      const Eigen::Vector3f tangent_local = worldVectorToLocal(
          active_local_frame, tangent_world);
      const float reward_scale = tinyMpcProgressTerminalRewardScale(
          (sample.speed_mps + full_progress_reward_speed_bias_mps)
              * curvature_speed_scale,
          terminal_speed_limit_mps) * curvature_speed_scale;
      if (advance && trajectory_handoff_hold_steps == 0u &&
          !terminal_position_hold && !progress_path.complete) {
        progress_state_linear_cost[i].segment<3>(6) =
            -reward_scale * (float)TINYMPC_PROGRESS_REWARD_WEIGHT
                * tangent_local;
      }
#if TINYMPC_PATH_TUNNEL_ENABLE
      const TinyMpcTunnelVector tunnel_tangent = tinyMpcTunnelVector(
          tangent_world.x(), tangent_world.y(), tangent_world.z());
      const TinyMpcTunnelVector *previous_normal_1 =
          previous_tunnel_frame.valid
              ? &previous_tunnel_frame.normal_1 : NULL;
      const TinyMpcTunnelFrame tunnel_frame = tinyMpcPathTunnelFrame(
          tunnel_tangent, previous_normal_1);
      /* X[0] is fixed to the measured initial condition in tiny_SolveLqr().
       * Constraining it cannot change the vehicle state and makes the QP
       * infeasible whenever a disturbance starts outside the tunnel. */
      if (i > 0 && tunnel_frame.valid) {
        setPathTunnelHalfspaces(i, position_world, tunnel_frame);
        previous_tunnel_frame = tunnel_frame;
      }
#endif
      const float geometric_yaw = terminal_position_hold
          ? horizon_yaw : atan2f(tangent_world.y(), tangent_world.x());
      if (progress_heading_alignment_active) {
        if (i == 0) {
          horizon_yaw = reference_yaw_phase_rad;
        } else {
          horizon_yaw = tinyMpcProgressSlewYawToward(
              horizon_yaw, geometric_yaw,
              heading_alignment_rate_rad_s * DT);
        }
      } else {
        horizon_yaw = tinyMpcProgressHorizonYaw(
            geometric_yaw, horizon_yaw, reference_yaw_phase_rad,
            maximum_local_yaw_deviation_rad, false);
      }
      const float feedforward_speed_mps =
          horizon_speed_mps
              + full_progress_reward_speed_bias_mps * reward_scale;
      const Eigen::Vector3f curvature_vector_world(
          trajectory_cos_yaw * sample.curvature_vector_per_m.x
              - trajectory_sin_yaw * sample.curvature_vector_per_m.y,
          trajectory_sin_yaw * sample.curvature_vector_per_m.x
              + trajectory_cos_yaw * sample.curvature_vector_per_m.y,
          sample.curvature_vector_per_m.z);
      const Eigen::Vector3f reference_acceleration =
          curvature_vector_world
              * (feedforward_speed_mps * feedforward_speed_mps)
          + tangent_world * (
              horizon_tangential_acceleration_mps2
              - TINYMPC_BANK_MODEL_DRAG_X_N_PER_MPS
                  * feedforward_speed_mps / TINYMPC_BANK_MODEL_MASS_KG);
      const TinyMpcProgressBankReference bank_reference =
          tinyMpcProgressBankReferenceFromAcceleration(
              reference_acceleration.x(), reference_acceleration.y(),
              reference_acceleration.z(), horizon_yaw, 9.81f);
      const float raw_reference_roll = bank_reference.valid
          ? bank_reference.roll_rad : 0.0f;
      const float raw_reference_pitch = bank_reference.valid
          ? bank_reference.pitch_rad : 0.0f;
      const float reference_roll = raw_reference_roll;
      /* Vision speed limits may change between adjacent preview knots.  The
       * corresponding longitudinal acceleration is valid, but presenting its
       * full attitude step as body-rate feed-forward excited the direct motor
       * model before the position loop could respond.  Bound only pitch here;
       * lateral bank and the separate cached emergency profile stay intact. */
      constexpr float progress_pitch_rate_limit_rad_s = 0.30f;
      const float reference_pitch = tinyMpcPathClamp(
          raw_reference_pitch,
          previous_horizon_pitch_rad - progress_pitch_rate_limit_rad_s * DT,
          previous_horizon_pitch_rad + progress_pitch_rate_limit_rad_s * DT);
      previous_horizon_pitch_rad = reference_pitch;
      const float raw_tilt_rad = hypotf(
          raw_reference_roll, reference_pitch);
      if (sample.curvature_magnitude_per_m > diag_curvature_per_m) {
        diag_curvature_per_m = sample.curvature_magnitude_per_m;
      }
      diag_speed_mps = T_MAX(diag_speed_mps, feedforward_speed_mps);
      const float raw_yaw_rate_rad_s =
          sample.curvature_per_m * feedforward_speed_mps;
      if (fabsf(raw_yaw_rate_rad_s) > fabsf(diag_yaw_rate_rad_s)) {
        diag_yaw_rate_rad_s = raw_yaw_rate_rad_s;
      }
      if (raw_tilt_rad > diag_tilt_rad) {
        diag_roll_rad = raw_reference_roll;
        diag_pitch_rad = reference_pitch;
        diag_tilt_rad = raw_tilt_rad;
      }
      struct quat reference_attitude = rpy2quat(
          mkvec(reference_roll, reference_pitch, horizon_yaw));
#if TINYMPC_FLIP_ENABLE
      if (flip_reference_active) {
        const float flip_sigma = tinyMpcFlipClamp01(
            flip_state.sigma + (float)i * DT / flip_config.duration_s);
        const TinyMpcFlipSample flip_sample = tinyMpcFlipSample(
            &flip_config, flip_sigma, horizon_yaw, raw_yaw_rate_rad_s);
        if (flip_sample.valid) {
          reference_attitude = mkquat(
              flip_sample.attitude_world_body.x,
              flip_sample.attitude_world_body.y,
              flip_sample.attitude_world_body.z,
              flip_sample.attitude_world_body.w);
          flip_reference_body_rates[i] = flip_sample.body_rate_rad_s;
          flip_reference_knot_active[i] = true;
          for (int motor = 0; motor < NINPUTS; ++motor) {
            flip_reference_motor_thrust_n[i][motor] =
                flip_sample.motor_thrust_n[motor];
          }
        }
      }
#endif
      progress_reference_attitudes[i] = tinyMpcProgressQuaternionMake(
          reference_attitude.x, reference_attitude.y,
          reference_attitude.z, reference_attitude.w);
      const Eigen::Vector3f reference_velocity_world =
          tangent_world * horizon_speed_mps;
      setLocalReferenceState(
          Xref[i], position_world, reference_attitude,
          reference_velocity_world, Eigen::Vector3f::Zero());
      if (terminal_position_hold) {
        /* Preserve the physical endpoint as the position target and provide a
         * convergent recovery velocity instead of the degenerate final
         * tangent.  Completion remains governed by measured endpoint distance
         * in tinyMpcProgressPathUpdate(). */
        constexpr float terminal_recovery_gain_per_s = 1.5f;
        Xref[i].segment<3>(6) = terminal_recovery_gain_per_s
            * (Xref[i].segment<3>(0) - x0.segment<3>(0));
      }
      reference_yaw_unwrapped_rad[i] = horizon_yaw;
      if (i < NHORIZON - 1) {
        const float thrust_scale = bank_reference.valid
            ? bank_reference.thrust_scale : 0.0f;
        for (int motor = 0; motor < NINPUTS; ++motor) {
          const float raw_uref = flip_reference_knot_active[i]
              ? flip_reference_motor_thrust_n[i][motor]
                  - tinympc_generated_physical_hover_thrust[motor]
              : tinympc_generated_physical_hover_thrust[motor] * thrust_scale;
          Uref[i](motor) = T_MIN(T_MAX(raw_uref, lcu(motor)), ucu(motor));
          diag_uref_clamped = diag_uref_clamped || Uref[i](motor) != raw_uref;
        }
        diag_thrust_scale = T_MAX(diag_thrust_scale, thrust_scale);
        horizon_progress = tinyMpcProgressPathAdvance(
            &progress_path, horizon_progress, horizon_speed_mps * DT);
        const TinyMpcPathSample next_sample = tinyMpcProgressPathSample(
            &progress_path, horizon_progress);
        const float next_curvature_speed_scale =
            tinyMpcProgressCentripetalSpeedScale(
                next_sample.speed_mps + full_progress_reward_speed_bias_mps,
                next_sample.curvature_magnitude_per_m,
                (float)TINYMPC_PROGRESS_MAX_CENTRIPETAL_ACCELERATION_MPS2);
        const float next_terminal_speed_limit_mps =
            tinyMpcProgressTerminalSpeedLimit(
                &progress_path, horizon_progress,
                (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2);
        float next_target_speed_mps = T_MIN(
            next_sample.speed_mps * next_curvature_speed_scale,
            next_terminal_speed_limit_mps);
#if defined(TINYMPC_VISION_ESPNET_DRONET_ENABLE)
        next_target_speed_mps = T_MIN(
            next_target_speed_mps, vision_track_speed_limit_mps);
#endif
#if TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
        next_target_speed_mps *= dronet_v2_speed_scale;
#endif
#if TINYMPC_VISION_RL_RESIDUAL_ENABLE
        next_target_speed_mps *= vision_residual_progress_speed_scale;
#endif
        const float next_speed_mps = tinyMpcProgressSlewSpeed(
            horizon_speed_mps, next_target_speed_mps,
            progress_entry_acceleration_mps2,
            (float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2, DT);
        const float next_reward_scale = tinyMpcProgressTerminalRewardScale(
            (next_sample.speed_mps + full_progress_reward_speed_bias_mps)
                * next_curvature_speed_scale,
            next_terminal_speed_limit_mps) * next_curvature_speed_scale;
        const float next_feedforward_speed_mps = next_speed_mps
            + full_progress_reward_speed_bias_mps * next_reward_scale;
        /* Knot zero computes attitude feed-forward from the reward-biased
         * speed.  Use the same physical speed at every later knot; dropping
         * the bias here introduced a one-knot pitch step (about 1.45 rad/s at
         * cruise) that the direct motor controller correctly tried to track. */
        horizon_tangential_acceleration_mps2 = tinyMpcPathClamp(
            T_MIN((next_speed_mps - horizon_speed_mps) / DT, 0.0f)
                + tinyMpcProgressTangentialAcceleration(
                    next_feedforward_speed_mps,
                    measured_tangent_speed_mps),
            -(float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2, 0.50f);
        horizon_speed_mps = next_speed_mps;
      }
    }
    tinyMpcProgressQuaternionHorizonBodyRates(
        progress_reference_attitudes, NHORIZON, DT,
        progress_reference_body_rates, progress_reconstruction_residual_rad);
    float progress_body_rate_max_p_rad_s = 0.0f;
    float progress_body_rate_max_q_rad_s = 0.0f;
    float progress_body_rate_max_r_rad_s = 0.0f;
    float progress_body_rate_max_norm_rad_s = 0.0f;
    float progress_reconstruction_max_residual_rad = 0.0f;
    bool progress_body_rate_reconstruction_violation = false;
    constexpr float progress_reconstruction_tolerance_rad = 1.0e-5f;
    for (int i = 0; i < NHORIZON; ++i) {
      const TinyMpcProgressBodyRate body_rate = flip_reference_knot_active[i]
          ? flip_reference_body_rates[i]
          : progress_reference_body_rates[i];
      Xref[i].segment<3>(9) << body_rate.x, body_rate.y, body_rate.z;
      progress_body_rate_max_p_rad_s = T_MAX(
          progress_body_rate_max_p_rad_s, fabsf(body_rate.x));
      progress_body_rate_max_q_rad_s = T_MAX(
          progress_body_rate_max_q_rad_s, fabsf(body_rate.y));
      progress_body_rate_max_r_rad_s = T_MAX(
          progress_body_rate_max_r_rad_s, fabsf(body_rate.z));
      const float rate_norm_rad_s = sqrtf(
          body_rate.x * body_rate.x + body_rate.y * body_rate.y
          + body_rate.z * body_rate.z);
      progress_body_rate_max_norm_rad_s = T_MAX(
          progress_body_rate_max_norm_rad_s, rate_norm_rad_s);
      if (!isfinite(body_rate.x) || !isfinite(body_rate.y)
          || !isfinite(body_rate.z) || !isfinite(rate_norm_rad_s)) {
        progress_body_rate_reconstruction_violation = true;
        ++progress_body_rate_reconstruction_violation_count;
      }
      if (i < NHORIZON - 1) {
        const float residual_rad = progress_reconstruction_residual_rad[i];
        if (isfinite(residual_rad)) {
          progress_reconstruction_max_residual_rad = T_MAX(
              progress_reconstruction_max_residual_rad, residual_rad);
        }
        if (!isfinite(residual_rad)
            || residual_rad > progress_reconstruction_tolerance_rad) {
          progress_body_rate_reconstruction_violation = true;
          ++progress_body_rate_reconstruction_violation_count;
        }
      }
    }
    if ((progress_invariant_diag_cycle % (uint32_t)MPC_RATE) == 0u) {
      DEBUG_PRINT(
          "PROGRESS tangential_accel command_speed_mps=%.5f measured_tangent_speed_mps=%.5f transient_mps2=%.5f steady_drag_mps2=%.5f total_mps2=%.5f time_constant_s=0.35000 maximum_transient_mps2=0.50000\n",
          (double)first_feedforward_speed_mps,
          (double)measured_tangent_speed_mps,
          (double)first_tangential_acceleration_mps2,
          (double)steady_drag_compensation_mps2,
          (double)(first_tangential_acceleration_mps2
              + steady_drag_compensation_mps2));
      DEBUG_PRINT(
          "PROGRESS bodyrate max_abs_pqr_rad_s=(%.5f,%.5f,%.5f) max_norm_rad_s=%.5f reconstruction_max_rad=%.8f reconstruction_tolerance_rad=0.00001000 violation=%u violation_count=%lu terminal=copy_final_interval\n",
          (double)progress_body_rate_max_p_rad_s,
          (double)progress_body_rate_max_q_rad_s,
          (double)progress_body_rate_max_r_rad_s,
          (double)progress_body_rate_max_norm_rad_s,
          (double)progress_reconstruction_max_residual_rad,
          progress_body_rate_reconstruction_violation ? 1u : 0u,
          (unsigned long)progress_body_rate_reconstruction_violation_count);
    }
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
    }
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
  if (advance && en_traj) {
    if (trajectory_handoff_hold_steps > 0) {
      --trajectory_handoff_hold_steps;
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
        &work, k, TINYMPC_PERCEPTION_HALFSPACE_SLOT,
        &normal_local, &velocity_normal, boundary_local,
        perception_halfspace_penalty, 1);
  }
}

static void __attribute__((unused)) updateRaceIntent(const state_t *state) {
  TinyRacerPerceptionObservation observation = {};
  sequentialObstacleLinkGetLatest(&observation);
#if defined(TINYMPC_PAPER_ABLATION_ENABLE_GATE)
  /* The paper-ablation course accelerates under the nominal straight-flight
   * controller and enables perception at x=3 m.  This keeps approach-speed
   * qualification independent of startup/background predictions while still
   * preserving every delivered inference and its recurrent/filter state. */
  if (state->position.x <
      (float)TINYMPC_PAPER_ABLATION_CONTROL_ENABLE_X_M) {
    observation = {};
  }
#endif
  filterPerceptionClearances(observation);
  maybeFuseGateCenterBearing(observation, *state);
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
#if defined(TINYMPC_VISION_ESPNET_DRONET_ENABLE)
  const bool allow_race_constraint_activation = false;
#else
  const bool allow_race_constraint_activation =
      !perception_recovery_active && race_intent.mode == TINYRACER_RACE_TRACK;
#endif
  tinyRacerRaceUpdate(
      &race_state, &observation, &race_config, DT, release_ready,
      perception_recovery_active,
      allow_race_constraint_activation,
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
#if TINYMPC_VISION_RL_RESIDUAL_ENABLE
  applyVisionResidualReference(observation);
#elif !TINYMPC_GATE_POSITION_FUSION_ENABLE
  applyVisionNavigation(
      observation);
#endif
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE || TINYMPC_JOINT_GATE_RL_ENABLE
  /* A committed visual gate owns a short straight crossing horizon. In the
   * visual-gate mode, REARM continues the intended acceleration- and
   * yaw-limited tangent handoff after transit clears the association. A dodge,
   * halfspace, or recovery otherwise takes the horizon back. */
  bool gate_servo_reference_active = gate_poc_associated;
#if TINYMPC_GATE_OBSTACLE_POC_ENABLE
  gate_servo_reference_active = gate_servo_reference_active ||
      gate_visual_phase == GATE_VISUAL_REARM;
#endif
  if (gate_servo_reference_active &&
      !race_intent.constraint_active &&
      tinyRacerGateServoAllowed(
          race_intent.mode, dodge_state.phase, perception_halfspace_active,
          perception_recovery_active)) {
    applyGateVisualServo(observation);
  }
#endif
  if (perception_recovery_active && !race_intent.constraint_active &&
      race_state.clear_samples >= race_config.clear_samples_required &&
      perception_recovery_distance_m >= perception_pass_distance_m) {
    rejoinTrajectory(position_world, heading_world);
    perception_recovery_active = false;
    perception_halfspace_active = false;
    perception_binary_constraint = false;
    perception_obstacle_active = false;
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


#if defined(TINYMPC_USE_ACTUATOR_LTI)
typedef Eigen::Matrix<float, TINYMPC_LEVEL_ACTUATOR_STATE_DIM, 1>
    LevelActuatorState;
static const TinyMpcBankedModelData *active_level_actuator_model =
    &tinympc_banked_models[TINYMPC_BANK_MODEL_LEVEL];
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

static bool levelBankErrorCoordinatesActive(
    const TinyMpcBankedModelData& model) {
  /* A full flip cannot be tracked in the absolute Rodrigues chart because
   * q_vec/q_w is singular at 180 degrees. During the maneuver, run the
   * existing level bundle in the same reference-relative attitude-error
   * coordinates used by banked bundles. The phase primitive remains the
   * source of Rref, omega_ref, and u_ref. */
  return flip_reference_active || power_loop_reference_active
      || model.model_id != TINYMPC_BANK_MODEL_LEVEL;
}

static float levelModelInputOriginN(
    const TinyMpcBankedModelData& model, int motor) {
  return levelBankErrorCoordinatesActive(model)
      ? model.physical_input[motor]
      : TINYMPC_LEVEL_HOVER_THRUST_N;
}

static float levelReferenceState(
    const TinyMpcBankedModelData& model, int knot, int state) {
  if (state < NSTATES) {
    return levelBankErrorCoordinatesActive(model) ? 0.0f : Xref[knot](state);
  }
  const int motor = state - NSTATES;
  const int input_knot = knot < NHORIZON - 1 ? knot : NHORIZON - 2;
  const float input_origin_n = levelModelInputOriginN(model, motor);
  const float target_thrust = T_MIN(T_MAX(
      input_origin_n + Uref[input_knot](motor), 0.0f),
      TINYMPC_LEVEL_MAX_MOTOR_THRUST_N);
  return thrustToLevelRotorState(target_thrust)
      - thrustToLevelRotorState(input_origin_n);
}

static float levelStateYawRad(const VectorNf& state);

static float levelProgressLinearCost(
    const TinyMpcBankedModelData& model, int knot, int state) {
  if (state >= NSTATES || state < 6 || state > 8) {
    return 0.0f;
  }
  if (!levelBankErrorCoordinatesActive(model) || state == 8) {
    return progress_state_linear_cost[knot](state);
  }
  /* Banked models use velocity error in the reference-yaw frame. Transform
   * the world/local-chart reward covector into those error coordinates. */
  const float yaw = levelStateYawRad(Xref[knot]);
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  const float qx = progress_state_linear_cost[knot](6);
  const float qy = progress_state_linear_cost[knot](7);
  return state == 6
      ? cosine * qx + sine * qy
      : -sine * qx + cosine * qy;
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

#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
static void solveReactivePowerLoopStoredLtv(void) {
  level_p[NHORIZON - 1].setZero();
  info.pri_res = 0.0f;
  info.dua_res = 0.0f;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (int knot = NHORIZON - 2; knot >= 0; --knot) {
      const int interval = reactivePowerLoopCacheInterval(knot);
      const int a_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM *
          TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      const int b_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM *
          NINPUTS;
      const int f_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      const int k_offset = interval * NINPUTS *
          TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      const int h_offset = interval * NINPUTS * NINPUTS;
      LevelActuatorState value_gradient;
      float rhs[NINPUTS];
      for (int row = 0; row < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
           ++row) {
        value_gradient(row) = level_p[knot + 1](row) +
            tinympc_reactive_loop_stored_ltv_P_affine[f_offset + row];
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -TINYMPC_REACTIVE_LOOP_STORED_LTV_RHO *
            (level_ZU_new[knot](motor) - level_YU[knot](motor));
        for (int state = 0;
             state < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++state) {
          value += tinympc_reactive_loop_stored_ltv_B[
              b_offset + state * NINPUTS + motor] * value_gradient(state);
        }
        rhs[motor] = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = 0.0f;
        for (int column = 0; column < NINPUTS; ++column) {
          value += tinympc_reactive_loop_stored_ltv_Hinv[
              h_offset + motor * NINPUTS + column] * rhs[column];
        }
        level_d[knot](motor) = value;
      }
      for (int state = 0;
           state < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++state) {
        float value = 0.0f;
        for (int row = 0;
             row < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++row) {
          value += tinympc_reactive_loop_stored_ltv_A[
              a_offset + row * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM +
                  state] * value_gradient(row);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value -= tinympc_reactive_loop_stored_ltv_K[
              k_offset + motor * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM +
                  state] * rhs[motor];
        }
        level_p[knot](state) = value;
      }
    }

    float actual[NSTATES];
    float reference[NSTATES];
    float error[NSTATES];
    for (int state = 0; state < NSTATES; ++state) {
      actual[state] = x0(state);
      reference[state] = Xref[0](state);
    }
    /* Never reconstruct the absolute acrobatic attitudes from Rodrigues
     * states here. Both charts are singular at the inverted point even when
     * the reference-relative error seen by the stored LTV cache is tiny. */
    const struct quat actual_local =
        worldQuaternionToLocalQuaternion(active_local_frame, attitude);
    const uint16_t reference_index = reactivePowerLoopReferenceIndex(0);
    const float *reference_data =
        tinympc_reactive_loop_reference_data[reference_index];
    const struct quat maneuver_attitude = mkquat(
        reference_data[4], reference_data[5], reference_data[6],
        reference_data[3]);
    const struct quat reference_world = composeWorldYawWithManeuver(
        rpy2quat(mkvec(0.0f, 0.0f, reactive_power_loop_yaw_world_rad)),
        maneuver_attitude);
    const struct quat reference_local =
        worldQuaternionToLocalQuaternion(active_local_frame, reference_world);
    const TinyMpcFrenetQuaternion actual_quaternion = {
        actual_local.w, actual_local.x, actual_local.y, actual_local.z};
    const TinyMpcFrenetQuaternion reference_quaternion = {
        reference_local.w, reference_local.x, reference_local.y,
        reference_local.z};
    tinyMpcStoredLtvErrorEncodeQuaternions(
        actual, reference, actual_quaternion, reference_quaternion, error);
    for (int state = 0; state < NSTATES; ++state) {
      level_Xhrz[0](state) = error[state];
    }
    const int initial_interval = reactivePowerLoopCacheInterval(0);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      level_Xhrz[0](NSTATES + motor) =
          level_motor_rotor_state_snapshot(motor) -
          tinympc_reactive_loop_stored_ltv_motor_state_reference[
              initial_interval * NINPUTS + motor];
    }
    for (int knot = 0; knot < NHORIZON - 1; ++knot) {
      const int interval = reactivePowerLoopCacheInterval(knot);
      const int a_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM *
          TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      const int b_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM *
          NINPUTS;
      const int f_offset = interval * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      const int k_offset = interval * NINPUTS *
          TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM;
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -level_d[knot](motor);
        for (int state = 0;
             state < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++state) {
          value -= tinympc_reactive_loop_stored_ltv_K[
              k_offset + motor * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM +
                  state] * level_Xhrz[knot](state);
        }
        level_Uhrz[knot](motor) = value;
      }
      for (int state = 0;
           state < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++state) {
        float value = tinympc_reactive_loop_stored_ltv_affine[f_offset + state];
        for (int column = 0;
             column < TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM; ++column) {
          value += tinympc_reactive_loop_stored_ltv_A[
              a_offset + state * TINYMPC_REACTIVE_LOOP_STORED_LTV_STATE_DIM +
                  column] * level_Xhrz[knot](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value += tinympc_reactive_loop_stored_ltv_B[
              b_offset + state * NINPUTS + motor] *
              level_Uhrz[knot](motor);
        }
        level_Xhrz[knot + 1](state) = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float baseline =
            tinympc_reactive_loop_stored_ltv_motor_command_reference[
                interval * NINPUTS + motor];
        const float lower = -baseline;
        const float upper = TINYMPC_LEVEL_MAX_MOTOR_THRUST_N - baseline;
        const float projected = T_MIN(T_MAX(
            level_YU[knot](motor) + level_Uhrz[knot](motor), lower), upper);
        level_YU[knot](motor) += level_Uhrz[knot](motor) - projected;
        level_ZU_new[knot](motor) = projected;
      }
    }
  }
  for (int knot = 0; knot < NHORIZON - 1; ++knot) {
    const int interval = reactivePowerLoopCacheInterval(knot);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float baseline =
          tinympc_reactive_loop_stored_ltv_motor_command_reference[
              interval * NINPUTS + motor];
      Uhrz[knot](motor) = baseline + level_Uhrz[knot](motor) -
          TINYMPC_LEVEL_HOVER_THRUST_N;
      ZU_new[knot](motor) = baseline + level_ZU_new[knot](motor) -
          TINYMPC_LEVEL_HOVER_THRUST_N;
      info.pri_res = T_MAX(info.pri_res, fabsf(
          level_Uhrz[knot](motor) - level_ZU_new[knot](motor)));
    }
  }
  info.iter = 5;
}
#endif

static float levelStateBankRad(const VectorNf& state) {
  const float rx = state(3);
  const float ry = state(4);
  const float rz = state(5);
  const float inverse_norm = 1.0f / sqrtf(
      1.0f + rx * rx + ry * ry + rz * rz);
  const float qw = inverse_norm;
  const float qx = rx * inverse_norm;
  const float qy = ry * inverse_norm;
  const float qz = rz * inverse_norm;
  return atan2f(
      2.0f * (qw * qx + qy * qz),
      1.0f - 2.0f * (qx * qx + qy * qy));
}

static float levelStatePitchRad(const VectorNf& state) {
  const float rx = state(3);
  const float ry = state(4);
  const float rz = state(5);
  const float inverse_norm = 1.0f / sqrtf(
      1.0f + rx * rx + ry * ry + rz * rz);
  const float qw = inverse_norm;
  const float qx = rx * inverse_norm;
  const float qy = ry * inverse_norm;
  const float qz = rz * inverse_norm;
  const float pitch_sine = T_MIN(T_MAX(
      2.0f * (qw * qy - qz * qx), -1.0f), 1.0f);
  return asinf(pitch_sine);
}

static float levelStateYawRad(const VectorNf& state) {
  const float rx = state(3);
  const float ry = state(4);
  const float rz = state(5);
  const float inverse_norm = 1.0f / sqrtf(
      1.0f + rx * rx + ry * ry + rz * rz);
  const float qw = inverse_norm;
  const float qx = rx * inverse_norm;
  const float qy = ry * inverse_norm;
  const float qz = rz * inverse_norm;
  return atan2f(
      2.0f * (qw * qz + qx * qy),
      1.0f - 2.0f * (qy * qy + qz * qz));
}

static void applyLevelBankInputFeedforward(
    const TinyMpcBankedModelData& model) {
  if (model.model_id == TINYMPC_BANK_MODEL_LEVEL) {
    return;
  }
  float collective_origin_n = 0.0f;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    collective_origin_n += model.physical_input[motor];
  }
  collective_origin_n /= (float)NINPUTS;
  for (int knot = 0; knot < NHORIZON - 1; ++knot) {
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float differential_n =
          model.physical_input[motor] - collective_origin_n;
      const float desired_physical_n = T_MIN(T_MAX(
          TINYMPC_LEVEL_HOVER_THRUST_N + Uref[knot](motor)
              + differential_n,
          0.0f), TINYMPC_LEVEL_MAX_MOTOR_THRUST_N);
      Uref[knot](motor) =
          desired_physical_n - model.physical_input[motor];
    }
  }
}

static void levelBankInitialErrorState(
    const TinyMpcBankedModelData& model, LevelActuatorState& state) {
  if (!levelBankErrorCoordinatesActive(model)) {
    for (int index = 0; index < NSTATES; ++index) {
      state(index) = x0(index);
    }
  } else {
    float actual[NSTATES];
    float reference[NSTATES];
    float error[NSTATES];
    for (int index = 0; index < NSTATES; ++index) {
      actual[index] = x0(index);
      reference[index] = Xref[0](index);
    }
    tinyMpcFrenetErrorEncode(actual, reference, error);
    for (int index = 0; index < NSTATES; ++index) {
      state(index) = error[index];
    }
  }
  for (int motor = 0; motor < NINPUTS; ++motor) {
    state(NSTATES + motor) = level_motor_rotor_state_snapshot(motor)
        - thrustToLevelRotorState(levelModelInputOriginN(model, motor));
  }
}

static void seedLevelActuatorOptimizerFromReference(void) {
  const TinyMpcBankedModelData& selected_model =
      *active_level_actuator_model;
  for (int knot = 0; knot < NHORIZON; ++knot) {
    for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
      const float reference = levelReferenceState(
          selected_model, knot, state);
      level_Xhrz[knot](state) = reference;
      level_ZX_new[knot](state) = reference;
      level_YX[knot](state) = 0.0f;
      level_p[knot](state) = 0.0f;
    }
    if (knot < NHORIZON - 1) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        level_Uhrz[knot](motor) = Uref[knot](motor);
        level_ZU_new[knot](motor) = Uref[knot](motor);
        level_YU[knot](motor) = 0.0f;
        level_d[knot](motor) = 0.0f;
      }
    }
  }
}

static void updateLevelActuatorModelSelection(void) {
  static bool maneuver_model_override_active = false;
  level_model_switched_this_solve = false;
  const bool aggressive_maneuver_active =
      flip_reference_active || power_loop_reference_active
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
      || reactive_power_loop_command.owns_reference
#endif
      ;
  if (aggressive_maneuver_active) {
    if (!maneuver_model_override_active) {
      tinyMpcBankSelectorReset(
          &level_bank_selector, &level_bank_selector_config);
      tinyMpcBrakingSelectorReset(
          &level_braking_selector, &level_braking_selector_config);
      level_bank_selection = {
          TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
      level_braking_selection = {
          TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
      active_level_actuator_model =
          &tinympc_banked_models[TINYMPC_BANK_MODEL_LEVEL];
      resetLevelActuatorDuals();
      maneuver_model_override_active = true;
      DEBUG_PRINT(
          "MANEUVER model override=level_frozen reference_coordinates=relative phase_ltv=%s kind=%s\n",
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
          reactive_power_loop_command.owns_reference ? "cached" : "unavailable",
#else
          "unavailable",
#endif
          power_loop_reference_active ? "power_loop" :
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
          (reactive_power_loop_command.owns_reference
              ? "reactive_power_loop" : "flip")
#else
          "flip"
#endif
          );
    }
    return;
  }
  if (maneuver_model_override_active) {
    tinyMpcBankSelectorReset(
        &level_bank_selector, &level_bank_selector_config);
    tinyMpcBrakingSelectorReset(
        &level_braking_selector, &level_braking_selector_config);
    level_bank_selection = {
        TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
    level_braking_selection = {
        TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
    active_level_actuator_model =
        &tinympc_banked_models[TINYMPC_BANK_MODEL_LEVEL];
    resetLevelActuatorDuals();
    maneuver_model_override_active = false;
    DEBUG_PRINT("MANEUVER model override released; bank selector reset\n");
  }
  const int previous_model = active_level_actuator_model->model_id;
  /* AVOID_LEFT/AVOID_RIGHT change the position and velocity reference, not
   * the vehicle's current operating point. Never select a steady-turn cache
   * from that discrete phase alone: the ordinary selector below admits a
   * banked chart only when reference and measured motion are local to it. */
  const float reference_bank_rad = levelStateBankRad(Xref[0]);
  const float measured_bank_rad = levelStateBankRad(x0);
  const float reference_pitch_rad = levelStatePitchRad(Xref[0]);
  const float measured_pitch_rad = levelStatePitchRad(x0);
  const bool terminal_transition_active =
      tinyMpcProgressPathCompletedLaps(
          &progress_path, progress_path.progress)
          >= (uint16_t)TINYMPC_PROGRESS_LAPS;
  const float selector_measured_bank_rad = terminal_transition_active
      ? tinyMpcBankMeasuredDemandForReference(
          reference_bank_rad, measured_bank_rad,
          level_bank_selector_config.exit_bank_rad)
      : measured_bank_rad;
  const Eigen::Vector3f reference_velocity = Xref[0].segment<3>(6);
  const float reference_tangent_speed_mps = reference_velocity.norm();
  const float measured_tangent_speed_mps = reference_tangent_speed_mps > 1.0e-5f
      ? x0.segment<3>(6).dot(
          reference_velocity / reference_tangent_speed_mps)
      : 0.0f;
  const float next_reference_tangent_speed_mps =
      reference_tangent_speed_mps > 1.0e-5f
      ? Xref[1].segment<3>(6).dot(
          reference_velocity / reference_tangent_speed_mps)
      : 0.0f;
  const float reference_deceleration_mps2 = T_MAX(
      (reference_tangent_speed_mps - next_reference_tangent_speed_mps) / DT,
      0.0f);
  const float braking_cache_deceleration_mps2 =
#if TINYMPC_BRAKING_CACHE_ENABLE
#if defined(TINYMPC_PAPER_EMERGENCY_TERMINAL_STOP)
      !paper_emergency_terminal_stop_latched &&
#endif
      fabsf(reference_deceleration_mps2
          - TINYMPC_BRAKING_DECELERATION_MPS2) <= 1.0f
      ? TINYMPC_BRAKING_DECELERATION_MPS2 : 0.0f;
#else
      0.0f;
#endif

  /* Emergency braking changes the reference immediately, but a cached model
   * is still only valid near its identified speed and pitch operating point.
   * Use the same locality-gated selector as every other braking transition;
   * the level model carries the initial transient until a brake chart is
   * actually local. */
  level_braking_selection = tinyMpcBrakingSelectorUpdate(
      &level_braking_selector, &level_braking_selector_config,
      braking_cache_deceleration_mps2, reference_pitch_rad,
      measured_pitch_rad, reference_bank_rad, measured_bank_rad,
      T_MAX(measured_tangent_speed_mps, 0.0f));
  int selected_model = level_braking_selection.active_model_id;
  bool bank_entry_wait = false;
  bool bank_entry_supported = true;
  if (selected_model != TINYMPC_BRAKING_MODEL_LEVEL) {
    if (level_bank_selector.active_model != TINYMPC_BANK_MODEL_LEVEL) {
      tinyMpcBankSelectorReset(
          &level_bank_selector, &level_bank_selector_config);
    }
    level_bank_selection = {
        (TinyMpcBankModelId)selected_model, reference_pitch_rad,
        true, selected_model != previous_model, selected_model != previous_model};
  } else {
    const bool entering_from_level =
        level_bank_selector.active_model == TINYMPC_BANK_MODEL_LEVEL;
    bank_entry_supported = tinyMpcBankEntrySupportedByMotion(
        reference_bank_rad, measured_bank_rad,
        reference_tangent_speed_mps, measured_tangent_speed_mps);
    const float selector_reference_bank_rad =
        entering_from_level && !bank_entry_supported
        ? 0.0f : reference_bank_rad;
    TinyMpcBankSelection turn_selection = tinyMpcBankSelectorUpdate(
        &level_bank_selector, &level_bank_selector_config,
        selector_reference_bank_rad, selector_measured_bank_rad);
    selected_model = (int)turn_selection.active_model;
    turn_selection.switched = selected_model != previous_model;
    turn_selection.reset_optimizer = turn_selection.switched;
    level_bank_selection = turn_selection;
    bank_entry_wait = entering_from_level && !bank_entry_supported
        && fabsf(reference_bank_rad)
            >= level_bank_selector_config.enter_bank_rad;
  }
  if (selected_model < 0 || selected_model >= TINYMPC_BANK_MODEL_COUNT) {
    return;
  }
  const TinyMpcBankedModelData *selected_bundle =
      &tinympc_banked_models[selected_model];
  if (level_bank_selection.switched) {
    active_level_actuator_model = selected_bundle;
    if (level_bank_selection.reset_optimizer) {
      resetLevelActuatorDuals();
    }
    level_model_switched_this_solve = true;
    if (selected_model >= TINYMPC_BANK_MODEL_BRAKE_LOW
        && selected_model <= TINYMPC_BANK_MODEL_BRAKE_MAXIMUM) {
      DEBUG_PRINT(
          "BRAKE model switch id=%d decel=%.3fm/s2 reference_pitch=%.4frad measured_pitch=%.4frad nominal_pitch=%.4frad speed=%.2fm/s count=%lu\n",
          selected_model, (double)reference_deceleration_mps2,
          (double)reference_pitch_rad, (double)measured_pitch_rad,
          (double)selected_bundle->nominal_pitch_rad,
          (double)selected_bundle->nominal_speed_mps,
          (unsigned long)level_braking_selector.switch_count);
    } else {
      DEBUG_PRINT(
          "BANK model switch id=%d demand=%.4frad reference=%.4frad measured=%.4frad nominal=%.4frad speed=%.2fm/s count=%lu\n",
          selected_model,
          (double)level_bank_selection.signed_bank_demand_rad,
          (double)reference_bank_rad, (double)measured_bank_rad,
          (double)selected_bundle->nominal_roll_rad,
          (double)selected_bundle->nominal_speed_mps,
          (unsigned long)level_bank_selector.switch_count);
    }
  } else if (bank_entry_wait) {
    static uint16_t entry_wait_log_divider = 0u;
    if ((entry_wait_log_divider++ % 25u) == 0u) {
      DEBUG_PRINT(
          "BANK entry wait reference=%.4frad measured=%.4frad reference_speed=%.3fm/s measured_tangent=%.3fm/s thresholds=60%%bank,70%%speed\n",
          (double)reference_bank_rad, (double)measured_bank_rad,
          (double)reference_tangent_speed_mps,
          (double)measured_tangent_speed_mps);
    }
  }
#if TINYMPC_BRAKING_CACHE_ENABLE
  else if (selected_model == TINYMPC_BRAKING_MODEL_LEVEL
      && reference_deceleration_mps2
          >= level_braking_selector_config.enter_deceleration_mps2
      && fabsf(reference_bank_rad)
          <= level_braking_selector_config.maximum_level_roll_rad) {
    static uint16_t braking_entry_wait_log_divider = 0u;
    if ((braking_entry_wait_log_divider++ % 25u) == 0u) {
      DEBUG_PRINT(
          "BRAKE entry wait decel=%.3fm/s2 reference_pitch=%.4frad measured_pitch=%.4frad measured_speed=%.3fm/s locality=%u\n",
          (double)reference_deceleration_mps2,
          (double)reference_pitch_rad, (double)measured_pitch_rad,
          (double)measured_tangent_speed_mps,
          level_braking_selection.entry_local ? 1u : 0u);
    }
  }
#endif
  else {
    active_level_actuator_model = selected_bundle;
  }
  applyLevelBankInputFeedforward(*selected_bundle);
}

static void solveLevelActuatorLti(void) {
  const TinyMpcBankedModelData& active_model =
      *active_level_actuator_model;
  info.pri_res = 0.0f;
  info.dua_res = 0.0f;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (int row = 0; row < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++row) {
      float terminal = 0.0f;
      for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
        terminal -= active_model.P[
                row * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
            * levelReferenceState(active_model, NHORIZON - 1, column);
      }
      terminal += levelProgressLinearCost(
          active_model, NHORIZON - 1, row);
      level_p[NHORIZON - 1](row) = terminal;
    }

    for (int k = NHORIZON - 2; k >= 0; --k) {
      float r_tilde[NINPUTS];
      float rhs[NINPUTS];
      for (int motor = 0; motor < NINPUTS; ++motor) {
        r_tilde[motor] = -TINYMPC_LEVEL_ACTUATOR_RHO
            * (level_ZU_new[k](motor) - level_YU[k](motor));
        for (int column = 0; column < NINPUTS; ++column) {
          r_tilde[motor] -= active_model.R[
              motor * NINPUTS + column] * Uref[k](column);
        }
        float value = r_tilde[motor] + active_model.BPf[motor];
        for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
          value += active_model.B[state * NINPUTS + motor]
              * level_p[k + 1](state);
        }
        rhs[motor] = value;
      }
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = 0.0f;
        for (int column = 0; column < NINPUTS; ++column) {
          value += active_model.Quu_inv[motor * NINPUTS + column]
              * rhs[column];
        }
        level_d[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
        float value = -active_model.Q_diagonal[state]
            * levelReferenceState(active_model, k, state)
            + levelProgressLinearCost(active_model, k, state)
            - TINYMPC_LEVEL_ACTUATOR_RHO
                * (level_ZX_new[k](state) - level_YX[k](state))
            + active_model.APf[state];
        for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
          value += active_model.AmBKt[
                  state * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
              * level_p[k + 1](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value -= active_model.K[
                  motor * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + state]
              * r_tilde[motor];
          value += active_model.coeff_d2p[state * NINPUTS + motor]
              * level_d[k](motor);
        }
        level_p[k](state) = value;
      }
    }

    levelBankInitialErrorState(active_model, level_Xhrz[0]);
    for (int k = 0; k < NHORIZON - 1; ++k) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        float value = -level_d[k](motor);
        for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
          value -= active_model.K[
                  motor * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + state]
              * level_Xhrz[k](state);
        }
        level_Uhrz[k](motor) = value;
      }
      for (int state = 0; state < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++state) {
        float value = active_model.affine[state];
        for (int column = 0; column < TINYMPC_LEVEL_ACTUATOR_STATE_DIM; ++column) {
          value += active_model.A[
                  state * TINYMPC_LEVEL_ACTUATOR_STATE_DIM + column]
              * level_Xhrz[k](column);
        }
        for (int motor = 0; motor < NINPUTS; ++motor) {
          value += active_model.B[state * NINPUTS + motor]
              * level_Uhrz[k](motor);
        }
        level_Xhrz[k + 1](state) = value;
      }
    }

    for (int k = 0; k < NHORIZON - 1; ++k) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float input_origin_n = levelModelInputOriginN(
            active_model, motor);
        const float lower = -input_origin_n;
        const float upper = TINYMPC_LEVEL_MAX_MOTOR_THRUST_N
            - input_origin_n;
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
        Eigen::Vector3f a_position = data.a_pos_hs[k][h];
        Eigen::Vector3f a_velocity = data.a_vel_hs[k][h];
        float boundary = data.b_hs[k][h];
        if (levelBankErrorCoordinatesActive(active_model)) {
          const float yaw = levelStateYawRad(Xref[k]);
          const float cosine = cosf(yaw);
          const float sine = sinf(yaw);
          a_position = Eigen::Vector3f(
              cosine * data.a_pos_hs[k][h].x()
                  + sine * data.a_pos_hs[k][h].y(),
              -sine * data.a_pos_hs[k][h].x()
                  + cosine * data.a_pos_hs[k][h].y(),
              data.a_pos_hs[k][h].z());
          a_velocity = Eigen::Vector3f(
              cosine * data.a_vel_hs[k][h].x()
                  + sine * data.a_vel_hs[k][h].y(),
              -sine * data.a_vel_hs[k][h].x()
                  + cosine * data.a_vel_hs[k][h].y(),
              data.a_vel_hs[k][h].z());
          boundary -= data.a_pos_hs[k][h].dot(Xref[k].head(3))
              + data.a_vel_hs[k][h].dot(Xref[k].segment(6, 3));
        }
        const float violation =
            a_position.dot(level_ZX_new[k].head(3))
            + a_velocity.dot(level_ZX_new[k].segment(6, 3)) - boundary;
        const float positive_violation = T_MAX(violation, 0.0f);
        const float penalty = data.slack_penalty_hs[k][h];
        const float slack = penalty > 0.0f
            ? positive_violation / (1.0f + penalty) : 0.0f;
        data.slack_used_hs[k][h] = slack;
        const float correction = positive_violation - slack;
        if (correction > 0.0f) {
          level_ZX_new[k].head(3) -= correction * a_position;
          level_ZX_new[k].segment(6, 3) -=
              correction * a_velocity;
        }
      }
      level_YX[k] -= level_ZX_new[k];
    }
  }

  for (int k = 0; k < NHORIZON; ++k) {
    if (!levelBankErrorCoordinatesActive(active_model)) {
      for (int state = 0; state < NSTATES; ++state) {
        Xhrz[k](state) = level_Xhrz[k](state);
        ZX_new[k](state) = level_ZX_new[k](state);
      }
    } else {
      float reference[NSTATES];
      float predicted_error[NSTATES];
      float projected_error[NSTATES];
      float predicted[NSTATES];
      float projected[NSTATES];
      for (int state = 0; state < NSTATES; ++state) {
        reference[state] = Xref[k](state);
        predicted_error[state] = level_Xhrz[k](state);
        projected_error[state] = level_ZX_new[k](state);
      }
      tinyMpcFrenetErrorDecode(predicted_error, reference, predicted);
      tinyMpcFrenetErrorDecode(projected_error, reference, projected);
      for (int state = 0; state < NSTATES; ++state) {
        Xhrz[k](state) = predicted[state];
        ZX_new[k](state) = projected[state];
      }
    }
    if (k < NHORIZON - 1) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float origin_delta_n =
            levelModelInputOriginN(active_model, motor)
                - TINYMPC_LEVEL_HOVER_THRUST_N;
        Uhrz[k](motor) = level_Uhrz[k](motor) + origin_delta_n;
        ZU_new[k](motor) = level_ZU_new[k](motor) + origin_delta_n;
      }
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

#if TINYMPC_RATE_CASCADE
static bool outerModelErrorCoordinatesActive(
    const TinyMpcOuterLoopModelData& model) {
  return flip_reference_active || power_loop_reference_active ||
      model.model_id != TINYMPC_BANK_MODEL_LEVEL;
}

static bool outerManeuverPrimitiveActive(void) {
  return flip_state.mode == TINYMPC_FLIP_ACTIVE
      || power_loop_state.mode == TINYMPC_POWER_LOOP_ACTIVE;
}

static float outerInputLower(
    const TinyMpcOuterLoopModelData& model, int input) {
  if (outerManeuverPrimitiveActive()
      && (input == 1 || input == 2)) {
    return -outer_maneuver_roll_pitch_rate_limit_rad_s;
  }
  return model.input_lower[input];
}

static float outerInputUpper(
    const TinyMpcOuterLoopModelData& model, int input) {
  if (outerManeuverPrimitiveActive()
      && (input == 1 || input == 2)) {
    return outer_maneuver_roll_pitch_rate_limit_rad_s;
  }
  return model.input_upper[input];
}

static float outerInputSlewPerSolve(int input) {
  if (outerManeuverPrimitiveActive()
      && (input == 1 || input == 2)) {
    return outer_maneuver_rate_slew_per_solve_rad_s;
  }
  return outer_input_slew_per_solve[input];
}

static float outerStateRollRad(const VectorNf& state) {
  const float rx = state(3);
  const float ry = state(4);
  const float rz = state(5);
  const float inverse_norm = 1.0f / sqrtf(
      1.0f + rx * rx + ry * ry + rz * rz);
  const float qw = inverse_norm;
  const float qx = rx * inverse_norm;
  const float qy = ry * inverse_norm;
  const float qz = rz * inverse_norm;
  return atan2f(
      2.0f * (qw * qx + qy * qz),
      1.0f - 2.0f * (qx * qx + qy * qy));
}

static float outerStateYawRad(const VectorNf& state) {
  const float rx = state(3);
  const float ry = state(4);
  const float rz = state(5);
  const float inverse_norm = 1.0f / sqrtf(
      1.0f + rx * rx + ry * ry + rz * rz);
  const float qw = inverse_norm;
  const float qx = rx * inverse_norm;
  const float qy = ry * inverse_norm;
  const float qz = rz * inverse_norm;
  return atan2f(
      2.0f * (qw * qz + qx * qy),
      1.0f - 2.0f * (qy * qy + qz * qz));
}

static float outerReferenceState(
    const TinyMpcOuterLoopModelData& model, int knot, int state) {
  return outerModelErrorCoordinatesActive(model)
      ? 0.0f : Xref[knot](state);
}

static float outerProgressLinearCost(
    const TinyMpcOuterLoopModelData& model, int knot, int state) {
  if (state < 6 || state > 8) {
    return 0.0f;
  }
  if (!outerModelErrorCoordinatesActive(model) || state == 8) {
    return progress_state_linear_cost[knot](state);
  }
  const float yaw = outerStateYawRad(Xref[knot]);
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);
  const float qx = progress_state_linear_cost[knot](6);
  const float qy = progress_state_linear_cost[knot](7);
  return state == 6
      ? cosine * qx + sine * qy
      : -sine * qx + cosine * qy;
}

static void resetOuterLoopDuals(void) {
  for (int knot = 0; knot < NHORIZON; ++knot) {
    outer_Xhrz[knot].setZero();
    outer_p[knot].setZero();
    outer_ZX_new[knot].setZero();
    outer_YX[knot].setZero();
    if (knot < NHORIZON - 1) {
      outer_Uref[knot].setZero();
      outer_Uhrz[knot].setZero();
      outer_d[knot].setZero();
      outer_ZU_new[knot].setZero();
      outer_YU[knot].setZero();
    }
  }
}

static void outerInitialState(
    const TinyMpcOuterLoopModelData& model, VectorNf& state) {
  if (!outerModelErrorCoordinatesActive(model)) {
    state = x0;
    return;
  }
  float actual[NSTATES];
  float reference[NSTATES];
  float error[NSTATES];
  for (int index = 0; index < NSTATES; ++index) {
    actual[index] = x0(index);
    reference[index] = Xref[0](index);
  }
  tinyMpcFrenetErrorEncode(actual, reference, error);
  for (int index = 0; index < NSTATES; ++index) {
    state(index) = error[index];
  }
}

static int outerBundleIndexForSelection(TinyMpcBankModelId selection) {
  const int selected = (int)selection;
  if (selected < TINYMPC_OUTER_LOOP_MODEL_COUNT) {
    return selected;
  }
  /* The identified bank currently ends at the verified 2.5m/s tier. Keep the
   * same turn side if the reference requests the not-yet-identified 3m/s
   * chart; collective/rate bounds still prevent extrapolation. */
  return tinyMpcBankModelSide(selection) < 0
      ? TINYMPC_BANK_MODEL_LEFT_VERY_HIGH
      : TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH;
}

static void updateOuterLoopModelSelection(void) {
  static bool maneuver_override_active = false;
  const bool aggressive_maneuver_active =
      flip_reference_active || power_loop_reference_active;
  if (aggressive_maneuver_active) {
    if (!maneuver_override_active) {
      tinyMpcBankSelectorReset(
          &outer_bank_selector, &outer_bank_selector_config);
      outer_bank_selection = {
          TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
      active_outer_loop_model =
          &tinympc_outer_loop_models[TINYMPC_BANK_MODEL_LEVEL];
      resetOuterLoopDuals();
      maneuver_override_active = true;
      DEBUG_PRINT(
          "MANEUVER outer model=identified_level reference_coordinates=relative kind=%s\n",
          power_loop_reference_active ? "power_loop" : "flip");
    }
    return;
  }
  if (maneuver_override_active) {
    tinyMpcBankSelectorReset(
        &outer_bank_selector, &outer_bank_selector_config);
    outer_bank_selection = {
        TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
    active_outer_loop_model =
        &tinympc_outer_loop_models[TINYMPC_BANK_MODEL_LEVEL];
    resetOuterLoopDuals();
    maneuver_override_active = false;
  }

  /* Avoidance remains a reference residual. Model selection must follow the
   * actual reference/measured bank, speed, and chart error below. */
  const float reference_bank_rad = outerStateRollRad(Xref[0]);
  const float measured_bank_rad = outerStateRollRad(x0);
  const bool terminal_transition_active =
      tinyMpcProgressPathCompletedLaps(
          &progress_path, progress_path.progress)
          >= (uint16_t)TINYMPC_PROGRESS_LAPS;
  const float selector_measured_bank_rad = terminal_transition_active
      ? tinyMpcBankMeasuredDemandForReference(
          reference_bank_rad, measured_bank_rad,
          outer_bank_selector_config.exit_bank_rad)
      : measured_bank_rad;
  const Eigen::Vector3f reference_velocity = Xref[0].segment<3>(6);
  const float reference_speed_mps = reference_velocity.norm();
  const float measured_tangent_speed_mps = reference_speed_mps > 1.0e-5f
      ? x0.segment<3>(6).dot(reference_velocity / reference_speed_mps)
      : 0.0f;
  const TinyMpcBankModelId current_model =
      outer_bank_selector.active_model;
  const TinyMpcBankModelId requested_model = tinyMpcRequestedBankModel(
      current_model, reference_bank_rad, &outer_bank_selector_config);
  const uint8_t current_tier = tinyMpcBankModelTier(current_model);
  const uint8_t requested_tier = tinyMpcBankModelTier(requested_model);
  const bool side_change = requested_tier > 0u && current_tier > 0u
      && tinyMpcBankModelSide(requested_model)
          != tinyMpcBankModelSide(current_model);
  const bool more_aggressive_transition = requested_model != current_model
      && (requested_tier > current_tier || side_change);
  bool transition_supported = true;
  bool motion_supported = true;
  bool nominal_speed_supported = true;
  bool chart_error_supported = true;
  float chart_error[NSTATES] = {0.0f};
  float minimum_nominal_fraction = 0.0f;
  if (more_aggressive_transition) {
    float actual[NSTATES];
    float reference[NSTATES];
    for (int state = 0; state < NSTATES; ++state) {
      actual[state] = x0(state);
      reference[state] = Xref[0](state);
    }
    tinyMpcFrenetErrorEncode(actual, reference, chart_error);
    const TinyMpcOuterLoopModelData& requested_bundle =
        tinympc_outer_loop_models[
            outerBundleIndexForSelection(requested_model)];
    const bool low_demand_entry =
        current_tier == 0u && requested_tier == 1u;
    const bool high_tier_promotion =
        current_tier > 0u && requested_tier > current_tier;
    minimum_nominal_fraction = tinyMpcBankCandidateSpeedLocalityFraction(
        low_demand_entry, high_tier_promotion, requested_tier);
    if (current_tier == 0u && requested_tier > 1u) {
      /* The level chart cannot close the remaining coordinated-turn yaw-rate
       * error by itself. Enter an already-local higher tier at 60% nominal
       * speed, then let the bumpless input constraints complete the transfer. */
      minimum_nominal_fraction = 0.60f;
    }
    motion_supported = tinyMpcBankEntrySupportedByMotion(
        reference_bank_rad, measured_bank_rad,
        reference_speed_mps, measured_tangent_speed_mps);
    nominal_speed_supported = tinyMpcBankEntrySupportedByNominalSpeed(
        requested_bundle.nominal_speed_mps,
        reference_speed_mps, measured_tangent_speed_mps,
        minimum_nominal_fraction);
    chart_error_supported = tinyMpcBankEntrySupportedByChartError(
        chart_error, 0.15f, 0.40f, 0.15f, 1.00f);
    transition_supported = motion_supported && nominal_speed_supported
        && chart_error_supported;
  }
  const float held_bank_rad = current_tier == 0u ? 0.0f
      : (float)tinyMpcBankModelSide(current_model)
          * tinyMpcBankNominalForTier(
              current_tier, &outer_bank_selector_config);
  const float selector_reference_bank_rad =
      more_aggressive_transition && !transition_supported
      ? held_bank_rad : reference_bank_rad;
  outer_bank_selection = tinyMpcBankSelectorUpdate(
      &outer_bank_selector, &outer_bank_selector_config,
      selector_reference_bank_rad, selector_measured_bank_rad);
  const int bundle_index = outerBundleIndexForSelection(
      outer_bank_selection.active_model);
  const TinyMpcOuterLoopModelData *selected_bundle =
      &tinympc_outer_loop_models[bundle_index];
  if (outer_bank_selection.switched) {
    active_outer_loop_model = selected_bundle;
    if (outer_bank_selection.reset_optimizer) {
      resetOuterLoopDuals();
    }
#if defined(CONFIG_PLATFORM_SITL)
    outer_model_transition_diag_steps = 30u;
#endif
    DEBUG_PRINT(
        "OUTER model switch requested=%d bundle=%d demand=%.4frad reference=%.4frad measured=%.4frad nominal=%.4frad speed=%.2fm/s count=%lu\n",
        (int)outer_bank_selection.active_model, bundle_index,
        (double)outer_bank_selection.signed_bank_demand_rad,
        (double)reference_bank_rad, (double)measured_bank_rad,
        (double)selected_bundle->nominal_roll_rad,
        (double)selected_bundle->nominal_speed_mps,
        (unsigned long)outer_bank_selector.switch_count);
  } else {
    active_outer_loop_model = selected_bundle;
  }
  if (more_aggressive_transition && !transition_supported) {
    static uint16_t locality_wait_log_divider = 0u;
    if ((locality_wait_log_divider++ % 25u) == 0u) {
      DEBUG_PRINT(
          "OUTER entry wait requested=%d current=%d reference_bank=%.4f measured_bank=%.4f reference_speed=%.3f measured_tangent=%.3f fraction=%.2f error=(lat_p=%.3f lat_v=%.3f yaw=%.3f yaw_rate=%.3f) support=(motion=%d speed=%d chart=%d)\n",
          (int)requested_model, (int)current_model,
          (double)reference_bank_rad, (double)measured_bank_rad,
          (double)reference_speed_mps, (double)measured_tangent_speed_mps,
          (double)minimum_nominal_fraction,
          (double)chart_error[1], (double)chart_error[7],
          (double)chart_error[5], (double)chart_error[11],
          motion_supported, nominal_speed_supported, chart_error_supported);
    }
  }
}

static void prepareOuterInputReference(
    const TinyMpcOuterLoopModelData& model) {
  for (int knot = 0; knot < NHORIZON - 1; ++knot) {
    float collective_thrust_n = 0.0f;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      collective_thrust_n += T_MAX(
          tinympc_generated_physical_hover_thrust[motor]
              + Uref[knot](motor),
          0.0f);
    }
    const float physical_input[NINPUTS] = {
      collective_thrust_n,
      Xref[knot](9), Xref[knot](10), Xref[knot](11),
    };
    for (int input = 0; input < NINPUTS; ++input) {
      outer_Uref[knot](input) = T_MIN(T_MAX(
          physical_input[input] - model.input_origin[input],
          outerInputLower(model, input)), outerInputUpper(model, input));
    }
  }
}

static void solveOuterLoopRateModel(void) {
  const TinyMpcOuterLoopModelData& active_model =
      *active_outer_loop_model;
  prepareOuterInputReference(active_model);
  info.pri_res = 0.0f;
  info.dua_res = 0.0f;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (int row = 0; row < NSTATES; ++row) {
      float terminal = 0.0f;
      for (int column = 0; column < NSTATES; ++column) {
        terminal -= active_model.P[row * NSTATES + column]
            * outerReferenceState(
                active_model, NHORIZON - 1, column);
      }
      terminal += outerProgressLinearCost(
          active_model, NHORIZON - 1, row);
      outer_p[NHORIZON - 1](row) = terminal;
    }

    for (int knot = NHORIZON - 2; knot >= 0; --knot) {
      float r_tilde[NINPUTS];
      float rhs[NINPUTS];
      for (int input = 0; input < NINPUTS; ++input) {
        r_tilde[input] = -TINYMPC_OUTER_LOOP_RHO
            * (outer_ZU_new[knot](input) - outer_YU[knot](input));
        for (int column = 0; column < NINPUTS; ++column) {
          r_tilde[input] -= active_model.R[input * NINPUTS + column]
              * outer_Uref[knot](column);
        }
        float value = r_tilde[input] + active_model.BPf[input];
        for (int state = 0; state < NSTATES; ++state) {
          value += active_model.B[state * NINPUTS + input]
              * outer_p[knot + 1](state);
        }
        rhs[input] = value;
      }
      for (int input = 0; input < NINPUTS; ++input) {
        float value = 0.0f;
        for (int column = 0; column < NINPUTS; ++column) {
          value += active_model.Quu_inv[input * NINPUTS + column]
              * rhs[column];
        }
        outer_d[knot](input) = value;
      }
      for (int state = 0; state < NSTATES; ++state) {
        float value = -active_model.Q_diagonal[state]
            * outerReferenceState(active_model, knot, state)
            + outerProgressLinearCost(active_model, knot, state)
            - TINYMPC_OUTER_LOOP_RHO
                * (outer_ZX_new[knot](state) - outer_YX[knot](state))
            + active_model.APf[state];
        for (int column = 0; column < NSTATES; ++column) {
          value += active_model.AmBKt[state * NSTATES + column]
              * outer_p[knot + 1](column);
        }
        for (int input = 0; input < NINPUTS; ++input) {
          value -= active_model.K[input * NSTATES + state]
              * r_tilde[input];
          /* The generated cache stores the upstream, unaugmented
           * K'R-(A-BK)'PB coefficient. This ADMM recursion solves with
           * R+rho*I, so add rho*K' here. Omitting it destroys nonzero-input
           * reference fixed points even though every zero-correction bundle
           * still appears valid. */
          const float augmented_coeff_d2p =
              active_model.coeff_d2p[state * NINPUTS + input]
              + TINYMPC_OUTER_LOOP_RHO
                  * active_model.K[input * NSTATES + state];
          value += augmented_coeff_d2p * outer_d[knot](input);
        }
        outer_p[knot](state) = value;
      }
    }

    outerInitialState(active_model, outer_Xhrz[0]);
    for (int knot = 0; knot < NHORIZON - 1; ++knot) {
      for (int input = 0; input < NINPUTS; ++input) {
        float value = -outer_d[knot](input);
        for (int state = 0; state < NSTATES; ++state) {
          value -= active_model.K[input * NSTATES + state]
              * outer_Xhrz[knot](state);
        }
        outer_Uhrz[knot](input) = value;
      }
      for (int state = 0; state < NSTATES; ++state) {
        float value = active_model.affine[state];
        for (int column = 0; column < NSTATES; ++column) {
          value += active_model.A[state * NSTATES + column]
              * outer_Xhrz[knot](column);
        }
        for (int input = 0; input < NINPUTS; ++input) {
          value += active_model.B[state * NINPUTS + input]
              * outer_Uhrz[knot](input);
        }
        outer_Xhrz[knot + 1](state) = value;
      }
    }

    for (int knot = 0; knot < NHORIZON - 1; ++knot) {
      for (int input = 0; input < NINPUTS; ++input) {
        const float maximum_trim = outerManeuverPrimitiveActive()
            ? outer_maneuver_trim_limit[input]
            : outer_tracking_trim_limit[input];
        TinyMpcRateCommandBounds input_bounds =
            tinyMpcRateReferenceTrimBounds(
                outerInputLower(active_model, input),
                outerInputUpper(active_model, input),
                outer_Uref[knot](input), maximum_trim);
        if (knot == 0) {
          const float previous_physical_input = mpc_has_run
              ? (input == 0
                    ? active_rate_command.collective_thrust_n
                    : active_rate_command.body_rate_rad_s[input - 1])
              : (input == 0 ? hoverCollectiveThrustN() : 0.0f);
          const float previous_correction = previous_physical_input
              - active_model.input_origin[input];
          input_bounds = tinyMpcRateIntersectSlewBounds(
              input_bounds, previous_correction,
              outerInputSlewPerSolve(input));
        }
        outer_YU[knot](input) += outer_Uhrz[knot](input);
        outer_ZU_new[knot](input) = T_MIN(T_MAX(
            outer_YU[knot](input), input_bounds.lower), input_bounds.upper);
        outer_YU[knot](input) -= outer_ZU_new[knot](input);
      }
    }
    for (int knot = 0; knot < NHORIZON; ++knot) {
      outer_YX[knot] += outer_Xhrz[knot];
      outer_ZX_new[knot] = outer_YX[knot];
      for (int halfspace = 0; halfspace < MAX_HS; ++halfspace) {
        if (!data.en_hs[knot][halfspace]) {
          continue;
        }
        Eigen::Vector3f a_position = data.a_pos_hs[knot][halfspace];
        Eigen::Vector3f a_velocity = data.a_vel_hs[knot][halfspace];
        float boundary = data.b_hs[knot][halfspace];
        if (outerModelErrorCoordinatesActive(active_model)) {
          const float yaw = outerStateYawRad(Xref[knot]);
          const float cosine = cosf(yaw);
          const float sine = sinf(yaw);
          a_position = Eigen::Vector3f(
              cosine * data.a_pos_hs[knot][halfspace].x()
                  + sine * data.a_pos_hs[knot][halfspace].y(),
              -sine * data.a_pos_hs[knot][halfspace].x()
                  + cosine * data.a_pos_hs[knot][halfspace].y(),
              data.a_pos_hs[knot][halfspace].z());
          a_velocity = Eigen::Vector3f(
              cosine * data.a_vel_hs[knot][halfspace].x()
                  + sine * data.a_vel_hs[knot][halfspace].y(),
              -sine * data.a_vel_hs[knot][halfspace].x()
                  + cosine * data.a_vel_hs[knot][halfspace].y(),
              data.a_vel_hs[knot][halfspace].z());
          boundary -= data.a_pos_hs[knot][halfspace].dot(
              Xref[knot].head(3))
              + data.a_vel_hs[knot][halfspace].dot(
                  Xref[knot].segment(6, 3));
        }
        const float violation =
            a_position.dot(outer_ZX_new[knot].head(3))
            + a_velocity.dot(outer_ZX_new[knot].segment(6, 3))
            - boundary;
        const float positive_violation = T_MAX(violation, 0.0f);
        const float penalty = data.slack_penalty_hs[knot][halfspace];
        const float slack = penalty > 0.0f
            ? positive_violation / (1.0f + penalty) : 0.0f;
        data.slack_used_hs[knot][halfspace] = slack;
        const float correction = positive_violation - slack;
        if (correction > 0.0f) {
          outer_ZX_new[knot].head(3) -= correction * a_position;
          outer_ZX_new[knot].segment(6, 3) -= correction * a_velocity;
        }
      }
      outer_YX[knot] -= outer_ZX_new[knot];
    }
  }

  for (int knot = 0; knot < NHORIZON; ++knot) {
    if (!outerModelErrorCoordinatesActive(active_model)) {
      Xhrz[knot] = outer_Xhrz[knot];
      ZX_new[knot] = outer_ZX_new[knot];
    } else {
      float reference[NSTATES];
      float predicted_error[NSTATES];
      float projected_error[NSTATES];
      float predicted[NSTATES];
      float projected[NSTATES];
      for (int state = 0; state < NSTATES; ++state) {
        reference[state] = Xref[knot](state);
        predicted_error[state] = outer_Xhrz[knot](state);
        projected_error[state] = outer_ZX_new[knot](state);
      }
      tinyMpcFrenetErrorDecode(predicted_error, reference, predicted);
      tinyMpcFrenetErrorDecode(projected_error, reference, projected);
      for (int state = 0; state < NSTATES; ++state) {
        Xhrz[knot](state) = predicted[state];
        ZX_new[knot](state) = projected[state];
      }
    }
    if (knot < NHORIZON - 1) {
      Uhrz[knot] = outer_Uhrz[knot];
      ZU_new[knot] = outer_ZU_new[knot];
      for (int input = 0; input < NINPUTS; ++input) {
        info.pri_res = T_MAX(info.pri_res, fabsf(
            outer_Uhrz[knot](input) - outer_ZU_new[knot](input)));
      }
    }
    for (int state = 0; state < NSTATES; ++state) {
      info.pri_res = T_MAX(info.pri_res, fabsf(
          outer_Xhrz[knot](state) - outer_ZX_new[knot](state)));
    }
  }
  info.iter = 5;
#if defined(CONFIG_PLATFORM_SITL)
  if (outer_model_transition_diag_steps > 0u) {
    const float reference_roll_rad = outerStateRollRad(Xref[0]);
    const float measured_roll_rad = outerStateRollRad(x0);
    DEBUG_PRINT(
        "OUTER handoff model=%d remaining=%u roll=(ref=%.4f actual=%.4f error=%.4f) error=(p=%.3f,%.3f,%.3f r=%.3f,%.3f,%.3f) uref_phys=(%.4f,%.3f,%.3f,%.3f) primal_phys=(%.4f,%.3f,%.3f,%.3f) projected_phys=(%.4f,%.3f,%.3f,%.3f) residual=%.4f\n",
        active_model.model_id,
        (unsigned int)outer_model_transition_diag_steps,
        (double)reference_roll_rad, (double)measured_roll_rad,
        (double)(measured_roll_rad - reference_roll_rad),
        (double)outer_Xhrz[0](0), (double)outer_Xhrz[0](1),
        (double)outer_Xhrz[0](2), (double)outer_Xhrz[0](9),
        (double)outer_Xhrz[0](10), (double)outer_Xhrz[0](11),
        (double)(active_model.input_origin[0] + outer_Uref[0](0)),
        (double)(active_model.input_origin[1] + outer_Uref[0](1)),
        (double)(active_model.input_origin[2] + outer_Uref[0](2)),
        (double)(active_model.input_origin[3] + outer_Uref[0](3)),
        (double)(active_model.input_origin[0] + outer_Uhrz[0](0)),
        (double)(active_model.input_origin[1] + outer_Uhrz[0](1)),
        (double)(active_model.input_origin[2] + outer_Uhrz[0](2)),
        (double)(active_model.input_origin[3] + outer_Uhrz[0](3)),
        (double)(active_model.input_origin[0] + outer_ZU_new[0](0)),
        (double)(active_model.input_origin[1] + outer_ZU_new[0](1)),
        (double)(active_model.input_origin[2] + outer_ZU_new[0](2)),
        (double)(active_model.input_origin[3] + outer_ZU_new[0](3)),
        (double)info.pri_res);
    --outer_model_transition_diag_steps;
  }
#endif
}
#endif



static void tinympcControllerTask(void *parameters) {
  (void)parameters;
  uint32_t log_counter = 0;

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
#if defined(CONFIG_PLATFORM_SITL)
    const bool diagnostic_enabled = sitlDiagEnabled();
    uint64_t diagnostic_start_us = 0u;
    uint32_t diagnostic_start_tick = 0u;
    if (diagnostic_enabled) {
      diagnostic_start_us = usecTimestamp();
      diagnostic_start_tick = xTaskGetTickCount();
    }
#endif

    setpoint_t setpoint_task;
    sensorData_t sensors_task;
    state_t state_task;
    uint32_t solve_tick;
    bool reset_requested;
#if defined(CONFIG_PLATFORM_SITL)
    uint64_t diagnostic_release_us;
    uint32_t diagnostic_release_rtos_tick;
    uint32_t diagnostic_release_sequence;
    uint32_t diagnostic_release_due_count;
    uint32_t diagnostic_release_mutex_miss_count;
    uint32_t diagnostic_semaphore_coalesced_count;
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
    float level_motor_rotor_state_estimate_task[NINPUTS];
#endif
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &planner_setpoint, sizeof(setpoint_task));
    memcpy(&sensors_task, &planner_sensors, sizeof(sensors_task));
    memcpy(&state_task, &planner_state, sizeof(state_task));
    solve_tick = planner_tick;
    reset_requested = planner_reset_requested;
#if defined(CONFIG_PLATFORM_SITL)
    diagnostic_release_us = 0u;
    diagnostic_release_rtos_tick = 0u;
    diagnostic_release_sequence = 0u;
    diagnostic_release_due_count = 0u;
    diagnostic_release_mutex_miss_count = 0u;
    diagnostic_semaphore_coalesced_count = 0u;
    if (diagnostic_enabled) {
      diagnostic_release_us = __atomic_load_n(
          &planner_diag_release_us, __ATOMIC_RELAXED);
      diagnostic_release_rtos_tick = __atomic_load_n(
          &planner_diag_release_rtos_tick, __ATOMIC_RELAXED);
      diagnostic_release_sequence = __atomic_load_n(
          &planner_diag_release_sequence, __ATOMIC_RELAXED);
      diagnostic_release_due_count = __atomic_load_n(
          &planner_diag_release_due_count, __ATOMIC_RELAXED);
      diagnostic_release_mutex_miss_count = __atomic_load_n(
          &planner_diag_release_mutex_miss_count, __ATOMIC_RELAXED);
      diagnostic_semaphore_coalesced_count = __atomic_load_n(
          &planner_diag_semaphore_coalesced_count, __ATOMIC_RELAXED);
    }
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
    memcpy(level_motor_rotor_state_estimate_task,
           planner_level_motor_rotor_state_estimate,
           sizeof(level_motor_rotor_state_estimate_task));
#endif
    planner_reset_requested = false;
    xSemaphoreGive(dataMutex);

    constexpr uint32_t expected_solve_tick_gap = M2T(1000 / MPC_RATE);
    uint32_t current_solve_tick_gap = 0u;
    if (solve_tick_timing_initialized) {
      const uint32_t solve_tick_gap = solve_tick - previous_solve_tick;
      current_solve_tick_gap = solve_tick_gap;
      solve_tick_gap_min = T_MIN(solve_tick_gap_min, solve_tick_gap);
      solve_tick_gap_max = T_MAX(solve_tick_gap_max, solve_tick_gap);
      ++solve_tick_gap_samples;
      if (solve_tick_gap > expected_solve_tick_gap) {
        solve_tick_skipped_periods +=
            solve_tick_gap / expected_solve_tick_gap - 1u;
      }
      if (solve_tick_gap_samples >= (uint32_t)MPC_RATE) {
        DEBUG_PRINT(
            "MPC cadence expected_ticks=%lu min_ticks=%lu max_ticks=%lu skipped_periods=%lu samples=%lu\n",
            (unsigned long)expected_solve_tick_gap,
            (unsigned long)solve_tick_gap_min,
            (unsigned long)solve_tick_gap_max,
            (unsigned long)solve_tick_skipped_periods,
            (unsigned long)solve_tick_gap_samples);
        solve_tick_gap_min = UINT32_MAX;
        solve_tick_gap_max = 0u;
        solve_tick_gap_samples = 0u;
        solve_tick_skipped_periods = 0u;
      }
    } else {
      solve_tick_timing_initialized = true;
    }
    previous_solve_tick = solve_tick;

    if (reset_requested) {
      step = 0;
      tinyMpcFlipReset(&flip_state);
      flip_reference_active = false;
      flip_recovery_active = false;
      flip_recovery_settle_steps = 0u;
      flip_recovery_elapsed_steps = 0u;
      tinyMpcPowerLoopReset(&power_loop_state);
      power_loop_reference_active = false;
      power_loop_recovery_active = false;
      power_loop_recovery_settle_steps = 0u;
      power_loop_recovery_elapsed_steps = 0u;
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
      pitch_through_brake_reference_active = false;
      pitch_through_brake_entry_altitude_world_m = 0.0f;
#endif
#if defined(TINYMPC_TRAJECTORY_CANONICAL_FIGURE8)
      figure8_midcourse_dual_reset_complete = false;
#endif
#if TINYMPC_RATE_CASCADE
      resetOuterLoopDuals();
      tinyMpcBankSelectorReset(
          &outer_bank_selector, &outer_bank_selector_config);
      outer_bank_selection = {
          TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
      active_outer_loop_model =
          &tinympc_outer_loop_models[TINYMPC_BANK_MODEL_LEVEL];
      outer_warm_start_yaw_rad = 0.0f;
      outer_warm_start_yaw_initialized = false;
      outer_frame_reseed_requested = false;
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
      resetLevelActuatorDuals();
      tinyMpcBankSelectorReset(
          &level_bank_selector, &level_bank_selector_config);
      tinyMpcBrakingSelectorReset(
          &level_braking_selector, &level_braking_selector_config);
      level_bank_selection = {
          TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
      level_braking_selection = {
          TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
      active_level_actuator_model =
          &tinympc_banked_models[TINYMPC_BANK_MODEL_LEVEL];
#endif
      const struct quat handoff_attitude = qnormalize(mkquat(
          state_task.attitudeQuaternion.x, state_task.attitudeQuaternion.y,
          state_task.attitudeQuaternion.z, state_task.attitudeQuaternion.w));
      const float handoff_yaw = quat2rpy(handoff_attitude).z;
      trajectory_cos_yaw = cosf(handoff_yaw);
      trajectory_sin_yaw = sinf(handoff_yaw);
      trajectory_yaw_rotation = rpy2quat(mkvec(0.0f, 0.0f, handoff_yaw));
      reference_yaw_phase_rad = handoff_yaw;
#if TINYMPC_YAW_SPIN_TEST_ENABLE
      yaw_spin_test_anchor_world = Eigen::Vector3f(
          state_task.position.x, state_task.position.y, state_task.position.z);
      yaw_spin_test_initial_yaw_rad = handoff_yaw;
      yaw_spin_test_step = 0u;
      yaw_spin_test_reported_revolutions = 0u;
      yaw_spin_test_last_phase = UINT8_MAX;
      yaw_spin_test_complete_reported = false;
      DEBUG_PRINT(
          "YAW_SPIN_TEST start anchor=(%.3f,%.3f,%.3f) rate=%.3frad/s revolutions=%.2f straight=(%.3fm/s,%.3fs) duration=%.3fs\n",
          (double)yaw_spin_test_anchor_world.x(),
          (double)yaw_spin_test_anchor_world.y(),
          (double)yaw_spin_test_anchor_world.z(),
          (double)(float)TINYMPC_YAW_SPIN_TEST_RATE_RAD_S,
          (double)(float)TINYMPC_YAW_SPIN_TEST_REVOLUTIONS,
          (double)(float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS,
          (double)(float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S,
          (double)(6.2831853071795864769f
              * (float)TINYMPC_YAW_SPIN_TEST_REVOLUTIONS
              / fabsf((float)TINYMPC_YAW_SPIN_TEST_RATE_RAD_S)
              + (((float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_SPEED_MPS > 0.0f)
                  ? (float)TINYMPC_YAW_SPIN_TEST_STRAIGHT_DURATION_S : 0.0f)));
#endif
      progress_heading_alignment_active = false;
      progress_heading_alignment_complete = false;
      trajectory_handoff_hold_steps = (uint16_t)(MPC_RATE * 2 / 5);
      vertical_active_sensing_start_tick = solve_tick;
#if TINYMPC_REACTIVE_REFERENCE_FREE
      tinyRacerReactiveReset(&reactive_reference_state);
      tinyRacerSquareOpeningReset(&reactive_square_opening_state);
      reactive_square_collision_stop_context = false;
      reactive_reference_command = {
          0.0f, 0.0f, 0.0f, TINYRACER_REACTIVE_CRUISE, 0, false};
      reactive_reference_last_sample = 0u;
      reactive_reference_speed_mps = 0.0f;
      reactive_reference_lateral_speed_mps = 0.0f;
      reactive_reference_altitude_world_m = state_task.position.z;
      reactive_reference_heading_world_rad = handoff_yaw;
      reactive_reference_turn_rate_rad_s = 0.0f;
      reactive_reference_backtrack_anchor_world = Eigen::Vector3f::Zero();
      reactive_reference_initialized = true;
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
      tinyMpcReactivePowerLoopReset(&reactive_power_loop_state);
      reactive_power_loop_command = {};
      reactive_power_loop_step = 0u;
      reactive_power_loop_anchor_world = Eigen::Vector3f::Zero();
      reactive_power_loop_recovery_anchor_world = Eigen::Vector3f::Zero();
      reactive_power_loop_yaw_world_rad = handoff_yaw;
#endif
      DEBUG_PRINT(
          "Route-free reactive TinyMPC enabled: direct motors cruise=%.3fm/s cruise_accel=%.2fm/s2 center_trigger=%.2f all_sector_trigger=%.2f release=%.2f brake=%.1fm/s2 backtrack=%.2fm@%.2fm/s yaw=%.1fdeg/s lateral=%.2fm/s yaw_accel=%.2frad/s2 settle_samples=%u min_maneuver_samples=%u clear_samples=%u rail_cue_samples=%u gate_confidence=%.2f geofence=disabled altitude=%.2fm\n",
          (double)reactive_reference_config.cruise_speed_mps,
          (double)(float)TINYMPC_REACTIVE_CRUISE_ACCELERATION_MPS2,
          (double)reactive_reference_config.trigger_probability,
          (double)reactive_reference_config.all_sector_trigger_probability,
          (double)reactive_reference_config.release_probability,
          (double)TINYRACER_EMERGENCY_BRAKE_DECELERATION_MPS2,
          (double)reactive_reference_config.backtrack_distance_m,
          (double)reactive_reference_config.backtrack_speed_mps,
          (double)reactive_reference_config.turn_rate_deg_s,
          (double)reactive_reference_config.translation_speed_mps,
          (double)(float)TINYMPC_REACTIVE_YAW_ACCELERATION_RAD_S2,
          (unsigned)reactive_reference_config.settled_samples_required,
          (unsigned)reactive_reference_config.minimum_maneuver_samples,
          (unsigned)reactive_reference_config.clear_samples_required,
          (unsigned)reactive_reference_config.rail_cue_samples_required,
          (double)TINYRACER_REACTIVE_GATE_CONFIDENCE_THRESHOLD,
          (double)reactive_reference_altitude_world_m);
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
      DEBUG_PRINT(
          "Reactive power loop enabled: maneuver=power_loop_vertical_360 radius=1.00m rise=2.00m peak_speed=3.80m/s trigger=center_risk<=%.2f for %u fresh frames entry=stopped+level altitude>=0.75m path_geofence=disabled sequence=%u knots/%0.2fs cache=phase_ltv_current_cf21b direct_motors=1 one_shot=1\n",
          (double)reactive_power_loop_config.maximum_center_risk,
          (unsigned)reactive_power_loop_config.clear_samples_required,
          (unsigned)TINYMPC_REACTIVE_LOOP_SAMPLE_COUNT,
          (double)TINYMPC_REACTIVE_LOOP_DURATION_S);
#endif
#endif
      const float first_x = trajectory_reference_data[0][0];
      const float first_y = trajectory_reference_data[0][1];
      trajectory_origin_world = Eigen::Vector3f(
          state_task.position.x, state_task.position.y, state_task.position.z) - Eigen::Vector3f(
          trajectory_cos_yaw * first_x - trajectory_sin_yaw * first_y,
          trajectory_sin_yaw * first_x + trajectory_cos_yaw * first_y,
                          trajectory_reference_data[0][2]);
      /* One millimeter is sub-knot numerical/estimator slack (5% of the
       * smallest tested 1 m/s command step). It is applied once to cumulative
       * forward displacement, never once per solve. The target may lead by at
       * most one maximum configured command step plus this same slack. */
      constexpr float progress_physical_tolerance_m = 0.001f;
      tinyMpcProgressPathInitLaps(
          &progress_path, &trajectory_reference_data[0][0],
          TRAJECTORY_REFERENCE_DIM, progress_sample_count,
          4u, 60u, progress_minimum_speed_mps, progress_maximum_speed_mps,
          0.75f, progress_physical_tolerance_m,
          progress_maximum_speed_mps * DT, 0.15f,
          progress_control_lap_count);
#if defined(TINYMPC_TRAJECTORY_IMAV22_CIRCLE)
      /* Let the virtual route clock retain useful preview through braking and
       * avoidance, but never accumulate the multi-metre catch-up error that
       * previously drove the outer MPC far above nominal route speed. */
      tinyMpcProgressPathSetMaximumTargetLead(&progress_path, 0.5f);
#endif
      tinyMpcProgressPathSetAnalyticalDerivatives(
          &progress_path, &trajectory_reference_data[0][7],
          TRAJECTORY_REFERENCE_DIM);
      buildProgressCurvatureSpeedEnvelope();
      progress_completion_reported = false;
      progress_reference_speed_mps = 0.0f;
      progress_reference_acceleration_mps2 = 0.0f;
      progress_invariant_diag_cycle = 0u;
      progress_body_rate_reconstruction_violation_count = 0u;
      progress_last_reported_measured_lap = 0u;
      resetUncappedProgressDiagnostics();
      DEBUG_PRINT(
          "Progress reference limits=measured_projection=forward_displacement+0.001m target_lead=%s yaw_phase_slew=uncapped roll=uncapped pitch_rate=0.30rad/s local_yaw=uncapped\n",
          progress_path.target_lead_bound_enabled
              ? "configured maximum" : "unbounded");
      DEBUG_PRINT(
          "UNCAPPED diagnostics thresholds speed>0.15m/s yaw_rate>1.5708rad/s tilt>0.1745rad thrust_scale>0.05 or Uref clamp\n");
      if ((float)TINYMPC_PROGRESS_SPEED_MPS > 0.0f) {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=constant %.3fm/s curvature_gain=0.75m progress_tolerance=0.001m\n",
            (unsigned int)progress_sample_count,
            (double)progress_minimum_speed_mps);
      } else {
        DEBUG_PRINT(
            "Progress path ready samples=%u speed=0.05..0.15m/s curvature_gain=0.75m progress_tolerance=0.001m\n",
            (unsigned int)progress_sample_count);
      }
      DEBUG_PRINT(
          "Progress experiment laps=%u cooldown_laps=%u segments_per_lap=%u entry_acceleration=%.3fm/s2 terminal_deceleration=%.3fm/s2 terminal_sample=%lu\n",
          (unsigned int)TINYMPC_PROGRESS_LAPS,
          (unsigned int)(progress_control_lap_count
              - (uint16_t)TINYMPC_PROGRESS_LAPS),
          (unsigned int)(progress_path.count - 1u),
          (double)(float)TINYMPC_PROGRESS_ENTRY_ACCELERATION_MPS2,
          (double)(float)TINYMPC_PROGRESS_TERMINAL_DECELERATION_MPS2,
          (unsigned long)(progress_path.virtual_count - 1u));
      DEBUG_PRINT("Trajectory origin=(%.2f,%.2f,%.2f) yaw=%.1fdeg hold=%.1fs\n",
                  (double)trajectory_origin_world.x(),
                  (double)trajectory_origin_world.y(),
                  (double)trajectory_origin_world.z(),
                  (double)(handoff_yaw * 57.2957795f),
                  (double)trajectory_handoff_hold_steps / MPC_RATE);
#if TINYMPC_VERTICAL_ACTIVE_SENSING_ENABLE
      DEBUG_PRINT(
          "Vertical active sensing waveform=square amplitude=+/-%.3fm period=%.3fs initial_phase=low clock=elapsed_flight_time residual_vertical_authority=unchanged\n",
          (double)vertical_active_sensing_config.amplitude_m,
          (double)vertical_active_sensing_config.period_s);
#endif
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

    updateInitialState(&sensors_task, &state_task);
#if TINYMPC_RATE_CASCADE
    /* The level outer chart is expressed in the solve's yaw-aligned local
     * frame. Cold-start its auxiliaries after 5 deg of accumulated chart
     * rotation. Banked bundles use reference-relative coordinates and need no
     * such reseed. */
    constexpr float maximum_outer_warm_start_yaw_change_rad =
        0.0872664626f;
    if (!outer_warm_start_yaw_initialized) {
      outer_warm_start_yaw_rad = active_local_frame.yaw_world;
      outer_warm_start_yaw_initialized = true;
    } else if (active_outer_loop_model->model_id
                   == TINYMPC_BANK_MODEL_LEVEL
        && fabsf(tinyMpcProgressUnwrapYawNear(
            active_local_frame.yaw_world, outer_warm_start_yaw_rad)
            - outer_warm_start_yaw_rad) >=
        maximum_outer_warm_start_yaw_change_rad) {
      outer_frame_reseed_requested = true;
      outer_warm_start_yaw_rad = active_local_frame.yaw_world;
    } else if (active_outer_loop_model->model_id
                   != TINYMPC_BANK_MODEL_LEVEL) {
      outer_warm_start_yaw_rad = active_local_frame.yaw_world;
    }
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
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
    } else if (active_level_actuator_model->model_id
                   == TINYMPC_BANK_MODEL_LEVEL
        && fabsf(tinyMpcProgressUnwrapYawNear(
            active_local_frame.yaw_world, level_warm_start_yaw_rad)
            - level_warm_start_yaw_rad) >=
        maximum_level_warm_start_yaw_change_rad) {
      level_frame_reseed_requested = true;
      level_warm_start_yaw_rad = active_local_frame.yaw_world;
    } else if (active_level_actuator_model->model_id
                   != TINYMPC_BANK_MODEL_LEVEL) {
      /* Banked auxiliaries are reference-relative and invariant to world yaw. */
      level_warm_start_yaw_rad = active_local_frame.yaw_world;
    }
#endif
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
    for (int motor = 0; motor < NINPUTS; ++motor) {
      level_motor_rotor_state_snapshot(motor) =
          level_motor_rotor_state_estimate_task[motor];
    }
#endif
    // Intentionally preserve TinyMPC's state auxiliaries and duals.
    updateHorizonReference(
        &setpoint_task,
        race_intent.mode == TINYRACER_RACE_TRACK);
    /* Apply the exogenous sensing motion after constructing the nominal
     * horizon and before updateRaceIntent() consumes any learned V4 residual.
     * The policy's vertical residual channel is therefore unchanged. */
    applyVerticalActiveSensingReference(solve_tick);
#if defined(CONFIG_PLATFORM_SITL) && TINYMPC_VISION_RL_RESIDUAL_ENABLE
    publishEspNetV7ActorState(state_task, solve_tick);
#endif
    {
      static uint8_t previous_aiding_inhibit = EstimatorAidingInhibitNone;
      const struct quat aiding_attitude = qnormalize(attitude);
      const float aiding_beam_vertical = 1.0f - 2.0f * (
          aiding_attitude.x * aiding_attitude.x
          + aiding_attitude.y * aiding_attitude.y);
      const float aiding_body_rate_rad_s = x0.segment<3>(9).norm();
      const bool aggressive_reference_active =
          flip_reference_active || power_loop_reference_active
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
          || pitch_through_brake_reference_active
#endif
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
          || reactive_power_loop_command.owns_reference
#endif
          ;
      /* Continue using the deck through ordinary bank. During a maneuver,
       * suppress it only while its body-down view is geometrically poor or
       * exposure-time attitude skew is large. This bounds the inertial-only
       * interval and lets valid measurements correct drift before release. */
      const bool flowdeck_geometry_invalid =
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
          pitch_through_brake_reference_active ||
#endif
          (aggressive_reference_active
              && (aiding_beam_vertical <= 0.50f
                  || aiding_body_rate_rad_s >= 5.0f));
      const uint8_t aiding_inhibit =
          flowdeck_geometry_invalid
          ? (EstimatorAidingInhibitFlow
              | EstimatorAidingInhibitDownRange)
          : EstimatorAidingInhibitNone;
      estimatorSetAidingInhibit(aiding_inhibit);
      if (aiding_inhibit != previous_aiding_inhibit) {
        DEBUG_PRINT(
            "ESTIMATOR aiding flow=%s down_range=%s beam_vertical=%.3f body_rate=%.3frad/s propagation=imu covariance=process_noise\n",
            (aiding_inhibit & EstimatorAidingInhibitFlow)
                ? "inhibited" : "enabled",
            (aiding_inhibit & EstimatorAidingInhibitDownRange)
                ? "inhibited" : "enabled",
            (double)aiding_beam_vertical,
            (double)aiding_body_rate_rad_s);
        previous_aiding_inhibit = aiding_inhibit;
      }
    }
#if !TINYMPC_REACTIVE_REFERENCE_FREE
    updateRaceIntent(&state_task);
    applyRaceIntent();
#endif
#if TINYMPC_RATE_CASCADE
    updateOuterLoopModelSelection();
    if (outer_frame_reseed_requested) {
      if (!outer_bank_selection.switched) {
        resetOuterLoopDuals();
      }
      outer_frame_reseed_requested = false;
    }
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
    updateLevelActuatorModelSelection();
    if (level_frame_reseed_requested) {
      if (
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
          reactivePowerLoopActive()
#else
          false
#endif
          ) {
        resetLevelActuatorDuals();
      } else if (!level_model_switched_this_solve) {
        seedLevelActuatorOptimizerFromReference();
      }
      level_frame_reseed_requested = false;
    }
#endif

    const uint64_t solve_start_us = usecTimestamp();
#if TINYMPC_RATE_CASCADE
    solveOuterLoopRateModel();
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
    if (reactivePowerLoopActive()) {
      solveReactivePowerLoopStoredLtv();
    } else {
      solveLevelActuatorLti();
    }
#else
    solveLevelActuatorLti();
#endif
#else
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
#endif
    const uint64_t solve_finish_us = usecTimestamp();
    const uint32_t solve_us = (uint32_t)(solve_finish_us - solve_start_us);
#if defined(CONFIG_PLATFORM_SITL)
    float diagnostic_first_action_raw[NINPUTS] = {0.0f};
    uint32_t diagnostic_clamp_mask = 0u;
#endif
#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
    /* Convert the full input horizon before taking the publication mutex. The
     * 500 Hz callback then sees only one bounded memcpy critical section, not
     * 76 thrust-to-command conversions. */
    float prepared_motor_plan[TINYMPC_DIRECT_PLAN_INPUT_KNOTS][NINPUTS];
    for (uint32_t knot = 0u; knot < TINYMPC_DIRECT_PLAN_INPUT_KNOTS;
         ++knot) {
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float motor_thrust =
            TINYMPC_LEVEL_HOVER_THRUST_N + ZU_new[knot](motor);
        const float command =
            tinympc_generated_thrust_to_normalized_command(motor_thrust);
        if (knot == 0u) {
#if defined(CONFIG_PLATFORM_SITL)
          if (diagnostic_enabled) {
            diagnostic_first_action_raw[motor] = command;
            if (!std::isfinite(command)) {
              diagnostic_clamp_mask |= 1u << (motor + 8);
            } else if (command <= 0.0f) {
              diagnostic_clamp_mask |= 1u << motor;
            } else if (command >= 1.0f) {
              diagnostic_clamp_mask |= 1u << (motor + 4);
            }
          }
#endif
        }
        prepared_motor_plan[knot][motor] =
            T_MIN(T_MAX(command, 0.0f), 1.0f);
      }
    }
#endif
    xSemaphoreTake(dataMutex, portMAX_DELAY);
#if TINYMPC_RATE_CASCADE
    /* The identified outer model's input is physical
     * [collective thrust N, desired body p/q/r rad/s]. Publish that optimized
     * input directly; never reinterpret its rate channels as motor thrusts. */
    const TinyMpcOuterLoopModelData& publication_model =
        *active_outer_loop_model;
    float physical_rate_input[NINPUTS];
    for (int input = 0; input < NINPUTS; ++input) {
      const float lower = publication_model.input_origin[input]
          + outerInputLower(publication_model, input);
      const float upper = publication_model.input_origin[input]
          + outerInputUpper(publication_model, input);
      float optimized_input = publication_model.input_origin[input]
          + outer_ZU_new[0](input);
      if (outerManeuverPrimitiveActive()) {
        const float feedforward_input = publication_model.input_origin[input]
            + outer_Uref[0](input);
        const float trim = T_MIN(T_MAX(
            optimized_input - feedforward_input,
            -outer_maneuver_trim_limit[input]),
            outer_maneuver_trim_limit[input]);
        optimized_input = feedforward_input + trim;
      }
      physical_rate_input[input] = T_MIN(T_MAX(
          optimized_input,
          lower), upper);
    }
    active_rate_command.collective_thrust_n = physical_rate_input[0];
    for (int axis = 0; axis < 3; ++axis) {
      active_rate_command.body_rate_rad_s[axis] =
          physical_rate_input[axis + 1];
    }
    if (planner_rate_pid_reset_pending) {
      active_rate_pid_reset_requested = true;
      planner_rate_pid_reset_pending = false;
    }
    const float equal_motor_command = T_MIN(T_MAX(
        tinympc_generated_thrust_to_normalized_command(
            active_rate_command.collective_thrust_n / (float)NINPUTS),
        0.0f), 1.0f);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      /* Retain a meaningful common-mode value for existing diagnostics only;
       * the stock legacy mixer, not this array, generates motor differentials. */
      active_motor_commands[motor] = equal_motor_command;
#if defined(CONFIG_PLATFORM_SITL)
      if (diagnostic_enabled) {
        diagnostic_first_action_raw[motor] = equal_motor_command;
      }
#endif
    }
#elif defined(TINYMPC_DIRECT_PLAN_REPLAY)
    memcpy(active_motor_plan, prepared_motor_plan,
           sizeof(active_motor_plan));
    for (int motor = 0; motor < NINPUTS; ++motor) {
      active_motor_commands[motor] = active_motor_plan[0][motor];
    }
#else
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float correction_now =
#if defined(TINYMPC_USE_ACTUATOR_LTI)
          ZU_new[0](motor);
#else
          Uhrz[0](motor);
#endif
      const float motor_thrust_now =
#if defined(TINYMPC_USE_ACTUATOR_LTI)
          TINYMPC_LEVEL_HOVER_THRUST_N + correction_now;
#else
          tinympc_generated_physical_hover_thrust[motor] + correction_now;
#endif
      float command =
          tinympc_generated_thrust_to_normalized_command(
              motor_thrust_now);
#if defined(CONFIG_PLATFORM_SITL)
      if (diagnostic_enabled) {
        diagnostic_first_action_raw[motor] = command;
        if (!std::isfinite(command)) {
          diagnostic_clamp_mask |= 1u << (motor + 8);
        } else if (command <= 0.0f) {
          diagnostic_clamp_mask |= 1u << motor;
        } else if (command >= 1.0f) {
          diagnostic_clamp_mask |= 1u << (motor + 4);
        }
      }
#endif
      active_motor_commands[motor] = T_MIN(T_MAX(command, 0.0f), 1.0f);
    }
#endif
    plan_start_tick = solve_tick;
    mpc_has_run = true;
    xSemaphoreGive(dataMutex);



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
#if TINYMPC_DIRECT_DRONET_SERVO
  controllerPidInit();
#if defined(CONFIG_PLATFORM_SITL)
  const paramVarId_t thrust_base_id =
      paramGetVarId("posCtlPid", "thrustBase");
  if (PARAM_VARID_IS_VALID(thrust_base_id)) {
    paramSetInt(
        thrust_base_id, TINYMPC_DIRECT_DRONET_SITL_THRUST_BASE_PWM);
    DEBUG_PRINT(
        "Tiny-PULP-DroNet CrazySim PID hover feed-forward=%u PWM (cf21B_500 mass/thrust calibrated)\n",
        (unsigned)TINYMPC_DIRECT_DRONET_SITL_THRUST_BASE_PWM);
  } else {
    DEBUG_PRINT(
        "Tiny-PULP-DroNet CrazySim PID hover feed-forward parameter unavailable\n");
  }
#endif
  sequentialObstacleLinkInit();
  tinyPulpDronetV3ServoReset(&direct_dronet_servo);
  pulpDronetV2BrakeReset(&direct_dronet_v2_brake);
  direct_dronet_command = {0.0f, 0.0f};
  direct_dronet_last_sample = 0u;
  DEBUG_PRINT(
      "Paper vision controller enabled: stock PID, no TinyMPC, mode=%d vmax=%.3fm/s\n",
      (int)TINYMPC_PID_VISION_MODE,
      (double)direct_dronet_servo.max_forward_speed_mps);
  direct_dronet_takeoff_initialized = false;
  direct_dronet_takeoff_start_tick = 0u;
  direct_dronet_takeoff_x_m = 0.0f;
  direct_dronet_takeoff_y_m = 0.0f;
  direct_dronet_takeoff_yaw_deg = 0.0f;
  return;
#else
  /* Start MPC initialization*/
#if TINYMPC_VISION_DRONETV2_BRAKE_ENABLE
  pulpDronetV2BrakeReset(&dronet_v2_brake);
  dronet_v2_speed_scale = 0.0f;
  dronet_v2_last_sample = 0u;
  DEBUG_PRINT(
      "PULP-DroNetV2 paper braking enabled through TinyMPC reference: steering silenced, integral=0.2, velocity_alpha=0.6\n");
#endif
#if TINYMPC_RATE_CASCADE
  attitudeControllerInit(1.0f / (float)LQR_RATE);
  rate_pid_was_tracking_fresh_command = false;
#endif
  estimatorSetAidingInhibit(EstimatorAidingInhibitNone);
  configureFlipInputPrimitive();
  tinyMpcFlipReset(&flip_state);
  flip_reference_active = false;
  flip_recovery_active = false;
  flip_recovery_settle_steps = 0u;
  flip_recovery_elapsed_steps = 0u;
  tinyMpcPowerLoopReset(&power_loop_state);
  power_loop_reference_active = false;
  power_loop_recovery_active = false;
  power_loop_recovery_settle_steps = 0u;
  power_loop_recovery_elapsed_steps = 0u;
  vertical_active_sensing_start_tick = 0u;
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
  tiny_SetStateLinearCost(&work, progress_state_linear_cost);
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uref[k] = ug;
    Uhrz[k] = ug;
    ZU[k] = ug;
    ZU_new[k] = ug;
    YU[k].setZero();
  }
  for (int k = 0; k < NHORIZON; ++k) {
    progress_state_linear_cost[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    YX[k].setZero();
  }
#if TINYMPC_RATE_CASCADE
  resetOuterLoopDuals();
  tinyMpcBankSelectorReset(
      &outer_bank_selector, &outer_bank_selector_config);
  outer_bank_selection = {
      TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
  active_outer_loop_model =
      &tinympc_outer_loop_models[TINYMPC_BANK_MODEL_LEVEL];
  outer_warm_start_yaw_rad = 0.0f;
  outer_warm_start_yaw_initialized = false;
  outer_frame_reseed_requested = false;
  DEBUG_PRINT(
      "Outer MPC model=identified_collective_rate bundles=%d inner_rate_hz=%d input=[T_N,p,q,r]\n",
      TINYMPC_OUTER_LOOP_MODEL_COUNT, LQR_RATE);
#elif defined(TINYMPC_USE_ACTUATOR_LTI)
  resetLevelActuatorDuals();
  tinyMpcBankSelectorReset(
      &level_bank_selector, &level_bank_selector_config);
  tinyMpcBrakingSelectorReset(
      &level_braking_selector, &level_braking_selector_config);
  level_bank_selection = {
      TINYMPC_BANK_MODEL_LEVEL, 0.0f, false, false, false};
  level_braking_selection = {
      TINYMPC_BRAKING_MODEL_LEVEL, false, false, false, false};
  active_level_actuator_model =
      &tinympc_banked_models[TINYMPC_BANK_MODEL_LEVEL];
  DEBUG_PRINT(
      "Level MPC cost=%s Qyaw=%.1f Qyaw_rate=%.1f Ryaw=%.6f\n",
      TINYMPC_LEVEL_COST_MODE_NAME,
      (double)TINYMPC_LEVEL_Q_YAW,
      (double)TINYMPC_LEVEL_Q_YAW_RATE,
      (double)TINYMPC_LEVEL_R_YAW_EIGENVALUE);
  DEBUG_PRINT(
      "Progress reward weight=%.3f cost=-lambda*sum(tangent_dot_velocity) equivalent_speed_bias=%.3fm/s\n",
      (double)(float)TINYMPC_PROGRESS_REWARD_WEIGHT,
      (double)((float)TINYMPC_PROGRESS_REWARD_WEIGHT /
          tinympc_generated_Q_diagonal[6]));
#if TINYMPC_FLIP_ENABLE
  DEBUG_PRINT(
      "Flip primitive enabled trigger_s_m=%.3f..%.3f duration_s=%.3f pitch_direction=%d reference=quaternion+body_rate+motor_feedforward model=level_frozen_relative_error\n",
      (double)flip_config.trigger_start_s_m,
      (double)flip_config.trigger_end_s_m,
      (double)flip_config.duration_s,
      (int)flip_config.pitch_direction);
#else
  DEBUG_PRINT("Flip primitive disabled at compile time\n");
#endif
#if TINYMPC_POWER_LOOP_ENABLE
  DEBUG_PRINT(
      "Power loop enabled trigger_s_m=%.3f..%.3f radius=%.3fm bottom_speed=%.3fm/s top_speed=%.3fm/s spatial_reference=position+velocity+quaternion+body_rate+motor_feedforward\n",
      (double)power_loop_config.trigger_start_s_m,
      (double)power_loop_config.trigger_end_s_m,
      (double)power_loop_config.radius_m,
      (double)power_loop_config.bottom_speed_mps,
      (double)power_loop_config.top_speed_mps);
#else
  DEBUG_PRINT("Power loop disabled at compile time\n");
#endif
#if TINYMPC_PITCH_THROUGH_BRAKE_ENABLE
  DEBUG_PRINT(
      "Pitch-through emergency brake enabled: cache=none direct_motors=1 pitch=%.1f..%.1fdeg speed_index=%.1f..%.1fm/s pitch_rate=%.1frad/s altitude_budget=%.2fm reverse_recovery=allowed\n",
      (double)(pitch_through_brake_config.minimum_braking_pitch_rad *
          57.2957795f),
      (double)(pitch_through_brake_config.maximum_braking_pitch_rad *
          57.2957795f),
      (double)pitch_through_brake_config.minimum_pitch_speed_mps,
      (double)pitch_through_brake_config.maximum_pitch_speed_mps,
      (double)pitch_through_brake_config.maximum_pitch_rate_rad_s,
      (double)pitch_through_brake_config.maximum_altitude_loss_m);
#else
  DEBUG_PRINT("Pitch-through emergency brake disabled at compile time\n");
#endif
  for (int motor = 0; motor < NINPUTS; ++motor) {
    level_motor_rotor_state_estimate[motor] = 0.0f;
    planner_level_motor_rotor_state_estimate[motor] = 0.0f;
  }
#endif
  tiny_ClearPositionHalfspaces(&work);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 1;
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
  solve_tick_timing_initialized = false;
  previous_solve_tick = 0u;
  solve_tick_gap_min = UINT32_MAX;
  solve_tick_gap_max = 0u;
  solve_tick_gap_samples = 0u;
  solve_tick_skipped_periods = 0u;

#if defined(CONFIG_PLATFORM_SITL)
  sitlDiagInit();
#endif

  sequentialObstacleLinkInit();

#if TINYMPC_REACTIVE_REFERENCE_FREE
  tinyRacerReactiveReset(&reactive_reference_state);
  reactive_reference_command = {
      0.0f, 0.0f, 0.0f, TINYRACER_REACTIVE_CRUISE, 0, false};
  reactive_reference_last_sample = 0u;
  reactive_reference_speed_mps = 0.0f;
  reactive_reference_lateral_speed_mps = 0.0f;
  reactive_reference_altitude_world_m = 0.0f;
  reactive_reference_heading_world_rad = 0.0f;
  reactive_reference_turn_rate_rad_s = 0.0f;
  reactive_reference_initialized = false;
#if TINYMPC_REACTIVE_POWER_LOOP_ENABLE
  tinyMpcReactivePowerLoopReset(&reactive_power_loop_state);
  reactive_power_loop_command = {};
  reactive_power_loop_step = 0u;
  reactive_power_loop_anchor_world = Eigen::Vector3f::Zero();
  reactive_power_loop_recovery_anchor_world = Eigen::Vector3f::Zero();
  reactive_power_loop_yaw_world_rad = 0.0f;
#endif
#endif

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
#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
    for (uint32_t knot = 0u; knot < TINYMPC_DIRECT_PLAN_INPUT_KNOTS;
         ++knot) {
      active_motor_plan[knot][motor] = 0.0f;
    }
    cached_motor_commands[motor] = 0.0f;
#else
    cached_motor_command[motor] = 0.0f;
#endif
  }
#if TINYMPC_RATE_CASCADE
  memset(&active_rate_command, 0, sizeof(active_rate_command));
  memset(&cached_rate_command, 0, sizeof(cached_rate_command));
  cached_rate_command_valid = false;
  cached_rate_command_tick = 0u;
  rate_pid_was_tracking_fresh_command = false;
  planner_rate_pid_reset_pending = false;
  active_rate_pid_reset_requested = false;
#endif
#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
  cached_motor_plan_valid = false;
  cached_motor_plan_tick = 0u;
#else
  cached_motor_command_valid = false;
  cached_motor_command_tick = 0u;
#endif
  planner_reset_requested = true;
  xSemaphoreGive(dataMutex);
  
  if (en_traj) {
    DEBUG_PRINT("Stored trajectory enabled\n");
  } else {
    DEBUG_PRINT("Commander/setpoint mode enabled\n");
  }
#if TINYMPC_RATE_CASCADE
  DEBUG_PRINT(
      "TinyMPC outer loop -> 500Hz body-rate PID -> legacy motor mixer enabled\n");
  DEBUG_PRINT(
      "BRAKE cache unavailable: rate-cascade outer bank is turn-only; use TINYMPC_RATE_CASCADE=0 with actuator LTI\n");
#else
  DEBUG_PRINT("Exclusive TinyMPC direct motor control enabled\n");
#if TINYMPC_BRAKING_CACHE_ENABLE
  DEBUG_PRINT("BRAKE cache enabled: locality-gated speed/pitch matrices\n");
#else
  DEBUG_PRINT("BRAKE cache disabled by experiment build; level matrix remains active during braking\n");
#endif
#endif
#endif
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
#if TINYMPC_DIRECT_DRONET_SERVO
#if defined(TINYMPC_SITL_START_DELAY_MS)
  if (xTaskGetTickCount() < M2T(TINYMPC_SITL_START_DELAY_MS)) {
    memset(control, 0, sizeof(*control));
    control->controlMode = controlModeLegacy;
    return;
  }
#endif
  if (!direct_dronet_takeoff_initialized) {
    direct_dronet_takeoff_initialized = true;
    direct_dronet_takeoff_start_tick = tick;
    direct_dronet_takeoff_x_m = state->position.x;
    direct_dronet_takeoff_y_m = state->position.y;
    direct_dronet_takeoff_yaw_deg = state->attitude.yaw;
  }
  const float target_altitude_m =
      (float)TINYMPC_DIRECT_DRONET_HOLD_ALTITUDE_M;
  const uint32_t ramp_steps = (uint32_t)ceilf(
      (target_altitude_m - TINYMPC_DIRECT_DRONET_TAKEOFF_START_ALTITUDE_M) /
      TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_M);
  const uint32_t ramp_duration_ms =
      ramp_steps * TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_MS;
  const uint32_t takeoff_elapsed_ticks =
      tick - direct_dronet_takeoff_start_tick;
  const bool takeoff_complete = takeoff_elapsed_ticks >= M2T(
      ramp_duration_ms + TINYMPC_DIRECT_DRONET_TAKEOFF_HOLD_MS);
  if (!takeoff_complete) {
    const uint32_t elapsed_steps = T_MIN(
        takeoff_elapsed_ticks / M2T(TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_MS),
        ramp_steps);
    setpoint_t takeoff_setpoint = {};
    takeoff_setpoint.mode.x = modeAbs;
    takeoff_setpoint.mode.y = modeAbs;
    takeoff_setpoint.mode.z = modeAbs;
    takeoff_setpoint.mode.yaw = modeAbs;
    takeoff_setpoint.position.x = direct_dronet_takeoff_x_m;
    takeoff_setpoint.position.y = direct_dronet_takeoff_y_m;
    takeoff_setpoint.position.z = T_MIN(
        target_altitude_m,
        TINYMPC_DIRECT_DRONET_TAKEOFF_START_ALTITUDE_M +
            elapsed_steps * TINYMPC_DIRECT_DRONET_TAKEOFF_STEP_M);
    takeoff_setpoint.attitude.yaw = direct_dronet_takeoff_yaw_deg;
    controllerPid(control, &takeoff_setpoint, sensors, state, tick);
    return;
  }
  TinyRacerPerceptionObservation observation = {};
  const bool available = sequentialObstacleLinkGetLatest(&observation);
  const bool paper_control_enabled = state->position.x >=
      (float)TINYMPC_PAPER_ABLATION_CONTROL_ENABLE_X_M;
  if (!paper_control_enabled) {
    direct_dronet_command = {
        direct_dronet_servo.max_forward_speed_mps, 0.0f};
    tinyPulpDronetV3ServoReset(&direct_dronet_servo);
    pulpDronetV2BrakeReset(&direct_dronet_v2_brake);
  } else if (available && observation.valid &&
      observation.has_navigation_command &&
      observation.received_age_ms <= 200u) {
    if (observation.sample != direct_dronet_last_sample) {
#if TINYMPC_PID_VISION_MODE == TINYMPC_PID_VISION_DRONETV3
      direct_dronet_command = tinyPulpDronetV3ServoStep(
          &direct_dronet_servo,
          TINYMPC_DIRECT_DRONET_STEERING_SILENCED
              ? 0.0f : observation.steering_command,
          observation.collision_probability);
#elif TINYMPC_PID_VISION_MODE == TINYMPC_PID_VISION_DRONETV2
      direct_dronet_command.forward_velocity_mps =
          direct_dronet_servo.max_forward_speed_mps * pulpDronetV2BrakeStep(
              &direct_dronet_v2_brake,
              observation.collision_probability);
      direct_dronet_command.yaw_rate_deg_s = 0.0f;
#elif TINYMPC_PID_VISION_MODE == TINYMPC_PID_VISION_NANOFLOW
      direct_dronet_command.forward_velocity_mps =
          direct_dronet_servo.max_forward_speed_mps;
      direct_dronet_command.yaw_rate_deg_s =
          observation.steering_command * 57.2957795131f;
#elif TINYMPC_PID_VISION_MODE == TINYMPC_PID_VISION_TINYVPC
      direct_dronet_command.forward_velocity_mps =
          direct_dronet_servo.max_forward_speed_mps *
          (1.0f - fminf(fmaxf(observation.collision_probability, 0.0f), 1.0f));
      direct_dronet_command.yaw_rate_deg_s =
          fminf(fmaxf(observation.steering_command, -1.0f), 1.0f) * 90.0f;
#endif
      direct_dronet_last_sample = observation.sample;
    }
  } else {
    direct_dronet_command = {0.0f, 0.0f};
    tinyPulpDronetV3ServoReset(&direct_dronet_servo);
    pulpDronetV2BrakeReset(&direct_dronet_v2_brake);
  }
  const float direct_forward_speed_mps =
      direct_dronet_command.forward_velocity_mps;
  const float direct_yaw_rate_deg_s = direct_dronet_command.yaw_rate_deg_s;
  setpoint_t direct_setpoint = {};
  direct_setpoint.mode.x = modeVelocity;
  direct_setpoint.mode.y = modeVelocity;
  direct_setpoint.mode.z = modeAbs;
  direct_setpoint.mode.yaw = modeVelocity;
  direct_setpoint.velocity.x = direct_forward_speed_mps;
  direct_setpoint.velocity.y = 0.0f;
  direct_setpoint.position.z = target_altitude_m;
#if TINYMPC_PID_VISION_MODE == TINYMPC_PID_VISION_NANOFLOW
  const TinyMpcVerticalActiveSensingConfig nanoflow_survey = {
      0.10f, 2.0f};
  const float survey_elapsed_s =
      (float)(takeoff_elapsed_ticks - M2T(
          ramp_duration_ms + TINYMPC_DIRECT_DRONET_TAKEOFF_HOLD_MS)) /
      (float)configTICK_RATE_HZ;
  direct_setpoint.position.z += tinyMpcVerticalActiveSensingOffset(
      &nanoflow_survey, survey_elapsed_s);
#endif
  direct_setpoint.attitudeRate.yaw = direct_yaw_rate_deg_s;
  direct_setpoint.velocity_body = true;
  controllerPid(control, &direct_setpoint, sensors, state, tick);
  return;
#else
  const bool controller_reactivated =
      last_controller_tick == 0 || tick - last_controller_tick > M2T(200);
  last_controller_tick = tick;
  // CrazySim's SITL stabilizer intentionally bypasses the hardware supervisor
  // and always permits motor output. Mirror that contract here; hardware still
  // uses the real arming/supervisor state.
#if defined(CONFIG_PLATFORM_SITL)
#if !defined(TINYMPC_SITL_START_DELAY_MS)
  #define TINYMPC_SITL_START_DELAY_MS 0U
#endif
  const bool diagnostic_enabled = sitlDiagEnabled();
  const uint32_t sitl_now_tick = xTaskGetTickCount();
  const bool motors_allowed =
      sitl_now_tick >= M2T(TINYMPC_SITL_START_DELAY_MS);
#else
  const bool motors_allowed = supervisorAreMotorsAllowedToRun();
#endif
  const bool motors_allowed_changed = motors_allowed != motors_were_allowed;
  bool has_run_snapshot = false;
  uint32_t plan_start_tick_snapshot = 0;
  float motor_command_snapshot[NINPUTS] = {0.0f};
#if TINYMPC_RATE_CASCADE
  bool rate_pid_reset_requested_snapshot = false;
  TinyMpcRateCommand rate_command_snapshot = cached_rate_command;
  has_run_snapshot = cached_rate_command_valid;
  plan_start_tick_snapshot = cached_rate_command_tick;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    motor_command_snapshot[motor] = cached_motor_command[motor];
  }
  if (controller_reactivated || !motors_allowed) {
    cached_rate_command_valid = false;
    cached_motor_command_valid = false;
    has_run_snapshot = false;
  }
#elif defined(TINYMPC_DIRECT_PLAN_REPLAY)
  bool plan_replay_fresh_snapshot = cached_motor_plan_valid;
  has_run_snapshot = cached_motor_plan_valid;
  plan_start_tick_snapshot = cached_motor_plan_tick;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    motor_command_snapshot[motor] = cached_motor_commands[motor];
  }
  if (controller_reactivated || !motors_allowed) {
    cached_motor_plan_valid = false;
    plan_replay_fresh_snapshot = false;
    has_run_snapshot = false;
  } else if (has_run_snapshot) {
    plan_replay_fresh_snapshot = tinyMpcDirectPlanReplaySelect(
        tick, plan_start_tick_snapshot, TINYMPC_DIRECT_PLAN_KNOT_TICKS,
        TINYMPC_DIRECT_PLAN_INPUT_KNOTS).valid;
  }
#else
  has_run_snapshot = cached_motor_command_valid;
  plan_start_tick_snapshot = cached_motor_command_tick;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    motor_command_snapshot[motor] = cached_motor_command[motor];
  }
  if (controller_reactivated || !motors_allowed) {
    cached_motor_command_valid = false;
    has_run_snapshot = false;
  }
#endif
#if defined(CONFIG_PLATFORM_SITL)
  const bool diagnostic_release_due =
      diagnostic_enabled && motors_allowed && RATE_DO_EXECUTE(MPC_RATE, tick);
  if (diagnostic_release_due) {
    ++sitl_diag_release_due_count;
  }
#endif
  if (dataMutex != NULL && xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    if (controller_reactivated || (!motors_allowed && motors_were_allowed)) {
      mpc_has_run = false;
      planner_reset_requested = true;
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
      for (int motor = 0; motor < NINPUTS; ++motor) {
        level_motor_rotor_state_estimate[motor] = 0.0f;
      }
#endif
    }
    if (motors_allowed && RATE_DO_EXECUTE(MPC_RATE, tick)) {
      memcpy(&planner_setpoint, setpoint, sizeof(planner_setpoint));
      memcpy(&planner_sensors, sensors, sizeof(planner_sensors));
      memcpy(&planner_state, state, sizeof(planner_state));
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
      memcpy(planner_level_motor_rotor_state_estimate,
             level_motor_rotor_state_estimate,
             sizeof(planner_level_motor_rotor_state_estimate));
#endif
      planner_tick = tick;
#if defined(CONFIG_PLATFORM_SITL)
      if (diagnostic_enabled) {
        __atomic_store_n(
            &planner_diag_release_us, usecTimestamp(), __ATOMIC_RELAXED);
        __atomic_store_n(
            &planner_diag_release_rtos_tick, sitl_now_tick,
            __ATOMIC_RELAXED);
        __atomic_store_n(
            &planner_diag_release_sequence, ++sitl_diag_release_sequence,
            __ATOMIC_RELAXED);
        const BaseType_t release_result = xSemaphoreGive(runTaskSemaphore);
        if (release_result != pdTRUE) {
          ++sitl_diag_semaphore_coalesced_count;
        }
        __atomic_store_n(
            &planner_diag_release_due_count, sitl_diag_release_due_count,
            __ATOMIC_RELAXED);
        __atomic_store_n(
            &planner_diag_release_mutex_miss_count,
            sitl_diag_release_mutex_miss_count, __ATOMIC_RELAXED);
        __atomic_store_n(
            &planner_diag_semaphore_coalesced_count,
            sitl_diag_semaphore_coalesced_count, __ATOMIC_RELAXED);
      } else {
        xSemaphoreGive(runTaskSemaphore);
      }
#else
      xSemaphoreGive(runTaskSemaphore);
#endif
    }
    has_run_snapshot = mpc_has_run;
    plan_start_tick_snapshot = plan_start_tick;
#if TINYMPC_RATE_CASCADE
    rate_command_snapshot = active_rate_command;
    rate_pid_reset_requested_snapshot = active_rate_pid_reset_requested;
    active_rate_pid_reset_requested = false;
    cached_rate_command = rate_command_snapshot;
    cached_rate_command_valid = has_run_snapshot;
    cached_rate_command_tick = plan_start_tick_snapshot;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      motor_command_snapshot[motor] = active_motor_commands[motor];
      cached_motor_command[motor] = motor_command_snapshot[motor];
    }
    cached_motor_command_valid = has_run_snapshot;
    cached_motor_command_tick = plan_start_tick_snapshot;
#elif defined(TINYMPC_DIRECT_PLAN_REPLAY)
    const TinyMpcDirectPlanReplaySelection replay_selection =
        tinyMpcDirectPlanReplaySelect(
            tick, plan_start_tick_snapshot, TINYMPC_DIRECT_PLAN_KNOT_TICKS,
            TINYMPC_DIRECT_PLAN_INPUT_KNOTS);
    plan_replay_fresh_snapshot = has_run_snapshot && replay_selection.valid;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      motor_command_snapshot[motor] =
          active_motor_plan[replay_selection.knot][motor];
      cached_motor_commands[motor] = motor_command_snapshot[motor];
    }
    cached_motor_plan_valid = has_run_snapshot;
    cached_motor_plan_tick = plan_start_tick_snapshot;
#else
    for (int motor = 0; motor < NINPUTS; ++motor) {
      motor_command_snapshot[motor] = active_motor_commands[motor];
      cached_motor_command[motor] = motor_command_snapshot[motor];
    }
    cached_motor_command_valid = has_run_snapshot;
    cached_motor_command_tick = plan_start_tick_snapshot;
#endif
    xSemaphoreGive(dataMutex);
  } else {
#if defined(CONFIG_PLATFORM_SITL)
#if defined(TINYMPC_DIRECT_PLAN_REPLAY)
    if (diagnostic_enabled) {
      ++sitl_diag_release_mutex_miss_count;
    }
#else
    if (diagnostic_release_due) {
      ++sitl_diag_release_mutex_miss_count;
    }
#endif
#endif
  }
  motors_were_allowed = motors_allowed;

  const bool command_is_fresh = has_run_snapshot &&
#if TINYMPC_RATE_CASCADE
      (tick - plan_start_tick_snapshot <=
       M2T(TINYMPC_RATE_COMMAND_MAX_AGE_MS));
#elif defined(TINYMPC_DIRECT_PLAN_REPLAY)
      plan_replay_fresh_snapshot;
#else
      (tick - plan_start_tick_snapshot <=
       M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS));
#endif
#if defined(CONFIG_PLATFORM_SITL)
  if (diagnostic_enabled) {
    bool diagnostic_command_finite = true;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      diagnostic_command_finite = diagnostic_command_finite &&
          std::isfinite(motor_command_snapshot[motor]);
    }
#if TINYMPC_RATE_CASCADE
    diagnostic_command_finite = diagnostic_command_finite &&
        std::isfinite(rate_command_snapshot.collective_thrust_n);
    for (int axis = 0; axis < 3; ++axis) {
      diagnostic_command_finite = diagnostic_command_finite &&
          std::isfinite(rate_command_snapshot.body_rate_rad_s[axis]);
    }
#endif
    const uint32_t diagnostic_fallback_reason = !motors_allowed ? 1u
        : !has_run_snapshot ? 2u
        : !command_is_fresh ? 3u
        : !diagnostic_command_finite ? 4u : 0u;
    if (diagnostic_fallback_reason != sitl_diag_previous_fallback_reason) {
    sitl_diag_previous_fallback_reason = diagnostic_fallback_reason;
    TinyMpcSitlDiagRecord& record = sitl_diag_fallback_record;
    memset(&record, 0, sizeof(record));
    record.event = 2u;
    record.release_sequence = sitl_diag_release_sequence;
    record.solve_sequence = __atomic_load_n(
        &sitl_diag_solve_sequence, __ATOMIC_RELAXED);
    record.release_tick = 0u;
    record.start_tick = 0u;
    record.finish_tick = sitl_now_tick;
    record.plan_tick = plan_start_tick_snapshot;
    record.plan_age_ticks = has_run_snapshot
        ? tick - plan_start_tick_snapshot : UINT32_MAX;
    record.release_due_count = sitl_diag_release_due_count;
    record.release_mutex_miss_count = sitl_diag_release_mutex_miss_count;
    record.semaphore_coalesced_count =
        sitl_diag_semaphore_coalesced_count;
    record.fallback_reason = diagnostic_fallback_reason;
    record.finish_us = usecTimestamp();
    const float estimator_values[13] = {
        state->position.x, state->position.y, state->position.z,
        state->velocity.x, state->velocity.y, state->velocity.z,
        state->attitudeQuaternion.x, state->attitudeQuaternion.y,
        state->attitudeQuaternion.z, state->attitudeQuaternion.w,
        radians(sensors->gyro.x), radians(sensors->gyro.y),
        radians(sensors->gyro.z)};
    memcpy(record.estimator, estimator_values, sizeof(estimator_values));
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float hover_command =
          tinympc_generated_thrust_to_normalized_command(
              tinympc_generated_physical_hover_thrust[motor]);
      record.first_action_raw[motor] = motor_command_snapshot[motor];
      record.first_action_clamped[motor] = !motors_allowed ? 0.0f
          : diagnostic_fallback_reason == 0u
              ? motor_command_snapshot[motor] : hover_command;
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
      record.rotor_state[motor] = level_motor_rotor_state_estimate[motor];
#endif
    }
    sitlDiagPublish(&record);
    }
  }
#endif
#if TINYMPC_RATE_CASCADE
  bool rate_command_is_finite =
      std::isfinite(rate_command_snapshot.collective_thrust_n);
  for (int axis = 0; axis < 3; ++axis) {
    rate_command_is_finite = rate_command_is_finite &&
        std::isfinite(rate_command_snapshot.body_rate_rad_s[axis]);
  }
  const bool track_fresh_rate_command =
      command_is_fresh && rate_command_is_finite;
  if (controller_reactivated || motors_allowed_changed ||
      rate_pid_reset_requested_snapshot ||
      track_fresh_rate_command != rate_pid_was_tracking_fresh_command) {
    /* Reset integral memory at every authority/fallback boundary. A stale MPC
     * plan falls back to rate-stabilized hover, never a stale rate target. */
#if defined(CONFIG_PLATFORM_SITL)
    /* CrazySim's pinned firmware predates the attitude-value reset API. */
    attitudeControllerResetAllPID();
#else
    attitudeControllerResetAllPID(
        state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
#endif
    /* If the reset lands between 500 Hz rate updates, do not retain a stale
     * torque command for the intervening 1 ms legacy-mixer cycle. */
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
  }
  rate_pid_was_tracking_fresh_command = track_fresh_rate_command;

  control->controlMode = controlModeLegacy;
  if (!motors_allowed) {
    control->thrust = 0.0f;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
  } else {
    const float collective_thrust_n = track_fresh_rate_command
        ? rate_command_snapshot.collective_thrust_n
        : hoverCollectiveThrustN();
    control->thrust = collectiveThrustToLegacyCommand(collective_thrust_n);
    if (RATE_DO_EXECUTE(LQR_RATE, tick)) {
      const float desired_roll_rate_deg_s = track_fresh_rate_command
          ? degrees(rate_command_snapshot.body_rate_rad_s[0]) : 0.0f;
      /* TinyMPC stores the raw body-y gyro convention. Crazyflie's stock rate
       * PID and legacy mixer use the opposite pitch-rate sign. */
      const float desired_pitch_rate_deg_s = track_fresh_rate_command
          ? -degrees(rate_command_snapshot.body_rate_rad_s[1]) : 0.0f;
      const float desired_yaw_rate_deg_s = track_fresh_rate_command
          ? degrees(rate_command_snapshot.body_rate_rad_s[2]) : 0.0f;
      attitudeControllerCorrectRatePID(
          sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
          desired_roll_rate_deg_s, desired_pitch_rate_deg_s,
          desired_yaw_rate_deg_s);
      attitudeControllerGetActuatorOutput(
          &control->roll, &control->pitch, &control->yaw);
      control->yaw = -control->yaw;
#if defined(TINYMPC_USE_ACTUATOR_LTI) && !TINYMPC_RATE_CASCADE
      /* Keep the augmented rotor state tied to what the inner loop actually
       * sends through the legacy X mixer. This does not pretend that the old
       * direct-motor B matrix models the closed rate loop; it only prevents
       * the next 50 Hz solve from starting with fictitious rotor speeds. */
      const int32_t legacy_roll = control->roll / 2.0f;
      const int32_t legacy_pitch = control->pitch / 2.0f;
      int32_t mixed_pwm[NINPUTS] = {
        (int32_t)control->thrust - legacy_roll + legacy_pitch + control->yaw,
        (int32_t)control->thrust - legacy_roll - legacy_pitch - control->yaw,
        (int32_t)control->thrust + legacy_roll - legacy_pitch + control->yaw,
        (int32_t)control->thrust + legacy_roll + legacy_pitch - control->yaw,
      };
      int32_t highest_pwm = 0;
      for (int motor = 0; motor < NINPUTS; ++motor) {
        highest_pwm = T_MAX(highest_pwm, mixed_pwm[motor]);
      }
      const int32_t common_reduction = T_MAX(
          highest_pwm - (int32_t)UINT16_MAX, 0);
      const float estimator_alpha = 1.0f - expf(
          -(1.0f / (float)LQR_RATE) /
              TINYMPC_LEVEL_MOTOR_TIME_CONSTANT_S);
      for (int motor = 0; motor < NINPUTS; ++motor) {
        const float normalized_command =
            (float)T_MIN(T_MAX(
                mixed_pwm[motor] - common_reduction, 0),
                (int32_t)UINT16_MAX) / (float)UINT16_MAX;
        const float target_thrust_n = T_MIN(
            tinympc_generated_normalized_command_to_thrust(
                normalized_command),
            TINYMPC_LEVEL_MAX_MOTOR_THRUST_N);
        const float target_rotor_state =
            thrustToLevelRotorState(target_thrust_n);
        level_motor_rotor_state_estimate[motor] += estimator_alpha *
            (target_rotor_state - level_motor_rotor_state_estimate[motor]);
      }
#endif
#if defined(CONFIG_PLATFORM_SITL)
      static uint32_t rate_cascade_debug_counter = 0u;
      if (rate_cascade_debug_counter < 2000u &&
          (rate_cascade_debug_counter % 50u) == 0u) {
        DEBUG_PRINT(
            "RATE cascade fresh=%d T=%.4fN omega_d=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f) legacy=(%d,%d,%d,%.0f)\n",
            track_fresh_rate_command,
            (double)collective_thrust_n,
            (double)(track_fresh_rate_command
                ? rate_command_snapshot.body_rate_rad_s[0] : 0.0f),
            (double)(track_fresh_rate_command
                ? rate_command_snapshot.body_rate_rad_s[1] : 0.0f),
            (double)(track_fresh_rate_command
                ? rate_command_snapshot.body_rate_rad_s[2] : 0.0f),
            (double)radians(sensors->gyro.x),
            (double)radians(sensors->gyro.y),
            (double)radians(sensors->gyro.z),
            (int)control->roll, (int)control->pitch, (int)control->yaw,
            (double)control->thrust);
      }
      ++rate_cascade_debug_counter;
#endif
    }
  }
#else
  control->controlMode = controlModePWM;
  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; ++motor) {
    const float hover_command =
        tinympc_generated_thrust_to_normalized_command(
            tinympc_generated_physical_hover_thrust[motor]);
    float command = command_is_fresh ? motor_command_snapshot[motor]
                                     : hover_command;
#if defined(CONFIG_PLATFORM_SITL)
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
#if defined(TINYMPC_USE_ACTUATOR_LTI)
    {
    const float target_thrust_n = motors_allowed
        ? T_MIN(tinympc_generated_normalized_command_to_thrust(command),
                TINYMPC_LEVEL_MAX_MOTOR_THRUST_N)
        : 0.0f;
    const float target_rotor_state = thrustToLevelRotorState(target_thrust_n);
    /* The out-of-tree controller is called by the 1 kHz stabilizer loop in
     * direct-motor mode. Integrate the rotor observer at that callback period;
     * using LQR_RATE here applied a 2 ms update every 1 ms and halved the
     * modeled motor time constant. */
    const float estimator_alpha = 1.0f - expf(
        -(1.0f / (float)RATE_MAIN_LOOP) /
            TINYMPC_LEVEL_MOTOR_TIME_CONSTANT_S);
    level_motor_rotor_state_estimate[motor] += estimator_alpha
        * (target_rotor_state - level_motor_rotor_state_estimate[motor]);
    }
#endif
  }
#endif
#endif
}

#if defined(__cplusplus)
}
#endif
