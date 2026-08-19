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
#else
#include "trajectories/50hz/traj_circle_50hz.h"
#endif
#ifndef TRAJECTORY_HAS_MOTOR_FEEDFORWARD
#define TRAJECTORY_HAS_MOTOR_FEEDFORWARD 0
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
  0.28f, 0.70f, 2, 0.18f, 2
};
typedef struct {
  Eigen::Vector3f normal_world;
  float boundary_world;
} PerceptionPlane;
static PerceptionPlane perception_stop_plane;
static bool perception_obstacle_active = false;
static bool perception_halfspace_active = false;
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

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(
    TRAJECTORY_REFERENCE_DIM == (TRAJECTORY_HAS_MOTOR_FEEDFORWARD ? 17 : 13),
    "trajectory reference dimension mismatch");

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
#endif
static VectorMf acro_motor_baseline_n;
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
static float acro_altitude_estimate_m = 0.0f;
static bool acro_altitude_initialized = false;
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

static void sampleTrajectoryReference(
    float sample_position, float reference[TRAJECTORY_REFERENCE_DIM]);

#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
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

#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
  // The hover model now governs tracking error, not the absolute attitude.
  // Recenter its Rodrigues chart on the current reference every solve.  This
  // prevents the q_xyz/q_w state from becoming singular when a flip crosses
  // 180 degrees, while retaining TinyMPC's fixed offline A/B and Riccati cache.
  const float current_time_s = trajectory_handoff_hold_steps > 0
      ? 0.0f : (float)step / (float)MPC_RATE;
  float reference[TRAJECTORY_REFERENCE_DIM];
  sampleTrajectoryReference(
      current_time_s / TRAJECTORY_SAMPLE_DT_S, reference);
  const struct quat reference_attitude = qnormalize(qqmul(
      trajectory_yaw_rotation,
      mkquat(reference[4], reference[5], reference[6], reference[3])));
  const Eigen::Vector3f reference_position = trajectory_origin_world + Eigen::Vector3f(
      trajectory_cos_yaw * reference[0] - trajectory_sin_yaw * reference[1],
      trajectory_sin_yaw * reference[0] + trajectory_cos_yaw * reference[1],
      reference[2]);
  const Eigen::Vector3f reference_velocity(
      trajectory_cos_yaw * reference[7] - trajectory_sin_yaw * reference[8],
      trajectory_sin_yaw * reference[7] + trajectory_cos_yaw * reference[8],
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
#else
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

static void sampleTrajectoryReference(
    float sample_position, float reference[TRAJECTORY_REFERENCE_DIM]) {
      if (sample_position > (float)(traj_length - 1)) {
        sample_position = (float)(traj_length - 1);
      }
      const uint32_t lower_idx = (uint32_t)floorf(sample_position);
      const uint32_t upper_idx =
          lower_idx + 1 < traj_length ? lower_idx + 1 : lower_idx;
      const float alpha = sample_position - (float)lower_idx;
  for (int field = 0; field < TRAJECTORY_REFERENCE_DIM; ++field) {
    reference[field] =
        (1.0f - alpha) * trajectory_reference_data[lower_idx][field]
        + alpha * trajectory_reference_data[upper_idx][field];
  }

  // Quaternion signs are continuous in generated trajectories.
  float quaternion_norm_sq = 0.0f;
  for (int field = 3; field <= 6; ++field) {
    quaternion_norm_sq += reference[field] * reference[field];
      }
  const float quaternion_inverse_norm = 1.0f / sqrtf(quaternion_norm_sq);
  for (int field = 3; field <= 6; ++field) {
    reference[field] *= quaternion_inverse_norm;
  }
}

static float cross2d(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return a.x() * b.y() - a.y() * b.x();
}

static Eigen::Vector3f trajectoryPointWorld(uint32_t knot) {
  const float x = trajectory_reference_data[knot][0];
  const float y = trajectory_reference_data[knot][1];
  return trajectory_origin_world + Eigen::Vector3f(
      trajectory_cos_yaw * x - trajectory_sin_yaw * y,
      trajectory_sin_yaw * x + trajectory_cos_yaw * y,
      trajectory_reference_data[knot][2]);
}

static void rejoinTrajectory(
    const Eigen::Vector3f& position_world,
    const Eigen::Vector3f& heading_world) {
  uint32_t nearest = step;
  float nearest_forward_distance = INFINITY;
  for (uint32_t knot = step; knot + 1 < traj_length; ++knot) {
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
    for (uint32_t knot = step; knot < traj_length; ++knot) {
      Eigen::Vector3f point = trajectoryPointWorld(knot);
      point.z() = 0.0f;
      const float forward_distance = heading_world.dot(point - position_world);
      if (forward_distance >= 0.0f && forward_distance < nearest_forward_distance) {
        nearest_forward_distance = forward_distance;
        nearest = knot;
      }
    }
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
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
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
      const float maximum_motor_thrust =
          tinympc_generated_physical_hover_thrust[motor]
          + tinympc_generated_input_upper[motor * (NHORIZON - 1)];
      ucu(motor) = maximum_motor_thrust - acro_motor_baseline_n(motor);
    }
#else
    const float current_time_s = (float)step / (float)MPC_RATE;
    float yaw_reference = step == 0
        ? active_local_frame.yaw_world : reference_yaw_phase_rad;
    for (int i = 0; i < NHORIZON; ++i) {
      const float reference_time_s = trajectory_handoff_hold_steps > 0
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
          Xref[i],
          trajectory_origin_world + reference_position,
          reference_attitude,
          reference_velocity,
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
  if (advance && en_traj &&
      (float)step / (float)MPC_RATE < TRAJECTORY_DURATION_S) {
    if (trajectory_handoff_hold_steps > 0) {
      --trajectory_handoff_hold_steps;
    } else {
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
    if ((race_intent.constraint_active || perception_recovery_active) &&
        observation.valid &&
        observation.received_age_ms <= race_config.maximum_age_ms) {
      expandObstacleCylinder(position_world, observation);
    }
    updateObstacleTangent(position_world);
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
  if (perception_recovery_active && !race_intent.constraint_active &&
      race_state.clear_samples >= race_config.clear_samples_required &&
      perception_recovery_distance_m >= perception_pass_distance_m) {
    const struct vec rpy = quat2rpy(qnormalize(attitude));
    rejoinTrajectory(position_world, Eigen::Vector3f(
        cosf(rpy.z), sinf(rpy.z), 0.0f));
    perception_recovery_active = false;
    perception_halfspace_active = false;
    tiny_ClearPositionHalfspaces(&work);
    DEBUG_PRINT("Vision cylinder retained; tangent disabled\n");
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
    updateObstacleTangent(position_world);
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
  if (!createObstacleCylinder(position_world, observation)) {
    race_state.obstacle_constraint_active = false;
    race_intent.constraint_active = false;
    race_intent.mode = TINYRACER_RACE_TRACK;
    return;
  }
  race_state.pass_side = trajectorySideOfCylinder();
  race_intent.pass_side = race_state.pass_side;
  DEBUG_PRINT("Vision blocked action=%s path=%.0fdeg sectors=0x%x clearances=(%.2f,%.2f,%.2f,%.2f)\n",
              race_intent.pass_side > 0 ? "left" : "right",
              (double)(path_bearing * 57.2957795f), relevant_sector_mask,
              (double)observation.clearance_m[0],
              (double)observation.clearance_m[1],
              (double)observation.clearance_m[2],
              (double)observation.clearance_m[3]);
}

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
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &planner_setpoint, sizeof(setpoint_task));
    memcpy(&sensors_task, &planner_sensors, sizeof(sensors_task));
    memcpy(&state_task, &planner_state, sizeof(state_task));
    solve_tick = planner_tick;
    reset_requested = planner_reset_requested;
    planner_reset_requested = false;
    xSemaphoreGive(dataMutex);

    if (reset_requested) {
      step = 0;
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
      acro_altitude_initialized = false;
#endif
      const struct quat handoff_attitude = qnormalize(mkquat(
          state_task.attitudeQuaternion.x, state_task.attitudeQuaternion.y,
          state_task.attitudeQuaternion.z, state_task.attitudeQuaternion.w));
      const float handoff_yaw = quat2rpy(handoff_attitude).z;
      trajectory_cos_yaw = cosf(handoff_yaw);
      trajectory_sin_yaw = sinf(handoff_yaw);
      trajectory_yaw_rotation = rpy2quat(mkvec(0.0f, 0.0f, handoff_yaw));
      trajectory_handoff_hold_steps = (uint16_t)(MPC_RATE * 2 / 5);
      const float first_x = trajectory_reference_data[0][0];
      const float first_y = trajectory_reference_data[0][1];
      trajectory_origin_world = Eigen::Vector3f(
          state_task.position.x, state_task.position.y, state_task.position.z) - Eigen::Vector3f(
          trajectory_cos_yaw * first_x - trajectory_sin_yaw * first_y,
          trajectory_sin_yaw * first_x + trajectory_cos_yaw * first_y,
                          trajectory_reference_data[0][2]);
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
      perception_recovery_distance_m = 0.0f;
      resetPerceptionFilter();
      tiny_ClearPositionHalfspaces(&work);
    }

    updateInitialState(&sensors_task, &state_task);
    // Intentionally preserve TinyMPC's state auxiliaries and duals.
    updateHorizonReference(
        &setpoint_task,
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
        true);
    // Vision/racing halfspaces are expressed in the ordinary local-position
    // chart. They are intentionally detached from the acrobatic error chart.
    tiny_ClearPositionHalfspaces(&work);
#else
        race_intent.mode == TINYRACER_RACE_TRACK);
    updateRaceIntent(&state_task);
    applyRaceIntent();
#endif

    tiny_UpdateLinearCost(&work);
    const uint64_t solve_start_us = usecTimestamp();
    tiny_SolveAdmm(&work);
    const uint32_t solve_us = (uint32_t)(usecTimestamp() - solve_start_us);
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float motor_thrust_newtons =
#if TRAJECTORY_HAS_MOTOR_FEEDFORWARD
          acro_motor_baseline_n(motor) + Uhrz[0](motor);
#else
          tinympc_generated_physical_hover_thrust[motor] + Uhrz[0](motor);
#endif
      active_motor_commands[motor] =
          tinympc_generated_thrust_to_normalized_command(
              motor_thrust_newtons);
    }
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
      DEBUG_PRINT(
          "MPC: iterations=%d solve_us=%lu ref_local=(%.2f,%.2f,%.2f) plane_violation=%.3f consensus_error=%.3f slack=%.3f\n",
          info.iter, (unsigned long)solve_us,
          (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2),
          (double)max_primal_violation,
          (double)max_primal_aux_gap,
          (double)max_halfspace_slack);
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
  const bool motors_allowed = supervisorAreMotorsAllowedToRun();
  bool has_run_snapshot = false;
  uint32_t plan_start_tick_snapshot = 0;
  float motor_command_snapshot[NINPUTS] = {0.0f};
  if (dataMutex != NULL && xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    if (controller_reactivated || (!motors_allowed && motors_were_allowed)) {
      mpc_has_run = false;
      planner_reset_requested = true;
    }
    if (motors_allowed && RATE_DO_EXECUTE(MPC_RATE, tick)) {
      memcpy(&planner_setpoint, setpoint, sizeof(planner_setpoint));
      memcpy(&planner_sensors, sensors, sizeof(planner_sensors));
      memcpy(&planner_state, state, sizeof(planner_state));
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
    float command = command_is_fresh
        ? motor_command_snapshot[motor] : hover_command;
    if (!std::isfinite(command)) {
      command = hover_command;
    }
    if (command < 0.0f) {
      command = 0.0f;
    } else if (command > 1.0f) {
      command = 1.0f;
    }
    control->normalizedForces[motor] = command;
  }
}

#ifdef __cplusplus
}
#endif
