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
#include "controller_pid.h"
#include "position_controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "supervisor.h"
#include "sequential_obstacle_link.h"

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

static_assert(NSTATES == TINYMPC_GENERATED_STATE_DIM, "generated state dimension mismatch");
static_assert(NINPUTS == TINYMPC_GENERATED_INPUT_DIM, "generated input dimension mismatch");

/* Include trajectory to track */
// #include "trajectories/5hz/traj_straight_5hz.h"
#include "trajectories/5hz/traj_circle_5hz.h"
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

// Three candidate image-aligned boundaries project to vertical planes through
// the vehicle. Slice order is right-to-left.
static const float perception_boundary_angle_deg[3] = {
    -60.0f, 0.0f, 60.0f};
static const float perception_safe_min_m = 0.30f;
static const uint32_t perception_max_age_ms = 250;
static const float perception_halfspace_penalty = 500.0f;
// Use the latest GAP8 result directly. A one-sample window retains the same
// bookkeeping and decision code without adding temporal detection latency.
static const uint8_t perception_average_window = 1;
static const uint8_t perception_side_open_votes = 1;
static const uint8_t perception_all_blocked_votes = 1;
static const float perception_trigger_average = 2.0f;
static const float perception_clear_average = 1.0f;
static const uint32_t perception_post_clear_hold_ms = 500;
static const int perception_pid_lookahead_knots = 5;
static const float perception_max_plan_projection_m = 0.75f;
static const float perception_velocity_projection_margin_m = 0.15f;
static bool perception_halfspace_active = false;
static int8_t perception_boundary_index = -1;
static uint8_t perception_safe_mask = 0;
static Eigen::Vector3f perception_halfspace_normal = Eigen::Vector3f::Zero();
static float perception_halfspace_boundary = 0.0f;
static uint8_t perception_danger_history[perception_average_window] = {0};
static uint8_t perception_safe_history[perception_average_window] = {0};
static uint8_t perception_open_sum[SEQUENTIAL_OBSTACLE_DIRECTIONS] = {0};
static uint8_t perception_history_next = 0;
static uint8_t perception_history_count = 0;
static uint8_t perception_danger_sum = 0;
static float perception_danger_average = 0.0f;
static uint32_t perception_last_sample = 0;
static int8_t perception_evasion_side = 0;  // +1 right, -1 left.
static int8_t perception_remembered_safe_side = 0;  // +1 right, -1 left.
static TickType_t perception_post_clear_deadline = 0;

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

// Helper variables
// static bool isInit = false;  // fix for tracking problem - UNUSED, commented out
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = true;   // Track the generated stored trajectory.
static const uint32_t traj_length = T_ARRAY_SIZE(trajectory_reference_data);

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(TRAJECTORY_REFERENCE_DIM == 13, "trajectory reference dimension mismatch");

static struct quat attitude;
static MpcLocalFrame active_local_frame;
static float reference_yaw_unwrapped_rad[NHORIZON];
static float reference_yaw_phase_rad = 0.0f;

// TinyMPC plans a dynamically feasible position path. The stock cascaded PID
// tracks a near-term point from that path and owns attitude, rate, and motor control.
static setpoint_t mpc_setpoint_pid;
static bool mpc_has_run = false;
static uint32_t last_controller_tick = 0;
static uint32_t plan_start_tick = 0;
static bool motors_were_allowed = false;
static VectorNf published_Xhrz[NHORIZON];
static float published_yaw_unwrapped_rad[NHORIZON];
static int published_pid_lookahead_knots = 0;
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

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    const float current_time_s = (float)step / (float)MPC_RATE;
    float yaw_reference = step == 0
        ? active_local_frame.yaw_world : reference_yaw_phase_rad;
    for (int i = 0; i < NHORIZON; ++i) {
      const float reference_time_s = current_time_s + (float)i * DT;
      const float sample_position = reference_time_s / TRAJECTORY_SAMPLE_DT_S;
      float reference[TRAJECTORY_REFERENCE_DIM];
      sampleTrajectoryReference(sample_position, reference);
      const struct quat reference_attitude = mkquat(
          reference[4], reference[5], reference[6], reference[3]);
      setLocalReferenceState(
          Xref[i],
          Eigen::Vector3f(reference[0], reference[1], reference[2]),
          reference_attitude,
          Eigen::Vector3f(reference[7], reference[8], reference[9]),
          Eigen::Vector3f(reference[10], reference[11], reference[12]));
      yaw_reference = unwrapNear(quat2rpy(reference_attitude).z, yaw_reference);
      reference_yaw_unwrapped_rad[i] = yaw_reference;
      if (i < NHORIZON - 1) {
        Uref[i].setZero();
      }
    }
    reference_yaw_phase_rad = reference_yaw_unwrapped_rad[0];
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
    // // xg(1) = 1.0;
    // // xg(2) = 2.0;
  }
  if (en_traj && (float)step / (float)MPC_RATE < TRAJECTORY_DURATION_S) {
    step += 1;
  }
}

static void resetStateWarmStartInActiveFrame(void) {
  // State warm starts belong to the previous local frame.
  for (int k = 0; k < NHORIZON; ++k) {
    Xhrz[k] = Xref[k];
    ZX[k] = Xref[k];
    ZX_new[k] = Xref[k];
    YX[k].setZero();
  }
}

static void clearPerceptionHalfspace(void) {
  tiny_ClearPositionHalfspaces(&work);
  perception_halfspace_active = false;
  perception_boundary_index = -1;
  perception_halfspace_normal.setZero();
  perception_halfspace_boundary = 0.0f;
  perception_post_clear_deadline = 0;
}

static void applyPerceptionHalfspace(void) {
  const Eigen::Vector3f zero_velocity = Eigen::Vector3f::Zero();
  const Eigen::Vector3f normal_local =
      worldVectorToLocal(active_local_frame, perception_halfspace_normal);
  const float boundary_local = perception_halfspace_boundary
      - perception_halfspace_normal.x() * active_local_frame.origin_x
      - perception_halfspace_normal.y() * active_local_frame.origin_y
      - perception_halfspace_normal.z() * active_local_frame.origin_z;
  const int constrained_knots =
      TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS < NHORIZON
          ? TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS : NHORIZON;
  for (int k = 0; k < constrained_knots; ++k) {
    tiny_SetKinematicHalfspace(
        &work, k, 0, &normal_local, &zero_velocity,
        boundary_local, perception_halfspace_penalty, 1);
  }
}

static void armPostClearHalfspace(const state_t *state, int8_t evasion_side) {
  tiny_ClearPositionHalfspaces(&work);
  const struct vec rpy = quat2rpy(qnormalize(attitude));
  const float left_x = -sinf(rpy.z);
  const float left_y = cosf(rpy.z);
  const float normal_sign = evasion_side > 0 ? 1.0f : -1.0f;
  perception_halfspace_normal = Eigen::Vector3f(
      normal_sign * left_x, normal_sign * left_y, 0.0f);
  perception_halfspace_boundary =
      perception_halfspace_normal(0) * state->position.x +
      perception_halfspace_normal(1) * state->position.y;
  perception_halfspace_active = true;
  perception_boundary_index = 4;
  perception_post_clear_deadline =
      xTaskGetTickCount() + pdMS_TO_TICKS(perception_post_clear_hold_ms);
}

static void resetPerceptionAverage(void) {
  memset(perception_danger_history, 0, sizeof(perception_danger_history));
  memset(perception_safe_history, 0, sizeof(perception_safe_history));
  memset(perception_open_sum, 0, sizeof(perception_open_sum));
  perception_history_next = 0;
  perception_history_count = 0;
  perception_danger_sum = 0;
  perception_danger_average = 0.0f;
  perception_last_sample = 0;
  perception_remembered_safe_side = 0;
}

static void updatePerceptionAverage(uint8_t safe_mask) {
  if (perception_history_count == perception_average_window) {
    perception_danger_sum -=
        perception_danger_history[perception_history_next];
    const uint8_t old_safe_mask =
        perception_safe_history[perception_history_next];
    for (int direction = 0; direction < SEQUENTIAL_OBSTACLE_DIRECTIONS;
         ++direction) {
      perception_open_sum[direction] -=
          (old_safe_mask >> direction) & 1u;
    }
  } else {
    perception_history_count++;
  }

  uint8_t safe_count = 0;
  for (int direction = 0; direction < SEQUENTIAL_OBSTACLE_DIRECTIONS;
       ++direction) {
    const uint8_t is_safe = (safe_mask >> direction) & 1u;
    safe_count += is_safe;
    perception_open_sum[direction] += is_safe;
  }
  const uint8_t dangerous_count =
      SEQUENTIAL_OBSTACLE_DIRECTIONS - safe_count;
  perception_danger_history[perception_history_next] = dangerous_count;
  perception_safe_history[perception_history_next] = safe_mask;
  perception_danger_sum += dangerous_count;
  perception_history_next =
      (perception_history_next + 1u) % perception_average_window;
  perception_danger_average = (float)perception_danger_sum /
      (float)perception_history_count;
}

static void updatePerceptionHalfspace(const state_t *state) {
  float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS];
  uint32_t age_ms = 0;
  uint32_t sample = 0;

  if (!sequentialObstacleLinkGetLatest(clearance_m, &age_ms, &sample) ||
      age_ms > perception_max_age_ms) {
    perception_safe_mask = 0;
    clearPerceptionHalfspace();
    resetPerceptionAverage();
    return;
  }

  perception_safe_mask = 0;
  int safe_count = 0;
  for (int direction = 0; direction < SEQUENTIAL_OBSTACLE_DIRECTIONS;
       ++direction) {
    if (clearance_m[direction] >= perception_safe_min_m) {
      perception_safe_mask |= (uint8_t)(1u << direction);
      safe_count++;
    }
  }
  if (sample != perception_last_sample) {
    perception_last_sample = sample;
    updatePerceptionAverage(perception_safe_mask);
    const uint8_t right_open =
        perception_open_sum[0] + perception_open_sum[1];
    const uint8_t left_open =
        perception_open_sum[2] + perception_open_sum[3];
    if (right_open > left_open) {
      perception_remembered_safe_side = 1;
    } else if (left_open > right_open) {
      perception_remembered_safe_side = -1;
    }
  }
  if (perception_history_count < perception_average_window) {
    return;
  }
  if (perception_halfspace_active && perception_boundary_index == 4) {
    const bool post_clear_hold_valid =
        (int32_t)(xTaskGetTickCount() - perception_post_clear_deadline) < 0;
    if (post_clear_hold_valid &&
        perception_danger_average <= perception_trigger_average) {
      return;
    }
    // A post-clear side-retention plane must never mask a newly observed
    // obstacle. Remove it and process the current frame below so a stopping
    // or lateral avoidance plane is installed in this same MPC update.
    clearPerceptionHalfspace();
  }
  if (perception_halfspace_active &&
      perception_danger_average < perception_clear_average) {
    if (perception_boundary_index >= 0 &&
        perception_boundary_index < 3 && perception_evasion_side != 0) {
      armPostClearHalfspace(state, perception_evasion_side);
      return;
    }
    clearPerceptionHalfspace();
    return;
  }
  if (!perception_halfspace_active &&
      perception_danger_average <= perception_trigger_average) {
    return;
  }
  const bool all_directions_blocked = safe_count == 0;
  uint8_t all_blocked_votes = 0;
  for (int history = 0; history < perception_history_count; ++history) {
    all_blocked_votes += perception_safe_history[history] == 0;
  }
  int selected_boundary = -1;
  int selected_right_minus_left = 0;
  if (perception_halfspace_active) {
    if (perception_boundary_index == 3) {
      if (all_directions_blocked) {
        if (perception_remembered_safe_side == 0) {
          return;
        }
        selected_right_minus_left = perception_remembered_safe_side;
        selected_boundary = selected_right_minus_left > 0 ? 0 : 2;
        clearPerceptionHalfspace();
      } else {
        const uint8_t right_open_votes = perception_open_sum[0];
        const uint8_t left_open_votes = perception_open_sum[3];
        if (right_open_votes < perception_side_open_votes &&
            left_open_votes < perception_side_open_votes) {
          return;
        }
        if (right_open_votes > left_open_votes) {
          selected_boundary = 0;
          selected_right_minus_left = 1;
        } else {
          selected_boundary = 2;
          selected_right_minus_left = -1;
        }
        clearPerceptionHalfspace();
      }
    } else if (!all_directions_blocked ||
               all_blocked_votes < perception_all_blocked_votes) {
      return;
    } else if (perception_remembered_safe_side != 0 &&
               perception_evasion_side == perception_remembered_safe_side) {
      // This lateral plane was selected from the last directional evidence
      // before the view became fully blocked. Keep following that remembered
      // escape instead of alternating between lateral and stop planes.
      return;
    } else {
      // Escalate a latched lateral plane only after the entire camera view
      // has been blocked persistently, rather than on one noisy frame.
      clearPerceptionHalfspace();
    }
  }

  if (!all_directions_blocked && selected_boundary < 0) {
    int selected_imbalance = 0;
    selected_right_minus_left = 0;
    for (int boundary = 0; boundary < 3; ++boundary) {
      int safe_right = 0;
      for (int direction = 0; direction <= boundary; ++direction) {
        safe_right += perception_open_sum[direction];
      }
      int safe_left = 0;
      for (int direction = boundary + 1;
           direction < SEQUENTIAL_OBSTACLE_DIRECTIONS; ++direction) {
        safe_left += perception_open_sum[direction];
      }
      const int right_minus_left = safe_right - safe_left;
      const int imbalance = right_minus_left < 0
          ? -right_minus_left : right_minus_left;
      if (imbalance > selected_imbalance) {
        selected_boundary = boundary;
        selected_imbalance = imbalance;
        selected_right_minus_left = right_minus_left;
      }
    }
  }
  if (!all_directions_blocked && selected_boundary < 0) {
    // A split safe mask has no preferred side. Use the same deterministic
    // left escape as an all-blocked frame.
    selected_boundary = 2;
    selected_right_minus_left = -1;
  }

  const struct vec rpy = quat2rpy(qnormalize(attitude));
  float normal_angle = rpy.z;
  if (selected_boundary >= 0) {
    normal_angle +=
        radians(perception_boundary_angle_deg[selected_boundary] + 90.0f);
    // n.p <= b selects the right side. Reverse n when the left side is safer.
    if (selected_right_minus_left < 0) {
      normal_angle += radians(180.0f);
    }
  }
  perception_halfspace_normal =
      Eigen::Vector3f(cosf(normal_angle), sinf(normal_angle), 0.0f);
  perception_halfspace_boundary =
      perception_halfspace_normal(0) * state->position.x +
      perception_halfspace_normal(1) * state->position.y;

  perception_halfspace_active = true;
  perception_boundary_index = selected_boundary >= 0
      ? (int8_t)selected_boundary : 3;
  if (selected_boundary >= 0) {
    perception_evasion_side = selected_right_minus_left > 0 ? 1 : -1;
  }
}

static float projectPublishedPlanToPerceptionHalfspace(
    const state_t *state) {
  if (!perception_halfspace_active) {
    return 0.0f;
  }
  const int constrained_knots =
      TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS < NHORIZON
          ? TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS : NHORIZON;
  const Eigen::Vector3f current_position(
      state->position.x, state->position.y, state->position.z);
  const float current_safe_distance = perception_halfspace_boundary -
      perception_halfspace_normal.dot(current_position);
  const bool project_velocity =
      current_safe_distance <= perception_velocity_projection_margin_m;
  float maximum_projection = 0.0f;
  for (int k = 0; k < constrained_knots; ++k) {
    const float violation =
        perception_halfspace_normal.dot(published_Xhrz[k].head(3)) -
        perception_halfspace_boundary;
    if (violation > 0.0f) {
      published_Xhrz[k].head(3) -=
          violation * perception_halfspace_normal;
      maximum_projection = T_MAX(maximum_projection, violation);
    }

    // The cascaded PID consumes both the projected position and the MPC
    // velocity feedforward. Do not allow that feedforward to carry the
    // vehicle through a plane whose position target has just been projected
    // onto the safe side. Preserve all tangential velocity.
    if (project_velocity) {
      Eigen::Vector3f published_velocity = published_Xhrz[k].segment<3>(6);
      const float forbidden_velocity =
          perception_halfspace_normal.dot(published_velocity);
      if (forbidden_velocity > 0.0f) {
        published_Xhrz[k].segment<3>(6) -=
            forbidden_velocity * perception_halfspace_normal;
      }
    }
  }
  if (maximum_projection > perception_max_plan_projection_m) {
    for (int k = 0; k < NHORIZON; ++k) {
      published_Xhrz[k](0) = state->position.x;
      published_Xhrz[k](1) = state->position.y;
      published_Xhrz[k](2) = state->position.z;
    }
  }
  return maximum_projection;
}

static int lastPidTrackingKnot(void) {
  int knot = NHORIZON - 1;
  const int constrained_knots =
      TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS < NHORIZON
          ? TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS : NHORIZON;
  if (constrained_knots > 0 && knot >= constrained_knots) {
    knot = constrained_knots - 1;
  }
  return knot;
}

static void updatePidSetpointFromPlan(const uint32_t tick) {
  const float elapsed_s =
      (float)(tick - plan_start_tick) / (float)RATE_MAIN_LOOP;
  float knot_position = elapsed_s / DT;
  knot_position += (float)published_pid_lookahead_knots;
  const int last_knot = lastPidTrackingKnot();
  if (knot_position < 0.0f) {
    knot_position = 0.0f;
  }
  if (knot_position > (float)last_knot) {
    knot_position = (float)last_knot;
  }

  const int lower_knot = (int)floorf(knot_position);
  const int upper_knot =
      lower_knot < last_knot ? lower_knot + 1 : lower_knot;
  const float alpha = knot_position - (float)lower_knot;

  memset(&mpc_setpoint_pid, 0, sizeof(mpc_setpoint_pid));
  mpc_setpoint_pid.mode.x = modeAbs;
  mpc_setpoint_pid.mode.y = modeAbs;
  mpc_setpoint_pid.mode.z = modeAbs;
  mpc_setpoint_pid.mode.yaw = modeAbs;
  mpc_setpoint_pid.position.x =
      (1.0f - alpha) * published_Xhrz[lower_knot](0) + alpha * published_Xhrz[upper_knot](0);
  mpc_setpoint_pid.position.y =
      (1.0f - alpha) * published_Xhrz[lower_knot](1) + alpha * published_Xhrz[upper_knot](1);
  mpc_setpoint_pid.position.z =
      (1.0f - alpha) * published_Xhrz[lower_knot](2) + alpha * published_Xhrz[upper_knot](2);
  mpc_setpoint_pid.velocity.x =
      (1.0f - alpha) * published_Xhrz[lower_knot](6) + alpha * published_Xhrz[upper_knot](6);
  mpc_setpoint_pid.velocity.y =
      (1.0f - alpha) * published_Xhrz[lower_knot](7) + alpha * published_Xhrz[upper_knot](7);
  mpc_setpoint_pid.velocity.z =
      (1.0f - alpha) * published_Xhrz[lower_knot](8) + alpha * published_Xhrz[upper_knot](8);

  const float yaw_unwrapped_rad =
      (1.0f - alpha) * published_yaw_unwrapped_rad[lower_knot]
      + alpha * published_yaw_unwrapped_rad[upper_knot];
  mpc_setpoint_pid.attitude.yaw =
      remainderf(degrees(yaw_unwrapped_rad), 360.0f);
}

static void holdCurrentPose(setpoint_t *hold, const state_t *state) {
  memset(hold, 0, sizeof(*hold));
  hold->mode.x = modeAbs;
  hold->mode.y = modeAbs;
  hold->mode.z = modeAbs;
  hold->mode.yaw = modeAbs;
  hold->position.x = state->position.x;
  hold->position.y = state->position.y;
  hold->position.z = state->position.z;
  hold->attitude.yaw = state->attitude.yaw;
}

static bool planIsPublishable(
    int *bad_knot, int *bad_state, float *bad_value) {
  static const float max_position_change_m = 5.0f;
  static const float max_velocity_mps = 5.0f;

  for (int k = 0; k < NHORIZON; ++k) {
    for (int j = 0; j < NSTATES; ++j) {
      const float value = Xhrz[k](j);
      if (!std::isfinite(value)) {
        *bad_knot = k;
        *bad_state = j;
        *bad_value = value;
        return false;
      }
    }
    for (int j = 0; j < 3; ++j) {
      if (fabsf(Xhrz[k](j)) > max_position_change_m) {
        *bad_knot = k;
        *bad_state = j;
        *bad_value = Xhrz[k](j);
        return false;
      }
      if (fabsf(Xhrz[k](6 + j)) > max_velocity_mps) {
        *bad_knot = k;
        *bad_state = 6 + j;
        *bad_value = Xhrz[k](6 + j);
        return false;
      }
    }
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    for (int j = 0; j < NINPUTS; ++j) {
      if (!std::isfinite(Uhrz[k](j))) {
        *bad_knot = k;
        *bad_state = NSTATES + j;
        *bad_value = Uhrz[k](j);
        return false;
      }
    }
  }
  return true;
}

static VectorNf localPlanStateToWorld(const VectorNf& state_local) {
  VectorNf state_world = state_local;
  const Eigen::Vector3f position_world = localVectorToWorld(
      active_local_frame,
      Eigen::Vector3f(state_local(0), state_local(1), state_local(2)))
      + Eigen::Vector3f(
          active_local_frame.origin_x,
          active_local_frame.origin_y,
          active_local_frame.origin_z);
  const Eigen::Vector3f velocity_world = localVectorToWorld(
      active_local_frame,
      Eigen::Vector3f(state_local(6), state_local(7), state_local(8)));
  state_world.head(3) = position_world;
  state_world.segment<3>(6) = velocity_world;
  return state_world;
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
      perception_halfspace_active = false;
      perception_boundary_index = -1;
      perception_safe_mask = 0;
      perception_evasion_side = 0;
      perception_post_clear_deadline = 0;
      tiny_ClearPositionHalfspaces(&work);
      resetPerceptionAverage();
    }

    updateInitialState(&sensors_task, &state_task);
    updateHorizonReference(&setpoint_task);
    resetStateWarmStartInActiveFrame();
    updatePerceptionHalfspace(&state_task);
    if (perception_halfspace_active) {
      applyPerceptionHalfspace();
    }

    tiny_UpdateLinearCost(&work);
    const uint64_t solve_start_us = usecTimestamp();
    tiny_SolveAdmm(&work);
    const uint32_t solve_us = (uint32_t)(usecTimestamp() - solve_start_us);
    result = info.status_val * info.iter;

    int continuity_knot = (int)ceilf((1.0f / (float)MPC_RATE) / DT);
    const int last_tracking_knot = lastPidTrackingKnot();
    if (continuity_knot < 1) {
      continuity_knot = 1;
    }
    if (continuity_knot > last_tracking_knot) {
      continuity_knot = last_tracking_knot;
    }

    int bad_knot = -1;
    int bad_state = -1;
    float bad_value = 0.0f;
    const bool plan_publishable =
        planIsPublishable(&bad_knot, &bad_state, &bad_value);
    float published_projection_m = 0.0f;
    if (plan_publishable) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      const float previous_x = mpc_has_run
          ? mpc_setpoint_pid.position.x : state_task.position.x;
      const float previous_y = mpc_has_run
          ? mpc_setpoint_pid.position.y : state_task.position.y;
      const float previous_z = mpc_has_run
          ? mpc_setpoint_pid.position.z : state_task.position.z;
      for (int k = 0; k < NHORIZON; ++k) {
        published_Xhrz[k] = localPlanStateToWorld(Xhrz[k]);
        published_yaw_unwrapped_rad[k] = reference_yaw_unwrapped_rad[k];
      }
      if (continuity_knot == 0) {
        published_Xhrz[0](0) = previous_x;
        published_Xhrz[0](1) = previous_y;
        published_Xhrz[0](2) = previous_z;
      } else {
        for (int k = 0; k <= continuity_knot; ++k) {
          const float blend = (float)k / (float)continuity_knot;
          published_Xhrz[k](0) =
              (1.0f - blend) * previous_x
              + blend * published_Xhrz[continuity_knot](0);
          published_Xhrz[k](1) =
              (1.0f - blend) * previous_y
              + blend * published_Xhrz[continuity_knot](1);
          published_Xhrz[k](2) =
              (1.0f - blend) * previous_z
              + blend * published_Xhrz[continuity_knot](2);
        }
      }
      published_projection_m =
          projectPublishedPlanToPerceptionHalfspace(&state_task);
      published_pid_lookahead_knots = perception_halfspace_active
          ? perception_pid_lookahead_knots : 0;
      plan_start_tick = solve_tick;
      mpc_has_run = true;
      xSemaphoreGive(dataMutex);
    } else {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      published_projection_m =
          projectPublishedPlanToPerceptionHalfspace(&state_task);
      published_pid_lookahead_knots = perception_halfspace_active
          ? perception_pid_lookahead_knots : 0;
      xSemaphoreGive(dataMutex);
      DEBUG_PRINT(
          "MPC: rejected plan knot=%d state=%d value=%.3g; retaining prior plan\n",
          bad_knot, bad_state, (double)bad_value);
    }

    {
      float max_primal_violation = -1000000.0f;
      float max_aux_violation = -1000000.0f;
      float max_published_violation = -1000000.0f;
      float max_halfspace_slack = 0.0f;
      float max_primal_aux_gap = 0.0f;
      for (int k = 0; k <= continuity_knot; ++k) {
        const float primal_violation =
            data.a_pos_hs[k][0].dot(Xhrz[k].head(3)) - data.b_hs[k][0];
        const float aux_violation =
            data.a_pos_hs[k][0].dot(ZX_new[k].head(3)) - data.b_hs[k][0];
        const float published_violation =
            perception_halfspace_normal.dot(published_Xhrz[k].head(3)) -
            perception_halfspace_boundary;
        const float primal_aux_gap =
            (Xhrz[k].head(3) - ZX_new[k].head(3)).cwiseAbs().maxCoeff();
        max_primal_violation = T_MAX(max_primal_violation, primal_violation);
        max_aux_violation = T_MAX(max_aux_violation, aux_violation);
        max_published_violation =
            T_MAX(max_published_violation, published_violation);
        max_halfspace_slack =
            T_MAX(max_halfspace_slack, data.slack_used_hs[k][0]);
        max_primal_aux_gap = T_MAX(max_primal_aux_gap, primal_aux_gap);
      }
      DEBUG_PRINT("MPC: pos=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f)\n",
                  (double)x0(0), (double)x0(1), (double)x0(2),
                  (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2));
      DEBUG_PRINT("MPC: iter=%d solve=%luus\n",
                  info.iter, (unsigned long)solve_us);
      DEBUG_PRINT(
          "MPC: vision mask=%u avg=%.2f n=%u side=%d plane=%d active=%d corr=%.3f hs[0:%d] primal=%.3f aux=%.3f pub=%.3f slack=%.3f gap=%.3f accepted=%d\n",
          (unsigned int)perception_safe_mask,
          (double)perception_danger_average,
          (unsigned int)perception_history_count,
          perception_remembered_safe_side,
          perception_boundary_index,
          perception_halfspace_active ? 1 : 0,
          (double)published_projection_m,
          continuity_knot,
          (double)max_primal_violation,
          (double)max_aux_violation,
          (double)max_published_violation,
          (double)max_halfspace_slack,
          (double)max_primal_aux_gap,
          plan_publishable ? 1 : 0);
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
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
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
  stgs.en_cstr_states = 1;  // Constraints active
  stgs.max_iter = 5;
  stgs.iters_check_rho_update = 0;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  /* End of MPC initialization */  
  step = 0;
  perception_halfspace_active = false;
  perception_boundary_index = -1;
  perception_safe_mask = 0;
  perception_evasion_side = 0;
  perception_post_clear_deadline = 0;
  resetPerceptionAverage();
  mpc_has_run = false;
  last_controller_tick = 0;
  plan_start_tick = 0;
  published_pid_lookahead_knots = 0;
  motors_were_allowed = false;
  controllerPidInit();

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
  for (int k = 0; k < NHORIZON; ++k) {
    published_Xhrz[k].setZero();
    published_yaw_unwrapped_rad[k] = 0.0f;
  }
  planner_reset_requested = true;
  xSemaphoreGive(dataMutex);
  
  if (en_traj) {
    DEBUG_PRINT("Stored trajectory enabled\n");
  } else {
    DEBUG_PRINT("Commander/setpoint mode enabled\n");
  }
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
    if (mpc_has_run) {
      updatePidSetpointFromPlan(tick);
    }
    has_run_snapshot = mpc_has_run;
    xSemaphoreGive(dataMutex);
  }
  motors_were_allowed = motors_allowed;

  /* Output control: TinyMPC plans; the stock cascaded PID stabilizes. */
  setpoint_t hold_setpoint;
  const setpoint_t *pid_setpoint = &mpc_setpoint_pid;
  if (!has_run_snapshot) {
    holdCurrentPose(&hold_setpoint, state);
    pid_setpoint = &hold_setpoint;
  }
  positionControllerSetVelocityFeedforward(pid_setpoint == &mpc_setpoint_pid);
  controllerPid(control, pid_setpoint, sensors, state, tick);
}

/**
 * Tunning variables for the full state quaternion LQR controller
 */
// PARAM_GROUP_START(ctrlMPC)
// /**
//  * @brief K gain
//  */
// PARAM_ADD(PARAM_FLOAT, u_hover, &u_hover)

// PARAM_GROUP_STOP(ctrlMPC)

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

// Note: LOG macros disabled due to C++ string literal compatibility with new firmware
/*
LOG_GROUP_START(ctrlMPC)

LOG_ADD(LOG_INT8, result, &result)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

LOG_ADD(LOG_FLOAT, zu0, &(ZU_new[0](0)))
LOG_ADD(LOG_FLOAT, zu1, &(ZU_new[0](1)))
LOG_ADD(LOG_FLOAT, zu2, &(ZU_new[0](2)))
LOG_ADD(LOG_FLOAT, zu3, &(ZU_new[0](3)))

LOG_GROUP_STOP(ctrlMPC)
*/

#ifdef __cplusplus
}
#endif
