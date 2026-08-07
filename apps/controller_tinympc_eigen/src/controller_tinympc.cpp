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

/* Include trajectory to track */
// #include "trajectories/50hz/traj_straight_50hz.h"
#include "trajectories/50hz/traj_circle_50hz.h"
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
static const TinyRacerRaceConfig race_config = {
  0.30f, 0.0f, 0.10f, 0.05f, 0.35f, 0.35f, 0.25f, 250,
  0.35f, 0.70f, 3
};
typedef struct {
  Eigen::Vector3f normal_world;
  float boundary_world;
} PerceptionPlane;
static PerceptionPlane perception_stop_plane;
static PerceptionPlane perception_recovery_plane;
static bool perception_recovery_active = false;
static Eigen::Vector3f avoidance_left_world = Eigen::Vector3f::Zero();
static Eigen::Vector3f avoidance_start_world = Eigen::Vector3f::Zero();
static TinyRacerRaceState race_state;
static TinyRacerRaceIntent race_intent;

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
static bool takeoff_complete = false;
static uint16_t takeoff_stable_samples = 0;

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(TRAJECTORY_REFERENCE_DIM == 13, "trajectory reference dimension mismatch");

static struct quat attitude;
static MpcLocalFrame active_local_frame;
static float reference_yaw_unwrapped_rad[NHORIZON];
static float reference_yaw_phase_rad = 0.0f;

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

void updateHorizonReference(const setpoint_t *setpoint, bool advance) {
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
  if (advance && en_traj &&
      (float)step / (float)MPC_RATE < TRAJECTORY_DURATION_S) {
    step += 1;
  }
}

static void applyRaceIntent(void) {
  const Eigen::Vector3f velocity_normal = Eigen::Vector3f::Zero();
  const Eigen::Vector3f lateral_local = worldVectorToLocal(
      active_local_frame, avoidance_left_world);
  for (int k = 0; k < NHORIZON; ++k) {
    Xref[k].head(3) += lateral_local * race_intent.lateral_offset_m *
        ((float)k / (float)(NHORIZON - 1));
  }

  const PerceptionPlane *plane = race_intent.constraint_active
      ? &perception_stop_plane
      : (perception_recovery_active ? &perception_recovery_plane : NULL);
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

static void updateRaceIntent(const state_t *state) {
  TinyRacerPerceptionObservation observation = {};
  sequentialObstacleLinkGetLatest(&observation);
  const bool was_active = race_intent.constraint_active;
  const bool continuing_encounter = race_intent.mode == TINYRACER_RACE_RECOVER;
  const Eigen::Vector3f position_world(
      state->position.x, state->position.y, state->position.z);
  const Eigen::Vector3f velocity_world(
      state->velocity.x, state->velocity.y, state->velocity.z);
  const float lateral_displacement = (float)race_intent.pass_side *
      avoidance_left_world.dot(position_world - avoidance_start_world);
  const bool release_ready = was_active &&
      fabsf(avoidance_left_world.dot(velocity_world)) < 0.10f &&
      lateral_displacement >= race_config.bypass_offset_m;
  tinyRacerRaceUpdate(
      &race_state, &observation, &race_config, DT, release_ready,
      perception_recovery_active, &race_intent);
  if (perception_recovery_active && !race_intent.constraint_active &&
      race_state.clear_samples >= race_config.clear_samples_required &&
      perception_stop_plane.normal_world.dot(position_world) >=
          perception_stop_plane.boundary_world + perception_pass_distance_m) {
    perception_recovery_active = false;
    tiny_ClearPositionHalfspaces(&work);
    DEBUG_PRINT("Vision recovery plane cleared; returning to line\n");
  }
  if (was_active == race_intent.constraint_active) {
    return;
  }

  tiny_ClearPositionHalfspaces(&work);
  if (!race_intent.constraint_active) {
    const Eigen::Vector3f pass_direction =
        (float)race_intent.pass_side * avoidance_left_world;
    perception_recovery_plane.normal_world = -pass_direction;
    perception_recovery_plane.boundary_world =
        perception_recovery_plane.normal_world.dot(position_world);
    perception_recovery_active = true;
    race_state.clear_samples = 0;
    DEBUG_PRINT("Vision forward plane replaced by recovery side plane lateral=%.2f\n",
                (double)lateral_displacement);
    return;
  }

  const struct vec rpy = quat2rpy(qnormalize(attitude));
  perception_recovery_active = false;
  if (continuing_encounter) {
    perception_stop_plane.boundary_world =
        perception_stop_plane.normal_world.dot(position_world) +
        race_intent.stop_boundary_distance_m;
    DEBUG_PRINT("Vision blocked again; continuing pass=%s\n",
                race_intent.pass_side > 0 ? "left" : "right");
    return;
  }
  avoidance_left_world = Eigen::Vector3f(-sinf(rpy.z), cosf(rpy.z), 0.0f);
  avoidance_start_world = position_world;
  perception_stop_plane.normal_world = Eigen::Vector3f(
      cosf(rpy.z), sinf(rpy.z), 0.0f);
  perception_stop_plane.boundary_world =
      perception_stop_plane.normal_world.dot(position_world) +
      race_intent.stop_boundary_distance_m;
  DEBUG_PRINT("Vision blocked pass=%s clearances=(%.2f,%.2f,%.2f,%.2f)\n",
              race_intent.pass_side > 0 ? "left" : "right",
              (double)observation.clearance_m[0],
              (double)observation.clearance_m[1],
              (double)observation.clearance_m[2],
              (double)observation.clearance_m[3]);
}

static void updateTakeoffPhase(const state_t *state) {
  if (!en_traj || takeoff_complete) {
    takeoff_complete = true;
    return;
  }
  const uint32_t takeoff_end =
      (uint32_t)(TRAJECTORY_TAKEOFF_DURATION_S * MPC_RATE + 0.5f);
  if (step < takeoff_end || state->position.z < 0.32f ||
      fabsf(state->velocity.z) > 0.15f) {
    takeoff_stable_samples = 0;
    return;
  }
  if (++takeoff_stable_samples >= (uint16_t)(3 * MPC_RATE / 10)) {
    takeoff_complete = true;
    DEBUG_PRINT("Takeoff complete z=%.2f; vision enabled\n",
                (double)state->position.z);
  }
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
      takeoff_complete = !en_traj;
      takeoff_stable_samples = 0;
      tinyRacerRaceReset(&race_state);
      memset(&race_intent, 0, sizeof(race_intent));
      perception_recovery_active = false;
      tiny_ClearPositionHalfspaces(&work);
    }

    updateInitialState(&sensors_task, &state_task);
    // Intentionally preserve TinyMPC's state auxiliaries and duals.
    updateTakeoffPhase(&state_task);
    if (takeoff_complete) {
      updateRaceIntent(&state_task);
    }
    const uint32_t takeoff_end =
        (uint32_t)(TRAJECTORY_TAKEOFF_DURATION_S * MPC_RATE + 0.5f);
    updateHorizonReference(
        &setpoint_task,
        (step < takeoff_end || takeoff_complete) && !race_intent.pause_reference);
    applyRaceIntent();

    tiny_UpdateLinearCost(&work);
    const uint64_t solve_start_us = usecTimestamp();
    tiny_SolveAdmm(&work);
    const uint32_t solve_us = (uint32_t)(usecTimestamp() - solve_start_us);
    result = info.status_val * info.iter;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (int motor = 0; motor < NINPUTS; ++motor) {
      const float motor_thrust_newtons =
          tinympc_generated_physical_hover_thrust[motor] + Uhrz[0](motor);
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
  takeoff_complete = !en_traj;
  takeoff_stable_samples = 0;
  tinyRacerRaceReset(&race_state);
  memset(&race_intent, 0, sizeof(race_intent));
  perception_recovery_active = false;
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
