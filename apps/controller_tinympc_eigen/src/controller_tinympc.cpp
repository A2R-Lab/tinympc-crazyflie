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
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "supervisor.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (10 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC ADMM"
#define TINYMPC_TASK_PRI              1

// Rodriguez parameters conversion function (needed for old firmware compatibility)
static inline struct vec quat2rp(struct quat q) {
  struct vec v;
  float w_abs = fabsf(q.w);
  if (w_abs > 1e-6f) { // Avoid division by near-zero
    v.x = q.x / q.w;
    v.y = q.y / q.w;
    v.z = q.z / q.w;
  } else {
    // Handle singular case
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
  }
  return v;
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
#include "trajectories/5hz/traj_straight_5hz.h"
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

// A world-fixed camera-style boundary captured on the first MPC update. In a
// top-down view, its boundary line extends 45 degrees left of camera-forward
// through the drone; the selected half-space keeps the plan left of that line.
static const float camera_halfspace_angle_deg = 45.0f;
static const float camera_halfspace_penalty = 500.0f;
static bool camera_halfspace_armed = false;
static Eigen::Vector3f camera_halfspace_normal = Eigen::Vector3f::Zero();
static float camera_halfspace_boundary = 0.0f;

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
static const uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static uint32_t traj_idx = 0;

static_assert(
    TRAJECTORY_SAMPLE_RATE_HZ == TINYMPC_GENERATED_SOLVE_RATE_HZ,
    "trajectory and MPC solve rates must match");
static_assert(TRAJECTORY_STATE_DIM == NSTATES, "trajectory state dimension mismatch");

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

// TinyMPC plans a dynamically feasible position path. The stock cascaded PID
// tracks a near-term point from that path and owns attitude, rate, and motor control.
static setpoint_t mpc_setpoint_pid;
static bool mpc_has_run = false;
static uint32_t last_controller_tick = 0;
static uint32_t plan_start_tick = 0;
static bool motors_were_allowed = false;
static float pid_yaw_reference_deg = 0.0f;
static VectorNf published_Xhrz[NHORIZON];
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

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->position.z;
  // Body velocity error, [m/s]                          
  x0(6) = state->velocity.x;
  x0(7) = state->velocity.y;
  x0(8) = state->velocity.z;
  // Angular rate error, [rad/s]
  x0(9)  = radians(sensors->gyro.x);   
  x0(10) = radians(sensors->gyro.y);
  x0(11) = radians(sensors->gyro.z);
  attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude
  phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
  // Attitude error
  x0(3) = phi.x;
  x0(4) = phi.y;
  x0(5) = phi.z;
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    const float current_time_s = (float)step / (float)MPC_RATE;
    traj_idx = (uint32_t)floorf(current_time_s / TRAJECTORY_SAMPLE_DT_S);
    if (traj_idx >= traj_length) {
      traj_idx = traj_length - 1;
    }
    for (int i = 0; i < NHORIZON; ++i) {
      const float reference_time_s = current_time_s + (float)i * DT;
      float sample_position = reference_time_s / TRAJECTORY_SAMPLE_DT_S;
      if (sample_position > (float)(traj_length - 1)) {
        sample_position = (float)(traj_length - 1);
      }
      const uint32_t lower_idx = (uint32_t)floorf(sample_position);
      const uint32_t upper_idx =
          lower_idx + 1 < traj_length ? lower_idx + 1 : lower_idx;
      const float alpha = sample_position - (float)lower_idx;
      for (int j = 0; j < NSTATES; ++j) {
        Xref[i](j) = (1.0f - alpha) * X_ref_data[lower_idx][j]
                     + alpha * X_ref_data[upper_idx][j];
      }
      if (i < NHORIZON - 1) {
        Uref[i].setZero();
      }
    }
  }
  else {
    xg(0)  = setpoint->position.x;
    xg(1)  = setpoint->position.y;
    xg(2)  = setpoint->position.z;
    xg(6)  = setpoint->velocity.x;
    xg(7)  = setpoint->velocity.y;
    xg(8)  = setpoint->velocity.z;
    xg(9)  = radians(setpoint->attitudeRate.roll);
    xg(10) = radians(setpoint->attitudeRate.pitch);
    xg(11) = radians(setpoint->attitudeRate.yaw);
    desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                        radians(setpoint->attitude.pitch), 
                        radians(setpoint->attitude.yaw));
    attitude = rpy2quat(desired_rpy);
    phi = quat2rp(qnormalize(attitude));  
    xg(3) = phi.x;
    xg(4) = phi.y;
    xg(5) = phi.z;
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
    // // xg(1) = 1.0;
    // // xg(2) = 2.0;
  }
  if (en_traj && (float)step / (float)MPC_RATE < TRAJECTORY_DURATION_S) {
    step += 1;
  }
}

static void armCameraHalfspace(const state_t *state) {
  const struct vec rpy = quat2rpy(qnormalize(attitude));
  const float normal_angle = rpy.z - radians(camera_halfspace_angle_deg);
  camera_halfspace_normal =
      Eigen::Vector3f(cosf(normal_angle), sinf(normal_angle), 0.0f);
  camera_halfspace_boundary =
      camera_halfspace_normal(0) * state->position.x +
      camera_halfspace_normal(1) * state->position.y +
      camera_halfspace_normal(2) * state->position.z;

  tiny_ClearPositionHalfspaces(&work);
  const Eigen::Vector3f zero_velocity = Eigen::Vector3f::Zero();
  const int constrained_knots =
      TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS < NHORIZON
          ? TINYMPC_GENERATED_CONSTRAINED_HORIZON_KNOTS : NHORIZON;
  for (int k = 0; k < constrained_knots; ++k) {
    tiny_SetKinematicHalfspace(
        &work, k, 0, &camera_halfspace_normal, &zero_velocity,
        camera_halfspace_boundary, camera_halfspace_penalty, 1);
  }

  camera_halfspace_armed = true;
  DEBUG_PRINT("Camera half-space armed: n=(%.2f,%.2f) b=%.2f penalty=%.1f\n",
              (double)camera_halfspace_normal(0),
              (double)camera_halfspace_normal(1),
              (double)camera_halfspace_boundary,
              (double)camera_halfspace_penalty);
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

  mpc_setpoint_pid.attitude.yaw = pid_yaw_reference_deg;
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
    const state_t *state, int *bad_knot, int *bad_state, float *bad_value) {
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
    const float position[3] = {
        state->position.x, state->position.y, state->position.z};
    for (int j = 0; j < 3; ++j) {
      if (fabsf(Xhrz[k](j) - position[j]) > max_position_change_m) {
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
      traj_idx = 0;
      camera_halfspace_armed = false;
      tiny_ClearPositionHalfspaces(&work);
    }

    updateInitialState(&sensors_task, &state_task);
    updateHorizonReference(&setpoint_task);
    if (!camera_halfspace_armed) {
      armCameraHalfspace(&state_task);
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
        planIsPublishable(&state_task, &bad_knot, &bad_state, &bad_value);
    if (plan_publishable) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      const float previous_x = mpc_has_run
          ? mpc_setpoint_pid.position.x : state_task.position.x;
      const float previous_y = mpc_has_run
          ? mpc_setpoint_pid.position.y : state_task.position.y;
      const float previous_z = mpc_has_run
          ? mpc_setpoint_pid.position.z : state_task.position.z;
      for (int k = 0; k < NHORIZON; ++k) {
        published_Xhrz[k] = Xhrz[k];
      }
      if (continuity_knot == 0) {
        published_Xhrz[0](0) = previous_x;
        published_Xhrz[0](1) = previous_y;
        published_Xhrz[0](2) = previous_z;
      } else {
        for (int k = 0; k <= continuity_knot; ++k) {
          const float blend = (float)k / (float)continuity_knot;
          published_Xhrz[k](0) =
              (1.0f - blend) * previous_x + blend * Xhrz[continuity_knot](0);
          published_Xhrz[k](1) =
              (1.0f - blend) * previous_y + blend * Xhrz[continuity_knot](1);
          published_Xhrz[k](2) =
              (1.0f - blend) * previous_z + blend * Xhrz[continuity_knot](2);
        }
      }
      plan_start_tick = solve_tick;
      mpc_has_run = true;
      xSemaphoreGive(dataMutex);
    } else {
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
            camera_halfspace_normal.dot(Xhrz[k].head(3)) -
            camera_halfspace_boundary;
        const float aux_violation =
            camera_halfspace_normal.dot(ZX_new[k].head(3)) -
            camera_halfspace_boundary;
        const float published_violation =
            camera_halfspace_normal.dot(published_Xhrz[k].head(3)) -
            camera_halfspace_boundary;
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
          "MPC: hs[0:%d] primal=%.3f aux=%.3f pub=%.3f slack=%.3f gap=%.3f accepted=%d\n",
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
  stgs.max_iter = 10;
  stgs.iters_check_rho_update = 0;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  /* End of MPC initialization */  
  step = 0;
  traj_idx = 0;
  camera_halfspace_armed = false;
  mpc_has_run = false;
  last_controller_tick = 0;
  plan_start_tick = 0;
  motors_were_allowed = false;
  pid_yaw_reference_deg = 0.0f;
  controllerPidInit();

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
    if (controller_reactivated) {
      pid_yaw_reference_deg = state->attitude.yaw;
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
