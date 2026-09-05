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

// Isolated 100 Hz MPC / 500 Hz LQR controller for CrazySim.

#include "Eigen.h"
#ifndef CONFIG_PLATFORM_SITL
#error "This controller copy is specialized for CrazySim; use the hardware workspace for flight"
#endif
#define TINYMPC_HOVER_ONLY 1
#define TINYMPC_LQR_FEEDBACK 1
#define TINYMPC_FRESH_HOVER_SNAPSHOT 1
#define TINYMPC_STAGE_TRACE 0

#ifndef TINYMPC_HOVER_HEIGHT_STEP_TEST
#define TINYMPC_HOVER_HEIGHT_STEP_TEST 0
#endif
#ifndef TINYMPC_HOVER_PITCH_STEP_TEST
#define TINYMPC_HOVER_PITCH_STEP_TEST 0
#endif
#ifndef TINYMPC_RIGID_ADMM_ITERATIONS
#define TINYMPC_RIGID_ADMM_ITERATIONS 2
#endif
static_assert(TINYMPC_RIGID_ADMM_ITERATIONS >= 1 && TINYMPC_RIGID_ADMM_ITERATIONS <= 5,
              "Rigid-body ADMM iteration budget must be between 1 and 5");
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <cmath>
#include "tinympc_hover_lqr_params.h"
#include "tinympc_hover_lqr_gain.h"
#include "tinympc_lqr_feedback.h"
#include "tinympc_lqr_nominal.h"
#include "tinympc_fallback_diag.h"
#define NHORIZON TINYMPC_GENERATED_HORIZON_KNOTS
using namespace Eigen;

#if defined(__cplusplus)
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include "tinympc_sitl_mmap_diag.h"

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
#include "estimator.h"
#include "attitude_controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "supervisor.h"
#include "power_distribution.h"
#include "tinympc_handoff.h"
#include "tinyracer_debug.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (10 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME             "TINYMPC ADMM"
// CRTP RX/TX and Kalman run at 2, but RX also blocks on port queues whose
// consumers (CRTP/platform services) run at 0. Share the lowest tier with
// those services so the entire receive path can progress during a solve.
// RTOS time slicing plus the cycle-end delay preserve service/idle progress.
#define TINYMPC_TASK_PRI              0

struct MpcLocalFrame {
  float origin_x;
  float origin_y;
  float origin_z;
  float yaw_world;
  float cos_yaw;
  float sin_yaw;
};

// All members belong to one solve generation and are published together.
struct TinyMpcLqrPlan {
  MpcLocalFrame frame;
  TinyMpcLqrNominal nominal;
};
static TinyMpcLqrPlan active_lqr_plan = {}, cached_lqr_plan = {};
static bool lqr_frame_initialized = false; // Worker-owned; reset with solver history.

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

static bool lqrMeasuredState(const MpcLocalFrame& frame, const sensorData_t* sensors,
                             const state_t* state, float measured[12]) {
  const Eigen::Vector3f position = worldVectorToLocal(frame, Eigen::Vector3f(
      state->position.x - frame.origin_x, state->position.y - frame.origin_y,
      state->position.z - frame.origin_z));
  const Eigen::Vector3f velocity = worldVectorToLocal(frame, Eigen::Vector3f(
      state->velocity.x, state->velocity.y, state->velocity.z));
  const struct quat measured_q = mkquat(state->attitudeQuaternion.x,
      state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  const float qnorm2 = measured_q.x * measured_q.x + measured_q.y * measured_q.y
      + measured_q.z * measured_q.z + measured_q.w * measured_q.w;
  if (!std::isfinite(qnorm2) || qnorm2 <= 1.0e-6f) return false;
  const struct vec rod = worldQuaternionToLocalRodrigues(frame, qnormalize(measured_q));
  const float packed[12] = {position.x(), position.y(), position.z(), rod.x, rod.y, rod.z,
      velocity.x(), velocity.y(), velocity.z(), radians(sensors->gyro.x),
      radians(sensors->gyro.y), radians(sensors->gyro.z)};
  for (int s = 0; s < 12; ++s) {
    if (!std::isfinite(packed[s])) return false;
  }
  memcpy(measured, packed, sizeof(packed));
  return true;
}

/* math3d's qqmul(q, p) stores the Hamilton product p*q. Put the maneuver
 * first so this returns the conventional world-yaw * maneuver composition:
 * the loop thrust direction then rotates with its world-frame path. */
// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

static void mpcStageBegin(const char *) {}
static void mpcStageEnd(const char *) {}

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
// Preserve the existing absolute safety deadline when raising release rate.
#define TINYMPC_DIRECT_COMMAND_MAX_AGE_MS 60U
static_assert(MPC_RATE == 100 && LQR_RATE == 500, "hover LQR requires 100/500 Hz");
static_assert(NHORIZON >= 7 && TINYMPC_GENERATED_MODEL_DT_S == 0.01f,
              "LQR nominal rollout requires six 10 ms input intervals");
#define TINYMPC_RATE_COMMAND_MAX_AGE_MS (3U * (1000U / MPC_RATE))

static_assert(NSTATES == TINYMPC_GENERATED_STATE_DIM, "generated state dimension mismatch");
static_assert(NINPUTS == TINYMPC_GENERATED_INPUT_DIM, "generated input dimension mismatch");
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
static VectorNf state_linear_cost[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;

// ADMM projected state and dual storage
static VectorNf ZX[NHORIZON];
static VectorNf ZX_new[NHORIZON];
static VectorNf YX[NHORIZON];

static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg;
static VectorMf ug;

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

static struct quat attitude;
static MpcLocalFrame active_local_frame;
static bool mpc_has_run = false;
static uint32_t last_controller_tick = 0;
static uint32_t plan_start_tick = 0;
static bool motors_were_allowed = false;
static TinyMpcFallbackDiag fallback_diag = {};
extern "C" uint8_t mpc_bench_run;
static float active_motor_commands[NINPUTS];
/* The 500 Hz callback must not replace a fresh MPC command with hover merely
 * because the worker holds dataMutex during a nonblocking read attempt.  This
 * callback-owned cache is refreshed only after a coherent mutex-protected
 * snapshot and remains subject to the normal command-age safety limit. */
static bool cached_motor_command_valid = false;
static uint32_t cached_motor_command_tick = 0u;
static float cached_motor_command[NINPUTS] = {0.0f};
static bool solve_tick_timing_initialized = false;
static uint32_t previous_solve_tick = 0u;
static uint32_t solve_tick_gap_min = UINT32_MAX;
static uint32_t solve_tick_gap_max = 0u;
static uint32_t solve_tick_gap_samples = 0u;
static uint32_t solve_tick_skipped_periods = 0u;
static SemaphoreHandle_t runTaskSemaphore = NULL;
static SemaphoreHandle_t dataMutex = NULL;
static StaticSemaphore_t dataMutexBuffer;
static bool mpc_initialized = false;
static bool selection_reset_pending = false;
static void initializeMpcController(void);
static setpoint_t planner_setpoint;
static sensorData_t planner_sensors;
static state_t planner_state;
static uint32_t planner_tick = 0;
static uint64_t planner_snapshot_us = 0;
static uint32_t planner_generation = 0;
static bool planner_reset_requested = false;
/*
 * Binary, append-only diagnostic stream. Initialization exclusively creates,
 * sizes, maps, and pre-touches the file before controller tasks start. Runtime
 * producers reserve one no-wrap mmap slot with a lock-free atomic and publish
 * checksum-protected records by release-committing the slot. There is no
 * diagnostic task, critical section, allocation, syscall, or runtime flush.
 *
 * Schema version 3, native little-endian CrazySim host representation.
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
  float local_frame[4]; /* world origin xyz and world yaw */
  float initial_state[12]; /* solver local-frame x0 */
  float horizon_reference[20][12];
  float horizon_primal_state[20][12];
  float horizon_projected_state[20][12];
  float horizon_primal_input[19][4];
  float horizon_projected_input[19][4];
} TinyMpcSitlDiagRecord;
static_assert(NHORIZON == 20, "SITL horizon schema requires 20 knots");
static_assert(NSTATES == 12, "SITL horizon schema requires 12 states");
static_assert(NINPUTS == 4, "SITL horizon schema requires four inputs");
static_assert(sizeof(TinyMpcSitlDiagRecord) == 3848u,
              "SITL diagnostic schema layout changed");

static constexpr uint32_t TINYMPC_SITL_DIAG_MAGIC = 0x544d5043u;
static constexpr uint16_t TINYMPC_SITL_DIAG_VERSION = 3u;
static constexpr uint32_t TINYMPC_SITL_DIAG_HEARTBEAT_TICKS = M2T(1000);
static_assert(TINYMPC_SITL_MMAP_RECORD_SIZE ==
                  sizeof(TinyMpcSitlDiagRecord),
              "mmap slot payload and v3 record sizes differ");
static_assert(TINYMPC_SITL_MMAP_FILE_SIZE == 63442944u,
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
      "MPCDIAG mmap schema=3 mode=taskless record=%u slot=%u capacity=%u bytes=%u path=%s\n",
      (unsigned int)sizeof(TinyMpcSitlDiagRecord),
      (unsigned int)TINYMPC_SITL_MMAP_SLOT_SIZE,
      (unsigned int)TINYMPC_SITL_MMAP_CAPACITY,
      (unsigned int)TINYMPC_SITL_MMAP_FILE_SIZE, path);
}
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
  attitude = qnormalize(mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
      state->attitudeQuaternion.w));
  if (!lqr_frame_initialized) {
  active_local_frame.origin_x = state->position.x;
  active_local_frame.origin_y = state->position.y;
  active_local_frame.origin_z = state->position.z;
  active_local_frame.yaw_world = quat2rpy(attitude).z;
  active_local_frame.cos_yaw = cosf(active_local_frame.yaw_world);
  active_local_frame.sin_yaw = sinf(active_local_frame.yaw_world);
    lqr_frame_initialized = true;
  }
  // Hover warm starts, new measurements, and references share one fixed chart.
  // Unlike the recentered model, its measured position is generally nonzero.
  float measured[12];
  if (!lqrMeasuredState(active_local_frame, sensors, state, measured)) {
    x0.setConstant(NAN);
    return;
  }
  for (int s = 0; s < NSTATES; ++s) x0(s) = measured[s];
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

static void updateHorizonReference(const setpoint_t *setpoint) {
  tiny_ClearPositionHalfspaces(&work);
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
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
}

static void resetRigidHoverOptimizer(void) {
  lqr_frame_initialized = false;
  Qu.setZero();
  for (int k = 0; k < NHORIZON; ++k) {
    Xhrz[k].setZero();
    Xref[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    YX[k].setZero();
    YU[k].setZero();
    p[k].setZero();
    state_linear_cost[k].setZero();
  }
  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uhrz[k] = ug;
    Uref[k] = ug;
    ZU[k] = ug;
    ZU_new[k] = ug;
    d[k].setZero();
    q[k].setZero();
    r[k].setZero();
    r_tilde[k].setZero();
  }
}

static void tinympcControllerTask(void *parameters) {
  (void)parameters;
  initializeMpcController();
  __atomic_store_n(&mpc_initialized, true, __ATOMIC_RELEASE);
  uint32_t log_counter = 0;

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    const uint64_t worker_start_us = usecTimestamp();
    mpcStageBegin("snapshot");
    const bool diagnostic_enabled = sitlDiagEnabled();
    uint64_t diagnostic_start_us = 0u;
    uint32_t diagnostic_start_tick = 0u;
    if (diagnostic_enabled) {
      diagnostic_start_us = usecTimestamp();
      diagnostic_start_tick = xTaskGetTickCount();
    }

    setpoint_t setpoint_task;
    sensorData_t sensors_task;
    state_t state_task;
    uint32_t solve_tick;
    bool reset_requested;
    uint64_t diagnostic_release_us;
    uint32_t diagnostic_release_rtos_tick;
    uint32_t diagnostic_release_sequence;
    uint32_t diagnostic_release_due_count;
    uint32_t diagnostic_release_mutex_miss_count;
    uint32_t diagnostic_semaphore_coalesced_count;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &planner_setpoint, sizeof(setpoint_task));
    memcpy(&sensors_task, &planner_sensors, sizeof(sensors_task));
    memcpy(&state_task, &planner_state, sizeof(state_task));
    solve_tick = planner_tick;
    const uint64_t snapshot_us = planner_snapshot_us;
    const uint32_t solve_generation = planner_generation;
    reset_requested = planner_reset_requested;
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
    planner_reset_requested = false;
    xSemaphoreGive(dataMutex);
    tinyRacerDebug.snapshot_queue_us = (uint32_t)(usecTimestamp() - snapshot_us);
    mpcStageEnd("snapshot");
    mpcStageBegin("reset");

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
    } else {
      solve_tick_timing_initialized = true;
    }
    previous_solve_tick = solve_tick;

    mpcStageEnd("reset"); // Dispatch only; detailed reset work follows.
    if (reset_requested) {
      mpcStageBegin("reset.duals");
      resetRigidHoverOptimizer();
      mpcStageEnd("reset.duals");
      mpcStageBegin("reset.halfspaces");
      tiny_ClearPositionHalfspaces(&work);
      mpcStageEnd("reset.halfspaces");
    }

    mpcStageBegin("state");
    // Hold the first armed snapshot, not a point advancing along a route.
    // This worker-only reference never replaces the commander's PID setpoint.
    static setpoint_t hover_reference;
#if TINYMPC_HOVER_HEIGHT_STEP_TEST
    static uint32_t hover_height_step_start_tick = 0u;
    static bool hover_height_step_applied = false;
#endif
#if TINYMPC_HOVER_PITCH_STEP_TEST
    static uint32_t hover_pitch_step_start_tick = 0u;
    static bool hover_pitch_step_applied = false;
#endif
    if (reset_requested) {
      memset(&hover_reference, 0, sizeof(hover_reference));
      hover_reference.position = state_task.position;
      hover_reference.attitude.yaw = state_task.attitude.yaw;
#if TINYMPC_HOVER_HEIGHT_STEP_TEST
      hover_height_step_start_tick = solve_tick;
      hover_height_step_applied = false;
#endif
#if TINYMPC_HOVER_PITCH_STEP_TEST
      hover_pitch_step_start_tick = solve_tick;
      hover_pitch_step_applied = false;
      DEBUG_PRINT("HOVER_PITCH_STEP armed tick=%lu delay_ms=3000 target_deg=60 position_hold=1\n",
                  (unsigned long)solve_tick);
#endif
    }
#if TINYMPC_HOVER_PITCH_STEP_TEST
    // Abrupt command: no future-knot anticipation, interpolation, or dual reset.
    // Keep the position/velocity goal and hover input reference unchanged so
    // the solver must reconcile them with the requested large attitude step.
    if (!hover_pitch_step_applied &&
        (uint32_t)(solve_tick - hover_pitch_step_start_tick) >= M2T(3000)) {
      hover_reference.attitude.pitch = 60.0f;
      hover_pitch_step_applied = true;
      DEBUG_PRINT("HOVER_PITCH_STEP applied tick=%lu elapsed_ms=%lu from_deg=0 to_deg=60\n",
                  (unsigned long)solve_tick,
                  (unsigned long)((solve_tick - hover_pitch_step_start_tick) * portTICK_PERIOD_MS));
    }
#endif
#if TINYMPC_HOVER_HEIGHT_STEP_TEST
    // Step the whole reference horizon after two seconds of initial hover.
    // Preserve x/y/yaw and zero desired velocity; no preview of the step.
    if (!hover_height_step_applied &&
        (uint32_t)(solve_tick - hover_height_step_start_tick) >= M2T(2000)) {
      hover_reference.position.z += 1.0f;
      hover_height_step_applied = true;
      DEBUG_PRINT("HOVER_HEIGHT_STEP applied tick=%lu target_z=%.3f delta_m=1 delay_ms=2000\n",
                  (unsigned long)solve_tick, (double)hover_reference.position.z);
    }
#endif
    setpoint_task = hover_reference;
    updateInitialState(&sensors_task, &state_task);
    mpcStageEnd("state");
    mpcStageBegin("reference");
    // Intentionally preserve TinyMPC's state auxiliaries and duals.
    updateHorizonReference(&setpoint_task);
    estimatorSetAidingInhibit(EstimatorAidingInhibitNone);
    mpcStageEnd("reference");
    const uint64_t solve_start_us = usecTimestamp();
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
    const uint64_t solve_finish_us = usecTimestamp();
    const uint32_t solve_us = (uint32_t)(solve_finish_us - solve_start_us);
    if (diagnostic_enabled) {
      TinyMpcSitlDiagRecord& record = sitl_diag_solve_record;
      memset(&record, 0, sizeof(record));
      record.event = 1u;
      record.release_sequence = diagnostic_release_sequence;
      record.solve_sequence = __atomic_add_fetch(
          &sitl_diag_solve_sequence, 1u, __ATOMIC_RELAXED);
      record.release_tick = diagnostic_release_rtos_tick;
      record.start_tick = solve_tick;
      record.finish_tick = xTaskGetTickCount();
      record.plan_tick = solve_tick;
      record.plan_age_ticks = record.finish_tick - solve_tick;
      record.release_due_count = diagnostic_release_due_count;
      record.release_mutex_miss_count = diagnostic_release_mutex_miss_count;
      record.semaphore_coalesced_count = diagnostic_semaphore_coalesced_count;
      record.solve_us = solve_us;
      record.solve_tick_gap = 0u;
      record.model_id = 0;
      record.solver_iterations = TINYMPC_RIGID_ADMM_ITERATIONS;
      record.release_us = diagnostic_release_us;
      record.start_us = solve_start_us;
      record.finish_us = solve_finish_us;
      record.primal_residual = info.pri_res;
      record.dual_residual = info.dua_res;
      const float estimator_values[13] = {
          state_task.position.x, state_task.position.y, state_task.position.z,
          state_task.velocity.x, state_task.velocity.y, state_task.velocity.z,
          state_task.attitudeQuaternion.x, state_task.attitudeQuaternion.y,
          state_task.attitudeQuaternion.z, state_task.attitudeQuaternion.w,
          radians(sensors_task.gyro.x), radians(sensors_task.gyro.y),
          radians(sensors_task.gyro.z)};
      memcpy(record.estimator, estimator_values, sizeof(estimator_values));
      record.local_frame[0] = active_local_frame.origin_x;
      record.local_frame[1] = active_local_frame.origin_y;
      record.local_frame[2] = active_local_frame.origin_z;
      record.local_frame[3] = active_local_frame.yaw_world;
      for (int state = 0; state < NSTATES; ++state) {
        record.reference[state] = Xref[0](state);
        record.initial_state[state] = x0(state);
      }
      for (int knot = 0; knot < NHORIZON; ++knot) {
        for (int state = 0; state < NSTATES; ++state) {
          const float reference = Xref[knot](state);
          const float primal = Xhrz[knot](state);
          const float projected = ZX_new[knot](state);
          record.horizon_reference[knot][state] = reference;
          record.horizon_primal_state[knot][state] = primal;
          record.horizon_projected_state[knot][state] = projected;
          record.optimizer_max_abs = T_MAX(record.optimizer_max_abs,
              T_MAX(fabsf(primal), fabsf(projected)));
          if (!std::isfinite(reference)) record.numeric_flags |= 1u;
          if (!std::isfinite(primal)) record.numeric_flags |= 2u;
          if (!std::isfinite(projected)) record.numeric_flags |= 4u;
        }
      }
      for (int knot = 0; knot < NHORIZON - 1; ++knot) {
        for (int input = 0; input < NINPUTS; ++input) {
          const float primal = Uhrz[knot](input);
          const float projected = ZU_new[knot](input);
          record.horizon_primal_input[knot][input] = primal;
          record.horizon_projected_input[knot][input] = projected;
          if (!std::isfinite(primal)) record.numeric_flags |= 8u;
          if (!std::isfinite(projected)) record.numeric_flags |= 16u;
        }
      }
      sitlDiagPublish(&record);
    }
    mpcStageBegin("publish");
    bool raw_plan_finite = true;
    float diagnostic_first_action_raw[NINPUTS] = {0.0f};
    uint32_t diagnostic_clamp_mask = 0u;
    // Prepare and validate direct commands before taking the publication lock.
    // No other task writes these solver-owned arrays.
    float prepared_direct_commands[NINPUTS];
    bool prepared_direct_valid = x0.allFinite();
    TinyMpcLqrPlan prepared_lqr_plan = {};
    prepared_lqr_plan.frame = active_local_frame;
    float projected_inputs[6][NINPUTS];
    for (int k = 0; k < 6; ++k)
      for (int m = 0; m < NINPUTS; ++m) projected_inputs[k][m] = ZU_new[k](m);
    // Recompute a dynamically consistent nominal trajectory from the exact
    // projected inputs that the fast feedback loop will use. Do not pair a
    // primal state trajectory with a different, post-projection input.
    prepared_direct_valid = prepared_direct_valid && tinyMpcLqrNominalBuild(
        &prepared_lqr_plan.nominal, x0.data(), projected_inputs,
        tinympc_generated_A, tinympc_generated_B, tinympc_generated_f);
    prepared_direct_valid = prepared_direct_valid
        && std::isfinite(active_local_frame.origin_x)
        && std::isfinite(active_local_frame.origin_y)
        && std::isfinite(active_local_frame.origin_z)
        && std::isfinite(active_local_frame.yaw_world)
        && std::isfinite(active_local_frame.cos_yaw)
        && std::isfinite(active_local_frame.sin_yaw);
    for (int k = 0; k < NHORIZON; ++k)
      prepared_direct_valid = prepared_direct_valid && Xhrz[k].allFinite();
    for (int motor = 0; motor < NINPUTS; ++motor) {
      // The nominal rollout above uses these same projected inputs.
      const float correction = ZU_new[0](motor);
      const float thrust = tinympc_generated_physical_hover_thrust[motor] + correction;
      const float command = tinympc_generated_thrust_to_normalized_command(thrust);
      prepared_direct_valid = prepared_direct_valid && std::isfinite(correction)
          && std::isfinite(thrust) && std::isfinite(command);
      prepared_direct_commands[motor] = T_MIN(T_MAX(command, 0.0f), 1.0f);
    }
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(active_motor_commands, prepared_direct_commands, sizeof(prepared_direct_commands));
    active_lqr_plan = prepared_lqr_plan;
    raw_plan_finite = prepared_direct_valid;
    plan_start_tick = solve_tick;
    // Reject non-finite plans before a callback can transfer motor authority.
    mpc_has_run = solve_generation == planner_generation && raw_plan_finite;
    xSemaphoreGive(dataMutex);
    tinyRacerDebug.publish_age_us = (uint32_t)(usecTimestamp() - snapshot_us);
    tinyRacerDebug.preparation_us = (uint32_t)(solve_start_us - worker_start_us);
    mpcStageEnd("publish");
    mpcStageBegin("diagnostics");
    if (solve_tick_gap_samples >= (uint32_t)MPC_RATE) {
      DEBUG_PRINT(
          "MPC snapshot gaps target_ticks=%lu min_ticks=%lu max_ticks=%lu skipped_release_slots=%lu samples=%lu\n",
          (unsigned long)expected_solve_tick_gap,
          (unsigned long)solve_tick_gap_min, (unsigned long)solve_tick_gap_max,
          (unsigned long)solve_tick_skipped_periods, (unsigned long)solve_tick_gap_samples);
      solve_tick_gap_min = UINT32_MAX; solve_tick_gap_max = 0u;
      solve_tick_gap_samples = 0u; solve_tick_skipped_periods = 0u;
    }
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
      tinyRacerDebug.plane_violation = max_primal_violation;
      tinyRacerDebug.consensus_error = max_primal_aux_gap;
      tinyRacerDebug.slack = max_halfspace_slack;
      // Full-horizon/state diagnostics: the legacy position-only consensus
      // number does not establish attitude/rate or input convergence.
      float state_gap = 0.0f, input_gap = 0.0f;
      for (int k = 0; k < NHORIZON; ++k)
        state_gap = T_MAX(state_gap, (Xhrz[k] - ZX_new[k]).cwiseAbs().maxCoeff());
      for (int k = 0; k < NHORIZON - 1; ++k)
        input_gap = T_MAX(input_gap, (Uhrz[k] - ZU_new[k]).cwiseAbs().maxCoeff());
      tinyRacerDebug.admm_state_gap = state_gap;
      tinyRacerDebug.admm_input_gap_n = input_gap;
      if ((log_counter % MPC_RATE) == 0U) {
        DEBUG_PRINT("LQR-DIAG xGap=%.4f uGapN=%.4f posErr=(%.2f,%.2f,%.2f) sat=%u\n",
            (double)state_gap, (double)input_gap,
            (double)(Xref[0](0) - x0(0)), (double)(Xref[0](1) - x0(1)),
            (double)(Xref[0](2) - x0(2)), (unsigned)tinyRacerDebug.lqr_saturation);
      }
      tinyRacerDebug.solve_us = solve_us;
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
    // Print from the low-priority worker, never the stabilizer callback.
    static uint32_t reported_handoff_event = 0;
    TinyMpcFallbackDiag diagnostic_snapshot;
    taskENTER_CRITICAL();
    diagnostic_snapshot = fallback_diag;
    taskEXIT_CRITICAL();
    if (diagnostic_snapshot.events != reported_handoff_event || (log_counter % MPC_RATE) == 0u) {
      reported_handoff_event = diagnostic_snapshot.events;
      DEBUG_PRINT("HANDOFF-DIAG active=%u guard=%u reason=%u age_ms=%lu fb_age_ms=%lu direct_ms=%lu catch=%u vision=%u\n",
          (unsigned)diagnostic_snapshot.active, (unsigned)diagnostic_snapshot.guard,
          (unsigned)diagnostic_snapshot.reason, (unsigned long)diagnostic_snapshot.age_ms,
          (unsigned long)diagnostic_snapshot.fault_age_ms,
          (unsigned long)diagnostic_snapshot.direct_ms,
          (unsigned)tinyRacerDebug.line_catch, (unsigned)tinyRacerDebug.line_vision_stop);
    }
    mpcStageEnd("diagnostics");
    log_counter++;
    (void)worker_start_us;
  }
}

static void initializeMpcController(void) {
#if TINYMPC_GENERATED_MEASURED_BRUSHLESS
  DEBUG_PRINT("ACTUATOR-v1: BrushJAX cubic; mass=43g; published-not-calibrated\n");
  DEBUG_PRINT("ACTUATOR-FAST-v1: cached solver references and command/rotor shortcut\n");
  DEBUG_PRINT("PLAN-AGE-v1: latest coherent snapshot; retry releases; short publication lock\n");
#endif
  DEBUG_PRINT("HANDOFF-v5: PID until fresh plan; hover_only=%u trace=%u\n",
              (unsigned)TINYMPC_HOVER_ONLY, (unsigned)TINYMPC_STAGE_TRACE);
  DEBUG_PRINT("MPC worker priority=%u; radio/estimator retain priority\n", (unsigned)TINYMPC_TASK_PRI);
  DEBUG_PRINT("MODEL: 12-state rigid body; motor lag ignored; direct motor; ADMM iterations=%u\n", (unsigned)TINYMPC_RIGID_ADMM_ITERATIONS);
  DEBUG_PRINT("LQR-v2: MPC=100Hz LQR=500Hz; fixed hover frame; reset ADMM; projected time-aligned nominal; max_age=60ms\n");
  /* Start MPC initialization*/
  estimatorSetAidingInhibit(EstimatorAidingInhibitNone);
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
  tiny_SetStateLinearCost(&work, state_linear_cost);
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uref[k] = ug;
    Uhrz[k] = ug;
    ZU[k] = ug;
    ZU_new[k] = ug;
    YU[k].setZero();
  }
  for (int k = 0; k < NHORIZON; ++k) {
    state_linear_cost[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    YX[k].setZero();
  }
  tiny_ClearPositionHalfspaces(&work);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 1;
  stgs.max_iter = TINYMPC_RIGID_ADMM_ITERATIONS;
  stgs.iters_check_rho_update = 0;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  /* End of MPC initialization */  
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

  sitlDiagInit();

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int motor = 0; motor < NINPUTS; ++motor) {
    active_motor_commands[motor] = 0.0f;
    cached_motor_command[motor] = 0.0f;
  }
  cached_motor_command_valid = false;
  cached_motor_command_tick = 0u;
  planner_reset_requested = true;
  xSemaphoreGive(dataMutex);
  
  DEBUG_PRINT("Hover-only: fixed handoff position/yaw\n");
  DEBUG_PRINT("Exclusive TinyMPC direct motor control enabled\n");
#if   TINYMPC_BRAKING_CACHE_ENABLE
  DEBUG_PRINT("BRAKE cache enabled: locality-gated speed/pitch matrices\n");
#else
  DEBUG_PRINT("BRAKE cache disabled by experiment build; level matrix remains active during braking\n");
#endif
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTreeInit(void) {
  __atomic_store_n(&selection_reset_pending, true, __ATOMIC_RELEASE);
  // Never perform matrix setup in the real-time stabilizer task.
  static bool task_initialized = false;
  if (!task_initialized) {
    runTaskSemaphore = xSemaphoreCreateBinary();
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
    if (runTaskSemaphore == NULL || dataMutex == NULL) {
      // Leave the selector on PID fallback if allocation failed.
      return;
    }
    task_initialized = true;
    STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask,
                           TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);
  }
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!__atomic_load_n(&mpc_initialized, __ATOMIC_ACQUIRE)) {
    tinyRacerDebug.pid_fallback = 1u;
    controllerPid(control, setpoint, sensors, state, tick);
    return;
  }
  const bool controller_reactivated =
      __atomic_load_n(&selection_reset_pending, __ATOMIC_ACQUIRE) ||
      last_controller_tick == 0 || tick - last_controller_tick > M2T(200);
  last_controller_tick = tick;
  // CrazySim's SITL stabilizer intentionally bypasses the hardware supervisor
  // and always permits motor output. Mirror that contract here; hardware still
  // uses the real arming/supervisor state.
#if !defined(TINYMPC_SITL_START_DELAY_MS)
  #define TINYMPC_SITL_START_DELAY_MS 0U
#endif
  const bool diagnostic_enabled = sitlDiagEnabled();
  const uint32_t sitl_now_tick = xTaskGetTickCount();
  const bool motors_allowed =
      sitl_now_tick >= M2T(TINYMPC_SITL_START_DELAY_MS);
  const bool __attribute__((unused)) motors_allowed_changed =
      motors_allowed != motors_were_allowed;
  static uint32_t bench_start_tick = 0u;
  static bool bench_applied = false;
  if (supervisorIsArmed()) mpc_bench_run = 0u;
  if (!mpc_bench_run) bench_start_tick = 0u;
  else if (!bench_start_tick) bench_start_tick = tick;
  if (bench_start_tick && tick - bench_start_tick >= M2T(5000)) mpc_bench_run = 0u;
  const bool bench_active = mpc_bench_run && !supervisorIsArmed();
  const bool bench_changed = bench_active != bench_applied;
  static bool release_pending = false;
  const bool release_enabled = motors_allowed || bench_active;
  release_pending = release_enabled && (release_pending || RATE_DO_EXECUTE(MPC_RATE, tick));
  bool has_run_snapshot = false;
  uint32_t plan_start_tick_snapshot = 0;
  float motor_command_snapshot[NINPUTS] = {0.0f};
  has_run_snapshot = cached_motor_command_valid;
  plan_start_tick_snapshot = cached_motor_command_tick;
  for (int motor = 0; motor < NINPUTS; ++motor) {
    motor_command_snapshot[motor] = cached_motor_command[motor];
  }
  if (controller_reactivated || !motors_allowed) {
    cached_motor_command_valid = false;
    has_run_snapshot = false;
  }
  const bool diagnostic_release_due =
      diagnostic_enabled && motors_allowed && RATE_DO_EXECUTE(MPC_RATE, tick);
  if (diagnostic_release_due) {
    ++sitl_diag_release_due_count;
  }
  if (dataMutex != NULL && xSemaphoreTake(dataMutex, 0) == pdTRUE) {
    if (controller_reactivated || bench_changed || (!motors_allowed && motors_were_allowed)) {
      ++planner_generation;
      __atomic_store_n(&selection_reset_pending, false, __ATOMIC_RELEASE);
      mpc_has_run = false;
      planner_reset_requested = true;
      bench_applied = bench_active;
    }
    if ((motors_allowed || bench_active) &&
        (TINYMPC_FRESH_HOVER_SNAPSHOT || RATE_DO_EXECUTE(MPC_RATE, tick))) {
      memcpy(&planner_setpoint, setpoint, sizeof(planner_setpoint));
      memcpy(&planner_sensors, sensors, sizeof(planner_sensors));
      memcpy(&planner_state, state, sizeof(planner_state));
      planner_tick = tick;
      planner_snapshot_us = usecTimestamp();
      if (release_pending) {
        xSemaphoreGive(runTaskSemaphore);
        release_pending = false; // Full binary semaphore already represents this request.
      }
    }
    has_run_snapshot = mpc_has_run;
    plan_start_tick_snapshot = plan_start_tick;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      motor_command_snapshot[motor] = active_motor_commands[motor];
      cached_motor_command[motor] = motor_command_snapshot[motor];
    }
    cached_lqr_plan = active_lqr_plan;
    cached_motor_command_valid = has_run_snapshot;
    cached_motor_command_tick = plan_start_tick_snapshot;
    xSemaphoreGive(dataMutex);
  } else {
    if (diagnostic_release_due) {
      ++sitl_diag_release_mutex_miss_count;
    }
  }
  motors_were_allowed = motors_allowed;
  static uint32_t last_ready_plan_tick = UINT32_MAX;
  if (controller_reactivated || bench_changed) {
    tinyRacerDebug.held_plan_max_ms = 0u;
    last_ready_plan_tick = UINT32_MAX;
  }
  if ((motors_allowed || bench_active) && has_run_snapshot) {
    const uint32_t candidate_age = (uint32_t)(tick - plan_start_tick_snapshot) * portTICK_PERIOD_MS;
    tinyRacerDebug.held_plan_max_ms = T_MAX(tinyRacerDebug.held_plan_max_ms, candidate_age);
    if (last_ready_plan_tick != plan_start_tick_snapshot) {
      last_ready_plan_tick = plan_start_tick_snapshot;
      tinyRacerDebug.ready_plan_age_ms = candidate_age;
    }
  }
  static float lqr_commands[NINPUTS] = {};
  static float lqr_thrust_n[NINPUTS] = {};
  static bool lqr_ready = false;
  static uint32_t lqr_command_plan_tick = 0u, last_lqr_tick = 0u;
  if (controller_reactivated || bench_changed || (!motors_allowed && !bench_active)) {
    lqr_ready = false;
    last_lqr_tick = 0u;
    tinyRacerDebug.lqr_gap_max_ms = 0u;
  }
  if ((motors_allowed || bench_active) && has_run_snapshot &&
      tick - plan_start_tick_snapshot <= M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS) &&
      RATE_DO_EXECUTE(LQR_RATE, tick)) {
    const uint64_t lqr_start_us = usecTimestamp();
    float measured[12], target[12], feedforward[4];
    const uint32_t nominal_age_ms = (uint32_t)(tick - plan_start_tick_snapshot) * portTICK_PERIOD_MS;
    lqr_ready = lqrMeasuredState(cached_lqr_plan.frame, sensors, state, measured)
        && tinyMpcLqrNominalSample(&cached_lqr_plan.nominal, nominal_age_ms, target, feedforward);
    if (lqr_ready) {
      uint8_t saturation_mask = 0u;
      TinyMpcLqrThrustDiagnostic thrust_diagnostic;
      lqr_ready = tinyMpcLqrFeedbackThrust(measured, target,
          feedforward, tinympc_hover_lqr_gain,
          tinympc_generated_physical_hover_thrust, tinympc_hover_lqr_max_thrust_n,
          &saturation_mask, &thrust_diagnostic);
      tinyRacerDebug.lqr_saturation = saturation_mask;
      // Simulation transmits the final bounded LQR thrust in explicit SI units.
      if (lqr_ready) {
        memcpy(lqr_thrust_n, thrust_diagnostic.bounded_n, sizeof(lqr_thrust_n));
        // Freshness checks consume this snapshot too; in this mode it is N.
        memcpy(lqr_commands, lqr_thrust_n, sizeof(lqr_commands));
      }
    }
    lqr_command_plan_tick = plan_start_tick_snapshot;
    ++tinyRacerDebug.lqr_cycles;
    if (last_lqr_tick) tinyRacerDebug.lqr_gap_max_ms = T_MAX(tinyRacerDebug.lqr_gap_max_ms,
        (uint32_t)(tick - last_lqr_tick) * portTICK_PERIOD_MS);
    last_lqr_tick = tick;
    tinyRacerDebug.lqr_us = (uint32_t)(usecTimestamp() - lqr_start_us);
  }
  tinyRacerDebug.lqr_valid = lqr_ready && has_run_snapshot
      && tick - lqr_command_plan_tick <= M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS);
  if (!tinyRacerDebug.lqr_valid) tinyRacerDebug.lqr_saturation = 0u;
  // Hold the last 500 Hz feedback output on intervening 1 kHz callbacks.
  // Its age belongs to the plan actually used, not a newer pending publication.
  if (lqr_ready) {
    memcpy(motor_command_snapshot, lqr_commands, sizeof(lqr_commands));
    plan_start_tick_snapshot = lqr_command_plan_tick;
  } else {
    for (int motor = 0; motor < NINPUTS; ++motor) motor_command_snapshot[motor] = NAN;
  }
  if (bench_active || bench_changed) {
    // Benchmark plans never gain actuator authority, including the exit tick.
    control->controlMode = controlModePWM;
    control->thrust = 0.0f;
    control->roll = control->pitch = control->yaw = 0;
    for (int motor = 0; motor < NINPUTS; ++motor) control->normalizedForces[motor] = 0.0f;
    tinyRacerDebug.pid_fallback = 1u;
    return;
  }

  const bool command_is_fresh = has_run_snapshot &&
      (tick - plan_start_tick_snapshot <=
       M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS));
  static bool direct_was_active = false;
  static bool fallback_latched = false;
  const bool line_fault = false;
  static setpoint_t handoff_commander_setpoint;
  if (controller_reactivated
      || motors_allowed_changed
  ) handoff_commander_setpoint = *setpoint;
  // A landing, stop, or new pilot target must return authority to PID rather
  // than being ignored by the fixed-anchor hover reference.
  const bool hover_command_unchanged =
      true;
  if (controller_reactivated || !motors_allowed) {
    direct_was_active = false;
    fallback_latched = false;
  }
  const bool valid_motor_plan =
      tinyMpcHandoffPlanValid(
      has_run_snapshot, tick, plan_start_tick_snapshot,
      M2T(TINYMPC_DIRECT_COMMAND_MAX_AGE_MS), motor_command_snapshot);
  const uint8_t diagnostic_guard = (!motors_allowed ? 1u : 0u)
      | (!has_run_snapshot ? 2u : 0u)
      | (has_run_snapshot && !command_is_fresh ? 4u : 0u)
      | (has_run_snapshot && command_is_fresh && !valid_motor_plan ? 8u : 0u)
      | (!hover_command_unchanged ? 16u : 0u)
      | (fallback_latched ? 32u : 0u)
      | (line_fault ? 64u : 0u);
  const bool diagnostic_direct = motors_allowed && valid_motor_plan &&
      !fallback_latched && hover_command_unchanged && !line_fault;
  taskENTER_CRITICAL();
  tinyMpcFallbackDiagUpdate(&fallback_diag, controller_reactivated,
      diagnostic_direct, diagnostic_guard, tick * portTICK_PERIOD_MS,
      has_run_snapshot ? (uint32_t)(tick - plan_start_tick_snapshot) * portTICK_PERIOD_MS : UINT32_MAX);
  tinyRacerDebug.direct_active = fallback_diag.active;
  tinyRacerDebug.fallback_guard = fallback_diag.guard;
  tinyRacerDebug.fallback_reason = fallback_diag.reason;
  tinyRacerDebug.plan_age_ms = fallback_diag.age_ms;
  tinyRacerDebug.fallback_age_ms = fallback_diag.fault_age_ms;
  tinyRacerDebug.direct_duration_ms = fallback_diag.direct_ms;
  taskEXIT_CRITICAL();
  if (!motors_allowed || !valid_motor_plan || fallback_latched || !hover_command_unchanged || line_fault) {
    uint32_t pid_tick = tick;
    if (direct_was_active) {
      // Re-enter feedback control with current measurements, not old PID
      // integrators. Latch PID until disarm/reselection to avoid oscillation.
      attitudeControllerResetAllPID();
      controllerPidInit();
      pid_tick = tick - tick % (RATE_MAIN_LOOP / POSITION_RATE);
      fallback_latched = true;
    }
    direct_was_active = false;
    tinyRacerDebug.pid_fallback = 1u;
    controllerPid(control, setpoint, sensors, state, pid_tick);
    return;
  }
  direct_was_active = true;
  tinyRacerDebug.pid_fallback = 0u;
  if (diagnostic_enabled) {
    bool diagnostic_command_finite = true;
    for (int motor = 0; motor < NINPUTS; ++motor) {
      diagnostic_command_finite = diagnostic_command_finite &&
          std::isfinite(motor_command_snapshot[motor]);
    }
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
      // Actuator-event first_action_* are newtons in direct-SI mode.
      const float hover_command = tinympc_generated_physical_hover_thrust[motor];
      record.first_action_raw[motor] = motor_command_snapshot[motor];
      record.first_action_clamped[motor] = !motors_allowed ? 0.0f
          : diagnostic_fallback_reason == 0u
              ? motor_command_snapshot[motor] : hover_command;
    }
    sitlDiagPublish(&record);
    }
  }
  control->controlMode = controlModeMotorThrustSI;
  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; ++motor) {
    float thrust_n = command_is_fresh ? lqr_thrust_n[motor]
        : tinympc_generated_physical_hover_thrust[motor];
    if (!std::isfinite(thrust_n)) thrust_n = 0.0f;
    control->motorThrustN[motor] = motors_allowed ? fmaxf(thrust_n, 0.0f) : 0.0f;
  }
}

#if defined(__cplusplus)
}
#endif
