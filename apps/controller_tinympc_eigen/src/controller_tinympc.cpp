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
#include "tinympc/position_projection.h"
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

#include "controller.h"
#include "supervisor.h"
#if TINYMPC_ESPNET_STRAIGHT_TEST
#include "espnet_collision_link.h"
#include "tinympc_depthgate_planes.h"
#include "tinympc_depthgate_timing.h"
#include "tinympc_espnet_straight.h"
#include "tinympc_vision_stop.h"
#include "tinympc_vision_brake.h"
#include "tinympc_distance_brake.h"
#include "tinympc_brake_level.h"
#include "tinympc_vision_resume.h"
#include "tinympc_brake_hold.h"
#include "tinympc_gate_servo.h"
#include "tinympc_hover_trim.h"
#include "tinympc_attitude_cache.h"
#include "tinympc_depthgate_cache.h"
#endif
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (3 * configMINIMAL_STACK_SIZE)

// Ported from 0091e2c: per-solve position/yaw frame with world-aligned z.
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

static struct vec worldQuaternionToLocalRodrigues(
    const MpcLocalFrame& frame, struct quat quaternion_world_body) {
  const float half_yaw = 0.5f * frame.yaw_world;
  const float c = cosf(half_yaw);
  const float s = sinf(half_yaw);
  const struct quat quaternion_local_body = mkquat(
      c * quaternion_world_body.x + s * quaternion_world_body.y,
      c * quaternion_world_body.y - s * quaternion_world_body.x,
      c * quaternion_world_body.z - s * quaternion_world_body.w,
      c * quaternion_world_body.w + s * quaternion_world_body.z);
  const float denominator = fabsf(quaternion_local_body.w) > 1e-6f
      ? quaternion_local_body.w : copysignf(1e-6f, quaternion_local_body.w);
  return mkvec(quaternion_local_body.x / denominator,
      quaternion_local_body.y / denominator,
      quaternion_local_body.z / denominator);
}

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

static void initializeMpcSolver();
#ifdef TINYMPC_BENCHMARK_ONLY
static void benchmarkMpcSolver();
#endif
static volatile bool mpc_solver_ready;
static float mpc_hover_trim[4];
#if TINYMPC_ESPNET_STRAIGHT_TEST
// Transfer a small event to appMain; console I/O must never block MPC.
struct EspnetConsoleEvent {
  bool pending;
  unsigned type, reason; // 1 run, 2 brake, 3 settled, 4 rejected
  int speed_mm_s, peak_mm_s, time_ms, travel_mm;
  unsigned danger_count;
  int turn_direction;
  int target_mm_s, brake_distance_mm;
};
static EspnetConsoleEvent espnet_console_events[8];
static unsigned espnet_event_read, espnet_event_write;
struct EspnetSpeedEvent {
  bool pending;
  int forward_mm_s, peak_mm_s, target_mm_s;
  int error_x_mm, pitch_mrad, pitch_diff_mn;
  unsigned phase;
};
static EspnetSpeedEvent espnet_speed_event;
struct GateConsoleEvent {
  bool pending;
  unsigned phase, matched, centered, reject;
  uint32_t age_ms;
  int ex_milli, ey_milli, distance_mm;
};
static GateConsoleEvent gate_console_event;

#endif

void appMain() {
#if TINYMPC_ESPNET_STRAIGHT_TEST
  // Initialize UART and create its receiver before an airborne controller
  // handoff. The stabilizer callback must not initialize peripherals/tasks.
  espnetCollisionLinkInit();
#endif
  initializeMpcSolver();
#ifdef TINYMPC_BENCHMARK_ONLY
  benchmarkMpcSolver();
  DEBUG_PRINT("DISARMED BENCHMARK ONLY: OOT disabled\n");
#else
  mpc_solver_ready = true;
#endif
  DEBUG_PRINT("MPC ready: 2m/s; center>0.95 brake; 3 clear frames resume; 15s; pitch40; 5iter\n");

  DEBUG_PRINT("Fixed hover offsets mN: -6 +10 +6 -10\n");
  DEBUG_PRINT("GATE: rail+corner+edge; align/pass 0.5m/s; 1m then original heading\n");

  while(1) {
#if TINYMPC_ESPNET_STRAIGHT_TEST
    EspnetConsoleEvent event = {};
    EspnetSpeedEvent speed;
    GateConsoleEvent gate;
    taskENTER_CRITICAL();
    if (espnet_event_read != espnet_event_write) {
      event = espnet_console_events[espnet_event_read];
      espnet_event_read = (espnet_event_read + 1u) % 8u;
    }
    speed = espnet_speed_event;
    espnet_speed_event.pending = false;
    gate = gate_console_event;
    gate_console_event.pending = false;
    taskEXIT_CRITICAL();
    if (gate.pending) {
      const char *names[] = {"SEARCH", "SLOW", "ALIGN", "PASS", "RESUME", "ABORT"};
      DEBUG_PRINT("GATE %s rails=%u center=%u err=%d,%d distance=%dmm why=%u age=%lums\n",
          names[gate.phase < 6 ? gate.phase : 5], gate.matched, gate.centered,
          gate.ex_milli, gate.ey_milli, gate.distance_mm, gate.reject, (unsigned long)gate.age_ms);
    }
    if (event.pending) {
      if (event.type == 11) DEBUG_PRINT("DISTANCE BRAKE RUN: target=%dmm/s distance=%dmm; vision OFF\n", event.target_mm_s, event.brake_distance_mm);
      else if (event.type == 1) DEBUG_PRINT("VISION BRAKE RUN: 2m/s; center>0.95; 1 frame\n");
      else if (event.type == 2) {
        const char* reason = event.reason == TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED ? "SPEED REACHED"
            : event.reason == TINYMPC_ESPNET_STRAIGHT_DISTANCE ? "DISTANCE LIMIT"
            : event.reason == TINYMPC_ESPNET_STRAIGHT_TIMEOUT ? "TIMEOUT"
            : event.reason == TINYMPC_ESPNET_STRAIGHT_CANCEL ? "CANCEL"
            : event.reason == TINYMPC_ESPNET_STRAIGHT_DANGER ? "VISION DANGER"
            : event.reason == TINYMPC_ESPNET_STRAIGHT_STALE ? "VISION STALE" : "INVALID STATE";
        DEBUG_PRINT("BRAKE START %s: speed=%dmm/s target=0\n", reason, event.speed_mm_s);
        if (event.reason == TINYMPC_ESPNET_STRAIGHT_DANGER)
          DEBUG_PRINT("VISION sectors=%u; after braking: %s\n", event.danger_count,
              event.turn_direction > 0 ? "LEFT" : event.turn_direction < 0 ? "RIGHT" : "HOLD YAW");
      } else if (event.type == 3) {
        DEBUG_PRINT("BRAKE SETTLED: time=%dms travel=%dmm peak=%dmm/s\n",
            event.time_ms, event.travel_mm, event.peak_mm_s);
      } else if (event.type == 5) DEBUG_PRINT("VISION YAW START: %s 45deg\n", event.turn_direction > 0 ? "LEFT" : "RIGHT");
      else if (event.type == 6) DEBUG_PRINT("VISION YAW COMPLETE: line rotated; waiting for clear view\n");
      else if (event.type == 7) DEBUG_PRINT("VISION SEQUENCE CANCELLED: unsafe/stale/cancel/limit; hold\n");
      else if (event.type == 8) DEBUG_PRINT("VISION RESUME: next straight leg along rotated heading\n");
      else if (event.type == 9) DEBUG_PRINT("VISION CLEAR: 3 frames; resuming 2m/s from stopped position\n");
      else if (event.type == 10) DEBUG_PRINT("BRAKE HOLD: XY locked 500mm behind; correcting drift\n");
      else if (event.type == 12) DEBUG_PRINT("BRAKE HOLD: XY locked at stopping position\n");
      else if (event.type == 13) DEBUG_PRINT("BRAKE LEVEL: anticipating zero speed; v=%dmm/s\n", event.speed_mm_s);
      else DEBUG_PRINT("VISION START REJECTED: reason=%u\n", event.reason);
    }
    if (speed.pending) {
      DEBUG_PRINT("ESPNet speed mm/s: forward=%d peak=%d target=%d\n",
          speed.forward_mm_s, speed.peak_mm_s, speed.target_mm_s);
      DEBUG_PRINT("MPC phase=%u errX=%dmm pitchR=%dmrad pitchDiff=%dmN\n",
          speed.phase, speed.error_x_mm, speed.pitch_mrad, speed.pitch_diff_mn);
    }
#endif
    vTaskDelay(M2T(20));
  }
}

// Model and solve timing come from the generated upstream specialization.
#define DT TINYMPC_GENERATED_MODEL_DT_S
#define MPC_RATE TINYMPC_GENERATED_SOLVE_RATE_HZ
#if TINYMPC_ESPNET_STRAIGHT_TEST
static_assert(MPC_RATE == 100, "DepthGate must retain the 100 Hz control update");
#endif
#define LQR_RATE RATE_500_HZ  // control frequency

static_assert(NSTATES == TINYMPC_GENERATED_STATE_DIM, "generated state dimension mismatch");
static_assert(NINPUTS == TINYMPC_GENERATED_INPUT_DIM, "generated input dimension mismatch");

/* Include trajectory to track */
// #include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"  // Large circle (1m radius)
#include "traj_circle_small.h"  // Small circle (0.5m radius, 100 Hz)
#if defined(CIRCLE_REFERENCE_SAMPLE_RATE_HZ)
static_assert(CIRCLE_REFERENCE_SAMPLE_RATE_HZ == MPC_RATE,
    "circle reference sample rate must match MPC updates");
#endif
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

static VectorNf ZX[NHORIZON], ZX_new[NHORIZON], YX[NHORIZON];
static MatrixNf Acx;
static VectorNf lcx, ucx, stateConstraintWeights;
// Component-wise Rodrigues box; angle equivalents apply to single-axis tilt.
static constexpr float tiltRodriguesBound = 0.577350269f; // tan(60 degrees / 2), roll
static constexpr float pitchRodriguesBound = 0.363970234f; // tan(40 degrees / 2)
static constexpr float bodyRateBound = 1.780235837f; // 102 degrees/s; 15% below 120
static constexpr float yawRateBound = 5.585053606f; // 320 degrees/s for tangent yaw
static float mpc_constraints[4]; // predicted tilt/rate excess, primal residual, solve us
static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1];
static VectorMf d[NHORIZON-1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON];

static VectorNf q[NHORIZON-1];
static VectorNf qBase[NHORIZON-1], terminalBase;
static VectorMf r[NHORIZON-1];
static VectorMf r_tilde[NHORIZON-1];

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;
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

// Helper variables
static uint64_t startTimestamp;
// static bool isInit = false;  // fix for tracking problem - UNUSED, commented out
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
static int8_t result = 0;
static uint32_t step = 0;
// The ESPNet profile owns the complete horizon reference. Keep the generated
// circle available only for the explicitly selected legacy profile so a stale
// trajectory flag cannot change a straight-line flight build.
#if TINYMPC_ESPNET_STRAIGHT_TEST
static bool en_traj = false;
#else
static bool en_traj = true;
#endif
static const uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static const uint8_t traj_hold = 1;
static uint32_t traj_idx = 0;
static const float legacy_hover_command[NINPUTS] = {
    0.7f, 0.663f, 0.7373f, 0.633f};

static MpcLocalFrame active_local_frame;

// Basic mode - no obstacle avoidance constraints

static void loadGeneratedSolverData(void) {
  for (int row = 0; row < NSTATES; ++row) {
    f(row) = tinympc_generated_f[row];
    APf(row) = attitude_cache_APf[row];
    for (int column = 0; column < NSTATES; ++column) {
      const int index = row * NSTATES + column;
      A(row, column) = tinympc_generated_A[index];
      Pinf(row, column) = attitude_cache_Pinf[index];
      AmBKt(row, column) = attitude_cache_AmBKt[index];
    }
    for (int column = 0; column < NINPUTS; ++column) {
      const int index = row * NINPUTS + column;
      B(row, column) = tinympc_generated_B[index];
      coeff_d2p(row, column) = attitude_cache_coeff_d2p[index];
    }
  }

  Q.setZero();
  R.setZero();
  for (int input = 0; input < NINPUTS; ++input) {
    BPf(input) = attitude_cache_BPf[input];
    ug(input) = tinympc_generated_hover_reference[input];
    lcu(input) = tinympc_generated_input_lower[input * (NHORIZON - 1)];
    ucu(input) = tinympc_generated_input_upper[input * (NHORIZON - 1)];
    R(input, input) = tinympc_generated_R_diagonal[input];
    for (int state = 0; state < NSTATES; ++state) {
      Kinf(input, state) = attitude_cache_Kinf[input * NSTATES + state];
    }
    for (int column = 0; column < NINPUTS; ++column) {
      Quu_inv(input, column) =
          attitude_cache_Quu_inv[input * NINPUTS + column];
    }
  }
  for (int state = 0; state < NSTATES; ++state) {
    Q(state, state) = tinympc_generated_Q_diagonal[state];
  }
}

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  const struct quat attitude = qnormalize(mkquat(
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
  x0.segment<3>(3) << attitude_local.x, attitude_local.y, attitude_local.z;
  x0.segment<3>(6) = velocity_local;
  // Gyroscope rates already use body axes; do not yaw-rotate them.
  x0.segment<3>(9) << radians(sensors->gyro.x),
      radians(sensors->gyro.y), radians(sensors->gyro.z);
}

static void setLocalReferenceState(
    VectorNf& target, const Eigen::Vector3f& position_world,
    struct quat attitude_world_body, const Eigen::Vector3f& velocity_world,
    const Eigen::Vector3f& angular_velocity_body) {
  const Eigen::Vector3f position_local = worldVectorToLocal(
      active_local_frame, position_world - Eigen::Vector3f(
          active_local_frame.origin_x, active_local_frame.origin_y,
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

#if TINYMPC_ESPNET_STRAIGHT_TEST
static uint8_t esp_test_run;
static uint8_t esp_test_external; // Explicit Python full-state trajectory mode.
static float esp_test_config_speed = 2.0f, esp_test_config_distance = 0.0f;
static float esp_test_target_speed, esp_test_brake_distance;
static TinyBrakeLevel esp_test_level;
static tinympcEspnetStraightState esp_test_state;
static Eigen::Vector3f esp_test_origin, esp_test_hold;
static float esp_test_yaw, esp_test_distance, esp_test_speed;
static float esp_test_center, esp_test_settle_s;
static uint32_t esp_test_age_ms = UINT32_MAX, esp_test_tick;
static uint8_t esp_test_phase, esp_test_reason, esp_test_fresh;
static uint16_t esp_test_handoff;
static bool esp_test_settled;
static bool esp_test_hold_locked;
static float esp_test_peak_speed;
static uint32_t esp_test_speed_print_tick;
static float esp_test_brake_elapsed, esp_test_brake_start_distance;
static float esp_test_yaw_reference, esp_test_turn_goal, esp_test_turn_settle;
static int esp_test_turn_direction;
static unsigned esp_test_turn_stage; // 0 none, 1 braking, 2 turning, 3 waiting clear, 4 terminal
static float esp_test_clear_s, esp_test_mission_elapsed, esp_test_mission_travel;
static bool esp_test_mission_active, esp_test_resume_heading;
static Eigen::Vector3f esp_test_mission_last_position;
static tinympcVisionResumeState esp_test_resume_state;
static TinyGateServo gate_servo;
static Eigen::Vector3f gate_pass_origin, gate_acquire_origin;
static float gate_distance;
static uint32_t gate_print_tick;

// Explicit opt-in: old ESPNet flight behavior remains selected until enabled.
static uint8_t dg_enable, dg_fresh, dg_count, dg_mode, dg_fault;
static float dg_speed = .1f, dg_clearance = .5f;
static TinyDepthGateHistory dg_pose_history = {};
// Symmetric center rays from calibrated HM01B0 fx=89.15584 at width160.
static float dg_ray_slope = .598203f, dg_activation = 2.f;
static uint32_t dg_age_ms = UINT32_MAX, dg_sample, dg_tick;
static uint16_t dg_sequence;
static float dg_depth[3], dg_violation, dg_command_speed;
static Eigen::Vector2f dg_world_n[2];
static float dg_world_b[2], dg_local_b[2];
static Eigen::Vector3f dg_hold;
static bool dg_have_planes, dg_was_moving;
static float dg_elapsed;
static bool dg_cache_selected;

static void selectDepthGateCache(bool enabled) {
  if(enabled==dg_cache_selected) return;
  const float *pf=enabled?depthgate_cache_Pinf:attitude_cache_Pinf;
  const float *am=enabled?depthgate_cache_AmBKt:attitude_cache_AmBKt;
  const float *cd=enabled?depthgate_cache_coeff_d2p:attitude_cache_coeff_d2p;
  const float *af=enabled?depthgate_cache_APf:attitude_cache_APf;
  const float *bf=enabled?depthgate_cache_BPf:attitude_cache_BPf;
  const float *ki=enabled?depthgate_cache_Kinf:attitude_cache_Kinf;
  const float *qu=enabled?depthgate_cache_Quu_inv:attitude_cache_Quu_inv;
  const float *sw=enabled?depthgate_cache_state_weights:attitude_cache_state_weights;
  for(int i=0;i<NSTATES;++i) {
    APf(i)=af[i]; stateConstraintWeights(i)=sw[i];
    for(int j=0;j<NSTATES;++j) {Pinf(i,j)=pf[i*NSTATES+j];AmBKt(i,j)=am[i*NSTATES+j];}
    for(int j=0;j<NINPUTS;++j) coeff_d2p(i,j)=cd[i*NINPUTS+j];
  }
  for(int i=0;i<NINPUTS;++i) {
    BPf(i)=bf[i];
    for(int j=0;j<NSTATES;++j) Kinf(i,j)=ki[i*NSTATES+j];
    for(int j=0;j<NINPUTS;++j) Quu_inv(i,j)=qu[i*NINPUTS+j];
  }
  stgs.max_iter=5; // Same bounded iteration budget at 100 Hz in both modes.
  dg_cache_selected=enabled;
}

static void setDepthGateHoldReference(const Eigen::Vector3f& position) {
  dg_hold=position; dg_hold.z()=esp_test_origin.z();
  dg_command_speed=0; dg_was_moving=false;
  for(int k=0;k<NHORIZON;++k) {
    setLocalReferenceState(Xref[k],dg_hold,rpy2quat(mkvec(0,0,esp_test_yaw)),
        Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero());
    if(k<NHORIZON-1) Uref[k]=ug;
  }
}

static void updateDepthGateReference(const state_t& state, uint32_t tick) {
  const Eigen::Vector3f pos(state.position.x,state.position.y,state.position.z);
  const Eigen::Vector3f vel(state.velocity.x,state.velocity.y,state.velocity.z);
  if(!dg_tick) {
    dg_hold=pos; dg_have_planes=false; dg_count=dg_mode=0;
    dg_pose_history = {};
    dg_command_speed=dg_elapsed=0; dg_was_moving=false;
    if(esp_test_run) dg_fault=7; // Require RUN release after mode changes.
  }
  const float dt = dg_tick ? fminf((tick-dg_tick)*portTICK_PERIOD_MS*.001f,.1f) : DT;
  dg_tick=tick;
  tinyDepthGateRecord(&dg_pose_history, {tick*portTICK_PERIOD_MS,
      pos.x(),pos.y(),active_local_frame.yaw_world});
  DepthGateObservation obs={};
  const bool received=depthGateLinkGetLatest(&obs);
  dg_age_ms=received ? obs.received_age_ms : UINT32_MAX;
  dg_fresh=received && tinyDepthGateFresh(obs.valid,obs.received_age_ms,obs.inference_us);
  const float padding=DG_MAX_SPEED*(DG_MAX_RECEIVE_AGE_MS*.001f+.05f);
  const bool config_ok=std::isfinite(dg_speed) && dg_speed>=0 && dg_speed<=DG_MAX_SPEED &&
      std::isfinite(dg_clearance) && dg_clearance>=0 && dg_clearance+padding<=2.f &&
      std::isfinite(dg_ray_slope) && dg_ray_slope>=.05f && dg_ray_slope<=2.f &&
      std::isfinite(dg_activation) && dg_activation>dg_clearance+padding && dg_activation<=6.f;
  if (!esp_test_run) { dg_fault=0; dg_elapsed=0; }
  if (dg_fresh && (!dg_have_planes || obs.sample!=dg_sample)) {
    // Anchor each plane to measured capture pose. Camera capture and inference
    // are synchronous; allow travel before the following result arrives.
    TinyDepthGatePose capture_pose = {};
    const bool have_pose=tinyDepthGateCapturePose(&dg_pose_history,
        tick*portTICK_PERIOD_MS,obs.received_age_ms,obs.inference_us,&capture_pose);
    const TinyDepthGatePlanes planes=tinyDepthGatePlanes(obs.inverse_depth,
        dg_ray_slope,dg_clearance+padding,dg_activation,6.f,dg_mode);
    if (!have_pose || !planes.valid || !pos.allFinite() || !vel.allFinite()) dg_fresh=0;
    else {
      dg_sample=obs.sample; dg_sequence=obs.sequence; dg_have_planes=true;
      dg_count=planes.count; dg_mode=planes.mode;
      const float yaw=capture_pose.yaw;
      const float cy=cosf(yaw),sy=sinf(yaw);
      const Eigen::Vector2f capture(capture_pose.x,capture_pose.y);
      for(int i=0;i<3;++i) dg_depth[i]=planes.depth[i];
      for(unsigned i=0;i<planes.count;++i) {
        dg_world_n[i]=Eigen::Vector2f(cy*planes.nx[i]-sy*planes.ny[i],
                                    sy*planes.nx[i]+cy*planes.ny[i]);
        dg_world_b[i]=planes.b[i]+dg_world_n[i].dot(capture);
      }
    }
  }
  // A dropout latches hold until RUN is released; never auto-resume blind.
  if (esp_test_run && (!dg_fresh || !config_ok)) dg_fault=1;
  if (esp_test_run && (fabsf(state.attitude.roll)>15.f ||
                      fabsf(state.attitude.pitch)>15.f)) dg_fault=2;
  if (esp_test_run && (!pos.allFinite() || !vel.allFinite() || vel.head<2>().norm()>DG_MAX_SPEED)) dg_fault=8;
  for(int k=0;k<NHORIZON;++k) {
    data.count_xy_hs[k]=(k>0 && dg_have_planes)?dg_count:0;
    for(unsigned i=0;i<dg_count;++i) {
      const Eigen::Vector3f n=worldVectorToLocal(active_local_frame,
          Eigen::Vector3f(dg_world_n[i].x(),dg_world_n[i].y(),0));
      data.a_xy_hs[k][i]=n.head<2>();
      dg_local_b[i]=dg_world_b[i]-dg_world_n[i].dot(pos.head<2>());
      data.b_xy_hs[k][i]=dg_local_b[i];
      if (esp_test_run && dg_local_b[i]<0.f) dg_fault=3;
    }
  }
  if(esp_test_run) dg_elapsed+=dt;
  if(dg_elapsed>=10.f) dg_fault=4;
  const bool moving=esp_test_run && dg_fresh && !dg_fault;
  if(!moving && dg_was_moving) dg_hold=pos;
  dg_was_moving=moving;
  dg_command_speed=moving?fminf(dg_speed,dg_command_speed+.5f*dt):0.f;
  const Eigen::Vector3f forward(cosf(esp_test_yaw),sinf(esp_test_yaw),0);
  static Eigen::Vector3f refs[NHORIZON]; // Single stabilizer caller; spare its small stack.
  for(int k=0;k<NHORIZON;++k) {
    Eigen::Vector3f goal=moving ? pos+forward*(dg_command_speed*k*DT) : dg_hold;
    goal.z()=esp_test_origin.z();
    const Eigen::Vector3f local=worldVectorToLocal(active_local_frame,goal-pos);
    Eigen::Vector2f projected=local.head<2>();
    if (moving && !tiny_ProjectXY(local.head<2>(),data.a_xy_hs[1],data.b_xy_hs[1],
        dg_count,Eigen::Vector2f(-100,-100),Eigen::Vector2f(100,100),
        Eigen::Vector2f(1,1),projected)) { dg_fault=5; projected.setZero(); }
    refs[k]=pos+Eigen::Vector3f(active_local_frame.cos_yaw*projected.x()-active_local_frame.sin_yaw*projected.y(),
        active_local_frame.sin_yaw*projected.x()+active_local_frame.cos_yaw*projected.y(),local.z());
  }
  for(int k=0;k<NHORIZON;++k) {
    const Eigen::Vector3f desired_v=moving && k<NHORIZON-1 ?
        Eigen::Vector3f((refs[k+1]-refs[k])/DT):Eigen::Vector3f::Zero();
    setLocalReferenceState(Xref[k],refs[k],rpy2quat(mkvec(0,0,esp_test_yaw)),desired_v,Eigen::Vector3f::Zero());
    if(k<NHORIZON-1) Uref[k]=ug;
  }
  if(dg_fault && moving) setDepthGateHoldReference(pos);
  esp_test_fresh=dg_fresh; esp_test_age_ms=dg_age_ms;
  esp_test_speed=forward.dot(vel); esp_test_phase=moving?1:0;
}


static void resetEspnetStraightTest(const state_t& state, uint32_t tick,
                                   float yaw) {
  for (int i = 0; i < 4; ++i) {
    mpc_hover_trim[i] = tinympc_hover_default_trim[i];
    // Preserve physical actuator limits about the calibrated equilibrium.
    lcu(i) = tinympc_generated_input_lower[i * (NHORIZON - 1)] - mpc_hover_trim[i];
    ucu(i) = tinympc_generated_input_upper[i * (NHORIZON - 1)] - mpc_hover_trim[i];
  }
  tinympcEspnetStraightInit(&esp_test_state);
  esp_test_target_speed = esp_test_config_speed;
  esp_test_brake_distance = esp_test_config_distance;
  esp_test_level = {};
  lcx(4) = -pitchRodriguesBound;
  esp_test_resume_state = {};
  gate_servo = {};
  gate_distance = 0;
  gate_print_tick = tick;
  esp_test_run = 0;
  esp_test_origin = Eigen::Vector3f(
      state.position.x, state.position.y, state.position.z);
  esp_test_hold = esp_test_origin;
  esp_test_yaw = yaw;
  esp_test_yaw_reference = yaw;
  esp_test_turn_goal = yaw;
  esp_test_turn_settle = 0.0f;
  esp_test_turn_direction = 0;
  esp_test_turn_stage = 0;
  esp_test_clear_s = 0.0f;
  esp_test_mission_elapsed = esp_test_mission_travel = 0.0f;
  esp_test_mission_active = esp_test_resume_heading = false;
  esp_test_mission_last_position = esp_test_origin;
  esp_test_tick = tick;
  esp_test_settle_s = 0.0f;
  esp_test_settled = false;
  esp_test_hold_locked = false;
  esp_test_peak_speed = 0.0f;
  esp_test_speed_print_tick = tick;
  esp_test_brake_elapsed = 0.0f;
  esp_test_brake_start_distance = 0.0f;
  dg_hold=esp_test_origin; dg_have_planes=false; dg_count=dg_mode=dg_fault=0;
  dg_tick=0; dg_command_speed=dg_elapsed=0; dg_was_moving=false;
  ++esp_test_handoff; // First OOT solve initialized; flight script may request RUN.
}

static void updateEspnetStraightReference(const state_t& state, uint32_t tick) {
  EspnetCollisionObservation collision = {};
  const bool received = espnetCollisionLinkGetLatest(&collision);
  esp_test_age_ms = received ? collision.received_age_ms : UINT32_MAX;
  esp_test_center = received ? collision.probability[1] : 0.0f;
  esp_test_fresh = received && tinympcEspnetCollisionFresh(
      collision.valid, esp_test_age_ms);
  const Eigen::Vector3f position(state.position.x, state.position.y,
                                 state.position.z);
  const Eigen::Vector3f velocity(state.velocity.x, state.velocity.y,
                                 state.velocity.z);
  // Camera-forward on the standard AI-deck mount is Crazyflie body +X.
  Eigen::Vector3f forward(cosf(esp_test_yaw), sinf(esp_test_yaw), 0.0f);
  esp_test_distance = forward.dot(position - esp_test_origin);
  esp_test_speed = forward.dot(velocity);
  const float dt = tick == esp_test_tick ? 1.0f / MPC_RATE
      : (tick - esp_test_tick) * (portTICK_PERIOD_MS / 1000.0f);
  esp_test_tick = tick;
  if (esp_test_mission_active && std::isfinite(dt) && dt > 0.0f) {
    esp_test_mission_elapsed += dt;
    const Eigen::Vector3f displacement = position - esp_test_mission_last_position;
    esp_test_mission_travel += displacement.head<2>().norm();
    esp_test_mission_last_position = position;
  }
  const bool mission_limit = esp_test_mission_active &&
      (esp_test_mission_elapsed >= 15.0f ||
       !std::isfinite(esp_test_mission_elapsed) || !std::isfinite(esp_test_mission_travel));
  const tinympcVisionDecision vision = tinympcVisionClassify(esp_test_fresh,
      collision.probability[0], collision.probability[1], collision.probability[2]);
  // Revalidate on the actual resume tick: a new blocked frame must not
  // bypass the stronger all-clear condition used for automatic restart.
  if (esp_test_resume_heading && (!vision.valid || vision.dangerous_sectors != 0 ||
      !std::isfinite(velocity.norm()) || velocity.norm() > .15f)) {
    esp_test_state.phase = TINYMPC_ESPNET_STRAIGHT_STOP;
    esp_test_state.reason = TINYMPC_ESPNET_STRAIGHT_DANGER;
    esp_test_turn_stage = 3;
    esp_test_clear_s = 0.0f;
    esp_test_resume_heading = false;
  }
  const auto previous_phase = esp_test_state.phase;
  const bool requested = esp_test_run != 0;
  const bool previously_settled = esp_test_settled;
  // Single vision-triggered stop; no yaw or automatic restart.
  esp_test_turn_direction = 0;
  if (esp_test_brake_distance != 0.0f) {
    tinympcDistanceBrakeStep(&esp_test_state,
        previous_phase == TINYMPC_ESPNET_STRAIGHT_IDLE ? 0.0f : esp_test_distance,
        velocity.norm(), requested, dt, esp_test_target_speed, esp_test_brake_distance);
  } else tinympcVisionBrakeStep(&esp_test_state, esp_test_distance,
      velocity.norm(), esp_test_fresh, collision.probability[0],
      collision.probability[1], collision.probability[2], requested, dt, collision.sequence);
  if (mission_limit) {
    const auto reason = esp_test_mission_elapsed >= 15.0f
        ? TINYMPC_ESPNET_STRAIGHT_TIMEOUT : TINYMPC_ESPNET_STRAIGHT_INVALID;
    tinympcEspnetStraightStop(&esp_test_state, reason, esp_test_distance);
  }
  // A danger stop keeps the mission request active so clear-frame confirmation
  // can restart it. Other stop reasons terminate the mission.
  if (esp_test_state.phase != TINYMPC_ESPNET_STRAIGHT_RUN &&
      !(esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP &&
        esp_test_state.reason == TINYMPC_ESPNET_STRAIGHT_DANGER && requested))
    esp_test_run = 0;
  if (previous_phase == TINYMPC_ESPNET_STRAIGHT_IDLE &&
      esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_RUN) {
    // Measure the travel from the accepted start, retaining the chosen heading.
    const float mission_height = esp_test_hold.z();
    esp_test_origin = position;
    esp_test_hold = position;
    if (esp_test_mission_active) {
      esp_test_origin.z() = mission_height;
      esp_test_hold.z() = mission_height;
    }
    if (!esp_test_resume_heading)
      esp_test_yaw = quat2rpy(qnormalize(mkquat(
          state.attitudeQuaternion.x, state.attitudeQuaternion.y,
          state.attitudeQuaternion.z, state.attitudeQuaternion.w))).z;
    esp_test_resume_heading = false;
    esp_test_yaw_reference = esp_test_yaw;
    forward = Eigen::Vector3f(cosf(esp_test_yaw), sinf(esp_test_yaw), 0.0f);
    esp_test_distance = 0.0f;
    esp_test_state.s = 0.0f;
    esp_test_state.v = TINYMPC_VISION_TARGET_SPEED;
    if (!esp_test_mission_active) {
      esp_test_peak_speed = 0.0f;
      esp_test_mission_elapsed = esp_test_mission_travel = 0.0f;
      esp_test_mission_active = true;
    }
    esp_test_mission_last_position = position;
    esp_test_settled = false;
    esp_test_hold_locked = false;
    esp_test_settle_s = 0.0f;
    esp_test_turn_stage = 0;
    esp_test_turn_settle = 0.0f;
    esp_test_clear_s = 0.0f;
  }
  // Gate phases remain RUN to the flight script; only a terminal brake lands.
  EspnetGateObservation gate_observation = {};
  const bool gate_received = espnetGateLinkGetLatest(&gate_observation);
  TinyGateInput gate_input = {};
  const struct vec gate_rpy = quat2rpy(qnormalize(mkquat(
      state.attitudeQuaternion.x, state.attitudeQuaternion.y,
      state.attitudeQuaternion.z, state.attitudeQuaternion.w)));
  const float gate_yaw_error = atan2f(sinf(gate_rpy.z-esp_test_yaw), cosf(gate_rpy.z-esp_test_yaw));
  gate_input.pose_aligned = fabsf(gate_rpy.x) < radians(5.0f) &&
      fabsf(gate_rpy.y) < radians(5.0f) && fabsf(gate_yaw_error) < radians(5.0f);
  gate_input.fresh = gate_received && gate_observation.valid &&
      gate_observation.received_age_ms <= 400;
  gate_input.sample = gate_observation.sample;
  gate_input.edge_mask = gate_observation.corner_edge_mask;
  for (unsigned i = 0; i < 2; ++i) gate_input.rail[i] = gate_observation.rail_probability[i];
  for (unsigned i = 0; i < 4; ++i) {
    gate_input.x[i] = gate_observation.corner_x[i];
    gate_input.y[i] = gate_observation.corner_y[i];
    gate_input.confidence[i] = gate_observation.corner_confidence[i];
  }
  const unsigned previous_gate_phase = gate_servo.phase;
  if (gate_servo.phase == GATE_PASS)
    gate_distance = forward.dot(position - gate_pass_origin);
  tinyGateStep(&gate_servo, &gate_input,
      esp_test_brake_distance == 0.0f && esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_RUN,
      velocity.norm(), gate_distance, dt);
  if (gate_servo.phase == GATE_SLOW && previous_gate_phase != GATE_SLOW) {
    gate_acquire_origin = position;
    gate_distance = 0;
  }
  // Bound the search; a missing opening must not cause indefinite lateral travel.
  if ((gate_servo.phase == GATE_SLOW || gate_servo.phase == GATE_ALIGN) &&
      ((position-gate_acquire_origin).norm() > 1.5f || position.z() < .2f || position.z() > 1.5f)) {
    gate_servo.phase = GATE_ABORT;
    gate_servo.reject = 4;
  }
  if (gate_servo.phase == GATE_PASS && previous_gate_phase != GATE_PASS) {
    gate_pass_origin = position;
    gate_distance = 0;
  }
  if (gate_servo.phase == GATE_COOLDOWN && previous_gate_phase == GATE_PASS) {
    // New parallel straight leg starts at the measured gate exit, same yaw.
    esp_test_origin = esp_test_hold = position;
    esp_test_state.s = 0;
    esp_test_state.v = TINYMPC_VISION_TARGET_SPEED;
    esp_test_distance = 0;
  }
  if (gate_servo.phase == GATE_ABORT) {
    tinympcEspnetStraightStop(&esp_test_state,
        gate_servo.reject == 3 ? TINYMPC_ESPNET_STRAIGHT_STALE : TINYMPC_ESPNET_STRAIGHT_INVALID,
        esp_test_distance);
    esp_test_run = 0;
  }
  const bool gate_active = gate_servo.phase >= GATE_SLOW && gate_servo.phase <= GATE_PASS;
  if (gate_active) {
    // Discard the straight-leg reference backlog while the gate owns the reference.
    esp_test_state.s = esp_test_distance;
    esp_test_state.v = gate_servo.forward;
    esp_test_hold = position;
  }
  if (previous_gate_phase != gate_servo.phase ||
      tick-gate_print_tick >= 1000) {
    gate_print_tick = tick;
    GateConsoleEvent gate_event = {true, gate_servo.phase, gate_servo.matched,
        gate_servo.centered, gate_servo.reject,
        gate_received ? gate_observation.received_age_ms : UINT32_MAX,
        (int)(gate_servo.ex*1000), (int)(gate_servo.ey*1000), (int)(gate_distance*1000)};
    taskENTER_CRITICAL();
    gate_console_event = gate_event;
    taskEXIT_CRITICAL();
  }
  if (esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP &&
      previous_phase != TINYMPC_ESPNET_STRAIGHT_STOP) {
    esp_test_hold_locked = false;
    esp_test_brake_start_distance = esp_test_distance;
    esp_test_brake_elapsed = 0.0f;
    if (esp_test_turn_direction != 0) esp_test_turn_stage = 1;
  } else if (esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP && !esp_test_settled && std::isfinite(dt)) {
    esp_test_brake_elapsed += dt;
  }
  bool hold_just_locked = false;
  bool level_entered = false;
  const bool distance_braking = esp_test_brake_distance > 0 &&
      esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP;
  if (distance_braking) {
    // x0 pitch is quaternion Rodrigues, opposite the telemetry Euler-pitch
    // convention. Negative local pitch and body-Y rate brake forward travel.
    level_entered = tinyBrakeLevelStep(&esp_test_level, esp_test_speed,
        -2.0f*atanf(x0(4)), -x0(10), dt);
    if (esp_test_level.phase != 0)
      lcx(4) = -0.069926812f; // tan(8deg/2): stop requesting a deep braking tilt.
  }
  if (esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP &&
      !esp_test_settled) {
    hold_just_locked = tinympcBrakeHoldUpdate(&esp_test_hold_locked,
        &esp_test_hold.x(), &esp_test_hold.y(), position.x(), position.y(),
        velocity.x(), velocity.y());
    // A sampled zero crossing can skip the narrow speed band. Capture once
    // on reversal too, rather than dragging the hold target backward forever.
    if (distance_braking && !esp_test_hold_locked && esp_test_speed <= 0) {
      esp_test_hold_locked = true;
      hold_just_locked = true;
    }
    // The rearward obstacle-clearance offset is specific to the vision test.
    // A distance brake must not command a second, backward leg after stopping.
    if (hold_just_locked && esp_test_brake_distance == 0.0f)
      esp_test_hold -= 0.5f * forward;
    esp_test_settle_s = velocity.norm() < 0.10f
        ? esp_test_settle_s + dt : 0.0f;
    esp_test_settled = esp_test_settle_s >= 0.20f;
  }
  bool resumed_after_danger = false;
  if (esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP &&
      esp_test_state.reason == TINYMPC_ESPNET_STRAIGHT_DANGER &&
      esp_test_settled && requested && !mission_limit) {
    const bool dangerous = esp_test_fresh &&
        collision.probability[1] > TINYMPC_VISION_BRAKE_THRESHOLD;
    if (tinympcVisionResumeUpdate(&esp_test_resume_state, esp_test_fresh,
        dangerous, collision.sequence)) {
      const float mission_height = esp_test_hold.z();
      tinympcEspnetStraightInit(&esp_test_state);
      esp_test_state.phase = TINYMPC_ESPNET_STRAIGHT_RUN;
      esp_test_state.previous_run = true;
      esp_test_state.v = TINYMPC_VISION_TARGET_SPEED;
      esp_test_origin = position;
      esp_test_origin.z() = mission_height;
      esp_test_hold = esp_test_origin;
      esp_test_distance = 0.0f;
      esp_test_brake_elapsed = 0.0f;
      esp_test_settle_s = 0.0f;
      esp_test_settled = false;
      esp_test_hold_locked = false;
      esp_test_resume_state = {};
      esp_test_mission_last_position = position;
      resumed_after_danger = true;
    }
  } else if (esp_test_state.phase != TINYMPC_ESPNET_STRAIGHT_STOP ||
             esp_test_state.reason != TINYMPC_ESPNET_STRAIGHT_DANGER) {
    esp_test_resume_state = {};
  }
  // Track measured forward speed, including hover and braking.
  esp_test_speed = forward.dot(velocity);
  if (std::isfinite(esp_test_speed))
    esp_test_peak_speed = fmaxf(esp_test_peak_speed, esp_test_speed);
  EspnetConsoleEvent event = {};
  if (previous_phase != esp_test_state.phase)
    event.type = esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_RUN ? 1 : 2;
  else if (requested && esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_IDLE)
    event.type = 4;
  if (hold_just_locked && !event.type)
    event.type = esp_test_brake_distance == 0.0f ? 10 : 12;
  if (level_entered && !event.type) event.type = 13;
  if (!previously_settled && esp_test_settled) event.type = 3;
  if (resumed_after_danger) event.type = 9;
  if (esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_STOP &&
      (esp_test_turn_stage >= 1 && esp_test_turn_stage <= 3)) {
    const float actual_yaw = quat2rpy(qnormalize(mkquat(
        state.attitudeQuaternion.x, state.attitudeQuaternion.y,
        state.attitudeQuaternion.z, state.attitudeQuaternion.w))).z;
    const bool safe_side = vision.valid && vision.dangerous_sectors < 3 &&
        (esp_test_turn_stage == 3 || (esp_test_turn_direction > 0 ? collision.probability[0] <= .9f : collision.probability[2] <= .9f));
    if (!safe_side || !requested || mission_limit) {
      esp_test_turn_stage = 4;
      esp_test_run = 0;
      if (!requested) esp_test_state.reason = TINYMPC_ESPNET_STRAIGHT_CANCEL;
      else if (!vision.valid && !mission_limit) esp_test_state.reason = TINYMPC_ESPNET_STRAIGHT_STALE;
      esp_test_yaw_reference = actual_yaw;
      event.type = 7;
    } else if (esp_test_turn_stage == 3) {
      if (tinympcVisionResumeClear(esp_test_fresh, collision.probability[0],
          collision.probability[1], collision.probability[2], velocity.norm(), dt, &esp_test_clear_s)) {
        // Keep the planned turned heading and start a new position trajectory.
        tinympcEspnetStraightInit(&esp_test_state);
        esp_test_state.previous_run = false;
        esp_test_resume_heading = true;
        esp_test_run = 1;
        esp_test_turn_stage = 0;
        event.type = 8;
      }
    } else if (esp_test_settled && previously_settled) {
      if (esp_test_turn_stage == 1) {
        esp_test_turn_goal = tinympcVisionTurnHeading(esp_test_yaw, esp_test_turn_direction);
        esp_test_yaw_reference = actual_yaw;
        esp_test_turn_stage = 2;
        event.type = 5;
      }
      esp_test_yaw_reference = tinympcVisionYawAdvance(esp_test_yaw_reference, esp_test_turn_goal, dt);
      const float yaw_error = atan2f(sinf(esp_test_turn_goal-actual_yaw), cosf(esp_test_turn_goal-actual_yaw));
      const float reference_error = atan2f(sinf(esp_test_turn_goal-esp_test_yaw_reference), cosf(esp_test_turn_goal-esp_test_yaw_reference));
      esp_test_turn_settle = fabsf(yaw_error) < radians(3.0f) && fabsf(reference_error) < .001f
          ? esp_test_turn_settle + dt : 0.0f;
      if (esp_test_turn_settle >= .2f) {
        esp_test_turn_stage = 3;
        esp_test_yaw = esp_test_turn_goal;
        esp_test_yaw_reference = esp_test_turn_goal;
        esp_test_origin = esp_test_hold;
        esp_test_state.s = 0.0f;
        esp_test_clear_s = 0.0f;
        event.type = 6;
      }
    }
  }
  if (esp_test_run == 0) esp_test_mission_active = false;
  event.danger_count = vision.valid ? tinympcVisionBrakeDangerCount(
      collision.probability[0], collision.probability[1], collision.probability[2]) : 0;
  event.turn_direction = esp_test_turn_direction;
  if (event.type) {
    auto milli = [](float value) -> int {
      return std::isfinite(value) ? (int)(fmaxf(-1000.0f, fminf(1000.0f, value))*1000) : 0;
    };
    event.pending = true;
    if (event.type == 1 && esp_test_brake_distance != 0.0f) {
      event.type = 11;
      event.target_mm_s = milli(esp_test_target_speed);
      event.brake_distance_mm = milli(esp_test_brake_distance);
    }
    event.reason = esp_test_state.reason;
    event.speed_mm_s = milli(esp_test_speed);
    event.peak_mm_s = milli(esp_test_peak_speed);
    event.time_ms = milli(esp_test_brake_elapsed);
    event.travel_mm = milli(esp_test_distance - esp_test_brake_start_distance);
    taskENTER_CRITICAL();
    const unsigned next = (espnet_event_write + 1u) % 8u;
    if (next != espnet_event_read) {
      espnet_console_events[espnet_event_write] = event;
      espnet_event_write = next;
    }
    taskEXIT_CRITICAL();
  }
  esp_test_phase = (uint8_t)esp_test_state.phase;
  esp_test_reason = (uint8_t)esp_test_state.reason;
  const Eigen::Vector3f frame_origin(active_local_frame.origin_x,
      active_local_frame.origin_y, active_local_frame.origin_z);
  const struct vec reference_rodrigues = worldQuaternionToLocalRodrigues(
      active_local_frame, rpy2quat(mkvec(0.0f, 0.0f, esp_test_yaw_reference)));
  float reference_s = esp_test_state.s;
  float reference_v = esp_test_state.v;
  const float actual_yaw = quat2rpy(qnormalize(mkquat(
      state.attitudeQuaternion.x, state.attitudeQuaternion.y,
      state.attitudeQuaternion.z, state.attitudeQuaternion.w))).z;
  const Eigen::Vector3f camera_left(-sinf(actual_yaw), cosf(actual_yaw), 0);
  Eigen::Vector3f gate_velocity = camera_left*gate_servo.lateral +
      Eigen::Vector3f(0, 0, gate_servo.vertical) + forward*gate_servo.forward;
  if (position.z() <= .25f && gate_velocity.z() < 0) gate_velocity.z() = 0;
  if (position.z() >= 1.45f && gate_velocity.z() > 0) gate_velocity.z() = 0;
  for (int k = 0; k < NHORIZON; ++k) {
    const bool moving = esp_test_state.phase == TINYMPC_ESPNET_STRAIGHT_RUN;
    Eigen::Vector3f target = moving
        ? esp_test_origin + forward * reference_s : esp_test_hold;
    const bool leveling = distance_braking && esp_test_level.phase == 1;
    // During recovery, match measured horizontal velocity so its tracking cost
    // does not fight the level-attitude/zero-rate goal. Taper this relief to
    // zero within 0.8 s, then resume stationary position hold.
    const Eigen::Vector3f level_velocity = leveling
        ? forward * fmaxf(0.0f, esp_test_speed) *
            fmaxf(0.0f, 1.0f-esp_test_level.elapsed/.8f)
        : Eigen::Vector3f::Zero().eval();
    if (leveling && !esp_test_hold_locked)
      target = Eigen::Vector3f(position.x(),position.y(),esp_test_hold.z()) + level_velocity*(k*DT);
    if (gate_active) {
      target = gate_servo.phase == GATE_PASS
          ? gate_pass_origin + forward*(.5f*(gate_servo.elapsed+k*DT))
          : position + gate_velocity*(k*DT);
    }
    Xref[k].setZero();
    Xref[k].head<3>() = worldVectorToLocal(active_local_frame,
                                          target - frame_origin);
    Xref[k](3) = reference_rodrigues.x;
    Xref[k](4) = reference_rodrigues.y;
    Xref[k](5) = reference_rodrigues.z;
    Xref[k].segment<3>(6) = worldVectorToLocal(active_local_frame,
        gate_active ? gate_velocity : leveling ? level_velocity : forward * (moving ? reference_v : 0.0f));
    if (k < NHORIZON - 1) Uref[k] = ug;
    if (moving) reference_s += reference_v * DT;
  }
}

// Firmware's C macros use void* for function pointers and mutable string
// declarations. Register the identical tables with C++-typed null pointers.
static struct param_s dg_params[] __attribute__((section(".param.dgAvoid"),used)) = {
  {PARAM_GROUP|PARAM_START,0,const_cast<char*>("dgAvoid"),nullptr,nullptr,nullptr},
  {PARAM_UINT8,0,const_cast<char*>("enable"),&dg_enable,nullptr,nullptr},
  {PARAM_FLOAT,0,const_cast<char*>("speed"),&dg_speed,nullptr,nullptr},
  {PARAM_FLOAT,0,const_cast<char*>("clearance"),&dg_clearance,nullptr,nullptr},
  {PARAM_FLOAT,0,const_cast<char*>("raySlope"),&dg_ray_slope,nullptr,nullptr},
  {PARAM_FLOAT,0,const_cast<char*>("range"),&dg_activation,nullptr,nullptr},
  {PARAM_GROUP|PARAM_STOP,0,const_cast<char*>("stop_dgAvoid"),nullptr,nullptr,nullptr},
};
static const struct log_s dg_avoid_logs[] __attribute__((section(".log.dgAvoid"),used)) = {
  {LOG_GROUP|LOG_START,const_cast<char*>("dgAvoid"),nullptr},
  {LOG_UINT8,const_cast<char*>("enabled"),&dg_enable},
  {LOG_UINT8,const_cast<char*>("fresh"),&dg_fresh},
  {LOG_UINT8,const_cast<char*>("count"),&dg_count},
  {LOG_UINT8,const_cast<char*>("mode"),&dg_mode},
  {LOG_UINT8,const_cast<char*>("fault"),&dg_fault},
  {LOG_UINT16,const_cast<char*>("seq"),&dg_sequence},
  {LOG_UINT32,const_cast<char*>("ageMs"),&dg_age_ms},
  {LOG_FLOAT,const_cast<char*>("left"),&dg_depth[0]},
  {LOG_FLOAT,const_cast<char*>("center"),&dg_depth[1]},
  {LOG_FLOAT,const_cast<char*>("right"),&dg_depth[2]},
  {LOG_FLOAT,const_cast<char*>("bound0"),&dg_local_b[0]},
  {LOG_FLOAT,const_cast<char*>("bound1"),&dg_local_b[1]},
  {LOG_FLOAT,const_cast<char*>("violation"),&dg_violation},
  {LOG_FLOAT,const_cast<char*>("cmdSpeed"),&dg_command_speed},
  {LOG_GROUP|LOG_STOP,const_cast<char*>("stop_dgAvoid"),nullptr},
};
static struct param_s esp_test_params[]
    __attribute__((section(".param.espTest"), used)) = {
  {PARAM_GROUP | PARAM_START, 0, const_cast<char*>("espTest"), nullptr, nullptr, nullptr},
  {PARAM_UINT8, 0, const_cast<char*>("run"), &esp_test_run, nullptr, nullptr},
  {PARAM_UINT8, 0, const_cast<char*>("external"), &esp_test_external, nullptr, nullptr},
  {PARAM_FLOAT, 0, const_cast<char*>("speed"), &esp_test_config_speed, nullptr, nullptr},
  {PARAM_FLOAT, 0, const_cast<char*>("distance"), &esp_test_config_distance, nullptr, nullptr},
  {PARAM_GROUP | PARAM_STOP, 0, const_cast<char*>("stop_espTest"), nullptr, nullptr, nullptr},
};
static const struct log_s esp_test_logs[]
    __attribute__((section(".log.espTest"), used)) = {
  {LOG_GROUP | LOG_START, const_cast<char*>("espTest"), nullptr},
  {LOG_UINT16, const_cast<char*>("handoff"), &esp_test_handoff},
  {LOG_UINT8, const_cast<char*>("phase"), &esp_test_phase},
  {LOG_UINT8, const_cast<char*>("run"), &esp_test_run},
  {LOG_UINT8, const_cast<char*>("reason"), &esp_test_reason},
  {LOG_UINT8, const_cast<char*>("brakeFrames"), &esp_test_state.brake_frames},
  {LOG_UINT8, const_cast<char*>("clearFrames"), &esp_test_resume_state.clear_frames},
  {LOG_UINT8, const_cast<char*>("holdLocked"), &esp_test_hold_locked},
  {LOG_UINT8, const_cast<char*>("fresh"), &esp_test_fresh},
  {LOG_FLOAT, const_cast<char*>("center"), &esp_test_center},
  {LOG_FLOAT, const_cast<char*>("distance"), &esp_test_distance},
  {LOG_FLOAT, const_cast<char*>("speed"), &esp_test_speed},
  {LOG_UINT32, const_cast<char*>("ageMs"), &esp_test_age_ms},
  {LOG_GROUP | LOG_STOP, const_cast<char*>("stop_espTest"), nullptr},
};
static const struct log_s gate_servo_logs[]
    __attribute__((section(".log.gateNav"), used)) = {
  {LOG_GROUP | LOG_START, const_cast<char*>("gateNav"), nullptr},
  {LOG_UINT8, const_cast<char*>("phase"), &gate_servo.phase},
  {LOG_UINT8, const_cast<char*>("rails"), &gate_servo.matched},
  {LOG_UINT8, const_cast<char*>("center"), &gate_servo.centered},
  {LOG_UINT8, const_cast<char*>("reject"), &gate_servo.reject},
  {LOG_FLOAT, const_cast<char*>("errX"), &gate_servo.ex},
  {LOG_FLOAT, const_cast<char*>("errY"), &gate_servo.ey},
  {LOG_FLOAT, const_cast<char*>("distance"), &gate_distance},
  {LOG_FLOAT, const_cast<char*>("vLeft"), &gate_servo.lateral},
  {LOG_FLOAT, const_cast<char*>("vUp"), &gate_servo.vertical},
  {LOG_GROUP | LOG_STOP, const_cast<char*>("stop_gateNav"), nullptr},
};

static const struct log_s mpc_constraint_logs[]
    __attribute__((section(".log.mpcLimit"), used)) = {
  {LOG_GROUP | LOG_START, const_cast<char*>("mpcLimit"), nullptr},
  {LOG_FLOAT, const_cast<char*>("tiltErr"), &mpc_constraints[0]},
  {LOG_FLOAT, const_cast<char*>("rateErr"), &mpc_constraints[1]},
  {LOG_FLOAT, const_cast<char*>("primal"), &mpc_constraints[2]},
  {LOG_FLOAT, const_cast<char*>("solveUs"), &mpc_constraints[3]},
  {LOG_GROUP | LOG_STOP, const_cast<char*>("stop_mpcLimit"), nullptr},
};
static bool esp_test_initialized;
static uint32_t esp_test_last_control_tick;
// Numeric telemetry avoids console bursts in the stabilizer task.
static float mpc_direction[9];
static const struct log_s mpc_direction_logs[]
    __attribute__((section(".log.mpcDir"), used)) = {
  {LOG_GROUP | LOG_START, const_cast<char*>("mpcDir"), nullptr},
  {LOG_FLOAT, const_cast<char*>("refX"), &mpc_direction[0]},
  {LOG_FLOAT, const_cast<char*>("refVx"), &mpc_direction[1]},
  {LOG_FLOAT, const_cast<char*>("vx"), &mpc_direction[2]},
  {LOG_FLOAT, const_cast<char*>("pitchR"), &mpc_direction[3]},
  {LOG_FLOAT, const_cast<char*>("wy"), &mpc_direction[4]},
  {LOG_FLOAT, const_cast<char*>("u0"), &mpc_direction[5]},
  {LOG_FLOAT, const_cast<char*>("u1"), &mpc_direction[6]},
  {LOG_FLOAT, const_cast<char*>("u2"), &mpc_direction[7]},
  {LOG_FLOAT, const_cast<char*>("u3"), &mpc_direction[8]},
  {LOG_GROUP | LOG_STOP, const_cast<char*>("stop_mpcDir"), nullptr},
};
#endif

void updateHorizonReference(const setpoint_t *setpoint) {
  // Full-state packets carry quaternion attitude, velocity and body rates.
  // Advance their reference consistently over the MPC prediction horizon.
  if (setpoint->mode.quat == modeAbs) {
    const auto &q = setpoint->attitudeQuaternion;
    const float yaw = quat2rpy(qnormalize(mkquat(q.x, q.y, q.z, q.w))).z;
    const float yaw_rate = radians(setpoint->attitudeRate.yaw);
    for (int k = 0; k < NHORIZON; ++k) {
      const float t = k * DT;
      const Eigen::Vector3f velocity(setpoint->velocity.x,
          setpoint->velocity.y, setpoint->velocity.z);
      const Eigen::Vector3f acceleration(setpoint->acceleration.x,
          setpoint->acceleration.y, setpoint->acceleration.z);
      setLocalReferenceState(Xref[k],
          Eigen::Vector3f(setpoint->position.x, setpoint->position.y,
              setpoint->position.z) + velocity*t + acceleration*(0.5f*t*t),
          rpy2quat(mkvec(0, 0, yaw + yaw_rate*t)),
          velocity + acceleration*t, Eigen::Vector3f(0, 0, yaw_rate));
      if (k < NHORIZON - 1) Uref[k] = ug;
    }
    return;
  }
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = step / traj_hold;
    }
    const bool trajectory_complete = traj_idx == traj_length - 1;
    // Even a held world reference must be expressed in this solve's frame.
    for (int i = 0; i < NHORIZON; ++i) {
      // Retain the legacy 12-state table: reconstruct its Rodrigues attitude
      // before transforming, rather than replacing it with a new trajectory.
      const float* reference = X_ref_data[traj_idx];
      const struct quat reference_attitude = qnormalize(mkquat(
          reference[3], reference[4], reference[5], 1.0f));
      // At the closing point, command a level stationary hold at its yaw.
      const struct quat target_attitude = trajectory_complete
          ? rpy2quat(mkvec(0.0f, 0.0f, quat2rpy(reference_attitude).z))
          : reference_attitude;
      setLocalReferenceState(Xref[i],
          Eigen::Vector3f(reference[0], reference[1], reference[2]),
          target_attitude,
          trajectory_complete ? Eigen::Vector3f::Zero().eval()
              : Eigen::Vector3f(reference[6], reference[7], reference[8]),
          trajectory_complete ? Eigen::Vector3f::Zero().eval()
              : Eigen::Vector3f(reference[9], reference[10], reference[11]));
      if (i < NHORIZON - 1) {
        if (trajectory_complete) {
          Uref[i] = ug;
          continue;
        }
        for (int j = 0; j < NINPUTS; ++j) {
          const float legacy_command =
              legacy_hover_command[j] + U_ref_data[traj_idx][j];
          Uref[i](j) =
              tinympc_generated_normalized_command_to_thrust(legacy_command)
              - tinympc_generated_normalized_command_to_thrust(
                  legacy_hover_command[j]);
        }
      }
    }
  }
  else {
    const struct quat reference_attitude = rpy2quat(mkvec(
        radians(setpoint->attitude.roll), radians(setpoint->attitude.pitch),
        radians(setpoint->attitude.yaw)));
    setLocalReferenceState(xg,
        Eigen::Vector3f(setpoint->position.x, setpoint->position.y, setpoint->position.z),
        reference_attitude,
        Eigen::Vector3f(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z),
        Eigen::Vector3f(radians(setpoint->attitudeRate.roll),
            radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw)));
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
    // // xg(1) = 1.0;
    // // xg(2) = 2.0;
  }

  if (en_traj && traj_idx < traj_length - 1) {
    step += 1;
  }
}

// Half-space constraint function removed for basic functionality test

// Reset only the bounded input warm start on controller reselection. Matrix
// setup stays in appMain, outside the active stabilizer handoff.
static void resetMpcInputWarmStart() {
  for (int k = 0; k < NHORIZON - 1; ++k) {
    Uref[k] = ug;
    Uhrz[k] = ug;
    ZU[k] = ug;
    ZU_new[k] = ug;
    YU[k].setZero();
  }
}

static void initializeMpcSolver() {
  /* Start MPC initialization*/
  loadGeneratedSolverData();

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 1, DT, &A, &B, &f);
  tiny_InitSettings(&stgs);
  stgs.rho_init = attitude_cache_rho;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct (pass 0 for state constraints - not used)
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
  data.q_base = qBase; data.terminal_base = &terminalBase;
  data.coeff_d2p_zero = true; // Generated cache explicitly contains zeros.
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  lcx.setConstant(-1e6f); ucx.setConstant(1e6f);
  lcx(3) = -tiltRodriguesBound; ucx(3) = tiltRodriguesBound;
  lcx(4) = -pitchRodriguesBound; ucx(4) = pitchRodriguesBound;
  for (int i = 9; i <= 11; ++i) { lcx(i) = -bodyRateBound; ucx(i) = bodyRateBound; }
  lcx(11) = -yawRateBound; ucx(11) = yawRateBound;
  for (int i = 0; i < NSTATES; ++i) stateConstraintWeights(i) = attitude_cache_state_weights[i];
  data.state_constraint_weights = &stateConstraintWeights;
  tiny_SetStateBound(&work, &Acx, &lcx, &ucx);
  for (int k = 0; k < NHORIZON; ++k) {
    data.en_hs[k] = 0; ZX[k].setZero(); ZX_new[k].setZero(); YX[k].setZero();
  }
  resetMpcInputWarmStart();

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 1;  // Predicted tilt and body-rate bounds
  stgs.max_iter = 5; // Bounded runtime; residual/violation telemetry exposes incomplete solves.
  stgs.verbose = 0;
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 1e-3;
  stgs.tol_abs_prim = 1e-3;

  /* End of MPC initialization */  
}

#ifdef TINYMPC_BENCHMARK_ONLY
#ifdef TINYMPC_DEPTHGATE_BENCHMARK_ONLY
#include "tinympc_depthgate_benchmark.inc"
#else
#include "tinympc_benchmark.inc"
#endif
#endif

void controllerOutOfTreeInit(void) {
  if (mpc_solver_ready) resetMpcInputWarmStart();
#if TINYMPC_ESPNET_STRAIGHT_TEST
  esp_test_initialized = false;
  esp_test_run = 0;
#endif
  step = 0;
  traj_idx = 0;
  
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  if (!mpc_solver_ready) {
    control->controlMode = controlModePWM;
    for (int motor = 0; motor < 4; ++motor) control->normalizedForces[motor] = 0.0f;
    return;
  }
#if TINYMPC_ESPNET_STRAIGHT_TEST
  if (setpoint->mode.z == modeDisable || !supervisorAreMotorsAllowedToRun()) {
    esp_test_initialized = false;
    esp_test_run = 0;
    control->controlMode = controlModePWM;
    for (int motor = 0; motor < 4; ++motor) control->normalizedForces[motor] = 0.0f;
    return;
  }
  if (tick - esp_test_last_control_tick > M2T(200)) esp_test_initialized = false;
  esp_test_last_control_tick = tick;
#endif
  // Get current time
  startTimestamp = usecTimestamp();

  /* Controller rate */
  if (RATE_DO_EXECUTE(MPC_RATE, tick)
#if TINYMPC_ESPNET_STRAIGHT_TEST
      || !esp_test_initialized // Never reuse a pre-handoff motor solution.
#endif
      ) {
    // Recenter measurements and references together, only at a solve boundary.
    updateInitialState(sensors, state);
    // Get command reference
#if TINYMPC_ESPNET_STRAIGHT_TEST
    if (!esp_test_initialized) {
      resetEspnetStraightTest(*state, tick, active_local_frame.yaw_world);
      esp_test_initialized = true;
    }
    selectDepthGateCache(dg_enable!=0);
    if (dg_enable) updateDepthGateReference(*state,tick);
    else if (esp_test_external) updateHorizonReference(setpoint);
    else updateEspnetStraightReference(*state, tick);
    if (!dg_enable) {
      dg_tick=0;
      for(int k=0;k<NHORIZON;++k) data.count_xy_hs[k]=0;
    }
#else
    updateHorizonReference(setpoint);
#endif

    // Cold-start both input and state ADMM variables together. Reusing only
    // input warm starts with reset state duals failed the high-rate regression.
    resetMpcInputWarmStart();
    /* Reinitialize state slacks in the newly recentered frame; never reuse
     * position/attitude duals from the previous local frame. Measured x0 stays
     * untouched, including when it is already outside the predicted bounds. */
    ZX_new[0] = x0; YX[0].setZero();
    for (int k = 1; k < NHORIZON; ++k) {
      ZX_new[k] = A * ZX_new[k-1] + B * ZU_new[k-1] + f;
      YX[k].setZero();
    }
    for (int k = 1; k < NHORIZON; ++k)
      ZX_new[k] = ZX_new[k].cwiseMin(ucx).cwiseMax(lcx);
    /* MPC solve */
    // Solve optimization problem using ADMM
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
    mpc_constraints[0] = mpc_constraints[1] = 0.0f;
#if TINYMPC_ESPNET_STRAIGHT_TEST
    dg_violation=0.f;
#endif
    // Roll out the projected actuator commands, not just the state slack.
    VectorNf predicted = x0;
    for (int k = 1; k < NHORIZON; ++k) {
      predicted = (A * predicted + B * ZU_new[k-1] + f).eval();
#if TINYMPC_ESPNET_STRAIGHT_TEST
      for(int j=0;j<data.count_xy_hs[k];++j)
        dg_violation=fmaxf(dg_violation,data.a_xy_hs[k][j].dot(predicted.head<2>())-data.b_xy_hs[k][j]);
#endif
      for (int i = 3; i <= 4; ++i)
        mpc_constraints[0] = fmaxf(mpc_constraints[0], fabsf(predicted(i)) - ucx(i));
      for (int i = 9; i <= 11; ++i)
        mpc_constraints[1] = fmaxf(mpc_constraints[1], fabsf(predicted(i)) - ucx(i));
    }
    mpc_constraints[2] = info.pri_res;
    mpc_constraints[3] = (float)(usecTimestamp() - startTimestamp);
#if TINYMPC_ESPNET_STRAIGHT_TEST
    if(dg_enable && esp_test_run && !dg_fault && (data.xy_hs_projection_failed ||
        !predicted.allFinite() || dg_violation>.05f)) {
      dg_fault=6;
      // Latch a hold for the next 100 Hz update. Never add an unbudgeted
      // second solve; finite-iteration violation remains visible in telemetry.
    }
#endif
 
#if TINYMPC_ESPNET_STRAIGHT_TEST
    mpc_direction[0] = Xref[0](0);
    mpc_direction[1] = Xref[0](6);
    mpc_direction[2] = x0(6);
    mpc_direction[3] = x0(4);
    mpc_direction[4] = x0(10);
    for (int i = 0; i < 4; ++i) mpc_direction[5+i] = ZU_new[0](i);
    // Publish a coherent post-solve snapshot even in IDLE and settled hover.
    // Only appMain performs console I/O, at 2 Hz.
    if (tick - esp_test_speed_print_tick >= M2T(500)) {
      auto milli = [](float value) -> int {
        return std::isfinite(value)
            ? (int)(fmaxf(-1000.0f, fminf(1000.0f, value)) * 1000.0f) : 0;
      };
      EspnetSpeedEvent speed = {};
      speed.pending = true;
      speed.forward_mm_s = milli(esp_test_speed);
      speed.peak_mm_s = milli(esp_test_peak_speed);
      speed.target_mm_s = milli(esp_test_state.v);
      speed.error_x_mm = milli(Xref[0](0));
      speed.pitch_mrad = milli(x0(4));
      speed.pitch_diff_mn = milli(-ZU_new[0](0) + ZU_new[0](1)
          + ZU_new[0](2) - ZU_new[0](3));
      speed.phase = esp_test_phase;
      taskENTER_CRITICAL();
      espnet_speed_event = speed;
      taskEXIT_CRITICAL();
      esp_test_speed_print_tick = tick;
    }
#endif
    result =  info.status_val * info.iter;
    
    

  }

  /* Output control — pure MPC, apply projected ADMM solution directly */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] =
        tinympc_generated_thrust_to_normalized_command(
            ZU_new[0](0) + tinympc_generated_physical_hover_thrust[0] + mpc_hover_trim[0]);
    control->normalizedForces[1] =
        tinympc_generated_thrust_to_normalized_command(
            ZU_new[0](1) + tinympc_generated_physical_hover_thrust[1] + mpc_hover_trim[1]);
    control->normalizedForces[2] =
        tinympc_generated_thrust_to_normalized_command(
            ZU_new[0](2) + tinympc_generated_physical_hover_thrust[2] + mpc_hover_trim[2]);
    control->normalizedForces[3] =
        tinympc_generated_thrust_to_normalized_command(
            ZU_new[0](3) + tinympc_generated_physical_hover_thrust[3] + mpc_hover_trim[3]);
  }
  control->controlMode = controlModePWM;

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;
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
