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
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM

#include "cpx.h"                  // AI-deck (GAP8) CPX bridge
#include "cpx_internal_router.h"  // cpxSendPacketBlockingTimeout()

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"
#define TINYMPC_TASK_STACKSIZE        (3 * configMINIMAL_STACK_SIZE)

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

// Macro variables - define locally to avoid dependency issues
#define DT 0.002f       // dt
#define NHORIZON 25     // horizon steps (must match constants.h if used)
#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

/* Include trajectory to track */
#include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"  // Large circle (1m radius)
// #include "traj_circle_small.h"  // Small circle (0.5m radius)
// #include "traj_perching.h"
//#include "traj_straight_line.h"  // Straight line (0,0,0.5) to (1,0,0.5)

// Precomputed data and cache, in params_*.h
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixMNf Klqr;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1]; 
static VectorMf Ulqr;
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
static float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};  // cf1
// static float u_hover[4] = {0.7467, 0.667f, 0.78, 0.7f};  // cf2 not correct
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = true;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
//static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

// ---- Vision-servo tunables
// which is a C TU because PARAM_ADD string macros don't compile cleanly here) ----
extern uint8_t  visEnable;      // 0 = trajectory/setpoint, 1 = visual servo
extern float    visTargetZ;     // m, altitude to hold while servoing
extern uint16_t visTargetArea;  // px, desired orange-blob area (standoff setpoint)
extern float    visKpYaw;       // rad of yaw-goal offset per pixel of x error
extern float    visKpFwd;       // m of forward-goal offset per pixel of area error
extern float    visFwdMax;      // m, clamp on forward goal offset
extern float    visYawMax;      // rad, clamp on yaw goal offset
extern uint16_t visTimeoutMs;   // detection considered stale after this

// Telemetry (defined here, logged by vis_servo_params.c).
float   vis_log_fwd   = 0.0f;
float   vis_log_dyaw  = 0.0f;
uint8_t vis_log_fresh = 0;

// Vision target frame width (centroid space) — must match the GAP8 output frame.
#define VIS_FRAME_WIDTH 160

// Basic mode - no obstacle avoidance constraints

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
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref[i](j) = U_ref_data[traj_idx][j];
          }          
        }
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
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref[0](2)));

  // Trajectory progression
  if (en_traj) {
    if (traj_idx >= traj_length - 1 - NHORIZON + 1) { 
      // Reached end of trajectory - hold at final position
      // Don't reset step, just stay at the end
    } 
    else {
      step += 1;
    }
  }
}

// Half-space constraint function removed for basic functionality test

// ====================== AI-deck (GAP8) CPX bridge ============
// Matches the protocol in esp_color_object/comms/comms_deck.c (GAP8 firmware).
// STM32 sends an 8-byte TriggerRequest, GAP8 captures+processes a frame and
// replies with a 26-byte DetectionResponse.
#define AID_PROTO_MAGIC          0xA5u
#define AID_PROTO_VERSION        0x01u
#define AID_MSG_TRIGGER          0x01u
#define AID_MSG_DETECTION        0x81u
#define AID_MSG_ERROR            0xE1u
#define AID_STATUS_TRIGGERED        (1u << 0)
#define AID_STATUS_FOUND            (1u << 1)
#define AID_STATUS_DEPTH_DEFAULTED  (1u << 2)
#define AID_TRIGGER_PERIOD_MPC   10    // every Nth MPC tick (10 -> 10 Hz @ 100 Hz MPC_RATE)
#define AID_TX_TIMEOUT_MS        2     // bounded so MPC tick never stalls on UART

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  msg_type;     // AID_MSG_TRIGGER
  uint8_t  seq;
  uint8_t  flags;
  uint8_t  reserved0;
  uint16_t depth_mm;     // 0 -> GAP8 uses its default
} AidTriggerReq_t;

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  msg_type;     // AID_MSG_DETECTION or AID_MSG_ERROR
  uint8_t  seq;
  uint8_t  status;
  uint8_t  error;
  int16_t  centroid_x;
  int16_t  centroid_y;
  uint16_t bright_pixels;
  int16_t  real_x_mm;
  int16_t  real_y_mm;
  uint16_t real_z_mm;
  uint32_t frame_id;
  uint32_t timestamp_ms;
} AidDetectionResp_t;

static AidDetectionResp_t g_aidDet       = {0};
static volatile uint32_t  g_aidSeq       = 0;   // seqlock for g_aidDet
static volatile uint32_t  g_aidTickRx    = 0;   // FreeRTOS tick at last valid detection
static volatile uint32_t  g_aidBadPkts   = 0;   // bad length / magic / version
static volatile uint32_t  g_aidErrPkts   = 0;   // GAP8 reported an error (msg_type=0xE1)
static volatile uint32_t  g_aidRxOk      = 0;   // count of accepted detection packets
static volatile uint32_t  g_aidTxOk      = 0;   // triggers queued successfully
static volatile uint32_t  g_aidTxFail    = 0;   // trigger send timeouts
static uint8_t            g_aidTxSeq     = 0;   // outgoing trigger seq counter

static void aideckRxCb(const CPXPacket_t* rx) {
  if (rx->route.source != CPX_T_GAP8) return;
  if (rx->dataLength != sizeof(AidDetectionResp_t)) {
    g_aidBadPkts++;
    return;
  }
  AidDetectionResp_t pkt;
  memcpy(&pkt, rx->data, sizeof(pkt));
  if (pkt.magic != AID_PROTO_MAGIC || pkt.version != AID_PROTO_VERSION) {
    g_aidBadPkts++;
    return;
  }
  if (pkt.msg_type == AID_MSG_ERROR) {
    g_aidErrPkts++;
    // Still publish the response so the error/status fields show up in logs.
  } else if (pkt.msg_type != AID_MSG_DETECTION) {
    g_aidBadPkts++;
    return;
  }

  // Seqlock publish: odd seq during write, even when stable.
  uint32_t s = g_aidSeq + 1;
  g_aidSeq = s;
  g_aidDet = pkt;
  g_aidTickRx = xTaskGetTickCount();
  g_aidSeq = s + 1;
  if (pkt.msg_type == AID_MSG_DETECTION) g_aidRxOk++;
}

static void aideckSendTrigger(void) {
  CPXPacket_t tx;
  cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &tx.route);
  AidTriggerReq_t req = {
    .magic       = AID_PROTO_MAGIC,
    .version     = AID_PROTO_VERSION,
    .msg_type    = AID_MSG_TRIGGER,
    .seq         = g_aidTxSeq++,
    .flags       = 0,
    .reserved0   = 0,
    .depth_mm    = 0,    // 0 -> GAP8 uses DEFAULT_DEPTH_MM (500)
  };
  memcpy(tx.data, &req, sizeof(req));
  tx.dataLength = sizeof(req);
  if (cpxSendPacketBlockingTimeout(&tx, AID_TX_TIMEOUT_MS)) g_aidTxOk++;
  else g_aidTxFail++;
}
// ================================================================================

// ====================== Vision reference builder ==========================
static float vis_hold_x = 0.0f, vis_hold_y = 0.0f, vis_hold_yaw = 0.0f;
static bool  vis_have_hold = false;

static inline float vis_clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void buildVisionReference(const state_t *state,
                                 const AidDetectionResp_t *det,
                                 bool found, uint32_t ageMs) {
  const float yaw = quat2rpy(attitude).z;   // current yaw (rad); attitude set in updateInitialState()
  const bool fresh = found && (ageMs <= visTimeoutMs);

  float gx, gy, goalYaw;
  if (fresh) {
    float exPix   = (float)det->centroid_x - (VIS_FRAME_WIDTH / 2.0f);  // + = target to the right
    float areaErr = (float)visTargetArea - (float)det->bright_pixels;   // + = too far (blob too small)
    float fwd  = vis_clampf(visKpFwd * areaErr, -visFwdMax, visFwdMax);
    float dyaw = vis_clampf(-visKpYaw * exPix,  -visYawMax, visYawMax);
    goalYaw = yaw + dyaw;
    gx = state->position.x + fwd * cosf(goalYaw);   // forward offset along goal heading
    gy = state->position.y + fwd * sinf(goalYaw);
    vis_hold_x = gx; vis_hold_y = gy; vis_hold_yaw = goalYaw; vis_have_hold = true;
    vis_log_fwd = fwd; vis_log_dyaw = dyaw; vis_log_fresh = 1;
  } else {
    if (!vis_have_hold) {  // first run without a target: latch current pose
      vis_hold_x = state->position.x; vis_hold_y = state->position.y;
      vis_hold_yaw = yaw; vis_have_hold = true;
    }
    gx = vis_hold_x; gy = vis_hold_y; goalYaw = vis_hold_yaw;
    vis_log_fwd = 0.0f; vis_log_dyaw = 0.0f; vis_log_fresh = 0;
  }

  // Assemble goal state: position (gx,gy,targetZ), level attitude at goalYaw,
  // zero reference velocity and body rates.
  xg(0) = gx; xg(1) = gy; xg(2) = visTargetZ;
  xg(6) = 0.0f; xg(7) = 0.0f; xg(8) = 0.0f;
  xg(9) = 0.0f; xg(10) = 0.0f; xg(11) = 0.0f;
  desired_rpy = mkvec(0.0f, 0.0f, goalYaw);
  attitude = rpy2quat(desired_rpy);
  phi = quat2rp(qnormalize(attitude));
  xg(3) = phi.x; xg(4) = phi.y; xg(5) = phi.z;

  tiny_SetGoalState(&work, Xref, &xg);
  tiny_SetGoalInput(&work, Uref, &ug);
}
// ================================================================================

void controllerOutOfTreeInit(void) {
  /* Start MPC initialization*/

  // Precompute/Cache
  // #include "params_500hz.h"
  #include "params_100hz.h"  // the original stable gains
  // #include "params_constrained.h"

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Original stable rho
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct (pass 0 for state constraints - not used)
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalState(&work, Xref, &xg);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;  // No state constraints for basic test
  stgs.max_iter = 2;        // Original working value
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  Klqr << 
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* End of MPC initialization */
  step = 0;
  traj_iter = 0;

  // Subscribe to AI-deck (GAP8) CPX_F_APP packets. CPX is initialized by the
  // firmware on boot; the callback runs on the CPX FreeRTOS task, not here.
  cpxRegisterAppMessageHandler(aideckRxCb);
  DEBUG_PRINT("AI-deck CPX handler registered\n");

  DEBUG_PRINT("Straight line trajectory (1m forward)\n");
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Get current time
  startTimestamp = usecTimestamp();

  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  updateInitialState(sensors, state);

  /* Controller rate */
  if (RATE_DO_EXECUTE(MPC_RATE, tick)) {
    // When vision is not enabled, we revert back to the fixed setpoints
    if (!visEnable) {
      updateHorizonReference(setpoint);
    }

    // AI-deck: periodically poke GAP8 for a fresh detection -- ONLY when vision
    // servo is enabled. Touching CPX with no AI-deck present (uninitialized CPX
    // queues) asserts in FreeRTOS queue.c and crashes the firmware, so this guard
    // lets pure-MPC control run/tune without the deck. TX is bounded
    // (AID_TX_TIMEOUT_MS) so the MPC tick can never stall on UART.
    static uint32_t aidTrigCounter = 0;
    if (visEnable && (aidTrigCounter++ % AID_TRIGGER_PERIOD_MPC) == 0) {
      aideckSendTrigger();
    }

    // Snapshot latest detection (seqlock — safe wrt CPX rx task). Zero-filled
    // until the first valid response arrives.
    AidDetectionResp_t aidDet;
    uint32_t aidS1, aidS2;
    do {
      aidS1 = g_aidSeq;
      memcpy(&aidDet, &g_aidDet, sizeof(aidDet));
      aidS2 = g_aidSeq;
    } while (aidS1 != aidS2 || (aidS1 & 1u));
    const uint32_t aidAgeMs =
        (xTaskGetTickCount() - g_aidTickRx) * portTICK_PERIOD_MS;
    const bool aidFound = (aidDet.status & AID_STATUS_FOUND) != 0;
    // 
    if (visEnable) {
      buildVisionReference(state, &aidDet, aidFound, aidAgeMs);
    } else {
      (void)aidDet; (void)aidAgeMs; (void)aidFound;
    }

    // Periodic health print so the link is observable from the firmware console.
    static uint32_t aidLogCounter = 0;
    if ((aidLogCounter++ % 500) == 0) {  // every 5s at MPC_RATE=100Hz
      DEBUG_PRINT("AIDECK tx=%lu/%lu rx=%lu bad=%lu err=%lu found=%d cx=%d cy=%d age=%lums\n",
                  (unsigned long)g_aidTxOk,
                  (unsigned long)(g_aidTxOk + g_aidTxFail),
                  (unsigned long)g_aidRxOk,
                  (unsigned long)g_aidBadPkts,
                  (unsigned long)g_aidErrPkts,
                  (int)aidFound,
                  (int)aidDet.centroid_x,
                  (int)aidDet.centroid_y,
                  (unsigned long)aidAgeMs);
    }

    /* MPC solve */
    // Solve optimization problem using ADMM
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
 
    result =  info.status_val * info.iter;
    
    // Detailed logging every 5 seconds (kept sparse to avoid flooding CPX /
    // stealing time from the 1 kHz loop; bump back to %50 for dense debugging).
    static uint32_t mpc_log_counter = 0;
    if (mpc_log_counter % 500 == 0) {  // 100Hz / 500 = every 5s
      DEBUG_PRINT("MPC: pos=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f)\n", 
                  (double)x0(0), (double)x0(1), (double)x0(2),
                  (double)Xref[0](0), (double)Xref[0](1), (double)Xref[0](2));
      DEBUG_PRINT("MPC: u=(%.2f,%.2f,%.2f,%.2f) iter=%d\n",
                  (double)(Uhrz[0](0) + u_hover[0]), (double)(Uhrz[0](1) + u_hover[1]),
                  (double)(Uhrz[0](2) + u_hover[2]), (double)(Uhrz[0](3) + u_hover[3]),
                  info.iter);
    }
    mpc_log_counter++;
    
    // Position logging disabled
    // static uint32_t pos_log_counter = 0;
    // if (pos_log_counter % 50 == 0) {
    //   DEBUG_PRINT("POS: x=%.2f y=%.2f z=%.2f cstr=%d\n", 
    //               (double)x0(0), (double)x0(1), (double)x0(2), obs_constraint_active);
    // }
    // pos_log_counter++;
  }

  /* Output control — pure MPC, apply projected ADMM solution directly */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = ZU_new[0](0) + u_hover[0];  // PWM 0..1
    control->normalizedForces[1] = ZU_new[0](1) + u_hover[1];
    control->normalizedForces[2] = ZU_new[0](2) + u_hover[2];
    control->normalizedForces[3] = ZU_new[0](3) + u_hover[3];
  }
  control->controlMode = controlModePWM;
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[1]));

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

// Live view of the AI-deck detection stream in cfclient.
LOG_GROUP_START(aideck)
LOG_ADD(LOG_INT16,  cx,        &g_aidDet.centroid_x)
LOG_ADD(LOG_INT16,  cy,        &g_aidDet.centroid_y)
LOG_ADD(LOG_UINT16, px,        &g_aidDet.bright_pixels)
LOG_ADD(LOG_INT16,  rx_mm,     &g_aidDet.real_x_mm)
LOG_ADD(LOG_INT16,  ry_mm,     &g_aidDet.real_y_mm)
LOG_ADD(LOG_UINT16, rz_mm,     &g_aidDet.real_z_mm)
LOG_ADD(LOG_UINT8,  status,    &g_aidDet.status)
LOG_ADD(LOG_UINT8,  err,       &g_aidDet.error)
LOG_ADD(LOG_UINT32, frame,     &g_aidDet.frame_id)
LOG_ADD(LOG_UINT32, ts_ms,     &g_aidDet.timestamp_ms)
LOG_ADD(LOG_UINT32, rx_ok,     (uint32_t*)&g_aidRxOk)
LOG_ADD(LOG_UINT32, tx_ok,     (uint32_t*)&g_aidTxOk)
LOG_ADD(LOG_UINT32, tx_fail,   (uint32_t*)&g_aidTxFail)
LOG_ADD(LOG_UINT32, bad,       (uint32_t*)&g_aidBadPkts)
LOG_ADD(LOG_UINT32, err_pkts,  (uint32_t*)&g_aidErrPkts)
LOG_GROUP_STOP(aideck)

#ifdef __cplusplus
}
#endif
