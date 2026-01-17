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

#include <Eigen.h>
using namespace Eigen;

#include "tinympc_cf_adapter.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "controller_pid.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM
#include "peer_localization.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

// #include "tinympc/tinympc.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Macro variables, model dimensions in tinympc/types.h
#define DT 0.01f       // dt (100 Hz)
#define NHORIZON 25    // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NX0 4          // base state dim: x, y, vx, vy
#define NU0 2          // base input dim: ax, ay
#define NSTATES (NX0 + NX0 * NX0)
#define NINPUTS (NU0 + NX0 * NU0 + NU0 * NX0 + NU0 * NU0)
#define MPC_RATE RATE_100_HZ  // control frequency

#define T_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

using MatrixNf = Matrix<float, NSTATES, NSTATES>;
using MatrixNMf = Matrix<float, NSTATES, NINPUTS>;
using MatrixMNf = Matrix<float, NINPUTS, NSTATES>;
using MatrixMf = Matrix<float, NINPUTS, NINPUTS>;
using VectorNf = Matrix<float, NSTATES, 1>;
using VectorMf = Matrix<float, NINPUTS, 1>;
using VectorBase = Matrix<float, NX0, 1>;
using VectorU0 = Matrix<float, NU0, 1>;

/* Allocate global variables for MPC */
static VectorBase x0;
static VectorBase xg;
static VectorNf x1_lifted;
static VectorU0 u0_base;

static float x0_flat[NSTATES];
static float x1_flat[NSTATES];
static float u0_flat[NINPUTS];
static float Xref_flat[NHORIZON * NSTATES];
static float Uref_flat[(NHORIZON - 1) * NINPUTS];

// Helper variables
static uint64_t startTimestamp;
// static bool isInit = false;  // fix for tracking problem - UNUSED, commented out
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;

// ========== PSD Obstacle Avoidance Configuration ==========
static uint8_t en_psd_avoidance = 0;
static uint8_t psd_max_peers = 3;
static float psd_peer_radius = 0.3f;
static float psd_peer_margin = 0.15f;
static int32_t psd_peer_max_age_ms = 500;
static float psd_rho = 5.0f;

static float psd_disks[PEER_LOCALIZATION_MAX_NEIGHBORS * 3];

static void build_lifted_flat(const VectorBase& xb, float* out_lifted) {
  if (!out_lifted) {
    return;
  }
  for (int i = 0; i < NX0; ++i) {
    out_lifted[i] = xb(i);
  }
  for (int j = 0; j < NX0; ++j) {
    for (int i = 0; i < NX0; ++i) {
      out_lifted[NX0 + j * NX0 + i] = xb(i) * xb(j);
    }
  }
}

static int gather_psd_disks(float* out_disks, int max_disks) {
  if (!out_disks || max_disks <= 0) {
    return 0;
  }
  TickType_t now = xTaskGetTickCount();
  int count = 0;

  uint8_t max_peers = psd_max_peers;
  if (max_peers > PEER_LOCALIZATION_MAX_NEIGHBORS) {
    max_peers = PEER_LOCALIZATION_MAX_NEIGHBORS;
  }
  if (max_peers > (uint8_t)max_disks) {
    max_peers = (uint8_t)max_disks;
  }
  const float radius = psd_peer_radius + psd_peer_margin;

  for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS && count < max_peers; ++i) {
    peerLocalizationOtherPosition_t const *other = peerLocalizationGetPositionByIdx(i);
    if (!other || other->id == 0) {
      continue;
    }
    if (psd_peer_max_age_ms >= 0) {
      TickType_t age = now - other->pos.timestamp;
      if (age > (TickType_t)psd_peer_max_age_ms) {
        continue;
      }
    }
    const int base = 3 * count;
    out_disks[base + 0] = other->pos.x;
    out_disks[base + 1] = other->pos.y;
    out_disks[base + 2] = radius;
    count++;
  }
  return count;
}

void updateInitialState(const sensorData_t *sensors, const state_t *state) {
  (void)sensors;
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->velocity.x;
  x0(3) = state->velocity.y;
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Reference from commander: planar (x, y, vx, vy)
  xg(0) = setpoint->position.x;
  xg(1) = setpoint->position.y;
  xg(2) = setpoint->velocity.x;
  xg(3) = setpoint->velocity.y;

  float lifted_ref[NSTATES];
  build_lifted_flat(xg, lifted_ref);

  for (int i = 0; i < NHORIZON; ++i) {
    for (int j = 0; j < NSTATES; ++j) {
      Xref_flat[i * NSTATES + j] = lifted_ref[j];
    }
    if (i < NHORIZON - 1) {
      for (int j = 0; j < NINPUTS; ++j) {
        Uref_flat[i * NINPUTS + j] = 0.0f;
      }
    }
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref_flat[2]));

  // No internal trajectory in planar PSD mode
}

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/

  tinympc_cf_init_defaults();
  tinympc_cf_set_settings(2, 0);
  memset(Xref_flat, 0, sizeof(Xref_flat));
  memset(Uref_flat, 0, sizeof(Uref_flat));
  memset(x0_flat, 0, sizeof(x0_flat));
  memset(x1_flat, 0, sizeof(x1_flat));
  memset(u0_flat, 0, sizeof(u0_flat));
  x1_lifted.setZero();
  u0_base.setZero();

  // PSD enable (requires a lifted PSD solver codegen)
  if (en_psd_avoidance) {
    int psd_result = tinympc_cf_enable_psd(NX0, NU0, psd_rho);
    if (psd_result == 0) {
      DEBUG_PRINT("PSD enabled (rho=%.2f)\n", (double)psd_rho);
    } else {
      DEBUG_PRINT("PSD enable failed: %d (needs lifted solver)\n", psd_result);
    }
  }

  /* End of MPC initialization */
  en_traj = false;
  step = 0;  
  controllerPidInit();
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
    // Get command reference
    updateHorizonReference(setpoint);

    if (en_psd_avoidance) {
      int disk_count = gather_psd_disks(psd_disks, psd_max_peers);
      tinympc_cf_set_psd_disks(psd_disks, disk_count);
    }

    /* MPC solve */
    build_lifted_flat(x0, x0_flat);
    tinympc_cf_set_x0(x0_flat, NSTATES);
    tinympc_cf_set_reference(Xref_flat, NSTATES, NHORIZON, Uref_flat, NINPUTS);
    result = (int8_t)tinympc_cf_solve();
    tinympc_cf_get_x_pred_1(x1_flat, NSTATES);
    tinympc_cf_get_u0(u0_flat, NINPUTS, TinympcControlOutputKind::ZNEW);

    for (int i = 0; i < NSTATES; ++i) {
      x1_lifted(i) = x1_flat[i];
    }
    for (int j = 0; j < NU0; ++j) {
      u0_base(j) = u0_flat[j];
    }
 
    // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
    // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
    // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
    // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
    // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
    // DEBUG_PRINT("%.2f, %.2f, %.2f, %.2f \n", (double)(Xref_flat[5]), (double)(u0(2)), (double)(u0(3)), (double)(u0(0)));
  }

  // Use PID controller to track MPC-provided planar setpoint
  setpoint_t sp = *setpoint;
  sp.mode.x = modeAbs;
  sp.mode.y = modeAbs;
  sp.position.x = x1_lifted(0);
  sp.position.y = x1_lifted(1);
  sp.velocity.x = x1_lifted(2);
  sp.velocity.y = x1_lifted(3);
  sp.velocity_body = false;
  controllerPid(control, &sp, sensors, state, tick);

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

PARAM_GROUP_START(psdAvoid)
  /**
   * @brief Enable PSD obstacle avoidance
   */
  PARAM_ADD(PARAM_UINT8, enable, &en_psd_avoidance)
  /**
   * @brief Max peers to consider (<= PEER_LOCALIZATION_MAX_NEIGHBORS)
   */
  PARAM_ADD(PARAM_UINT8, maxPeers, &psd_max_peers)
  /**
   * @brief Safety radius around each peer [m]
   */
  PARAM_ADD(PARAM_FLOAT, radius, &psd_peer_radius)
  /**
   * @brief Additional safety margin [m]
   */
  PARAM_ADD(PARAM_FLOAT, margin, &psd_peer_margin)
  /**
   * @brief Max peer localization age [ms], <0 disables filtering
   */
  PARAM_ADD(PARAM_INT32, maxAgeMs, &psd_peer_max_age_ms)
  /**
   * @brief PSD penalty rho
   */
  PARAM_ADD(PARAM_FLOAT, rho, &psd_rho)
PARAM_GROUP_STOP(psdAvoid)

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
