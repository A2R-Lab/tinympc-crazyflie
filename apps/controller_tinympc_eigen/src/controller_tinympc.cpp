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
#include <cmath>
using namespace Eigen;

#include "tinympc_cf_adapter.hpp"
#include "tinympc/psd_support.hpp"

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
#include "psd_params.h"

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

static float psd_disks[PEER_LOCALIZATION_MAX_NEIGHBORS * 3];

enum class PlanMode {
  PSD = 0,
  NOMINAL = 1,
};

struct PlanCache {
  float states[NHORIZON * NX0];
  float inputs[(NHORIZON - 1) * NU0];
  uint32_t start_step;
  int last_iters;
  PlanMode mode;
  bool valid;
};

static PlanCache plan_cache;
static bool psd_constraints_active = false;

struct PeerHistory {
  uint8_t id;
  float x;
  float y;
  uint32_t timestamp;
  float vx;
  float vy;
  bool valid;
};

static PeerHistory peer_history[PEER_LOCALIZATION_MAX_NEIGHBORS];
static float psd_disks_tv[NHORIZON * PEER_LOCALIZATION_MAX_NEIGHBORS * 3];
static int psd_disks_tv_counts[NHORIZON];

// Logging vars are defined in psd_params.c

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

static inline int clamp_index(int idx, int lo, int hi) {
  if (idx < lo) return lo;
  if (idx > hi) return hi;
  return idx;
}

static float signed_distance_point_disks(const VectorBase& x,
                                         const float* disks,
                                         int count) {
  float best = 1e9f;
  for (int i = 0; i < count; ++i) {
    const int base = 3 * i;
    const float dx = x(0) - disks[base + 0];
    const float dy = x(1) - disks[base + 1];
    const float r = disks[base + 2];
    const float sd = std::sqrt(dx * dx + dy * dy) - r;
    if (sd < best) {
      best = sd;
    }
  }
  return best;
}

static void update_peer_history(void) {
  TickType_t now = xTaskGetTickCount();
  for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
    peerLocalizationOtherPosition_t const *other = peerLocalizationGetPositionByIdx(i);
    if (!other || other->id == 0) {
      continue;
    }
    PeerHistory &ph = peer_history[i];
    if (ph.valid && ph.id == other->id) {
      const uint32_t dt_ticks = now - ph.timestamp;
      const float dt = (dt_ticks > 0) ? (0.001f * (float)dt_ticks) : DT;
      if (dt > 1e-6f) {
        ph.vx = (other->pos.x - ph.x) / dt;
        ph.vy = (other->pos.y - ph.y) / dt;
      }
    } else {
      ph.vx = 0.0f;
      ph.vy = 0.0f;
    }
    ph.id = other->id;
    ph.x = other->pos.x;
    ph.y = other->pos.y;
    ph.timestamp = now;
    ph.valid = true;
  }
}

static int predict_disks_per_stage(float* out_disks,
                                   int* out_counts,
                                   int max_per_stage) {
  if (!out_disks || !out_counts || max_per_stage <= 0) {
    return 0;
  }
  const float radius = psd_peer_radius + psd_peer_margin;
  for (int k = 0; k < NHORIZON; ++k) {
    int count = 0;
    float t = DT * (float)k;
    for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS && count < max_per_stage; ++i) {
      const PeerHistory &ph = peer_history[i];
      if (!ph.valid || ph.id == 0) {
        continue;
      }
      if (psd_peer_max_age_ms >= 0) {
        TickType_t age = xTaskGetTickCount() - ph.timestamp;
        if (age > (TickType_t)psd_peer_max_age_ms) {
          continue;
        }
      }
      const int base = (k * max_per_stage + count) * 3;
      out_disks[base + 0] = ph.x + ph.vx * t;
      out_disks[base + 1] = ph.y + ph.vy * t;
      out_disks[base + 2] = radius;
      count++;
    }
    out_counts[k] = count;
  }
  return max_per_stage;
}

static void set_tracking_refs(const PlanCache& plan, uint32_t current_step) {
  if (!plan.valid) {
    return;
  }
  const int max_idx = NHORIZON - 1;
  const int max_u_idx = NHORIZON - 2;
  const int offset = (int)current_step - (int)plan.start_step;

  for (int i = 0; i < NHORIZON; ++i) {
    int idx = clamp_index(offset + i, 0, max_idx);
    VectorBase xb;
    for (int j = 0; j < NX0; ++j) {
      xb(j) = plan.states[idx * NX0 + j];
    }
    float lifted_ref[NSTATES];
    build_lifted_flat(xb, lifted_ref);
    for (int j = 0; j < NSTATES; ++j) {
      Xref_flat[i * NSTATES + j] = lifted_ref[j];
    }
  }

  for (int i = 0; i < NHORIZON - 1; ++i) {
    int idx = clamp_index(offset + i, 0, max_u_idx);
    for (int j = 0; j < NINPUTS; ++j) {
      Uref_flat[i * NINPUTS + j] = 0.0f;
    }
    for (int j = 0; j < NU0; ++j) {
      Uref_flat[i * NINPUTS + j] = plan.inputs[idx * NU0 + j];
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

static void disks_to_vector(const float* disks, int count,
                            std::vector<std::array<tinytype,3>>& out) {
  out.clear();
  out.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const int base = 3 * i;
    out.push_back({{(tinytype)disks[base + 0],
                    (tinytype)disks[base + 1],
                    (tinytype)disks[base + 2]}});
  }
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
  plan_cache.valid = false;
  plan_cache.start_step = 0;
  plan_cache.last_iters = 0;
  plan_cache.mode = PlanMode::NOMINAL;
  psd_constraints_active = false;
  memset(peer_history, 0, sizeof(peer_history));
  memset(psd_disks_tv, 0, sizeof(psd_disks_tv));
  memset(psd_disks_tv_counts, 0, sizeof(psd_disks_tv_counts));

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
    update_peer_history();

    // Command reference for planner seed
    updateHorizonReference(setpoint);

    int disk_count = 0;
    if (en_psd_avoidance) {
      disk_count = gather_psd_disks(psd_disks, psd_max_peers);
      psd_log_disk_count = disk_count;
    }

    const bool need_replan =
      (!plan_cache.valid) ||
      (step - plan_cache.start_step >= psd_replan_stride) ||
      (step >= plan_cache.start_step + NHORIZON - psd_horizon_guard);

    if (need_replan) {
      float sd_seed = (disk_count > 0) ? signed_distance_point_disks(x0, psd_disks, disk_count) : 1e9f;
      float goal_dist = std::sqrt(xg(0) * xg(0) + xg(1) * xg(1));
      float on_thresh = std::min(psd_base_on, goal_dist + psd_goal_on_bias);
      float off_thresh = std::max(on_thresh + psd_off_hysteresis,
                                  std::min(psd_base_off, goal_dist + psd_goal_off_bias));

      if (!psd_constraints_active && sd_seed < on_thresh) {
        psd_constraints_active = true;
      } else if (psd_constraints_active && sd_seed > off_thresh) {
        psd_constraints_active = false;
      }

      tinympc_cf_set_psd_enabled(psd_constraints_active ? 1 : 0);
      if (psd_constraints_active && disk_count > 0) {
        predict_disks_per_stage(psd_disks_tv, psd_disks_tv_counts, psd_max_peers);
        tinympc_cf_set_psd_disks_tv(psd_disks_tv, psd_disks_tv_counts, NHORIZON, psd_max_peers);
      } else {
        tinympc_cf_clear_psd_disks();
      }

      build_lifted_flat(x0, x0_flat);
      tinympc_cf_set_x0(x0_flat, NSTATES);
      tinympc_cf_set_reference(Xref_flat, NSTATES, NHORIZON, Uref_flat, NINPUTS);
      result = (int8_t)tinympc_cf_solve();

      float x0_pred[NSTATES];
      tinympc_cf_get_x_pred_0(x0_pred, NSTATES);
      tinyVector x0_lifted(NSTATES);
      for (int i = 0; i < NSTATES; ++i) {
        x0_lifted(i) = (tinytype)x0_pred[i];
      }
      std::vector<std::array<tinytype,3>> disk_vec;
      disks_to_vector(psd_disks, disk_count, disk_vec);
      PsdCertificate cert = tiny_psd_certificate_2d(x0_lifted, disk_vec, NX0);

      psd_log_trace_gap = (float)cert.trace_gap;
      psd_log_eta_min = (float)cert.eta_min;
      psd_log_certified = cert.certified ? 1 : 0;

      if (cert.certified || !psd_constraints_active) {
        tinympc_cf_get_solution_base_states(plan_cache.states, NX0, NHORIZON);
        tinympc_cf_get_solution_base_inputs(plan_cache.inputs, NU0, NHORIZON);
        plan_cache.start_step = step;
        plan_cache.mode = psd_constraints_active ? PlanMode::PSD : PlanMode::NOMINAL;
        plan_cache.valid = true;
        psd_log_plan_status = 1;
      } else {
        psd_log_plan_status = 0;
      }
      psd_log_plan_mode = (plan_cache.mode == PlanMode::PSD) ? 1 : 0;
    }

    // Tracker solve using plan references
    if (plan_cache.valid) {
      set_tracking_refs(plan_cache, step);
      psd_log_plan_age = (int32_t)(step - plan_cache.start_step);
    }

    tinympc_cf_set_psd_enabled(0);
    build_lifted_flat(x0, x0_flat);
    tinympc_cf_set_x0(x0_flat, NSTATES);
    tinympc_cf_set_reference(Xref_flat, NSTATES, NHORIZON, Uref_flat, NINPUTS);
    result = (int8_t)tinympc_cf_solve();

    tinympc_cf_get_x_pred_1(x1_flat, NSTATES);
    for (int i = 0; i < NSTATES; ++i) {
      x1_lifted(i) = x1_flat[i];
    }

    float x0_track[NSTATES];
    tinympc_cf_get_x_pred_0(x0_track, NSTATES);
    tinyVector x0_track_lifted(NSTATES);
    for (int i = 0; i < NSTATES; ++i) {
      x0_track_lifted(i) = (tinytype)x0_track[i];
    }
    std::vector<std::array<tinytype,3>> disk_vec_track;
    disks_to_vector(psd_disks, disk_count, disk_vec_track);
    PsdCertificate cert_track = tiny_psd_certificate_2d(x0_track_lifted, disk_vec_track, NX0);
    psd_log_certified = cert_track.certified ? 1 : 0;
    psd_log_trace_gap = (float)cert_track.trace_gap;
    psd_log_eta_min = (float)cert_track.eta_min;

    if (!cert_track.certified) {
      x1_lifted(0) = x0(0);
      x1_lifted(1) = x0(1);
      x1_lifted(2) = 0.0f;
      x1_lifted(3) = 0.0f;
    }

    step++;
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
