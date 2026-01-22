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

// Macro variables, model dimensions in tinympc/constants.h
#define DT 0.002f       // dt
// NHORIZON defined in constants.h (must match for half-space constraint arrays)
#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

/* Include trajectory to track */
// #include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"  // Large circle (1m radius)
// #include "traj_circle_small.h"  // Small circle (0.5m radius)
// #include "traj_perching.h"
#include "traj_straight_line.h"  // Straight line (0,0,0.5) to (1,0,0.5)

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

// Half-space obstacle constraint mode
// HALFSPACE_CSTR_MODE:
// 0 = off (straight line only)
// 1 = log only (no constraints applied)
// 2 = enable half-space constraints for obstacle avoidance
// HALFSPACE_CSTR_MODE: 0=off, 1=log, 2=ADMM projection, 3=reference push (soft)
#define HALFSPACE_CSTR_MODE 2

// Storage for state constraints (always needed for solver)
static MatrixNf Acx;
static VectorNf lcx;
static VectorNf ucx;
static VectorNf YX[NHORIZON];
static VectorNf ZX[NHORIZON];
static VectorNf ZX_new[NHORIZON];

#if HALFSPACE_CSTR_MODE >= 1
// Obstacle parameters (static disk near the path)
static float obs_cx = 0.5f;   // obstacle center x
static float obs_cy = 0.15f;  // obstacle center y (offset so path brushes past)
static float obs_cz = 0.5f;   // obstacle center z
static float obs_r = 0.10f;   // obstacle radius (smaller = gentler push)
static float obs_activation_dist = 0.5f;  // activate constraint when ref is within this distance
static int obs_horizon_start = 5;  // only apply constraint to horizon steps >= this (softer)
static uint32_t hs_log_counter = 0;
#endif

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

static uint64_t hs_start_time = 0;
static bool hs_warmup_done = false;

static void updateHalfspaceConstraints(void) {
  // Disable all half-space constraints by default
  for (int k = 0; k < NHORIZON; ++k) {
    data.en_hs[k] = 0;
  }
  
#if HALFSPACE_CSTR_MODE >= 1
  // Warm-up period: don't activate constraints for first 2 seconds
  if (!hs_warmup_done) {
    if (hs_start_time == 0) {
      hs_start_time = usecTimestamp();
    }
    if (usecTimestamp() - hs_start_time < 2000000) {  // 2 seconds
      stgs.en_cstr_states = 0;
      return;
    }
    hs_warmup_done = true;
    DEBUG_PRINT("HS warmup done, enabling constraints\n");
  }
  
  Eigen::Vector3f obs_center(obs_cx, obs_cy, obs_cz);
  bool any_active = false;
  // For each horizon step, check if ref is near obstacle
  for (int k = obs_horizon_start; k < NHORIZON; ++k) {
    Eigen::Vector3f ref_pos(Xref[k](0), Xref[k](1), Xref[k](2));
    Eigen::Vector3f diff = ref_pos - obs_center;
    float dist_to_obs = diff.norm();
    
    if (dist_to_obs < obs_activation_dist) {
      Eigen::Vector3f n;
      
      if (dist_to_obs > 1e-4f) {
        n = diff / dist_to_obs;
      } else {
        n = Eigen::Vector3f(0.0f, -1.0f, 0.0f);  // Default escape direction
      }
      
#if HALFSPACE_CSTR_MODE == 2
      // ADMM half-space projection (unstable, not recommended)
      data.a_hs[k] = -n;
      data.b_hs[k] = -n.dot(obs_center) - obs_r;
      data.en_hs[k] = 1;
      any_active = true;
#elif HALFSPACE_CSTR_MODE == 3
      // Soft reference push: if ref is inside obstacle+margin, push it out
      float penetration = (obs_r + 0.05f) - dist_to_obs;  // How far inside the safety zone
      if (penetration > 0) {
        // Push the reference outward by the penetration amount (capped)
        float push = (penetration < 0.02f) ? penetration : 0.02f;  // Max 2cm per step
        Xref[k](0) += n(0) * push;
        Xref[k](1) += n(1) * push;
        Xref[k](2) += n(2) * push;
        any_active = true;
      }
#endif
    }
  }
  
#if HALFSPACE_CSTR_MODE == 2
  // Only enable ADMM state constraints for mode 2
  stgs.en_cstr_states = any_active ? 1 : 0;
#else
  // Modes 0, 1, 3: no ADMM state constraints
  stgs.en_cstr_states = 0;
#endif
  
  // Logging
  if (hs_log_counter % 25 == 0) {
    Eigen::Vector3f pos(x0(0), x0(1), x0(2));
    float dist = (pos - obs_center).norm();
    // Log attitude from MPC solution (Rodrigues params: roll/pitch/yaw)
    float roll = Xhrz[1](3);   // phi_x
    float pitch = Xhrz[1](4); // phi_y
    DEBUG_PRINT("HS: act=%d d=%.2f r=%.2f p=%.2f\n", 
                any_active ? 1 : 0, (double)dist, (double)roll, (double)pitch);
  }
  hs_log_counter++;
#endif
}

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/

  // Precompute/Cache
  // #include "params_500hz.h"
  #include "params_100hz.h"

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
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
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);
  
  // Initialize state constraint data
  for (int i = 0; i < NSTATES; ++i) {
    lcx(i) = -1e6f;
    ucx(i) = 1e6f;
  }
  for (int k = 0; k < NHORIZON; ++k) {
    YX[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    // Initialize half-space constraints to disabled
    data.a_hs[k].setZero();
    data.b_hs[k] = 1e6f;
    data.en_hs[k] = 0;
  }
  tiny_SetStateBound(&work, &Acx, &lcx, &ucx);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;  // Will be enabled dynamically when near obstacle
  stgs.max_iter = 3;        // a bit more for constraints
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
  en_traj = true;  // Enable circle trajectory tracking
  step = 0;  
  traj_iter = 0;
  
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
    // Get command reference
    updateHorizonReference(setpoint);

    // Half-space obstacle avoidance constraints
    updateHalfspaceConstraints();

    /* MPC solve */
    // Solve optimization problem using ADMM
    tiny_UpdateLinearCost(&work);
    tiny_SolveAdmm(&work);
 
    // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
    // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
    // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
    // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
    // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
    result =  info.status_val * info.iter;
    
    // Position logging disabled
    // static uint32_t pos_log_counter = 0;
    // if (pos_log_counter % 50 == 0) {
    //   DEBUG_PRINT("POS: x=%.2f y=%.2f z=%.2f cstr=%d\n", 
    //               (double)x0(0), (double)x0(1), (double)x0(2), obs_constraint_active);
    // }
    // pos_log_counter++;
  }

  if (RATE_DO_EXECUTE(LQR_RATE, tick)) {
    // Reference from MPC
    Ulqr = -(Kinf) * (x0 - Xhrz[1]) + ZU_new[0];
    
    /* Output control */
    if (setpoint->mode.z == modeDisable) {
      control->normalizedForces[0] = 0.0f;
      control->normalizedForces[1] = 0.0f;
      control->normalizedForces[2] = 0.0f;
      control->normalizedForces[3] = 0.0f;
    } else {
      control->normalizedForces[0] = Ulqr(0) + u_hover[0];  // PWM 0..1
      control->normalizedForces[1] = Ulqr(1) + u_hover[1];
      control->normalizedForces[2] = Ulqr(2) + u_hover[2];
      control->normalizedForces[3] = Ulqr(3) + u_hover[3];
    } 
    control->controlMode = controlModePWM;
  }
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

#ifdef __cplusplus
}
#endif
