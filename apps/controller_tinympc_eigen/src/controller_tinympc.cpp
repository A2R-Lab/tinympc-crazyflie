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
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "stabilizer_types.h"  // For controlModePWM

#include "cpp_compat.h"   // needed to compile Cpp to C

// #include "tinympc/tinympc.h"

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

// Macro variables, model dimensions in tinympc/types.h
#define DT 0.002f       // dt
#define NHORIZON 25   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSTATES 12
#define NINPUTS 4
#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

#define T_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

using MatrixNf = Matrix<float, NSTATES, NSTATES>;
using MatrixNMf = Matrix<float, NSTATES, NINPUTS>;
using MatrixMNf = Matrix<float, NINPUTS, NSTATES>;
using MatrixMf = Matrix<float, NINPUTS, NINPUTS>;
using VectorNf = Matrix<float, NSTATES, 1>;
using VectorMf = Matrix<float, NINPUTS, 1>;

/* Include trajectory to track */
#include "traj_fig8_12.h"
//#include "traj_circle_500hz.h"
// #include "traj_perching.h"

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

static VectorMf Ulqr;

static VectorNf x0;
static VectorNf xg;
static VectorNf x1;
static VectorMf u0;

static float x0_flat[NSTATES];
static float x1_flat[NSTATES];
static float u0_flat[NINPUTS];
static float Xref_flat[NHORIZON * NSTATES];
static float Uref_flat[(NHORIZON - 1) * NINPUTS];

// Helper variables
static uint64_t startTimestamp;
// static bool isInit = false;  // fix for tracking problem - UNUSED, commented out
// static uint32_t mpcTime = 0;  // UNUSED (was for logging), commented out
static float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};  // cf1
// static float u_hover[4] = {0.7467, 0.667f, 0.78, 0.7f};  // cf2 not correct
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

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

  for (int i = 0; i < NSTATES; ++i) {
    x0_flat[i] = x0(i);
  }
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref_flat[i * NSTATES + j] = X_ref_data[traj_idx][j];
        }
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref_flat[i * NINPUTS + j] = U_ref_data[traj_idx][j];
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
    for (int i = 0; i < NHORIZON; ++i) {
      for (int j = 0; j < NSTATES; ++j) {
        Xref_flat[i * NSTATES + j] = xg(j);
      }
      if (i < NHORIZON - 1) {
        for (int j = 0; j < NINPUTS; ++j) {
          Uref_flat[i * NINPUTS + j] = 0.0f;
        }
      }
    }
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref_flat[2]));

  // stop trajectory executation
  if (en_traj) {
    if (traj_iter >= user_traj_iter) en_traj = false;

    if (traj_idx >= traj_length - 1 - NHORIZON + 1) { 
      // complete one trajectory
      step = 0; 
      traj_iter += 1;
    } 
    else step += 1;
  }
}

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/

  // Precompute/Cache
  // #include "params_500hz.h"
  #include "params_100hz.h"

  // End of Precompute/Cache

  tinympc_cf_init_defaults();
  tinympc_cf_set_settings(2, 0);
  memset(Xref_flat, 0, sizeof(Xref_flat));
  memset(Uref_flat, 0, sizeof(Uref_flat));
  memset(x0_flat, 0, sizeof(x0_flat));
  memset(x1_flat, 0, sizeof(x1_flat));
  memset(u0_flat, 0, sizeof(u0_flat));
  x1.setZero();
  u0.setZero();

  Klqr << 
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* End of MPC initialization */  
  en_traj = true;
  step = 0;  
  traj_iter = 0;
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

    /* MPC solve */
    tinympc_cf_set_x0(x0_flat, NSTATES);
    tinympc_cf_set_reference(Xref_flat, NSTATES, NHORIZON, Uref_flat, NINPUTS);
    result = (int8_t)tinympc_cf_solve();
    tinympc_cf_get_x_pred_1(x1_flat, NSTATES);
    tinympc_cf_get_u0(u0_flat, NINPUTS, TinympcControlOutputKind::ZNEW);

    for (int i = 0; i < NSTATES; ++i) {
      x1(i) = x1_flat[i];
    }
    for (int j = 0; j < NINPUTS; ++j) {
      u0(j) = u0_flat[j];
    }
 
    // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
    // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
    // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
    // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
    // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
    // DEBUG_PRINT("%.2f, %.2f, %.2f, %.2f \n", (double)(Xref_flat[5]), (double)(u0(2)), (double)(u0(3)), (double)(u0(0)));
  }

  if (RATE_DO_EXECUTE(LQR_RATE, tick)) {
    // Reference from MPC
    Ulqr = -(Kinf) * (x0 - x1) + u0;
    
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
    control->controlMode = control_mode_t::controlModeForce;
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
