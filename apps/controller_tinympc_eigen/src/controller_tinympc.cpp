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
 * controller_tinympc.cpp - TinyMPC-ADMM based controller for new crazyflie firmware.
 * Adapted from Ishaan's implementation for the latest firmware API.
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

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "eventtrigger.h"
#include "stabilizer_types.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"

// Rodriguez parameters conversion function (missing in new firmware)
static inline struct vec quat2rp(struct quat q) {
  struct vec v;
  v.x = q.x/q.w;
  v.y = q.y/q.w;
  v.z = q.z/q.w;
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
#define NHORIZON 15   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE 500  // control frequency

/* Include trajectory to track */
#include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"
// #include "traj_perching.h"

// Precomputed data and cache, in params_*.h
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1]; 

// static float Xhrz_log[NHORIZON*12]; // Currently unused - for future logging
// static float Uhrz_log[(NHORIZON-1)*4]; // Currently unused - for future logging
static uint32_t iter_log;
static float pri_resid_log;
static float dual_resid_log;
static float ref_x;
static float ref_y;
static float ref_z;

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
// static bool isInit = false;  // Currently unused - for future tracking
static uint32_t mpcTime = 0;
static float u_hover[4] = {0.6641f, 0.6246f, 0.7216f, 0.5756f};  // cf1
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 3;       // hold current trajectory for this no of steps
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
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < 3; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        for (int j = 6; j < 9; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        ref_x = X_ref_data[traj_idx][0];
        ref_y = X_ref_data[traj_idx][1];
        ref_z = X_ref_data[traj_idx][2];
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
  }

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
  #include "params_500hz.h"

  // End of Precompute/Cache
  traj_length = traj_length * traj_hold;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  
  /* Set up constraints */
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 6;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 2;
  stgs.tol_abs_dual = 1e-2;
  stgs.tol_abs_prim = 1e-2;

  /* End of MPC initialization */  
  en_traj = true;
  step = 0;  
  traj_iter = 0;
  
  DEBUG_PRINT("TinyMPC controller initialized successfully!\n");
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {

  /* Controller rate */
  if (!RATE_DO_EXECUTE(MPC_RATE, stabilizerStep)) {
    return;
  }
  
  // Get current time
  updateHorizonReference(setpoint);
  /* Get current state (initial state for MPC) */
  updateInitialState(sensors, state);

  /* MPC solve */
  tiny_UpdateLinearCost(&work);
  startTimestamp = usecTimestamp();
  tiny_SolveAdmm(&work);
  mpcTime = usecTimestamp() - startTimestamp;
 
  result = info.status_val * info.iter;
  
  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
  } else {
    // Convert TinyMPC motor forces to legacy control format
    // Get optimized motor forces (adding baseline hover forces)
    float f0 = ZU_new[0](0) + u_hover[0];  // Motor 0 force
    float f1 = ZU_new[0](1) + u_hover[1];  // Motor 1 force  
    float f2 = ZU_new[0](2) + u_hover[2];  // Motor 2 force
    float f3 = ZU_new[0](3) + u_hover[3];  // Motor 3 force
    
    // Invert the standard quadrotor mixing matrix:
    // f0 = thrust - roll - pitch - yaw
    // f1 = thrust - roll + pitch + yaw  
    // f2 = thrust + roll + pitch - yaw
    // f3 = thrust + roll - pitch + yaw
    
    // Inverse mixing to get thrust, roll, pitch, yaw components
    float thrust_force = 0.25f * (f0 + f1 + f2 + f3);
    float roll_force = 0.25f * (-f0 - f1 + f2 + f3);
    float pitch_force = 0.25f * (-f0 + f1 + f2 - f3);
    float yaw_force = 0.25f * (-f0 + f1 - f2 + f3);
    
    // Convert to legacy control format
    // Thrust in units expected by firmware (scale motor forces to 0-65535)
    control->thrust = (uint16_t)(thrust_force * 65535.0f / 15.0f);  // Assuming max thrust ~15N total
    
    // Roll/pitch in degrees (approximate conversion from force differences)
    const float arm = 0.707106781f * 0.046f;  // ARM_LENGTH = 46mm
    control->roll = roll_force * arm * 57.3f;   // Convert torque to angle (rad to deg)
    control->pitch = pitch_force * arm * 57.3f;
    control->yaw = yaw_force * 57.3f;           // Yaw rate in deg/s
  } 

  control->controlMode = controlModeLegacy;

  //Save results for logging
  iter_log = info.iter;
  pri_resid_log = info.pri_res;
  dual_resid_log = info.dua_res;
}

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

/*
#ifdef __cplusplus
extern "C" {
#endif

LOG_GROUP_START(ctrlMPC)

LOG_ADD(LOG_INT32, iters, &iter_log)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, primal_residual, &pri_resid_log)
LOG_ADD(LOG_FLOAT, dual_residual, &dual_resid_log)
LOG_ADD(LOG_FLOAT, ref_x, &ref_x)
LOG_ADD(LOG_FLOAT, ref_y, &ref_y)
LOG_ADD(LOG_FLOAT, ref_z, &ref_z)

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
*/

#ifdef __cplusplus
}
#endif
