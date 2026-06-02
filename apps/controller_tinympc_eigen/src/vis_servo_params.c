/**
 * vis_servo_params.c - PARAM/LOG plumbing for the TinyMPC vision-servo mode.
 *
 * The control logic lives in controller_tinympc.cpp; this file owns the tunable
 * storage and exposes it via the visMpc PARAM/LOG groups. It's a separate C TU
 * because the PARAM_ADD/LOG_ADD string-literal macros don't compile cleanly in
 * the C++ controller translation unit (see the commented-out groups there).
 *
 * Operational model (props OFF for all bench checks):
 *   1. Flash. visMpc.enable defaults to 0 -> controller behaves exactly as
 *      before (flies its precomputed trajectory; camera is telemetry only).
 *   2. On the bench, set visMpc.enable = 1 and watch the visMpc/aideck logs:
 *      confirm visMpc.fresh tracks the orange target, and that dyaw/fwd move in
 *      the right direction (target right -> dyaw negative; target far -> fwd +).
 *   3. Only then fly. WARNING: with enable=1 the MPC drives PWM straight from
 *      the vision goal, including climbing to targetZ on arm. Set visMpc.targetZ
 *      sensibly and test cautiously (tether / low gains first).
 */

#include "param.h"
#include "log.h"
#include <stdint.h>

uint8_t  visEnable     = 0;        // 0 = MPC trajectory/setpoint; 1 = visual servo
float    visTargetZ    = 0.5f;     // m, altitude to hold while servoing
uint16_t visTargetArea = 1500;     // px, desired orange blob area (standoff)
float    visKpYaw      = 0.0015f;  // rad of yaw-goal offset per pixel of x error
float    visKpFwd      = 0.00015f; // m of forward-goal offset per pixel of area error
float    visFwdMax     = 0.25f;    // m, clamp on forward goal offset
float    visYawMax     = 0.20f;    // rad, clamp on yaw goal offset (~11 deg)
uint16_t visTimeoutMs  = 300;      // detection considered stale after this

// Telemetry owned by controller_tinympc.cpp.
extern float   vis_log_fwd;
extern float   vis_log_dyaw;
extern uint8_t vis_log_fresh;

PARAM_GROUP_START(visMpc)
PARAM_ADD(PARAM_UINT8,  enable,     &visEnable)
PARAM_ADD(PARAM_FLOAT,  targetZ,    &visTargetZ)
PARAM_ADD(PARAM_UINT16, targetArea, &visTargetArea)
PARAM_ADD(PARAM_FLOAT,  kpYaw,      &visKpYaw)
PARAM_ADD(PARAM_FLOAT,  kpFwd,      &visKpFwd)
PARAM_ADD(PARAM_FLOAT,  fwdMax,     &visFwdMax)
PARAM_ADD(PARAM_FLOAT,  yawMax,     &visYawMax)
PARAM_ADD(PARAM_UINT16, timeoutMs,  &visTimeoutMs)
PARAM_GROUP_STOP(visMpc)

LOG_GROUP_START(visMpc)
LOG_ADD(LOG_FLOAT, fwd,   &vis_log_fwd)    // forward goal offset this tick (m)
LOG_ADD(LOG_FLOAT, dyaw,  &vis_log_dyaw)   // yaw goal offset this tick (rad)
LOG_ADD(LOG_UINT8, fresh, &vis_log_fresh)  // 1 = fresh target drove the goal
LOG_GROUP_STOP(visMpc)
