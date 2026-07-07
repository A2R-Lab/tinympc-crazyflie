/*
 * gate_vis_params.c
 * PARAM/LOG plumbing for the gate-vision MPC. Kept in a C translation unit
 * because the LOG/PARAM macros do not compile cleanly in the C++ controller TU
 * (string-literal warnings-as-errors). Storage lives in controller_tinympc.cpp
 * (gateEnable, gate_target_speed, gate_engaged, gate_source) and gate_pnp.c
 * (the g_gate_* tunables/outputs); this file only exposes them.
 */
#include "param.h"
#include "log.h"
#include "gate_pnp.h"

/* Defined in controller_tinympc.cpp (inside its extern "C" block -> C linkage). */
extern uint8_t gateEnable;
extern float   gate_target_speed;
extern uint8_t gate_engaged;
extern uint8_t gate_source;
extern float   gate_tc_speed;
extern float   gate_tc_distance;
extern float   gate_tc_center_tol;
extern float   gate_tc_min_margin;
extern float   gate_tc_max_age_s;
extern float   ctrlGainScale;
extern float   ctrlGnColl;
extern float   ctrlGnAtt;
extern uint8_t ctrlCollPrio;
extern float   ctrlUHover;
extern float   ctrlGateLookahead;
extern uint8_t fuseEnable;
extern uint8_t fuseInject;
extern float   fuseStd;
extern float   fuseMaxInnov;
extern float   fuseGateWX;
extern float   fuseGateWY;
extern float   fuseGateWZ;
extern float   fuseGateBX;
extern float   fuseGateBY;
extern float   fuseGateBZ;
extern float   fuseYawWin;
extern float   fuseYawRateMax;
extern uint8_t g_fuse_gate;
extern float   g_fuse_dx;
extern float   g_fuse_dy;
extern float   g_fuse_dz;
extern uint32_t g_fuse_n;
extern uint8_t  circuitEnable;
extern float    frameYaw;
extern uint32_t circ_idx;
extern uint8_t  circStride;
extern uint8_t  circMan;
extern float    circYawMan;
extern uint8_t  actualYawFrame;
extern float    circDmax;
extern float    yawKi;
extern float    yawImax;
extern float    g_yaw_i;
extern float    g_chart_yaw;
extern uint8_t  g_motor_sat;
extern float    g_torque_z;
extern float    g_gyro_z;
extern float    g_thrust;
extern float    g_fmax;
extern float    g_fmin;
extern float    g_f0, g_f1, g_f2, g_f3;
extern uint8_t  yawTest;
extern float    yawTestTau;

/**
 * Gate-vision MPC tunables. enable=0 by default (the controller falls back to the
 * commander/trajectory path). Intrinsics default to the AI-deck calibration and
 * gate size to the sim default — verify both against the physical setup.
 */
PARAM_GROUP_START(visMpc)
PARAM_ADD(PARAM_UINT8,  enable, &gateEnable)
PARAM_ADD(PARAM_FLOAT,  speed,  &gate_target_speed)
PARAM_ADD(PARAM_FLOAT,  fx,     &g_gate_fx)
PARAM_ADD(PARAM_FLOAT,  fy,     &g_gate_fy)
PARAM_ADD(PARAM_FLOAT,  cx,     &g_gate_cx)
PARAM_ADD(PARAM_FLOAT,  cy,     &g_gate_cy)
PARAM_ADD(PARAM_FLOAT,  imgW,   &g_gate_img_w)
PARAM_ADD(PARAM_FLOAT,  imgH,   &g_gate_img_h)
PARAM_ADD(PARAM_FLOAT,  crnH,   &g_gate_corner_h)
PARAM_ADD(PARAM_FLOAT,  gateW,  &g_gate_width_m)
PARAM_ADD(PARAM_FLOAT,  gateH,  &g_gate_height_m)
PARAM_ADD(PARAM_FLOAT,  mntFwd, &g_gate_mount_fwd_m)
PARAM_ADD(PARAM_FLOAT,  mntUp,  &g_gate_mount_up_m)
PARAM_ADD(PARAM_FLOAT,  mntPit, &g_gate_mount_pitch_rad)
PARAM_ADD(PARAM_FLOAT,  rMin,   &g_gate_min_range_m)
PARAM_ADD(PARAM_FLOAT,  rMax,   &g_gate_max_range_m)
PARAM_ADD(PARAM_UINT32, maxAge, &g_gate_max_age_ms)
PARAM_ADD(PARAM_FLOAT,  tcSpd,  &gate_tc_speed)
PARAM_ADD(PARAM_FLOAT,  tcDist, &gate_tc_distance)
PARAM_ADD(PARAM_FLOAT,  tcTol,  &gate_tc_center_tol)
PARAM_ADD(PARAM_FLOAT,  tcMrg,  &gate_tc_min_margin)
PARAM_ADD(PARAM_FLOAT,  tcAge,  &gate_tc_max_age_s)
PARAM_ADD(PARAM_FLOAT,  gnScale, &ctrlGainScale)
PARAM_ADD(PARAM_FLOAT,  gnColl,  &ctrlGnColl)
PARAM_ADD(PARAM_FLOAT,  gnAtt,   &ctrlGnAtt)
PARAM_ADD(PARAM_UINT8,  collPr,  &ctrlCollPrio)
PARAM_ADD(PARAM_FLOAT,  uHov,    &ctrlUHover)
PARAM_ADD(PARAM_FLOAT,  lookah,  &ctrlGateLookahead)
PARAM_ADD(PARAM_UINT8,  fuse,    &fuseEnable)
PARAM_ADD(PARAM_UINT8,  fuseInj, &fuseInject)
PARAM_ADD(PARAM_FLOAT,  fuseStd, &fuseStd)
PARAM_ADD(PARAM_FLOAT,  fuseMax, &fuseMaxInnov)
PARAM_ADD(PARAM_FLOAT,  gWX,     &fuseGateWX)
PARAM_ADD(PARAM_FLOAT,  gWY,     &fuseGateWY)
PARAM_ADD(PARAM_FLOAT,  gWZ,     &fuseGateWZ)
PARAM_ADD(PARAM_FLOAT,  gBX,     &fuseGateBX)
PARAM_ADD(PARAM_FLOAT,  gBY,     &fuseGateBY)
PARAM_ADD(PARAM_FLOAT,  gBZ,     &fuseGateBZ)
PARAM_ADD(PARAM_FLOAT,  yawWin,  &fuseYawWin)
PARAM_ADD(PARAM_FLOAT,  yawRtMx, &fuseYawRateMax)
PARAM_ADD(PARAM_UINT8,  circuit, &circuitEnable)
PARAM_ADD(PARAM_UINT8,  circStr, &circStride)
PARAM_ADD(PARAM_UINT8,  circMan, &circMan)
PARAM_ADD(PARAM_FLOAT,  circYaw, &circYawMan)
PARAM_ADD(PARAM_UINT8,  yawFrame, &actualYawFrame)
PARAM_ADD(PARAM_FLOAT,  circDmax, &circDmax)
PARAM_ADD(PARAM_UINT8,  yawTest, &yawTest)
PARAM_ADD(PARAM_FLOAT,  yawTau,  &yawTestTau)
PARAM_ADD(PARAM_FLOAT,  yawKi,   &yawKi)
PARAM_ADD(PARAM_FLOAT,  yawImax, &yawImax)
PARAM_GROUP_STOP(visMpc)

/**
 * Gate-vision MPC status: engaged flag, control source, projection validity, and
 * the recovered world-frame gate center + range.
 */
LOG_GROUP_START(gateMpc)
LOG_ADD(LOG_UINT8, engaged, &gate_engaged)
LOG_ADD(LOG_UINT8, source,  &gate_source)
LOG_ADD(LOG_UINT8, valid,   &g_gate_valid)
LOG_ADD(LOG_FLOAT, gx,      &g_gate_center_x)
LOG_ADD(LOG_FLOAT, gy,      &g_gate_center_y)
LOG_ADD(LOG_FLOAT, gz,      &g_gate_center_z)
LOG_ADD(LOG_FLOAT, range,   &g_gate_range_m)
/* Debug: projection bail diagnostics. reason: 0=ok 1=stale 2=degenerate
 * 3=too_far 4=too_close 5=null. dbgW/dbgH px (post un-squish), dbgR m, age ms. */
LOG_ADD(LOG_UINT8,  reason, &g_gate_dbg_reason)
LOG_ADD(LOG_FLOAT,  dbgW,   &g_gate_dbg_width)
LOG_ADD(LOG_FLOAT,  dbgH,   &g_gate_dbg_height)
LOG_ADD(LOG_FLOAT,  dbgR,   &g_gate_dbg_range)
LOG_ADD(LOG_UINT32, age,    &g_gate_dbg_age_ms)
/* Vision fusion: live drift = surveyed gate - computed gate (the EKF correction),
 * logged even when not injecting; fn = number of fixes injected. */
LOG_ADD(LOG_FLOAT,  fdx,    &g_fuse_dx)
LOG_ADD(LOG_FLOAT,  fdy,    &g_fuse_dy)
LOG_ADD(LOG_FLOAT,  fdz,    &g_fuse_dz)
LOG_ADD(LOG_UINT32, fn,     &g_fuse_n)
LOG_ADD(LOG_UINT8,  fGate,  &g_fuse_gate)
/* Chart circuit: current chart heading + lap progress index. */
LOG_ADD(LOG_FLOAT,  frmYaw, &frameYaw)
LOG_ADD(LOG_UINT32, circIdx,&circ_idx)
/* Yaw integrator: chart_yaw error [rad] and the make-up torque [N*m] it is applying. */
LOG_ADD(LOG_FLOAT,  chYaw,  &g_chart_yaw)
LOG_ADD(LOG_FLOAT,  yawI,   &g_yaw_i)
LOG_ADD(LOG_UINT8,  satM,   &g_motor_sat)
/* Cause/effect: commanded yaw torque (the control signal) vs measured body yaw rate. */
LOG_ADD(LOG_FLOAT,  tauZ,   &g_torque_z)
LOG_ADD(LOG_FLOAT,  gyroZ,  &g_gyro_z)
/* Thrust headroom: is a sink command-saturated (control) or voltage-limited (power)? */
LOG_ADD(LOG_FLOAT,  thrust, &g_thrust)
LOG_ADD(LOG_FLOAT,  fmax,   &g_fmax)
LOG_ADD(LOG_FLOAT,  fmin,   &g_fmin)
/* Per-motor forces [N]: one hot + others low = allocation; all four hot = thrust wall. */
LOG_ADD(LOG_FLOAT,  f0,     &g_f0)
LOG_ADD(LOG_FLOAT,  f1,     &g_f1)
LOG_ADD(LOG_FLOAT,  f2,     &g_f2)
LOG_ADD(LOG_FLOAT,  f3,     &g_f3)
LOG_GROUP_STOP(gateMpc)
