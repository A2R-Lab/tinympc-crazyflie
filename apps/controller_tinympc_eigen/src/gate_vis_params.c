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
extern float   ctrlUHover;
extern float   ctrlGateLookahead;

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
PARAM_ADD(PARAM_FLOAT,  uHov,    &ctrlUHover)
PARAM_ADD(PARAM_FLOAT,  lookah,  &ctrlGateLookahead)
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
LOG_GROUP_STOP(gateMpc)
