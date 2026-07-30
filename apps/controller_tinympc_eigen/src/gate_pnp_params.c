/*
 * gate_pnp_params.c
 * PARAM/LOG plumbing for the gate-vision projection (Stage 1: perception only).
 * (PARAM/LOG macros don't compile in the C++ controller TU.) The g_gate_*
 * storage is DEFINED in gate_pnp.c; this file only exposes it.
 *
 * visGate PARAM group -- camera/gate/mount tunables. Defaults match the AI-deck
 * calibration and the training gate size, but the mount extrinsics and gate size
 * MUST be verified against the physical setup.
 * visGate LOG group -- the recovered world-frame gate center + validity + the
 * projection debug (why it bailed). Watch gx/gy/gz while holding a gate in view.
 */
#include "param.h"
#include "log.h"
#include <stdint.h>

/* Tunables (defined in gate_pnp.c). */
extern float g_gate_fx;
extern float g_gate_fy;
extern float g_gate_cx;
extern float g_gate_cy;
extern float g_gate_img_w;
extern float g_gate_img_h;
extern float g_gate_corner_h;
extern float g_gate_width_m;
extern float g_gate_height_m;
extern float g_gate_mount_fwd_m;
extern float g_gate_mount_up_m;
extern float g_gate_mount_pitch_rad;
extern float g_gate_mount_yaw_rad;
extern float g_gate_min_range_m;
extern float g_gate_max_range_m;
extern uint32_t g_gate_max_age_ms;

/* Outputs (defined in gate_pnp.c). */
extern float g_gate_center_x;
extern float g_gate_center_y;
extern float g_gate_center_z;
extern float g_gate_range_m;
extern uint8_t g_gate_valid;
extern float g_gate_dbg_width;
extern float g_gate_dbg_height;
extern float g_gate_dbg_range;
extern uint32_t g_gate_dbg_age_ms;
extern uint8_t g_gate_dbg_reason;
extern float g_gate_rel_x;
extern float g_gate_rel_y;
extern float g_gate_rel_z;

/* Stage 2 gate navigation (defined in controller_tinympc.cpp). */
extern uint8_t gateNavEn;
extern float   gateThrough;
extern float   gateSpeed;
extern uint8_t g_gate_latched;
extern float   g_gate_tx;
extern float   g_gate_ty;
extern float   g_gate_tz;

/* Stage 3 two-gate circuit (defined in controller_tinympc.cpp). */
extern uint8_t circEn;
extern float   gAx, gAy, gAz;
extern float   gBx, gBy, gBz;
extern float   loopWidth;
extern float   circSpeed;
extern int8_t  circDir;
extern float   g_circ_phase;


PARAM_GROUP_START(visGate)
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
PARAM_ADD(PARAM_FLOAT,  mntPit, &g_gate_mount_pitch_rad)   /* + = camera tilted DOWN [rad] */
PARAM_ADD(PARAM_FLOAT,  mntYaw, &g_gate_mount_yaw_rad)     /* + = camera tilted LEFT [rad] */
PARAM_ADD(PARAM_FLOAT,  rMin,   &g_gate_min_range_m)
PARAM_ADD(PARAM_FLOAT,  rMax,   &g_gate_max_range_m)
PARAM_ADD(PARAM_UINT32, maxAge, &g_gate_max_age_ms)
PARAM_GROUP_STOP(visGate)

/* Stage 2 gate navigation tunables. navEn=0 by default -> commander/hover unchanged. */
PARAM_GROUP_START(gateNav)
PARAM_ADD(PARAM_UINT8, navEn,   &gateNavEn)
PARAM_ADD(PARAM_FLOAT, through, &gateThrough)
PARAM_ADD(PARAM_FLOAT, speed,   &gateSpeed)
PARAM_GROUP_STOP(gateNav)

/* Stage 3 two-gate racetrack circuit. en=0 by default -> commander/hover unchanged.
 * gAx..gBz are the two SURVEYED gate world positions (relative to takeoff origin), and
 * they double as the landmarks the Stage 4 fusion below corrects against -- so a survey
 * error shows up as a constant position bias, not just a wonky path. Defaults: gate A 1 m
 * ahead / 0.5 m up, gate B 1.5 m to its right, same plane; loopW=0.75 (= half the gate
 * spacing) makes the loop a true circle through both. */
PARAM_GROUP_START(circuit)
PARAM_ADD(PARAM_UINT8, en,    &circEn)
PARAM_ADD(PARAM_FLOAT, gAx,   &gAx)
PARAM_ADD(PARAM_FLOAT, gAy,   &gAy)
PARAM_ADD(PARAM_FLOAT, gAz,   &gAz)
PARAM_ADD(PARAM_FLOAT, gBx,   &gBx)
PARAM_ADD(PARAM_FLOAT, gBy,   &gBy)
PARAM_ADD(PARAM_FLOAT, gBz,   &gBz)
PARAM_ADD(PARAM_FLOAT, loopW, &loopWidth)
PARAM_ADD(PARAM_FLOAT, speed, &circSpeed)
/* Which way round the loop: +1 joins heading FORWARD and takes the near gate (A) first;
 * -1 is the mirror image, which from the takeoff origin sends the drone backwards away
 * from the gates and around the back arc first. Live-flippable -- no reflash. */
PARAM_ADD(PARAM_INT8,  dir,   &circDir)
PARAM_GROUP_STOP(circuit)


LOG_GROUP_START(visGate)
LOG_ADD(LOG_FLOAT,  gx,     &g_gate_center_x)   /* world-frame gate center */
LOG_ADD(LOG_FLOAT,  gy,     &g_gate_center_y)
LOG_ADD(LOG_FLOAT,  gz,     &g_gate_center_z)
LOG_ADD(LOG_FLOAT,  range,  &g_gate_range_m)
LOG_ADD(LOG_UINT8,  valid,  &g_gate_valid)
LOG_ADD(LOG_FLOAT,  dbgW,   &g_gate_dbg_width)
LOG_ADD(LOG_FLOAT,  dbgH,   &g_gate_dbg_height)
LOG_ADD(LOG_FLOAT,  dbgR,   &g_gate_dbg_range)
LOG_ADD(LOG_UINT32, dbgAge, &g_gate_dbg_age_ms)
LOG_ADD(LOG_UINT8,  reason, &g_gate_dbg_reason)  /* 0=ok 1=stale 2=degenerate 3=far 4=near 5=null */
/* Stage 2 gate navigation: latched flag + world through-point target. */
LOG_ADD(LOG_UINT8,  ltch,   &g_gate_latched)
LOG_ADD(LOG_FLOAT,  tx,     &g_gate_tx)
LOG_ADD(LOG_FLOAT,  ty,     &g_gate_ty)
LOG_ADD(LOG_FLOAT,  tz,     &g_gate_tz)
LOG_ADD(LOG_FLOAT,  phase,  &g_circ_phase)   /* Stage 3 circuit loop phase [rad] */
LOG_GROUP_STOP(visGate)
