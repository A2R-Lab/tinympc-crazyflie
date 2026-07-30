/*
 * gate_pnp.h
 * Simplified 2-DOF gate projection for the Crazyflie STM32.
 *
 * Lifts the 8 normalized gate-corner detections produced by the GAP8 frontnet
 * CNN (see gate8_link.h) into a metric GateVisionPacket expressed in the
 * estimator world frame, using a bearing + apparent-size pinhole estimate with
 * the AI-deck camera calibration. This is the inverse of the sim's forward
 * projection; it is NOT a full 6-DOF PnP (no gate-plane orientation).
 *
 * Frames:
 *   - corner coords: native 160x160 pixels, order
 *     TLx,TLy,TRx,TRy,BRx,BRy,BLx,BLy.
 *   - camera: x right, y down, z forward (optical axis).
 *   - body: x forward, y left, z up (Crazyflie).
 *   - world: estimator global frame (matches state->x/y/z).
 *
 * Tunables (g_gate_* below) default to the real calibration / sim gate geometry
 * but MUST be verified against the physical gate and camera mount on hardware.
 */
#ifndef GATE_PNP_H
#define GATE_PNP_H

#include <stdint.h>
#include <stdbool.h>
#include "gate_tinympc_core.h"   /* GateVisionPacket, DroneState */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Tunable parameters (exposed as the visMpc PARAM group, see gate_vis_params.c) --- */
/* Camera intrinsics, in pixels, in the reference image frame (img_w x img_h). */
extern float g_gate_fx;
extern float g_gate_fy;
extern float g_gate_cx;
extern float g_gate_cy;
/* Reference image dimensions for the pixel-valued corners. */
extern float g_gate_img_w;
extern float g_gate_img_h;
extern float g_gate_corner_h;   /* net-input frame height (native HM01B0: 160) */

/* Debug: last-projection diagnostics (see gate_pnp.c for reason codes). */
extern float    g_gate_dbg_width;
extern float    g_gate_dbg_height;
extern float    g_gate_dbg_range;
extern uint32_t g_gate_dbg_age_ms;
extern uint8_t  g_gate_dbg_reason;
/* Physical gate aperture size, meters (outer corner-to-corner span). */
extern float g_gate_width_m;
extern float g_gate_height_m;
/* Camera mount relative to body: forward/up offset (m), downward pitch and left yaw (rad).
 * pitch/yaw are the boresight trims: they absorb the camera not looking exactly down the
 * body +x axis (physical mount slop, or a principal point that isn't where the intrinsics
 * claim). Both are pure bearing corrections -- they rotate the measured drone->gate vector
 * and do NOT touch the range. Calibrate them by parking the drone a measured distance
 * square-on to a gate and trimming until its recovered center is aligned with the
 * surveyed gate center. */
extern float g_gate_mount_fwd_m;
extern float g_gate_mount_up_m;
extern float g_gate_mount_pitch_rad;   /* + = camera tilted DOWN from body +x */
extern float g_gate_mount_yaw_rad;     /* + = camera tilted LEFT of body +x  */
/* Validity gating. */
extern float g_gate_min_range_m;
extern float g_gate_max_range_m;
extern uint32_t g_gate_max_age_ms;

/* Latest projection outputs, for logging (gateMpc LOG group). */
extern float g_gate_center_x;   /* world-frame gate center */
extern float g_gate_center_y;
extern float g_gate_center_z;
extern float g_gate_range_m;
extern uint8_t g_gate_valid;

/*
 * Project the corners into `out` (world frame). Returns true and sets out->valid
 * when a usable gate is reconstructed; returns false (out->valid=false) when the
 * data is stale, the corners are degenerate, or the range is out of bounds.
 */
bool gate_pnp_project(const float corners[8], uint32_t age_ms,
                      const DroneState* state, GateVisionPacket* out);

#ifdef __cplusplus
}
#endif

#endif /* GATE_PNP_H */
