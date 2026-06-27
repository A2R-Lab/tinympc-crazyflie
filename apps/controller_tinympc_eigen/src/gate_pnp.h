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
 *   - corner coords: normalized [0,1], order TLx,TLy,TRx,TRy,BRx,BRy,BLx,BLy.
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
/* Reference image dimensions that the normalized [0,1] corners map onto.
 * Default 160x160: corners normalize per-axis over the 160-wide crop and the
 * vertical resize cancels, so u=x*160, v=y*160. Adjust if the CNN normalization
 * convention differs (verify on hardware). */
extern float g_gate_img_w;
extern float g_gate_img_h;
/* Physical gate aperture size, meters (outer corner-to-corner span). */
extern float g_gate_width_m;
extern float g_gate_height_m;
/* Camera mount relative to body: forward/up offset (m) and downward pitch (rad). */
extern float g_gate_mount_fwd_m;
extern float g_gate_mount_up_m;
extern float g_gate_mount_pitch_rad;
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
