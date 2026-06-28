/*
 * gate_pnp.c  -- see gate_pnp.h for the contract and frame conventions.
 *
 * Simplified 2-DOF gate estimate:
 *   range  <- focal * physical_gate_size / apparent_pixel_size
 *   bearing<- (centroid_px - principal_point) / focal
 * yields the gate center in the camera frame, which is rotated into body then
 * world frame. With only center+range we synthesize an upright, drone-facing set
 * of metric corners about the center (orientation is not recovered); the gate
 * reference only uses the center, so this is sufficient for the first increment.
 */
#include "gate_pnp.h"
#include <math.h>
#include <string.h>

/* --- Tunable defaults (verify on hardware). Calibration: esp_color_object/camera_calibration.yaml --- */
float g_gate_fx = 89.20f;
float g_gate_fy = 89.50f;
float g_gate_cx = 81.09f;
float g_gate_cy = 73.37f;
float g_gate_img_w = 160.0f;         /* camera-native frame width  (intrinsics frame) */
float g_gate_img_h = 160.0f;         /* camera-native frame height (intrinsics frame) */
/* The GAP8 gate8 detector emits PIXEL coords in its net-input frame: 160 wide x
 * 96 tall (the 160x160 camera crop after a vertical INTER_AREA resize 160->96).
 * Width is unchanged so x passes through; y is squished and must be scaled back
 * to the camera-native frame by img_h / corner_h before applying fy/cy. Set
 * corner_h = img_h (160) if the detector ever emits y in the camera-native frame. */
float g_gate_corner_h = 96.0f;       /* net-input frame height the corners live in */
float g_gate_width_m = 0.4826f;      /* gate inner size the model was trained on  */
float g_gate_height_m = 0.4826f;     /* (dataset metadata gate_inner_size_m). PARAM visMpc.gateW/H */
float g_gate_mount_fwd_m = 0.0f;
float g_gate_mount_up_m = 0.0f;
float g_gate_mount_pitch_rad = 0.0f; /* +down tilt of the camera about body-left axis */
float g_gate_min_range_m = 0.30f;
float g_gate_max_range_m = 8.00f;
uint32_t g_gate_max_age_ms = 200;

/* --- Latest outputs for logging --- */
float g_gate_center_x = 0.0f;
float g_gate_center_y = 0.0f;
float g_gate_center_z = 0.0f;
float g_gate_range_m = 0.0f;
uint8_t g_gate_valid = 0;

/* --- Debug: expose why the last projection bailed (logged for HW triage) ---
 * reason codes: 0=ok 1=stale(age) 2=degenerate(width/height<1) 3=too_far(>max)
 * 4=too_close(<min) 5=null-arg. width/height in px (post un-squish), range in m. */
float    g_gate_dbg_width  = 0.0f;
float    g_gate_dbg_height = 0.0f;
float    g_gate_dbg_range  = 0.0f;
uint32_t g_gate_dbg_age_ms = 0;
uint8_t  g_gate_dbg_reason = 0;

/* Rotate vector v (body frame) into world frame by quaternion q=(x,y,z,w). */
static void quat_rotate(float qx, float qy, float qz, float qw,
                        const float v[3], float out[3]) {
  /* out = v + 2*q_vec x (q_vec x v + w*v) */
  const float tx = 2.0f * (qy * v[2] - qz * v[1]);
  const float ty = 2.0f * (qz * v[0] - qx * v[2]);
  const float tz = 2.0f * (qx * v[1] - qy * v[0]);
  out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
  out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
  out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

bool gate_pnp_project(const float corners[8], uint32_t age_ms,
                      const DroneState* state, GateVisionPacket* out) {
  memset(out, 0, sizeof(*out));
  out->valid = false;
  out->raw_valid = false;
  out->source = GATE_SOURCE_CAMERA_INVALID;
  out->invalid_reason = GATE_INVALID_GEOMETRY_FAILED;
  g_gate_valid = 0;

  if (corners == 0 || state == 0) {
    g_gate_dbg_reason = 5;
    return false;
  }
  g_gate_dbg_age_ms = age_ms;
  if (age_ms > g_gate_max_age_ms) {
    out->invalid_reason = GATE_INVALID_NO_CONTOUR; /* stale: treat as no detection */
    g_gate_dbg_reason = 1;
    return false;
  }

  /* Corners are PIXEL coords from the GAP8 (gate8) net frame (160 x 96), NOT
   * normalized. x already matches the camera-native frame; un-squish y back to
   * it by img_h / corner_h so it lines up with fy/cy. (See header above.) */
  const float v_scale = (g_gate_corner_h > 1.0f) ? (g_gate_img_h / g_gate_corner_h) : 1.0f;
  float u[4], vpx[4];
  for (int i = 0; i < 4; ++i) {
    u[i]   = corners[2 * i + 0];
    vpx[i] = corners[2 * i + 1] * v_scale;
  }
  /* Corner order is IPPE_SQUARE (per the gate8 model README / training labels),
   * NOT raster: idx 0,1,2,3 = visual TL, BL, BR, TR -- traverse down the left
   * edge, across the bottom, up the right edge. So the TOP edge is idx0-idx3,
   * BOTTOM idx1-idx2, LEFT idx0-idx1, RIGHT idx3-idx2. (Validated against the
   * golden frame: the raster pairing gave width~3px -> range>cap -> range=0.) */
  const float uc = 0.25f * (u[0] + u[1] + u[2] + u[3]);
  const float vc = 0.25f * (vpx[0] + vpx[1] + vpx[2] + vpx[3]);
  /* apparent extents: width from top/bottom edges, height from left/right edges. */
  const float width_px  = 0.5f * (fabsf(u[3] - u[0]) + fabsf(u[2] - u[1]));
  const float height_px = 0.5f * (fabsf(vpx[1] - vpx[0]) + fabsf(vpx[2] - vpx[3]));
  g_gate_dbg_width = width_px;
  g_gate_dbg_height = height_px;
  if (width_px < 1.0f || height_px < 1.0f) {
    out->invalid_reason = GATE_INVALID_GEOMETRY_FAILED;
    g_gate_dbg_reason = 2;
    return false;
  }

  const float range_w = g_gate_fx * g_gate_width_m / width_px;
  const float range_h = g_gate_fy * g_gate_height_m / height_px;
  const float range = 0.5f * (range_w + range_h);
  g_gate_dbg_range = range;
  if (!isfinite(range) || range > g_gate_max_range_m) {
    out->invalid_reason = GATE_INVALID_NO_CONTOUR;  /* nothing usable */
    g_gate_dbg_reason = 3;
    return false;
  }
  if (range < g_gate_min_range_m) {
    /* Gate is too close to range reliably: it fills/overflows the frame. Signal
     * the core's terminal-commit path so it can fly through on the last good
     * estimate rather than dropping the detection cold. */
    out->invalid_reason = GATE_INVALID_GATE_CLIPPED_OUT_OF_FRAME;
    out->projection_gate_clipped = true;
    g_gate_dbg_reason = 4;
    return false;
  }

  /* gate center in camera frame (x right, y down, z forward). */
  const float xr = (uc - g_gate_cx) / g_gate_fx * range;
  const float yd = (vc - g_gate_cy) / g_gate_fy * range;
  const float zf = range;

  /* camera -> body (x fwd, y left, z up), then apply mount pitch about body-left. */
  float bx = zf;       /* forward  */
  float by = -xr;      /* left     */
  float bz = -yd;      /* up       */
  const float cp = cosf(g_gate_mount_pitch_rad);
  const float sp = sinf(g_gate_mount_pitch_rad);
  /* rotate (fwd,up) by +pitch (camera looks down): fwd' = c*fwd - s*up ... */
  const float bx_t =  cp * bx + sp * bz;
  const float bz_t = -sp * bx + cp * bz;
  bx = bx_t + g_gate_mount_fwd_m;
  bz = bz_t + g_gate_mount_up_m;

  const float g_body[3] = { bx, by, bz };
  float g_rot[3];
  quat_rotate(state->qx, state->qy, state->qz, state->qw, g_body, g_rot);
  const float gx = state->x + g_rot[0];
  const float gy = state->y + g_rot[1];
  const float gz = state->z + g_rot[2];

  /* Fill the packet (world frame). Orientation is unknown, so synthesize an
   * upright set of corners about the center using the physical half extents;
   * the gate reference uses only the averaged center. Layout matches the sim:
   * TL,TR,BR,BL with +y left, +z up. */
  const float hw = 0.5f * g_gate_width_m;
  const float hh = 0.5f * g_gate_height_m;
  out->gate_corners_m[0] = gx; out->gate_corners_m[1]  = gy + hw; out->gate_corners_m[2]  = gz + hh; /* TL */
  out->gate_corners_m[3] = gx; out->gate_corners_m[4]  = gy - hw; out->gate_corners_m[5]  = gz + hh; /* TR */
  out->gate_corners_m[6] = gx; out->gate_corners_m[7]  = gy - hw; out->gate_corners_m[8]  = gz - hh; /* BR */
  out->gate_corners_m[9] = gx; out->gate_corners_m[10] = gy + hw; out->gate_corners_m[11] = gz - hh; /* BL */

  out->y_min = gy - hw; out->y_max = gy + hw;
  out->z_min = gz - hh; out->z_max = gz + hh;
  out->margin_left = hw; out->margin_right = hw;
  out->margin_bottom = hh; out->margin_top = hh;

  const float lat_err = gy - state->y;     /* gate center offset from drone */
  const float ver_err = gz - state->z;
  out->lateral_error_m = lat_err;
  out->vertical_error_m = ver_err;
  out->margin = fminf(hw - fabsf(lat_err), hh - fabsf(ver_err));
  out->sigma_m = 0.02f;
  out->confidence = 1.0f;
  out->distance_to_gate_m = range;
  out->has_bounds = true;
  out->has_distance = true;
  out->has_corners = true;
  out->projection_gate_clipped = false;
  out->valid = true;
  out->raw_valid = true;
  out->source = GATE_SOURCE_CAMERA_VALID;
  out->invalid_reason = GATE_INVALID_NONE;

  g_gate_center_x = gx; g_gate_center_y = gy; g_gate_center_z = gz;
  g_gate_range_m = range;
  g_gate_valid = 1;
  g_gate_dbg_reason = 0;
  return true;
}
