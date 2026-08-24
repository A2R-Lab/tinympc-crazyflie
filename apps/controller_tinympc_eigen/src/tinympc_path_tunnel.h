#pragma once

#include <math.h>
#include <stdbool.h>

typedef struct {
  float x;
  float y;
  float z;
} TinyMpcTunnelVector;

typedef struct {
  TinyMpcTunnelVector tangent;
  TinyMpcTunnelVector normal_1;
  TinyMpcTunnelVector normal_2;
  bool valid;
} TinyMpcTunnelFrame;

static inline TinyMpcTunnelVector tinyMpcTunnelVector(
    float x, float y, float z) {
  const TinyMpcTunnelVector vector = {x, y, z};
  return vector;
}

static inline float tinyMpcTunnelDot(
    TinyMpcTunnelVector a, TinyMpcTunnelVector b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline TinyMpcTunnelVector tinyMpcTunnelScale(
    TinyMpcTunnelVector vector, float scale) {
  return tinyMpcTunnelVector(
      vector.x * scale, vector.y * scale, vector.z * scale);
}

static inline TinyMpcTunnelVector tinyMpcTunnelSubtract(
    TinyMpcTunnelVector a, TinyMpcTunnelVector b) {
  return tinyMpcTunnelVector(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline TinyMpcTunnelVector tinyMpcTunnelCross(
    TinyMpcTunnelVector a, TinyMpcTunnelVector b) {
  return tinyMpcTunnelVector(
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x);
}

static inline bool tinyMpcTunnelNormalize(TinyMpcTunnelVector *vector) {
  const float norm_squared = tinyMpcTunnelDot(*vector, *vector);
  if (!isfinite(norm_squared) || norm_squared <= 1.0e-12f) {
    return false;
  }
  *vector = tinyMpcTunnelScale(*vector, 1.0f / sqrtf(norm_squared));
  return true;
}

/* Construct an orthonormal cross-section frame.  When a previous normal is
 * supplied it is projected onto the new normal plane (parallel transport),
 * avoiding arbitrary normal flips along a 3D horizon.  The initial frame uses
 * world up, so normal_1 is lateral and normal_2 is vertical on level paths. */
static inline TinyMpcTunnelFrame tinyMpcPathTunnelFrame(
    TinyMpcTunnelVector tangent,
    const TinyMpcTunnelVector *previous_normal_1) {
  TinyMpcTunnelFrame frame = {};
  frame.tangent = tangent;
  if (!tinyMpcTunnelNormalize(&frame.tangent)) {
    return frame;
  }

  bool have_normal = false;
  if (previous_normal_1 != NULL) {
    frame.normal_1 = tinyMpcTunnelSubtract(
        *previous_normal_1,
        tinyMpcTunnelScale(
            frame.tangent,
            tinyMpcTunnelDot(*previous_normal_1, frame.tangent)));
    have_normal = tinyMpcTunnelNormalize(&frame.normal_1);
  }
  if (!have_normal) {
    const TinyMpcTunnelVector world_up = tinyMpcTunnelVector(0.0f, 0.0f, 1.0f);
    frame.normal_1 = tinyMpcTunnelCross(world_up, frame.tangent);
    have_normal = tinyMpcTunnelNormalize(&frame.normal_1);
  }
  if (!have_normal) {
    const TinyMpcTunnelVector world_x = tinyMpcTunnelVector(1.0f, 0.0f, 0.0f);
    frame.normal_1 = tinyMpcTunnelCross(world_x, frame.tangent);
    have_normal = tinyMpcTunnelNormalize(&frame.normal_1);
  }
  if (!have_normal) {
    return frame;
  }

  frame.normal_2 = tinyMpcTunnelCross(frame.tangent, frame.normal_1);
  frame.valid = tinyMpcTunnelNormalize(&frame.normal_2);
  return frame;
}
