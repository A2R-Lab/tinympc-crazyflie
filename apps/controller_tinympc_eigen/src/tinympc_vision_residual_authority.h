#ifndef TINYMPC_VISION_RESIDUAL_AUTHORITY_H
#define TINYMPC_VISION_RESIDUAL_AUTHORITY_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Versioned physical authority for vision packet V4 residual references.
 *
 * These defaults are the conservative, previously approved contract.  The
 * build-time values are deliberately explicit so a future, flight-validated
 * full-authority profile can be selected without duplicating decoder limits.
 * Do not raise them from model output statistics alone.
 */
#define TINYMPC_VISION_RESIDUAL_AUTHORITY_ABI_VERSION 1u

#ifndef TINYMPC_VISION_RESIDUAL_AUTHORITY_VERSION
#define TINYMPC_VISION_RESIDUAL_AUTHORITY_VERSION 1
#endif
#if TINYMPC_VISION_RESIDUAL_AUTHORITY_VERSION != 1
#error "unsupported TinyMPC vision residual authority version"
#endif

#ifndef TINYMPC_MAXIMUM_LATERAL_RATE_MPS
#define TINYMPC_MAXIMUM_LATERAL_RATE_MPS 2.25f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_LATERAL_RATE_LIMIT_MPS
#define TINYMPC_VISION_RESIDUAL_LATERAL_RATE_LIMIT_MPS \
  TINYMPC_MAXIMUM_LATERAL_RATE_MPS
#endif
#ifndef TINYMPC_VISION_RESIDUAL_VERTICAL_RATE_LIMIT_MPS
#define TINYMPC_VISION_RESIDUAL_VERTICAL_RATE_LIMIT_MPS 0.20f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_GATE_LATERAL_OFFSET_LIMIT_M
#define TINYMPC_VISION_RESIDUAL_GATE_LATERAL_OFFSET_LIMIT_M 0.25f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_FULL_LATERAL_OFFSET_LIMIT_M
#define TINYMPC_VISION_RESIDUAL_FULL_LATERAL_OFFSET_LIMIT_M 0.45f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_VERTICAL_OFFSET_LIMIT_M
#define TINYMPC_VISION_RESIDUAL_VERTICAL_OFFSET_LIMIT_M 0.25f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_MIN_PROGRESS_SPEED_SCALE
#define TINYMPC_VISION_RESIDUAL_MIN_PROGRESS_SPEED_SCALE 0.20f
#endif
#ifndef TINYMPC_VISION_RESIDUAL_MAX_PROGRESS_SPEED_SCALE
#define TINYMPC_VISION_RESIDUAL_MAX_PROGRESS_SPEED_SCALE 1.00f
#endif

typedef struct {
  uint32_t abi_version;
  float lateral_rate_limit_mps;
  float vertical_rate_limit_mps;
  float gate_lateral_offset_limit_m;
  float full_lateral_offset_limit_m;
  float vertical_offset_limit_m;
  float minimum_progress_speed_scale;
  float maximum_progress_speed_scale;
} TinyMpcVisionResidualAuthority;

static inline TinyMpcVisionResidualAuthority
tinyMpcVisionResidualFullAuthorityV1(void) {
  /* "Full" means the existing non-gate envelope, not unbounded authority. */
  const TinyMpcVisionResidualAuthority authority = {
      TINYMPC_VISION_RESIDUAL_AUTHORITY_ABI_VERSION,
      (float)TINYMPC_VISION_RESIDUAL_LATERAL_RATE_LIMIT_MPS,
      (float)TINYMPC_VISION_RESIDUAL_VERTICAL_RATE_LIMIT_MPS,
      (float)TINYMPC_VISION_RESIDUAL_GATE_LATERAL_OFFSET_LIMIT_M,
      (float)TINYMPC_VISION_RESIDUAL_FULL_LATERAL_OFFSET_LIMIT_M,
      (float)TINYMPC_VISION_RESIDUAL_VERTICAL_OFFSET_LIMIT_M,
      (float)TINYMPC_VISION_RESIDUAL_MIN_PROGRESS_SPEED_SCALE,
      (float)TINYMPC_VISION_RESIDUAL_MAX_PROGRESS_SPEED_SCALE,
  };
  return authority;
}

static inline bool tinyMpcVisionResidualAuthorityIsValid(
    const TinyMpcVisionResidualAuthority *authority) {
  return authority != NULL &&
      authority->abi_version == TINYMPC_VISION_RESIDUAL_AUTHORITY_ABI_VERSION &&
      isfinite(authority->lateral_rate_limit_mps) &&
      authority->lateral_rate_limit_mps > 0.0f &&
      isfinite(authority->vertical_rate_limit_mps) &&
      authority->vertical_rate_limit_mps > 0.0f &&
      isfinite(authority->gate_lateral_offset_limit_m) &&
      authority->gate_lateral_offset_limit_m > 0.0f &&
      isfinite(authority->full_lateral_offset_limit_m) &&
      authority->full_lateral_offset_limit_m >=
          authority->gate_lateral_offset_limit_m &&
      isfinite(authority->vertical_offset_limit_m) &&
      authority->vertical_offset_limit_m > 0.0f &&
      isfinite(authority->minimum_progress_speed_scale) &&
      authority->minimum_progress_speed_scale > 0.0f &&
      isfinite(authority->maximum_progress_speed_scale) &&
      authority->maximum_progress_speed_scale >=
          authority->minimum_progress_speed_scale;
}

static inline bool tinyMpcVisionResidualPacketWithinAuthority(
    const TinyMpcVisionResidualAuthority *authority,
    float lateral_rate_mps, float vertical_rate_mps,
    float progress_speed_scale) {
  return tinyMpcVisionResidualAuthorityIsValid(authority) &&
      isfinite(lateral_rate_mps) &&
      fabsf(lateral_rate_mps) <= authority->lateral_rate_limit_mps &&
      isfinite(vertical_rate_mps) &&
      fabsf(vertical_rate_mps) <= authority->vertical_rate_limit_mps &&
      isfinite(progress_speed_scale) &&
      progress_speed_scale >= authority->minimum_progress_speed_scale &&
      progress_speed_scale <= authority->maximum_progress_speed_scale;
}

static inline float tinyMpcVisionResidualClampSymmetric(
    float value, float limit) {
  if (!isfinite(value) || !isfinite(limit) || limit <= 0.0f) {
    return 0.0f;
  }
  return fminf(fmaxf(value, -limit), limit);
}

static inline float tinyMpcVisionResidualLateralOffsetLimit(
    const TinyMpcVisionResidualAuthority *authority,
    bool gate_envelope_active) {
  return gate_envelope_active ? authority->gate_lateral_offset_limit_m
                              : authority->full_lateral_offset_limit_m;
}

#endif
