#include "tinympc_vision_residual_authority.h"

#include <assert.h>
#include <math.h>

static bool near(float actual, float expected) {
  return fabsf(actual - expected) < 1.0e-6f;
}

int main(void) {
  const TinyMpcVisionResidualAuthority authority =
      tinyMpcVisionResidualFullAuthorityV1();
  assert(tinyMpcVisionResidualAuthorityIsValid(&authority));
  assert(authority.abi_version == 1u);
  assert(near(authority.lateral_rate_limit_mps,
              (float)TINYMPC_VISION_RESIDUAL_LATERAL_RATE_LIMIT_MPS));
  assert(near(authority.vertical_rate_limit_mps,
              (float)TINYMPC_VISION_RESIDUAL_VERTICAL_RATE_LIMIT_MPS));
  assert(near(tinyMpcVisionResidualLateralOffsetLimit(&authority, true),
              (float)TINYMPC_VISION_RESIDUAL_GATE_LATERAL_OFFSET_LIMIT_M));
  assert(near(tinyMpcVisionResidualLateralOffsetLimit(&authority, false),
              (float)TINYMPC_VISION_RESIDUAL_FULL_LATERAL_OFFSET_LIMIT_M));

  assert(tinyMpcVisionResidualPacketWithinAuthority(
      &authority, authority.lateral_rate_limit_mps,
      -authority.vertical_rate_limit_mps,
      authority.minimum_progress_speed_scale));
  assert(tinyMpcVisionResidualPacketWithinAuthority(
      &authority, -authority.lateral_rate_limit_mps,
      authority.vertical_rate_limit_mps,
      authority.maximum_progress_speed_scale));
  assert(!tinyMpcVisionResidualPacketWithinAuthority(
      &authority, authority.lateral_rate_limit_mps + 0.001f, 0.0f, 1.0f));
  assert(!tinyMpcVisionResidualPacketWithinAuthority(
      &authority, 0.0f, authority.vertical_rate_limit_mps + 0.001f, 1.0f));
  assert(!tinyMpcVisionResidualPacketWithinAuthority(
      &authority, 0.0f, 0.0f,
      authority.minimum_progress_speed_scale - 0.001f));
  assert(!tinyMpcVisionResidualPacketWithinAuthority(
      &authority, NAN, 0.0f, 1.0f));

  assert(near(tinyMpcVisionResidualClampSymmetric(
                  2.0f, authority.full_lateral_offset_limit_m),
              authority.full_lateral_offset_limit_m));
  assert(near(tinyMpcVisionResidualClampSymmetric(
                  -2.0f, authority.vertical_offset_limit_m),
              -authority.vertical_offset_limit_m));
  assert(near(tinyMpcVisionResidualClampSymmetric(
                  NAN, authority.vertical_offset_limit_m),
              0.0f));

  TinyMpcVisionResidualAuthority invalid = authority;
  invalid.abi_version++;
  assert(!tinyMpcVisionResidualAuthorityIsValid(&invalid));
  invalid = authority;
  invalid.gate_lateral_offset_limit_m =
      invalid.full_lateral_offset_limit_m + 0.01f;
  assert(!tinyMpcVisionResidualAuthorityIsValid(&invalid));
  return 0;
}
