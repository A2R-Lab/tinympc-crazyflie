#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "tinympc_progress_yaw.h"

static bool near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static TinyMpcProgressQuaternion quaternionFromRpy(
    float roll, float pitch, float yaw) {
  const float cr = cosf(0.5f * roll);
  const float sr = sinf(0.5f * roll);
  const float cp = cosf(0.5f * pitch);
  const float sp = sinf(0.5f * pitch);
  const float cy = cosf(0.5f * yaw);
  const float sy = sinf(0.5f * yaw);
  return tinyMpcProgressQuaternionNormalize(tinyMpcProgressQuaternionMake(
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy));
}

static bool bodyRateFinite(TinyMpcProgressBodyRate rate) {
  return isfinite(rate.x) && isfinite(rate.y) && isfinite(rate.z);
}

static void testGeometricBankReference(void) {
  const TinyMpcProgressBankReference hover =
      tinyMpcProgressBankReferenceFromAcceleration(
          0.0f, 0.0f, 0.0f, 0.7f, 9.81f);
  assert(hover.valid);
  assert(near(hover.roll_rad, 0.0f, 1.0e-7f));
  assert(near(hover.pitch_rad, 0.0f, 1.0e-7f));
  assert(near(hover.thrust_scale, 0.0f, 1.0e-7f));

  const TinyMpcProgressBankReference left_turn =
      tinyMpcProgressBankReferenceFromAcceleration(
          0.0f, 2.0f, 0.0f, 0.0f, 9.81f);
  assert(left_turn.valid);
  assert(near(left_turn.roll_rad, -atan2f(2.0f, 9.81f), 1.0e-6f));
  assert(near(left_turn.pitch_rad, 0.0f, 1.0e-6f));
  assert(left_turn.thrust_scale > 0.0f);

  const TinyMpcProgressBankReference forward_acceleration =
      tinyMpcProgressBankReferenceFromAcceleration(
          2.0f, 0.0f, 0.0f, 0.0f, 9.81f);
  assert(forward_acceleration.valid);
  assert(near(forward_acceleration.roll_rad, 0.0f, 1.0e-6f));
  assert(near(
      forward_acceleration.pitch_rad, atan2f(2.0f, 9.81f), 1.0e-6f));
}

static void testUncappedHorizonFollowsFullGeometricTangent(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const float reference_phase = 0.0f;
  const float maximum_deviation = 15.0f * degrees_to_radians;
  float horizon_yaw = reference_phase;

  const float geometric_yaws_deg[] = {0.0f, 11.0f, 22.0f, 33.0f, 44.0f};
  for (unsigned int i = 0;
       i < sizeof(geometric_yaws_deg) / sizeof(geometric_yaws_deg[0]); ++i) {
    const float geometric_yaw =
        geometric_yaws_deg[i] * degrees_to_radians;
    horizon_yaw = tinyMpcProgressHorizonYaw(
        geometric_yaw, horizon_yaw, reference_phase, maximum_deviation,
        false);
    assert(near(horizon_yaw, geometric_yaw, 1.0e-6f));
  }

  assert(horizon_yaw > maximum_deviation + 0.50f);
}

static void testDefaultHorizonRemainsLocallyBounded(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const float reference_phase = 0.0f;
  const float maximum_deviation = 15.0f * degrees_to_radians;
  float horizon_yaw = reference_phase;

  const float geometric_yaws_deg[] = {0.0f, 11.0f, 22.0f, 33.0f, 44.0f};
  for (unsigned int i = 0;
       i < sizeof(geometric_yaws_deg) / sizeof(geometric_yaws_deg[0]); ++i) {
    horizon_yaw = tinyMpcProgressHorizonYaw(
        geometric_yaws_deg[i] * degrees_to_radians, horizon_yaw,
        reference_phase, maximum_deviation, true);
    assert(horizon_yaw <= reference_phase + maximum_deviation + 1.0e-6f);
  }

  assert(near(horizon_yaw, maximum_deviation, 1.0e-6f));
}

static void testUncappedHorizonIsContinuousAcrossAngleSeam(void) {
  const float previous_horizon_yaw = 3.10f;
  const float geometric_yaw = -3.00f;
  const float horizon_yaw = tinyMpcProgressHorizonYaw(
      geometric_yaw, previous_horizon_yaw, 3.10f, 0.2617993878f, false);
  assert(horizon_yaw > previous_horizon_yaw);
  assert(near(horizon_yaw, 3.2831853f, 1.0e-5f));
}

static void testTangentHeadingSequenceUnwrapsContinuously(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const float wrapped_yaws_deg[] = {170.0f, 179.0f, -179.0f, -170.0f};
  const float expected_yaws_deg[] = {170.0f, 179.0f, 181.0f, 190.0f};
  float yaw = wrapped_yaws_deg[0] * degrees_to_radians;
  for (unsigned int i = 0;
       i < sizeof(wrapped_yaws_deg) / sizeof(wrapped_yaws_deg[0]); ++i) {
    yaw = tinyMpcProgressUnwrapYawNear(
        wrapped_yaws_deg[i] * degrees_to_radians, yaw);
    assert(near(yaw, expected_yaws_deg[i] * degrees_to_radians, 1.0e-5f));
  }
}

static void testHeadingAlignmentSlewBoundsInitialQuarterTurn(void) {
  const float maximum_step = 0.60f * 0.02f;
  float yaw = 0.0f;
  yaw = tinyMpcProgressSlewYawToward(
      yaw, -0.5f * 3.14159265358979323846f, maximum_step);
  assert(near(yaw, -maximum_step, 1.0e-7f));
  for (int step = 0; step < 200; ++step) {
    const float previous = yaw;
    yaw = tinyMpcProgressSlewYawToward(
        yaw, -0.5f * 3.14159265358979323846f, maximum_step);
    assert(fabsf(yaw - previous) <= maximum_step + 1.0e-7f);
  }
  assert(near(yaw, -0.5f * 3.14159265358979323846f, 1.0e-6f));
}

static void testHeadingAlignmentSlewUsesShortestSeamDirection(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const float current = 179.0f * degrees_to_radians;
  const float next = tinyMpcProgressSlewYawToward(
      current, -179.0f * degrees_to_radians,
      0.5f * degrees_to_radians);
  assert(next > current);
  assert(near(next - current, 0.5f * degrees_to_radians, 1.0e-6f));
}

static void testInitialHeadingAlignmentDoesNotRearm(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const float maximum_step = 0.60f * 0.02f;
  const float release_error = 3.0f * degrees_to_radians;
  float phase = 0.0f;
  float remaining = 0.0f;
  bool complete = false;
  bool active = true;
  for (int step = 0; step < 200 && active; ++step) {
    active = tinyMpcProgressUpdateInitialHeadingAlignment(
        90.0f * degrees_to_radians, maximum_step, release_error,
        &phase, &complete, &remaining);
  }
  assert(!active);
  assert(complete);
  assert(fabsf(remaining) <= release_error);

  active = tinyMpcProgressUpdateInitialHeadingAlignment(
      105.0f * degrees_to_radians, maximum_step, release_error,
      &phase, &complete, &remaining);
  assert(!active);
  assert(complete);
  assert(near(phase, 105.0f * degrees_to_radians, 1.0e-6f));
  assert(near(remaining, 0.0f, 1.0e-7f));

  phase = 179.0f * degrees_to_radians;
  active = tinyMpcProgressUpdateInitialHeadingAlignment(
      -179.0f * degrees_to_radians, maximum_step, release_error,
      &phase, &complete, &remaining);
  assert(!active);
  assert(phase > 3.14159265358979323846f);
  assert(near(phase, 181.0f * degrees_to_radians, 1.0e-6f));
}

static void testQuaternionIdentityHasZeroBodyRate(void) {
  const TinyMpcProgressQuaternion identity = {0.0f, 0.0f, 0.0f, 1.0f};
  const TinyMpcProgressBodyRate rate = tinyMpcProgressQuaternionBodyRate(
      identity, identity, 0.02f);
  assert(near(rate.x, 0.0f, 1.0e-7f));
  assert(near(rate.y, 0.0f, 1.0e-7f));
  assert(near(rate.z, 0.0f, 1.0e-7f));
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      identity, identity, rate, 0.02f) <= 1.0e-5f);
}

static void testPureYawProducesPositiveBodyR(void) {
  const TinyMpcProgressQuaternion current = quaternionFromRpy(
      0.0f, 0.0f, 0.2f);
  const TinyMpcProgressQuaternion next = quaternionFromRpy(
      0.0f, 0.0f, 0.22f);
  const TinyMpcProgressBodyRate rate = tinyMpcProgressQuaternionBodyRate(
      current, next, 0.02f);
  assert(near(rate.x, 0.0f, 1.0e-5f));
  assert(near(rate.y, 0.0f, 1.0e-5f));
  assert(near(rate.z, 1.0f, 1.0e-5f));
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      current, next, rate, 0.02f) <= 1.0e-5f);
}

static void testBankedCoordinatedYawProducesCoupledBodyRates(void) {
  const TinyMpcProgressQuaternion current = quaternionFromRpy(
      0.35f, -0.20f, 0.40f);
  const TinyMpcProgressQuaternion next = quaternionFromRpy(
      0.35f, -0.20f, 0.44f);
  const TinyMpcProgressBodyRate rate = tinyMpcProgressQuaternionBodyRate(
      current, next, 0.02f);
  assert(fabsf(rate.x) > 0.10f);
  assert(fabsf(rate.y) > 0.10f);
  assert(rate.z > 1.0f);
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      current, next, rate, 0.02f) <= 1.0e-5f);
}

static void testBodyConventionMatchesForwardReconstruction(void) {
  const float dt_s = 0.02f;
  const TinyMpcProgressQuaternion current = quaternionFromRpy(
      -0.31f, 0.17f, 1.20f);
  const TinyMpcProgressBodyRate expected = {0.70f, -0.40f, 1.10f};
  const TinyMpcProgressBodyRate rotation_vector = {
      expected.x * dt_s, expected.y * dt_s, expected.z * dt_s};
  const TinyMpcProgressQuaternion next = tinyMpcProgressQuaternionMultiply(
      tinyMpcProgressQuaternionExp(rotation_vector), current);
  const TinyMpcProgressBodyRate recovered =
      tinyMpcProgressQuaternionBodyRate(current, next, dt_s);
  assert(near(recovered.x, expected.x, 2.0e-5f));
  assert(near(recovered.y, expected.y, 2.0e-5f));
  assert(near(recovered.z, expected.z, 2.0e-5f));
  const float residual = tinyMpcProgressQuaternionReconstructionResidual(
      current, next, recovered, dt_s);
  assert(residual <= 1.0e-5f);
  printf("forward reconstruction residual %.9g rad\n", (double)residual);
}

static void testQuaternionYawSeamUsesShortestArc(void) {
  const float degrees_to_radians = 0.01745329251994329577f;
  const TinyMpcProgressQuaternion current = quaternionFromRpy(
      0.0f, 0.0f, 179.0f * degrees_to_radians);
  const TinyMpcProgressQuaternion next = quaternionFromRpy(
      0.0f, 0.0f, -179.0f * degrees_to_radians);
  const TinyMpcProgressBodyRate rate = tinyMpcProgressQuaternionBodyRate(
      current, next, 0.02f);
  assert(rate.z > 0.0f);
  assert(near(rate.z, 1.74532925f, 2.0e-5f));
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      current, next, rate, 0.02f) <= 1.0e-5f);
}

static void testNearZeroAndNearPiRatesRemainFinite(void) {
  const TinyMpcProgressQuaternion identity = {0.0f, 0.0f, 0.0f, 1.0f};
  const TinyMpcProgressQuaternion near_zero = tinyMpcProgressQuaternionExp(
      tinyMpcProgressBodyRateMake(1.0e-9f, -2.0e-9f, 3.0e-9f));
  const TinyMpcProgressQuaternion near_pi = tinyMpcProgressQuaternionExp(
      tinyMpcProgressBodyRateMake(3.1415752f, 0.0f, 0.0f));
  const TinyMpcProgressBodyRate tiny_rate =
      tinyMpcProgressQuaternionBodyRate(identity, near_zero, 1.0f);
  const TinyMpcProgressBodyRate large_rate =
      tinyMpcProgressQuaternionBodyRate(identity, near_pi, 1.0f);
  assert(bodyRateFinite(tiny_rate));
  assert(bodyRateFinite(large_rate));
  assert(near(tiny_rate.x, 1.0e-9f, 1.0e-10f));
  assert(near(large_rate.x, 3.1415752f, 2.0e-5f));
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      identity, near_zero, tiny_rate, 1.0f) <= 1.0e-5f);
  assert(tinyMpcProgressQuaternionReconstructionResidual(
      identity, near_pi, large_rate, 1.0f) <= 1.0e-5f);
}

static void testHorizonCopiesFinalIntervalRate(void) {
  TinyMpcProgressQuaternion attitudes[3] = {
      quaternionFromRpy(0.0f, 0.0f, 0.00f),
      quaternionFromRpy(0.1f, 0.0f, 0.02f),
      quaternionFromRpy(0.2f, 0.0f, 0.04f),
  };
  TinyMpcProgressBodyRate rates[3];
  float residuals[2];
  tinyMpcProgressQuaternionHorizonBodyRates(
      attitudes, 3u, 0.02f, rates, residuals);
  assert(near(rates[2].x, rates[1].x, 1.0e-7f));
  assert(near(rates[2].y, rates[1].y, 1.0e-7f));
  assert(near(rates[2].z, rates[1].z, 1.0e-7f));
  assert(residuals[0] <= 1.0e-5f);
  assert(residuals[1] <= 1.0e-5f);
}

int main(void) {
  testGeometricBankReference();
  testUncappedHorizonFollowsFullGeometricTangent();
  testDefaultHorizonRemainsLocallyBounded();
  testUncappedHorizonIsContinuousAcrossAngleSeam();
  testTangentHeadingSequenceUnwrapsContinuously();
  testHeadingAlignmentSlewBoundsInitialQuarterTurn();
  testHeadingAlignmentSlewUsesShortestSeamDirection();
  testInitialHeadingAlignmentDoesNotRearm();
  testQuaternionIdentityHasZeroBodyRate();
  testPureYawProducesPositiveBodyR();
  testBankedCoordinatedYawProducesCoupledBodyRates();
  testBodyConventionMatchesForwardReconstruction();
  testQuaternionYawSeamUsesShortestArc();
  testNearZeroAndNearPiRatesRemainFinite();
  testHorizonCopiesFinalIntervalRate();
  puts("tinympc progress yaw tests passed");
  return 0;
}
