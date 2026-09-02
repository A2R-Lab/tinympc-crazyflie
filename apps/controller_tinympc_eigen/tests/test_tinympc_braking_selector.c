#include "tinympc_braking_selector.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static TinyMpcBrakingSelectorConfig configWithDwell(uint16_t dwell_steps) {
  const TinyMpcBrakingSelectorConfig config = {
      1.0f, 0.5f, 0.10f,
      {-0.5113f, -0.4919f, -0.4721f, -0.4519f, -0.4312f},
      0.12f, 0.20f, 0.12f, 0.05f, dwell_steps};
  return config;
}

static TinyMpcBrakingSelector initializedSelector(
    const TinyMpcBrakingSelectorConfig *config) {
  TinyMpcBrakingSelector selector;
  tinyMpcBrakingSelectorReset(&selector, config);
  return selector;
}

static TinyMpcBrakingSelection update(
    TinyMpcBrakingSelector *selector,
    const TinyMpcBrakingSelectorConfig *config,
    float reference_deceleration_mps2,
    float reference_pitch_rad,
    float measured_pitch_rad,
    float reference_roll_rad,
    float measured_roll_rad,
    float measured_speed_mps) {
  return tinyMpcBrakingSelectorUpdate(
      selector, config, reference_deceleration_mps2,
      reference_pitch_rad, measured_pitch_rad,
      reference_roll_rad, measured_roll_rad, measured_speed_mps);
}

static void testModelIdsAndNominalSpeeds(void) {
  const int expected_models[] = {11, 12, 13, 14, 15};
  const float expected_speeds[] = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
  for (uint8_t tier = 1u; tier <= TINYMPC_BRAKING_TIER_COUNT; ++tier) {
    assert(tinyMpcBrakingModelForTier(tier) == expected_models[tier - 1u]);
    assert(tinyMpcBrakingTierForModel(expected_models[tier - 1u]) == tier);
    assert(tinyMpcBrakingNominalSpeedMps(tier) == expected_speeds[tier - 1u]);
  }
  assert(tinyMpcBrakingModelForTier(0u) == 0);
  assert(tinyMpcBrakingModelForTier(6u) == 0);
  assert(tinyMpcBrakingTierForModel(10) == 0u);
  assert(tinyMpcBrakingTierForModel(16) == 0u);
}

static void testNearestTierMidpoints(void) {
  assert(tinyMpcNearestBrakingSpeedTier(0.0f) == 1u);
  assert(tinyMpcNearestBrakingSpeedTier(1.249f) == 1u);
  assert(tinyMpcNearestBrakingSpeedTier(1.25f) == 2u);
  assert(tinyMpcNearestBrakingSpeedTier(1.75f) == 3u);
  assert(tinyMpcNearestBrakingSpeedTier(2.25f) == 4u);
  assert(tinyMpcNearestBrakingSpeedTier(2.75f) == 5u);
  assert(tinyMpcNearestBrakingSpeedTier(10.0f) == 5u);
  assert(tinyMpcNearestBrakingSpeedTier(-0.01f) == 0u);
  assert(tinyMpcNearestBrakingSpeedTier(NAN) == 0u);
}

static void testDecelerationOwnsActivation(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(2u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  const float decelerations[] = {0.0f, 0.49f, 0.50f, 0.99f};
  for (unsigned int index = 0u;
       index < sizeof(decelerations) / sizeof(decelerations[0]); ++index) {
    const TinyMpcBrakingSelection selection = update(
        &selector, &config, decelerations[index], -0.45f, -0.45f,
        0.0f, 0.0f, 2.0f);
    assert(selection.sample_valid);
    assert(selection.active_model_id == 0);
    assert(!selection.switched && !selection.reset_optimizer);
  }
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, 0.20f, 0.20f, 0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 0);
  selection = update(
      &selector, &config, 6.0f, -0.099f, -0.099f, 0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 0);
  selection = update(
      &selector, &config, 6.0f, -0.47f, -0.47f, 0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 13);
}

static void testEverySpeedSelectsExpectedBundle(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  const float speeds[] = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
  for (unsigned int index = 0u;
       index < sizeof(speeds) / sizeof(speeds[0]); ++index) {
    TinyMpcBrakingSelector selector = initializedSelector(&config);
    const TinyMpcBrakingSelection selection = update(
        &selector, &config, 6.0f, -0.45f, -0.40f,
        0.0f, 0.0f, speeds[index]);
    assert(selection.sample_valid && selection.entry_local);
    assert(selection.active_model_id == 11 + (int)index);
    assert(selection.switched && selection.reset_optimizer);
  }
}

static void testEntryRequiresSpeedAndPitchLocality(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 0.79f);
  assert(!selection.entry_local && selection.active_model_id == 0);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.329f,
      0.0f, 0.0f, 1.0f);
  assert(!selection.entry_local && selection.active_model_id == 0);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.392f,
      0.0f, 0.0f, 1.19f);
  assert(selection.entry_local);
  assert(selection.active_model_id == 11);
}

static void testRollDemandAndMeasuredRollFailClosed(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(20u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 13);

  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.121f, 0.0f, 2.0f);
  assert(selection.sample_valid);
  assert(selection.active_model_id == 0);
  assert(selection.switched && selection.reset_optimizer);

  tinyMpcBrakingSelectorReset(&selector, &config);
  (void)update(
      &selector, &config, 6.0f, -0.45f, -0.45f, 0.0f, 0.0f, 2.0f);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, -0.121f, 2.0f);
  assert(selection.active_model_id == 0);
  assert(selection.switched && selection.reset_optimizer);
}

static void testRollBoundaryIsInclusive(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  const TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      config.maximum_level_roll_rad, -config.maximum_level_roll_rad, 1.5f);
  assert(selection.active_model_id == 12);
}

static void testSpeedHysteresisAndDwell(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(2u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.5f);
  assert(selection.active_model_id == 12);

  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.799f);
  assert(selection.active_model_id == 12);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.801f);
  assert(selection.active_model_id == 12 && !selection.switched);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.801f);
  assert(selection.active_model_id == 13);
  assert(selection.switched && selection.reset_optimizer);

  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.701f);
  assert(selection.active_model_id == 13);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.699f);
  assert(selection.active_model_id == 13);
  selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.699f);
  assert(selection.active_model_id == 12);
}

static void testTierSwitchRequiresCandidatePitchLocality(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.43f, -0.43f,
      0.0f, 0.0f, 3.0f);
  assert(selection.active_model_id == 15);

  selection = update(
      &selector, &config, 6.0f, -0.47f, -0.70f,
      0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 15);
  assert(!selection.switched);

  selection = update(
      &selector, &config, 6.0f, -0.47f, -0.47f,
      0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 13);
  assert(selection.switched && selection.reset_optimizer);
}

static void testNearZeroReferenceReleasesAfterDwell(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(2u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 2.5f);
  assert(selection.active_model_id == 14);
  for (int step = 0; step < 2; ++step) {
    selection = update(
        &selector, &config, config.exit_deceleration_mps2, -0.05f, -0.30f,
        0.0f, 0.0f, 2.5f);
    assert(selection.active_model_id == 14);
  }
  selection = update(
      &selector, &config, 0.0f, 0.0f, -0.25f,
      0.0f, 0.0f, 2.5f);
  assert(selection.active_model_id == 0);
  assert(selection.switched && selection.reset_optimizer);
}

static void testDecelerationThresholdHysteresisRetainsBrake(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  TinyMpcBrakingSelection selection = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 1.0f);
  assert(selection.active_model_id == 11);
  selection = update(
      &selector, &config, 0.75f, -0.10f, -0.20f,
      0.0f, 0.0f, 1.0f);
  assert(selection.active_model_id == 11 && !selection.switched);
  selection = update(
      &selector, &config, 0.49f, -0.049f, -0.20f,
      0.0f, 0.0f, 1.0f);
  assert(selection.active_model_id == 0);
}

static void testNonfiniteInputsFailClosed(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(100u);
  const float invalid_values[] = {NAN, INFINITY, -INFINITY};
  for (unsigned int field = 0u; field < 6u; ++field) {
    for (unsigned int value = 0u;
         value < sizeof(invalid_values) / sizeof(invalid_values[0]); ++value) {
      TinyMpcBrakingSelector selector = initializedSelector(&config);
      (void)update(
          &selector, &config, 6.0f, -0.45f, -0.45f,
          0.0f, 0.0f, 3.0f);
      float inputs[] = {6.0f, -0.45f, -0.45f, 0.0f, 0.0f, 3.0f};
      inputs[field] = invalid_values[value];
      const TinyMpcBrakingSelection selection = update(
          &selector, &config, inputs[0], inputs[1], inputs[2], inputs[3],
          inputs[4], inputs[5]);
      assert(!selection.sample_valid);
      assert(selection.active_model_id == 0);
      assert(selection.switched && selection.reset_optimizer);
    }
  }
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  const TinyMpcBrakingSelection negative_speed = update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, -0.01f);
  assert(!negative_speed.sample_valid);
  assert(negative_speed.active_model_id == 0);
}

static void testInvalidConfigAndStateFailClosed(void) {
  TinyMpcBrakingSelectorConfig invalid = configWithDwell(0u);
  invalid.maximum_entry_speed_error_mps = 0.5f;
  TinyMpcBrakingSelector selector = initializedSelector(&invalid);
  TinyMpcBrakingSelection selection = update(
      &selector, &invalid, 6.0f, -1.0f, -1.0f,
      0.0f, 0.0f, 2.0f);
  assert(!selection.sample_valid);
  assert(selection.active_model_id == 0);

  const TinyMpcBrakingSelectorConfig config = configWithDwell(0u);
  selector.active_model_id = 99;
  selector.steps_since_switch = 0u;
  selector.switch_count = 42u;
  selection = update(
      &selector, &config, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 2.0f);
  assert(selection.sample_valid);
  assert(selection.active_model_id == 0);
  assert(selector.switch_count == 0u);
}

static void testResetAndNullAreSafe(void) {
  const TinyMpcBrakingSelectorConfig config = configWithDwell(4u);
  tinyMpcBrakingSelectorReset(NULL, &config);
  TinyMpcBrakingSelector selector = initializedSelector(&config);
  (void)update(
      &selector, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 2.0f);
  tinyMpcBrakingSelectorReset(&selector, &config);
  assert(selector.active_model_id == 0);
  assert(selector.steps_since_switch == config.minimum_dwell_steps);
  assert(selector.switch_count == 0u);
  const TinyMpcBrakingSelection selection = tinyMpcBrakingSelectorUpdate(
      NULL, &config, 6.0f, -0.45f, -0.45f,
      0.0f, 0.0f, 2.0f);
  assert(selection.active_model_id == 0);
  assert(!selection.sample_valid);
}

int main(void) {
  testModelIdsAndNominalSpeeds();
  testNearestTierMidpoints();
  testDecelerationOwnsActivation();
  testEverySpeedSelectsExpectedBundle();
  testEntryRequiresSpeedAndPitchLocality();
  testRollDemandAndMeasuredRollFailClosed();
  testRollBoundaryIsInclusive();
  testSpeedHysteresisAndDwell();
  testTierSwitchRequiresCandidatePitchLocality();
  testNearZeroReferenceReleasesAfterDwell();
  testDecelerationThresholdHysteresisRetainsBrake();
  testNonfiniteInputsFailClosed();
  testInvalidConfigAndStateFailClosed();
  testResetAndNullAreSafe();
  puts("tinympc braking selector tests passed");
  return 0;
}
