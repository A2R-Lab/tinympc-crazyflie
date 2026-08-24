#include "tinympc_bank_selector.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static TinyMpcBankSelectorConfig configWithDwell(uint16_t dwell_steps) {
  const TinyMpcBankSelectorConfig config = {
      0.135f, 0.297f, 0.497f, 0.705f, 0.886f,
      0.067f, 0.045f, 0.020f, dwell_steps};
  return config;
}

static TinyMpcBankSelector initializedSelector(
    const TinyMpcBankSelectorConfig *config) {
  TinyMpcBankSelector selector;
  tinyMpcBankSelectorReset(&selector, config);
  return selector;
}

static void testStraightDemandStaysLevel(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(2u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  for (int step = 0; step < 20; ++step) {
    const TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
        &selector, &config, 0.0f, 0.01f);
    assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
    assert(!selection.switched);
    assert(!selection.reset_optimizer);
  }
}

static void testSignedLeftAndRightSelection(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.14f, 0.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  assert(selection.switched && selection.reset_optimizer);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.14f, -0.15f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  assert(selection.switched && selection.reset_optimizer);
  assert(selector.switch_count == 2u);
}

static void testMagnitudeSelectsEverySignedTier(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  const float demands[] = {
      -config.low_bank_rad, config.low_bank_rad,
      -config.medium_bank_rad, config.medium_bank_rad,
      -config.high_bank_rad, config.high_bank_rad,
      -config.very_high_bank_rad, config.very_high_bank_rad,
      -config.maximum_bank_rad, config.maximum_bank_rad,
  };
  const TinyMpcBankModelId expected[] = {
      TINYMPC_BANK_MODEL_LEFT_LOW, TINYMPC_BANK_MODEL_RIGHT_LOW,
      TINYMPC_BANK_MODEL_LEFT_MEDIUM, TINYMPC_BANK_MODEL_RIGHT_MEDIUM,
      TINYMPC_BANK_MODEL_LEFT_HIGH, TINYMPC_BANK_MODEL_RIGHT_HIGH,
      TINYMPC_BANK_MODEL_LEFT_VERY_HIGH,
      TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH,
      TINYMPC_BANK_MODEL_LEFT_MAXIMUM,
      TINYMPC_BANK_MODEL_RIGHT_MAXIMUM,
  };
  for (unsigned int index = 0u;
       index < sizeof(demands) / sizeof(demands[0]); ++index) {
    TinyMpcBankSelector selector = initializedSelector(&config);
    const TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
        &selector, &config, demands[index], demands[index]);
    assert(selection.active_model == expected[index]);
    assert(selection.switched && selection.reset_optimizer);
  }
}

static void testThresholdsAndHysteresisUseReferenceAndState(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.066f, 0.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.067f, 0.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.050f, -0.020f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.020f, -0.050f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.044f, -0.044f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.0f, 0.14f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.14f, 0.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.0f, 0.14f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.235f, 0.235f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.230f, 0.230f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.236f, 0.236f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_MEDIUM);
}

static void testMinimumDwellDelaysOppositeSwitch(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(3u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.14f, -0.14f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  for (int held_step = 0; held_step < 3; ++held_step) {
    selection = tinyMpcBankSelectorUpdate(
        &selector, &config, 0.14f, 0.14f);
    assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
    assert(!selection.switched);
  }
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.14f, 0.14f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  assert(selection.switched && selection.reset_optimizer);
}

static void testMeasuredDivergenceCannotPromoteReferencedTier(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -config.low_bank_rad, 0.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  for (int step = 0; step < 20; ++step) {
    selection = tinyMpcBankSelectorUpdate(
        &selector, &config, -config.low_bank_rad,
        -1.5f * config.high_bank_rad);
    assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
    assert(selection.signed_bank_demand_rad == -config.low_bank_rad);
    assert(!selection.switched);
  }
}

static void testLevelCrossingRecoveryCannotPromoteOrFlipChart(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, config.low_bank_rad, config.low_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.03f, 2.0f * config.maximum_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  assert(fabsf(selection.signed_bank_demand_rad - config.low_bank_rad)
      < 1.0e-6f);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, 0.03f, -2.0f * config.maximum_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  assert(selection.signed_bank_demand_rad > 0.0f);

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -config.medium_bank_rad,
      -config.medium_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_MEDIUM);
}

static void testResetRestoresDeterministicLevelState(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(4u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  (void)tinyMpcBankSelectorUpdate(&selector, &config, -0.14f, -0.14f);
  assert(selector.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  tinyMpcBankSelectorReset(&selector, &config);
  assert(selector.active_model == TINYMPC_BANK_MODEL_LEVEL);
  assert(selector.steps_since_switch == config.minimum_dwell_steps);
  assert(selector.switch_count == 0u);
}

static void testInvalidAndNonfiniteDemandCannotSelectSide(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, NAN, INFINITY);
  assert(!selection.demand_valid);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
  assert(isfinite(selection.signed_bank_demand_rad));

  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, NAN, -0.14f);
  assert(!selection.demand_valid);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, -0.14f, NAN);
  assert(selection.demand_valid);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, NAN, NAN);
  assert(!selection.demand_valid);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_LOW);
  assert(!selection.switched);
}

static void testInvalidConfigFailsClosed(void) {
  const TinyMpcBankSelectorConfig invalid = {
      0.135f, 0.297f, 0.497f, 0.705f, 0.886f,
      0.10f, 0.20f, 0.02f, 0u};
  TinyMpcBankSelector selector = initializedSelector(&invalid);
  const TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &invalid, -1.0f, -1.0f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
  assert(!selection.demand_valid);
  assert(!selection.switched);
}

static void testSteadyTurnEntryRequiresMeasuredMotionSupport(void) {
  assert(!tinyMpcBankEntrySupportedByMotion(
      -0.135f, -0.135f, 0.0f, 0.0f));
  assert(!tinyMpcBankEntrySupportedByMotion(
      -0.135f, 0.0f, 1.0f, 0.0f));
  assert(!tinyMpcBankEntrySupportedByMotion(
      -0.135f, 0.090f, 1.0f, 0.80f));
  assert(!tinyMpcBankEntrySupportedByMotion(
      -0.135f, -0.040f, 1.0f, 0.69f));
  assert(tinyMpcBankEntrySupportedByMotion(
      -0.135f, -0.050f, 1.0f, 0.60f));
  assert(!tinyMpcBankEntrySupportedByMotion(
      -0.135f, -0.120f, 1.0f, 0.59f));
  assert(tinyMpcBankEntrySupportedByMotion(
      -0.135f, -0.120f, 1.0f, 0.90f));
  assert(tinyMpcBankEntrySupportedByMotion(
      0.040f, 0.0f, 1.0f, 0.0f));
  assert(!tinyMpcBankEntrySupportedByMotion(
      NAN, -0.10f, 1.0f, 0.8f));
}

static void testDirectTierEntryRequiresNominalSpeedLocality(void) {
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(2.0f, 0.0f, 0.0f, 0.65f));
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(2.0f, 1.29f, 1.30f, 0.65f));
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(2.0f, 1.30f, 1.29f, 0.65f));
  assert(tinyMpcBankEntrySupportedByNominalSpeed(2.0f, 1.30f, 1.30f, 0.65f));
  assert(tinyMpcBankEntrySupportedByNominalSpeed(1.0f, 0.20f, 0.20f, 0.20f));
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(NAN, 1.0f, 1.0f, 0.65f));
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(-1.0f, 1.0f, 1.0f, 0.65f));
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(1.0f, 1.0f, 1.0f, 1.1f));
}

static void testOnlyHighPromotionUsesTighterSpeedLocality(void) {
  assert(tinyMpcBankCandidateSpeedLocalityFraction(true, false, 1u) == 0.20f);
  assert(tinyMpcBankCandidateSpeedLocalityFraction(false, false, 2u) == 0.70f);
  /* Preserve the prior high-demand initial-low gate; this is not promotion. */
  assert(tinyMpcBankCandidateSpeedLocalityFraction(false, false, 3u) == 0.65f);
  const float high_fraction = tinyMpcBankCandidateSpeedLocalityFraction(
      false, true, 3u);
  assert(high_fraction == 0.75f);
  assert(!tinyMpcBankEntrySupportedByNominalSpeed(
      2.0f, 2.0f, 1.499f, high_fraction));
  assert(tinyMpcBankEntrySupportedByNominalSpeed(
      2.0f, 2.0f, 1.500f, high_fraction));
}

static void testStagedPromotionAndHighHysteresisRemain(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config, config.low_bank_rad, config.low_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_LOW);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, config.medium_bank_rad, config.medium_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_MEDIUM);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config, config.high_bank_rad, config.high_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_HIGH);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      config.very_high_bank_rad, config.very_high_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      config.maximum_bank_rad, config.maximum_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_MAXIMUM);

  const float very_high_maximum_boundary =
      0.5f * (config.very_high_bank_rad + config.maximum_bank_rad);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      very_high_maximum_boundary - config.transition_hysteresis_rad + 0.001f,
      very_high_maximum_boundary - config.transition_hysteresis_rad + 0.001f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_MAXIMUM);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      very_high_maximum_boundary - config.transition_hysteresis_rad - 0.001f,
      very_high_maximum_boundary - config.transition_hysteresis_rad - 0.001f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_VERY_HIGH);

  const float medium_high_boundary =
      0.5f * (config.medium_bank_rad + config.high_bank_rad);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      medium_high_boundary - config.transition_hysteresis_rad + 0.001f,
      medium_high_boundary - config.transition_hysteresis_rad + 0.001f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_HIGH);
  selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      medium_high_boundary - config.transition_hysteresis_rad - 0.001f,
      medium_high_boundary - config.transition_hysteresis_rad - 0.001f);
  assert(selection.active_model == TINYMPC_BANK_MODEL_RIGHT_MEDIUM);
  assert(selector.switch_count == 8u);
}

static void testSteadyTurnEntryRequiresLocalChartError(void) {
  float error[12] = {0.0f};
  assert(tinyMpcBankEntrySupportedByChartError(
      error, 0.10f, 0.40f, 0.15f, 0.40f));
  error[1] = 0.1001f;
  assert(!tinyMpcBankEntrySupportedByChartError(
      error, 0.10f, 0.40f, 0.15f, 0.40f));
  error[1] = 0.0f;
  error[7] = -0.401f;
  assert(!tinyMpcBankEntrySupportedByChartError(
      error, 0.10f, 0.40f, 0.15f, 0.40f));
  error[7] = 0.0f;
  error[5] = -0.151f;
  assert(!tinyMpcBankEntrySupportedByChartError(
      error, 0.10f, 0.40f, 0.15f, 0.40f));
  error[5] = 0.0f;
  error[11] = NAN;
  assert(!tinyMpcBankEntrySupportedByChartError(
      error, 0.10f, 0.40f, 0.15f, 0.40f));
}

static void testSmoothRampIsDeterministic(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(2u);
  TinyMpcBankSelector first = initializedSelector(&config);
  TinyMpcBankSelector second = initializedSelector(&config);
  for (int step = 0; step <= 80; ++step) {
    const float reference = -0.60f + 0.015f * (float)step;
    const float measured = reference * 0.85f;
    const TinyMpcBankSelection a = tinyMpcBankSelectorUpdate(
        &first, &config, reference, measured);
    const TinyMpcBankSelection b = tinyMpcBankSelectorUpdate(
        &second, &config, reference, measured);
    assert(a.active_model == b.active_model);
    assert(a.switched == b.switched);
    assert(a.reset_optimizer == b.reset_optimizer);
    assert(first.steps_since_switch == second.steps_since_switch);
    assert(first.switch_count == second.switch_count);
  }
  assert(first.active_model == TINYMPC_BANK_MODEL_RIGHT_HIGH);
  assert(first.switch_count == 7u);
}

static void testTerminalReferenceReleasesMeasuredBank(void) {
  assert(tinyMpcBankMeasuredDemandForReference(0.0f, -0.30f, 0.045f)
      == 0.0f);
  assert(tinyMpcBankMeasuredDemandForReference(0.045f, 0.30f, 0.045f)
      == 0.0f);
  assert(tinyMpcBankMeasuredDemandForReference(0.046f, 0.30f, 0.045f)
      == 0.30f);
  assert(tinyMpcBankMeasuredDemandForReference(NAN, 0.30f, 0.045f)
      == 0.0f);
}

static void testTerminalRampCannotSwitchToOppositeBank(void) {
  const TinyMpcBankSelectorConfig config = configWithDwell(0u);
  TinyMpcBankSelector selector = initializedSelector(&config);
  TinyMpcBankSelection selection = tinyMpcBankSelectorUpdate(
      &selector, &config,
      -config.maximum_bank_rad, -config.maximum_bank_rad);
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEFT_MAXIMUM);
  uint8_t previous_tier = tinyMpcBankModelTier(selection.active_model);
  const float references[] = {
      -config.very_high_bank_rad, -config.high_bank_rad,
      -config.medium_bank_rad, -config.low_bank_rad, 0.0f};
  for (unsigned int index = 0u;
       index < sizeof(references) / sizeof(references[0]); ++index) {
    const float measured_for_selector = tinyMpcBankMeasuredDemandForReference(
        references[index], config.maximum_bank_rad, config.exit_bank_rad);
    selection = tinyMpcBankSelectorUpdate(
        &selector, &config, references[index], measured_for_selector);
    const uint8_t tier = tinyMpcBankModelTier(selection.active_model);
    assert(tinyMpcBankModelSide(selection.active_model) <= 0);
    assert(tier <= previous_tier);
    previous_tier = tier;
  }
  assert(selection.active_model == TINYMPC_BANK_MODEL_LEVEL);
}

int main(void) {
  testStraightDemandStaysLevel();
  testSignedLeftAndRightSelection();
  testMagnitudeSelectsEverySignedTier();
  testThresholdsAndHysteresisUseReferenceAndState();
  testMinimumDwellDelaysOppositeSwitch();
  testMeasuredDivergenceCannotPromoteReferencedTier();
  testLevelCrossingRecoveryCannotPromoteOrFlipChart();
  testResetRestoresDeterministicLevelState();
  testInvalidAndNonfiniteDemandCannotSelectSide();
  testInvalidConfigFailsClosed();
  testSteadyTurnEntryRequiresMeasuredMotionSupport();
  testDirectTierEntryRequiresNominalSpeedLocality();
  testOnlyHighPromotionUsesTighterSpeedLocality();
  testStagedPromotionAndHighHysteresisRemain();
  testSteadyTurnEntryRequiresLocalChartError();
  testSmoothRampIsDeterministic();
  testTerminalReferenceReleasesMeasuredBank();
  testTerminalRampCannotSwitchToOppositeBank();
  puts("tinympc bank selector tests passed");
  return 0;
}
