#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "tinyracer_reactive_escape.h"

static void assertClose(float actual, float expected) {
  assert(fabsf(actual - expected) < 1.0e-6f);
}

static void testCruiseUntilCenterTrigger(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  const TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.5f);
  assert(fabsf(config.turn_rate_deg_s * 0.0174532925199433f - 0.50f)
      < 1.0e-6f);

  TinyRacerReactiveCommand command = tinyRacerReactiveStep(
      &state, &config, 0.2f, 0.849f, 0.3f, false);
  assert(command.phase == TINYRACER_REACTIVE_CRUISE);
  assertClose(command.forward_speed_mps, 1.5f);
  assertClose(command.lateral_speed_mps, 0.0f);
  assertClose(command.yaw_rate_deg_s, 0.0f);

  command = tinyRacerReactiveStep(
      &state, &config, 0.1f, 0.85f, 0.8f, false);
  assert(command.changed);
  assert(command.phase == TINYRACER_REACTIVE_BRAKE);
  assert(command.turn_direction == 1);
  assertClose(command.forward_speed_mps, 0.0f);
  assertClose(command.yaw_rate_deg_s, 0.0f);
}

static void testDirectionLatchesWhileBraking(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  const TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);

  (void)tinyRacerReactiveStep(&state, &config, 0.8f, 0.9f, 0.1f, false);
  assert(state.turn_direction == -1);
  TinyRacerReactiveCommand command;
  for (int sample = 0; sample < config.settled_samples_required; ++sample) {
    command = tinyRacerReactiveStep(
        &state, &config, 0.0f, 0.8f, 1.0f, true);
  }
  assert(command.phase == TINYRACER_REACTIVE_TURN_SCAN);
  assert(command.turn_direction == -1);
  assertClose(command.yaw_rate_deg_s, -config.turn_rate_deg_s);
}

static void testHysteresisAndClearConfirmation(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);
  config.minimum_maneuver_samples = 1u;
  (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.9f, 0.8f, false);
  for (int sample = 0; sample < config.settled_samples_required; ++sample) {
    (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.9f, 0.8f, true);
  }

  for (int sample = 0; sample < 2; ++sample) {
    const TinyRacerReactiveCommand command = tinyRacerReactiveStep(
        &state, &config, 0.1f, 0.54f, 0.8f, true);
    assert(command.phase == TINYRACER_REACTIVE_TURN_SCAN);
  }
  (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.60f, 0.8f, true);
  assert(state.clear_samples == 0u);
  for (int sample = 0; sample < 2; ++sample) {
    (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.54f, 0.8f, true);
  }
  const TinyRacerReactiveCommand yaw_stopped = tinyRacerReactiveStep(
      &state, &config, 0.1f, 0.54f, 0.8f, true);
  assert(yaw_stopped.changed);
  assert(yaw_stopped.phase == TINYRACER_REACTIVE_TURN_SCAN);
  assertClose(yaw_stopped.yaw_rate_deg_s, 0.0f);
  TinyRacerReactiveCommand released;
  for (int sample = 0; sample < config.settled_samples_required; ++sample) {
    released = tinyRacerReactiveStep(
        &state, &config, 0.9f, 0.95f, 0.9f, true);
  }
  assert(released.changed);
  assert(released.phase == TINYRACER_REACTIVE_CRUISE);
  assertClose(released.forward_speed_mps, 1.0f);
}

static void testAmbiguousEncountersAlternate(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);
  config.settled_samples_required = 1u;
  config.minimum_maneuver_samples = 1u;
  config.clear_samples_required = 1u;

  TinyRacerReactiveCommand command = tinyRacerReactiveStep(
      &state, &config, 0.4f, 0.9f, 0.42f, false);
  assert(command.turn_direction == 1);
  (void)tinyRacerReactiveStep(&state, &config, 0.4f, 0.1f, 0.42f, true);
  (void)tinyRacerReactiveStep(&state, &config, 0.4f, 0.1f, 0.42f, true);
  (void)tinyRacerReactiveStep(&state, &config, 0.4f, 0.1f, 0.42f, true);
  command = tinyRacerReactiveStep(
      &state, &config, 0.4f, 0.9f, 0.42f, false);
  assert(command.turn_direction == -1);
}

static void testMinimumTurnBeforeRelease(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);
  config.settled_samples_required = 1u;
  config.minimum_maneuver_samples = 4u;
  config.clear_samples_required = 1u;

  (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.9f, 0.8f, false);
  (void)tinyRacerReactiveStep(&state, &config, 0.1f, 0.2f, 0.8f, true);
  for (uint8_t sample = 0u;
       sample < config.minimum_maneuver_samples - 1u; ++sample) {
    const TinyRacerReactiveCommand command = tinyRacerReactiveStep(
        &state, &config, 0.1f, 0.2f, 0.8f, true);
    assert(command.phase == TINYRACER_REACTIVE_TURN_SCAN);
  }
  const TinyRacerReactiveCommand yaw_stopped = tinyRacerReactiveStep(
      &state, &config, 0.1f, 0.2f, 0.8f, true);
  assert(yaw_stopped.changed);
  assert(yaw_stopped.phase == TINYRACER_REACTIVE_TURN_SCAN);
  assertClose(yaw_stopped.yaw_rate_deg_s, 0.0f);
  const TinyRacerReactiveCommand released = tinyRacerReactiveStep(
      &state, &config, 0.1f, 0.2f, 0.8f, true);
  assert(released.changed);
  assert(released.phase == TINYRACER_REACTIVE_CRUISE);
}

static void testAllDangerousScansTowardLeastRisk(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  const TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);

  TinyRacerReactiveCommand command = tinyRacerReactiveStep(
      &state, &config, 0.92f, 0.95f, 0.84f, false);
  assert(command.phase == TINYRACER_REACTIVE_BRAKE);
  assert(command.turn_direction == -1);
  assertClose(command.forward_speed_mps, 0.0f);
  assertClose(command.yaw_rate_deg_s, 0.0f);
  for (int sample = 0; sample < config.settled_samples_required; ++sample) {
    command = tinyRacerReactiveStep(
        &state, &config, 0.92f, 0.95f, 0.84f, true);
  }
  assert(command.phase == TINYRACER_REACTIVE_TURN_SCAN);
  assertClose(command.yaw_rate_deg_s, -config.turn_rate_deg_s);
}

static void testConfirmedRailCueAimsTowardOpening(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);

  /* Ordinary risk would turn left (+1). One stopped pass-right cue is not
   * enough to override it. The second consecutive cue latches body-right. */
  TinyRacerReactiveCommand command = tinyRacerReactiveStepWithRail(
      &state, &config, 0.1f, 0.90f, 0.8f, false, 0, false);
  assert(command.turn_direction == 1);
  command = tinyRacerReactiveStepWithRail(
      &state, &config, 0.1f, 0.90f, 0.8f, true, -1, true);
  assert(command.turn_direction == 1);
  assert(!state.rail_direction_latched);
  command = tinyRacerReactiveStepWithRail(
      &state, &config, 0.1f, 0.90f, 0.8f, true, -1, true);
  assert(command.turn_direction == -1);
  assert(state.rail_direction_latched);
  for (int sample = 2; sample < config.settled_samples_required; ++sample) {
    command = tinyRacerReactiveStepWithRail(
        &state, &config, 0.1f, 0.90f, 0.8f, false, 0, true);
  }
  assert(command.phase == TINYRACER_REACTIVE_TRANSLATE_OPENING);
  assertClose(command.lateral_speed_mps, -config.translation_speed_mps);
  assertClose(command.yaw_rate_deg_s, 0.0f);

  /* The cue may disappear after it is latched. Continue right until both the
   * minimum sidestep and center-clear confirmation have completed. */
  for (uint8_t sample = 0u;
       sample < config.minimum_maneuver_samples - config.clear_samples_required;
       ++sample) {
    command = tinyRacerReactiveStepWithRail(
        &state, &config, 0.1f, 0.90f, 0.8f, false, 0, false);
    assertClose(command.lateral_speed_mps, -config.translation_speed_mps);
  }
  for (uint8_t sample = 0u; sample < config.clear_samples_required; ++sample) {
    command = tinyRacerReactiveStepWithRail(
        &state, &config, 0.1f, 0.54f, 0.8f, false, 0, false);
  }
  assert(command.changed);
  assert(command.phase == TINYRACER_REACTIVE_TRANSLATE_OPENING);
  assertClose(command.lateral_speed_mps, 0.0f);
  for (uint8_t sample = 0u; sample < config.settled_samples_required;
       ++sample) {
    command = tinyRacerReactiveStepWithRail(
        &state, &config, 0.1f, 0.54f, 0.8f, false, 0, true);
  }
  assert(command.phase == TINYRACER_REACTIVE_CRUISE);
  assertClose(command.forward_speed_mps, config.cruise_speed_mps);
}

static void testUnconfirmedRailCuePreservesRiskFallback(void) {
  TinyRacerReactiveState state;
  tinyRacerReactiveReset(&state);
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);
  (void)tinyRacerReactiveStepWithRail(
      &state, &config, 0.8f, 0.90f, 0.1f, false, 0, false);
  for (int sample = 0; sample < config.settled_samples_required; ++sample) {
    (void)tinyRacerReactiveStepWithRail(
        &state, &config, 0.8f, 0.90f, 0.1f,
        sample == 0, 1, true);
  }
  assert(state.phase == TINYRACER_REACTIVE_TURN_SCAN);
  assert(state.turn_direction == -1);
  assert(!state.rail_direction_latched);
}

static void testTranslationSpeedMustBePositiveAndFinite(void) {
  TinyRacerReactiveConfig config = tinyRacerReactiveDefaultConfig(1.0f);
  assert(tinyRacerReactiveConfigValid(&config));
  config.translation_speed_mps = 0.0f;
  assert(!tinyRacerReactiveConfigValid(&config));
  config.translation_speed_mps = NAN;
  assert(!tinyRacerReactiveConfigValid(&config));
}

int main(void) {
  testCruiseUntilCenterTrigger();
  testDirectionLatchesWhileBraking();
  testHysteresisAndClearConfirmation();
  testAmbiguousEncountersAlternate();
  testAllDangerousScansTowardLeastRisk();
  testConfirmedRailCueAimsTowardOpening();
  testUnconfirmedRailCuePreservesRiskFallback();
  testMinimumTurnBeforeRelease();
  testTranslationSpeedMustBePositiveAndFinite();
  puts("tinyracer reactive escape tests passed");
  return 0;
}
