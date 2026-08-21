#include "tinyracer_racing.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static void setAllClearances(
    TinyRacerPerceptionObservation *observation, float clearance_m) {
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    observation->clearance_m[sector] = clearance_m;
  }
}

int main(void) {
  const TinyRacerRaceConfig config = {
    0.25f, 0.0f, 0.10f, 0.05f, 0.35f, 0.35f, 0.25f, 250,
    0.25f, 0.70f, 3, 0.20f, 2, 0.25f, 2
  };
  TinyRacerRaceState state;
  TinyRacerRaceIntent intent;
  TinyRacerPerceptionObservation observation;
  memset(&observation, 0, sizeof(observation));
  tinyRacerRaceReset(&state);

  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(!intent.perception_fresh && !intent.constraint_active);

  observation.valid = true;
  observation.has_metric_clearance = true;
  observation.gate_valid = true;
  observation.sample = 1;
  setAllClearances(&observation, 0.30f);
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    observation.confidence[sector] = 6.0f;
  }
  observation.clearance_m[2] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.perception_fresh && intent.gate_valid);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.constraint_active && intent.constraint_changed);
  assert(intent.pass_side == 0);
  assert(intent.sector[2].boundary_distance_m > 0.099f &&
         intent.sector[2].boundary_distance_m < 0.101f);
  assert(intent.lateral_offset_m == 0.0f);
  state.pass_side = 1;  // The controller selects this from trajectory geometry.

  observation.clearance_m[2] = 0.40f;
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(intent.constraint_active && intent.constraint_changed);
  assert(!intent.sector[2].active && intent.sector[2].changed);

  setAllClearances(&observation, 0.40f);
  observation.received_age_ms = 251;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0f, &intent);
  assert(!intent.perception_fresh && intent.constraint_active);

  observation.received_age_ms = 0;
  observation.sample++;
  state.lateral_offset_m = config.bypass_offset_m;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, true, false, true, 0x0f, &intent);
  assert(intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, true, false, true, 0x0f, &intent);
  assert(!intent.constraint_active && intent.constraint_changed);
  assert(intent.mode == TINYRACER_RACE_RECOVER);

  const float held_offset = state.lateral_offset_m;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, true, false, 0x0f, &intent);
  assert(state.lateral_offset_m == held_offset);

  observation.sample++;
  setAllClearances(&observation, 0.20f);
  observation.clearance_m[2] = 0.24f;
  observation.clearance_m[3] = 0.24f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, true, false, 0x0f, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0c, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 0x0c, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.40f);
  observation.clearance_m[1] = 0.24f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.20f);
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);
  observation.sample++;
  setAllClearances(&observation, 0.30f);
  observation.clearance_m[0] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active && intent.pass_side == 0);

  tinyRacerRaceReset(&state);
  observation.sample++;
  observation.clearance_m[1] = 0.20f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(!intent.constraint_active);
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 1, &intent);
  assert(intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.sample++;
  setAllClearances(&observation, 0.20f);
  observation.confidence[2] = -0.1f;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(!intent.constraint_active);

  tinyRacerRaceReset(&state);
  observation.confidence[2] = 0.0f;
  for (int sample = 0; sample < 3; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
    assert(!intent.constraint_active);
  }
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false, true, 1u << 2, &intent);
  assert(intent.constraint_active);

  /* A danger-only head must provide two spatial sectors; a DroNet-style
   * non-spatial collision score never sets has_sector_danger. */
  tinyRacerRaceReset(&state);
  memset(&observation, 0, sizeof(observation));
  observation.valid = true;
  observation.has_sector_danger = true;
  observation.danger_probability[1] = 0.60f;
  for (int sample = 0; sample < 6; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                        true, 0x06, &intent);
  }
  assert(!intent.constraint_active);
  observation.danger_probability[2] = 0.60f;
  for (int sample = 0; sample < 3; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                        true, 0x06, &intent);
    assert(!intent.constraint_active);
  }
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                      true, 0x06, &intent);
  assert(intent.constraint_active);

  /* Irrelevant outer sectors must not inflate the mean confidence and wrap
   * the evidence count when all four network regions report danger. */
  tinyRacerRaceReset(&state);
  memset(&intent, 0, sizeof(intent));
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    observation.danger_probability[sector] = 1.0f;
  }
  for (int sample = 0; sample < config.blocked_samples_required; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                        true, 0x06, &intent);
  }
  assert(intent.constraint_active);

  /* Match the official PULP-DroNet navigation law: target forward speed is
   * vmax*(1-collision), steering is scaled to yaw rate, and both outputs use
   * the published alpha=0.3 previous-output low-pass convention. */
  TinyRacerNavigationState navigation_state;
  TinyRacerNavigationIntent navigation_intent;
  const TinyRacerNavigationConfig navigation_config = {
    250, 0.50f, 2.09439510239f, 0.30f
  };
  memset(&observation, 0, sizeof(observation));
  observation.valid = true;
  observation.has_navigation_command = true;
  observation.sample = 1;
  observation.steering_command = 0.5f;
  observation.collision_probability = 0.2f;
  tinyRacerNavigationReset(&navigation_state);
  tinyRacerNavigationUpdate(
      &navigation_state, &observation, &navigation_config,
      0.02f, 0.4f, &navigation_intent);
  assert(navigation_intent.active && navigation_intent.new_sample);
  assert(fabsf(navigation_intent.forward_speed_mps - 0.28f) < 1.0e-6f);
  assert(fabsf(navigation_intent.yaw_rate_rad_s - 0.7330383f) < 1.0e-5f);
  assert(fabsf(navigation_intent.heading_world_rad - 0.4146608f) < 1.0e-5f);
  tinyRacerNavigationUpdate(
      &navigation_state, &observation, &navigation_config,
      0.02f, 0.4f, &navigation_intent);
  assert(navigation_intent.active && !navigation_intent.new_sample);
  assert(fabsf(navigation_intent.heading_world_rad - 0.4293215f) < 1.0e-5f);
  observation.received_age_ms = 251;
  tinyRacerNavigationUpdate(
      &navigation_state, &observation, &navigation_config,
      0.02f, 0.4f, &navigation_intent);
  assert(!navigation_intent.active);

  /* The racing enhancement latches DroNet's pass direction, performs a
   * lateral sidestep, holds that lane until the obstacle is passed, and only
   * then rejoins. It must not integrate DroNet yaw into the aircraft heading. */
  TinyRacerDodgeState dodge_state;
  TinyRacerDodgeIntent dodge_intent;
  const TinyRacerDodgeConfig dodge_config = {
    0.25f, 0.18f, 0.10f, 0.50f, 0.25f,
    0.12f, 0.45f, 0.35f, 0.08f, 0.20f, true, 0, 2, 2
  };
  tinyRacerDodgeReset(&dodge_state);
  memset(&navigation_intent, 0, sizeof(navigation_intent));
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.collision_probability = 0.8f;
  navigation_intent.yaw_rate_rad_s = -0.4f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_SIDESTEP);
  assert(dodge_intent.pass_side == -1);
  assert(fabsf(dodge_intent.lateral_offset_m + 0.05f) < 1.0e-6f);
  navigation_intent.new_sample = false;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_PASS);
  assert(fabsf(dodge_intent.lateral_offset_m + 0.10f) < 1.0e-6f);
  assert(fabsf(dodge_state.forward_distance_m) < 1.0e-6f);
  navigation_intent.new_sample = true;
  navigation_intent.collision_probability = 0.1f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_PASS);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_REJOIN);
  while (dodge_intent.phase == TINYRACER_DODGE_REJOIN) {
    navigation_intent.new_sample = false;
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
        &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  assert(fabsf(dodge_intent.lateral_offset_m) < 1.0e-6f);
  assert(!dodge_state.armed);
  navigation_intent.collision_probability = 0.8f;
  navigation_intent.new_sample = true;
  for (int sample = 0; sample < 3; ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
        &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  }
  navigation_intent.collision_probability = 0.1f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(!dodge_state.armed);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_state.armed);

  /* A new sustained threat during rejoin is a distinct close-spaced slalom
   * encounter and must be allowed to reverse the selected pass side. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_REJOIN;
  dodge_state.lateral_offset_m = -0.08f;
  navigation_intent.collision_probability = 0.8f;
  navigation_intent.yaw_rate_rad_s = 0.4f;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_REJOIN);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_SIDESTEP);
  assert(dodge_intent.pass_side == 1);

  /* A known curved track may prescribe the outside lane. That geometric
   * choice must dominate a noisy or frame-sensitive navigation steering sign. */
  TinyRacerDodgeConfig curved_dodge_config = dodge_config;
  curved_dodge_config.allow_rejoin_redirect = false;
  curved_dodge_config.preferred_pass_side = -1;
  tinyRacerDodgeReset(&dodge_state);
  navigation_intent.collision_probability = 0.8f;
  navigation_intent.yaw_rate_rad_s = 0.4f;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &curved_dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &curved_dodge_config, 0.1f, 0.45f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_SIDESTEP);
  assert(dodge_intent.pass_side == -1);

  /* A combined navigation+gate head must not let gate centering fight an
   * active avoidance reference. */
  assert(tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_TRACK, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_SIDESTEP, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_PASS, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REJOIN, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_BLOCKED, TINYRACER_DODGE_TRACK, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_TRACK, true, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_TRACK, false, true));

  /* Gate completion is a physical, ordered plane crossing inside the usable
   * opening. Merely seeing a gate or crossing its infinite plane off-center
   * must not arm the next course event. */
  const TinyRacerGateDefinition gates[] = {
    {1.0f, 0.0f, 1.5f, 1.0f, 0.0f, 0.125f, 0.125f},
    {2.0f, 1.0f, 1.5f, 0.0f, 1.0f, 0.125f, 0.125f},
  };
  TinyRacerGateProgress gate_progress;
  tinyRacerGateProgressReset(&gate_progress);
  assert(!tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 0.8f, 0.0f, 1.5f));
  assert(!tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 1.1f, 0.20f, 1.5f));
  assert(gate_progress.next_gate == 0);
  assert(!tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 0.9f, 0.02f, 1.5f));
  assert(tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 1.1f, 0.02f, 1.5f));
  assert(gate_progress.next_gate == 1);
  assert(!tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 2.0f, 0.8f, 1.5f));
  assert(tinyRacerGateProgressUpdate(
      &gate_progress, gates, 2, 2.02f, 1.1f, 1.52f));
  assert(gate_progress.next_gate == 2);
  return 0;
}
