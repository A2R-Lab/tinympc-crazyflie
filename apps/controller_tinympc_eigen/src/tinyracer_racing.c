#include "tinyracer_racing.h"

#include <math.h>
#include <string.h>

static const float body_bearing_rad[TINYRACER_CLEARANCE_SECTORS] = {
  0.6981317f, 0.2327106f, -0.2327106f, -0.6981317f
};

static float moveToward(float value, float target, float amount) {
  return value < target ? fminf(value + amount, target)
                        : fmaxf(value - amount, target);
}

static float confidenceWeight(float score) {
  return 1.0f / (1.0f + expf(-fminf(fmaxf(score, -10.0f), 10.0f)));
}

void tinyRacerRaceReset(TinyRacerRaceState *state) {
  memset(state, 0, sizeof(*state));
}

void tinyRacerRaceUpdate(
    TinyRacerRaceState *state,
    const TinyRacerPerceptionObservation *observation,
    const TinyRacerRaceConfig *config,
    float dt_s,
    bool release_ready,
    bool hold_offset,
    bool allow_activation,
    uint8_t relevant_sector_mask,
    TinyRacerRaceIntent *intent) {
  memset(intent, 0, sizeof(*intent));
  const bool was_active = state->obstacle_constraint_active;
  bool sector_was_active[TINYRACER_CLEARANCE_SECTORS];
  memcpy(sector_was_active, state->sector_active, sizeof(sector_was_active));
  const bool fresh = observation->valid &&
      observation->received_age_ms <= config->maximum_age_ms;
  const bool new_sample = fresh &&
      (!state->has_sample || observation->sample != state->last_sample);

  if (new_sample) {
    int dangerous_sector_count = 0;
    float dangerous_confidence_sum = 0.0f;
    bool path_hard_blocked = false;
    bool all_safe = true;
    bool dangerous[TINYRACER_CLEARANCE_SECTORS];
    for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
      const bool confident =
          observation->confidence[sector] >= config->confidence_threshold;
      dangerous[sector] = (relevant_sector_mask & (1u << sector)) &&
          confident && observation->clearance_m[sector] <
              config->clearance_threshold_m;
      dangerous_sector_count += dangerous[sector] ? 1 : 0;
      dangerous_confidence_sum += dangerous[sector]
          ? confidenceWeight(observation->confidence[sector]) : 0.0f;
      path_hard_blocked |= dangerous[sector] &&
          observation->clearance_m[sector] <=
              config->hard_clearance_threshold_m;
      all_safe &= confident && observation->clearance_m[sector] >=
          config->release_clearance_m;
    }
    const bool path_blocked = dangerous_sector_count >= 1 || path_hard_blocked;
    const float mean_confidence = dangerous_sector_count > 0
        ? dangerous_confidence_sum / dangerous_sector_count : 0.0f;
    const uint8_t evidence_required = config->blocked_samples_required +
        (uint8_t)lroundf(4.0f * (1.0f - mean_confidence));
    state->blocked_samples = !was_active && allow_activation && path_blocked
        ? (state->blocked_samples < UINT8_MAX ? state->blocked_samples + 1
                                               : state->blocked_samples)
        : 0;
    if (!was_active && allow_activation &&
        state->blocked_samples >= evidence_required) {
      state->obstacle_constraint_active = true;
      state->blocked_samples = 0;
      state->pass_side = 0;
      state->clear_samples = 0;
      memcpy(state->sector_active, dangerous, sizeof(state->sector_active));
    } else {
      state->clear_samples = all_safe
          ? (state->clear_samples < UINT8_MAX ? state->clear_samples + 1
                                               : state->clear_samples)
          : 0;
    }
    const bool false_alarm = was_active &&
        state->clear_samples >= 2 * config->clear_samples_required &&
        fabsf(state->lateral_offset_m) < 0.10f;
    if (false_alarm || (was_active &&
        state->clear_samples >= config->clear_samples_required &&
        release_ready && fabsf(state->lateral_offset_m) >= config->bypass_offset_m)) {
      state->obstacle_constraint_active = false;
      memset(state->sector_active, 0, sizeof(state->sector_active));
    } else if (was_active) {
      memcpy(state->sector_active, dangerous, sizeof(state->sector_active));
    }
    state->has_sample = true;
    state->last_sample = observation->sample;
  }

  const float target_offset = state->obstacle_constraint_active
      ? (state->clear_samples == 0
          ? state->pass_side * config->maximum_bypass_offset_m
          : (fabsf(state->lateral_offset_m) < config->bypass_offset_m
              ? state->pass_side * config->bypass_offset_m
              : state->lateral_offset_m))
      : (hold_offset ? state->lateral_offset_m : 0.0f);
  const float offset_rate = state->obstacle_constraint_active
      ? config->bypass_rate_mps : config->recovery_rate_mps;
  state->lateral_offset_m = moveToward(
      state->lateral_offset_m, target_offset, offset_rate * dt_s);

  intent->mode = state->obstacle_constraint_active
      ? TINYRACER_RACE_BLOCKED
      : (fabsf(state->lateral_offset_m) > 0.001f
          ? TINYRACER_RACE_RECOVER : TINYRACER_RACE_TRACK);
  intent->perception_fresh = fresh;
  intent->gate_valid = fresh && observation->gate_valid;
  intent->constraint_active = state->obstacle_constraint_active;
  intent->constraint_changed = was_active != state->obstacle_constraint_active;
  intent->pass_side = state->pass_side;
  intent->lateral_offset_m = state->lateral_offset_m;
  intent->stop_boundary_distance_m = config->clearance_threshold_m;
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    intent->sector[sector].active = state->sector_active[sector];
    intent->sector[sector].changed =
        sector_was_active[sector] != state->sector_active[sector];
    intent->constraint_changed |= intent->sector[sector].changed;
    intent->sector[sector].body_bearing_rad = body_bearing_rad[sector];
    intent->sector[sector].boundary_distance_m = fmaxf(
        observation->clearance_m[sector] - config->safety_margin_m,
        config->minimum_boundary_distance_m);
    if (intent->sector[sector].active &&
        intent->sector[sector].boundary_distance_m <
        intent->stop_boundary_distance_m) {
      intent->stop_boundary_distance_m =
          intent->sector[sector].boundary_distance_m;
    }
  }
}
