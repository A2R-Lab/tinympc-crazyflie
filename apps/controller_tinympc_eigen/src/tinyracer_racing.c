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

void tinyRacerNavigationReset(TinyRacerNavigationState *state) {
  memset(state, 0, sizeof(*state));
}

void tinyRacerNavigationUpdate(
    TinyRacerNavigationState *state,
    const TinyRacerPerceptionObservation *observation,
    const TinyRacerNavigationConfig *config,
    float dt_s,
    float measured_heading_world_rad,
    TinyRacerNavigationIntent *intent) {
  memset(intent, 0, sizeof(*intent));
  const bool fresh = observation->valid &&
      observation->has_navigation_command &&
      observation->received_age_ms <= config->maximum_age_ms;
  if (!fresh) {
    return;
  }
  if (!state->initialized) {
    state->initialized = true;
    state->last_sample = UINT32_MAX;
    state->heading_world_rad = measured_heading_world_rad;
  }
  if (observation->sample != state->last_sample) {
    const float old_weight = fminf(fmaxf(
        config->previous_output_weight, 0.0f), 1.0f);
    const float new_weight = 1.0f - old_weight;
    const float collision = fminf(fmaxf(
        observation->collision_probability, 0.0f), 1.0f);
    const float steering = fminf(fmaxf(
        observation->steering_command, -1.0f), 1.0f);
    const float target_speed =
        config->maximum_forward_speed_mps * (1.0f - collision);
    const float target_yaw_rate =
        config->yaw_rate_scale_rad_s * steering;
    state->forward_speed_mps = old_weight * state->forward_speed_mps +
        new_weight * target_speed;
    state->yaw_rate_rad_s = old_weight * state->yaw_rate_rad_s +
        new_weight * target_yaw_rate;
    state->last_sample = observation->sample;
    intent->new_sample = true;
  }
  state->heading_world_rad = remainderf(
      state->heading_world_rad + state->yaw_rate_rad_s * dt_s,
      6.28318530717958647692f);
  intent->active = true;
  intent->forward_speed_mps = state->forward_speed_mps;
  intent->yaw_rate_rad_s = state->yaw_rate_rad_s;
  intent->heading_world_rad = state->heading_world_rad;
  intent->collision_probability = fminf(fmaxf(
      observation->collision_probability, 0.0f), 1.0f);
}

void tinyRacerDodgeReset(TinyRacerDodgeState *state) {
  memset(state, 0, sizeof(*state));
  state->phase = TINYRACER_DODGE_TRACK;
  state->pass_side = 1;
  state->armed = true;
}

static int8_t selectedPassSide(
    const TinyRacerNavigationIntent *navigation,
    const TinyRacerDodgeConfig *config) {
  if (config->preferred_pass_side < 0) {
    return -1;
  }
  if (config->preferred_pass_side > 0) {
    return 1;
  }
  return navigation->yaw_rate_rad_s < 0.0f ? -1 : 1;
}

bool tinyRacerGateServoAllowed(
    TinyRacerRaceMode race_mode,
    TinyRacerDodgePhase dodge_phase,
    bool halfspace_active,
    bool recovery_active) {
  return race_mode == TINYRACER_RACE_TRACK &&
      dodge_phase == TINYRACER_DODGE_TRACK &&
      !halfspace_active && !recovery_active;
}

void tinyRacerGateProgressReset(TinyRacerGateProgress *progress) {
  memset(progress, 0, sizeof(*progress));
}

bool tinyRacerGateProgressUpdate(
    TinyRacerGateProgress *progress,
    const TinyRacerGateDefinition *gates,
    uint8_t gate_count,
    float position_x_m,
    float position_y_m,
    float position_z_m) {
  if (progress->next_gate >= gate_count) {
    return false;
  }
  const TinyRacerGateDefinition *gate = &gates[progress->next_gate];
  const float dx = position_x_m - gate->center_x_m;
  const float dy = position_y_m - gate->center_y_m;
  const float signed_distance = gate->normal_x * dx + gate->normal_y * dy;
  if (!progress->plane_initialized) {
    progress->plane_initialized = true;
    progress->previous_signed_distance_m = signed_distance;
    return false;
  }
  const bool crossed_plane = progress->previous_signed_distance_m <= 0.0f &&
      signed_distance > 0.0f;
  progress->previous_signed_distance_m = signed_distance;
  if (!crossed_plane) {
    return false;
  }
  const float lateral_error = -gate->normal_y * dx + gate->normal_x * dy;
  const float vertical_error = position_z_m - gate->center_z_m;
  if (fabsf(lateral_error) > gate->maximum_lateral_error_m ||
      fabsf(vertical_error) > gate->maximum_vertical_error_m) {
    return false;
  }
  ++progress->next_gate;
  progress->plane_initialized = false;
  return true;
}

void tinyRacerDodgeUpdate(
    TinyRacerDodgeState *state,
    const TinyRacerNavigationIntent *navigation,
    const TinyRacerDodgeConfig *config,
    float dt_s,
    float measured_forward_speed_mps,
    TinyRacerDodgeIntent *intent) {
  const float dt = fmaxf(dt_s, 0.0f);
  const bool new_sample = navigation->active && navigation->new_sample;
  if (new_sample) {
    if (navigation->collision_probability >= config->trigger_probability) {
      if (state->trigger_samples < UINT8_MAX) {
        ++state->trigger_samples;
      }
    } else {
      state->trigger_samples = 0;
    }
    if (navigation->collision_probability <= config->release_probability) {
      if (state->clear_samples < UINT8_MAX) {
        ++state->clear_samples;
      }
    } else {
      state->clear_samples = 0;
    }
  }

  if (!navigation->active && state->phase == TINYRACER_DODGE_TRACK) {
    memset(intent, 0, sizeof(*intent));
    intent->phase = state->phase;
    intent->pass_side = state->pass_side;
    return;
  }

  if (state->phase == TINYRACER_DODGE_TRACK && !state->armed) {
    state->rearm_distance_m +=
        fmaxf(measured_forward_speed_mps, 0.0f) * dt;
    if (state->rearm_distance_m >= config->minimum_rearm_distance_m &&
        state->clear_samples >= config->clear_samples_required) {
      state->armed = true;
      state->trigger_samples = 0;
    }
  }

  if (state->phase == TINYRACER_DODGE_TRACK &&
      state->armed &&
      state->trigger_samples >= config->trigger_samples_required) {
    state->phase = TINYRACER_DODGE_SIDESTEP;
    state->pass_side = selectedPassSide(navigation, config);
    state->forward_distance_m = 0.0f;
    state->clear_samples = 0;
  }

  /* Alternating half-corridor obstacles can overlap in the camera view, so
   * collision probability may never fall below the release threshold between
   * them. Once the first obstacle's full bypass distance is complete, accept
   * a sustained opposite steering sign as the next encounter and cross
   * directly to its pass lane. The distance and opposite-side requirements
   * prevent noise near the first obstacle from reversing a committed bypass. */
  const int8_t redirected_side = selectedPassSide(navigation, config);
  if (config->allow_rejoin_redirect &&
      state->phase == TINYRACER_DODGE_PASS &&
      state->forward_distance_m >= config->minimum_pass_distance_m &&
      state->trigger_samples >= config->trigger_samples_required &&
      redirected_side != state->pass_side) {
    state->phase = TINYRACER_DODGE_SIDESTEP;
    state->pass_side = redirected_side;
    state->forward_distance_m = 0.0f;
    state->trigger_samples = 0;
    state->clear_samples = 0;
  }

  /* A racing slalom can also present the next obstacle before the previous
   * lateral offset has fully decayed. Redirect during REJOIN rather than
   * waiting to reach TRACK. */
  if (config->allow_rejoin_redirect &&
      state->phase == TINYRACER_DODGE_REJOIN &&
      state->trigger_samples >= config->trigger_samples_required) {
    state->phase = TINYRACER_DODGE_SIDESTEP;
    state->pass_side = redirected_side;
    state->forward_distance_m = 0.0f;
    state->trigger_samples = 0;
    state->clear_samples = 0;
  }

  float target_offset = 0.0f;
  float rate = config->rejoin_rate_mps;
  float forward_speed = navigation->active
      ? navigation->forward_speed_mps : config->rejoin_forward_speed_mps;
  switch (state->phase) {
    case TINYRACER_DODGE_SIDESTEP:
      target_offset = (float)state->pass_side * config->lateral_offset_m;
      /* Close slalom obstacles can require crossing from one pass lane to the
       * other. Use a faster banked redirect only while the existing and target
       * offsets have opposite signs; ordinary single-obstacle sidesteps retain
       * the conservative rate. */
      rate = state->lateral_offset_m * target_offset < 0.0f
          ? 2.0f * config->sidestep_rate_mps
          : config->sidestep_rate_mps;
      forward_speed = config->sidestep_forward_speed_mps;
      state->forward_distance_m +=
          fmaxf(measured_forward_speed_mps, 0.0f) * dt;
      break;
    case TINYRACER_DODGE_PASS:
      target_offset = (float)state->pass_side * config->lateral_offset_m;
      rate = config->sidestep_rate_mps;
      forward_speed = config->pass_forward_speed_mps;
      state->forward_distance_m +=
          fmaxf(measured_forward_speed_mps, 0.0f) * dt;
      if (state->forward_distance_m >= config->minimum_pass_distance_m &&
          state->clear_samples >= config->clear_samples_required) {
        state->phase = TINYRACER_DODGE_REJOIN;
        target_offset = 0.0f;
        rate = config->rejoin_rate_mps;
        forward_speed = config->rejoin_forward_speed_mps;
      }
      break;
    case TINYRACER_DODGE_REJOIN:
      target_offset = 0.0f;
      rate = config->rejoin_rate_mps;
      forward_speed = config->rejoin_forward_speed_mps;
      break;
    case TINYRACER_DODGE_TRACK:
    default:
      target_offset = 0.0f;
      rate = config->rejoin_rate_mps;
      break;
  }

  const float previous_offset = state->lateral_offset_m;
  state->lateral_offset_m = moveToward(
      state->lateral_offset_m, target_offset, fmaxf(rate, 0.0f) * dt);
  if (state->phase == TINYRACER_DODGE_SIDESTEP &&
      fabsf(state->lateral_offset_m - target_offset) < 1.0e-4f) {
    state->phase = TINYRACER_DODGE_PASS;
    /* minimum_pass_distance_m describes travel in the established bypass
     * lane. Distance accumulated while building the lateral offset does not
     * prove that the vehicle has passed the obstacle. */
    state->forward_distance_m = 0.0f;
    state->clear_samples = 0;
  } else if (state->phase == TINYRACER_DODGE_REJOIN &&
             fabsf(state->lateral_offset_m) < 1.0e-4f) {
    state->phase = TINYRACER_DODGE_TRACK;
    state->armed = false;
    state->rearm_distance_m = 0.0f;
    state->trigger_samples = 0;
    state->clear_samples = 0;
  }

  memset(intent, 0, sizeof(*intent));
  intent->phase = state->phase;
  intent->pass_side = state->pass_side;
  intent->lateral_offset_m = state->lateral_offset_m;
  intent->lateral_rate_mps = dt > 0.0f
      ? (state->lateral_offset_m - previous_offset) / dt : 0.0f;
  intent->forward_speed_mps = forward_speed;
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
      const bool metric_danger = observation->has_metric_clearance &&
          confident && observation->clearance_m[sector] <
              config->clearance_threshold_m;
      const bool probability_danger = observation->has_sector_danger &&
          observation->danger_probability[sector] >=
              config->danger_probability_threshold;
      dangerous[sector] = (relevant_sector_mask & (1u << sector)) &&
          (metric_danger || probability_danger);
      dangerous_sector_count += dangerous[sector] ? 1 : 0;
      dangerous_confidence_sum += dangerous[sector]
          ? (probability_danger
              ? observation->danger_probability[sector]
              : confidenceWeight(observation->confidence[sector]))
          : 0.0f;
      path_hard_blocked |= metric_danger && dangerous[sector] &&
          observation->clearance_m[sector] <=
              config->hard_clearance_threshold_m;
      const bool sector_safe = observation->has_metric_clearance
          ? confident && observation->clearance_m[sector] >=
                config->release_clearance_m
          : (!observation->has_sector_danger ||
             observation->danger_probability[sector] <
                0.8f * config->danger_probability_threshold);
      all_safe &= sector_safe;
    }
    const int required_sectors = observation->has_metric_clearance
        ? 1 : config->danger_sectors_required;
    const bool path_blocked = dangerous_sector_count >= required_sectors ||
        path_hard_blocked;
    const float mean_confidence = dangerous_sector_count > 0
        ? fminf(fmaxf(dangerous_confidence_sum / dangerous_sector_count,
                      0.0f), 1.0f)
        : 0.0f;
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
        (observation->has_metric_clearance
             ? observation->clearance_m[sector]
             : config->clearance_threshold_m) - config->safety_margin_m,
        config->minimum_boundary_distance_m);
    if (intent->sector[sector].active &&
        intent->sector[sector].boundary_distance_m <
        intent->stop_boundary_distance_m) {
      intent->stop_boundary_distance_m =
          intent->sector[sector].boundary_distance_m;
    }
  }
}
