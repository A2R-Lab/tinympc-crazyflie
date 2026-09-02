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

static float riskScaledAvoidanceRate(
    const TinyRacerDodgeConfig *config, float risk) {
  const float minimum_rate_mps =
      fmaxf(config->minimum_lateral_rate_mps, 0.0f);
  const float maximum_rate_mps = fmaxf(
      config->maximum_lateral_rate_mps, minimum_rate_mps);
  const float trigger = fminf(fmaxf(config->trigger_probability, 0.0f), 1.0f);
  const float bounded_risk = fminf(fmaxf(risk, trigger), 1.0f);
  const float urgency = trigger < 1.0f
      ? (bounded_risk - trigger) / (1.0f - trigger) : 1.0f;
  return minimum_rate_mps +
      urgency * (maximum_rate_mps - minimum_rate_mps);
}

static void startRejoinSpline(
    TinyRacerDodgeState *state,
    const TinyRacerDodgeConfig *config,
    const TinyRacerDodgeFeedback *feedback) {
  if (state->rejoin_spline_active || feedback == NULL || !feedback->valid ||
      config->rejoin_spline_length_m <= 1.0e-6f) {
    return;
  }
  const float return_direction =
      state->phase == TINYRACER_DODGE_REJOIN_LEFT ? -1.0f : 1.0f;
  const float requested_start_slope =
      fmaxf(config->rejoin_lateral_rate_mps, 0.0f) /
      fmaxf(config->rejoin_forward_speed_mps, 1.0e-3f);
  /* A cubic with zero terminal slope stays monotone when the initial slope
   * does not exceed three times the remaining offset over its length. This
   * matters for recovery entries already close to the route center. */
  const float monotone_start_slope_limit =
      3.0f * fabsf(state->lateral_offset_m) /
      config->rejoin_spline_length_m;
  state->rejoin_spline_active = true;
  state->rejoin_spline_start_forward_progress_m =
      feedback->forward_progress_m;
  state->rejoin_spline_start_offset_m = state->lateral_offset_m;
  state->rejoin_spline_start_slope = return_direction * fminf(
      requested_start_slope, monotone_start_slope_limit);
  state->rejoin_spline_progress_m = 0.0f;
  state->rejoin_spline_slope = state->rejoin_spline_start_slope;
}

static float riskScaledAvoidanceOffset(
    const TinyRacerDodgeConfig *config, float risk) {
  const float maximum_offset_m =
      fmaxf(config->maximum_lateral_offset_m, 0.0f);
  const float minimum_offset_m = fminf(
      fmaxf(config->minimum_lateral_offset_m, 0.0f), maximum_offset_m);
  const float trigger = fminf(fmaxf(config->trigger_probability, 0.0f), 1.0f);
  const float bounded_risk = fminf(fmaxf(risk, trigger), 1.0f);
  const float urgency = trigger < 1.0f
      ? (bounded_risk - trigger) / (1.0f - trigger) : 1.0f;
  return minimum_offset_m + urgency * (maximum_offset_m - minimum_offset_m);
}

static float confidenceWeight(float score) {
  return 1.0f / (1.0f + expf(-fminf(fmaxf(score, -10.0f), 10.0f)));
}

static uint8_t dangerSectorForClearanceSector(int sector) {
  static const uint8_t danger_sector_for_clearance[4] = {
    TINYRACER_DANGER_LEFT, TINYRACER_DANGER_CENTER,
    TINYRACER_DANGER_CENTER, TINYRACER_DANGER_RIGHT,
  };
  return danger_sector_for_clearance[sector];
}

static float dangerForClearanceSector(
    const TinyRacerPerceptionObservation *observation, int sector) {
  return observation->danger_probability[
      dangerSectorForClearanceSector(sector)];
}

void tinyRacerNavigationReset(TinyRacerNavigationState *state) {
  memset(state, 0, sizeof(*state));
}

void tinyRacerNavigationFuseSectorRisk(
    TinyRacerPerceptionObservation *observation, float risk_threshold,
    float ambiguity_margin, int8_t ambiguous_pass_side) {
  if (!observation->has_sector_danger) {
    return;
  }
  float maximum_risk = 0.0f;
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    const float risk = fminf(fmaxf(
        observation->danger_probability[sector], 0.0f), 1.0f);
    maximum_risk = fmaxf(maximum_risk, risk);
  }
  const float left_risk = observation->danger_probability[TINYRACER_DANGER_LEFT];
  const float center_risk = fminf(fmaxf(
      observation->danger_probability[TINYRACER_DANGER_CENTER], 0.0f), 1.0f);
  const float right_risk = observation->danger_probability[TINYRACER_DANGER_RIGHT];
  /* Forward speed depends only on what lies in the flight direction. Side
   * sectors continue to select the pass direction, but a high side-on score
   * must not reduce progress to zero after the bypass lane is captured. */
  observation->collision_probability = center_risk;
  if (maximum_risk < risk_threshold) {
    return;
  }
  /* Positive steering/pass side is left. Move away from the half-image with
   * greater aggregate risk; only the configured ambiguity band may use the
   * caller's curvature-aware tie preference. */
  const float bounded_ambiguity_margin = fmaxf(ambiguity_margin, 0.0f);
  if (fabsf(left_risk - right_risk) <= bounded_ambiguity_margin &&
      ambiguous_pass_side != 0) {
    observation->steering_command = ambiguous_pass_side > 0 ? 1.0f : -1.0f;
  } else if (left_risk > right_risk + 1.0e-6f) {
    observation->steering_command = -1.0f;
  } else if (right_risk > left_risk + 1.0e-6f) {
    observation->steering_command = 1.0f;
  }
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
  if (observation->has_sector_danger) {
    intent->left_collision_probability = fminf(fmaxf(
        observation->danger_probability[TINYRACER_DANGER_LEFT], 0.0f), 1.0f);
    intent->center_collision_probability = fminf(fmaxf(
        observation->danger_probability[TINYRACER_DANGER_CENTER], 0.0f), 1.0f);
    intent->right_collision_probability = fminf(fmaxf(
        observation->danger_probability[TINYRACER_DANGER_RIGHT], 0.0f), 1.0f);
  } else if (intent->yaw_rate_rad_s < 0.0f) {
    /* Negative steering avoids right, so the scalar threat is on the left. */
    intent->left_collision_probability = intent->collision_probability;
  } else {
    intent->right_collision_probability = intent->collision_probability;
  }
}

void tinyRacerDodgeReset(TinyRacerDodgeState *state) {
  memset(state, 0, sizeof(*state));
  state->phase = TINYRACER_DODGE_TRACK;
}

static void resetBacktrackRiskWindow(TinyRacerDodgeState *state) {
  state->backtrack_risk_sum = 0.0f;
  state->backtrack_risk_average = 0.0f;
  state->backtrack_risk_window_count = 0u;
  state->backtrack_risk_window_index = 0u;
}

static bool backtrackRiskAverageIsClear(
    TinyRacerDodgeState *state, float maximum_risk,
    float release_probability) {
  const uint8_t index = state->backtrack_risk_window_index;
  if (state->backtrack_risk_window_count >=
      TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES) {
    state->backtrack_risk_sum -= state->backtrack_risk_window[index];
  } else {
    ++state->backtrack_risk_window_count;
  }
  state->backtrack_risk_window[index] = maximum_risk;
  state->backtrack_risk_sum += maximum_risk;
  state->backtrack_risk_window_index = (uint8_t)(
      (index + 1u) % TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES);
  state->backtrack_risk_average = state->backtrack_risk_sum /
      (float)state->backtrack_risk_window_count;
  return state->backtrack_risk_window_count >=
             TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES &&
      state->backtrack_risk_average < release_probability;
}

static TinyRacerDodgePhase selectedAvoidanceState(
    const TinyRacerDodgeState *state,
    const TinyRacerNavigationIntent *navigation) {
  /* Navigation yaw already contains the sector-risk decision, including the
   * caller's curvature-aware tie preference. Honor that fused decision here;
   * comparing the raw sectors first would silently discard it and can choose
   * the outward side of a circular course near the arena boundary. */
  if (navigation->yaw_rate_rad_s > 1.0e-6f) {
    return TINYRACER_DODGE_AVOID_LEFT;
  }
  if (navigation->yaw_rate_rad_s < -1.0e-6f) {
    return TINYRACER_DODGE_AVOID_RIGHT;
  }
  const float left_risk = navigation->left_collision_probability;
  const float right_risk = navigation->right_collision_probability;
  if (right_risk > left_risk + 1.0e-6f) {
    return TINYRACER_DODGE_AVOID_LEFT;
  }
  if (left_risk > right_risk + 1.0e-6f) {
    return TINYRACER_DODGE_AVOID_RIGHT;
  }
  if (state->phase == TINYRACER_DODGE_AVOID_LEFT ||
      state->phase == TINYRACER_DODGE_HOLD_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
      state->phase == TINYRACER_DODGE_REARM_LEFT ||
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT) {
    return TINYRACER_DODGE_AVOID_LEFT;
  }
  if (state->phase == TINYRACER_DODGE_AVOID_RIGHT ||
      state->phase == TINYRACER_DODGE_HOLD_RIGHT ||
      state->phase == TINYRACER_DODGE_REJOIN_RIGHT ||
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT ||
      state->phase == TINYRACER_DODGE_REARM_RIGHT ||
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT) {
    return TINYRACER_DODGE_AVOID_RIGHT;
  }
  return navigation->yaw_rate_rad_s < 0.0f
      ? TINYRACER_DODGE_AVOID_RIGHT : TINYRACER_DODGE_AVOID_LEFT;
}

static TinyRacerDodgePhase recoveryStateForOffset(
    const TinyRacerDodgeState *state,
    const TinyRacerNavigationIntent *navigation,
    const TinyRacerDodgeConfig *config) {
  const float lateral_offset_m = state->lateral_offset_m;
  /* Use the same centered criterion as ordinary rejoin completion. Treating
   * a millimetric residual as an occupied bypass lane traps recovery in a
   * zero-width REJOIN/HOLD loop instead of allowing a fresh side decision. */
  if (fabsf(lateral_offset_m) <=
      fmaxf(config->rejoin_lateral_tolerance_m, 0.0f)) {
    return TINYRACER_DODGE_TRACK;
  }

  const float left_risk = navigation->left_collision_probability;
  const float right_risk = navigation->right_collision_probability;
  TinyRacerDodgePhase clear_side;
  if (left_risk + 1.0e-6f < right_risk) {
    clear_side = TINYRACER_DODGE_AVOID_LEFT;
  } else if (right_risk + 1.0e-6f < left_risk) {
    clear_side = TINYRACER_DODGE_AVOID_RIGHT;
  } else {
    /* With no measured advantage, keep the already occupied bypass lane. */
    clear_side = lateral_offset_m > 0.0f
        ? TINYRACER_DODGE_AVOID_LEFT : TINYRACER_DODGE_AVOID_RIGHT;
  }

  if (clear_side == TINYRACER_DODGE_AVOID_LEFT) {
    /* REJOIN_RIGHT is the zero-target return from the right bypass lane.
     * Only use AVOID_LEFT after reaching center; never use it as recentering. */
    return lateral_offset_m < 0.0f
        ? TINYRACER_DODGE_REJOIN_RIGHT
        : TINYRACER_DODGE_REDIRECT_PREP_LEFT;
  }
  return lateral_offset_m > 0.0f
      ? TINYRACER_DODGE_REJOIN_LEFT
      : TINYRACER_DODGE_REDIRECT_PREP_RIGHT;
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
    const TinyRacerDodgeFeedback *feedback,
    float dt_s,
    TinyRacerDodgeIntent *intent) {
  memset(intent, 0, sizeof(*intent));
  const bool avoidance_active =
      state->phase == TINYRACER_DODGE_AVOID_LEFT ||
      state->phase == TINYRACER_DODGE_AVOID_RIGHT ||
      state->phase == TINYRACER_DODGE_HOLD_LEFT ||
      state->phase == TINYRACER_DODGE_HOLD_RIGHT ||
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT ||
      state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_RIGHT ||
      state->phase == TINYRACER_DODGE_REARM_LEFT ||
      state->phase == TINYRACER_DODGE_REARM_RIGHT ||
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT ||
      state->phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
      state->phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT ||
      state->phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD ||
      state->phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
  const bool feedback_valid = feedback != NULL && feedback->valid &&
      isfinite(feedback->forward_progress_m) &&
      isfinite(feedback->lateral_offset_m) &&
      isfinite(feedback->lateral_speed_mps) &&
      isfinite(feedback->forward_speed_mps) &&
      isfinite(feedback->vertical_speed_mps) &&
      isfinite(feedback->tilt_rad) &&
      isfinite(feedback->body_rate_rad_s) &&
      isfinite(feedback->rejoin_heading_error_rad) &&
      isfinite(feedback->loop_scan_heading_error_rad);
  bool entered_backtrack_settle = false;
  bool entered_loop_scan = false;
  if (!navigation->active &&
      state->phase != TINYRACER_DODGE_EMERGENCY_BRAKE &&
      state->phase != TINYRACER_DODGE_BACKTRACK &&
      !avoidance_active) {
    state->phase = TINYRACER_DODGE_TRACK;
    state->lateral_offset_m = 0.0f;
    state->lateral_rate_mps = 0.0f;
    state->avoidance_target_offset_m = 0.0f;
    state->emergency_forward_speed_mps = 0.0f;
    state->recovery_forward_speed_mps = 0.0f;
    state->recovery_forward_speed_active = false;
    state->encounter_progress_valid = false;
    state->rearm_progress_valid = false;
    state->trigger_samples = 0u;
    state->redirect_trigger_samples = 0u;
    state->redirect_settle_samples = 0u;
    state->backtrack_cycle_count = 0u;
    state->loop_scan_pending = false;
    state->loop_escape_active = false;
    state->loop_escape_yaw_direction = 0;
    state->loop_scan_samples = 0u;
    state->loop_scan_risk_sum = 0.0f;
    resetBacktrackRiskWindow(state);
    state->backtrack_settle_samples = 0u;
    state->hold_clear_samples = 0u;
    state->rejoin_settle_samples = 0u;
    state->rearm_clear_samples = 0u;
    state->rejoin_alignment_clear_samples = 0u;
    intent->phase = state->phase;
    return;
  }

  if (navigation->new_sample) {
    state->left_collision_probability = fminf(fmaxf(
        navigation->left_collision_probability, 0.0f), 1.0f);
    state->right_collision_probability = fminf(fmaxf(
        navigation->right_collision_probability, 0.0f), 1.0f);
    state->center_collision_probability = fminf(fmaxf(
        navigation->center_collision_probability, 0.0f), 1.0f);
    const float maximum_risk = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
    if (state->center_collision_probability >
            config->emergency_brake_probability &&
        state->phase != TINYRACER_DODGE_LOOP_SCAN_LEFT &&
        state->phase != TINYRACER_DODGE_LOOP_SCAN_RIGHT &&
        state->phase != TINYRACER_DODGE_EMERGENCY_BRAKE &&
        state->phase != TINYRACER_DODGE_BACKTRACK &&
        state->phase != TINYRACER_DODGE_BACKTRACK_SETTLE) {
      const float measured_forward_speed_mps = feedback_valid
          ? fmaxf(feedback->forward_speed_mps, 0.0f) : 0.0f;
      const bool already_stopped = feedback_valid &&
          fabsf(feedback->tilt_rad) <=
              fmaxf(config->backtrack_settle_maximum_tilt_rad, 0.0f) &&
          fabsf(feedback->forward_speed_mps) <=
              fmaxf(config->backtrack_settle_maximum_forward_speed_mps,
                    0.0f) &&
          fabsf(feedback->lateral_speed_mps) <=
              fmaxf(config->backtrack_settle_maximum_lateral_speed_mps,
                    0.0f) &&
          fabsf(feedback->vertical_speed_mps) <=
              fmaxf(config->backtrack_settle_maximum_vertical_speed_mps,
                    0.0f) &&
          fabsf(feedback->body_rate_rad_s) <=
              fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f);
      if (state->backtrack_cycle_count < UINT8_MAX) {
        ++state->backtrack_cycle_count;
      }
      if (state->backtrack_cycle_count >= 3u) {
        state->loop_scan_pending = true;
        state->backtrack_cycle_count = 0u;
      }
      /* A full cached braking attitude is appropriate only while there is
       * forward momentum to remove. At an already settled stop it creates
       * a large pitch transient for no benefit; retreat directly instead.
       * Even when a three-cycle scan is pending, complete the retreat and its
       * moving-average clearance check before yawing. */
      state->phase = already_stopped
          ? TINYRACER_DODGE_BACKTRACK
          : TINYRACER_DODGE_EMERGENCY_BRAKE;
      state->emergency_forward_speed_mps = fmaxf(
          fmaxf(navigation->forward_speed_mps, 0.0f),
          measured_forward_speed_mps);
      state->recovery_forward_speed_mps = 0.0f;
      state->recovery_forward_speed_active = false;
      state->encounter_progress_valid = false;
      /* Preserve the current lane during the stop and retreat. Snapping an
       * active +/- offset back to zero would add a violent lateral reversal
       * precisely when center collision risk is highest. */
      state->rearm_progress_valid = false;
      state->trigger_samples = 0u;
      state->redirect_trigger_samples = 0u;
      state->redirect_settle_samples = 0u;
      state->loop_escape_active = false;
      state->loop_escape_yaw_direction = 0;
      resetBacktrackRiskWindow(state);
      state->backtrack_settle_samples = 0u;
      state->hold_clear_samples = 0u;
      state->rejoin_settle_samples = 0u;
      state->rearm_clear_samples = 0u;
      state->rejoin_alignment_clear_samples = 0u;
    } else if (state->phase == TINYRACER_DODGE_TRACK) {
      if (maximum_risk >= config->trigger_probability) {
        if (state->trigger_samples < UINT8_MAX) {
          ++state->trigger_samples;
        }
        /* Each ESPNet inference already spans its two-frame input window. A
         * fresh, threshold-crossing outer-sector result is therefore enough
         * evidence to start moving away from that side. Keep the additional
         * state-machine debounce only for center-only/ambiguous detections. */
        const float side_risk = fmaxf(
            state->left_collision_probability,
            state->right_collision_probability);
        const uint8_t trigger_samples_required =
            side_risk >= config->trigger_probability
            ? 1u : config->trigger_samples_required > 0u
            ? config->trigger_samples_required : 1u;
        if (state->trigger_samples >= trigger_samples_required) {
          const TinyRacerDodgePhase selected_phase =
              selectedAvoidanceState(state, navigation);
          /* At cruise speed, first shed longitudinal velocity before adding
           * the lateral acceleration. Reuse the existing redirect-settle
           * phase so the direct-motor controller never receives both axes as
           * one abrupt maneuver. Low-speed encounters still dodge at once. */
          const bool vehicle_ready_for_lateral = feedback_valid &&
              fabsf(feedback->tilt_rad) <=
                  fmaxf(config->redirect_maximum_tilt_rad, 0.0f) &&
              fabsf(feedback->forward_speed_mps) <=
                  fmaxf(config->redirect_maximum_forward_speed_mps, 0.0f) &&
              fabsf(feedback->lateral_speed_mps) <=
                  fmaxf(config->redirect_maximum_lateral_speed_mps, 0.0f) &&
              feedback->vertical_speed_mps >=
                  config->redirect_minimum_vertical_speed_mps &&
              fabsf(feedback->body_rate_rad_s) <=
                  fmaxf(config->backtrack_settle_maximum_body_rate_rad_s,
                        0.0f);
          state->phase = vehicle_ready_for_lateral
              ? selected_phase
              : selected_phase == TINYRACER_DODGE_AVOID_LEFT
                  ? TINYRACER_DODGE_REDIRECT_PREP_LEFT
                  : TINYRACER_DODGE_REDIRECT_PREP_RIGHT;
          const float selected_risk =
              selected_phase == TINYRACER_DODGE_AVOID_LEFT
              ? fmaxf(state->center_collision_probability,
                      state->right_collision_probability)
              : fmaxf(state->left_collision_probability,
                      state->center_collision_probability);
          state->avoidance_target_offset_m =
              riskScaledAvoidanceOffset(config, selected_risk);
          state->encounter_progress_valid = feedback_valid;
          if (feedback_valid) {
            state->encounter_start_forward_progress_m =
                feedback->forward_progress_m;
          }
          state->trigger_samples = 0u;
          state->redirect_trigger_samples = 0u;
          state->redirect_settle_samples = 0u;
          state->hold_clear_samples = 0u;
          state->rejoin_settle_samples = 0u;
          state->rearm_clear_samples = 0u;
          state->rejoin_alignment_clear_samples = 0u;
        }
      } else {
        state->trigger_samples = 0u;
      }
    } else if (state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
               state->phase == TINYRACER_DODGE_REJOIN_RIGHT ||
               state->phase == TINYRACER_DODGE_REARM_LEFT ||
               state->phase == TINYRACER_DODGE_REARM_RIGHT) {
      if (maximum_risk >= config->trigger_probability) {
        if (state->redirect_trigger_samples < UINT8_MAX) {
          ++state->redirect_trigger_samples;
        }
      } else {
        state->redirect_trigger_samples = 0u;
      }
      const uint8_t redirect_trigger_samples_required =
          config->redirect_trigger_samples_required > 0u
          ? config->redirect_trigger_samples_required : 1u;
      if (state->redirect_trigger_samples >=
          redirect_trigger_samples_required) {
        const bool rejoining_left =
            state->phase == TINYRACER_DODGE_REJOIN_LEFT;
        const bool rejoining_right =
            state->phase == TINYRACER_DODGE_REJOIN_RIGHT;
        if (rejoining_left || rejoining_right) {
          /* The camera now faces the merge tangent, so renewed risk means the
           * merge itself is blocked. Pause on the already-clear bypass lane
           * and advance before retrying; launching another AVOID on the same
           * side can only grow the outward excursion. Preserve the current
           * offset so HOLD cannot snap back to the old, larger target. */
          state->phase = rejoining_left
              ? TINYRACER_DODGE_HOLD_LEFT
              : TINYRACER_DODGE_HOLD_RIGHT;
          state->avoidance_target_offset_m = fabsf(state->lateral_offset_m);
          state->encounter_progress_valid = feedback_valid;
          if (feedback_valid) {
            state->encounter_start_forward_progress_m =
                feedback->forward_progress_m;
          }
          state->rejoin_spline_active = false;
        } else {
          /* REARM is already centered, so a new encounter should still use
           * the perception-selected clear side. */
          const TinyRacerDodgePhase redirect_side =
              selectedAvoidanceState(state, navigation);
          state->phase = redirect_side == TINYRACER_DODGE_AVOID_LEFT
              ? TINYRACER_DODGE_REDIRECT_PREP_LEFT
              : TINYRACER_DODGE_REDIRECT_PREP_RIGHT;
          state->encounter_progress_valid = false;
        }
        state->rearm_progress_valid = false;
        state->trigger_samples = 0u;
        state->redirect_trigger_samples = 0u;
        state->redirect_settle_samples = 0u;
        state->hold_clear_samples = 0u;
        state->rejoin_settle_samples = 0u;
        state->rearm_clear_samples = 0u;
        state->rejoin_alignment_clear_samples = 0u;
      }
    }
  }

  if (state->phase == TINYRACER_DODGE_EMERGENCY_BRAKE) {
    state->emergency_forward_speed_mps = fmaxf(
        state->emergency_forward_speed_mps
            - fmaxf(config->emergency_brake_deceleration_mps2, 0.0f)
                * fmaxf(dt_s, 0.0f),
        0.0f);
    const bool vehicle_stopped = feedback_valid &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->backtrack_settle_maximum_tilt_rad, 0.0f) &&
        fabsf(feedback->forward_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_forward_speed_mps, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_lateral_speed_mps, 0.0f) &&
        fabsf(feedback->vertical_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_vertical_speed_mps, 0.0f) &&
        fabsf(feedback->body_rate_rad_s) <=
            fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f);
    if (state->emergency_forward_speed_mps <= 0.0f &&
        navigation->new_sample && vehicle_stopped) {
      /* A pending three-cycle scan never bypasses retreat clearance. */
      state->phase = TINYRACER_DODGE_BACKTRACK;
      state->backtrack_settle_samples = 0u;
    }
  }

  if (state->phase == TINYRACER_DODGE_BACKTRACK) {
    state->lateral_rate_mps = 0.0f;
    const float maximum_risk = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
    if (navigation->new_sample) {
      const bool risk_is_clear = backtrackRiskAverageIsClear(
          state, maximum_risk, config->backtrack_release_probability);
      if (risk_is_clear) {
        state->phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
        state->backtrack_settle_samples = 0u;
        entered_backtrack_settle = true;
      }
    }
  }

  if (state->phase == TINYRACER_DODGE_BACKTRACK_SETTLE) {
    state->lateral_rate_mps = 0.0f;
    const bool vehicle_stable = feedback_valid &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->backtrack_settle_maximum_tilt_rad, 0.0f) &&
        fabsf(feedback->forward_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_forward_speed_mps, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_lateral_speed_mps, 0.0f) &&
        fabsf(feedback->vertical_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_vertical_speed_mps, 0.0f) &&
        fabsf(feedback->body_rate_rad_s) <=
            fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f);
    if (navigation->new_sample && vehicle_stable &&
        !entered_backtrack_settle) {
      /* Entry already required a clear retreat frame. Once stationary, do
       * not let renewed risk restart brake/retreat forever. The handoff below
       * compares the two side risks, continuing on the clearer occupied lane
       * or using a true zero-target REJOIN before crossing the route. */
      if (state->backtrack_settle_samples < UINT8_MAX) {
        ++state->backtrack_settle_samples;
      }
    } else if (!vehicle_stable) {
      state->backtrack_settle_samples = 0u;
    }
    const uint8_t settle_samples_required =
        config->backtrack_settle_samples_required > 0u
        ? config->backtrack_settle_samples_required : 1u;
    if (state->backtrack_settle_samples >= settle_samples_required) {
      if (state->loop_scan_pending) {
        state->phase = TINYRACER_DODGE_LOOP_SCAN_LEFT;
        entered_loop_scan = true;
        state->loop_scan_pending = false;
        state->loop_scan_samples = 0u;
        state->loop_scan_risk_sum = 0.0f;
        state->loop_escape_active = false;
        state->loop_escape_yaw_direction = 0;
      } else {
        state->phase = recoveryStateForOffset(state, navigation, config);
      }
      state->backtrack_settle_samples = 0u;
      state->rejoin_alignment_clear_samples = 0u;
      state->avoidance_target_offset_m = fmaxf(
          state->avoidance_target_offset_m, fabsf(state->lateral_offset_m));
      state->encounter_progress_valid = feedback_valid;
      if (feedback_valid) {
        state->encounter_start_forward_progress_m =
            feedback->forward_progress_m;
      }
      state->recovery_forward_speed_mps = 0.0f;
      state->recovery_forward_speed_active = true;
    }
  }

  const bool loop_scan_left =
      state->phase == TINYRACER_DODGE_LOOP_SCAN_LEFT;
  const bool loop_scan_right =
      state->phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT;
  if (loop_scan_left || loop_scan_right) {
    state->lateral_rate_mps = 0.0f;
    const float heading_tolerance_rad = fmaxf(
        config->rejoin_alignment_heading_tolerance_rad, 0.0f);
    const bool scan_pose_settled = feedback_valid &&
        fabsf(feedback->forward_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_forward_speed_mps, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_lateral_speed_mps, 0.0f) &&
        fabsf(feedback->vertical_speed_mps) <=
            fmaxf(config->backtrack_settle_maximum_vertical_speed_mps, 0.0f) &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->backtrack_settle_maximum_tilt_rad, 0.0f) &&
        fabsf(feedback->body_rate_rad_s) <= fminf(
            fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f),
            0.15f);
    if (!entered_loop_scan && navigation->new_sample && scan_pose_settled &&
        fabsf(feedback->loop_scan_heading_error_rad) <=
            heading_tolerance_rad) {
      state->loop_scan_risk_sum += state->center_collision_probability;
      if (state->loop_scan_samples < UINT8_MAX) {
        ++state->loop_scan_samples;
      }
    }
    const uint8_t samples_required = config->loop_scan_samples_required > 0u
        ? config->loop_scan_samples_required : 1u;
    if (state->loop_scan_samples >= samples_required) {
      const float average_risk = state->loop_scan_risk_sum /
          (float)state->loop_scan_samples;
      state->loop_scan_samples = 0u;
      state->loop_scan_risk_sum = 0.0f;
      const float clear_threshold = fminf(fmaxf(
          config->backtrack_release_probability, 0.0f), 1.0f);
      if (loop_scan_left) {
        state->loop_scan_left_risk = average_risk;
        if (average_risk < clear_threshold) {
          state->phase = TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
          state->loop_escape_yaw_direction = 1;
          state->loop_escape_active = true;
        } else {
          state->phase = TINYRACER_DODGE_LOOP_SCAN_RIGHT;
        }
      } else {
        state->loop_scan_right_risk = average_risk;
        if (average_risk < clear_threshold) {
          state->phase = TINYRACER_DODGE_LOOP_ESCAPE_FORWARD;
          state->loop_escape_yaw_direction = -1;
          state->loop_escape_active = true;
        } else {
          /* Neither yawed center view is clear. Retreat farther, then repeat
           * the scan instead of recentering and translating toward a blocked
           * side. BACKTRACK itself remains gated only by its risk average. */
          state->phase = TINYRACER_DODGE_BACKTRACK;
          state->loop_scan_pending = true;
          state->loop_escape_yaw_direction = 0;
          state->loop_escape_active = false;
          resetBacktrackRiskWindow(state);
        }
      }
      if (state->phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD) {
        state->encounter_progress_valid = feedback_valid;
        if (feedback_valid) {
          state->encounter_start_forward_progress_m =
              feedback->forward_progress_m;
        }
        state->recovery_forward_speed_mps = 0.0f;
        state->recovery_forward_speed_active = true;
      }
    }
  }

  const bool rejoin_profile_phase =
      state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_RIGHT;
  if (!rejoin_profile_phase) {
    state->rejoin_spline_active = false;
    state->rejoin_spline_progress_m = 0.0f;
    state->rejoin_spline_slope = 0.0f;
  } else if (feedback_valid) {
    startRejoinSpline(state, config, feedback);
  }

  const float dt = fmaxf(dt_s, 0.0f);
  const float previous_offset_m = state->lateral_offset_m;
  float signed_rate_mps = 0.0f;
  float avoidance_probability = 0.0f;
  const float maximum_offset = state->loop_escape_active
      ? fmaxf(config->loop_escape_maximum_offset_m,
              fmaxf(config->maximum_lateral_offset_m, 0.0f))
      : fmaxf(config->maximum_lateral_offset_m, 0.0f);
  const float rejoin_rate = fmaxf(config->rejoin_lateral_rate_mps, 0.0f);
  float target_offset_m = state->lateral_offset_m;
  float lateral_rate_limit_mps = 0.0f;
  if (state->phase == TINYRACER_DODGE_AVOID_LEFT) {
    avoidance_probability = fmaxf(state->center_collision_probability,
                                   state->right_collision_probability);
    state->avoidance_target_offset_m = fmaxf(
        state->avoidance_target_offset_m,
        riskScaledAvoidanceOffset(config, avoidance_probability));
    target_offset_m = state->avoidance_target_offset_m;
    lateral_rate_limit_mps =
        riskScaledAvoidanceRate(config, avoidance_probability);
  } else if (state->phase == TINYRACER_DODGE_AVOID_RIGHT) {
    avoidance_probability = fmaxf(state->left_collision_probability,
                                   state->center_collision_probability);
    state->avoidance_target_offset_m = fmaxf(
        state->avoidance_target_offset_m,
        riskScaledAvoidanceOffset(config, avoidance_probability));
    target_offset_m = -state->avoidance_target_offset_m;
    lateral_rate_limit_mps =
        riskScaledAvoidanceRate(config, avoidance_probability);
  } else if (state->phase == TINYRACER_DODGE_REJOIN_LEFT) {
    avoidance_probability = fmaxf(state->center_collision_probability,
                                   state->right_collision_probability);
    target_offset_m = 0.0f;
    lateral_rate_limit_mps = rejoin_rate;
  } else if (state->phase == TINYRACER_DODGE_REJOIN_RIGHT) {
    avoidance_probability = fmaxf(state->left_collision_probability,
                                   state->center_collision_probability);
    target_offset_m = 0.0f;
    lateral_rate_limit_mps = rejoin_rate;
  } else if (state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT) {
    avoidance_probability = fmaxf(state->center_collision_probability,
                                   state->right_collision_probability);
  } else if (state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT) {
    avoidance_probability = fmaxf(state->left_collision_probability,
                                   state->center_collision_probability);
  } else if (state->phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD) {
    avoidance_probability = state->center_collision_probability;
  } else if (state->phase == TINYRACER_DODGE_EMERGENCY_BRAKE ||
             state->phase == TINYRACER_DODGE_BACKTRACK ||
             state->phase == TINYRACER_DODGE_BACKTRACK_SETTLE ||
             state->phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
             state->phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT) {
    avoidance_probability = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
  }
  if ((state->phase == TINYRACER_DODGE_AVOID_LEFT ||
       state->phase == TINYRACER_DODGE_AVOID_RIGHT) &&
      !state->encounter_progress_valid && feedback_valid) {
    state->encounter_start_forward_progress_m = feedback->forward_progress_m;
    state->encounter_progress_valid = true;
  }
  const bool lateral_motion_active =
      state->phase == TINYRACER_DODGE_AVOID_LEFT ||
      state->phase == TINYRACER_DODGE_AVOID_RIGHT ||
      state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
      state->phase == TINYRACER_DODGE_REJOIN_RIGHT;
  const bool redirect_prep =
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT;
  const bool emergency_braking =
      state->phase == TINYRACER_DODGE_EMERGENCY_BRAKE;
  if (redirect_prep) {
    const float acceleration_mps2 =
        fmaxf(config->lateral_acceleration_mps2, 0.0f);
    state->lateral_rate_mps = acceleration_mps2 > 0.0f
        ? moveToward(state->lateral_rate_mps, 0.0f, acceleration_mps2 * dt)
        : 0.0f;
    state->lateral_offset_m += state->lateral_rate_mps * dt;
  } else if (emergency_braking) {
    /* Keep the escape direction while removing its velocity continuously.
     * Snapping an active lateral reference rate to zero at the same instant
     * as the 6 m/s^2 longitudinal stop creates a two-axis jerk that can tip
     * the direct-motor controller before braking has time to work. */
    const float acceleration_mps2 =
        fmaxf(config->lateral_acceleration_mps2, 0.0f);
    state->lateral_rate_mps = acceleration_mps2 > 0.0f
        ? moveToward(state->lateral_rate_mps, 0.0f, acceleration_mps2 * dt)
        : 0.0f;
    state->lateral_offset_m += state->lateral_rate_mps * dt;
  } else if (rejoin_profile_phase && state->rejoin_spline_active) {
    if (feedback_valid) {
      state->rejoin_spline_progress_m = fminf(fmaxf(
          feedback->forward_progress_m -
              state->rejoin_spline_start_forward_progress_m,
          0.0f), fmaxf(config->rejoin_spline_length_m, 0.0f));
    }
    tinyRacerFrenetSplineSample(
        state->rejoin_spline_progress_m,
        config->rejoin_spline_length_m,
        state->rejoin_spline_start_offset_m,
        state->rejoin_spline_start_slope,
        0.0f, 0.0f,
        &state->lateral_offset_m, &state->rejoin_spline_slope);
    state->lateral_rate_mps = state->rejoin_spline_slope *
        fmaxf(config->rejoin_forward_speed_mps, 0.0f);
  } else if (lateral_motion_active) {
    const float remaining_m = target_offset_m - state->lateral_offset_m;
    const float acceleration_mps2 =
        fmaxf(config->lateral_acceleration_mps2, 0.0f);
    float requested_rate_mps = 0.0f;
    if (fabsf(remaining_m) > 1.0e-6f) {
      const float direction = copysignf(1.0f, remaining_m);
      const float rate_toward_target_mps =
          direction * state->lateral_rate_mps;
      const float stopping_distance_m = acceleration_mps2 > 0.0f &&
          rate_toward_target_mps > 0.0f
          ? rate_toward_target_mps * rate_toward_target_mps /
              (2.0f * acceleration_mps2)
          : 0.0f;
      const float next_step_travel_m =
          fmaxf(rate_toward_target_mps, 0.0f) * dt;
      requested_rate_mps = acceleration_mps2 > 0.0f &&
          stopping_distance_m + next_step_travel_m >= fabsf(remaining_m)
          ? 0.0f : direction * lateral_rate_limit_mps;
    }
    state->lateral_rate_mps = acceleration_mps2 > 0.0f
        ? moveToward(state->lateral_rate_mps, requested_rate_mps,
                     acceleration_mps2 * dt)
        : requested_rate_mps;
    state->lateral_offset_m += state->lateral_rate_mps * dt;
    const bool crossed_target =
        (target_offset_m - previous_offset_m) *
            (target_offset_m - state->lateral_offset_m) <= 0.0f;
    if (crossed_target ||
        fabsf(target_offset_m - state->lateral_offset_m) < 1.0e-6f) {
      state->lateral_offset_m = target_offset_m;
      state->lateral_rate_mps = 0.0f;
    }
  } else {
    state->lateral_rate_mps = 0.0f;
  }
  state->lateral_offset_m = fminf(fmaxf(
      state->lateral_offset_m, -maximum_offset), maximum_offset);
  signed_rate_mps = dt > 0.0f
      ? (state->lateral_offset_m - previous_offset_m) / dt : 0.0f;
  if (rejoin_profile_phase && state->rejoin_spline_active) {
    signed_rate_mps = state->lateral_rate_mps;
  }
  if (state->phase == TINYRACER_DODGE_AVOID_LEFT) {
    if (state->lateral_offset_m >= state->avoidance_target_offset_m &&
        fabsf(state->lateral_rate_mps) < 1.0e-6f) {
      state->phase = TINYRACER_DODGE_HOLD_LEFT;
      signed_rate_mps = 0.0f;
    }
  } else if (state->phase == TINYRACER_DODGE_AVOID_RIGHT) {
    if (state->lateral_offset_m <= -state->avoidance_target_offset_m &&
        fabsf(state->lateral_rate_mps) < 1.0e-6f) {
      state->phase = TINYRACER_DODGE_HOLD_RIGHT;
      signed_rate_mps = 0.0f;
    }
  }

  if (state->phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD) {
    state->lateral_rate_mps = 0.0f;
    signed_rate_mps = 0.0f;
    const float forward_progress_m = feedback_valid &&
        state->encounter_progress_valid
        ? feedback->forward_progress_m - state->encounter_start_forward_progress_m
        : 0.0f;
    const bool directional_route_clear = navigation->new_sample &&
        feedback_valid && state->encounter_progress_valid &&
        forward_progress_m >=
            fmaxf(config->loop_escape_spline_length_m, 0.0f) &&
        state->center_collision_probability <
            config->backtrack_release_probability;
    if (directional_route_clear) {
      if (state->hold_clear_samples < UINT8_MAX) {
        ++state->hold_clear_samples;
      }
    } else if (navigation->new_sample) {
      state->hold_clear_samples = 0u;
    }
    const uint8_t clear_samples_required =
        config->hold_clear_samples_required > 0u
        ? config->hold_clear_samples_required : 1u;
    if (state->hold_clear_samples >= clear_samples_required) {
      state->phase = TINYRACER_DODGE_TRACK;
      state->lateral_offset_m = 0.0f;
      state->loop_escape_active = false;
      state->loop_escape_yaw_direction = 0;
      state->encounter_progress_valid = false;
      state->recovery_forward_speed_active = false;
      state->backtrack_cycle_count = 0u;
      state->hold_clear_samples = 0u;
    }
  }

  const bool redirect_prep_left =
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT;
  const bool redirect_prep_right =
      state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT;
  if (redirect_prep_left || redirect_prep_right) {
    const bool reference_stopped = fabsf(state->lateral_rate_mps) < 1.0e-6f;
    const bool vehicle_stable = feedback_valid && reference_stopped &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->redirect_maximum_tilt_rad, 0.0f) &&
        fabsf(feedback->forward_speed_mps) <=
            fmaxf(config->redirect_maximum_forward_speed_mps, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->redirect_maximum_lateral_speed_mps, 0.0f) &&
        feedback->vertical_speed_mps >=
            config->redirect_minimum_vertical_speed_mps &&
        fabsf(feedback->body_rate_rad_s) <=
            fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f);
    if (vehicle_stable) {
      if (state->redirect_settle_samples < UINT8_MAX) {
        ++state->redirect_settle_samples;
      }
    } else {
      state->redirect_settle_samples = 0u;
    }
    const uint8_t redirect_settle_samples_required =
        config->redirect_settle_samples_required > 0u
        ? config->redirect_settle_samples_required : 1u;
    if (state->redirect_settle_samples >= redirect_settle_samples_required) {
      state->phase = redirect_prep_left
          ? TINYRACER_DODGE_AVOID_LEFT : TINYRACER_DODGE_AVOID_RIGHT;
      state->avoidance_target_offset_m = 0.0f;
      state->encounter_progress_valid = feedback_valid;
      if (feedback_valid) {
        state->encounter_start_forward_progress_m =
            feedback->forward_progress_m;
      }
      state->rearm_progress_valid = false;
      state->trigger_samples = 0u;
      state->redirect_trigger_samples = 0u;
      state->redirect_settle_samples = 0u;
      state->hold_clear_samples = 0u;
      state->rejoin_settle_samples = 0u;
      state->rearm_clear_samples = 0u;
    }
  }

  const bool hold_left = state->phase == TINYRACER_DODGE_HOLD_LEFT;
  const bool hold_right = state->phase == TINYRACER_DODGE_HOLD_RIGHT;
  if (hold_left || hold_right) {
    avoidance_probability = hold_left
        ? fmaxf(state->center_collision_probability,
                state->right_collision_probability)
        : fmaxf(state->left_collision_probability,
                state->center_collision_probability);
    state->lateral_offset_m = hold_left
        ? state->avoidance_target_offset_m
        : -state->avoidance_target_offset_m;
    state->lateral_rate_mps = 0.0f;
    signed_rate_mps = 0.0f;
    const float maximum_risk = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
    const float forward_progress_m = feedback_valid
        ? feedback->forward_progress_m - state->encounter_start_forward_progress_m
        : 0.0f;
    const bool lane_captured = feedback_valid &&
        fabsf(feedback->lateral_offset_m - state->lateral_offset_m) <=
            fmaxf(config->rejoin_lateral_tolerance_m, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->rejoin_lateral_speed_tolerance_mps, 0.0f) &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->redirect_maximum_tilt_rad, 0.0f);
    const bool obstacle_cleared = navigation->new_sample && feedback_valid &&
        state->encounter_progress_valid && lane_captured &&
        forward_progress_m >=
            fmaxf(config->minimum_hold_forward_progress_m, 0.0f) &&
        maximum_risk < config->hold_release_probability;
    if (obstacle_cleared) {
      if (state->hold_clear_samples < UINT8_MAX) {
        ++state->hold_clear_samples;
      }
    } else if (navigation->new_sample) {
      state->hold_clear_samples = 0u;
    }
    const uint8_t hold_clear_samples_required =
        config->hold_clear_samples_required > 0u
        ? config->hold_clear_samples_required : 1u;
    if (state->hold_clear_samples >= hold_clear_samples_required) {
      state->phase = hold_left
          ? TINYRACER_DODGE_REJOIN_ALIGN_LEFT
          : TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
      state->hold_clear_samples = 0u;
      state->rejoin_alignment_clear_samples = 0u;
    }
  }

  /* A forward camera cannot validate a side-on return while the vehicle is
   * route-tangent. Hold the bypass lane until the camera is tangent to the
   * planned diagonal rejoin and consecutive fresh frames agree it is clear.
   * A blocked aligned view restarts HOLD from the current progress so the
   * vehicle advances past the obstruction before attempting another scan. */
  const bool rejoin_align_left =
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT;
  const bool rejoin_align_right =
      state->phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
  if (rejoin_align_left || rejoin_align_right) {
    state->lateral_offset_m = rejoin_align_left
        ? state->avoidance_target_offset_m
        : -state->avoidance_target_offset_m;
    state->lateral_rate_mps = 0.0f;
    signed_rate_mps = 0.0f;
    const float maximum_risk = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
    avoidance_probability = maximum_risk;
    const bool heading_aligned = feedback_valid &&
        fabsf(feedback->rejoin_heading_error_rad) <= fmaxf(
            config->rejoin_alignment_heading_tolerance_rad, 0.0f);
    if (navigation->new_sample && heading_aligned &&
        maximum_risk < config->hold_release_probability) {
      if (state->rejoin_alignment_clear_samples < UINT8_MAX) {
        ++state->rejoin_alignment_clear_samples;
      }
    } else if (navigation->new_sample) {
      state->rejoin_alignment_clear_samples = 0u;
      if (heading_aligned &&
          maximum_risk >= config->hold_release_probability) {
        state->phase = rejoin_align_left
            ? TINYRACER_DODGE_HOLD_LEFT : TINYRACER_DODGE_HOLD_RIGHT;
        state->encounter_progress_valid = feedback_valid;
        if (feedback_valid) {
          state->encounter_start_forward_progress_m =
              feedback->forward_progress_m;
        }
      }
    }
    const uint8_t clear_samples_required =
        config->rejoin_alignment_clear_samples_required > 0u
        ? config->rejoin_alignment_clear_samples_required : 1u;
    if ((rejoin_align_left || rejoin_align_right) &&
        state->rejoin_alignment_clear_samples >= clear_samples_required) {
      state->phase = rejoin_align_left
          ? TINYRACER_DODGE_REJOIN_LEFT : TINYRACER_DODGE_REJOIN_RIGHT;
      state->rejoin_alignment_clear_samples = 0u;
      if (feedback_valid) {
        startRejoinSpline(state, config, feedback);
      }
    }
  }

  const bool rejoin_left = state->phase == TINYRACER_DODGE_REJOIN_LEFT;
  const bool rejoin_right = state->phase == TINYRACER_DODGE_REJOIN_RIGHT;
  if (rejoin_left || rejoin_right) {
    if ((rejoin_left && state->lateral_offset_m < 0.0f) ||
        (rejoin_right && state->lateral_offset_m > 0.0f)) {
      state->lateral_offset_m = 0.0f;
      state->lateral_rate_mps = 0.0f;
      signed_rate_mps = 0.0f;
    }
    const bool reference_centered = fabsf(state->lateral_offset_m) <= 1.0e-6f;
    const bool vehicle_settled = feedback_valid && reference_centered &&
        fabsf(feedback->lateral_offset_m) <=
            fmaxf(config->rejoin_lateral_tolerance_m, 0.0f) &&
        fabsf(feedback->lateral_speed_mps) <=
            fmaxf(config->rejoin_lateral_speed_tolerance_mps, 0.0f) &&
        fabsf(feedback->tilt_rad) <=
            fmaxf(config->redirect_maximum_tilt_rad, 0.0f) &&
        fabsf(feedback->body_rate_rad_s) <=
            fmaxf(config->backtrack_settle_maximum_body_rate_rad_s, 0.0f) &&
        feedback->vertical_speed_mps >=
            config->redirect_minimum_vertical_speed_mps;
    if (vehicle_settled) {
      if (state->rejoin_settle_samples < UINT8_MAX) {
        ++state->rejoin_settle_samples;
      }
    } else {
      state->rejoin_settle_samples = 0u;
    }
    const uint8_t rejoin_settle_samples_required =
        config->rejoin_settle_samples_required > 0u
        ? config->rejoin_settle_samples_required : 1u;
    if (state->rejoin_settle_samples >= rejoin_settle_samples_required) {
      state->phase = rejoin_left
          ? TINYRACER_DODGE_REARM_LEFT : TINYRACER_DODGE_REARM_RIGHT;
      state->lateral_offset_m = 0.0f;
      state->encounter_progress_valid = false;
      state->rearm_progress_valid = feedback_valid;
      if (feedback_valid) {
        state->rearm_start_forward_progress_m = feedback->forward_progress_m;
      }
      state->rejoin_settle_samples = 0u;
      state->rearm_clear_samples = 0u;
      state->rejoin_alignment_clear_samples = 0u;
    }
  }
  const bool rearm_left = state->phase == TINYRACER_DODGE_REARM_LEFT;
  const bool rearm_right = state->phase == TINYRACER_DODGE_REARM_RIGHT;
  if (rearm_left || rearm_right) {
    state->lateral_offset_m = 0.0f;
    state->lateral_rate_mps = 0.0f;
    signed_rate_mps = 0.0f;
    avoidance_probability = rearm_left
        ? fmaxf(state->center_collision_probability,
                state->right_collision_probability)
        : fmaxf(state->left_collision_probability,
                state->center_collision_probability);
    const float maximum_risk = fmaxf(state->center_collision_probability,
        fmaxf(state->left_collision_probability,
              state->right_collision_probability));
    const float rearm_progress_m = feedback_valid &&
        state->rearm_progress_valid
        ? feedback->forward_progress_m - state->rearm_start_forward_progress_m
        : 0.0f;
    const bool rearm_clear = navigation->new_sample && feedback_valid &&
        state->rearm_progress_valid &&
        rearm_progress_m >=
            fmaxf(config->minimum_rearm_forward_progress_m, 0.0f) &&
        maximum_risk < config->hold_release_probability;
    if (rearm_clear) {
      if (state->rearm_clear_samples < UINT8_MAX) {
        ++state->rearm_clear_samples;
      }
    } else if (navigation->new_sample) {
      state->rearm_clear_samples = 0u;
    }
    const uint8_t rearm_clear_samples_required =
        config->rearm_clear_samples_required > 0u
        ? config->rearm_clear_samples_required : 1u;
    if (state->rearm_clear_samples >= rearm_clear_samples_required) {
      state->phase = TINYRACER_DODGE_TRACK;
      state->backtrack_cycle_count = 0u;
      state->loop_scan_pending = false;
      state->loop_escape_active = false;
      state->loop_escape_yaw_direction = 0;
      state->loop_scan_samples = 0u;
      state->loop_scan_risk_sum = 0.0f;
      state->rearm_progress_valid = false;
      state->redirect_trigger_samples = 0u;
      state->redirect_settle_samples = 0u;
      state->rearm_clear_samples = 0u;
    }
  }
  if (state->phase == TINYRACER_DODGE_TRACK) {
    avoidance_probability = 0.0f;
  }

  if (state->phase != TINYRACER_DODGE_REJOIN_LEFT &&
      state->phase != TINYRACER_DODGE_REJOIN_RIGHT) {
    state->rejoin_spline_active = false;
    state->rejoin_spline_slope = 0.0f;
  }

  intent->phase = state->phase;
  intent->lateral_offset_m = state->lateral_offset_m;
  intent->lateral_rate_mps = signed_rate_mps;
  const float navigation_forward_speed_mps =
      fmaxf(navigation->forward_speed_mps, 0.0f);
  float requested_forward_speed_mps = navigation_forward_speed_mps;
  if (state->phase == TINYRACER_DODGE_EMERGENCY_BRAKE) {
    requested_forward_speed_mps = state->emergency_forward_speed_mps;
  } else if (state->phase == TINYRACER_DODGE_BACKTRACK) {
    requested_forward_speed_mps = -fmaxf(config->backtrack_speed_mps, 0.0f);
  } else if (state->phase == TINYRACER_DODGE_BACKTRACK_SETTLE) {
    requested_forward_speed_mps = 0.0f;
  } else if (state->phase == TINYRACER_DODGE_LOOP_SCAN_LEFT ||
             state->phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT) {
    requested_forward_speed_mps = 0.0f;
  } else if (state->phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->avoid_forward_speed_mps, 0.0f));
  } else if (state->phase == TINYRACER_DODGE_REJOIN_ALIGN_LEFT ||
             state->phase == TINYRACER_DODGE_REJOIN_ALIGN_RIGHT) {
    requested_forward_speed_mps = 0.0f;
  } else if (state->phase == TINYRACER_DODGE_AVOID_LEFT ||
             state->phase == TINYRACER_DODGE_AVOID_RIGHT) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->avoid_forward_speed_mps, 0.0f));
  } else if (state->phase == TINYRACER_DODGE_HOLD_LEFT ||
             state->phase == TINYRACER_DODGE_HOLD_RIGHT) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->hold_forward_speed_mps, 0.0f));
  } else if (state->phase == TINYRACER_DODGE_REJOIN_LEFT ||
             state->phase == TINYRACER_DODGE_REJOIN_RIGHT) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->rejoin_forward_speed_mps, 0.0f));
  } else if (state->phase == TINYRACER_DODGE_REARM_LEFT ||
             state->phase == TINYRACER_DODGE_REARM_RIGHT) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->rearm_forward_speed_mps, 0.0f));
  } else if (state->phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT ||
             state->phase == TINYRACER_DODGE_REDIRECT_PREP_RIGHT) {
    requested_forward_speed_mps = fminf(navigation_forward_speed_mps,
        fmaxf(config->redirect_forward_speed_mps, 0.0f));
  }
  const bool recovery_ramp_eligible =
      state->phase != TINYRACER_DODGE_EMERGENCY_BRAKE &&
      state->phase != TINYRACER_DODGE_BACKTRACK &&
      state->phase != TINYRACER_DODGE_BACKTRACK_SETTLE;
  if (state->recovery_forward_speed_active && recovery_ramp_eligible) {
    const float recovery_acceleration_mps2 =
        fmaxf(config->recovery_forward_acceleration_mps2, 0.0f);
    if (requested_forward_speed_mps <= state->recovery_forward_speed_mps) {
      state->recovery_forward_speed_mps = requested_forward_speed_mps;
    } else {
      state->recovery_forward_speed_mps = fminf(
          state->recovery_forward_speed_mps + recovery_acceleration_mps2 * dt,
          requested_forward_speed_mps);
    }
    requested_forward_speed_mps = state->recovery_forward_speed_mps;
    /* Keep the limiter active until it reaches the actual TRACK request.
     * Releasing it at the lower avoidance/rearm cap exposed the next horizon
     * to a one-cycle speed-limit jump after a long recovery. */
    if (state->phase == TINYRACER_DODGE_TRACK &&
        state->recovery_forward_speed_mps >=
            navigation_forward_speed_mps - 1.0e-6f) {
      state->recovery_forward_speed_active = false;
    }
  }
  intent->forward_speed_mps = requested_forward_speed_mps;
  if (state->rejoin_spline_active) {
    state->lateral_rate_mps = state->rejoin_spline_slope *
        fmaxf(requested_forward_speed_mps, 0.0f);
    intent->lateral_rate_mps = state->lateral_rate_mps;
  }
  intent->avoidance_probability = avoidance_probability;
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
    bool counted_probability[TINYRACER_DANGER_SECTORS] = {false};
    bool dangerous[TINYRACER_CLEARANCE_SECTORS];
    for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
      const bool confident =
          observation->confidence[sector] >= config->confidence_threshold;
      const bool metric_danger = observation->has_metric_clearance &&
          confident && observation->clearance_m[sector] <
              config->clearance_threshold_m;
      const bool probability_danger = observation->has_sector_danger &&
          dangerForClearanceSector(observation, sector) >=
              config->danger_probability_threshold;
      dangerous[sector] = (relevant_sector_mask & (1u << sector)) &&
          (metric_danger || probability_danger);
      if (dangerous[sector]) {
        const uint8_t danger_sector = dangerSectorForClearanceSector(sector);
        if (metric_danger || !counted_probability[danger_sector]) {
          ++dangerous_sector_count;
          dangerous_confidence_sum += probability_danger
              ? dangerForClearanceSector(observation, sector)
              : confidenceWeight(observation->confidence[sector]);
        }
        counted_probability[danger_sector] |= probability_danger;
      }
      path_hard_blocked |= metric_danger && dangerous[sector] &&
          observation->clearance_m[sector] <=
              config->hard_clearance_threshold_m;
      const bool sector_safe = observation->has_metric_clearance
          ? confident && observation->clearance_m[sector] >=
                config->release_clearance_m
          : (!observation->has_sector_danger ||
             dangerForClearanceSector(observation, sector) <
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
