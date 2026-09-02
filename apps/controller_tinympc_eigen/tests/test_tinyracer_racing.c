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

static void assertCommittedAvoidance(bool avoid_left) {
  const TinyRacerDodgeConfig config = {
    .trigger_probability = 0.80f,
    .minimum_lateral_rate_mps = 0.80f,
    .maximum_lateral_rate_mps = 1.20f,
    .rejoin_lateral_rate_mps = 0.20f,
    .rejoin_spline_length_m = 0.0f,
    .lateral_acceleration_mps2 = 100.0f,
    .maximum_lateral_offset_m = 0.50f,
    .minimum_lateral_offset_m = 0.20f,
    .emergency_brake_probability = 0.90f,
    .emergency_brake_deceleration_mps2 = 6.00f,
    .backtrack_release_probability = 0.50f,
    .backtrack_speed_mps = 0.10f,
    .backtrack_settle_maximum_tilt_rad = 0.35f,
    .backtrack_settle_maximum_forward_speed_mps = 0.20f,
    .backtrack_settle_maximum_lateral_speed_mps = 0.20f,
    .backtrack_settle_maximum_vertical_speed_mps = 0.20f,
    .backtrack_settle_maximum_body_rate_rad_s = 0.50f,
    .recovery_forward_acceleration_mps2 = 0.50f,
    .avoid_forward_speed_mps = 0.35f,
    .hold_forward_speed_mps = 0.30f,
    .rejoin_forward_speed_mps = 0.20f,
    .rearm_forward_speed_mps = 0.25f,
    .redirect_forward_speed_mps = 0.10f,
    .hold_release_probability = 0.50f,
    .minimum_hold_forward_progress_m = 0.60f,
    .minimum_rearm_forward_progress_m = 0.40f,
    .rejoin_lateral_tolerance_m = 0.10f,
    .rejoin_lateral_speed_tolerance_mps = 0.10f,
    .redirect_maximum_tilt_rad = 0.35f,
    .redirect_maximum_forward_speed_mps = 0.50f,
    .redirect_maximum_lateral_speed_mps = 0.20f,
    .redirect_minimum_vertical_speed_mps = -0.20f,
    .trigger_samples_required = 1,
    .redirect_trigger_samples_required = 2,
    .redirect_settle_samples_required = 2,
    .backtrack_settle_samples_required = 2,
    .hold_clear_samples_required = 2,
    .rejoin_settle_samples_required = 2,
    .rearm_clear_samples_required = 2,
    .rejoin_alignment_heading_tolerance_rad = 0.20f,
    .rejoin_alignment_clear_samples_required = 2,
    .loop_scan_yaw_rad = 0.78539816f,
    .loop_scan_samples_required = 2,
    .loop_escape_lateral_step_m = 0.40f,
    .loop_escape_maximum_offset_m = 0.80f,
    .loop_escape_spline_length_m = 0.60f,
  };
  const TinyRacerDodgePhase expected_phase = avoid_left
      ? TINYRACER_DODGE_AVOID_LEFT : TINYRACER_DODGE_AVOID_RIGHT;
  const TinyRacerDodgePhase hold_phase = avoid_left
      ? TINYRACER_DODGE_HOLD_LEFT : TINYRACER_DODGE_HOLD_RIGHT;
  const TinyRacerDodgePhase rejoin_phase = avoid_left
      ? TINYRACER_DODGE_REJOIN_LEFT : TINYRACER_DODGE_REJOIN_RIGHT;
  const TinyRacerDodgePhase align_phase = avoid_left
      ? TINYRACER_DODGE_REJOIN_ALIGN_LEFT
      : TINYRACER_DODGE_REJOIN_ALIGN_RIGHT;
  const TinyRacerDodgePhase rearm_phase = avoid_left
      ? TINYRACER_DODGE_REARM_LEFT : TINYRACER_DODGE_REARM_RIGHT;
  const float direction = avoid_left ? 1.0f : -1.0f;
  TinyRacerDodgeState state;
  TinyRacerDodgeIntent intent;
  TinyRacerNavigationIntent navigation;
  TinyRacerDodgeFeedback feedback = {
    .valid = true,
    .forward_progress_m = 10.0f,
    .lateral_offset_m = 0.0f,
    .lateral_speed_mps = 0.0f,
    .forward_speed_mps = 0.40f,
    .vertical_speed_mps = 0.0f,
    .tilt_rad = 0.0f,
  };
  tinyRacerDodgeReset(&state);
  memset(&navigation, 0, sizeof(navigation));
  navigation.active = true;
  navigation.new_sample = true;
  navigation.forward_speed_mps = 0.4f;

  /* Entry is inclusive at 0.80; 0.79 must remain in TRACK. */
  if (avoid_left) {
    navigation.right_collision_probability = 0.79f;
  } else {
    navigation.left_collision_probability = 0.79f;
  }
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == TINYRACER_DODGE_TRACK);
  if (avoid_left) {
    navigation.right_collision_probability = 0.80f;
  } else {
    navigation.left_collision_probability = 0.80f;
  }
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == expected_phase);
  assert(fabsf(intent.lateral_rate_mps - direction * 0.80f) < 1.0e-6f);
  assert(fabsf(intent.lateral_offset_m - direction * 0.08f) < 1.0e-6f);
  assert(fabsf(state.avoidance_target_offset_m - 0.20f) < 1.0e-6f);

  /* Once avoidance is committed, fresh risk continuously scales the bounded
   * outward rate from its aggressive floor to its configured maximum. */
  if (avoid_left) {
    navigation.right_collision_probability = 1.0f;
  } else {
    navigation.left_collision_probability = 1.0f;
  }
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == expected_phase);
  assert(fabsf(intent.lateral_rate_mps - direction * 1.20f) < 1.0e-6f);
  assert(fabsf(intent.lateral_offset_m - direction * 0.20f) < 1.0e-6f);
  assert(fabsf(state.avoidance_target_offset_m - 0.50f) < 1.0e-6f);

  /* Clear, opposite-side, missing-sample, and inactive-navigation inputs do
   * not cancel or redirect the latched outbound leg. */
  float previous_progress = direction * intent.lateral_offset_m;
  for (int step = 0; intent.phase == expected_phase; ++step) {
    navigation.active = true;
    navigation.new_sample = true;
    navigation.left_collision_probability = 0.0f;
    navigation.center_collision_probability = 0.0f;
    navigation.right_collision_probability = 0.0f;
    if (step == 1) {
      if (avoid_left) {
        navigation.left_collision_probability = 0.80f;
      } else {
        navigation.right_collision_probability = 0.80f;
      }
    } else if (step == 2) {
      navigation.new_sample = false;
    } else if (step == 3) {
      navigation.active = false;
      navigation.new_sample = false;
    }
    tinyRacerDodgeUpdate(
        &state, &navigation, &config, &feedback, 0.10f, &intent);
    const float progress = direction * intent.lateral_offset_m;
    assert(progress + 1.0e-6f >= previous_progress);
    assert(progress <= 0.50f + 1.0e-6f);
    previous_progress = progress;
    assert(step < 10);
  }
  assert(intent.phase == hold_phase);
  assert(fabsf(intent.lateral_offset_m - direction * 0.50f) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);

  /* Clearance alone, progress alone, and the exact release-risk boundary are
   * insufficient. HOLD releases only after both measured conditions persist
   * for fresh samples. */
  navigation.active = true;
  navigation.new_sample = true;
  navigation.left_collision_probability = 0.0f;
  navigation.center_collision_probability = 0.0f;
  navigation.right_collision_probability = 0.0f;
  feedback.lateral_offset_m = direction * 0.50f;
  feedback.lateral_speed_mps = 0.0f;
  feedback.rejoin_heading_error_rad = 0.201f;
  feedback.forward_progress_m = 10.59f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == hold_phase);
  feedback.forward_progress_m = 10.60f;
  navigation.center_collision_probability = 0.50f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == hold_phase);
  navigation.center_collision_probability = 0.49f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == hold_phase);
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == hold_phase);
  navigation.new_sample = true;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == align_phase);
  assert(fabsf(intent.forward_speed_mps) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);

  /* The inward scan counts only fresh, aligned, strictly clear frames. Exact
   * heading tolerance qualifies; the exact risk boundary does not. */
  feedback.rejoin_heading_error_rad = 0.20f;
  navigation.center_collision_probability = 0.50f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == hold_phase);
  assert(fabsf(state.encounter_start_forward_progress_m -
               feedback.forward_progress_m) < 1.0e-6f);
  feedback.forward_progress_m += 0.60f;
  feedback.rejoin_heading_error_rad = 0.201f;
  navigation.center_collision_probability = 0.49f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == align_phase);
  feedback.rejoin_heading_error_rad = 0.20f;
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(state.rejoin_alignment_clear_samples == 0u);
  navigation.new_sample = true;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == align_phase);
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rejoin_phase);

  /* Navigation dropout cannot cancel REJOIN. Its slower reference rate must
   * not permit TRACK until measured lateral position and speed also settle. */
  navigation.active = false;
  navigation.new_sample = false;
  feedback.lateral_offset_m = direction * 0.30f;
  feedback.lateral_speed_mps = direction * 0.20f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rejoin_phase);
  assert(fabsf(intent.lateral_rate_mps + direction * 0.20f) < 1.0e-6f);
  navigation.active = true;
  previous_progress = direction * intent.lateral_offset_m;
  for (int step = 0; fabsf(intent.lateral_offset_m) > 1.0e-6f; ++step) {
    navigation.new_sample = false;
    navigation.left_collision_probability = 0.0f;
    navigation.center_collision_probability = 0.0f;
    navigation.right_collision_probability = 0.0f;
    tinyRacerDodgeUpdate(
        &state, &navigation, &config, &feedback, 0.10f, &intent);
    const float progress = direction * intent.lateral_offset_m;
    assert(progress <= previous_progress + 1.0e-6f);
    assert(progress >= -1.0e-6f);
    previous_progress = progress;
    assert(intent.phase == rejoin_phase);
    assert(step < 30);
  }
  assert(fabsf(intent.lateral_offset_m) < 1.0e-6f);
  feedback.lateral_offset_m = direction * 0.10f;
  feedback.lateral_speed_mps = direction * 0.11f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rejoin_phase);
  feedback.valid = false;
  feedback.lateral_speed_mps = NAN;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rejoin_phase);
  feedback.valid = true;
  feedback.lateral_speed_mps = direction * 0.10f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rejoin_phase);
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);

  /* True TRACK requires additional measured separation and fresh clear
   * samples. Exact risk/distance boundaries retain their documented
   * strict/inclusive semantics. */
  navigation.new_sample = true;
  navigation.center_collision_probability = 0.0f;
  feedback.forward_progress_m += 0.39f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  feedback.forward_progress_m += 0.01f;
  navigation.center_collision_probability = 0.50f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  navigation.center_collision_probability = 0.49f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  navigation.new_sample = true;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == TINYRACER_DODGE_TRACK);

  /* A distinct threat during REARM is allowed to choose either side, but it
   * must persist and pass through REDIRECT_PREP before lateral motion starts. */
  state.phase = rearm_phase;
  state.rearm_progress_valid = true;
  state.rearm_start_forward_progress_m = feedback.forward_progress_m;
  navigation.left_collision_probability = avoid_left ? 0.95f : 0.0f;
  navigation.center_collision_probability = 0.0f;
  navigation.right_collision_probability = avoid_left ? 0.0f : 0.95f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == rearm_phase);
  navigation.new_sample = true;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  const TinyRacerDodgePhase redirect_prep_phase = avoid_left
      ? TINYRACER_DODGE_REDIRECT_PREP_RIGHT
      : TINYRACER_DODGE_REDIRECT_PREP_LEFT;
  const TinyRacerDodgePhase redirected_phase = avoid_left
      ? TINYRACER_DODGE_AVOID_RIGHT : TINYRACER_DODGE_AVOID_LEFT;
  assert(intent.phase == redirect_prep_phase);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(intent.forward_speed_mps - 0.10f) < 1.0e-6f);
  navigation.active = false;
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.phase == redirected_phase);
  assert(state.encounter_progress_valid);
  assert(fabsf(state.encounter_start_forward_progress_m -
               feedback.forward_progress_m) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);
}

static void assertBacktrackSettleRecovery(
    const TinyRacerDodgeConfig *config, bool offset_left, bool clear_left) {
  const float direction = offset_left ? 1.0f : -1.0f;
  const bool recenter = offset_left != clear_left;
  const TinyRacerDodgePhase expected_recovery = clear_left
      ? (recenter ? TINYRACER_DODGE_REJOIN_RIGHT
                  : TINYRACER_DODGE_REDIRECT_PREP_LEFT)
      : (recenter ? TINYRACER_DODGE_REJOIN_LEFT
                  : TINYRACER_DODGE_REDIRECT_PREP_RIGHT);
  TinyRacerDodgeState state;
  TinyRacerDodgeIntent intent;
  TinyRacerNavigationIntent navigation;
  TinyRacerDodgeFeedback feedback = {
    .valid = true,
    .forward_progress_m = 0.0f,
    .lateral_offset_m = direction * 0.30f,
    .lateral_speed_mps = direction * 0.20f,
    .forward_speed_mps = -0.20f,
    .vertical_speed_mps = 0.20f,
    .tilt_rad = 0.35f,
    .body_rate_rad_s = 0.50f,
  };
  tinyRacerDodgeReset(&state);
  state.phase = TINYRACER_DODGE_BACKTRACK;
  state.lateral_offset_m = direction * 0.30f;
  memset(&navigation, 0, sizeof(navigation));
  navigation.active = true;
  navigation.new_sample = true;
  navigation.forward_speed_mps = 0.40f;
  navigation.left_collision_probability = clear_left ? 0.10f : 0.49f;
  navigation.center_collision_probability = 0.20f;
  navigation.right_collision_probability = clear_left ? 0.49f : 0.10f;

  /* A one-frame dip below the release threshold cannot end retreat. The
   * complete five-frame moving-risk window must be clear; its fifth sample
   * enters SETTLE but cannot also count as stationary evidence. */
  for (uint8_t sample = 1u;
       sample < TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES; ++sample) {
    tinyRacerDodgeUpdate(
        &state, &navigation, config, &feedback, 0.10f, &intent);
    assert(intent.phase == TINYRACER_DODGE_BACKTRACK);
    assert(state.backtrack_risk_window_count == sample);
    assert(state.backtrack_risk_average <
           config->backtrack_release_probability);
    assert(fabsf(intent.forward_speed_mps + 0.10f) < 1.0e-6f);
  }
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE);
  assert(state.backtrack_risk_window_count ==
         TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES);
  assert(state.backtrack_settle_samples == 0u);
  assert(fabsf(intent.lateral_offset_m - direction * 0.30f) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(intent.forward_speed_mps) < 1.0e-6f);

  /* Exact measured limits qualify. Stale perception cannot advance the
   * counter, and invalid feedback clears accumulated evidence. */
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE);
  assert(state.backtrack_settle_samples == 1u);
  navigation.new_sample = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 1u);
  feedback.valid = false;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.valid = true;
  navigation.new_sample = true;

  /* Every measured stability signal independently blocks release. */
  feedback.forward_speed_mps = 0.201f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.forward_speed_mps = -0.20f;
  feedback.lateral_speed_mps = direction * 0.201f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.lateral_speed_mps = direction * 0.20f;
  feedback.vertical_speed_mps = -0.201f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.vertical_speed_mps = 0.20f;
  feedback.tilt_rad = 0.351f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.tilt_rad = 0.35f;
  feedback.body_rate_rad_s = 0.501f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 0u);
  feedback.body_rate_rad_s = 0.50f;

  /* Once retreat has produced a clear frame, recovery selects the lower-risk
   * side. Crossing the route begins with a zero-target REJOIN; continuing on
   * the occupied clear side begins with its matching redirect preparation. */
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(state.backtrack_settle_samples == 1u);
  navigation.center_collision_probability = 0.96f;
  tinyRacerDodgeUpdate(
      &state, &navigation, config, &feedback, 0.10f, &intent);
  assert(intent.phase == expected_recovery);
  assert(fabsf(intent.forward_speed_mps - 0.05f) < 1.0e-6f);
  if (recenter) {
    assert(fabsf(intent.lateral_offset_m - direction * 0.28f) < 1.0e-6f);
    assert(fabsf(intent.lateral_rate_mps + direction * 0.20f) < 1.0e-6f);
  } else {
    assert(fabsf(intent.lateral_offset_m - direction * 0.30f) < 1.0e-6f);
    assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);
  }
}

static void assertFrenetRejoinSpline(void) {
  float offset_m = 0.0f;
  float slope = 0.0f;
  tinyRacerFrenetSplineSample(
      0.0f, 2.40f, 0.50f, -0.12f, 0.0f, 0.0f,
      &offset_m, &slope);
  assert(fabsf(offset_m - 0.50f) < 1.0e-6f);
  assert(fabsf(slope + 0.12f) < 1.0e-6f);
  tinyRacerFrenetSplineSample(
      1.20f, 2.40f, 0.50f, -0.12f, 0.0f, 0.0f,
      &offset_m, &slope);
  assert(offset_m > 0.0f && offset_m < 0.50f);
  assert(slope < 0.0f);
  tinyRacerFrenetSplineSample(
      2.40f, 2.40f, 0.50f, -0.12f, 0.0f, 0.0f,
      &offset_m, &slope);
  assert(fabsf(offset_m) < 1.0e-6f);
  assert(fabsf(slope) < 1.0e-6f);

  const TinyRacerDodgeConfig config = {
    .rejoin_lateral_rate_mps = 0.20f,
    .rejoin_spline_length_m = 0.60f,
    .maximum_lateral_offset_m = 0.50f,
    .rejoin_forward_speed_mps = 0.40f,
  };
  TinyRacerDodgeState state;
  TinyRacerDodgeIntent intent;
  TinyRacerNavigationIntent navigation;
  TinyRacerDodgeFeedback feedback = {
    .valid = true,
    .forward_progress_m = 2.0f,
    .lateral_offset_m = 0.50f,
    .forward_speed_mps = 0.40f,
  };
  tinyRacerDodgeReset(&state);
  memset(&navigation, 0, sizeof(navigation));
  state.phase = TINYRACER_DODGE_REJOIN_LEFT;
  state.lateral_offset_m = 0.50f;
  navigation.active = true;
  navigation.forward_speed_mps = 0.40f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(state.rejoin_spline_active);
  assert(fabsf(intent.lateral_offset_m - 0.50f) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps + 0.20f) < 1.0e-6f);
  feedback.forward_progress_m = 2.30f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(intent.lateral_offset_m > 0.0f &&
         intent.lateral_offset_m < 0.50f);
  assert(intent.lateral_rate_mps < 0.0f);
  feedback.forward_progress_m = 2.60f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(fabsf(intent.lateral_offset_m) < 1.0e-6f);
  assert(fabsf(intent.lateral_rate_mps) < 1.0e-6f);

  /* A recovery that starts near center must not spline through zero and then
   * return from the wrong side. */
  tinyRacerDodgeReset(&state);
  state.phase = TINYRACER_DODGE_REJOIN_LEFT;
  state.lateral_offset_m = 0.02f;
  feedback.forward_progress_m = 4.0f;
  feedback.lateral_offset_m = 0.02f;
  tinyRacerDodgeUpdate(
      &state, &navigation, &config, &feedback, 0.10f, &intent);
  assert(state.rejoin_spline_start_slope >= -0.100001f);
  for (int sample = 1; sample <= 6; ++sample) {
    feedback.forward_progress_m = 4.0f + 0.10f * (float)sample;
    tinyRacerDodgeUpdate(
        &state, &navigation, &config, &feedback, 0.10f, &intent);
    assert(intent.lateral_offset_m >= -1.0e-6f);
    assert(intent.lateral_offset_m <= 0.020001f);
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
                        true, 0x0a, &intent);
  }
  assert(!intent.constraint_active);
  observation.danger_probability[2] = 0.60f;
  for (int sample = 0; sample < 3; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                        true, 0x0a, &intent);
    assert(!intent.constraint_active);
  }
  observation.sample++;
  tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                      true, 0x0a, &intent);
  assert(intent.constraint_active);

  /* Three distinct network risks must satisfy multi-sector evidence without
   * counting the center value twice through the four-ray geometry. */
  tinyRacerRaceReset(&state);
  memset(&intent, 0, sizeof(intent));
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    observation.danger_probability[sector] = 1.0f;
  }
  for (int sample = 0; sample < config.blocked_samples_required; ++sample) {
    observation.sample++;
    tinyRacerRaceUpdate(&state, &observation, &config, 0.02f, false, false,
                        true, 0x0f, &intent);
  }
  assert(intent.constraint_active);

  /* Spatial ESPNet side risks choose the opposite pass direction, while
   * forward speed uses only the center sector. */
  TinyRacerPerceptionObservation sector_navigation;
  memset(&sector_navigation, 0, sizeof(sector_navigation));
  sector_navigation.valid = true;
  sector_navigation.has_navigation_command = true;
  sector_navigation.has_sector_danger = true;
  sector_navigation.collision_probability = 0.14f;
  sector_navigation.steering_command = 0.25f;
  sector_navigation.danger_probability[0] = 0.60f;
  sector_navigation.danger_probability[1] = 0.14f;
  sector_navigation.danger_probability[2] = 0.04f;
  tinyRacerNavigationFuseSectorRisk(&sector_navigation, 0.50f, 0.0f, 0);
  assert(fabsf(sector_navigation.collision_probability - 0.14f) < 1.0e-6f);
  assert(fabsf(sector_navigation.steering_command + 1.0f) < 1.0e-6f);
  sector_navigation.danger_probability[0] = 0.04f;
  sector_navigation.danger_probability[2] = 0.70f;
  tinyRacerNavigationFuseSectorRisk(&sector_navigation, 0.50f, 0.0f, 0);
  assert(fabsf(sector_navigation.collision_probability - 0.14f) < 1.0e-6f);
  assert(fabsf(sector_navigation.steering_command - 1.0f) < 1.0e-6f);
  sector_navigation.collision_probability = 0.10f;
  sector_navigation.steering_command = 0.25f;
  sector_navigation.danger_probability[2] = 0.40f;
  tinyRacerNavigationFuseSectorRisk(&sector_navigation, 0.50f, 0.0f, 0);
  assert(fabsf(sector_navigation.collision_probability - 0.14f) < 1.0e-6f);
  assert(fabsf(sector_navigation.steering_command - 0.25f) < 1.0e-6f);
  sector_navigation.danger_probability[0] = 0.52f;
  sector_navigation.danger_probability[1] = 0.99f;
  sector_navigation.danger_probability[2] = 0.43f;
  tinyRacerNavigationFuseSectorRisk(
      &sector_navigation, 0.50f, 0.15f, 1);
  assert(fabsf(sector_navigation.collision_probability - 0.99f) < 1.0e-6f);
  assert(fabsf(sector_navigation.steering_command - 1.0f) < 1.0e-6f);

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

  /* Left and right avoidance are symmetric committed out-and-back maneuvers. */
  assertFrenetRejoinSpline();
  assertCommittedAvoidance(true);
  assertCommittedAvoidance(false);

  /* The reactive controller also has a globally preemptive emergency brake
   * and BACKTRACK. */
  TinyRacerDodgeState dodge_state;
  TinyRacerDodgeIntent dodge_intent;
  const TinyRacerDodgeConfig dodge_config = {
    .trigger_probability = 0.80f,
    .minimum_lateral_rate_mps = 0.80f,
    .maximum_lateral_rate_mps = 1.20f,
    .rejoin_lateral_rate_mps = 0.20f,
    .rejoin_spline_length_m = 0.0f,
    .lateral_acceleration_mps2 = 100.0f,
    .maximum_lateral_offset_m = 0.50f,
    .minimum_lateral_offset_m = 0.20f,
    .emergency_brake_probability = 0.90f,
    .emergency_brake_deceleration_mps2 = 6.00f,
    .backtrack_release_probability = 0.50f,
    .backtrack_speed_mps = 0.10f,
    .backtrack_settle_maximum_tilt_rad = 0.35f,
    .backtrack_settle_maximum_forward_speed_mps = 0.20f,
    .backtrack_settle_maximum_lateral_speed_mps = 0.20f,
    .backtrack_settle_maximum_vertical_speed_mps = 0.20f,
    .backtrack_settle_maximum_body_rate_rad_s = 0.50f,
    .recovery_forward_acceleration_mps2 = 0.50f,
    .avoid_forward_speed_mps = 0.35f,
    .hold_forward_speed_mps = 0.30f,
    .rejoin_forward_speed_mps = 0.20f,
    .rearm_forward_speed_mps = 0.25f,
    .redirect_forward_speed_mps = 0.10f,
    .hold_release_probability = 0.50f,
    .minimum_hold_forward_progress_m = 0.60f,
    .minimum_rearm_forward_progress_m = 0.40f,
    .rejoin_lateral_tolerance_m = 0.10f,
    .rejoin_lateral_speed_tolerance_mps = 0.10f,
    .redirect_maximum_tilt_rad = 0.35f,
    .redirect_maximum_forward_speed_mps = 0.50f,
    .redirect_maximum_lateral_speed_mps = 0.20f,
    .redirect_minimum_vertical_speed_mps = -0.20f,
    .trigger_samples_required = 1,
    .redirect_trigger_samples_required = 2,
    .redirect_settle_samples_required = 2,
    .backtrack_settle_samples_required = 2,
    .hold_clear_samples_required = 2,
    .rejoin_settle_samples_required = 2,
    .rearm_clear_samples_required = 2,
    .rejoin_alignment_heading_tolerance_rad = 0.20f,
    .rejoin_alignment_clear_samples_required = 2,
    .loop_scan_yaw_rad = 0.78539816f,
    .loop_scan_samples_required = 2,
    .loop_escape_lateral_step_m = 0.40f,
    .loop_escape_maximum_offset_m = 0.80f,
    .loop_escape_spline_length_m = 0.60f,
  };
  TinyRacerDodgeFeedback dodge_feedback = {
    .valid = true,
    .forward_progress_m = 0.0f,
    .lateral_offset_m = 0.0f,
    .lateral_speed_mps = 0.0f,
    .forward_speed_mps = 0.40f,
    .vertical_speed_mps = 0.0f,
    .tilt_rad = 0.0f,
  };

  /* A genuinely ambiguous IMAV22 observation uses the circle's inward tie
   * preference, and that fused preference must survive into the dodge latch. */
  TinyRacerPerceptionObservation boundary_observation;
  memset(&boundary_observation, 0, sizeof(boundary_observation));
  boundary_observation.valid = true;
  boundary_observation.has_navigation_command = true;
  boundary_observation.has_sector_danger = true;
  boundary_observation.sample = 1;
  boundary_observation.collision_probability = 0.830f;
  boundary_observation.danger_probability[TINYRACER_DANGER_LEFT] = 0.830f;
  boundary_observation.danger_probability[TINYRACER_DANGER_CENTER] = 0.784f;
  boundary_observation.danger_probability[TINYRACER_DANGER_RIGHT] = 0.700f;
  tinyRacerNavigationFuseSectorRisk(
      &boundary_observation, 0.80f, 0.15f, 1);
  assert(boundary_observation.steering_command > 0.0f);
  tinyRacerNavigationReset(&navigation_state);
  tinyRacerNavigationUpdate(
      &navigation_state, &boundary_observation, &navigation_config,
      0.02f, 0.0f, &navigation_intent);
  assert(navigation_intent.yaw_rate_rad_s > 0.0f);
  tinyRacerDodgeReset(&dodge_state);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.02f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);

  /* A strongly sided obstacle outside the ambiguity band still overrides the
   * inward preference and selects the genuinely clearer right side. */
  boundary_observation.sample++;
  boundary_observation.collision_probability = 0.90f;
  boundary_observation.danger_probability[TINYRACER_DANGER_LEFT] = 0.90f;
  boundary_observation.danger_probability[TINYRACER_DANGER_CENTER] = 0.70f;
  boundary_observation.danger_probability[TINYRACER_DANGER_RIGHT] = 0.30f;
  tinyRacerNavigationFuseSectorRisk(
      &boundary_observation, 0.80f, 0.15f, 1);
  assert(boundary_observation.steering_command < 0.0f);
  tinyRacerNavigationUpdate(
      &navigation_state, &boundary_observation, &navigation_config,
      0.02f, 0.0f, &navigation_intent);
  assert(navigation_intent.yaw_rate_rad_s < 0.0f);
  tinyRacerDodgeReset(&dodge_state);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.02f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_RIGHT);

  tinyRacerDodgeReset(&dodge_state);
  memset(&navigation_intent, 0, sizeof(navigation_intent));
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.forward_speed_mps = 0.4f;
  navigation_intent.right_collision_probability = 0.8f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);

  /* A sector crossing is already the product of ESPNet's two-frame input and
   * commits immediately even when center-only events retain an extra debounce. */
  TinyRacerDodgeConfig debounced_trigger_config = dodge_config;
  debounced_trigger_config.trigger_samples_required = 2u;
  tinyRacerDodgeReset(&dodge_state);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &debounced_trigger_config,
      &dodge_feedback, 0.1f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);
  tinyRacerDodgeReset(&dodge_state);
  navigation_intent.right_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.80f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &debounced_trigger_config,
      &dodge_feedback, 0.1f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &debounced_trigger_config,
      &dodge_feedback, 0.1f, &dodge_intent);
  assert(dodge_intent.phase != TINYRACER_DODGE_TRACK);
  navigation_intent.center_collision_probability = 0.0f;
  navigation_intent.right_collision_probability = 0.80f;

  /* The same observation at cruise speed first enters the existing settle
   * phase. It must decelerate without lateral motion before committing the
   * selected side. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_feedback.forward_speed_mps = 0.80f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m) < 1.0e-6f);
  assert(fabsf(dodge_intent.forward_speed_mps - 0.10f) < 1.0e-6f);
  dodge_feedback.forward_speed_mps = 0.40f;
  dodge_feedback.body_rate_rad_s = 1.0f;
  for (int step = 0; step < 3; ++step) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT);
  }
  dodge_feedback.body_rate_rad_s = 0.0f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_REDIRECT_PREP_LEFT);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);

  /* Recovery must ramp continuously to the full TRACK request. Releasing the
   * limiter at the lower avoidance cap would create a 0.5 -> 1.5 m/s jump. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_TRACK;
  dodge_state.recovery_forward_speed_active = true;
  dodge_state.recovery_forward_speed_mps = 0.45f;
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.0f;
  navigation_intent.right_collision_probability = 0.0f;
  navigation_intent.forward_speed_mps = 1.50f;
  float previous_recovery_speed_mps = 0.45f;
  for (int step = 0; step < 21; ++step) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.forward_speed_mps >=
           previous_recovery_speed_mps - 1.0e-6f);
    assert(dodge_intent.forward_speed_mps <=
           previous_recovery_speed_mps + 0.050001f);
    previous_recovery_speed_mps = dodge_intent.forward_speed_mps;
    if (step < 20) {
      assert(dodge_state.recovery_forward_speed_active);
    }
  }
  assert(fabsf(dodge_intent.forward_speed_mps - 1.50f) < 1.0e-6f);
  assert(!dodge_state.recovery_forward_speed_active);

  tinyRacerDodgeReset(&dodge_state);
  navigation_intent.forward_speed_mps = 0.4f;
  navigation_intent.right_collision_probability = 0.8f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);

  /* Side-only risk and the exact center threshold do not brake. */
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.90f;
  navigation_intent.right_collision_probability = 0.95f;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_AVOID_LEFT);
  const float active_offset_before_emergency = dodge_intent.lateral_offset_m;
  navigation_intent.forward_speed_mps = 0.8f;
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.91f;
  navigation_intent.right_collision_probability = 0.0f;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m -
               active_offset_before_emergency) < 1.0e-6f);

  /* Emergency braking is symmetric and may interrupt either committed
   * outward leg; the repeated-cycle camera scan provides loop recovery. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_AVOID_RIGHT;
  dodge_state.lateral_offset_m = -0.10f;
  dodge_state.lateral_rate_mps = -0.20f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m + 0.10f) < 1.0e-6f);

  /* A settled, low-speed redirect does not need the aggressive cached pitch
   * maneuver. High center risk must retreat directly instead. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_REDIRECT_PREP_LEFT;
  dodge_state.lateral_offset_m = 0.10f;
  dodge_feedback.forward_speed_mps = 0.10f;
  dodge_feedback.lateral_speed_mps = 0.0f;
  dodge_feedback.vertical_speed_mps = 0.0f;
  dodge_feedback.tilt_rad = 0.0f;
  dodge_feedback.body_rate_rad_s = 0.0f;
  navigation_intent.center_collision_probability = 0.96f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  assert(fabsf(dodge_intent.forward_speed_mps + 0.10f) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m - 0.10f) < 1.0e-6f);
  dodge_feedback.forward_speed_mps = 0.40f;

  /* Every maneuver phase is preemptible, and braking preserves the current
   * lane instead of snapping the lateral reference back to center. */
  const TinyRacerDodgePhase preemptible_phases[] = {
    TINYRACER_DODGE_AVOID_LEFT,
    TINYRACER_DODGE_AVOID_RIGHT,
    TINYRACER_DODGE_HOLD_LEFT,
    TINYRACER_DODGE_HOLD_RIGHT,
    TINYRACER_DODGE_REJOIN_ALIGN_LEFT,
    TINYRACER_DODGE_REJOIN_ALIGN_RIGHT,
    TINYRACER_DODGE_REJOIN_LEFT,
    TINYRACER_DODGE_REJOIN_RIGHT,
    TINYRACER_DODGE_REARM_LEFT,
    TINYRACER_DODGE_REARM_RIGHT,
    TINYRACER_DODGE_REDIRECT_PREP_LEFT,
    TINYRACER_DODGE_REDIRECT_PREP_RIGHT,
  };
  for (size_t phase_index = 0;
       phase_index < sizeof(preemptible_phases) /
           sizeof(preemptible_phases[0]);
       ++phase_index) {
    tinyRacerDodgeReset(&dodge_state);
    dodge_state.phase = preemptible_phases[phase_index];
    dodge_state.lateral_offset_m =
        (phase_index & 1u) == 0u ? 0.30f : -0.30f;
    dodge_state.lateral_rate_mps =
        (phase_index & 1u) == 0u ? 0.20f : -0.20f;
    navigation_intent.active = true;
    navigation_intent.new_sample = true;
    navigation_intent.forward_speed_mps = 0.8f;
    navigation_intent.left_collision_probability = 0.0f;
    navigation_intent.center_collision_probability = 0.96f;
    navigation_intent.right_collision_probability = 0.0f;
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
    assert(fabsf(dodge_intent.lateral_offset_m -
                 ((phase_index & 1u) == 0u ? 0.30f : -0.30f)) < 1.0e-6f);
    assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  }

  /* Persistent risk cannot release BACKTRACK or arm a scan. Retreat continues
   * until a complete five-sample moving-risk window is below the threshold. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_BACKTRACK;
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.left_collision_probability = 0.96f;
  navigation_intent.center_collision_probability = 0.96f;
  navigation_intent.right_collision_probability = 0.70f;
  dodge_feedback.forward_speed_mps = 0.10f;
  for (uint8_t sample = 0u;
       sample < 4u * (TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES +
                      dodge_config.backtrack_settle_samples_required);
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
    assert(!dodge_state.loop_scan_pending);
    assert(fabsf(dodge_intent.forward_speed_mps + 0.10f) < 1.0e-6f);
  }
  assert(dodge_state.backtrack_risk_window_count ==
         TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES);
  assert(fabsf(dodge_state.backtrack_risk_average - 0.96f) < 1.0e-6f);

  /* Three repeated brake/backtrack entries arm a stationary left/right camera
   * scan. The third retreat must still obtain a clear five-frame average
   * before settling and starting the scan. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.lateral_offset_m = 0.20f;
  dodge_feedback.forward_speed_mps = 0.40f;
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.forward_speed_mps = 0.8f;
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.96f;
  navigation_intent.right_collision_probability = 0.0f;
  for (uint8_t cycle = 1u; cycle <= 2u; ++cycle) {
    dodge_state.phase = TINYRACER_DODGE_HOLD_LEFT;
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
    assert(dodge_state.backtrack_cycle_count == cycle);
  }
  dodge_state.phase = TINYRACER_DODGE_HOLD_LEFT;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(dodge_state.backtrack_cycle_count == 0u);
  assert(dodge_state.loop_scan_pending);

  dodge_feedback.forward_speed_mps = 0.10f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  assert(dodge_state.loop_scan_pending);

  /* A pending scan does not override sustained high retreat risk. */
  for (uint8_t sample = 0u; sample < 10u; ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
    assert(dodge_state.loop_scan_pending);
  }

  navigation_intent.left_collision_probability = 0.20f;
  navigation_intent.center_collision_probability = 0.20f;
  navigation_intent.right_collision_probability = 0.20f;
  for (uint8_t sample = 0u;
       sample < 2u * TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES &&
           dodge_state.phase == TINYRACER_DODGE_BACKTRACK;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE);
  assert(dodge_state.loop_scan_pending);
  assert(dodge_state.backtrack_risk_average <
         dodge_config.backtrack_release_probability);
  for (uint8_t sample = 0u;
       sample <= dodge_config.backtrack_settle_samples_required &&
           dodge_state.phase == TINYRACER_DODGE_BACKTRACK_SETTLE;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT);
  assert(!dodge_state.loop_scan_pending);
  assert(dodge_state.loop_scan_samples == 0u);

  dodge_feedback.loop_scan_heading_error_rad = 0.0f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_LEFT);
  assert(dodge_state.loop_scan_samples == 1u);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD);
  assert(fabsf(dodge_state.loop_scan_left_risk - 0.20f) < 1.0e-6f);
  assert(dodge_state.loop_escape_yaw_direction == 1);
  assert(dodge_state.loop_escape_active);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m - 0.20f) < 1.0e-6f);
  assert(dodge_intent.forward_speed_mps > 0.0f);
  assert(dodge_intent.forward_speed_mps <=
         dodge_config.avoid_forward_speed_mps);

  /* The scan route remains one continuous spline until its endpoint, then
   * returns directly to TRACK without a lateral avoid/rejoin state. */
  dodge_feedback.forward_progress_m =
      dodge_state.encounter_start_forward_progress_m + 0.70f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  assert(!dodge_state.loop_escape_active);

  /* If left is blocked but right is clear, retain the right scan yaw and take
   * that route forward. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_LOOP_SCAN_LEFT;
  dodge_state.lateral_offset_m = 0.20f;
  dodge_feedback.forward_progress_m = 0.0f;
  navigation_intent.center_collision_probability = 0.70f;
  for (uint8_t sample = 0u;
       sample < dodge_config.loop_scan_samples_required;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT);
  navigation_intent.center_collision_probability = 0.20f;
  for (uint8_t sample = 0u;
       sample < dodge_config.loop_scan_samples_required;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_ESCAPE_FORWARD);
  assert(dodge_state.loop_escape_yaw_direction == -1);
  assert(fabsf(dodge_intent.lateral_offset_m - 0.20f) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);

  /* If neither yawed center is clear, retreat farther and rescan. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_LOOP_SCAN_LEFT;
  navigation_intent.center_collision_probability = 0.70f;
  for (uint8_t sample = 0u;
       sample < dodge_config.loop_scan_samples_required;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_LOOP_SCAN_RIGHT);
  navigation_intent.center_collision_probability = 0.70f;
  for (uint8_t sample = 0u;
       sample < dodge_config.loop_scan_samples_required;
       ++sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
  }
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  assert(dodge_state.loop_scan_pending);
  assert(!dodge_state.loop_escape_active);
  assert(dodge_state.loop_escape_yaw_direction == 0);
  assert(fabsf(dodge_intent.forward_speed_mps + 0.10f) < 1.0e-6f);
  dodge_feedback.forward_speed_mps = 0.40f;

  /* With the flight acceleration limit, emergency braking slews an active
   * lateral rate toward zero instead of introducing a simultaneous lateral
   * command step alongside the 6 m/s^2 longitudinal stop. */
  TinyRacerDodgeConfig slewed_brake_config = dodge_config;
  slewed_brake_config.lateral_acceleration_mps2 = 4.0f;
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_HOLD_LEFT;
  dodge_state.lateral_offset_m = 0.10f;
  dodge_state.lateral_rate_mps = 1.0f;
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.forward_speed_mps = 0.8f;
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.96f;
  navigation_intent.right_collision_probability = 0.0f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &slewed_brake_config,
      &dodge_feedback, 0.02f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(fabsf(dodge_state.lateral_rate_mps - 0.92f) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_rate_mps - 0.92f) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m - 0.1184f) < 1.0e-6f);

  /* A sub-emergency obstacle rediscovered during rejoin pauses the merge on
   * the current bypass lane; it must not launch another outward avoidance. */
  TinyRacerDodgeConfig immediate_rejoin_config = dodge_config;
  immediate_rejoin_config.redirect_trigger_samples_required = 1;
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_REJOIN_RIGHT;
  dodge_state.lateral_offset_m = -0.30f;
  dodge_state.lateral_rate_mps = 0.20f;
  dodge_state.avoidance_target_offset_m = 0.40f;
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.forward_speed_mps = 0.20f;
  navigation_intent.left_collision_probability = 0.10f;
  navigation_intent.center_collision_probability = 0.90f;
  navigation_intent.right_collision_probability = 0.85f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &immediate_rejoin_config,
      &dodge_feedback, 0.001f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_HOLD_RIGHT);
  assert(fabsf(dodge_state.avoidance_target_offset_m - 0.30f) < 1.0e-6f);
  assert(!dodge_state.rejoin_spline_active);

  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_REJOIN_LEFT;
  dodge_state.lateral_offset_m = 0.30f;
  dodge_state.lateral_rate_mps = -0.20f;
  dodge_state.avoidance_target_offset_m = 0.40f;
  navigation_intent.left_collision_probability = 0.85f;
  navigation_intent.right_collision_probability = 0.10f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &immediate_rejoin_config,
      &dodge_feedback, 0.001f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_HOLD_LEFT);
  assert(fabsf(dodge_state.avoidance_target_offset_m - 0.30f) < 1.0e-6f);
  assert(!dodge_state.rejoin_spline_active);

  /* Retreat clearance is symmetric, measured-state gated, persistent, and
   * acceleration-bounded on its eventual return to the curved route. */
  assertBacktrackSettleRecovery(&dodge_config, true, true);
  assertBacktrackSettleRecovery(&dodge_config, true, false);
  assertBacktrackSettleRecovery(&dodge_config, false, true);
  assertBacktrackSettleRecovery(&dodge_config, false, false);

  /* Recovery uses the ordinary rejoin tolerance as its centered criterion;
   * a tiny floating-point offset must not create a zero-width bypass loop. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_state.phase = TINYRACER_DODGE_BACKTRACK_SETTLE;
  dodge_state.lateral_offset_m = -0.005f;
  dodge_state.backtrack_settle_samples =
      dodge_config.backtrack_settle_samples_required - 1u;
  memset(&navigation_intent, 0, sizeof(navigation_intent));
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  navigation_intent.left_collision_probability = 0.40f;
  navigation_intent.center_collision_probability = 0.40f;
  navigation_intent.right_collision_probability = 0.40f;
  memset(&dodge_feedback, 0, sizeof(dodge_feedback));
  dodge_feedback.valid = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
      0.10f, &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);

  /* From TRACK, center risk strictly above 0.90 also triggers braking. The brake
   * removes lateral action and reduces forward speed by exactly 6.0 m/s^2. */
  tinyRacerDodgeReset(&dodge_state);
  dodge_feedback.forward_speed_mps = 0.40f;
  navigation_intent.forward_speed_mps = 0.8f;
  navigation_intent.left_collision_probability = 0.0f;
  navigation_intent.right_collision_probability = 0.0f;
  navigation_intent.center_collision_probability = 0.91f;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(fabsf(dodge_intent.forward_speed_mps - 0.20f) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_rate_mps) < 1.0e-6f);
  assert(fabsf(dodge_intent.lateral_offset_m) < 1.0e-6f);
  navigation_intent.active = false;
  navigation_intent.new_sample = false;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  assert(fabsf(dodge_intent.forward_speed_mps) < 1.0e-6f);
  navigation_intent.active = true;
  navigation_intent.new_sample = true;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_EMERGENCY_BRAKE);
  dodge_feedback.forward_speed_mps = 0.10f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  assert(fabsf(dodge_intent.forward_speed_mps + 0.10f) < 1.0e-6f);
  navigation_intent.new_sample = false;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  navigation_intent.new_sample = true;
  navigation_intent.center_collision_probability = 0.50f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  navigation_intent.center_collision_probability = 0.49f;
  for (int clear_sample = 0; clear_sample < 3; ++clear_sample) {
    tinyRacerDodgeUpdate(
        &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback,
        0.1f, &dodge_intent);
    assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK);
  }
  /* The next clear sample displaces the original 0.96 emergency frame, so
   * the exact five-frame window becomes (0.50 + 4 * 0.49) / 5 = 0.492. */
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE);
  assert(fabsf(dodge_state.backtrack_risk_average - 0.492f) < 1.0e-6f);
  assert(fabsf(dodge_intent.forward_speed_mps) < 1.0e-6f);
  dodge_feedback.forward_speed_mps = 0.20f;
  dodge_feedback.lateral_speed_mps = 0.0f;
  dodge_feedback.vertical_speed_mps = 0.0f;
  dodge_feedback.tilt_rad = 0.0f;
  dodge_feedback.body_rate_rad_s = 0.0f;
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_BACKTRACK_SETTLE);
  tinyRacerDodgeUpdate(
      &dodge_state, &navigation_intent, &dodge_config, &dodge_feedback, 0.1f,
      &dodge_intent);
  assert(dodge_intent.phase == TINYRACER_DODGE_TRACK);
  assert(fabsf(dodge_intent.forward_speed_mps - 0.05f) < 1.0e-6f);

  /* A combined navigation+gate head must not let gate centering fight an
   * active avoidance reference. */
  assert(tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_TRACK, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_AVOID_LEFT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_AVOID_RIGHT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_HOLD_LEFT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_HOLD_RIGHT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REJOIN_LEFT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REJOIN_RIGHT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REARM_LEFT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REARM_RIGHT, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REDIRECT_PREP_LEFT,
      false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_REDIRECT_PREP_RIGHT,
      false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_EMERGENCY_BRAKE, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_BACKTRACK, false, false));
  assert(!tinyRacerGateServoAllowed(
      TINYRACER_RACE_TRACK, TINYRACER_DODGE_BACKTRACK_SETTLE,
      false, false));
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
