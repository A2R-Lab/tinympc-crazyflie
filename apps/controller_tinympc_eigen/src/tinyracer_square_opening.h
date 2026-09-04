#ifndef TINYRACER_SQUARE_OPENING_H
#define TINYRACER_SQUARE_OPENING_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float probability_threshold;
  float maximum_translation_speed_mps;
  float maximum_abs_roll_rad;
  float maximum_abs_pitch_rad;
  float minimum_abs_yaw_rate_rad_s;
  float maximum_abs_yaw_rate_rad_s;
  uint8_t confirmation_frames;
} TinyRacerSquareOpeningConfig;

typedef struct {
  uint8_t consecutive_positive_frames;
  bool trigger_eligible;
  bool square_opening_seen;
} TinyRacerSquareOpeningState;

static inline TinyRacerSquareOpeningConfig tinyRacerSquareOpeningDefaultConfig(
    float probability_threshold) {
  const TinyRacerSquareOpeningConfig config = {
      probability_threshold, 0.08f, 0.25f, 0.25f, 0.05f, 1.50f, 2u};
  return config;
}

static inline void tinyRacerSquareOpeningReset(
    TinyRacerSquareOpeningState *state) {
  if (state == NULL) {
    return;
  }
  state->consecutive_positive_frames = 0u;
  state->trigger_eligible = false;
  state->square_opening_seen = false;
}

static inline bool tinyRacerSquareOpeningConfigValid(
    const TinyRacerSquareOpeningConfig *config) {
  return config != NULL && isfinite(config->probability_threshold) &&
      config->probability_threshold > 0.0f &&
      config->probability_threshold < 1.0f &&
      isfinite(config->maximum_translation_speed_mps) &&
      config->maximum_translation_speed_mps >= 0.0f &&
      isfinite(config->maximum_abs_roll_rad) &&
      config->maximum_abs_roll_rad >= 0.0f &&
      isfinite(config->maximum_abs_pitch_rad) &&
      config->maximum_abs_pitch_rad >= 0.0f &&
      isfinite(config->minimum_abs_yaw_rate_rad_s) &&
      config->minimum_abs_yaw_rate_rad_s >= 0.0f &&
      isfinite(config->maximum_abs_yaw_rate_rad_s) &&
      config->maximum_abs_yaw_rate_rad_s >=
          config->minimum_abs_yaw_rate_rad_s &&
      config->confirmation_frames > 0u;
}

/* Update only counts newly arrived camera frames. A temporarily non-new frame
 * holds the candidate count; a stale stream or fresh negative resets it. The
 * advisory latch persists until TURN_SCAN is exited. */
static inline void tinyRacerSquareOpeningUpdate(
    TinyRacerSquareOpeningState *state,
    const TinyRacerSquareOpeningConfig *config,
    bool in_turn_scan, bool collision_stop_context,
    bool stream_fresh, bool new_frame, bool has_square_opening,
    float probability, float translation_speed_mps,
    float roll_rad, float pitch_rad, float yaw_rate_rad_s) {
  if (state == NULL || !tinyRacerSquareOpeningConfigValid(config)) {
    return;
  }
  if (!in_turn_scan) {
    tinyRacerSquareOpeningReset(state);
    return;
  }

  const bool kinematics_valid =
      isfinite(translation_speed_mps) && translation_speed_mps >= 0.0f &&
      isfinite(roll_rad) && isfinite(pitch_rad) &&
      isfinite(yaw_rate_rad_s);
  const float abs_yaw_rate_rad_s = fabsf(yaw_rate_rad_s);
  state->trigger_eligible = collision_stop_context && stream_fresh &&
      has_square_opening && kinematics_valid &&
      translation_speed_mps <= config->maximum_translation_speed_mps &&
      fabsf(roll_rad) <= config->maximum_abs_roll_rad &&
      fabsf(pitch_rad) <= config->maximum_abs_pitch_rad &&
      abs_yaw_rate_rad_s >= config->minimum_abs_yaw_rate_rad_s &&
      abs_yaw_rate_rad_s <= config->maximum_abs_yaw_rate_rad_s;

  if (!stream_fresh) {
    state->consecutive_positive_frames = 0u;
    return;
  }
  if (!new_frame) {
    return;
  }
  if (!state->trigger_eligible || !isfinite(probability) ||
      probability < config->probability_threshold) {
    state->consecutive_positive_frames = 0u;
    return;
  }
  if (state->consecutive_positive_frames < UINT8_MAX) {
    ++state->consecutive_positive_frames;
  }
  if (state->consecutive_positive_frames >= config->confirmation_frames) {
    state->square_opening_seen = true;
  }
}

#ifdef __cplusplus
}
#endif

#endif
