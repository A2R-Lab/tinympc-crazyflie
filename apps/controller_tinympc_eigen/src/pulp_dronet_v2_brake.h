#ifndef PULP_DRONET_V2_BRAKE_H
#define PULP_DRONET_V2_BRAKE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Float transcription of Listing 1 in Niculescu et al., JETCAS 2021.
 * The head-on experiment silences steering, so this component exposes only
 * the normalized forward-speed command consumed by the TinyMPC reference. */
typedef struct {
  float collision_integral;
  float filtered_speed_scale;
  float integral_threshold;
  float integral_maximum;
  float integral_weight;
  float velocity_alpha;
} PulpDronetV2Brake;

static inline PulpDronetV2Brake pulpDronetV2BrakeDefault(void) {
  const PulpDronetV2Brake brake = {
      0.0f, 0.0f, 0.30f, 3.0f, 0.20f, 0.60f};
  return brake;
}

static inline void pulpDronetV2BrakeReset(PulpDronetV2Brake *brake) {
  if (brake == NULL) {
    return;
  }
  brake->collision_integral = 0.0f;
  brake->filtered_speed_scale = 0.0f;
}

static inline bool pulpDronetV2BrakeValid(
    const PulpDronetV2Brake *brake) {
  return brake != NULL && isfinite(brake->collision_integral) &&
      isfinite(brake->filtered_speed_scale) &&
      isfinite(brake->integral_threshold) &&
      isfinite(brake->integral_maximum) &&
      brake->integral_maximum > 0.0f &&
      isfinite(brake->integral_weight) && brake->integral_weight >= 0.0f &&
      isfinite(brake->velocity_alpha) && brake->velocity_alpha >= 0.0f &&
      brake->velocity_alpha <= 1.0f;
}

static inline float pulpDronetV2BrakeStep(
    PulpDronetV2Brake *brake, float collision_probability) {
  if (!pulpDronetV2BrakeValid(brake) ||
      !isfinite(collision_probability)) {
    return 0.0f;
  }
  const float probability = fminf(fmaxf(collision_probability, 0.0f), 1.0f);
  brake->collision_integral = fminf(fmaxf(
      brake->collision_integral + probability - brake->integral_threshold,
      0.0f), brake->integral_maximum);
  /* The paper does not spell out the saturation after integral correction.
   * Bound probability before squaring so wind-up cannot command acceleration. */
  const float corrected = fminf(fmaxf(
      probability + brake->integral_weight * brake->collision_integral,
      0.0f), 1.0f);
  const float unfiltered = (1.0f - corrected) * (1.0f - corrected);
  brake->filtered_speed_scale =
      brake->velocity_alpha * unfiltered +
      (1.0f - brake->velocity_alpha) * brake->filtered_speed_scale;
  return brake->filtered_speed_scale;
}

#endif
