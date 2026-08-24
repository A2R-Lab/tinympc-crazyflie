#ifndef TINYMPC_FLIP_PRIMITIVE_H
#define TINYMPC_FLIP_PRIMITIVE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tinympc_progress_yaw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TINYMPC_FLIP_WAITING = 0,
  TINYMPC_FLIP_ACTIVE = 1,
  TINYMPC_FLIP_COMPLETE = 2,
  TINYMPC_FLIP_FAULT = 3,
} TinyMpcFlipMode;

typedef struct {
  float sigma;
  float motor_thrust_n[4];
} TinyMpcFlipInputKnot;

typedef struct {
  float trigger_start_s_m;
  float trigger_end_s_m;
  float duration_s;
  int8_t pitch_direction;
  const TinyMpcFlipInputKnot *input_knots;
  uint8_t input_knot_count;
  const int16_t *phase_model_ids;
  uint8_t phase_model_count;
} TinyMpcFlipConfig;

typedef struct {
  TinyMpcFlipMode mode;
  float sigma;
  float previous_s_m;
  bool previous_s_valid;
  uint16_t trigger_count;
} TinyMpcFlipState;

typedef struct {
  TinyMpcProgressQuaternion attitude_world_body;
  TinyMpcProgressBodyRate body_rate_rad_s;
  float motor_thrust_n[4];
  float pitch_rad;
  float pitch_rate_rad_s;
  int16_t phase_model_id;
  bool valid;
} TinyMpcFlipSample;

static inline bool tinyMpcFlipConfigValid(const TinyMpcFlipConfig *config) {
  if (config == NULL || !isfinite(config->trigger_start_s_m)
      || !isfinite(config->trigger_end_s_m)
      || !isfinite(config->duration_s)
      || config->trigger_start_s_m < 0.0f
      || config->trigger_end_s_m <= config->trigger_start_s_m
      || config->duration_s <= 0.0f
      || (config->pitch_direction != -1 && config->pitch_direction != 1)
      || config->input_knots == NULL || config->input_knot_count < 2u
      || (config->phase_model_count > 0u
          && config->phase_model_ids == NULL)) {
    return false;
  }
  float previous_sigma = -1.0f;
  for (uint8_t knot = 0u; knot < config->input_knot_count; ++knot) {
    const TinyMpcFlipInputKnot *input = &config->input_knots[knot];
    if (!isfinite(input->sigma) || input->sigma <= previous_sigma) {
      return false;
    }
    for (uint8_t motor = 0u; motor < 4u; ++motor) {
      if (!isfinite(input->motor_thrust_n[motor])
          || input->motor_thrust_n[motor] < 0.0f) {
        return false;
      }
    }
    previous_sigma = input->sigma;
  }
  return fabsf(config->input_knots[0].sigma) <= 1.0e-6f
      && fabsf(config->input_knots[config->input_knot_count - 1u].sigma
          - 1.0f) <= 1.0e-6f;
}

static inline void tinyMpcFlipReset(TinyMpcFlipState *state) {
  if (state == NULL) {
    return;
  }
  state->mode = TINYMPC_FLIP_WAITING;
  state->sigma = 0.0f;
  state->previous_s_m = 0.0f;
  state->previous_s_valid = false;
  state->trigger_count = 0u;
}

/* Course progress triggers the maneuver once. Maneuver phase then evolves on
 * its own clock, so path projection disturbances cannot freeze a vehicle
 * while it is inverted. */
static inline TinyMpcFlipMode tinyMpcFlipUpdate(
    TinyMpcFlipState *state, const TinyMpcFlipConfig *config,
    float measured_s_m, float dt_s, bool allow_trigger,
    bool advance_phase) {
  if (state == NULL || !tinyMpcFlipConfigValid(config)
      || !isfinite(measured_s_m) || !isfinite(dt_s) || dt_s <= 0.0f) {
    if (state != NULL) {
      state->mode = TINYMPC_FLIP_FAULT;
    }
    return TINYMPC_FLIP_FAULT;
  }
  bool entered = false;
  if (state->mode == TINYMPC_FLIP_WAITING && allow_trigger) {
    const bool in_window = measured_s_m >= config->trigger_start_s_m
        && measured_s_m <= config->trigger_end_s_m;
    const bool crossed_start = !state->previous_s_valid
        || state->previous_s_m < config->trigger_start_s_m;
    if (in_window && crossed_start) {
      state->mode = TINYMPC_FLIP_ACTIVE;
      state->sigma = 0.0f;
      ++state->trigger_count;
      entered = true;
    }
  }
  state->previous_s_m = measured_s_m;
  state->previous_s_valid = true;
  if (state->mode == TINYMPC_FLIP_ACTIVE && advance_phase && !entered) {
    state->sigma += dt_s / config->duration_s;
    if (state->sigma >= 1.0f) {
      state->sigma = 1.0f;
      state->mode = TINYMPC_FLIP_COMPLETE;
    }
  }
  return state->mode;
}

static inline float tinyMpcFlipClamp01(float value) {
  return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

/* Do not hand control back to a steady racing model merely because the
 * scheduled phase reached one. Recovery is complete only after the measured
 * thrust axis is upright and the measured body rate is small. */
static inline bool tinyMpcFlipRecoveryReady(
    float body_z_world_z, float body_rate_norm_rad_s,
    float minimum_body_z_world_z, float maximum_body_rate_rad_s) {
  return isfinite(body_z_world_z) && isfinite(body_rate_norm_rad_s)
      && isfinite(minimum_body_z_world_z)
      && isfinite(maximum_body_rate_rad_s)
      && body_z_world_z >= minimum_body_z_world_z
      && body_rate_norm_rad_s <= maximum_body_rate_rad_s;
}

static inline float tinyMpcFlipSmootherStep(float sigma) {
  sigma = tinyMpcFlipClamp01(sigma);
  return sigma * sigma * sigma
      * (10.0f + sigma * (-15.0f + 6.0f * sigma));
}

static inline float tinyMpcFlipSmootherStepDerivative(float sigma) {
  sigma = tinyMpcFlipClamp01(sigma);
  const float one_minus = 1.0f - sigma;
  return 30.0f * sigma * sigma * one_minus * one_minus;
}

static inline void tinyMpcFlipInterpolateInput(
    const TinyMpcFlipConfig *config, float sigma, float output_n[4]) {
  sigma = tinyMpcFlipClamp01(sigma);
  uint8_t lower = 0u;
  while (lower + 1u < config->input_knot_count
      && config->input_knots[lower + 1u].sigma < sigma) {
    ++lower;
  }
  const uint8_t upper = lower + 1u < config->input_knot_count
      ? lower + 1u : lower;
  const float span = config->input_knots[upper].sigma
      - config->input_knots[lower].sigma;
  const float alpha = span > 1.0e-7f
      ? (sigma - config->input_knots[lower].sigma) / span : 0.0f;
  for (uint8_t motor = 0u; motor < 4u; ++motor) {
    output_n[motor] = config->input_knots[lower].motor_thrust_n[motor]
        + alpha * (config->input_knots[upper].motor_thrust_n[motor]
            - config->input_knots[lower].motor_thrust_n[motor]);
  }
}

static inline TinyMpcFlipSample tinyMpcFlipSample(
    const TinyMpcFlipConfig *config, float sigma,
    float course_yaw_rad, float course_yaw_rate_rad_s) {
  TinyMpcFlipSample sample = {
      tinyMpcProgressQuaternionMake(0.0f, 0.0f, 0.0f, 1.0f),
      tinyMpcProgressBodyRateMake(0.0f, 0.0f, 0.0f),
      {0.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, -1, false};
  if (!tinyMpcFlipConfigValid(config) || !isfinite(sigma)
      || !isfinite(course_yaw_rad) || !isfinite(course_yaw_rate_rad_s)) {
    return sample;
  }
  sigma = tinyMpcFlipClamp01(sigma);
  const float direction = (float)config->pitch_direction;
  const float two_pi = 6.28318530717958647692f;
  sample.pitch_rad = direction * two_pi * tinyMpcFlipSmootherStep(sigma);
  sample.pitch_rate_rad_s = direction * two_pi
      * tinyMpcFlipSmootherStepDerivative(sigma) / config->duration_s;
  const float half_yaw = 0.5f * course_yaw_rad;
  const float half_pitch = 0.5f * sample.pitch_rad;
  const float cy = cosf(half_yaw);
  const float sy = sinf(half_yaw);
  const float cp = cosf(half_pitch);
  const float sp = sinf(half_pitch);
  sample.attitude_world_body = tinyMpcProgressQuaternionMake(
      -sy * sp, cy * sp, sy * cp, cy * cp);
  sample.body_rate_rad_s = tinyMpcProgressBodyRateMake(
      -course_yaw_rate_rad_s * sinf(sample.pitch_rad),
      sample.pitch_rate_rad_s,
      course_yaw_rate_rad_s * cosf(sample.pitch_rad));
  tinyMpcFlipInterpolateInput(config, sigma, sample.motor_thrust_n);
  if (config->phase_model_count > 0u) {
    uint8_t phase = (uint8_t)floorf(
        sigma * (float)config->phase_model_count);
    if (phase >= config->phase_model_count) {
      phase = config->phase_model_count - 1u;
    }
    sample.phase_model_id = config->phase_model_ids[phase];
  }
  sample.valid = true;
  return sample;
}

#ifdef __cplusplus
}
#endif

#endif
