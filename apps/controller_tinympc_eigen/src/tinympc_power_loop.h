#ifndef TINYMPC_POWER_LOOP_H
#define TINYMPC_POWER_LOOP_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "tinympc_progress_yaw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TINYMPC_POWER_LOOP_WAITING = 0,
  TINYMPC_POWER_LOOP_ACTIVE = 1,
  TINYMPC_POWER_LOOP_COMPLETE = 2,
  TINYMPC_POWER_LOOP_FAULT = 3,
} TinyMpcPowerLoopMode;

typedef struct {
  float trigger_start_s_m;
  float trigger_end_s_m;
  float radius_m;
  float bottom_speed_mps;
  float top_speed_mps;
  float mass_kg;
  float gravity_mps2;
  float maximum_motor_thrust_n;
  float pitch_inertia_kg_m2;
  float arm_offset_m;
} TinyMpcPowerLoopConfig;

typedef struct {
  TinyMpcPowerLoopMode mode;
  float sigma;
  float previous_s_m;
  bool previous_s_valid;
  bool armed;
  uint16_t trigger_count;
} TinyMpcPowerLoopState;

typedef struct {
  float position_forward_m;
  float position_up_m;
  float velocity_forward_mps;
  float velocity_up_mps;
  float acceleration_forward_mps2;
  float acceleration_up_mps2;
  float tangent_forward;
  float tangent_up;
  float speed_mps;
  float pitch_rad;
  float pitch_rate_rad_s;
  TinyMpcProgressQuaternion attitude_loop;
  TinyMpcProgressBodyRate body_rate_rad_s;
  float motor_thrust_n[4];
  bool valid;
} TinyMpcPowerLoopSample;

static inline float tinyMpcPowerLoopClamp01(float value) {
  return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

static inline bool tinyMpcPowerLoopConfigValid(
    const TinyMpcPowerLoopConfig *config) {
  if (config == NULL || !isfinite(config->trigger_start_s_m)
      || !isfinite(config->trigger_end_s_m)
      || !isfinite(config->radius_m)
      || !isfinite(config->bottom_speed_mps)
      || !isfinite(config->top_speed_mps)
      || !isfinite(config->mass_kg)
      || !isfinite(config->gravity_mps2)
      || !isfinite(config->maximum_motor_thrust_n)
      || !isfinite(config->pitch_inertia_kg_m2)
      || !isfinite(config->arm_offset_m)) {
    return false;
  }
  if (config->trigger_start_s_m < 0.0f
      || config->trigger_end_s_m <= config->trigger_start_s_m
      || config->radius_m <= 0.0f || config->bottom_speed_mps <= 0.0f
      || config->top_speed_mps <= config->bottom_speed_mps
      || config->mass_kg <= 0.0f || config->gravity_mps2 <= 0.0f
      || config->maximum_motor_thrust_n <= 0.0f
      || config->pitch_inertia_kg_m2 <= 0.0f
      || config->arm_offset_m <= 0.0f) {
    return false;
  }
  const float bottom_centripetal = config->bottom_speed_mps
      * config->bottom_speed_mps / config->radius_m;
  const float bottom_motor_thrust = config->mass_kg
      * (config->gravity_mps2 + bottom_centripetal) / 4.0f;
  const float top_centripetal = config->top_speed_mps
      * config->top_speed_mps / config->radius_m;
  return bottom_motor_thrust <= config->maximum_motor_thrust_n
      && top_centripetal > config->gravity_mps2;
}

static inline void tinyMpcPowerLoopReset(TinyMpcPowerLoopState *state) {
  if (state == NULL) {
    return;
  }
  state->mode = TINYMPC_POWER_LOOP_WAITING;
  state->sigma = 0.0f;
  state->previous_s_m = 0.0f;
  state->previous_s_valid = false;
  state->armed = false;
  state->trigger_count = 0u;
}

static inline float tinyMpcPowerLoopSpeed(
    const TinyMpcPowerLoopConfig *config, float sigma) {
  const float alpha = 6.28318530717958647692f
      * tinyMpcPowerLoopClamp01(sigma);
  const float speed_squared_amplitude = 0.5f
      * (config->top_speed_mps * config->top_speed_mps
          - config->bottom_speed_mps * config->bottom_speed_mps);
  return sqrtf(config->bottom_speed_mps * config->bottom_speed_mps
      + speed_squared_amplitude * (1.0f - cosf(alpha)));
}

static inline TinyMpcPowerLoopMode tinyMpcPowerLoopUpdate(
    TinyMpcPowerLoopState *state, const TinyMpcPowerLoopConfig *config,
    float measured_s_m, float dt_s, bool allow_trigger,
    bool entry_ready, bool advance_phase) {
  if (state == NULL || !tinyMpcPowerLoopConfigValid(config)
      || !isfinite(measured_s_m) || !isfinite(dt_s) || dt_s <= 0.0f) {
    if (state != NULL) {
      state->mode = TINYMPC_POWER_LOOP_FAULT;
    }
    return TINYMPC_POWER_LOOP_FAULT;
  }
  bool entered = false;
  if (state->mode == TINYMPC_POWER_LOOP_WAITING && allow_trigger) {
    const bool in_window = measured_s_m >= config->trigger_start_s_m
        && measured_s_m <= config->trigger_end_s_m;
    const bool crossed_start = !state->previous_s_valid
        || state->previous_s_m < config->trigger_start_s_m;
    if (in_window && crossed_start) {
      state->armed = true;
    }
    if (state->armed && entry_ready) {
      state->mode = TINYMPC_POWER_LOOP_ACTIVE;
      state->sigma = 0.0f;
      state->armed = false;
      ++state->trigger_count;
      entered = true;
    }
  }
  state->previous_s_m = measured_s_m;
  state->previous_s_valid = true;
  if (state->mode == TINYMPC_POWER_LOOP_ACTIVE
      && advance_phase && !entered) {
    const float two_pi = 6.28318530717958647692f;
    state->sigma += dt_s * tinyMpcPowerLoopSpeed(config, state->sigma)
        / (two_pi * config->radius_m);
    if (state->sigma >= 1.0f) {
      state->sigma = 1.0f;
      state->mode = TINYMPC_POWER_LOOP_COMPLETE;
    }
  }
  return state->mode;
}

static inline float tinyMpcPowerLoopPitchAtAlpha(
    const TinyMpcPowerLoopConfig *config, float alpha) {
  const float speed_squared_amplitude = 0.5f
      * (config->top_speed_mps * config->top_speed_mps
          - config->bottom_speed_mps * config->bottom_speed_mps);
  const float speed = sqrtf(
      config->bottom_speed_mps * config->bottom_speed_mps
      + speed_squared_amplitude * (1.0f - cosf(alpha)));
  const float tangential_acceleration =
      speed_squared_amplitude * sinf(alpha) / (2.0f * config->radius_m);
  const float centripetal_acceleration = speed * speed / config->radius_m;
  const float tangent_forward = cosf(alpha);
  const float tangent_up = sinf(alpha);
  const float normal_forward = -sinf(alpha);
  const float normal_up = cosf(alpha);
  const float acceleration_forward =
      tangential_acceleration * tangent_forward
      + centripetal_acceleration * normal_forward;
  const float acceleration_up = tangential_acceleration * tangent_up
      + centripetal_acceleration * normal_up;
  return atan2f(
      acceleration_forward, acceleration_up + config->gravity_mps2);
}

static inline float tinyMpcPowerLoopPitchRateAtAlpha(
    const TinyMpcPowerLoopConfig *config, float alpha) {
  const float epsilon = 1.0e-3f;
  const float pitch_before = tinyMpcPowerLoopPitchAtAlpha(
      config, alpha - epsilon);
  const float pitch_after = tinyMpcPowerLoopPitchAtAlpha(
      config, alpha + epsilon);
  const float pitch_delta = remainderf(
      pitch_after - pitch_before, 6.28318530717958647692f);
  const float sigma = alpha / 6.28318530717958647692f;
  const float alpha_rate = tinyMpcPowerLoopSpeed(config, sigma)
      / config->radius_m;
  return pitch_delta / (2.0f * epsilon) * alpha_rate;
}

static inline TinyMpcPowerLoopSample tinyMpcPowerLoopSample(
    const TinyMpcPowerLoopConfig *config, float sigma) {
  TinyMpcPowerLoopSample sample = {
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f,
      tinyMpcProgressQuaternionMake(0.0f, 0.0f, 0.0f, 1.0f),
      tinyMpcProgressBodyRateMake(0.0f, 0.0f, 0.0f),
      {0.0f, 0.0f, 0.0f, 0.0f}, false};
  if (!tinyMpcPowerLoopConfigValid(config) || !isfinite(sigma)) {
    return sample;
  }
  sigma = tinyMpcPowerLoopClamp01(sigma);
  const float two_pi = 6.28318530717958647692f;
  const float alpha = two_pi * sigma;
  const float sine = sinf(alpha);
  const float cosine = cosf(alpha);
  sample.position_forward_m = config->radius_m * sine;
  sample.position_up_m = config->radius_m * (1.0f - cosine);
  sample.tangent_forward = cosine;
  sample.tangent_up = sine;
  sample.speed_mps = tinyMpcPowerLoopSpeed(config, sigma);
  sample.velocity_forward_mps = sample.speed_mps * cosine;
  sample.velocity_up_mps = sample.speed_mps * sine;
  const float speed_squared_amplitude = 0.5f
      * (config->top_speed_mps * config->top_speed_mps
          - config->bottom_speed_mps * config->bottom_speed_mps);
  const float tangential_acceleration =
      speed_squared_amplitude * sine / (2.0f * config->radius_m);
  const float centripetal_acceleration =
      sample.speed_mps * sample.speed_mps / config->radius_m;
  sample.acceleration_forward_mps2 = tangential_acceleration * cosine
      - centripetal_acceleration * sine;
  sample.acceleration_up_mps2 = tangential_acceleration * sine
      + centripetal_acceleration * cosine;
  const float thrust_forward = sample.acceleration_forward_mps2;
  const float thrust_up = sample.acceleration_up_mps2 + config->gravity_mps2;
  const float thrust_acceleration = hypotf(thrust_forward, thrust_up);
  if (thrust_acceleration <= 1.0e-5f) {
    return sample;
  }
  sample.pitch_rad = atan2f(thrust_forward, thrust_up);
  const float half_pitch = 0.5f * sample.pitch_rad;
  sample.attitude_loop = tinyMpcProgressQuaternionMake(
      0.0f, sinf(half_pitch), 0.0f, cosf(half_pitch));
  sample.pitch_rate_rad_s = tinyMpcPowerLoopPitchRateAtAlpha(config, alpha);
  sample.body_rate_rad_s = tinyMpcProgressBodyRateMake(
      0.0f, sample.pitch_rate_rad_s, 0.0f);
  const float rate_epsilon = 2.0e-3f;
  const float pitch_rate_before = tinyMpcPowerLoopPitchRateAtAlpha(
      config, alpha - rate_epsilon);
  const float pitch_rate_after = tinyMpcPowerLoopPitchRateAtAlpha(
      config, alpha + rate_epsilon);
  const float alpha_rate = sample.speed_mps / config->radius_m;
  const float pitch_acceleration =
      (pitch_rate_after - pitch_rate_before) / (2.0f * rate_epsilon)
      * alpha_rate;
  const float pitch_moment_nm =
      config->pitch_inertia_kg_m2 * pitch_acceleration;
  const float pitch_differential_n =
      pitch_moment_nm / (4.0f * config->arm_offset_m);
  const float collective_motor_n =
      config->mass_kg * thrust_acceleration / 4.0f;
  sample.motor_thrust_n[0] = collective_motor_n - pitch_differential_n;
  sample.motor_thrust_n[1] = collective_motor_n + pitch_differential_n;
  sample.motor_thrust_n[2] = sample.motor_thrust_n[1];
  sample.motor_thrust_n[3] = sample.motor_thrust_n[0];
  for (uint8_t motor = 0u; motor < 4u; ++motor) {
    sample.motor_thrust_n[motor] = sample.motor_thrust_n[motor] < 0.0f
        ? 0.0f : (sample.motor_thrust_n[motor]
            > config->maximum_motor_thrust_n
                ? config->maximum_motor_thrust_n
                : sample.motor_thrust_n[motor]);
  }
  sample.valid = true;
  return sample;
}

#ifdef __cplusplus
}
#endif

#endif
