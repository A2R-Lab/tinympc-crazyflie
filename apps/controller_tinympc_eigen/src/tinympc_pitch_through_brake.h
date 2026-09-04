#ifndef TINYMPC_PITCH_THROUGH_BRAKE_H
#define TINYMPC_PITCH_THROUGH_BRAKE_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Moving-entry, minimum-forward-excursion emergency reference.
 *
 * This is deliberately not a loop and it does not use a cached hover model.
 * While forward velocity is positive it slews toward a speed-indexed braking
 * pitch and spends all available thrust, retaining only the differential
 * needed to produce the requested pitch acceleration.  Once forward velocity
 * crosses zero, the reference recovers attitude and altitude while allowing a
 * temporary backwards drift.  That makes the closest approach independent of
 * the later hover recovery.
 */
typedef struct {
  float minimum_braking_pitch_rad;
  float maximum_braking_pitch_rad;
  float minimum_pitch_speed_mps;
  float maximum_pitch_speed_mps;
  float maximum_pitch_rate_rad_s;
  float maximum_pitch_acceleration_rad_s2;
  float forward_release_speed_mps;
  float recovery_forward_gain_per_s;
  float recovery_maximum_acceleration_mps2;
  float recovery_altitude_gain_per_s2;
  float recovery_vertical_damping_per_s;
  float recovery_maximum_vertical_acceleration_mps2;
  float maximum_altitude_loss_m;
  float altitude_reserve_m;
  float signed_drag_n_per_mps;
  float mass_kg;
  float gravity_mps2;
  float maximum_motor_thrust_n;
  float pitch_inertia_kg_m2;
  float pitch_arm_m;
} TinyMpcPitchThroughBrakeConfig;

typedef struct {
  float forward_position_m;
  float up_position_m;
  float forward_speed_mps;
  float up_speed_mps;
  float pitch_rad;
  float pitch_rate_rad_s;
  float altitude_loss_at_origin_m;
} TinyMpcPitchThroughBrakeState;

typedef struct {
  TinyMpcPitchThroughBrakeState state;
  float target_pitch_rad;
  float forward_acceleration_mps2;
  float up_acceleration_mps2;
  float motor_thrust_n[4];
  bool braking;
  bool altitude_limited;
  bool valid;
} TinyMpcPitchThroughBrakeSample;

static inline float tinyMpcPitchThroughClamp(
    float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

static inline bool tinyMpcPitchThroughConfigValid(
    const TinyMpcPitchThroughBrakeConfig *config) {
  return config != NULL &&
      isfinite(config->minimum_braking_pitch_rad) &&
      isfinite(config->maximum_braking_pitch_rad) &&
      isfinite(config->minimum_pitch_speed_mps) &&
      isfinite(config->maximum_pitch_speed_mps) &&
      isfinite(config->maximum_pitch_rate_rad_s) &&
      isfinite(config->maximum_pitch_acceleration_rad_s2) &&
      isfinite(config->forward_release_speed_mps) &&
      isfinite(config->recovery_forward_gain_per_s) &&
      isfinite(config->recovery_maximum_acceleration_mps2) &&
      isfinite(config->recovery_altitude_gain_per_s2) &&
      isfinite(config->recovery_vertical_damping_per_s) &&
      isfinite(config->recovery_maximum_vertical_acceleration_mps2) &&
      isfinite(config->maximum_altitude_loss_m) &&
      isfinite(config->altitude_reserve_m) &&
      isfinite(config->signed_drag_n_per_mps) &&
      isfinite(config->mass_kg) && isfinite(config->gravity_mps2) &&
      isfinite(config->maximum_motor_thrust_n) &&
      isfinite(config->pitch_inertia_kg_m2) &&
      isfinite(config->pitch_arm_m) &&
      config->minimum_braking_pitch_rad > 0.0f &&
      config->maximum_braking_pitch_rad >=
          config->minimum_braking_pitch_rad &&
      config->maximum_pitch_speed_mps > config->minimum_pitch_speed_mps &&
      config->maximum_pitch_rate_rad_s > 0.0f &&
      config->maximum_pitch_acceleration_rad_s2 > 0.0f &&
      config->forward_release_speed_mps >= 0.0f &&
      config->recovery_forward_gain_per_s > 0.0f &&
      config->recovery_maximum_acceleration_mps2 > 0.0f &&
      config->recovery_altitude_gain_per_s2 > 0.0f &&
      config->recovery_vertical_damping_per_s > 0.0f &&
      config->recovery_maximum_vertical_acceleration_mps2 > 0.0f &&
      config->maximum_altitude_loss_m > config->altitude_reserve_m &&
      config->altitude_reserve_m >= 0.0f && config->mass_kg > 0.0f &&
      config->gravity_mps2 > 0.0f &&
      config->maximum_motor_thrust_n > 0.0f &&
      config->pitch_inertia_kg_m2 > 0.0f && config->pitch_arm_m > 0.0f;
}

static inline float tinyMpcPitchThroughSpeedPitchMagnitude(
    const TinyMpcPitchThroughBrakeConfig *config,
    float forward_speed_mps) {
  const float fraction = tinyMpcPitchThroughClamp(
      (forward_speed_mps - config->minimum_pitch_speed_mps) /
          (config->maximum_pitch_speed_mps -
              config->minimum_pitch_speed_mps),
      0.0f, 1.0f);
  return config->minimum_braking_pitch_rad + fraction *
      (config->maximum_braking_pitch_rad -
          config->minimum_braking_pitch_rad);
}

static inline float tinyMpcPitchThroughBrakingPitch(
    const TinyMpcPitchThroughBrakeConfig *config,
    float forward_speed_mps, float up_speed_mps,
    float total_altitude_loss_m, bool *altitude_limited) {
  if (altitude_limited != NULL) {
    *altitude_limited = false;
  }
  if (!tinyMpcPitchThroughConfigValid(config) ||
      !isfinite(forward_speed_mps) || !isfinite(up_speed_mps) ||
      !isfinite(total_altitude_loss_m)) {
    return 0.0f;
  }
  float magnitude = tinyMpcPitchThroughSpeedPitchMagnitude(
      config, fmaxf(forward_speed_mps, 0.0f));
  if (up_speed_mps < 0.0f) {
    const float remaining_loss_m = fmaxf(
        config->maximum_altitude_loss_m - config->altitude_reserve_m -
            fmaxf(total_altitude_loss_m, 0.0f),
        1.0e-3f);
    const float required_up_acceleration_mps2 = fminf(
        up_speed_mps * up_speed_mps / (2.0f * remaining_loss_m),
        config->recovery_maximum_vertical_acceleration_mps2);
    const float maximum_thrust_acceleration_mps2 =
        4.0f * config->maximum_motor_thrust_n / config->mass_kg;
    const float required_vertical_thrust_mps2 =
        config->gravity_mps2 + required_up_acceleration_mps2;
    const float altitude_pitch_limit_rad = required_vertical_thrust_mps2 >=
            maximum_thrust_acceleration_mps2
        ? 0.0f
        : acosf(tinyMpcPitchThroughClamp(
              required_vertical_thrust_mps2 /
                  maximum_thrust_acceleration_mps2,
              0.0f, 1.0f));
    if (altitude_pitch_limit_rad < magnitude) {
      magnitude = altitude_pitch_limit_rad;
      if (altitude_limited != NULL) {
        *altitude_limited = true;
      }
    }
  }
  return -magnitude;
}

static inline TinyMpcPitchThroughBrakeSample tinyMpcPitchThroughAdvance(
    const TinyMpcPitchThroughBrakeConfig *config,
    const TinyMpcPitchThroughBrakeState *initial, float dt_s) {
  TinyMpcPitchThroughBrakeSample sample = {0};
  if (!tinyMpcPitchThroughConfigValid(config) || initial == NULL ||
      !isfinite(dt_s) || dt_s <= 0.0f ||
      !isfinite(initial->forward_position_m) ||
      !isfinite(initial->up_position_m) ||
      !isfinite(initial->forward_speed_mps) ||
      !isfinite(initial->up_speed_mps) || !isfinite(initial->pitch_rad) ||
      !isfinite(initial->pitch_rate_rad_s) ||
      !isfinite(initial->altitude_loss_at_origin_m)) {
    return sample;
  }

  sample.state = *initial;
  sample.braking = initial->forward_speed_mps >
      config->forward_release_speed_mps;
  float desired_total_thrust_n = 0.0f;
  if (sample.braking) {
    const float total_altitude_loss_m =
        initial->altitude_loss_at_origin_m +
        fmaxf(-initial->up_position_m, 0.0f);
    sample.target_pitch_rad = tinyMpcPitchThroughBrakingPitch(
        config, initial->forward_speed_mps, initial->up_speed_mps,
        total_altitude_loss_m, &sample.altitude_limited);
    desired_total_thrust_n = 4.0f * config->maximum_motor_thrust_n;
  } else {
    const float desired_forward_acceleration_mps2 =
        tinyMpcPitchThroughClamp(
            -config->recovery_forward_gain_per_s *
                initial->forward_speed_mps,
            -config->recovery_maximum_acceleration_mps2,
            config->recovery_maximum_acceleration_mps2);
    const float total_altitude_error_m =
        initial->altitude_loss_at_origin_m - initial->up_position_m;
    const float desired_up_acceleration_mps2 =
        tinyMpcPitchThroughClamp(
            config->recovery_altitude_gain_per_s2 * total_altitude_error_m -
                config->recovery_vertical_damping_per_s *
                    initial->up_speed_mps,
            -config->recovery_maximum_vertical_acceleration_mps2,
            config->recovery_maximum_vertical_acceleration_mps2);
    const float drag_acceleration_mps2 =
        config->signed_drag_n_per_mps * initial->forward_speed_mps /
            config->mass_kg;
    const float thrust_forward_acceleration_mps2 =
        desired_forward_acceleration_mps2 - drag_acceleration_mps2;
    const float thrust_up_acceleration_mps2 =
        config->gravity_mps2 + desired_up_acceleration_mps2;
    sample.target_pitch_rad = atan2f(
        thrust_forward_acceleration_mps2,
        thrust_up_acceleration_mps2);
    desired_total_thrust_n = config->mass_kg * hypotf(
        thrust_forward_acceleration_mps2,
        thrust_up_acceleration_mps2);
    if (initial->forward_speed_mps > 0.0f || initial->up_speed_mps < 0.0f) {
      /* Recovery begins at the closest point, often with the thrust vector
       * still nearly horizontal. Keep full collective until descent is
       * arrested; otherwise the geometrically correct recovery direction has
       * too little vertical authority during the attitude transient. */
      desired_total_thrust_n = 4.0f * config->maximum_motor_thrust_n;
    }
  }

  /* Generate a second-order-continuous attitude reference from measured
   * pitch and pitch rate. A direct angle slew restarts at the maximum rate on
   * every receding-horizon solve and alternates saturated motor pairs once
   * the plant overshoots. The acceleration bound is therefore part of the
   * reference itself, not merely a feed-forward clamp. */
  const float desired_pitch_rate_rad_s = tinyMpcPitchThroughClamp(
      8.0f * (sample.target_pitch_rad - initial->pitch_rad),
      -config->maximum_pitch_rate_rad_s,
      config->maximum_pitch_rate_rad_s);
  const float pitch_acceleration_rad_s2 = tinyMpcPitchThroughClamp(
      (desired_pitch_rate_rad_s - initial->pitch_rate_rad_s) / dt_s,
      -config->maximum_pitch_acceleration_rad_s2,
      config->maximum_pitch_acceleration_rad_s2);
  const float next_pitch_rate_rad_s = tinyMpcPitchThroughClamp(
      initial->pitch_rate_rad_s + pitch_acceleration_rad_s2 * dt_s,
      -config->maximum_pitch_rate_rad_s,
      config->maximum_pitch_rate_rad_s);
  float next_pitch_rad = initial->pitch_rad +
      0.5f * (initial->pitch_rate_rad_s + next_pitch_rate_rad_s) * dt_s;
  if ((sample.target_pitch_rad - initial->pitch_rad) *
          (sample.target_pitch_rad - next_pitch_rad) <= 0.0f) {
    next_pitch_rad = sample.target_pitch_rad;
  }
  const float pitch_differential_n = tinyMpcPitchThroughClamp(
      config->pitch_inertia_kg_m2 * pitch_acceleration_rad_s2 /
          (4.0f * config->pitch_arm_m),
      -0.5f * config->maximum_motor_thrust_n,
      0.5f * config->maximum_motor_thrust_n);
  const float desired_collective_motor_n = tinyMpcPitchThroughClamp(
      desired_total_thrust_n / 4.0f, 0.0f,
      config->maximum_motor_thrust_n);
  const float collective_motor_n = fmaxf(
      fabsf(pitch_differential_n),
      fminf(desired_collective_motor_n,
            config->maximum_motor_thrust_n -
                fabsf(pitch_differential_n)));
  sample.motor_thrust_n[0] = collective_motor_n - pitch_differential_n;
  sample.motor_thrust_n[1] = collective_motor_n + pitch_differential_n;
  sample.motor_thrust_n[2] = sample.motor_thrust_n[1];
  sample.motor_thrust_n[3] = sample.motor_thrust_n[0];

  const float actual_total_thrust_n = 4.0f * collective_motor_n;
  const float midpoint_pitch_rad =
      0.5f * (initial->pitch_rad + next_pitch_rad);
  const float thrust_acceleration_mps2 =
      actual_total_thrust_n / config->mass_kg;
  sample.forward_acceleration_mps2 =
      thrust_acceleration_mps2 * sinf(midpoint_pitch_rad) +
      config->signed_drag_n_per_mps * initial->forward_speed_mps /
          config->mass_kg;
  sample.up_acceleration_mps2 =
      thrust_acceleration_mps2 * cosf(midpoint_pitch_rad) -
      config->gravity_mps2;
  sample.state.forward_speed_mps = initial->forward_speed_mps +
      sample.forward_acceleration_mps2 * dt_s;
  sample.state.up_speed_mps = initial->up_speed_mps +
      sample.up_acceleration_mps2 * dt_s;
  sample.state.forward_position_m = initial->forward_position_m +
      0.5f * (initial->forward_speed_mps +
          sample.state.forward_speed_mps) * dt_s;
  sample.state.up_position_m = initial->up_position_m +
      0.5f * (initial->up_speed_mps + sample.state.up_speed_mps) * dt_s;
  sample.state.pitch_rad = next_pitch_rad;
  sample.state.pitch_rate_rad_s = next_pitch_rate_rad_s;
  sample.valid = true;
  return sample;
}

static inline float tinyMpcPitchThroughIdealStoppingDistance(
    float speed_mps, float acceleration_mps2) {
  return isfinite(speed_mps) && isfinite(acceleration_mps2) &&
          speed_mps >= 0.0f && acceleration_mps2 > 0.0f
      ? speed_mps * speed_mps / (2.0f * acceleration_mps2)
      : INFINITY;
}

#ifdef __cplusplus
}
#endif

#endif  // TINYMPC_PITCH_THROUGH_BRAKE_H
