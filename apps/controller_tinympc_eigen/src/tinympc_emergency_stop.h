#ifndef TINYMPC_EMERGENCY_STOP_H
#define TINYMPC_EMERGENCY_STOP_H

#include <math.h>
#include <stdbool.h>

/*
 * Emergency braking is a receding-horizon operation.  The reference must be
 * rebuilt from measured motion until the vehicle is physically settled; a
 * reference clock reaching zero is not evidence that the vehicle stopped.
 */
static inline float tinyMpcEmergencyMeasuredForwardSpeed(
    float estimator_speed_mps, float position_delta_speed_mps,
    bool position_delta_valid) {
  const float selected = position_delta_valid &&
          isfinite(position_delta_speed_mps)
      ? position_delta_speed_mps : estimator_speed_mps;
  return isfinite(selected) ? fmaxf(selected, 0.0f) : 0.0f;
}

static inline bool tinyMpcEmergencyMotionSettled(
    float forward_speed_mps, float lateral_speed_mps,
    float maximum_horizontal_speed_mps) {
  return isfinite(forward_speed_mps) && isfinite(lateral_speed_mps) &&
      isfinite(maximum_horizontal_speed_mps) &&
      maximum_horizontal_speed_mps >= 0.0f &&
      hypotf(forward_speed_mps, lateral_speed_mps) <=
          maximum_horizontal_speed_mps;
}

static inline bool tinyMpcEmergencyStopReady(
    float signed_forward_speed_mps, float lateral_speed_mps,
    float maximum_horizontal_speed_mps, float pitch_rad,
    float body_rate_rad_s, float maximum_pitch_rad,
    float maximum_body_rate_rad_s) {
  return tinyMpcEmergencyMotionSettled(
          signed_forward_speed_mps, lateral_speed_mps,
          maximum_horizontal_speed_mps) &&
      isfinite(pitch_rad) && isfinite(body_rate_rad_s) &&
      isfinite(maximum_pitch_rad) && isfinite(maximum_body_rate_rad_s) &&
      maximum_pitch_rad >= 0.0f && maximum_body_rate_rad_s >= 0.0f &&
      fabsf(pitch_rad) <= maximum_pitch_rad &&
      body_rate_rad_s <= maximum_body_rate_rad_s;
}

static inline float tinyMpcEmergencySlewAttitude(
    float current_rad, float target_rad, float maximum_rate_rad_s,
    float dt_s) {
  if (!isfinite(current_rad) || !isfinite(target_rad) ||
      !isfinite(maximum_rate_rad_s) || !isfinite(dt_s) ||
      maximum_rate_rad_s <= 0.0f || dt_s <= 0.0f) {
    return isfinite(current_rad) ? current_rad : 0.0f;
  }
  const float maximum_step_rad = maximum_rate_rad_s * dt_s;
  return current_rad + fminf(fmaxf(
      target_rad - current_rad, -maximum_step_rad), maximum_step_rad);
}

/* Pitch required for a requested net longitudinal deceleration while holding
 * altitude. The signed drag coefficient is negative in forward flight. */
static inline float tinyMpcEmergencyPitchForNetDeceleration(
    float net_deceleration_mps2, float forward_speed_mps,
    float signed_drag_n_per_mps, float mass_kg, float gravity_mps2) {
  if (!isfinite(net_deceleration_mps2) ||
      !isfinite(forward_speed_mps) ||
      !isfinite(signed_drag_n_per_mps) || !isfinite(mass_kg) ||
      !isfinite(gravity_mps2) || net_deceleration_mps2 < 0.0f ||
      forward_speed_mps < 0.0f || mass_kg <= 0.0f || gravity_mps2 <= 0.0f) {
    return 0.0f;
  }
  const float passive_drag_deceleration_mps2 = fmaxf(
      -signed_drag_n_per_mps * forward_speed_mps / mass_kg, 0.0f);
  const float thrust_deceleration_mps2 = fmaxf(
      net_deceleration_mps2 - passive_drag_deceleration_mps2, 0.0f);
  return -atan2f(thrust_deceleration_mps2, gravity_mps2);
}

static inline float tinyMpcEmergencyHoldSpeedCommand(
    float signed_anchor_error_m, float signed_forward_speed_mps,
    float position_gain_per_s, float velocity_damping_gain,
    float maximum_speed_mps) {
  if (!isfinite(signed_anchor_error_m) ||
      !isfinite(signed_forward_speed_mps) ||
      !isfinite(position_gain_per_s) ||
      !isfinite(velocity_damping_gain) || !isfinite(maximum_speed_mps) ||
      position_gain_per_s < 0.0f || velocity_damping_gain < 0.0f ||
      maximum_speed_mps < 0.0f) {
    return 0.0f;
  }
  const float command_mps =
      position_gain_per_s * signed_anchor_error_m -
      velocity_damping_gain * signed_forward_speed_mps;
  return fminf(fmaxf(
      command_mps, -maximum_speed_mps), maximum_speed_mps);
}

#endif  // TINYMPC_EMERGENCY_STOP_H
