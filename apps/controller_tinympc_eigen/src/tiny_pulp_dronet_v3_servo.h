#ifndef TINY_PULP_DRONET_V3_SERVO_H
#define TINY_PULP_DRONET_V3_SERVO_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Faithful float form of the upstream Tiny-PULP-DroNet v3 IOTJ controller.
 * The benchmark overrides only max_forward_speed_mps to select the requested
 * speed. The published controller uses yaw_scale=120 deg/s and alpha=0.3. */
typedef struct {
  float max_forward_speed_mps;
  float yaw_scale_deg_s;
  float velocity_old_mps;
  float yaw_rate_old_deg_s;
  float alpha_velocity;
  float alpha_yaw;
  bool initialized;
} TinyPulpDronetV3Servo;

typedef struct {
  float forward_velocity_mps;
  float yaw_rate_deg_s;
} TinyPulpDronetV3Command;

static inline TinyPulpDronetV3Servo tinyPulpDronetV3ServoDefault(
    float max_forward_speed_mps) {
  TinyPulpDronetV3Servo servo = {
      max_forward_speed_mps, 120.0f, 0.0f, 0.0f, 0.3f, 0.3f, false};
  return servo;
}

static inline void tinyPulpDronetV3ServoReset(
    TinyPulpDronetV3Servo *servo) {
  if (servo == NULL) {
    return;
  }
  servo->velocity_old_mps = 0.0f;
  servo->yaw_rate_old_deg_s = 0.0f;
  servo->initialized = false;
}

static inline bool tinyPulpDronetV3ServoValid(
    const TinyPulpDronetV3Servo *servo) {
  return servo != NULL && isfinite(servo->max_forward_speed_mps) &&
      servo->max_forward_speed_mps > 0.0f &&
      isfinite(servo->yaw_scale_deg_s) && servo->yaw_scale_deg_s > 0.0f &&
      isfinite(servo->alpha_velocity) && servo->alpha_velocity >= 0.0f &&
      servo->alpha_velocity <= 1.0f && isfinite(servo->alpha_yaw) &&
      servo->alpha_yaw >= 0.0f && servo->alpha_yaw <= 1.0f;
}

static inline TinyPulpDronetV3Command tinyPulpDronetV3ServoStep(
    TinyPulpDronetV3Servo *servo, float normalized_yaw_rate,
    float collision_probability) {
  TinyPulpDronetV3Command command = {0.0f, 0.0f};
  if (!tinyPulpDronetV3ServoValid(servo) ||
      !isfinite(normalized_yaw_rate) || !isfinite(collision_probability)) {
    return command;
  }
  const float steering = fminf(fmaxf(normalized_yaw_rate, -1.0f), 1.0f);
  const float collision = fminf(fmaxf(collision_probability, 0.0f), 1.0f);
  const float velocity_new = (1.0f - collision) * servo->max_forward_speed_mps;
  const float yaw_new = steering * servo->yaw_scale_deg_s;
  command.forward_velocity_mps =
      (1.0f - servo->alpha_velocity) * velocity_new +
      servo->alpha_velocity * servo->velocity_old_mps;
  command.yaw_rate_deg_s =
      (1.0f - servo->alpha_yaw) * yaw_new +
      servo->alpha_yaw * servo->yaw_rate_old_deg_s;
  servo->velocity_old_mps = command.forward_velocity_mps;
  servo->yaw_rate_old_deg_s = command.yaw_rate_deg_s;
  servo->initialized = true;
  return command;
}

#endif
