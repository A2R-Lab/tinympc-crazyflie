#include "tiny_pulp_dronet_v3_servo.h"

#include <assert.h>
#include <math.h>

static bool near(float left, float right) {
  return fabsf(left - right) < 1.0e-6f;
}

int main(void) {
  TinyPulpDronetV3Servo servo = tinyPulpDronetV3ServoDefault(2.0f);
  assert(tinyPulpDronetV3ServoValid(&servo));
  TinyPulpDronetV3Command first =
      tinyPulpDronetV3ServoStep(&servo, 0.5f, 0.25f);
  assert(near(first.forward_velocity_mps, 1.05f));
  assert(near(first.yaw_rate_deg_s, 42.0f));
  TinyPulpDronetV3Command second =
      tinyPulpDronetV3ServoStep(&servo, 0.5f, 0.25f);
  assert(near(second.forward_velocity_mps, 1.365f));
  assert(near(second.yaw_rate_deg_s, 54.6f));
  TinyPulpDronetV3Command clipped =
      tinyPulpDronetV3ServoStep(&servo, 5.0f, 5.0f);
  assert(clipped.forward_velocity_mps >= 0.0f);
  assert(clipped.yaw_rate_deg_s <= 120.0f);
  tinyPulpDronetV3ServoReset(&servo);
  assert(!servo.initialized);
  return 0;
}
