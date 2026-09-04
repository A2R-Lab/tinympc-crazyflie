#include "tinympc_emergency_stop.h"

#include <assert.h>
#include <math.h>

int main(void) {
  /* The position-derived speed wins when the estimator is time-scaled. */
  assert(fabsf(tinyMpcEmergencyMeasuredForwardSpeed(
      0.96f, 2.06f, true) - 2.06f) < 1.0e-6f);
  assert(fabsf(tinyMpcEmergencyMeasuredForwardSpeed(
      0.96f, 2.06f, false) - 0.96f) < 1.0e-6f);
  assert(tinyMpcEmergencyMeasuredForwardSpeed(-0.2f, -0.1f, true) == 0.0f);

  /* A zero reference alone may not enter stop hold while motion remains. */
  assert(!tinyMpcEmergencyMotionSettled(0.11f, 0.0f, 0.10f));
  assert(!tinyMpcEmergencyMotionSettled(0.08f, 0.08f, 0.10f));
  assert(tinyMpcEmergencyMotionSettled(0.06f, 0.06f, 0.10f));
  assert(!tinyMpcEmergencyMotionSettled(NAN, 0.0f, 0.10f));

  /* Stop hold requires low signed motion and a captured, nearly-level body. */
  assert(tinyMpcEmergencyStopReady(
      -0.06f, 0.02f, 0.10f, 0.04f, 0.50f, 0.087f, 0.75f));
  assert(!tinyMpcEmergencyStopReady(
      -0.16f, 0.0f, 0.10f, 0.04f, 0.50f, 0.087f, 0.75f));
  assert(!tinyMpcEmergencyStopReady(
      0.02f, 0.0f, 0.10f, -0.30f, 0.50f, 0.087f, 0.75f));
  assert(!tinyMpcEmergencyStopReady(
      0.02f, 0.0f, 0.10f, 0.04f, 1.20f, 0.087f, 0.75f));

  /* Emergency attitude references move continuously in both directions. */
  assert(fabsf(tinyMpcEmergencySlewAttitude(
      0.08f, -0.47f, 3.0f, 0.02f) - 0.02f) < 1.0e-6f);
  assert(fabsf(tinyMpcEmergencySlewAttitude(
      -0.47f, 0.0f, 3.0f, 0.02f) + 0.41f) < 1.0e-6f);
  assert(fabsf(tinyMpcEmergencySlewAttitude(
      -0.02f, 0.0f, 3.0f, 0.02f)) < 1.0e-6f);
  assert(tinyMpcEmergencySlewAttitude(
      0.1f, -0.4f, NAN, 0.02f) == 0.1f);

  /* A 10 m/s^2 net stop at 2 m/s accounts for modeled passive drag. */
  const float pitch_10_mps2 = tinyMpcEmergencyPitchForNetDeceleration(
      10.0f, 2.0f, -0.02149163f, 0.04338f, 9.81f);
  assert(fabsf(pitch_10_mps2 + 0.7428f) < 1.0e-3f);
  assert(tinyMpcEmergencyPitchForNetDeceleration(
      10.0f, -1.0f, -0.021f, 0.043f, 9.81f) == 0.0f);

  /* The terminal anchor actively damps velocity in either direction. */
  assert(fabsf(tinyMpcEmergencyHoldSpeedCommand(
      0.0f, 0.16f, 1.5f, 1.25f, 0.30f) + 0.20f) < 1.0e-6f);
  assert(fabsf(tinyMpcEmergencyHoldSpeedCommand(
      0.0f, -0.16f, 1.5f, 1.25f, 0.30f) - 0.20f) < 1.0e-6f);
  assert(tinyMpcEmergencyHoldSpeedCommand(
      -1.0f, 0.0f, 1.5f, 1.25f, 0.30f) == -0.30f);
  return 0;
}
