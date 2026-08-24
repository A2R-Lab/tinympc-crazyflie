#include "tinympc_power_loop.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static TinyMpcPowerLoopConfig config(void) {
  const TinyMpcPowerLoopConfig result = {
      1.0f, 1.25f, 1.0f, 1.8f, 4.0f,
      0.04338f, 9.81f, 0.20f, 28.0e-6f, 0.035355f};
  return result;
}

static int near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static void testGeometryAndFeasibility(void) {
  TinyMpcPowerLoopConfig cfg = config();
  assert(tinyMpcPowerLoopConfigValid(&cfg));
  const TinyMpcPowerLoopSample bottom = tinyMpcPowerLoopSample(&cfg, 0.0f);
  const TinyMpcPowerLoopSample top = tinyMpcPowerLoopSample(&cfg, 0.5f);
  const TinyMpcPowerLoopSample finish = tinyMpcPowerLoopSample(&cfg, 1.0f);
  assert(bottom.valid && top.valid && finish.valid);
  assert(near(bottom.position_forward_m, 0.0f, 1.0e-6f));
  assert(near(bottom.position_up_m, 0.0f, 1.0e-6f));
  assert(near(top.position_forward_m, 0.0f, 1.0e-5f));
  assert(near(top.position_up_m, 2.0f, 1.0e-5f));
  assert(top.acceleration_up_mps2 < -9.81f);
  assert(fabsf(fabsf(top.pitch_rad) - 3.14159265358979323846f) < 1.0e-4f);
  assert(near(finish.position_forward_m, 0.0f, 1.0e-5f));
  assert(near(finish.position_up_m, 0.0f, 1.0e-5f));
  assert(bottom.motor_thrust_n[0] <= 0.20f);
}

static void testOneShotSpatialPhase(void) {
  TinyMpcPowerLoopConfig cfg = config();
  TinyMpcPowerLoopState state;
  tinyMpcPowerLoopReset(&state);
  assert(tinyMpcPowerLoopUpdate(
      &state, &cfg, 0.9f, 0.02f, true, false, true)
      == TINYMPC_POWER_LOOP_WAITING);
  assert(tinyMpcPowerLoopUpdate(
      &state, &cfg, 1.1f, 0.02f, true, false, true)
      == TINYMPC_POWER_LOOP_WAITING);
  assert(state.armed);
  /* Readiness may occur after the narrow spatial arming window. */
  assert(tinyMpcPowerLoopUpdate(
      &state, &cfg, 1.5f, 0.02f, true, true, true)
      == TINYMPC_POWER_LOOP_ACTIVE);
  for (int step = 0; step < 300 && state.mode == TINYMPC_POWER_LOOP_ACTIVE;
       ++step) {
    tinyMpcPowerLoopUpdate(
        &state, &cfg, 1.5f, 0.02f, false, true, true);
  }
  assert(state.mode == TINYMPC_POWER_LOOP_COMPLETE);
  assert(near(state.sigma, 1.0f, 1.0e-6f));
  assert(state.trigger_count == 1u);
}

int main(void) {
  testGeometryAndFeasibility();
  testOneShotSpatialPhase();
  puts("tinympc power loop tests passed");
  return 0;
}
