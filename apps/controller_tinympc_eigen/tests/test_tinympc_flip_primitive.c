#include "tinympc_flip_primitive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const TinyMpcFlipInputKnot inputs[] = {
    {0.0f, {0.10f, 0.10f, 0.10f, 0.10f}},
    {0.5f, {0.02f, 0.03f, 0.03f, 0.02f}},
    {1.0f, {0.10f, 0.10f, 0.10f, 0.10f}},
};
static const int16_t models[] = {10, 11, 12, 13};

static TinyMpcFlipConfig config(void) {
  const TinyMpcFlipConfig result = {
      2.0f, 2.5f, 0.8f, 1, inputs, 3u, models, 4u};
  return result;
}

static int near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static void testTriggerAndOneShotPhase(void) {
  TinyMpcFlipConfig cfg = config();
  TinyMpcFlipState state;
  tinyMpcFlipReset(&state);
  assert(tinyMpcFlipUpdate(&state, &cfg, 1.9f, 0.02f, true, true)
      == TINYMPC_FLIP_WAITING);
  assert(tinyMpcFlipUpdate(&state, &cfg, 2.1f, 0.02f, true, true)
      == TINYMPC_FLIP_ACTIVE);
  assert(near(state.sigma, 0.0f, 1.0e-7f));
  assert(state.trigger_count == 1u);
  assert(tinyMpcFlipUpdate(&state, &cfg, 2.2f, 0.02f, false, false)
      == TINYMPC_FLIP_ACTIVE);
  assert(near(state.sigma, 0.0f, 1.0e-7f));
  for (int step = 0; step < 41; ++step) {
    tinyMpcFlipUpdate(&state, &cfg, 2.3f, 0.02f, false, true);
  }
  assert(state.mode == TINYMPC_FLIP_COMPLETE);
  assert(near(state.sigma, 1.0f, 1.0e-6f));
  tinyMpcFlipUpdate(&state, &cfg, 2.1f, 0.02f, true, true);
  assert(state.trigger_count == 1u);
}

static void testQuaternionRatesInputsAndModels(void) {
  TinyMpcFlipConfig cfg = config();
  const TinyMpcFlipSample start = tinyMpcFlipSample(&cfg, 0.0f, 0.4f, 1.2f);
  const TinyMpcFlipSample middle = tinyMpcFlipSample(&cfg, 0.5f, 0.4f, 1.2f);
  const TinyMpcFlipSample finish = tinyMpcFlipSample(&cfg, 1.0f, 0.4f, 1.2f);
  assert(start.valid && middle.valid && finish.valid);
  assert(near(start.pitch_rad, 0.0f, 1.0e-6f));
  assert(near(fabsf(middle.pitch_rad), 3.14159265358979323846f, 1.0e-5f));
  assert(near(finish.pitch_rad, 6.28318530717958647692f, 1.0e-5f));
  assert(near(start.pitch_rate_rad_s, 0.0f, 1.0e-6f));
  assert(near(finish.pitch_rate_rad_s, 0.0f, 1.0e-6f));
  const float norm = sqrtf(
      middle.attitude_world_body.x * middle.attitude_world_body.x
      + middle.attitude_world_body.y * middle.attitude_world_body.y
      + middle.attitude_world_body.z * middle.attitude_world_body.z
      + middle.attitude_world_body.w * middle.attitude_world_body.w);
  assert(near(norm, 1.0f, 1.0e-6f));
  assert(near(middle.body_rate_rad_s.x, 0.0f, 1.0e-5f));
  assert(near(middle.body_rate_rad_s.z, -1.2f, 1.0e-5f));
  assert(near(middle.motor_thrust_n[0], 0.02f, 1.0e-6f));
  assert(middle.phase_model_id == 12);
  assert(finish.phase_model_id == 13);
}

static void testValidationFailsClosed(void) {
  TinyMpcFlipConfig cfg = config();
  cfg.duration_s = 0.0f;
  assert(!tinyMpcFlipConfigValid(&cfg));
  TinyMpcFlipState state;
  tinyMpcFlipReset(&state);
  assert(tinyMpcFlipUpdate(&state, &cfg, 2.1f, 0.02f, true, true)
      == TINYMPC_FLIP_FAULT);
}

static void testMeasuredRecoveryGate(void) {
  assert(tinyMpcFlipRecoveryReady(0.95f, 1.5f, 0.94f, 2.0f));
  assert(!tinyMpcFlipRecoveryReady(0.93f, 1.5f, 0.94f, 2.0f));
  assert(!tinyMpcFlipRecoveryReady(0.95f, 2.1f, 0.94f, 2.0f));
  assert(!tinyMpcFlipRecoveryReady(NAN, 1.0f, 0.94f, 2.0f));
}

int main(void) {
  assert(tinyMpcFlipConfigValid(&(TinyMpcFlipConfig){
      2.0f, 2.5f, 0.8f, 1, inputs, 3u, models, 4u}));
  testTriggerAndOneShotPhase();
  testQuaternionRatesInputsAndModels();
  testValidationFailsClosed();
  testMeasuredRecoveryGate();
  puts("tinympc flip primitive tests passed");
  return 0;
}
