#include <assert.h>
#include <stdio.h>

#include "tinympc_reactive_power_loop.h"

static void testClearStopLoopRecoverOnce(void) {
  TinyMpcReactivePowerLoopState state;
  tinyMpcReactivePowerLoopReset(&state);
  TinyMpcReactivePowerLoopConfig config =
      tinyMpcReactivePowerLoopDefaultConfig();
  config.clear_samples_required = 2u;
  config.settled_samples_required = 2u;
  config.recovery_samples_required = 2u;

  TinyMpcReactivePowerLoopCommand command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, true, 0.20f, true, true, false, false, false);
  assert(state.phase == TINYMPC_REACTIVE_LOOP_WAITING_CLEAR);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, true, 0.19f, true, true, false, false, false);
  assert(command.changed && command.request_stop);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 0.19f, true, true, true, false, false);
  assert(state.phase == TINYMPC_REACTIVE_LOOP_BRAKING);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 0.19f, true, true, true, false, false);
  assert(command.changed && command.active && command.owns_reference);
  assert(state.trigger_count == 1u);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 1.0f, false, false, false, true, false);
  assert(command.changed && command.request_stop && command.owns_reference);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 0.0f, true, true, true, false, true);
  assert(state.phase == TINYMPC_REACTIVE_LOOP_RECOVERY);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 0.0f, true, true, true, false, true);
  assert(command.changed && state.phase == TINYMPC_REACTIVE_LOOP_COMPLETE);
  for (int sample = 0; sample < 20; ++sample) {
    (void)tinyMpcReactivePowerLoopUpdate(
        &state, &config, true, 0.0f, true, true, true, false, true);
  }
  assert(state.trigger_count == 1u);
}

static void testDangerOrContainmentCancelsOnlyBeforeEntry(void) {
  TinyMpcReactivePowerLoopState state;
  tinyMpcReactivePowerLoopReset(&state);
  TinyMpcReactivePowerLoopConfig config =
      tinyMpcReactivePowerLoopDefaultConfig();
  config.clear_samples_required = 1u;
  (void)tinyMpcReactivePowerLoopUpdate(
      &state, &config, true, 0.0f, true, true, false, false, false);
  TinyMpcReactivePowerLoopCommand command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, true, 0.9f, true, true, false, false, false);
  assert(command.changed);
  assert(state.phase == TINYMPC_REACTIVE_LOOP_WAITING_CLEAR);
  (void)tinyMpcReactivePowerLoopUpdate(
      &state, &config, true, 0.0f, true, true, false, false, false);
  command = tinyMpcReactivePowerLoopUpdate(
      &state, &config, false, 0.0f, true, false, true, false, false);
  assert(command.changed);
  assert(state.phase == TINYMPC_REACTIVE_LOOP_WAITING_CLEAR);
}

static void testTranslatedPathMustFitArena(void) {
  const float pi = 3.14159265358979323846f;
  assert(tinyMpcReactivePowerLoopPathFitsArena(
      0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 4.0f, 0.15f));
  assert(!tinyMpcReactivePowerLoopPathFitsArena(
      3.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 4.0f, 0.15f));
  assert(tinyMpcReactivePowerLoopPathFitsArena(
      0.0f, 0.0f, 0.5f * pi, -1.0f, 1.0f, 0.0f, 0.0f,
      4.0f, 0.15f));
  assert(!tinyMpcReactivePowerLoopPathFitsArena(
      0.0f, 3.0f, 0.5f * pi, -1.0f, 1.0f, 0.0f, 0.0f,
      4.0f, 0.15f));
}

int main(void) {
  testClearStopLoopRecoverOnce();
  testDangerOrContainmentCancelsOnlyBeforeEntry();
  testTranslatedPathMustFitArena();
  puts("tinympc reactive power loop tests passed");
  return 0;
}
