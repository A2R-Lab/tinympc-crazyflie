#ifndef TINYMPC_DISTANCE_BRAKE_H
#define TINYMPC_DISTANCE_BRAKE_H
#include "tinympc_espnet_straight.h"

static inline void tinympcDistanceBrakeStep(tinympcEspnetStraightState *state,
    float measured_s, float total_speed, bool run, float dt,
    float target_speed, float brake_distance) {
  const bool rising = run && !state->previous_run;
  state->previous_run = run;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_STOP) return;
  if (!isfinite(target_speed) || target_speed <= 0 || target_speed > 12 ||
      !isfinite(brake_distance) || brake_distance <= 0 || brake_distance > 20 ||
      !isfinite(measured_s) || !isfinite(total_speed) || total_speed < 0 ||
      !isfinite(dt) || dt < 0 || !isfinite(state->s) ||
      !isfinite(state->v) || !isfinite(state->elapsed)) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_INVALID, measured_s);
    return;
  }
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_IDLE) {
    if (!rising) return;
    if (total_speed > .15f) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_MOVING;
      return;
    }
    state->phase = TINYMPC_ESPNET_STRAIGHT_RUN;
    state->reason = TINYMPC_ESPNET_STRAIGHT_NONE;
  }
  state->elapsed += dt;
  if (!run || state->elapsed >= 15 || fabsf(measured_s) >= brake_distance) {
    tinympcEspnetStraightStop(state, !run ? TINYMPC_ESPNET_STRAIGHT_CANCEL :
        state->elapsed >= 15 ? TINYMPC_ESPNET_STRAIGHT_TIMEOUT :
        TINYMPC_ESPNET_STRAIGHT_DISTANCE, measured_s);
    return;
  }
  state->v = target_speed;
  state->s += target_speed * fminf(dt, .1f);
}
#endif
