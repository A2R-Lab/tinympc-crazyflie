#ifndef TINYMPC_VISION_BRAKE_H
#define TINYMPC_VISION_BRAKE_H

#include "tinympc_espnet_straight.h"
#include "tinympc_vision_stop.h"

#define TINYMPC_VISION_BRAKE_THRESHOLD 0.95f

static inline unsigned tinympcVisionBrakeDangerCount(
    float left, float center, float right)
{
  return (unsigned)(left > TINYMPC_VISION_BRAKE_THRESHOLD) +
         (unsigned)(center > TINYMPC_VISION_BRAKE_THRESHOLD) +
         (unsigned)(right > TINYMPC_VISION_BRAKE_THRESHOLD);
}

/* One straight run, with a vision-triggered latched brake and no turn/resume.
 * Distance and attained speed never trigger braking in this mode. */
static inline void tinympcVisionBrakeStep(
    tinympcEspnetStraightState *state, float measured_s, float total_speed,
    bool fresh, float left, float center, float right, bool run, float dt,
    uint16_t sequence)
{
  const bool rising = run && !state->previous_run;
  state->previous_run = run;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_STOP) return;
  if (!isfinite(measured_s) || !isfinite(total_speed) || total_speed < 0.0f ||
      !isfinite(state->s) || !isfinite(state->v) ||
      !isfinite(state->elapsed) || !isfinite(dt) || dt < 0.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_INVALID, measured_s);
    return;
  }
  const tinympcVisionDecision vision =
      tinympcVisionClassify(fresh, left, center, right);
  const bool brake_danger = vision.valid && center > TINYMPC_VISION_BRAKE_THRESHOLD;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_IDLE) {
    state->brake_frames = 0;
    state->brake_sequence = sequence;
    if (!rising) return;
    if (!vision.valid) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_STALE;
      return;
    }
    if (brake_danger) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_DANGER;
      return;
    }
    if (total_speed > 0.15f) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_MOVING;
      return;
    }
    state->phase = TINYMPC_ESPNET_STRAIGHT_RUN;
    state->reason = TINYMPC_ESPNET_STRAIGHT_NONE;
  }
  if (!run) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_CANCEL, measured_s);
    return;
  }
  if (!vision.valid) {
    state->brake_frames = 0;
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_STALE, measured_s);
    return;
  }
  // One fresh qualifying observation immediately latches the brake.
  state->brake_sequence = sequence;
  state->brake_frames = brake_danger ? 1u : 0u;
  if (brake_danger) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DANGER, measured_s);
    return;
  }
  state->elapsed += dt;
  if (state->elapsed >= 15.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_TIMEOUT, measured_s);
    return;
  }
  state->s += TINYMPC_VISION_TARGET_SPEED * fminf(dt, 0.1f);
  state->v = TINYMPC_VISION_TARGET_SPEED;
}

#endif
