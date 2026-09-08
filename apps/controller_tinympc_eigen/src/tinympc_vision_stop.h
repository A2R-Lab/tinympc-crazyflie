#ifndef TINYMPC_VISION_STOP_H
#define TINYMPC_VISION_STOP_H

#include "tinympc_espnet_straight.h"

#define TINYMPC_VISION_TARGET_SPEED 2.0f

static inline void tinympcVisionAdvanceReference(float *s, float *v, float dt)
{
  if (!isfinite(dt) || dt < 0.0f) return;
  *s += TINYMPC_VISION_TARGET_SPEED * fminf(dt, 0.1f);
  *v = TINYMPC_VISION_TARGET_SPEED;
}

typedef struct {
  bool valid;
  unsigned dangerous_sectors;
  int turn_direction; /* +1: camera left; -1: camera right; 0: hold heading */
} tinympcVisionDecision;

static inline tinympcVisionDecision tinympcVisionClassify(
    bool fresh_valid, float left, float center, float right)
{
  tinympcVisionDecision result = {false, 0u, 0};
  result.valid = fresh_valid &&
      isfinite(left) && left >= 0.0f && left <= 1.0f &&
      isfinite(center) && center >= 0.0f && center <= 1.0f &&
      isfinite(right) && right >= 0.0f && right <= 1.0f;
  if (!result.valid) return result;
  const bool l = left > 0.9f;
  const bool c = center > 0.9f;
  const bool r = right > 0.9f;
  result.dangerous_sectors = (unsigned)l + (unsigned)c + (unsigned)r;
  if (result.dangerous_sectors == 2u) {
    if (!l) result.turn_direction = 1;
    else if (!r) result.turn_direction = -1;
    /* With center safe and both sides blocked, preserve camera heading. */
  }
  return result;
}

/* Use the same world heading for the yaw target and the next straight leg.
 * An invalid input does not invent a new direction. */
static inline float tinympcVisionTurnHeading(float prior_heading, int direction)
{
  if (!isfinite(prior_heading) || direction < -1 || direction > 1)
    return prior_heading;
  const float heading = prior_heading + direction * 0.78539816339744830962f;
  return atan2f(sinf(heading), cosf(heading));
}

/* After the turn has completed, require a continuous clear, stationary dwell.
 * One blocked sector also prevents automatic resumption. */
static inline bool tinympcVisionResumeClear(
    bool fresh_valid, float left, float center, float right,
    float total_speed, float dt, float *clear_time)
{
  if (!clear_time) return false;
  const tinympcVisionDecision vision =
      tinympcVisionClassify(fresh_valid, left, center, right);
  if (!vision.valid || vision.dangerous_sectors != 0u ||
      !isfinite(total_speed) || total_speed < 0.0f || total_speed > 0.15f ||
      !isfinite(dt) || dt < 0.0f ||
      !isfinite(*clear_time) || *clear_time < 0.0f) {
    *clear_time = 0.0f;
    return false;
  }
  *clear_time = fminf(0.3f, *clear_time + fminf(dt, 0.1f));
  return *clear_time >= 0.3f - 1e-6f;
}

/* Heading reference uses radians and a 30 degree/s bound. */
static inline float tinympcVisionYawAdvance(float current, float target, float dt)
{
  if (!isfinite(current) || !isfinite(target) || !isfinite(dt) || dt <= 0.0f)
    return current;
  const float pi = 3.14159265358979323846f;
  const float delta = atan2f(sinf(target - current), cosf(target - current));
  const float limit = (pi / 6.0f) * fminf(dt, 0.1f);
  const float step = fmaxf(-limit, fminf(limit, delta));
  return atan2f(sinf(current + step), cosf(current + step));
}

/* Two blocked sectors stop travel. A safe lateral sector requests a turn only
 * on an accepted RUN -> STOP transition. The caller brakes before turning.
 * STOP and the selected direction latch until a new controller handoff.
 * Rejected starts never request a turn. */
static inline void tinympcVisionStraightStep(
    tinympcEspnetStraightState *state, float measured_s, float total_speed,
    bool fresh_valid, float left, float center, float right,
    bool run, float dt, int *turn_direction)
{
  const bool rising = run && !state->previous_run;
  state->previous_run = run;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_STOP) return;
  *turn_direction = 0;
  if (!isfinite(measured_s) || !isfinite(total_speed) ||
      !isfinite(state->s) || !isfinite(state->v) ||
      !isfinite(state->elapsed) || !isfinite(dt) || dt < 0.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_INVALID, measured_s);
    return;
  }
  const tinympcVisionDecision vision =
      tinympcVisionClassify(fresh_valid, left, center, right);
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_IDLE) {
    if (!rising) return;
    if (!vision.valid) { state->reason = TINYMPC_ESPNET_STRAIGHT_STALE; return; }
    if (vision.dangerous_sectors >= 2u) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_DANGER;
      return;
    }
    if (fabsf(total_speed) > 0.15f) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_MOVING;
      return;
    }
    state->phase = TINYMPC_ESPNET_STRAIGHT_RUN;
    state->reason = TINYMPC_ESPNET_STRAIGHT_NONE;
  }
  if (!vision.valid) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_STALE, measured_s);
    return;
  }
  if (!run) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_CANCEL, measured_s);
    return;
  }
  if (vision.dangerous_sectors >= 2u) {
    *turn_direction = vision.turn_direction;
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DANGER, measured_s);
    return;
  }
  state->elapsed += dt;
  if (state->elapsed >= 15.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_TIMEOUT, measured_s);
    return;
  }
  if (fabsf(measured_s) >= 5.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DISTANCE, measured_s);
    return;
  }
  tinympcVisionAdvanceReference(&state->s, &state->v, dt);
}
#endif
