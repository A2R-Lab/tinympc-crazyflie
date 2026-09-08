#ifndef TINYMPC_ESPNET_STRAIGHT_H
#define TINYMPC_ESPNET_STRAIGHT_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Hardware ESPNet packets arrive every 266–267 ms. Allow one normal frame
 * interval plus jitter; a missed frame still expires before the next one. */
#define TINYMPC_ESPNET_COLLISION_MAX_AGE_MS 400u
static inline bool tinympcEspnetCollisionFresh(bool valid, uint32_t age_ms) {
  return valid && age_ms <= TINYMPC_ESPNET_COLLISION_MAX_AGE_MS;
}

typedef enum {
  TINYMPC_ESPNET_STRAIGHT_IDLE = 0,
  TINYMPC_ESPNET_STRAIGHT_RUN,
  TINYMPC_ESPNET_STRAIGHT_STOP
} tinympcEspnetStraightPhase;

typedef enum {
  TINYMPC_ESPNET_STRAIGHT_NONE = 0,
  TINYMPC_ESPNET_STRAIGHT_DANGER,
  TINYMPC_ESPNET_STRAIGHT_STALE,
  TINYMPC_ESPNET_STRAIGHT_DISTANCE,
  TINYMPC_ESPNET_STRAIGHT_CANCEL,
  TINYMPC_ESPNET_STRAIGHT_TIMEOUT,
  TINYMPC_ESPNET_STRAIGHT_INVALID,
  TINYMPC_ESPNET_STRAIGHT_MOVING,
  TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED = 8
} tinympcEspnetStraightReason;

typedef struct {
  tinympcEspnetStraightPhase phase;
  tinympcEspnetStraightReason reason;
  float s;
  float v;
  float elapsed;
  bool previous_run;
  uint16_t brake_sequence;
  uint8_t brake_frames;
} tinympcEspnetStraightState;

/* Handoff must observe run=0 before an explicit rising edge can launch. */
static inline void tinympcEspnetStraightInit(tinympcEspnetStraightState *state)
{
  state->phase = TINYMPC_ESPNET_STRAIGHT_IDLE;
  state->reason = TINYMPC_ESPNET_STRAIGHT_NONE;
  state->s = 0.0f;
  state->v = 0.0f;
  state->elapsed = 0.0f;
  state->previous_run = true;
  state->brake_sequence = 0;
  state->brake_frames = 0;
}

/* Also usable on copies of s/v to generate the prediction horizon. */
static inline void tinympcEspnetStraightAdvanceReference(
    float *s, float *v, float measured_s, float dt)
{
  if (!isfinite(dt) || dt <= 0.0f) return;
  dt = fminf(dt, 0.1f);
  const float remaining = fmaxf(0.0f, 5.0f - fmaxf(*s, measured_s));
  // Direct velocity command, followed by a zero target at the endpoint.
  const float next_v = remaining > 0.0001f ? 3.0f : 0.0f;
  const float next_s = *s + next_v * dt;
  *s = fminf(5.0f, next_s);
  *v = next_v;
  if (5.0f - *s <= 0.0001f) {
    *s = 5.0f;
    *v = 0.0f;
  }
}

static inline void tinympcEspnetStraightStop(
    tinympcEspnetStraightState *state, tinympcEspnetStraightReason reason,
    float measured_s)
{
  state->phase = TINYMPC_ESPNET_STRAIGHT_STOP;
  state->reason = reason;
  if (isfinite(measured_s)) state->s = measured_s;
  state->v = 0.0f;
}

/* measured_speed is total vehicle speed, not just forward speed. The caller
 * supplies freshness AND validity; all three sectors must be normalized probabilities.
 * A rejected start consumes the edge. STOP is latched until a new handoff. */
static inline void tinympcEspnetStraightStep(
    tinympcEspnetStraightState *state, float measured_s, float measured_speed,
    bool telemetry_fresh_valid, float left, float center, float right, bool run, float dt)
{
  const bool rising = run && !state->previous_run;
  state->previous_run = run;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_STOP) return;
  if (!isfinite(measured_s) || !isfinite(measured_speed) ||
      !isfinite(state->s) || !isfinite(state->v) ||
      !isfinite(state->elapsed) || !isfinite(dt) || dt < 0.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_INVALID, measured_s);
    return;
  }
  const bool valid = telemetry_fresh_valid &&
      isfinite(left) && left >= 0.0f && left <= 1.0f &&
      isfinite(center) && center >= 0.0f && center <= 1.0f &&
      isfinite(right) && right >= 0.0f && right <= 1.0f;
  const bool danger = left > 0.9f && center > 0.9f && right > 0.9f;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_IDLE) {
    if (!rising) return;
    if (!valid) { state->reason = TINYMPC_ESPNET_STRAIGHT_STALE; return; }
    if (danger) { state->reason = TINYMPC_ESPNET_STRAIGHT_DANGER; return; }
    if (fabsf(measured_speed) > 0.15f) {
      state->reason = TINYMPC_ESPNET_STRAIGHT_MOVING;
      return;
    }
    state->phase = TINYMPC_ESPNET_STRAIGHT_RUN;
    state->reason = TINYMPC_ESPNET_STRAIGHT_NONE;
  }
  if (!valid) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_STALE, measured_s);
    return;
  }
  if (danger) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DANGER, measured_s);
    return;
  }
  if (!run) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_CANCEL, measured_s);
    return;
  }
  state->elapsed += dt;
  if (state->elapsed >= 15.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_TIMEOUT, measured_s);
    return;
  }
  if (measured_s >= 5.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DISTANCE, measured_s);
    return;
  }
  tinympcEspnetStraightAdvanceReference(&state->s, &state->v, measured_s, dt);
  if (state->s >= 5.0f)
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DISTANCE, measured_s);
}
/* Vision-independent braking test: measured forward velocity triggers braking.
 * The measured travel and timeout bounds remain active if speed is not reached.
 * Like the vision test, a rejected rising edge is consumed and STOP is latched. */
static inline void tinympcSpeedBrakeStep(
    tinympcEspnetStraightState *state, float measured_s, float total_speed,
    float forward_speed, bool run, float dt)
{
  const bool rising = run && !state->previous_run;
  state->previous_run = run;
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_STOP) return;
  if (!isfinite(measured_s) || !isfinite(total_speed) ||
      !isfinite(forward_speed) || !isfinite(state->s) ||
      !isfinite(state->v) || !isfinite(state->elapsed) ||
      !isfinite(dt) || dt < 0.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_INVALID, measured_s);
    return;
  }
  if (state->phase == TINYMPC_ESPNET_STRAIGHT_IDLE) {
    if (!rising) return;
    if (fabsf(total_speed) > 0.15f) {
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
  state->elapsed += dt;
  if (state->elapsed >= 15.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_TIMEOUT, measured_s);
    return;
  }
  if (fabsf(measured_s) >= 1.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_DISTANCE, measured_s);
    return;
  }
  if (forward_speed >= 5.0f) {
    tinympcEspnetStraightStop(state, TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED, measured_s);
    return;
  }
  // Advance from the accepted start, retaining position error when tracking lags.
  state->s += 5.0f * fminf(dt, 0.1f);
  state->v = 5.0f;
}

/* Prediction only: the actual measured travel is bounded in Step, so a future
 * reference crossing the distance limit must not prematurely brake a slower real aircraft. */
static inline void tinympcSpeedBrakeAdvanceReference(float *s, float *v, float dt)
{
  if (!isfinite(dt) || dt <= 0.0f) return;
  *s += 5.0f * fminf(dt, 0.1f);
  *v = 5.0f;
}
#endif
