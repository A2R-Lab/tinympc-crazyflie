#ifndef TINYMPC_GATE_SERVO_H
#define TINYMPC_GATE_SERVO_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Image x points right, image y down; camera/body x is forward and y left.
 * A single rail permits a bounded lateral search, never a guessed gate center.
 * All four labeled corners are needed to confirm the aperture center. */
enum { GATE_SEARCH, GATE_SLOW, GATE_ALIGN, GATE_PASS, GATE_COOLDOWN, GATE_ABORT };
typedef struct {
  bool fresh;
  bool pose_aligned; /* Near-level camera and yaw aligned with passage heading. */
  uint32_t sample;
  float rail[2], x[4], y[4], confidence[4];
  uint8_t edge_mask;
} TinyGateInput;
typedef struct {
  uint8_t phase, matched, centered, reject;
  uint32_t last_sample;
  unsigned acquire_frames, center_frames;
  float elapsed, clear_s, ex, ey, lateral, vertical, forward;
} TinyGateServo;

static inline bool tinyGateUnit(float x) {
  return isfinite(x) && x >= 0 && x <= 1;
}
static inline void tinyGateGeometry(TinyGateServo *s, const TinyGateInput *g) {
  s->matched = s->centered = 0;
  s->ex = s->ey = 0;
  if (!g->fresh) return;
  for (unsigned side = 0; side < 2; ++side) {
    if (!tinyGateUnit(g->rail[side]) || g->rail[side] < .7f) continue;
    for (unsigned c = side; c < 4; c += 2)
      if (tinyGateUnit(g->confidence[c]) && g->confidence[c] >= .02f &&
          tinyGateUnit(g->x[c]) && tinyGateUnit(g->y[c]) &&
          (g->edge_mask & (1u << c))) s->matched |= 1u << side;
  }
  if (s->matched != 3) return;
  for (unsigned c = 0; c < 4; ++c)
    if (!tinyGateUnit(g->confidence[c]) || g->confidence[c] < .02f ||
        !tinyGateUnit(g->x[c]) || !tinyGateUnit(g->y[c])) return;
  /* Reject crossed, collapsed, and severely skewed corner configurations. */
  if (g->x[1]-g->x[0] < .12f || g->x[3]-g->x[2] < .12f ||
      g->y[2]-g->y[0] < .12f || g->y[3]-g->y[1] < .12f ||
      fabsf(g->x[0]-g->x[2]) > .25f || fabsf(g->x[1]-g->x[3]) > .25f ||
      fabsf(g->y[0]-g->y[1]) > .25f || fabsf(g->y[2]-g->y[3]) > .25f) return;
  s->centered = 1; /* Valid center estimate, not yet aligned. */
  s->ex = .25f*(g->x[0]+g->x[1]+g->x[2]+g->x[3])-.5f;
  s->ey = .25f*(g->y[0]+g->y[1]+g->y[2]+g->y[3])-.5f;
}

/* Progress is measured along the original heading from the aligned entry.
 * The one-meter command is odometry, not a monocular estimate of gate depth. */
static inline void tinyGateStep(TinyGateServo *s, const TinyGateInput *g,
    bool running, float speed, float progress, float dt) {
  s->forward = s->lateral = s->vertical = 0;
  if (!running) { *s = (TinyGateServo){0}; return; }
  if (!isfinite(speed) || speed < 0 || !isfinite(progress) ||
      !isfinite(dt) || dt <= 0 || dt > .2f) {
    s->phase = GATE_ABORT; s->reject = 1; return;
  }
  const bool new_frame = g->sample != s->last_sample;
  if (new_frame) s->last_sample = g->sample;
  tinyGateGeometry(s, g);
  if (s->phase == GATE_ABORT) return;
  if (s->phase == GATE_SEARCH) {
    if (!g->fresh || !s->matched) s->acquire_frames = 0;
    else if (new_frame) ++s->acquire_frames;
    if (s->acquire_frames >= 3) { s->phase = GATE_SLOW; s->elapsed = 0; }
    return;
  }
  if (s->phase == GATE_COOLDOWN) {
    s->clear_s = g->fresh && !s->matched ? s->clear_s + dt : 0;
    if (s->clear_s >= 1) { s->phase = GATE_SEARCH; s->acquire_frames = 0; }
    return;
  }
  s->elapsed += dt;
  if (s->elapsed > 10) { s->phase = GATE_ABORT; s->reject = 2; return; }
  if (s->phase == GATE_PASS) {
    if (progress >= 1) { s->phase = GATE_COOLDOWN; s->clear_s = 0; }
    else s->forward = .5f;
    return; /* Gate may leave the camera while passing; collision remains active. */
  }
  if (!g->fresh) { s->phase = GATE_ABORT; s->reject = 3; return; }
  if (s->phase == GATE_SLOW) {
    if (speed < .2f) { s->phase = GATE_ALIGN; s->center_frames = 0; }
    return;
  }
  if (s->centered) {
    s->lateral = -1.0f*s->ex;
    s->vertical = -1.0f*s->ey;
    const float magnitude = hypotf(s->lateral, s->vertical);
    if (magnitude > .5f) { s->lateral *= .5f/magnitude; s->vertical *= .5f/magnitude; }
    if (fabsf(s->ex) < .05f && fabsf(s->ey) < .05f && speed < .15f && g->pose_aligned) {
      if (new_frame) ++s->center_frames;
    } else s->center_frames = 0;
    if (s->center_frames >= 3) {
      s->phase = GATE_PASS; s->elapsed = 0;
      s->lateral = s->vertical = 0; s->forward = .5f;
    }
  } else {
    s->center_frames = 0;
    /* Rail on left => opening to its right; reverse for a right rail. */
    if (s->matched == 1) s->lateral = -.25f;
    if (s->matched == 2) s->lateral = .25f;
  }
}
#endif
