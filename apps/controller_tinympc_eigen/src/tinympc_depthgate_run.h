#ifndef TINYMPC_DEPTHGATE_RUN_H
#define TINYMPC_DEPTHGATE_RUN_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct {
  float origin_x, origin_y, forward_x, forward_y, elapsed, travel;
  bool running;
  uint8_t reason;
} TinyDepthGateRun;

static inline bool tinyDepthGateRunConfig(float distance, float timeout) {
  return isfinite(distance) && distance >= 0.f && distance <= 20.f &&
      isfinite(timeout) && timeout >= 1.f && timeout <= 120.f;
}

static inline void tinyDepthGateRunReset(TinyDepthGateRun *s) {
  s->origin_x=s->origin_y=s->forward_x=s->forward_y=0.f;
  s->elapsed=s->travel=0.f; s->running=false; s->reason=0;
}

/* Signed measured displacement, not commanded speed integrated over time.
 * Completion stays latched even if RUN remains asserted or options change. */
static inline uint8_t tinyDepthGateRunUpdate(TinyDepthGateRun *s, bool run,
    float x, float y, float yaw, float dt, float distance, float timeout) {
  if (!run) { tinyDepthGateRunReset(s); return 0; }
  const bool config_ok=tinyDepthGateRunConfig(distance,timeout);
  if (!config_ok) return s->reason ? s->reason : (s->reason=1);
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw) || !isfinite(dt) || dt<0.f)
    return s->reason ? s->reason : (s->reason=8);
  if (!s->running) {
    s->origin_x=x; s->origin_y=y;
    s->forward_x=cosf(yaw); s->forward_y=sinf(yaw);
    s->running=true;
  } else s->elapsed+=dt;
  s->travel=(x-s->origin_x)*s->forward_x+(y-s->origin_y)*s->forward_y;
  if (s->reason) return s->reason;
  if (distance>0.f && s->travel>=distance) return s->reason=9;
  if (s->elapsed>=timeout) return s->reason=4;
  return 0;
}
#endif
