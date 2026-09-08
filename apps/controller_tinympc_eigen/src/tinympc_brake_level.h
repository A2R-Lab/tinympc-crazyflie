#ifndef TINYMPC_BRAKE_LEVEL_H
#define TINYMPC_BRAKE_LEVEL_H
#include <math.h>
#include <stdbool.h>
typedef struct { unsigned phase; float elapsed; } TinyBrakeLevel;
/* Angles/rates are in the mission-forward convention: positive tilt/rate
 * accelerates backward. Estimate speed lost while removing that tilt, using
 * a 0.1 s response allowance and a nominal 90 deg/s recovery rate. */
static inline bool tinyBrakeLevelStep(TinyBrakeLevel *s, float speed,
    float brake_angle, float brake_rate, float dt) {
  if (!isfinite(speed) || !isfinite(brake_angle) || !isfinite(brake_rate) ||
      !isfinite(dt) || dt <= 0) return false;
  bool entered = false;
  if (s->phase == 0) {
    float anticipated = fmaxf(0, fminf(1.04719755f, brake_angle + .1f*fmaxf(0,brake_rate)));
    float recovery = .1f + anticipated/1.57079633f;
    float loss = 9.81f*tanf(anticipated)*(.1f + .5f*(recovery-.1f));
    if (speed <= .25f || (anticipated > .08726646f && speed <= loss+.25f)) {
      s->phase = 1; s->elapsed = 0; entered = true;
    }
  }
  if (s->phase == 1) {
    s->elapsed += fminf(dt,.1f);
    if ((fabsf(brake_angle) < .13962634f && fabsf(brake_rate) < .52359878f) ||
        s->elapsed >= .8f) s->phase = 2;
  }
  return entered;
}
#endif
