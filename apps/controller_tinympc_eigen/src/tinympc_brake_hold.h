#ifndef TINYMPC_BRAKE_HOLD_H
#define TINYMPC_BRAKE_HOLD_H
#include <stdbool.h>
#include <math.h>

/* Follow measured XY during the initial velocity brake, then latch once.
 * Later drift must create a position error, never move the hold target. */
static inline bool tinympcBrakeHoldUpdate(bool *locked, float *hold_x,
    float *hold_y, float x, float y, float vx, float vy) {
  if (*locked || !isfinite(x) || !isfinite(y) ||
      !isfinite(vx) || !isfinite(vy)) return false;
  *hold_x = x;
  *hold_y = y;
  *locked = hypotf(vx, vy) <= .15f;
  return *locked;
}
#endif
