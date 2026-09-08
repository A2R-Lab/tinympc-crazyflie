#ifndef TINYMPC_BRAKE_PROFILE_H
#define TINYMPC_BRAKE_PROFILE_H

#include <math.h>

typedef struct {
  float s; /* Displacement along the initial measured velocity direction, m. */
  float v; /* Nonnegative reference speed, m/s. */
} tinympcBrakeProfile;

/* Exact integration of dv/dt = -6 above 1 m/s, then -6*sqrt(v/1).
 * Below 1 m/s, sqrt(v) decreases linearly: speed and deceleration reach
 * zero together in finite time. Integrating displacement over the same
 * intervals keeps live references and horizon previews consistent.
 * Invalid inputs leave the reference unchanged; callers must initialize
 * finite s and nonnegative finite v from a valid state estimate.
 */
static inline void tinympcBrakeProfileStep(tinympcBrakeProfile *profile,
                                          float dt)
{
  if (!profile || !isfinite(dt) || dt <= 0.0f ||
      !isfinite(profile->s) || !isfinite(profile->v) || profile->v < 0.0f) {
    return;
  }

  if (profile->v > 1.0f) {
    const float to_taper = (profile->v - 1.0f) / 6.0f;
    const float step = fminf(dt, to_taper);
    const float next_v = dt >= to_taper ? 1.0f : profile->v - 6.0f * step;
    profile->s += 0.5f * (profile->v + next_v) * step;
    profile->v = next_v;
    dt -= step;
  }

  if (profile->v > 0.0f && dt > 0.0f) {
    const float root_v = sqrtf(profile->v);
    const float to_stop = root_v / 3.0f;
    const float step = fminf(dt, to_stop);
    const float next_root = dt >= to_stop ? 0.0f : root_v - 3.0f * step;
    profile->s += step * (profile->v + root_v * next_root +
                          next_root * next_root) / 3.0f;
    profile->v = next_root * next_root;
  }
}

#endif
