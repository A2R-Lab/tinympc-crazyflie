#ifndef TINYMPC_RATE_COMMAND_GUARD_H
#define TINYMPC_RATE_COMMAND_GUARD_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float lower;
  float upper;
} TinyMpcRateCommandBounds;

static inline TinyMpcRateCommandBounds tinyMpcRateReferenceTrimBounds(
    float model_lower, float model_upper,
    float reference_correction, float maximum_trim) {
  const float trim = fmaxf(maximum_trim, 0.0f);
  TinyMpcRateCommandBounds bounds = {
      fmaxf(model_lower, reference_correction - trim),
      fminf(model_upper, reference_correction + trim),
  };
  return bounds;
}

static inline TinyMpcRateCommandBounds tinyMpcRateIntersectSlewBounds(
    TinyMpcRateCommandBounds bounds,
    float previous_correction, float maximum_step) {
  const float step = fmaxf(maximum_step, 0.0f);
  const float slewed_lower = previous_correction - step;
  const float slewed_upper = previous_correction + step;
  const float reference_lower = bounds.lower;
  const float reference_upper = bounds.upper;
  bounds.lower = fmaxf(reference_lower, slewed_lower);
  bounds.upper = fminf(reference_upper, slewed_upper);
  if (bounds.lower > bounds.upper) {
    const float transition = previous_correction < reference_lower
        ? fminf(slewed_upper, reference_lower)
        : fmaxf(slewed_lower, reference_upper);
    bounds.lower = transition;
    bounds.upper = transition;
  }
  return bounds;
}

#ifdef __cplusplus
}
#endif

#endif
