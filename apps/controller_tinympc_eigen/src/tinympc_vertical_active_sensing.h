#ifndef TINYMPC_VERTICAL_ACTIVE_SENSING_H
#define TINYMPC_VERTICAL_ACTIVE_SENSING_H

#include <math.h>
#include <stddef.h>

typedef struct {
  float amplitude_m;
  float period_s;
} TinyMpcVerticalActiveSensingConfig;

/* NanoFlowNet-style active sensing uses a deliberately simple altitude
 * square wave. Flight time zero is the low phase; the high phase begins
 * halfway through each period. */
static inline float tinyMpcVerticalActiveSensingOffset(
    const TinyMpcVerticalActiveSensingConfig *config,
    float elapsed_flight_time_s) {
  if (config == NULL || !isfinite(config->amplitude_m) ||
      !isfinite(config->period_s) || config->amplitude_m < 0.0f ||
      config->period_s <= 0.0f || !isfinite(elapsed_flight_time_s)) {
    return 0.0f;
  }

  const float nonnegative_time_s =
      elapsed_flight_time_s > 0.0f ? elapsed_flight_time_s : 0.0f;
  const float phase_s = fmodf(nonnegative_time_s, config->period_s);
  return phase_s < 0.5f * config->period_s
      ? -config->amplitude_m
      : config->amplitude_m;
}

#endif
