#ifndef TINYMPC_LQR_NOMINAL_H
#define TINYMPC_LQR_NOMINAL_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TINYMPC_LQR_NOMINAL_STATE_COUNT 12u
#define TINYMPC_LQR_NOMINAL_INPUT_COUNT 4u
#define TINYMPC_LQR_NOMINAL_INTERVAL_COUNT 6u
#define TINYMPC_LQR_NOMINAL_SAMPLE_PERIOD_MS 10u
#define TINYMPC_LQR_NOMINAL_VALIDITY_MS 60u

typedef struct {
  float state[7][12];
  float input[6][4];
} TinyMpcLqrNominal;

/* Build the finite-horizon nominal rollout from the projected solver inputs.
 * Matrices are row-major. A false result may leave a partial rollout in out. */
static inline bool tinyMpcLqrNominalBuild(
    TinyMpcLqrNominal *out,
    const float x0[TINYMPC_LQR_NOMINAL_STATE_COUNT],
    const float projected[TINYMPC_LQR_NOMINAL_INTERVAL_COUNT]
                         [TINYMPC_LQR_NOMINAL_INPUT_COUNT],
    const float A[TINYMPC_LQR_NOMINAL_STATE_COUNT
                  * TINYMPC_LQR_NOMINAL_STATE_COUNT],
    const float B[TINYMPC_LQR_NOMINAL_STATE_COUNT
                  * TINYMPC_LQR_NOMINAL_INPUT_COUNT],
    const float f[TINYMPC_LQR_NOMINAL_STATE_COUNT]) {
  if (out == NULL || x0 == NULL || projected == NULL || A == NULL ||
      B == NULL || f == NULL) {
    return false;
  }

  for (unsigned state = 0u; state < TINYMPC_LQR_NOMINAL_STATE_COUNT;
       ++state) {
    if (!isfinite(x0[state]) || !isfinite(f[state])) {
      return false;
    }
    out->state[0][state] = x0[state];
  }
  for (unsigned index = 0u;
       index < TINYMPC_LQR_NOMINAL_STATE_COUNT
                   * TINYMPC_LQR_NOMINAL_STATE_COUNT;
       ++index) {
    if (!isfinite(A[index])) {
      return false;
    }
  }
  for (unsigned index = 0u;
       index < TINYMPC_LQR_NOMINAL_STATE_COUNT
                   * TINYMPC_LQR_NOMINAL_INPUT_COUNT;
       ++index) {
    if (!isfinite(B[index])) {
      return false;
    }
  }
  for (unsigned interval = 0u;
       interval < TINYMPC_LQR_NOMINAL_INTERVAL_COUNT; ++interval) {
    for (unsigned input = 0u; input < TINYMPC_LQR_NOMINAL_INPUT_COUNT;
         ++input) {
      if (!isfinite(projected[interval][input])) {
        return false;
      }
      out->input[interval][input] = projected[interval][input];
    }
  }

  for (unsigned interval = 0u;
       interval < TINYMPC_LQR_NOMINAL_INTERVAL_COUNT; ++interval) {
    for (unsigned row = 0u; row < TINYMPC_LQR_NOMINAL_STATE_COUNT; ++row) {
      float next = 0.0f;
      for (unsigned column = 0u;
           column < TINYMPC_LQR_NOMINAL_STATE_COUNT; ++column) {
        const float term = A[row * TINYMPC_LQR_NOMINAL_STATE_COUNT + column]
            * out->state[interval][column];
        if (!isfinite(term)) {
          return false;
        }
        next += term;
        if (!isfinite(next)) {
          return false;
        }
      }
      for (unsigned input = 0u; input < TINYMPC_LQR_NOMINAL_INPUT_COUNT;
           ++input) {
        const float term = B[row * TINYMPC_LQR_NOMINAL_INPUT_COUNT + input]
            * out->input[interval][input];
        if (!isfinite(term)) {
          return false;
        }
        next += term;
        if (!isfinite(next)) {
          return false;
        }
      }
      next += f[row];
      if (!isfinite(next)) {
        return false;
      }
      out->state[interval + 1u][row] = next;
    }
  }
  return true;
}

/* Age zero is the captured state timestamp, NOT publication time. Knots are linearly
 * interpolated; the feedforward is held over each 10 ms input interval. */
static inline bool tinyMpcLqrNominalSample(
    const TinyMpcLqrNominal *nominal, uint32_t age_ms,
    float target[TINYMPC_LQR_NOMINAL_STATE_COUNT],
    float feedforward[TINYMPC_LQR_NOMINAL_INPUT_COUNT]) {
  if (nominal == NULL || target == NULL || feedforward == NULL ||
      age_ms > TINYMPC_LQR_NOMINAL_VALIDITY_MS) {
    return false;
  }

  unsigned interval = age_ms / TINYMPC_LQR_NOMINAL_SAMPLE_PERIOD_MS;
  if (interval >= TINYMPC_LQR_NOMINAL_INTERVAL_COUNT) {
    interval = TINYMPC_LQR_NOMINAL_INTERVAL_COUNT - 1u;
  }
  const unsigned lower_state =
      age_ms == TINYMPC_LQR_NOMINAL_VALIDITY_MS ?
          TINYMPC_LQR_NOMINAL_INTERVAL_COUNT : interval;
  const unsigned upper_state =
      lower_state < TINYMPC_LQR_NOMINAL_INTERVAL_COUNT ?
          lower_state + 1u : lower_state;
  const float fraction =
      age_ms == TINYMPC_LQR_NOMINAL_VALIDITY_MS ? 0.0f :
          (float)(age_ms % TINYMPC_LQR_NOMINAL_SAMPLE_PERIOD_MS)
              / (float)TINYMPC_LQR_NOMINAL_SAMPLE_PERIOD_MS;

  float next_target[TINYMPC_LQR_NOMINAL_STATE_COUNT];
  float next_feedforward[TINYMPC_LQR_NOMINAL_INPUT_COUNT];
  for (unsigned state = 0u; state < TINYMPC_LQR_NOMINAL_STATE_COUNT;
       ++state) {
    const float lower = nominal->state[lower_state][state];
    const float upper = nominal->state[upper_state][state];
    if (!isfinite(lower) || !isfinite(upper)) {
      return false;
    }
    const float delta = upper - lower;
    if (!isfinite(delta)) {
      return false;
    }
    next_target[state] = lower + fraction * delta;
    if (!isfinite(next_target[state])) {
      return false;
    }
  }
  for (unsigned input = 0u; input < TINYMPC_LQR_NOMINAL_INPUT_COUNT;
       ++input) {
    next_feedforward[input] = nominal->input[interval][input];
    if (!isfinite(next_feedforward[input])) {
      return false;
    }
  }

  for (unsigned state = 0u; state < TINYMPC_LQR_NOMINAL_STATE_COUNT;
       ++state) {
    target[state] = next_target[state];
  }
  for (unsigned input = 0u; input < TINYMPC_LQR_NOMINAL_INPUT_COUNT;
       ++input) {
    feedforward[input] = next_feedforward[input];
  }
  return true;
}

#endif
