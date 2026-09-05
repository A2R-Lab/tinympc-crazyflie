#ifndef TINYMPC_LQR_FEEDBACK_H
#define TINYMPC_LQR_FEEDBACK_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tinympc_brushless_actuator.h"

#define TINYMPC_LQR_STATE_COUNT 12u
#define TINYMPC_LQR_MOTOR_COUNT 4u

/* Apply a row-major, motor-by-state feedback gain in physical thrust units.
 * Output is committed only after the complete calculation has validated. */
typedef struct {
  float requested_n[TINYMPC_LQR_MOTOR_COUNT];
  float bounded_n[TINYMPC_LQR_MOTOR_COUNT];
  uint8_t high_mask, low_mask;
} TinyMpcLqrThrustDiagnostic;

/* Force-only stage: callers may project requested_n before converting once. */
static inline bool tinyMpcLqrFeedbackThrust(
    const float state[TINYMPC_LQR_STATE_COUNT],
    const float target[TINYMPC_LQR_STATE_COUNT],
    const float feedforward_delta_n[TINYMPC_LQR_MOTOR_COUNT],
    const float gain[TINYMPC_LQR_MOTOR_COUNT * TINYMPC_LQR_STATE_COUNT],
    const float hover_n[TINYMPC_LQR_MOTOR_COUNT],
    const float max_thrust_n[TINYMPC_LQR_MOTOR_COUNT],
    uint8_t *saturation_mask,
    TinyMpcLqrThrustDiagnostic *diagnostic) {
  if (state == NULL || target == NULL || feedforward_delta_n == NULL ||
      gain == NULL || hover_n == NULL || max_thrust_n == NULL ||
      diagnostic == NULL || saturation_mask == NULL) {
    return false;
  }

  float error[TINYMPC_LQR_STATE_COUNT];
  for (unsigned state_index = 0; state_index < TINYMPC_LQR_STATE_COUNT;
       ++state_index) {
    if (!isfinite(state[state_index]) || !isfinite(target[state_index])) {
      return false;
    }
    error[state_index] = state[state_index] - target[state_index];
    if (!isfinite(error[state_index])) {
      return false;
    }
  }

  TinyMpcLqrThrustDiagnostic next_diagnostic = {{0}, {0}, 0u, 0u};
  uint8_t next_saturation_mask = 0u;
  for (unsigned motor = 0; motor < TINYMPC_LQR_MOTOR_COUNT; ++motor) {
    if (!isfinite(feedforward_delta_n[motor]) || !isfinite(hover_n[motor]) ||
        !isfinite(max_thrust_n[motor]) || max_thrust_n[motor] < 0.0f) {
      return false;
    }

    float feedback_n = 0.0f;
    for (unsigned state_index = 0; state_index < TINYMPC_LQR_STATE_COUNT;
         ++state_index) {
      const float gain_value =
          gain[motor * TINYMPC_LQR_STATE_COUNT + state_index];
      if (!isfinite(gain_value)) {
        return false;
      }
      const float correction_n = gain_value * error[state_index];
      if (!isfinite(correction_n)) {
        return false;
      }
      feedback_n += correction_n;
      if (!isfinite(feedback_n)) {
        return false;
      }
    }

    const float baseline_n = hover_n[motor] + feedforward_delta_n[motor];
    if (!isfinite(baseline_n)) {
      return false;
    }
    const float requested_thrust_n = baseline_n - feedback_n;
    if (!isfinite(requested_thrust_n)) {
      return false;
    }

    float bounded_thrust_n = requested_thrust_n;
    if (bounded_thrust_n < 0.0f) {
      bounded_thrust_n = 0.0f;
      next_saturation_mask |= (uint8_t)(1u << motor);
      next_diagnostic.low_mask |= (uint8_t)(1u << motor);
    } else if (bounded_thrust_n > max_thrust_n[motor]) {
      bounded_thrust_n = max_thrust_n[motor];
      next_saturation_mask |= (uint8_t)(1u << motor);
      next_diagnostic.high_mask |= (uint8_t)(1u << motor);
    }

    next_diagnostic.requested_n[motor] = requested_thrust_n;
    next_diagnostic.bounded_n[motor] = bounded_thrust_n;
  }

  *saturation_mask = next_saturation_mask;
  *diagnostic = next_diagnostic;
  return true;
}

/* Commit commands only after all four final physical forces validate. */
static inline bool tinyMpcLqrThrustCommands(
    const float bounded_n[TINYMPC_LQR_MOTOR_COUNT],
    float commands[TINYMPC_LQR_MOTOR_COUNT]) {
  if (bounded_n == NULL || commands == NULL) return false;
  float next_commands[TINYMPC_LQR_MOTOR_COUNT];
  for (unsigned motor = 0; motor < TINYMPC_LQR_MOTOR_COUNT; ++motor) {
    if (!isfinite(bounded_n[motor]) || bounded_n[motor] < 0.0f) return false;
    next_commands[motor] = tinyMpcBrushlessCommand(bounded_n[motor]);
    if (!isfinite(next_commands[motor])) return false;
  }
  for (unsigned motor = 0; motor < TINYMPC_LQR_MOTOR_COUNT; ++motor)
    commands[motor] = next_commands[motor];
  return true;
}

static inline bool tinyMpcLqrFeedbackDetailed(
    const float state[TINYMPC_LQR_STATE_COUNT],
    const float target[TINYMPC_LQR_STATE_COUNT],
    const float feedforward_delta_n[TINYMPC_LQR_MOTOR_COUNT],
    const float gain[TINYMPC_LQR_MOTOR_COUNT * TINYMPC_LQR_STATE_COUNT],
    const float hover_n[TINYMPC_LQR_MOTOR_COUNT],
    const float max_thrust_n[TINYMPC_LQR_MOTOR_COUNT],
    float commands[TINYMPC_LQR_MOTOR_COUNT], uint8_t *saturation_mask,
    TinyMpcLqrThrustDiagnostic *diagnostic) {
  if (commands == NULL || saturation_mask == NULL) return false;
  TinyMpcLqrThrustDiagnostic next_diagnostic;
  uint8_t next_saturation_mask;
  if (!tinyMpcLqrFeedbackThrust(state, target, feedforward_delta_n, gain,
      hover_n, max_thrust_n, &next_saturation_mask, &next_diagnostic) ||
      !tinyMpcLqrThrustCommands(next_diagnostic.bounded_n, commands)) return false;
  *saturation_mask = next_saturation_mask;
  if (diagnostic) *diagnostic = next_diagnostic;
  return true;
}

static inline bool tinyMpcLqrFeedback(
    const float state[TINYMPC_LQR_STATE_COUNT],
    const float target[TINYMPC_LQR_STATE_COUNT],
    const float feedforward_delta_n[TINYMPC_LQR_MOTOR_COUNT],
    const float gain[TINYMPC_LQR_MOTOR_COUNT * TINYMPC_LQR_STATE_COUNT],
    const float hover_n[TINYMPC_LQR_MOTOR_COUNT],
    const float max_thrust_n[TINYMPC_LQR_MOTOR_COUNT],
    float commands[TINYMPC_LQR_MOTOR_COUNT], uint8_t *saturation_mask) {
  return tinyMpcLqrFeedbackDetailed(state, target, feedforward_delta_n, gain,
      hover_n, max_thrust_n, commands, saturation_mask, NULL);
}

#endif
