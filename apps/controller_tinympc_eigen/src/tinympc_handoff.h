#ifndef TINYMPC_HANDOFF_H
#define TINYMPC_HANDOFF_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* A completed plan may receive authority only while its state snapshot is
 * recent and every motor command is finite and physically representable.
 * Use the snapshot tick, not publication time. Unsigned subtraction handles
 * tick wrap provided plan lifetimes are shorter than the wrap interval.
 * Arming/supervisor authorization remains the caller's responsibility. */
static inline bool tinyMpcHandoffPlanValid(
    bool has_plan, uint32_t now_tick, uint32_t plan_tick,
    uint32_t max_age_ticks, const float motor_commands[4]) {
  if (!has_plan || motor_commands == 0 ||
      (uint32_t)(now_tick - plan_tick) > max_age_ticks) {
    return false;
  }
  for (unsigned int motor = 0u; motor < 4u; ++motor) {
    if (!isfinite(motor_commands[motor]) || motor_commands[motor] < 0.0f ||
        motor_commands[motor] > 1.0f) {
      return false;
    }
  }
  return true;
}

/* Physical MPC output, not motor PWM. Check BEFORE clipping at publication
 * too: min/max macros can hide a NaN by selecting the other operand. */
static inline bool tinyMpcRatePlanValid(
    bool has_plan, uint32_t now_tick, uint32_t plan_tick,
    uint32_t max_age_ticks, float collective_n, const float rates[3]) {
  if (!has_plan || rates == 0 || !isfinite(collective_n) || collective_n <= 0.0f ||
      (uint32_t)(now_tick - plan_tick) > max_age_ticks) return false;
  for (unsigned axis = 0; axis < 3; ++axis) {
    if (!isfinite(rates[axis])) return false;
  }
  return true;
}

#endif
