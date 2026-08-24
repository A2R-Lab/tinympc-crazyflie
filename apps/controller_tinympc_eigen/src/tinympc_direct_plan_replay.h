#ifndef TINYMPC_DIRECT_PLAN_REPLAY_H
#define TINYMPC_DIRECT_PLAN_REPLAY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t knot;
  uint32_t age_ticks;
  bool valid;
} TinyMpcDirectPlanReplaySelection;

/* Select the zero-order-held input interval represented by a completed MPC
 * horizon. Unsigned subtraction intentionally preserves FreeRTOS tick-wrap
 * semantics. A plan is valid only while one of its input intervals exists. */
static inline TinyMpcDirectPlanReplaySelection tinyMpcDirectPlanReplaySelect(
    uint32_t now_tick, uint32_t plan_tick, uint32_t knot_ticks,
    uint32_t input_knot_count) {
  TinyMpcDirectPlanReplaySelection selection = {0u, 0u, false};
  if (knot_ticks == 0u || input_knot_count == 0u) {
    return selection;
  }
  selection.age_ticks = now_tick - plan_tick;
  selection.knot = selection.age_ticks / knot_ticks;
  selection.valid = selection.knot < input_knot_count;
  if (!selection.valid) {
    selection.knot = input_knot_count - 1u;
  }
  return selection;
}

#endif
