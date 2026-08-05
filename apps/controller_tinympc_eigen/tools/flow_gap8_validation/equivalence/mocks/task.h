#ifndef FLOW_EQUIV_TASK_H
#define FLOW_EQUIV_TASK_H

#include <stdint.h>

extern uint32_t flow_equiv_tick_ms;

static inline uint32_t xTaskGetTickCount(void) {
  return flow_equiv_tick_ms;
}

#endif
