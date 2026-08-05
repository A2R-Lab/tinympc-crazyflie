#ifndef EQUIVALENCE_TASK_H
#define EQUIVALENCE_TASK_H

#include "FreeRTOS.h"

extern TickType_t equivalenceTick;
static inline TickType_t xTaskGetTickCount(void) {
  return equivalenceTick;
}

#endif
