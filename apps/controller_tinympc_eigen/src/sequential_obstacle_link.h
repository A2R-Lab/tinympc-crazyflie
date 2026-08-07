/* Four-direction sequential perception receiver (GAP8 -> STM32 UART1). */
#ifndef __SEQUENTIAL_OBSTACLE_LINK_H__
#define __SEQUENTIAL_OBSTACLE_LINK_H__

#include <stdbool.h>
#include <stdint.h>

#include "tinyracer_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEQUENTIAL_OBSTACLE_DIRECTIONS TINYRACER_CLEARANCE_SECTORS

void sequentialObstacleLinkInit(void);
bool sequentialObstacleLinkGetLatest(TinyRacerPerceptionObservation *observation);

#ifdef __cplusplus
}
#endif
#endif
