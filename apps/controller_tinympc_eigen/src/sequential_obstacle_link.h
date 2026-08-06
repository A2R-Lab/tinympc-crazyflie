/* Four-direction sequential perception receiver (GAP8 -> STM32 UART1). */
#ifndef __SEQUENTIAL_OBSTACLE_LINK_H__
#define __SEQUENTIAL_OBSTACLE_LINK_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEQUENTIAL_OBSTACLE_DIRECTIONS 4

void sequentialObstacleLinkInit(void);
bool sequentialObstacleLinkGetLatest(
    float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS],
    uint32_t *age_ms, uint32_t *sample);

#ifdef __cplusplus
}
#endif
#endif
