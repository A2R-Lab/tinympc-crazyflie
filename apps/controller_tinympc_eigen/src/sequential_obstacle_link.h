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

typedef struct {
  bool valid;
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  bool imminent_threat;
  uint32_t received_age_ms;
  uint32_t sample;
} SequentialObstacleThreatObservation;

typedef struct {
  bool valid;
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  float corners_xy[8];
  uint32_t received_age_ms;
  uint32_t sample;
} SequentialObstacleGateObservation;

void sequentialObstacleLinkInit(void);
bool sequentialObstacleLinkGetLatest(TinyRacerPerceptionObservation *observation);
bool sequentialObstacleLinkGetLatestThreat(
    SequentialObstacleThreatObservation *observation);
bool sequentialObstacleLinkGetLatestGate(
    SequentialObstacleGateObservation *observation);

#ifdef __cplusplus
}
#endif
#endif
