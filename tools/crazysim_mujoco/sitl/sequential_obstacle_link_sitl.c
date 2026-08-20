#include "sequential_obstacle_link.h"

#include <string.h>

void sequentialObstacleLinkInit(void) {
}

bool sequentialObstacleLinkGetLatest(
    TinyRacerPerceptionObservation *observation) {
  memset(observation, 0, sizeof(*observation));
  return false;
}
