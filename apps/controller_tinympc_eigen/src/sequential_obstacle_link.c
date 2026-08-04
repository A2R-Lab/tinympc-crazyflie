#include "sequential_obstacle_link.h"

#include "FreeRTOS.h"
#include "log.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static volatile uint32_t g_lock = 0;
static float g_clearance[SEQUENTIAL_OBSTACLE_DIRECTIONS];
static float g_confidence[SEQUENTIAL_OBSTACLE_DIRECTIONS];
static uint8_t g_gateValid = 0;
static uint32_t g_captureTick = 0;
static volatile uint32_t g_rxTick = 0;
static volatile uint32_t g_rxOk = 0;
static volatile uint32_t g_crcErr = 0;
static volatile uint32_t g_badRx = 0;
static volatile uint32_t g_invalidRx = 0;
static volatile uint32_t g_duplicateRx = 0;
static volatile uint32_t g_sequenceGaps = 0;
static uint16_t g_lastSequence = 0;

void sequentialObstacleLinkInit(void) {
  memset(g_clearance, 0, sizeof(g_clearance));
  memset(g_confidence, 0, sizeof(g_confidence));
}

void sequentialObstacleLinkNoteBadRx(void) { g_badRx++; }
void sequentialObstacleLinkNoteCrcErr(void) { g_crcErr++; }

bool sequentialObstacleLinkPublishFromRx(const sequential_obstacle_msg_t *msg) {
  const sequential_obstacle_payload_t *p = &msg->p;
  if (p->sequence == 0 || p->gate_valid > 1) {
    g_invalidRx++;
    return false;
  }
  for (int i = 0; i < SEQUENTIAL_OBSTACLE_DIRECTIONS; ++i) {
    if (!isfinite(p->clearance_m[i]) || p->clearance_m[i] < 0.0f ||
        p->clearance_m[i] > 6.0f || !isfinite(p->confidence[i]) ||
        p->confidence[i] < -8.0f || p->confidence[i] > 12.0f) {
      g_invalidRx++;
      return false;
    }
  }
  if (p->sequence == g_lastSequence) {
    g_duplicateRx++;
    return false;
  }
  if (g_lastSequence != 0 &&
      p->sequence != (uint16_t)(g_lastSequence + 1) &&
      !(g_lastSequence == UINT16_MAX && p->sequence == 1)) {
    g_sequenceGaps++;
  }

  uint32_t lock = g_lock + 1;
  g_lock = lock;
  COMPILER_BARRIER();
  memcpy(g_clearance, p->clearance_m, sizeof(g_clearance));
  memcpy(g_confidence, p->confidence, sizeof(g_confidence));
  g_gateValid = p->gate_valid;
  g_captureTick = p->stm32_timestamp;
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_lock = lock + 1;
  g_lastSequence = p->sequence;
  g_rxOk++;
  return true;
}

bool sequentialObstacleLinkGetLatest(
    float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS],
    float confidence[SEQUENTIAL_OBSTACLE_DIRECTIONS],
    uint8_t *gate_valid, uint32_t *age_ms, uint32_t *stm32_capture_tick,
    uint32_t *sample) {
  uint32_t before, after, rx_tick, capture_tick;
  uint8_t gate;
  do {
    before = g_lock;
    COMPILER_BARRIER();
    memcpy(clearance_m, g_clearance, sizeof(g_clearance));
    memcpy(confidence, g_confidence, sizeof(g_confidence));
    gate = g_gateValid;
    rx_tick = g_rxTick;
    capture_tick = g_captureTick;
    COMPILER_BARRIER();
    after = g_lock;
  } while ((before & 1u) || before != after);
  if (g_rxOk == 0) return false;
  if (gate_valid) *gate_valid = gate;
  if (age_ms) {
    *age_ms = (xTaskGetTickCount() - rx_tick) * portTICK_PERIOD_MS;
  }
  if (stm32_capture_tick) *stm32_capture_tick = capture_tick;
  if (sample) *sample = before >> 1;
  return true;
}

LOG_GROUP_START(seqRx)
LOG_ADD(LOG_UINT32, rxOk, &g_rxOk)
LOG_ADD(LOG_UINT32, crcErr, &g_crcErr)
LOG_ADD(LOG_UINT32, badRx, &g_badRx)
LOG_ADD(LOG_UINT32, invalid, &g_invalidRx)
LOG_ADD(LOG_UINT32, duplicate, &g_duplicateRx)
LOG_ADD(LOG_UINT32, seqGap, &g_sequenceGaps)
LOG_ADD(LOG_FLOAT, d0, &g_clearance[0])
LOG_ADD(LOG_FLOAT, d1, &g_clearance[1])
LOG_ADD(LOG_FLOAT, d2, &g_clearance[2])
LOG_ADD(LOG_FLOAT, d3, &g_clearance[3])
LOG_ADD(LOG_FLOAT, c0, &g_confidence[0])
LOG_ADD(LOG_FLOAT, c1, &g_confidence[1])
LOG_ADD(LOG_FLOAT, c2, &g_confidence[2])
LOG_ADD(LOG_FLOAT, c3, &g_confidence[3])
LOG_GROUP_STOP(seqRx)
