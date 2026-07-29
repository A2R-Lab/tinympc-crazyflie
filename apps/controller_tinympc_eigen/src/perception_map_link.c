#include "perception_map_link.h"

#include "FreeRTOS.h"
#include "log.h"
#include "task.h"

#include <string.h>

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static volatile uint32_t g_mapSeqLock = 0;
static uint8_t g_obstaclePresence[PERCEPTION_MAP_CELLS];
static uint8_t g_inverseRange[PERCEPTION_MAP_CELLS];
static uint8_t g_uncertainty[PERCEPTION_MAP_CELLS];
static uint8_t g_gateOpening[PERCEPTION_MAP_CELLS];
static uint32_t g_captureTick = 0;
static volatile uint32_t g_rxTick = 0;
static volatile uint32_t g_rxOk = 0;
static volatile uint32_t g_crcErr = 0;
static volatile uint32_t g_badRx = 0;
static volatile uint32_t g_invalidRx = 0;
static volatile uint32_t g_duplicateRx = 0;
static volatile uint32_t g_sequenceGaps = 0;
static uint16_t g_lastWireSequence = 0;

static uint8_t getNibble(const uint8_t *packed, int index) {
  const uint8_t value = packed[index >> 1];
  return (index & 1) ? (value >> 4) : (value & 0x0f);
}

void perceptionMapLinkInit(void) {
  memset(g_obstaclePresence, 0, sizeof(g_obstaclePresence));
  memset(g_inverseRange, 0, sizeof(g_inverseRange));
  memset(g_uncertainty, 0, sizeof(g_uncertainty));
  memset(g_gateOpening, 0, sizeof(g_gateOpening));
}

void perceptionMapLinkNoteBadRx(void) {
  g_badRx++;
}

void perceptionMapLinkNoteCrcErr(void) {
  g_crcErr++;
}

bool perceptionMapLinkPublishFromRx(const perception_map_msg_t *msg) {
  const perception_map_payload_t *p = &msg->p;
  if (p->version != PERCEPTION_MAP_WIRE_VERSION ||
      p->width != PERCEPTION_MAP_W || p->height != PERCEPTION_MAP_H ||
      p->sequence == 0) {
    g_invalidRx++;
    return false;
  }
  if (p->sequence == g_lastWireSequence) {
    g_duplicateRx++;
    return false;
  }
  if (g_lastWireSequence != 0 &&
      p->sequence != (uint16_t)(g_lastWireSequence + 1) &&
      !(g_lastWireSequence == UINT16_MAX && p->sequence == 1)) {
    g_sequenceGaps++;
  }

  uint32_t lock = g_mapSeqLock + 1;
  g_mapSeqLock = lock;
  COMPILER_BARRIER();
  for (int cell = 0; cell < PERCEPTION_MAP_CELLS; ++cell) {
    g_obstaclePresence[cell] = getNibble(p->packed_u4, 4 * cell) * 17u;
    g_inverseRange[cell] = getNibble(p->packed_u4, 4 * cell + 1) * 17u;
    g_uncertainty[cell] = getNibble(p->packed_u4, 4 * cell + 2) * 17u;
    g_gateOpening[cell] = getNibble(p->packed_u4, 4 * cell + 3) * 17u;
  }
  g_captureTick = p->stm32_ts_echo;
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_mapSeqLock = lock + 1;
  g_lastWireSequence = p->sequence;
  g_rxOk++;
  return true;
}

bool perceptionMapLinkGetLatest(uint8_t obstacle_presence[PERCEPTION_MAP_CELLS],
                                uint8_t inverse_range[PERCEPTION_MAP_CELLS],
                                uint8_t uncertainty[PERCEPTION_MAP_CELLS],
                                uint8_t gate_opening[PERCEPTION_MAP_CELLS],
                                uint32_t *out_age_ms,
                                uint32_t *out_stm32_capture_tick,
                                uint32_t *out_sample) {
  uint32_t before, after, rxTick, captureTick;
  do {
    before = g_mapSeqLock;
    COMPILER_BARRIER();
    memcpy(obstacle_presence, g_obstaclePresence, PERCEPTION_MAP_CELLS);
    memcpy(inverse_range, g_inverseRange, PERCEPTION_MAP_CELLS);
    memcpy(uncertainty, g_uncertainty, PERCEPTION_MAP_CELLS);
    memcpy(gate_opening, g_gateOpening, PERCEPTION_MAP_CELLS);
    rxTick = g_rxTick;
    captureTick = g_captureTick;
    COMPILER_BARRIER();
    after = g_mapSeqLock;
  } while ((before & 1u) || before != after);
  if (g_rxOk == 0) return false;
  if (out_age_ms) {
    *out_age_ms = (xTaskGetTickCount() - rxTick) * portTICK_PERIOD_MS;
  }
  if (out_stm32_capture_tick) *out_stm32_capture_tick = captureTick;
  if (out_sample) *out_sample = before >> 1;
  return true;
}

LOG_GROUP_START(perceptMap)
LOG_ADD(LOG_UINT32, rxOk, &g_rxOk)
LOG_ADD(LOG_UINT32, crcErr, &g_crcErr)
LOG_ADD(LOG_UINT32, badRx, &g_badRx)
LOG_ADD(LOG_UINT32, invalid, &g_invalidRx)
LOG_ADD(LOG_UINT32, duplicate, &g_duplicateRx)
LOG_ADD(LOG_UINT32, seqGap, &g_sequenceGaps)
LOG_GROUP_STOP(perceptMap)
