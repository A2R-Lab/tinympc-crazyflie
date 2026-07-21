/*
 * flowdeck_obstacle_link.c
 * Published state for AI-deck obstacle-flow sector messages.
 *
 * The UART bytes are consumed by gate8_link.c because gate corners and flow sectors
 * share USART3. This module only owns validation counters, seqlock publication, and
 * a copy API for the controller.
 */
#include "flowdeck_obstacle_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"

#include <string.h>

static volatile uint32_t g_seq = 0;
static flow_obstacle_payload_t g_payload;
static volatile uint32_t g_rxTick = 0;
static volatile uint32_t g_rxOk = 0;
static volatile uint32_t g_crcErr = 0;
static volatile uint32_t g_badRx = 0;

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

void flowObstacleLinkInit(void) {
  memset(&g_payload, 0, sizeof(g_payload));
}

void flowObstacleLinkPublishFromRx(const flow_obstacle_msg_t *msg) {
  uint32_t s = g_seq + 1;
  g_seq = s;
  COMPILER_BARRIER();
  memcpy(&g_payload, &msg->p, sizeof(g_payload));
  if (g_payload.n_sectors > FLOW_OBS_SECT_MAX) {
    g_payload.n_sectors = FLOW_OBS_SECT_MAX;
  }
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_seq = s + 1;
  g_rxOk++;
}

void flowObstacleLinkNoteBadRx(void) {
  g_badRx++;
}

void flowObstacleLinkNoteCrcErr(void) {
  g_crcErr++;
}

bool flowObstacleLinkGetLatest(flow_obstacle_payload_t *out,
                               uint32_t *out_age_ms,
                               uint32_t *out_sample) {
  uint32_t s1, s2, rxTick;
  do {
    s1 = g_seq;
    COMPILER_BARRIER();
    if (out) {
      memcpy(out, &g_payload, sizeof(*out));
    }
    rxTick = g_rxTick;
    COMPILER_BARRIER();
    s2 = g_seq;
  } while ((s1 & 1u) || s1 != s2);

  if (g_rxOk == 0) {
    return false;
  }
  if (out_age_ms) {
    *out_age_ms = (xTaskGetTickCount() - rxTick) * portTICK_PERIOD_MS;
  }
  if (out_sample) {
    *out_sample = s1 >> 1;
  }
  return true;
}

LOG_GROUP_START(flowObsRx)
LOG_ADD(LOG_UINT32, rxOk,   &g_rxOk)
LOG_ADD(LOG_UINT32, crcErr, &g_crcErr)
LOG_ADD(LOG_UINT32, badRx,  &g_badRx)
LOG_GROUP_STOP(flowObsRx)
