/*
 * gate8_link.c
 * GAP8 AI-deck -> STM32 gate-corner vision link over UART1/USART3.
 *
 * Scan for the 4-byte header, read the packed payload + CRC32, validate, then
 * publish via a seqlock. v1 is receive-only and exposes the corners over LOG
 * for cfclient verification. C file, not the C++ controller TU, so LOG_*
 * macros compile.
 */
#include "gate8_link.h"
#include "flowdeck_obstacle_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart1.h"     /* uart1Init, uart1GetDataWithDefaultTimeout, USART3 */
#include "crc32.h"
#include "log.h"
#include "debug.h"

#include <string.h>

#define GATE8_BAUD       115200
#define GATE8_PAYLOAD_N  ((int)(sizeof(gate8_payload_t) + sizeof(uint32_t)))  /* payload + crc */
#define FLOW_OBS_PAYLOAD_N ((int)(sizeof(flow_obstacle_payload_t) + sizeof(uint32_t)))

/* Published state. Seqlock: RX task writes, controller and LOG read. */
static volatile uint32_t g_seq      = 0;   /* odd while writing, even when stable */
static float             g_corners[GATE8_N_CORNERS] = {0};
static uint32_t          g_ts       = 0;   /* stm32_timestamp from the message */
static volatile uint32_t g_rxTick   = 0;   /* tick at last valid message */
static volatile uint32_t g_rxOk     = 0;   /* accepted messages */
static volatile uint32_t g_crcErr   = 0;   /* CRC mismatches */
static volatile uint32_t g_badRx    = 0;   /* short or timed-out reads */
static volatile uint32_t g_rawBytes = 0;   /* total bytes seen on the UART */

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static void gate8PublishCorners(const gate8_msg_t *msg) {
  uint32_t s = g_seq + 1;
  g_seq = s;                                  /* mark odd, write in progress */
  COMPILER_BARRIER();
  memcpy(g_corners, msg->p.corner, sizeof(g_corners));
  g_ts = msg->p.stm32_timestamp;
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_seq = s + 1;                              /* mark even, stable */
  g_rxOk++;
}

static bool headerMatches(const uint8_t window[GATE8_HEADER_LEN], const char *header) {
  return memcmp(window, header, GATE8_HEADER_LEN) == 0;
}

static bool readBytes(uint8_t *dst, int n) {
  for (int i = 0; i < n; i++) {
    if (!uart1GetDataWithDefaultTimeout(dst++)) {
      return false;
    }
  }
  return true;
}

static void gate8RxTask(void *arg) {
  (void)arg;
  gate8_msg_t msg;
  flow_obstacle_msg_t flow_msg;
  uint8_t sync[GATE8_HEADER_LEN] = {0};

  while (1) {
    /* Sync to either 4-byte header. The AI-deck UART is shared by gate corners
     * and obstacle-flow sectors, so one RX task must dispatch both message types. */
    bool is_gate = false;
    bool is_flow = false;
    while (!is_gate && !is_flow) {
      uint8_t b;
      if (!uart1GetDataWithDefaultTimeout(&b)) { continue; }
      g_rawBytes++;
      memmove(sync, sync + 1, GATE8_HEADER_LEN - 1);
      sync[GATE8_HEADER_LEN - 1] = b;
      is_gate = headerMatches(sync, GATE8_MSG_HEADER);
      is_flow = headerMatches(sync, FLOW_OBS_MSG_HEADER);
    }

    if (is_flow) {
      memcpy(flow_msg.header, FLOW_OBS_MSG_HEADER, FLOW_OBS_HEADER_LEN);
      if (!readBytes((uint8_t *)&flow_msg.p, FLOW_OBS_PAYLOAD_N)) {
        flowObstacleLinkNoteBadRx();
        continue;
      }
      uint32_t crc = crc32CalculateBuffer(&flow_msg, FLOW_OBS_HEADER_LEN + sizeof(flow_obstacle_payload_t));
      if (crc != flow_msg.checksum) {
        flowObstacleLinkNoteCrcErr();
        continue;
      }
      flowObstacleLinkPublishFromRx(&flow_msg);
      continue;
    }

    memcpy(msg.header, GATE8_MSG_HEADER, GATE8_HEADER_LEN);
    if (!readBytes((uint8_t *)&msg.p, GATE8_PAYLOAD_N)) {
      g_badRx++;
      continue;
    }

    uint32_t crc = crc32CalculateBuffer(&msg, GATE8_HEADER_LEN + sizeof(gate8_payload_t));
    if (crc != msg.checksum) {
      g_crcErr++;
      continue;
    }

    gate8PublishCorners(&msg);
  }
}

void gate8LinkInit(void) {
  uart1Init(GATE8_BAUD);   /* USART3, the GAP8 deck UART */
  flowObstacleLinkInit();
  xTaskCreate(gate8RxTask, "GATE8RX", 2 * configMINIMAL_STACK_SIZE,
              NULL, tskIDLE_PRIORITY + 2, NULL);
  DEBUG_PRINT("vision link: UART1/USART3@%d started\n", GATE8_BAUD);
}

bool gate8LinkGetLatestSeq(float corners[GATE8_N_CORNERS], uint32_t *out_age_ms,
                           uint32_t *out_sample) {
  uint32_t s1, s2, rxTick;
  do {
    s1 = g_seq;
    COMPILER_BARRIER();
    memcpy(corners, g_corners, sizeof(float) * GATE8_N_CORNERS);
    rxTick = g_rxTick;
    COMPILER_BARRIER();
    s2 = g_seq;
  } while ((s1 & 1u) || s1 != s2);   /* retry if mid-write or changed */

  if (g_rxOk == 0) return false;
  if (out_age_ms) *out_age_ms = (xTaskGetTickCount() - rxTick) * portTICK_PERIOD_MS;
  /* g_seq advances by 2 per accepted message and is captured inside the seqlock,
   * so it identifies the sample just copied (unlike g_rxOk, which is bumped after
   * the lock is released). */
  if (out_sample) *out_sample = s1 >> 1;
  return true;
}

bool gate8LinkGetLatest(float corners[GATE8_N_CORNERS], uint32_t *out_age_ms) {
  return gate8LinkGetLatestSeq(corners, out_age_ms, 0);
}

/* LOG: corners and link health in cfclient. */
LOG_GROUP_START(gate8)
LOG_ADD(LOG_FLOAT, c0, &g_corners[0])
LOG_ADD(LOG_FLOAT, c1, &g_corners[1])
LOG_ADD(LOG_FLOAT, c2, &g_corners[2])
LOG_ADD(LOG_FLOAT, c3, &g_corners[3])
LOG_ADD(LOG_FLOAT, c4, &g_corners[4])
LOG_ADD(LOG_FLOAT, c5, &g_corners[5])
LOG_ADD(LOG_FLOAT, c6, &g_corners[6])
LOG_ADD(LOG_FLOAT, c7, &g_corners[7])
LOG_ADD(LOG_UINT32, rxOk,   &g_rxOk)
LOG_ADD(LOG_UINT32, crcErr, &g_crcErr)
LOG_ADD(LOG_UINT32, badRx,  &g_badRx)
LOG_ADD(LOG_UINT32, raw,    &g_rawBytes)
LOG_GROUP_STOP(gate8)
