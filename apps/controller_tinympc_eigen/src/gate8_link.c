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
#include "perception_map_link.h"
#include "sequential_obstacle_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart1.h"     /* uart1Init, uart1GetDataWithDefaultTimeout, USART3 */
#include "crc32.h"
#include "deck.h"
#include "log.h"
#include "debug.h"
#include "system.h"

#include <math.h>
#include <string.h>

#define GATE8_BAUD       115200
#define GATE8_PAYLOAD_N  ((int)(sizeof(gate8_payload_t) + sizeof(uint32_t)))  /* payload + crc */
#define PERCEPTION_MAP_PAYLOAD_N \
  ((int)(sizeof(perception_map_payload_t) + sizeof(uint32_t)))
#define SEQUENTIAL_OBSTACLE_PAYLOAD_N \
  ((int)(sizeof(sequential_obstacle_payload_t) + sizeof(uint32_t)))

/* Published state. Seqlock: RX task writes, controller and LOG read. */
static volatile uint32_t g_seq      = 0;   /* odd while writing, even when stable */
static float             g_corners[GATE8_N_CORNERS] = {0};
static uint32_t          g_ts       = 0;   /* stm32_timestamp from the message */
static volatile uint32_t g_rxTick   = 0;   /* tick at last valid message */
static volatile uint32_t g_rxOk     = 0;   /* accepted messages */
static volatile uint32_t g_crcErr   = 0;   /* CRC mismatches */
static volatile uint32_t g_badRx    = 0;   /* short or timed-out reads */
static volatile uint32_t g_rawBytes = 0;   /* total bytes seen on the UART */
static volatile uint32_t g_dupRx    = 0;   /* duplicate producer timestamp */
static volatile uint32_t g_invalidRx = 0;  /* CRC-valid, invalid numeric data */
static uint32_t          g_queueDrops = 0;
static uint32_t          g_stackFreeWords = 0;
static TaskHandle_t      g_rxTaskHandle = NULL;
static volatile uint32_t g_linkResets = 0;
static TickType_t        g_lastValidTick = 0;
static bool              g_everValid = false;
static uint32_t          g_lastProducerTs = 0;

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static bool gate8PublishCorners(const gate8_msg_t *msg) {
  for (uint8_t i = 0; i < GATE8_N_CORNERS; i++) {
    /* Permit moderate off-frame predictions but reject a corrupt/saturated
     * network output before it can look like a "gate clipped, commit" event. */
    const float lower = (i & 1u) ? -48.0f : -80.0f;
    const float upper = (i & 1u) ? 144.0f : 240.0f;
    if (!isfinite(msg->p.corner[i]) ||
        msg->p.corner[i] < lower || msg->p.corner[i] > upper) {
      g_invalidRx++;
      return false;
    }
  }
  /* Timestamp zero denotes the legacy producer and remains accepted. */
  if (msg->p.stm32_timestamp != 0 &&
      msg->p.stm32_timestamp == g_lastProducerTs) {
    g_dupRx++;
    return false;
  }
  uint32_t s = g_seq + 1;
  g_seq = s;                                  /* mark odd, write in progress */
  COMPILER_BARRIER();
  memcpy(g_corners, msg->p.corner, sizeof(g_corners));
  g_ts = msg->p.stm32_timestamp;
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_seq = s + 1;                              /* mark even, stable */
  g_rxOk++;
  if (msg->p.stm32_timestamp != 0) {
    g_lastProducerTs = msg->p.stm32_timestamp;
  }
  return true;
}

static bool headerMatches(const uint8_t window[GATE8_HEADER_LEN], const char *header) {
  return memcmp(window, header, GATE8_HEADER_LEN) == 0;
}

static bool readBytes(uint8_t *dst, int n) {
  for (int i = 0; i < n; i++) {
    if (!uart1GetDataWithDefaultTimeout(dst++)) {
      return false;
    }
    g_rawBytes++;
  }
  return true;
}

static void resetAiDeck(void) {
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(M2T(100));
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
  g_lastValidTick = xTaskGetTickCount();
  g_linkResets++;
}

static void gate8RxTask(void *arg) {
  (void)arg;
  gate8_msg_t msg;
  perception_map_msg_t map_msg;
  sequential_obstacle_msg_t sequential_msg;
  uint8_t sync[GATE8_HEADER_LEN] = {0};

  systemWaitStart();

  /* CONFIG_DECK_AI is deliberately off to avoid the stock CPX/UART2 stack,
   * so this link owns the reset sequence that driver would otherwise perform.
   * UART1 and its queue are already live before GAP8 is released. */
  resetAiDeck();

  while (1) {
    /* The AI-deck UART carries gate corners and neural danger maps. */
    bool is_gate = false;
    bool is_map = false;
    bool is_sequential = false;
    while (!is_gate && !is_map && !is_sequential) {
      uint8_t b;
      if (!uart1GetDataWithDefaultTimeout(&b)) {
        const TickType_t now = xTaskGetTickCount();
        const TickType_t timeout = g_everValid ? M2T(2000) : M2T(10000);
        if ((now - g_lastValidTick) > timeout) {
          resetAiDeck();
          memset(sync, 0, sizeof(sync));
        }
        continue;
      }
      g_rawBytes++;
      memmove(sync, sync + 1, GATE8_HEADER_LEN - 1);
      sync[GATE8_HEADER_LEN - 1] = b;
      is_gate = headerMatches(sync, GATE8_MSG_HEADER);
      is_map = headerMatches(sync, PERCEPTION_MAP_MSG_HEADER);
      is_sequential = headerMatches(sync, SEQUENTIAL_OBSTACLE_MSG_HEADER);
    }

    if (is_sequential) {
      memcpy(sequential_msg.header, SEQUENTIAL_OBSTACLE_MSG_HEADER,
             SEQUENTIAL_OBSTACLE_HEADER_LEN);
      if (!readBytes((uint8_t *)&sequential_msg.p,
                     SEQUENTIAL_OBSTACLE_PAYLOAD_N)) {
        sequentialObstacleLinkNoteBadRx();
        continue;
      }
      uint32_t crc = crc32CalculateBuffer(
          &sequential_msg, SEQUENTIAL_OBSTACLE_HEADER_LEN +
                               sizeof(sequential_obstacle_payload_t));
      if (crc != sequential_msg.checksum) {
        sequentialObstacleLinkNoteCrcErr();
        continue;
      }
      if (sequentialObstacleLinkPublishFromRx(&sequential_msg)) {
        g_lastValidTick = xTaskGetTickCount();
        g_everValid = true;
      }
      continue;
    }

    if (is_map) {
      memcpy(map_msg.header, PERCEPTION_MAP_MSG_HEADER,
             PERCEPTION_MAP_HEADER_LEN);
      if (!readBytes((uint8_t *)&map_msg.p, PERCEPTION_MAP_PAYLOAD_N)) {
        perceptionMapLinkNoteBadRx();
        continue;
      }
      uint32_t crc = crc32CalculateBuffer(
          &map_msg,
          PERCEPTION_MAP_HEADER_LEN + sizeof(perception_map_payload_t));
      if (crc != map_msg.checksum) {
        perceptionMapLinkNoteCrcErr();
        continue;
      }
      if (perceptionMapLinkPublishFromRx(&map_msg)) {
        g_lastValidTick = xTaskGetTickCount();
        g_everValid = true;
      }
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

    if (gate8PublishCorners(&msg)) {
      g_lastValidTick = xTaskGetTickCount();
      g_everValid = true;
    }
  }
}

void gate8LinkInit(void) {
  uart1Init(GATE8_BAUD);   /* USART3, the GAP8 deck UART */
  perceptionMapLinkInit();
  sequentialObstacleLinkInit();
  const BaseType_t taskCreated =
      xTaskCreate(gate8RxTask, "GATE8RX", 2 * configMINIMAL_STACK_SIZE,
                  NULL, tskIDLE_PRIORITY + 2, &g_rxTaskHandle);
  configASSERT(taskCreated == pdPASS);
  DEBUG_PRINT("vision link: UART1/USART3@%d started\n", GATE8_BAUD);
}


bool gate8LinkGetLatestSeqTimed(float corners[GATE8_N_CORNERS],
                               uint32_t *out_age_ms, uint32_t *out_sample,
                               uint32_t *out_stm32_capture_tick) {
  g_queueDrops = uart1QueueDrops();
  if (g_rxTaskHandle) {
    g_stackFreeWords = uxTaskGetStackHighWaterMark(g_rxTaskHandle);
  }
  uint32_t s1, s2, rxTick, captureTick;
  do {
    s1 = g_seq;
    COMPILER_BARRIER();
    memcpy(corners, g_corners, sizeof(float) * GATE8_N_CORNERS);
    rxTick = g_rxTick;
    captureTick = g_ts;
    COMPILER_BARRIER();
    s2 = g_seq;
  } while ((s1 & 1u) || s1 != s2);   /* retry if mid-write or changed */

  if (g_rxOk == 0) return false;
  if (out_age_ms) *out_age_ms = (xTaskGetTickCount() - rxTick) * portTICK_PERIOD_MS;
  /* g_seq advances by 2 per accepted message and is captured inside the seqlock,
   * so it identifies the sample just copied (unlike g_rxOk, which is bumped after
   * the lock is released). */
  if (out_sample) *out_sample = s1 >> 1;
  if (out_stm32_capture_tick) *out_stm32_capture_tick = captureTick;
  return true;
}

bool gate8LinkGetLatestSeq(float corners[GATE8_N_CORNERS], uint32_t *out_age_ms,
                           uint32_t *out_sample) {
  return gate8LinkGetLatestSeqTimed(corners, out_age_ms, out_sample, 0);
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
LOG_ADD(LOG_UINT32, dupRx,  &g_dupRx)
LOG_ADD(LOG_UINT32, invalid, &g_invalidRx)
LOG_ADD(LOG_UINT32, qDrop,  &g_queueDrops)
LOG_ADD(LOG_UINT32, stackFree, &g_stackFreeWords)
LOG_ADD(LOG_UINT32, resets, &g_linkResets)
LOG_GROUP_STOP(gate8)
