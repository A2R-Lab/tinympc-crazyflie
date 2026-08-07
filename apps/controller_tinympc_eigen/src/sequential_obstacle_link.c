#include "sequential_obstacle_link.h"

#include "FreeRTOS.h"
#include "task.h"
#include "crc32.h"
#include "deck.h"
#include "log.h"
#include "system.h"
#include "uart1.h"

#include <math.h>
#include <string.h>

#define SEQUENTIAL_OBSTACLE_MSG_HEADER "\x90\x19\x8\x38"
#define SEQUENTIAL_OBSTACLE_HEADER_LEN 4
#define SEQUENTIAL_OBSTACLE_BAUD 115200

typedef struct __attribute__((packed)) {
  uint32_t stm32_timestamp;
  uint16_t sequence;
  uint8_t gate_valid;
  uint8_t reserved;
  float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS];
  float confidence[SEQUENTIAL_OBSTACLE_DIRECTIONS];
} sequential_obstacle_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[SEQUENTIAL_OBSTACLE_HEADER_LEN];
  sequential_obstacle_payload_t payload;
  uint32_t checksum;
} sequential_obstacle_packet_t;

_Static_assert(sizeof(sequential_obstacle_packet_t) == 48,
               "sequential obstacle packet ABI changed");

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static volatile uint32_t sequence_lock;
static TinyRacerPerceptionObservation latest_observation;
static volatile uint32_t latest_rx_tick;
static volatile uint32_t accepted_packets;
static volatile uint32_t crc_errors;
static volatile uint32_t invalid_packets;
static volatile uint32_t short_reads;
static uint16_t latest_sequence;
static bool initialized;

static bool readBytes(uint8_t *destination, int count) {
  for (int index = 0; index < count; ++index) {
    if (!uart1GetDataWithDefaultTimeout(destination++)) {
      return false;
    }
  }
  return true;
}

static bool acceptPacket(const sequential_obstacle_packet_t *packet) {
  if (packet->payload.sequence == 0 || packet->payload.gate_valid > 1) {
    invalid_packets++;
    return false;
  }
  for (int direction = 0; direction < SEQUENTIAL_OBSTACLE_DIRECTIONS;
       ++direction) {
    if (!isfinite(packet->payload.clearance_m[direction]) ||
        packet->payload.clearance_m[direction] < 0.0f ||
        packet->payload.clearance_m[direction] > 6.0f ||
        !isfinite(packet->payload.confidence[direction])) {
      invalid_packets++;
      return false;
    }
  }
  if (packet->payload.sequence == latest_sequence) {
    return false;
  }

  const uint32_t lock = sequence_lock + 1;
  sequence_lock = lock;
  COMPILER_BARRIER();
  latest_observation.valid = true;
  latest_observation.source_timestamp = packet->payload.stm32_timestamp;
  latest_observation.sequence = packet->payload.sequence;
  latest_observation.gate_valid = packet->payload.gate_valid != 0;
  memcpy(latest_observation.clearance_m, packet->payload.clearance_m,
         sizeof(latest_observation.clearance_m));
  memcpy(latest_observation.confidence, packet->payload.confidence,
         sizeof(latest_observation.confidence));
  latest_rx_tick = xTaskGetTickCount();
  latest_sequence = packet->payload.sequence;
  COMPILER_BARRIER();
  sequence_lock = lock + 1;
  accepted_packets++;
  return true;
}

static void sequentialObstacleRxTask(void *parameters) {
  (void)parameters;
  sequential_obstacle_packet_t packet;
  uint8_t header_window[SEQUENTIAL_OBSTACLE_HEADER_LEN] = {0};

  systemWaitStart();
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(M2T(100));
  digitalWrite(DECK_GPIO_IO4, HIGH);
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

  while (true) {
    uint8_t byte;
    if (!uart1GetDataWithDefaultTimeout(&byte)) {
      continue;
    }
    memmove(header_window, header_window + 1,
            SEQUENTIAL_OBSTACLE_HEADER_LEN - 1);
    header_window[SEQUENTIAL_OBSTACLE_HEADER_LEN - 1] = byte;
    if (memcmp(header_window, SEQUENTIAL_OBSTACLE_MSG_HEADER,
               SEQUENTIAL_OBSTACLE_HEADER_LEN) != 0) {
      continue;
    }

    memcpy(packet.header, SEQUENTIAL_OBSTACLE_MSG_HEADER,
           SEQUENTIAL_OBSTACLE_HEADER_LEN);
    if (!readBytes((uint8_t *)&packet.payload,
                   sizeof(packet.payload) + sizeof(packet.checksum))) {
      short_reads++;
      memset(header_window, 0, sizeof(header_window));
      continue;
    }
    const uint32_t checksum = crc32CalculateBuffer(
        &packet, SEQUENTIAL_OBSTACLE_HEADER_LEN + sizeof(packet.payload));
    if (checksum != packet.checksum) {
      crc_errors++;
      memset(header_window, 0, sizeof(header_window));
      continue;
    }
    acceptPacket(&packet);
    memset(header_window, 0, sizeof(header_window));
  }
}

void sequentialObstacleLinkInit(void) {
  if (initialized) {
    return;
  }
  memset(&latest_observation, 0, sizeof(latest_observation));
  uart1Init(SEQUENTIAL_OBSTACLE_BAUD);
  const BaseType_t created = xTaskCreate(
      sequentialObstacleRxTask, "SEQRX", 2 * configMINIMAL_STACK_SIZE,
      NULL, tskIDLE_PRIORITY + 2, NULL);
  configASSERT(created == pdPASS);
  initialized = true;
}

bool sequentialObstacleLinkGetLatest(
    TinyRacerPerceptionObservation *observation) {
  uint32_t before;
  uint32_t after;
  uint32_t rx_tick;
  do {
    before = sequence_lock;
    COMPILER_BARRIER();
    memcpy(observation, &latest_observation, sizeof(*observation));
    rx_tick = latest_rx_tick;
    COMPILER_BARRIER();
    after = sequence_lock;
  } while ((before & 1u) || before != after);

  if (accepted_packets == 0) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  observation->received_age_ms =
      (xTaskGetTickCount() - rx_tick) * portTICK_PERIOD_MS;
  observation->sample = before >> 1;
  return true;
}

LOG_GROUP_START(seqRx)
LOG_ADD(LOG_UINT32, rxOk, &accepted_packets)
LOG_ADD(LOG_UINT32, crcErr, &crc_errors)
LOG_ADD(LOG_UINT32, invalid, &invalid_packets)
LOG_ADD(LOG_UINT32, shortRx, &short_reads)
LOG_ADD(LOG_FLOAT, d0, &latest_observation.clearance_m[0])
LOG_ADD(LOG_FLOAT, d1, &latest_observation.clearance_m[1])
LOG_ADD(LOG_FLOAT, d2, &latest_observation.clearance_m[2])
LOG_ADD(LOG_FLOAT, d3, &latest_observation.clearance_m[3])
LOG_GROUP_STOP(seqRx)
