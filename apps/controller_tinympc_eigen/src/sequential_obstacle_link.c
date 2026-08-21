#include "sequential_obstacle_link.h"
#include "tinyracer_vision_packet.h"

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
#define LEGACY_GATE_FX_NORMALIZED (89.15584f / 160.0f)
#define LEGACY_GATE_FY_NORMALIZED (89.46082f / 120.0f)

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
  latest_observation.has_metric_clearance = true;
  latest_observation.has_sector_danger = false;
  latest_observation.gate_valid = packet->payload.gate_valid != 0;
  latest_observation.gate_fx_normalized = LEGACY_GATE_FX_NORMALIZED;
  latest_observation.gate_fy_normalized = LEGACY_GATE_FY_NORMALIZED;
  latest_observation.gate_cx_normalized = 0.5f;
  latest_observation.gate_cy_normalized = 0.5f;
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

static bool acceptVisionPayload(
    const TinyRacerVisionV2Payload *payload,
    float gate_fx_normalized, float gate_fy_normalized,
    float gate_cx_normalized, float gate_cy_normalized) {
  const uint16_t known_flags = TINYRACER_VISION_HAS_METRIC_CLEARANCE |
      TINYRACER_VISION_HAS_SECTOR_DANGER | TINYRACER_VISION_GATE_VALID |
      TINYRACER_VISION_HAS_NAVIGATION_COMMAND;
  if (payload->sequence == 0 || (payload->flags & ~known_flags) != 0) {
    invalid_packets++;
    return false;
  }
  for (int sector = 0; sector < SEQUENTIAL_OBSTACLE_DIRECTIONS; ++sector) {
    if (!isfinite(payload->clearance_m[sector]) ||
        payload->clearance_m[sector] < 0.0f ||
        payload->clearance_m[sector] > 6.0f ||
        !isfinite(payload->confidence[sector]) ||
        !isfinite(payload->danger_probability[sector]) ||
        payload->danger_probability[sector] < 0.0f ||
        payload->danger_probability[sector] > 1.0f) {
      invalid_packets++;
      return false;
    }
  }
  for (int coordinate = 0; coordinate < 8; ++coordinate) {
    if (!isfinite(payload->gate_corners_xy[coordinate]) ||
        payload->gate_corners_xy[coordinate] < 0.0f ||
        payload->gate_corners_xy[coordinate] > 1.0f) {
      invalid_packets++;
      return false;
    }
  }
  if (!isfinite(payload->gate_confidence) ||
      payload->gate_confidence < 0.0f || payload->gate_confidence > 1.0f ||
      !isfinite(payload->steering_command) ||
      payload->steering_command < -1.0f ||
      payload->steering_command > 1.0f ||
      !isfinite(payload->collision_probability) ||
      payload->collision_probability < 0.0f ||
      payload->collision_probability > 1.0f ||
      !isfinite(gate_fx_normalized) || gate_fx_normalized < 0.05f ||
      !isfinite(gate_fy_normalized) || gate_fy_normalized < 0.05f ||
      !isfinite(gate_cx_normalized) || gate_cx_normalized < 0.0f ||
      gate_cx_normalized > 1.0f ||
      !isfinite(gate_cy_normalized) || gate_cy_normalized < 0.0f ||
      gate_cy_normalized > 1.0f) {
    invalid_packets++;
    return false;
  }
  if (payload->sequence == latest_sequence) {
    return false;
  }

  const uint32_t lock = sequence_lock + 1;
  sequence_lock = lock;
  COMPILER_BARRIER();
  latest_observation.valid = true;
  latest_observation.source_timestamp = payload->source_timestamp_ms;
  latest_observation.sequence = payload->sequence;
  latest_observation.has_metric_clearance =
      (payload->flags & TINYRACER_VISION_HAS_METRIC_CLEARANCE) != 0;
  latest_observation.has_sector_danger =
      (payload->flags & TINYRACER_VISION_HAS_SECTOR_DANGER) != 0;
  latest_observation.has_navigation_command =
      (payload->flags & TINYRACER_VISION_HAS_NAVIGATION_COMMAND) != 0;
  latest_observation.gate_valid =
      (payload->flags & TINYRACER_VISION_GATE_VALID) != 0;
  memcpy(latest_observation.clearance_m, payload->clearance_m,
         sizeof(latest_observation.clearance_m));
  memcpy(latest_observation.confidence, payload->confidence,
         sizeof(latest_observation.confidence));
  memcpy(latest_observation.danger_probability, payload->danger_probability,
         sizeof(latest_observation.danger_probability));
  latest_observation.steering_command = payload->steering_command;
  latest_observation.collision_probability = payload->collision_probability;
  memcpy(latest_observation.gate_corners_xy, payload->gate_corners_xy,
         sizeof(latest_observation.gate_corners_xy));
  latest_observation.gate_confidence = payload->gate_confidence;
  latest_observation.gate_fx_normalized = gate_fx_normalized;
  latest_observation.gate_fy_normalized = gate_fy_normalized;
  latest_observation.gate_cx_normalized = gate_cx_normalized;
  latest_observation.gate_cy_normalized = gate_cy_normalized;
  latest_rx_tick = xTaskGetTickCount();
  latest_sequence = payload->sequence;
  COMPILER_BARRIER();
  sequence_lock = lock + 1;
  accepted_packets++;
  return true;
}

static bool acceptV2Packet(const TinyRacerVisionV2Packet *packet) {
  return acceptVisionPayload(
      &packet->payload, LEGACY_GATE_FX_NORMALIZED,
      LEGACY_GATE_FY_NORMALIZED, 0.5f, 0.5f);
}

static bool acceptV3Packet(const TinyRacerVisionV3Packet *packet) {
  return acceptVisionPayload(
      &packet->payload.base,
      packet->payload.gate_fx_normalized,
      packet->payload.gate_fy_normalized,
      packet->payload.gate_cx_normalized,
      packet->payload.gate_cy_normalized);
}

static void sequentialObstacleRxTask(void *parameters) {
  (void)parameters;
  sequential_obstacle_packet_t packet;
  TinyRacerVisionV2Packet packet_v2;
  TinyRacerVisionV3Packet packet_v3;
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
    const bool is_v1 = memcmp(header_window, SEQUENTIAL_OBSTACLE_MSG_HEADER,
                              SEQUENTIAL_OBSTACLE_HEADER_LEN) == 0;
    const bool is_v2 = memcmp(header_window, TINYRACER_VISION_V2_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_v3 = memcmp(header_window, TINYRACER_VISION_V3_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    if (!is_v1 && !is_v2 && !is_v3) {
      continue;
    }

    uint8_t *remainder;
    size_t remainder_size;
    if (is_v3) {
      memcpy(packet_v3.header, TINYRACER_VISION_V3_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v3.payload;
      remainder_size = sizeof(packet_v3.payload) + sizeof(packet_v3.checksum);
    } else if (is_v2) {
      memcpy(packet_v2.header, TINYRACER_VISION_V2_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v2.payload;
      remainder_size = sizeof(packet_v2.payload) + sizeof(packet_v2.checksum);
    } else {
      memcpy(packet.header, SEQUENTIAL_OBSTACLE_MSG_HEADER,
             SEQUENTIAL_OBSTACLE_HEADER_LEN);
      remainder = (uint8_t *)&packet.payload;
      remainder_size = sizeof(packet.payload) + sizeof(packet.checksum);
    }
    if (!readBytes(remainder, (int)remainder_size)) {
      short_reads++;
      memset(header_window, 0, sizeof(header_window));
      continue;
    }
    const uint32_t checksum = is_v3
        ? crc32CalculateBuffer(&packet_v3,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v3.payload))
        : is_v2
        ? crc32CalculateBuffer(&packet_v2,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v2.payload))
        : crc32CalculateBuffer(&packet,
              SEQUENTIAL_OBSTACLE_HEADER_LEN + sizeof(packet.payload));
    const uint32_t expected = is_v3 ? packet_v3.checksum
        : (is_v2 ? packet_v2.checksum : packet.checksum);
    if (checksum != expected) {
      crc_errors++;
      memset(header_window, 0, sizeof(header_window));
      continue;
    }
    if (is_v3) {
      acceptV3Packet(&packet_v3);
    } else if (is_v2) {
      acceptV2Packet(&packet_v2);
    } else {
      acceptPacket(&packet);
    }
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
LOG_ADD(LOG_FLOAT, c0, &latest_observation.confidence[0])
LOG_ADD(LOG_FLOAT, c1, &latest_observation.confidence[1])
LOG_ADD(LOG_FLOAT, c2, &latest_observation.confidence[2])
LOG_ADD(LOG_FLOAT, c3, &latest_observation.confidence[3])
LOG_ADD(LOG_FLOAT, p0, &latest_observation.danger_probability[0])
LOG_ADD(LOG_FLOAT, p1, &latest_observation.danger_probability[1])
LOG_ADD(LOG_FLOAT, p2, &latest_observation.danger_probability[2])
LOG_ADD(LOG_FLOAT, p3, &latest_observation.danger_probability[3])
LOG_ADD(LOG_FLOAT, gateCf, &latest_observation.gate_confidence)
LOG_GROUP_STOP(seqRx)
