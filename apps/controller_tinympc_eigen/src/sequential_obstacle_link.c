#include "sequential_obstacle_link.h"
#include "tinympc_vision_residual_authority.h"
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
static volatile uint32_t threat_lock;
static SequentialObstacleThreatObservation latest_threat;
static volatile uint32_t latest_threat_rx_tick;
static volatile uint32_t accepted_threat_packets;
static uint16_t latest_threat_sequence;
static volatile uint32_t gate_lock;
static SequentialObstacleGateObservation latest_gate;
static volatile uint32_t latest_gate_rx_tick;
static volatile uint32_t accepted_gate_packets;
static uint16_t latest_gate_sequence;
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
  latest_observation.has_navigation_command = false;
  latest_observation.has_residual_reference = false;
  latest_observation.has_square_opening = false;
  latest_observation.has_collision_probability = false;
  latest_observation.has_normalized_yaw_rate = false;
  latest_observation.square_opening_visible_probability = 0.0f;
  latest_observation.lateral_reference_rate_mps = 0.0f;
  latest_observation.vertical_reference_rate_mps = 0.0f;
  latest_observation.progress_speed_scale = 1.0f;
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
    float gate_cx_normalized, float gate_cy_normalized,
    const TinyRacerVisionV4Payload *v4_payload) {
  const uint16_t known_flags = TINYRACER_VISION_HAS_METRIC_CLEARANCE |
      TINYRACER_VISION_HAS_SECTOR_DANGER | TINYRACER_VISION_GATE_VALID |
      TINYRACER_VISION_HAS_NAVIGATION_COMMAND |
      (v4_payload != NULL ? TINYRACER_VISION_HAS_RESIDUAL_REFERENCE : 0u);
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
  const bool has_residual_reference = v4_payload != NULL &&
      (payload->flags & TINYRACER_VISION_HAS_RESIDUAL_REFERENCE) != 0;
  const TinyMpcVisionResidualAuthority residual_authority =
      tinyMpcVisionResidualFullAuthorityV1();
  if (has_residual_reference &&
      !tinyMpcVisionResidualPacketWithinAuthority(
          &residual_authority,
          v4_payload->lateral_reference_rate_mps,
          v4_payload->vertical_reference_rate_mps,
          v4_payload->progress_speed_scale)) {
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
  latest_observation.has_residual_reference = has_residual_reference;
  latest_observation.has_square_opening = false;
  latest_observation.has_collision_probability = false;
  latest_observation.has_normalized_yaw_rate = false;
  latest_observation.square_opening_visible_probability = 0.0f;
  latest_observation.gate_valid =
      (payload->flags & TINYRACER_VISION_GATE_VALID) != 0;
  memcpy(latest_observation.clearance_m, payload->clearance_m,
         sizeof(latest_observation.clearance_m));
  memcpy(latest_observation.confidence, payload->confidence,
         sizeof(latest_observation.confidence));
  latest_observation.danger_probability[TINYRACER_DANGER_LEFT] =
      payload->danger_probability[0];
  latest_observation.danger_probability[TINYRACER_DANGER_CENTER] = fmaxf(
      payload->danger_probability[1], payload->danger_probability[2]);
  latest_observation.danger_probability[TINYRACER_DANGER_RIGHT] =
      payload->danger_probability[3];
  latest_observation.steering_command = payload->steering_command;
  latest_observation.collision_probability = payload->collision_probability;
  if (has_residual_reference) {
    latest_observation.lateral_reference_rate_mps =
        v4_payload->lateral_reference_rate_mps;
    latest_observation.vertical_reference_rate_mps =
        v4_payload->vertical_reference_rate_mps;
    latest_observation.progress_speed_scale = v4_payload->progress_speed_scale;
  } else {
    latest_observation.lateral_reference_rate_mps = 0.0f;
    latest_observation.vertical_reference_rate_mps = 0.0f;
    latest_observation.progress_speed_scale = 1.0f;
  }
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
      LEGACY_GATE_FY_NORMALIZED, 0.5f, 0.5f, NULL);
}

static bool acceptV3Packet(const TinyRacerVisionV3Packet *packet) {
  return acceptVisionPayload(
      &packet->payload.base,
      packet->payload.gate_fx_normalized,
      packet->payload.gate_fy_normalized,
      packet->payload.gate_cx_normalized,
      packet->payload.gate_cy_normalized, NULL);
}

static bool acceptV4Packet(const TinyRacerVisionV4Packet *packet) {
  return acceptVisionPayload(
      &packet->payload.base.base,
      packet->payload.base.gate_fx_normalized,
      packet->payload.base.gate_fy_normalized,
      packet->payload.base.gate_cx_normalized,
      packet->payload.base.gate_cy_normalized, &packet->payload);
}

static bool acceptV5Packet(const TinyRacerVisionV5Packet *packet) {
  TinyRacerVisionV4Payload legacy;
  memset(&legacy, 0, sizeof(legacy));
  TinyRacerVisionV2Payload *base = &legacy.base.base;
  base->source_timestamp_ms = packet->payload.source_timestamp_ms;
  base->sequence = packet->payload.sequence;
  base->flags = packet->payload.flags;
  memcpy(base->clearance_m, packet->payload.clearance_m, sizeof(base->clearance_m));
  memcpy(base->confidence, packet->payload.confidence, sizeof(base->confidence));
  base->danger_probability[0] = packet->payload.danger_probability[0];
  base->danger_probability[1] = packet->payload.danger_probability[1];
  base->danger_probability[2] = packet->payload.danger_probability[1];
  base->danger_probability[3] = packet->payload.danger_probability[2];
  base->steering_command = packet->payload.steering_command;
  base->collision_probability = packet->payload.collision_probability;
  memcpy(base->gate_corners_xy, packet->payload.gate_corners_xy,
         sizeof(base->gate_corners_xy));
  base->gate_confidence = packet->payload.gate_confidence;
  legacy.base.gate_fx_normalized = packet->payload.gate_fx_normalized;
  legacy.base.gate_fy_normalized = packet->payload.gate_fy_normalized;
  legacy.base.gate_cx_normalized = packet->payload.gate_cx_normalized;
  legacy.base.gate_cy_normalized = packet->payload.gate_cy_normalized;
  legacy.lateral_reference_rate_mps = packet->payload.lateral_reference_rate_mps;
  legacy.vertical_reference_rate_mps = packet->payload.vertical_reference_rate_mps;
  legacy.progress_speed_scale = packet->payload.progress_speed_scale;
  return acceptVisionPayload(base, legacy.base.gate_fx_normalized,
      legacy.base.gate_fy_normalized, legacy.base.gate_cx_normalized,
      legacy.base.gate_cy_normalized, &legacy);
}

static bool acceptV6Packet(const TinyRacerVisionV6Packet *packet) {
  const TinyRacerVisionV6Payload *payload = &packet->payload;
  const uint16_t known_flags = TINYRACER_VISION_V6_HAS_COLLISION |
                               TINYRACER_VISION_V6_HAS_RECOVERY;
  if (payload->sequence == 0u || payload->flags != known_flags) {
    invalid_packets++;
    return false;
  }
  int selected_sector = TINYRACER_DANGER_LEFT;
  float maximum_collision = -1.0f;
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    if (!isfinite(payload->collision_probability[sector]) ||
        payload->collision_probability[sector] < 0.0f ||
        payload->collision_probability[sector] > 1.0f ||
        !isfinite(payload->rail_present_probability[sector]) ||
        payload->rail_present_probability[sector] < 0.0f ||
        payload->rail_present_probability[sector] > 1.0f ||
        !isfinite(payload->pass_right_probability[sector]) ||
        payload->pass_right_probability[sector] < 0.0f ||
        payload->pass_right_probability[sector] > 1.0f) {
      invalid_packets++;
      return false;
    }
    if (payload->collision_probability[sector] > maximum_collision) {
      maximum_collision = payload->collision_probability[sector];
      selected_sector = sector;
    }
  }
  if (payload->sequence == latest_sequence) {
    return false;
  }

  const float selected_rail =
      payload->rail_present_probability[selected_sector];
  const bool rail_present = selected_rail >= 0.45f;
  const bool pass_right =
      payload->pass_right_probability[selected_sector] >= 0.50f;
  const uint32_t lock = sequence_lock + 1;
  sequence_lock = lock;
  COMPILER_BARRIER();
  memset(&latest_observation, 0, sizeof(latest_observation));
  latest_observation.valid = true;
  latest_observation.source_timestamp = payload->source_timestamp_ms;
  latest_observation.sequence = payload->sequence;
  latest_observation.has_sector_danger = true;
  latest_observation.has_navigation_command = true;
  latest_observation.progress_speed_scale = 1.0f;
  latest_observation.gate_valid = rail_present;
  latest_observation.gate_confidence = selected_rail;
  latest_observation.steering_command = rail_present
      ? (pass_right ? -1.0f : 1.0f) : 0.0f;
  latest_observation.collision_probability = maximum_collision;
  memcpy(latest_observation.danger_probability,
         payload->collision_probability,
         sizeof(latest_observation.danger_probability));
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    latest_observation.clearance_m[sector] = 6.0f;
  }
  latest_observation.gate_fx_normalized = LEGACY_GATE_FX_NORMALIZED;
  latest_observation.gate_fy_normalized = LEGACY_GATE_FY_NORMALIZED;
  latest_observation.gate_cx_normalized = 0.5f;
  latest_observation.gate_cy_normalized = 0.5f;
  latest_rx_tick = xTaskGetTickCount();
  latest_sequence = payload->sequence;
  COMPILER_BARRIER();
  sequence_lock = lock + 1;
  accepted_packets++;
  return true;
}

static bool acceptV7Packet(const TinyRacerVisionV7Packet *packet) {
  const TinyRacerVisionV7Payload *payload = &packet->payload;
  const uint16_t known_flags = TINYRACER_VISION_V7_HAS_COLLISION |
      TINYRACER_VISION_V7_HAS_SQUARE_OPENING;
  if (payload->sequence == 0u || payload->flags != known_flags ||
      !isfinite(payload->square_opening_visible_probability) ||
      payload->square_opening_visible_probability < 0.0f ||
      payload->square_opening_visible_probability > 1.0f) {
    invalid_packets++;
    return false;
  }
  float maximum_collision = -1.0f;
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    const float probability = payload->collision_probability[sector];
    if (!isfinite(probability) || probability < 0.0f || probability > 1.0f) {
      invalid_packets++;
      return false;
    }
    maximum_collision = fmaxf(maximum_collision, probability);
  }
  if (payload->sequence == latest_sequence) {
    return false;
  }

  const uint32_t lock = sequence_lock + 1;
  sequence_lock = lock;
  COMPILER_BARRIER();
  memset(&latest_observation, 0, sizeof(latest_observation));
  latest_observation.valid = true;
  latest_observation.source_timestamp = payload->source_timestamp_ms;
  latest_observation.sequence = payload->sequence;
  latest_observation.has_sector_danger = true;
  latest_observation.has_square_opening = true;
  latest_observation.progress_speed_scale = 1.0f;
  latest_observation.collision_probability = maximum_collision;
  latest_observation.square_opening_visible_probability =
      payload->square_opening_visible_probability;
  memcpy(latest_observation.danger_probability,
         payload->collision_probability,
         sizeof(latest_observation.danger_probability));
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    latest_observation.clearance_m[sector] = 6.0f;
  }
  latest_observation.gate_fx_normalized = LEGACY_GATE_FX_NORMALIZED;
  latest_observation.gate_fy_normalized = LEGACY_GATE_FY_NORMALIZED;
  latest_observation.gate_cx_normalized = 0.5f;
  latest_observation.gate_cy_normalized = 0.5f;
  latest_rx_tick = xTaskGetTickCount();
  latest_sequence = payload->sequence;
  COMPILER_BARRIER();
  sequence_lock = lock + 1;
  accepted_packets++;
  return true;
}

static bool acceptV8Packet(const TinyRacerVisionV8Packet *packet) {
  const TinyRacerVisionV8Payload *payload = &packet->payload;
  const uint16_t known_flags = TINYRACER_VISION_V8_HAS_NAVIGATION |
      TINYRACER_VISION_V8_HAS_SQUARE_OPENING;
  if (payload->sequence == 0u || payload->flags != known_flags ||
      !isfinite(payload->normalized_yaw_rate) ||
      payload->normalized_yaw_rate < -1.0f ||
      payload->normalized_yaw_rate > 1.0f ||
      !isfinite(payload->collision_probability) ||
      payload->collision_probability < 0.0f ||
      payload->collision_probability > 1.0f ||
      !isfinite(payload->square_opening_visible_probability) ||
      payload->square_opening_visible_probability < 0.0f ||
      payload->square_opening_visible_probability > 1.0f) {
    invalid_packets++;
    return false;
  }
  if (payload->sequence == latest_sequence) {
    return false;
  }

  const uint32_t lock = sequence_lock + 1;
  sequence_lock = lock;
  COMPILER_BARRIER();
  memset(&latest_observation, 0, sizeof(latest_observation));
  latest_observation.valid = true;
  latest_observation.source_timestamp = payload->source_timestamp_ms;
  latest_observation.sequence = payload->sequence;
  latest_observation.has_navigation_command = true;
  latest_observation.has_collision_probability = true;
  latest_observation.has_normalized_yaw_rate = true;
  latest_observation.has_square_opening = true;
  latest_observation.progress_speed_scale = 1.0f;
  latest_observation.steering_command = payload->normalized_yaw_rate;
  latest_observation.collision_probability = payload->collision_probability;
  latest_observation.square_opening_visible_probability =
      payload->square_opening_visible_probability;
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    latest_observation.clearance_m[sector] = 6.0f;
  }
  latest_observation.gate_fx_normalized = LEGACY_GATE_FX_NORMALIZED;
  latest_observation.gate_fy_normalized = LEGACY_GATE_FY_NORMALIZED;
  latest_observation.gate_cx_normalized = 0.5f;
  latest_observation.gate_cy_normalized = 0.5f;
  latest_rx_tick = xTaskGetTickCount();
  latest_sequence = payload->sequence;
  COMPILER_BARRIER();
  sequence_lock = lock + 1;
  accepted_packets++;
  return true;
}

static bool acceptThreatPacket(const TinyRacerOlgmdThreatPacket *packet) {
  const TinyRacerOlgmdThreatPayload *payload = &packet->payload;
  if (payload->frame_sequence == 0u || payload->imminent_threat > 1u ||
      payload->reserved != 0u) {
    invalid_packets++;
    return false;
  }
  if (accepted_threat_packets != 0u &&
      payload->frame_sequence == latest_threat_sequence) {
    return false;
  }
  const uint32_t lock = threat_lock + 1u;
  threat_lock = lock;
  COMPILER_BARRIER();
  latest_threat.valid = true;
  latest_threat.source_timestamp_ms = payload->source_timestamp_ms;
  latest_threat.sequence = payload->frame_sequence;
  latest_threat.imminent_threat = payload->imminent_threat != 0u;
  latest_threat_rx_tick = xTaskGetTickCount();
  latest_threat_sequence = payload->frame_sequence;
  COMPILER_BARRIER();
  threat_lock = lock + 1u;
  accepted_threat_packets++;
  return true;
}

static bool acceptGatePacket(const TinyRacerGateObservationPacket *packet) {
  const TinyRacerGateObservationPayload *payload = &packet->payload;
  if (payload->frame_sequence == 0u || payload->inference_valid > 1u ||
      payload->reserved != 0u) {
    invalid_packets++;
    return false;
  }
  for (int index = 0; index < 8; ++index) {
    if (!isfinite(payload->corners_xy[index]) ||
        payload->corners_xy[index] < -1.0f ||
        payload->corners_xy[index] > 2.0f) {
      invalid_packets++;
      return false;
    }
  }
  if (accepted_gate_packets != 0u &&
      payload->frame_sequence == latest_gate_sequence) {
    return false;
  }
  const uint32_t lock = gate_lock + 1u;
  gate_lock = lock;
  COMPILER_BARRIER();
  latest_gate.valid = payload->inference_valid != 0u;
  latest_gate.source_timestamp_ms = payload->source_timestamp_ms;
  latest_gate.sequence = payload->frame_sequence;
  memcpy(latest_gate.corners_xy, payload->corners_xy,
         sizeof(latest_gate.corners_xy));
  latest_gate_rx_tick = xTaskGetTickCount();
  latest_gate_sequence = payload->frame_sequence;
  COMPILER_BARRIER();
  gate_lock = lock + 1u;
  accepted_gate_packets++;
  return true;
}

static void sequentialObstacleRxTask(void *parameters) {
  (void)parameters;
  sequential_obstacle_packet_t packet;
  TinyRacerVisionV2Packet packet_v2;
  TinyRacerVisionV3Packet packet_v3;
  TinyRacerVisionV4Packet packet_v4;
  TinyRacerVisionV5Packet packet_v5;
  TinyRacerVisionV6Packet packet_v6;
  TinyRacerVisionV7Packet packet_v7;
  TinyRacerVisionV8Packet packet_v8;
  TinyRacerOlgmdThreatPacket threat_packet;
  TinyRacerGateObservationPacket gate_packet;
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
    const bool is_v4 = memcmp(header_window, TINYRACER_VISION_V4_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_v5 = memcmp(header_window, TINYRACER_VISION_V5_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_v6 = memcmp(header_window, TINYRACER_VISION_V6_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_v7 = memcmp(header_window, TINYRACER_VISION_V7_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_v8 = memcmp(header_window, TINYRACER_VISION_V8_HEADER,
                              TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_threat = memcmp(header_window, TINYRACER_OLGMD_THREAT_HEADER,
                                  TINYRACER_VISION_HEADER_LEN) == 0;
    const bool is_gate = memcmp(header_window, TINYRACER_GATE_OBSERVATION_HEADER,
                                TINYRACER_VISION_HEADER_LEN) == 0;
    if (!is_v1 && !is_v2 && !is_v3 && !is_v4 && !is_v5 && !is_v6 &&
        !is_v7 && !is_v8 && !is_threat && !is_gate) {
      continue;
    }

    uint8_t *remainder;
    size_t remainder_size;
    if (is_gate) {
      memcpy(gate_packet.header, TINYRACER_GATE_OBSERVATION_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&gate_packet.payload;
      remainder_size = sizeof(gate_packet.payload) + sizeof(gate_packet.checksum);
    } else if (is_threat) {
      memcpy(threat_packet.header, TINYRACER_OLGMD_THREAT_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&threat_packet.payload;
      remainder_size = sizeof(threat_packet.payload) +
          sizeof(threat_packet.checksum);
    } else if (is_v8) {
      memcpy(packet_v8.header, TINYRACER_VISION_V8_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v8.payload;
      remainder_size = sizeof(packet_v8.payload) + sizeof(packet_v8.checksum);
    } else if (is_v7) {
      memcpy(packet_v7.header, TINYRACER_VISION_V7_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v7.payload;
      remainder_size = sizeof(packet_v7.payload) + sizeof(packet_v7.checksum);
    } else if (is_v6) {
      memcpy(packet_v6.header, TINYRACER_VISION_V6_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v6.payload;
      remainder_size = sizeof(packet_v6.payload) + sizeof(packet_v6.checksum);
    } else if (is_v5) {
      memcpy(packet_v5.header, TINYRACER_VISION_V5_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v5.payload;
      remainder_size = sizeof(packet_v5.payload) + sizeof(packet_v5.checksum);
    } else if (is_v4) {
      memcpy(packet_v4.header, TINYRACER_VISION_V4_HEADER,
             TINYRACER_VISION_HEADER_LEN);
      remainder = (uint8_t *)&packet_v4.payload;
      remainder_size = sizeof(packet_v4.payload) + sizeof(packet_v4.checksum);
    } else if (is_v3) {
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
    const uint32_t checksum = is_gate
        ? crc32CalculateBuffer(&gate_packet,
              TINYRACER_VISION_HEADER_LEN + sizeof(gate_packet.payload))
        : is_threat
        ? crc32CalculateBuffer(&threat_packet,
              TINYRACER_VISION_HEADER_LEN + sizeof(threat_packet.payload))
        : is_v8
        ? crc32CalculateBuffer(&packet_v8,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v8.payload))
        : is_v7
        ? crc32CalculateBuffer(&packet_v7,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v7.payload))
        : is_v6
        ? crc32CalculateBuffer(&packet_v6,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v6.payload))
        : is_v5
        ? crc32CalculateBuffer(&packet_v5,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v5.payload))
        : is_v4
        ? crc32CalculateBuffer(&packet_v4,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v4.payload))
        : is_v3
        ? crc32CalculateBuffer(&packet_v3,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v3.payload))
        : is_v2
        ? crc32CalculateBuffer(&packet_v2,
              TINYRACER_VISION_HEADER_LEN + sizeof(packet_v2.payload))
        : crc32CalculateBuffer(&packet,
              SEQUENTIAL_OBSTACLE_HEADER_LEN + sizeof(packet.payload));
    const uint32_t expected = is_gate ? gate_packet.checksum
        : (is_threat ? threat_packet.checksum
        : (is_v8 ? packet_v8.checksum
        : (is_v7 ? packet_v7.checksum
        : (is_v6 ? packet_v6.checksum
        : (is_v5 ? packet_v5.checksum
        : (is_v4 ? packet_v4.checksum
        : (is_v3 ? packet_v3.checksum
        : (is_v2 ? packet_v2.checksum : packet.checksum))))))));
    if (checksum != expected) {
      crc_errors++;
      memset(header_window, 0, sizeof(header_window));
      continue;
    }
    if (is_gate) {
      acceptGatePacket(&gate_packet);
    } else if (is_threat) {
      acceptThreatPacket(&threat_packet);
    } else if (is_v8) {
      acceptV8Packet(&packet_v8);
    } else if (is_v7) {
      acceptV7Packet(&packet_v7);
    } else if (is_v6) {
      acceptV6Packet(&packet_v6);
    } else if (is_v5) {
      acceptV5Packet(&packet_v5);
    } else if (is_v4) {
      acceptV4Packet(&packet_v4);
    } else if (is_v3) {
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
  memset(&latest_threat, 0, sizeof(latest_threat));
  memset(&latest_gate, 0, sizeof(latest_gate));
  uart1Init(SEQUENTIAL_OBSTACLE_BAUD);
  const BaseType_t created = xTaskCreate(
      sequentialObstacleRxTask, "SEQRX", 2 * configMINIMAL_STACK_SIZE,
      NULL, tskIDLE_PRIORITY + 2, NULL);
  configASSERT(created == pdPASS);
  initialized = true;
}

bool sequentialObstacleLinkGetLatestThreat(
    SequentialObstacleThreatObservation *observation) {
  uint32_t before;
  uint32_t after;
  uint32_t rx_tick;
  do {
    before = threat_lock;
    COMPILER_BARRIER();
    memcpy(observation, &latest_threat, sizeof(*observation));
    rx_tick = latest_threat_rx_tick;
    COMPILER_BARRIER();
    after = threat_lock;
  } while ((before & 1u) || before != after);
  if (accepted_threat_packets == 0u) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  observation->received_age_ms =
      (xTaskGetTickCount() - rx_tick) * portTICK_PERIOD_MS;
  observation->sample = before >> 1;
  return true;
}

bool sequentialObstacleLinkGetLatestGate(
    SequentialObstacleGateObservation *observation) {
  uint32_t before;
  uint32_t after;
  uint32_t rx_tick;
  do {
    before = gate_lock;
    COMPILER_BARRIER();
    memcpy(observation, &latest_gate, sizeof(*observation));
    rx_tick = latest_gate_rx_tick;
    COMPILER_BARRIER();
    after = gate_lock;
  } while ((before & 1u) || before != after);
  if (accepted_gate_packets == 0u) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  observation->received_age_ms =
      (xTaskGetTickCount() - rx_tick) * portTICK_PERIOD_MS;
  observation->sample = before >> 1;
  return true;
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
LOG_ADD(LOG_UINT32, threatOk, &accepted_threat_packets)
LOG_ADD(LOG_UINT32, gateOk, &accepted_gate_packets)
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
LOG_ADD(LOG_FLOAT, yawNorm, &latest_observation.steering_command)
LOG_ADD(LOG_FLOAT, collision, &latest_observation.collision_probability)
LOG_ADD(LOG_FLOAT, gateCf, &latest_observation.gate_confidence)
LOG_ADD(LOG_FLOAT, sqOpen, &latest_observation.square_opening_visible_probability)
LOG_GROUP_STOP(seqRx)
