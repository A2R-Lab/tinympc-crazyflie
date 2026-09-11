/* ESPNet GAP8 collision telemetry only. This module never commands motors. */
#include "espnet_collision_link.h"
#include "depthgate_packet.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crc32.h"
#include "log.h"
#include "system.h"
#include "uart1.h"
#include <string.h>

#define ESPN_PACKET_SIZE 22u
#define ESPN_GATE_PACKET_SIZE 52u
#define PERCEPTION_MAP_PACKET_SIZE 222u
#define ESPN_UART_BAUD 115200
static const uint8_t header[4] = {0x90, 0x19, 0x08, 0x43};
static const uint8_t gate_header[4] = {0x90, 0x19, 0x08, 0x47};
static const uint8_t perception_map_header[4] = {0x90, 0x19, 0x08, 0x37};
static EspnetCollisionObservation latest;
static EspnetGateObservation gate_latest;
static uint32_t gate_received_tick, gate_crc_errors, gate_invalid_packets;
static uint32_t received_tick, crc_errors, invalid_packets;
static bool initialized;

static PerceptionMapObservation perception_map_latest;
static uint32_t perception_map_received_tick;
static uint32_t perception_map_crc_errors, perception_map_invalid_packets;

static DepthGatePayload latest_depthgate;
static uint32_t dg_rx_ok, dg_crc_errors, dg_invalid, dg_stale, dg_short;
static uint32_t dg_rx_tick;
static bool dg_received;
static bool acceptDepthGatePacket(const DepthGatePacket *packet) {
  if (!depthGatePayloadValid(&packet->payload)) { dg_invalid++; return false; }
  /* Reject repeated/backward sequence numbers, including wraparound. After
   * two seconds of silence permit a rebooted GAP8 sequence to recover. */
  uint16_t delta = (uint16_t)(packet->payload.sequence - latest_depthgate.sequence);
  if (dg_received && (xTaskGetTickCount() - dg_rx_tick) * portTICK_PERIOD_MS < 2000u &&
      (delta == 0u || delta >= 32768u)) { dg_stale++; return false; }
  taskENTER_CRITICAL();
  memcpy(&latest_depthgate, &packet->payload, sizeof(latest_depthgate));
  dg_rx_tick = xTaskGetTickCount();
  dg_rx_ok++;
  dg_received = true;
  taskEXIT_CRITICAL();
  return true;
}
bool depthGateLinkGetLatest(DepthGateObservation *observation) {
  if (observation == NULL) return false;
  taskENTER_CRITICAL();
  memset(observation, 0, sizeof(*observation));
  const bool received = dg_received;
  if (received) {
    observation->valid = true;
    memcpy(observation->inverse_depth, latest_depthgate.inverse_depth,
           sizeof(observation->inverse_depth));
    memcpy(observation->corners_xy, latest_depthgate.corners_xy,
           sizeof(observation->corners_xy));
    memcpy(observation->visibility_logits, latest_depthgate.visibility_logits,
           sizeof(observation->visibility_logits));
    observation->source_timestamp_ms = latest_depthgate.source_timestamp_ms;
    observation->sequence = latest_depthgate.sequence;
    observation->status = latest_depthgate.status;
    observation->inference_us = latest_depthgate.inference_us;
    observation->sample = dg_rx_ok;
  }
  observation->received_age_ms = received
      ? (xTaskGetTickCount() - dg_rx_tick) * portTICK_PERIOD_MS : UINT32_MAX;
  taskEXIT_CRITICAL();
  return received;
}
static uint32_t logDepthGateAge(uint32_t timestamp, void *data) {
  (void)timestamp; (void)data;
  return dg_received ? (xTaskGetTickCount() - dg_rx_tick) * portTICK_PERIOD_MS : UINT32_MAX;
}
static logByFunction_t dg_age_log = {.acquireUInt32 = logDepthGateAge, .data = 0};

static uint16_t readLe16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readLe32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
      ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint8_t perceptionMapValue(const PerceptionMapObservation *observation,
                           unsigned x, unsigned y,
                           PerceptionMapChannel channel) {
  if (observation == NULL || x >= PERCEPTION_MAP_WIDTH ||
      y >= PERCEPTION_MAP_HEIGHT || channel > PERCEPTION_MAP_GATE_OPENING) {
    return 0u;
  }
  const unsigned nibble =
      4u * (y * PERCEPTION_MAP_WIDTH + x) + (unsigned)channel;
  const uint8_t packed = observation->packed_u4[nibble >> 1];
  return (nibble & 1u) ? (packed >> 4) : (packed & 0x0fu);
}

static bool acceptPerceptionMapPacket(
    const uint8_t packet[PERCEPTION_MAP_PACKET_SIZE]) {
  if (memcmp(packet, perception_map_header, sizeof(perception_map_header)) != 0) {
    return false;
  }
  if (crc32CalculateBuffer(packet, PERCEPTION_MAP_PACKET_SIZE - 4u) !=
      readLe32(packet + PERCEPTION_MAP_PACKET_SIZE - 4u)) {
    ++perception_map_crc_errors;
    return false;
  }
  const uint16_t sequence = readLe16(packet + 12);
  const uint8_t version = packet[14];
  const uint8_t width = packet[15];
  const uint8_t height = packet[16];
  const uint8_t flags = packet[17];
  if (sequence == 0u || version != PERCEPTION_MAP_WIRE_VERSION ||
      width != PERCEPTION_MAP_WIDTH || height != PERCEPTION_MAP_HEIGHT ||
      (flags & ~PERCEPTION_MAP_FLAG_DANGER_RAW_U8) != 0u) {
    ++perception_map_invalid_packets;
    return false;
  }
  taskENTER_CRITICAL();
  if (sequence == perception_map_latest.sequence) {
    taskEXIT_CRITICAL();
    return false;
  }
  memcpy(perception_map_latest.packed_u4, packet + 18,
         sizeof(perception_map_latest.packed_u4));
  perception_map_latest.gap8_timestamp_us = readLe32(packet + 4);
  perception_map_latest.stm32_timestamp_echo = readLe32(packet + 8);
  perception_map_latest.sequence = sequence;
  perception_map_latest.version = version;
  perception_map_latest.width = width;
  perception_map_latest.height = height;
  perception_map_latest.flags = flags;
  ++perception_map_latest.sample;
  perception_map_received_tick = xTaskGetTickCount();
  taskEXIT_CRITICAL();
  return true;
}

bool perceptionMapLinkGetLatest(PerceptionMapObservation *observation) {
  if (observation == NULL) return false;
  taskENTER_CRITICAL();
  *observation = perception_map_latest;
  const bool received = perception_map_latest.sequence != 0u;
  observation->received_age_ms = received
      ? (xTaskGetTickCount() - perception_map_received_tick) * portTICK_PERIOD_MS
      : UINT32_MAX;
  taskEXIT_CRITICAL();
  return received;
}

static uint32_t logPerceptionMapAge(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  PerceptionMapObservation observation;
  perceptionMapLinkGetLatest(&observation);
  return observation.received_age_ms;
}
static logByFunction_t perception_map_age_log = {
  .acquireUInt32 = logPerceptionMapAge, .data = NULL
};

/* Header + 14-byte payload + CRC32(header,payload), little endian.
 * Payload: source_ms:u32, sequence:u16, probabilities[3]:u16, valid:u8,
 * reserved:u8. Q15 includes both endpoints (0..32768). */
static bool acceptPacket(const uint8_t packet[ESPN_PACKET_SIZE]) {
  if (memcmp(packet, header, sizeof(header)) != 0) return false;
  if (crc32CalculateBuffer(packet, ESPN_PACKET_SIZE - 4u) !=
      readLe32(packet + 18)) {
    ++crc_errors;
    return false;
  }
  const uint16_t sequence = readLe16(packet + 8);
  const uint8_t valid = packet[16];
  if (sequence == 0u || valid > 1u || packet[17] != 0u) {
    ++invalid_packets;
    return false;
  }
  float probability[3];
  for (unsigned i = 0; i < 3; ++i) {
    const uint16_t value = readLe16(packet + 10 + 2 * i);
    if (value > 32768u || (!valid && value != 0u)) {
      ++invalid_packets;
      return false;
    }
    probability[i] = value / 32768.0f;
  }
  taskENTER_CRITICAL();
  if (sequence == latest.sequence) {
    taskEXIT_CRITICAL();
    return false; /* Duplicates must not refresh freshness. */
  }
  latest.valid = valid != 0;
  memcpy(latest.probability, probability, sizeof(probability));
  latest.source_timestamp_ms = readLe32(packet + 4);
  latest.sequence = sequence;
  ++latest.sample;
  received_tick = xTaskGetTickCount();
  taskEXIT_CRITICAL();
  return true;
}

bool espnetCollisionLinkGetLatest(EspnetCollisionObservation *observation) {
  if (observation == NULL) return false;
  taskENTER_CRITICAL();
  *observation = latest;
  const bool received = latest.sequence != 0u;
  observation->received_age_ms = received
      ? (xTaskGetTickCount() - received_tick) * portTICK_PERIOD_MS : UINT32_MAX;
  taskEXIT_CRITICAL();
  return received;
}

/* v1: header, source_ms:u32, seq:u16, version:u8, valid:u8, edgeMask:u8, reserved:u8,
 * rails[2], affordances[3], corners[4][x,y,confidence] as Q15, CRC32. */
static bool acceptGatePacket(const uint8_t packet[ESPN_GATE_PACKET_SIZE]) {
  if (memcmp(packet, gate_header, 4) != 0) return false;
  if (crc32CalculateBuffer(packet, 48) != readLe32(packet + 48)) {
    ++gate_crc_errors;
    return false;
  }
  const uint16_t sequence = readLe16(packet + 8);
  if (!sequence || packet[10] != 1 || packet[11] > 1 || packet[12] > 15 || packet[13] != 0 || (!packet[11] && packet[12])) {
    ++gate_invalid_packets;
    return false;
  }
  float values[17];
  for (unsigned i = 0; i < 17; ++i) {
    uint16_t q = readLe16(packet + 14 + 2 * i);
    if (q > 32768 || (!packet[11] && q != 0)) {
      ++gate_invalid_packets;
      return false;
    }
    values[i] = q / 32768.0f;
  }
  taskENTER_CRITICAL();
  if (sequence == gate_latest.sequence) {
    taskEXIT_CRITICAL();
    return false;
  }
  gate_latest.valid = packet[11] != 0;
  gate_latest.corner_edge_mask = packet[12];
  memcpy(gate_latest.rail_probability, values, 2 * sizeof(float));
  memcpy(gate_latest.affordance_probability, values + 2, 3 * sizeof(float));
  for (unsigned i = 0; i < 4; ++i) {
    gate_latest.corner_x[i] = values[5 + 3 * i];
    gate_latest.corner_y[i] = values[6 + 3 * i];
    gate_latest.corner_confidence[i] = values[7 + 3 * i];
  }
  gate_latest.sequence = sequence;
  gate_latest.source_timestamp_ms = readLe32(packet + 4);
  ++gate_latest.sample;
  gate_received_tick = xTaskGetTickCount();
  taskEXIT_CRITICAL();
  return true;
}

bool espnetGateLinkGetLatest(EspnetGateObservation *observation) {
  if (observation == NULL) return false;
  taskENTER_CRITICAL();
  *observation = gate_latest;
  const bool received = gate_latest.sequence != 0;
  observation->received_age_ms = received
      ? (xTaskGetTickCount() - gate_received_tick) * portTICK_PERIOD_MS : UINT32_MAX;
  taskEXIT_CRITICAL();
  return received;
}

static uint32_t logGateAge(uint32_t timestamp, void *data) {
  (void)timestamp; (void)data;
  EspnetGateObservation observation;
  espnetGateLinkGetLatest(&observation);
  return observation.received_age_ms;
}
static logByFunction_t gate_age_log = {.acquireUInt32 = logGateAge, .data = NULL};

static uint32_t logAge(uint32_t timestamp, void *data) {
  (void)timestamp;
  (void)data;
  EspnetCollisionObservation observation;
  espnetCollisionLinkGetLatest(&observation);
  return observation.received_age_ms;
}

static logByFunction_t age_log = {.acquireUInt32 = logAge, .data = NULL};

static void acceptDepthGateBytes(const uint8_t *bytes) {
  if (memcmp(bytes, DEPTHGATE_HEADER, 4) != 0) return;
  DepthGatePacket packet;
  memcpy(&packet, bytes, sizeof(packet));
  if (crc32CalculateBuffer(bytes, 76) != packet.checksum) {
    dg_crc_errors++;
    return;
  }
  acceptDepthGatePacket(&packet);
}

static void collisionRxTask(void *unused) {
  (void)unused;
  uint8_t window[PERCEPTION_MAP_PACKET_SIZE];
  unsigned count = 0;
  systemWaitStart();
  for (;;) {
    uint8_t byte;
    if (!uart1GetDataWithDefaultTimeout(&byte)) {
      for (unsigned i = 0; i + 4 <= count; ++i) {
        if (memcmp(window + i, DEPTHGATE_HEADER, 4) == 0 && count - i < sizeof(DepthGatePacket)) { dg_short++; break; }
      }
      count = 0; /* A truncated packet cannot span a receive timeout. */
      continue;
    }
    if (count == sizeof(window)) {
      memmove(window, window + 1, sizeof(window) - 1u);
      --count;
    }
    window[count++] = byte;
    if (count >= ESPN_PACKET_SIZE) acceptPacket(window + count - ESPN_PACKET_SIZE);
    if (count >= ESPN_GATE_PACKET_SIZE) acceptGatePacket(window + count - ESPN_GATE_PACKET_SIZE);
    if (count >= sizeof(DepthGatePacket))
      acceptDepthGateBytes(window + count - sizeof(DepthGatePacket));
    if (count == PERCEPTION_MAP_PACKET_SIZE) acceptPerceptionMapPacket(window);
  }
}

void espnetCollisionLinkInit(void) {
  if (initialized) return;
  uart1Init(ESPN_UART_BAUD);
  const BaseType_t created = xTaskCreate(collisionRxTask, "ESPNRX",
      2 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  configASSERT(created == pdPASS);
  initialized = true;
}

LOG_GROUP_START(collision)
LOG_ADD(LOG_FLOAT, pLeft, &latest.probability[0])
LOG_ADD(LOG_FLOAT, pCenter, &latest.probability[1])
LOG_ADD(LOG_FLOAT, pRight, &latest.probability[2])
LOG_ADD(LOG_UINT8, valid, &latest.valid)
LOG_ADD(LOG_UINT16, seq, &latest.sequence)
LOG_ADD(LOG_UINT32, rxOk, &latest.sample)
LOG_ADD(LOG_UINT32, srcMs, &latest.source_timestamp_ms)
LOG_ADD(LOG_UINT32, crcErr, &crc_errors)
LOG_ADD(LOG_UINT32, invalid, &invalid_packets)
LOG_ADD_BY_FUNCTION(LOG_UINT32, ageMs, &age_log)
LOG_GROUP_STOP(collision)

LOG_GROUP_START(gate)
LOG_ADD(LOG_FLOAT, pLeft, &gate_latest.rail_probability[0])
LOG_ADD(LOG_FLOAT, pRight, &gate_latest.rail_probability[1])
LOG_ADD(LOG_FLOAT, pNone, &gate_latest.affordance_probability[0])
LOG_ADD(LOG_FLOAT, openLeft, &gate_latest.affordance_probability[1])
LOG_ADD(LOG_FLOAT, openRight, &gate_latest.affordance_probability[2])
LOG_ADD(LOG_UINT8, valid, &gate_latest.valid)
LOG_ADD(LOG_UINT8, edgeMask, &gate_latest.corner_edge_mask)
LOG_ADD(LOG_UINT16, seq, &gate_latest.sequence)
LOG_ADD(LOG_UINT32, rxOk, &gate_latest.sample)
LOG_ADD(LOG_UINT32, crcErr, &gate_crc_errors)
LOG_ADD(LOG_UINT32, invalid, &gate_invalid_packets)
LOG_ADD_BY_FUNCTION(LOG_UINT32, ageMs, &gate_age_log)
LOG_ADD(LOG_FLOAT, ltX, &gate_latest.corner_x[0])
LOG_ADD(LOG_FLOAT, ltY, &gate_latest.corner_y[0])
LOG_ADD(LOG_FLOAT, ltConf, &gate_latest.corner_confidence[0])
LOG_ADD(LOG_FLOAT, rtX, &gate_latest.corner_x[1])
LOG_ADD(LOG_FLOAT, rtY, &gate_latest.corner_y[1])
LOG_ADD(LOG_FLOAT, rtConf, &gate_latest.corner_confidence[1])
LOG_ADD(LOG_FLOAT, lbX, &gate_latest.corner_x[2])
LOG_ADD(LOG_FLOAT, lbY, &gate_latest.corner_y[2])
LOG_ADD(LOG_FLOAT, lbConf, &gate_latest.corner_confidence[2])
LOG_ADD(LOG_FLOAT, rbX, &gate_latest.corner_x[3])
LOG_ADD(LOG_FLOAT, rbY, &gate_latest.corner_y[3])
LOG_ADD(LOG_FLOAT, rbConf, &gate_latest.corner_confidence[3])
LOG_GROUP_STOP(gate)

LOG_GROUP_START(pmap)
LOG_ADD(LOG_UINT16, seq, &perception_map_latest.sequence)
LOG_ADD(LOG_UINT8, version, &perception_map_latest.version)
LOG_ADD(LOG_UINT8, width, &perception_map_latest.width)
LOG_ADD(LOG_UINT8, height, &perception_map_latest.height)
LOG_ADD(LOG_UINT8, flags, &perception_map_latest.flags)
LOG_ADD(LOG_UINT32, rxOk, &perception_map_latest.sample)
LOG_ADD(LOG_UINT32, crcErr, &perception_map_crc_errors)
LOG_ADD(LOG_UINT32, invalid, &perception_map_invalid_packets)
LOG_ADD_BY_FUNCTION(LOG_UINT32, ageMs, &perception_map_age_log)
LOG_GROUP_STOP(pmap)

/* Host logger includes sequence in each block to expose sample boundaries. */
LOG_GROUP_START(dg)
LOG_ADD(LOG_UINT16, seq, &latest_depthgate.sequence)
LOG_ADD(LOG_UINT16, status, &latest_depthgate.status)
LOG_ADD(LOG_UINT32, sourceMs, &latest_depthgate.source_timestamp_ms)
LOG_ADD(LOG_UINT32, inferUs, &latest_depthgate.inference_us)
LOG_ADD(LOG_FLOAT, invLeft, &latest_depthgate.inverse_depth[0])
LOG_ADD(LOG_FLOAT, invCenter, &latest_depthgate.inverse_depth[1])
LOG_ADD(LOG_FLOAT, invRight, &latest_depthgate.inverse_depth[2])
LOG_GROUP_STOP(dg)
LOG_GROUP_START(dgRx)
LOG_ADD(LOG_UINT32, ok, &dg_rx_ok)
LOG_ADD(LOG_UINT32, crcErr, &dg_crc_errors)
LOG_ADD(LOG_UINT32, invalid, &dg_invalid)
LOG_ADD(LOG_UINT32, stale, &dg_stale)
LOG_ADD(LOG_UINT32, shortRx, &dg_short)
LOG_ADD_BY_FUNCTION(LOG_UINT32, ageMs, &dg_age_log)
LOG_GROUP_STOP(dgRx)
LOG_GROUP_START(dgLT)
LOG_ADD(LOG_FLOAT, x, &latest_depthgate.corners_xy[0])
LOG_ADD(LOG_FLOAT, y, &latest_depthgate.corners_xy[1])
LOG_ADD(LOG_FLOAT, logit, &latest_depthgate.visibility_logits[0])
LOG_GROUP_STOP(dgLT)
LOG_GROUP_START(dgRT)
LOG_ADD(LOG_FLOAT, x, &latest_depthgate.corners_xy[2])
LOG_ADD(LOG_FLOAT, y, &latest_depthgate.corners_xy[3])
LOG_ADD(LOG_FLOAT, logit, &latest_depthgate.visibility_logits[1])
LOG_GROUP_STOP(dgRT)
LOG_GROUP_START(dgLB)
LOG_ADD(LOG_FLOAT, x, &latest_depthgate.corners_xy[4])
LOG_ADD(LOG_FLOAT, y, &latest_depthgate.corners_xy[5])
LOG_ADD(LOG_FLOAT, logit, &latest_depthgate.visibility_logits[2])
LOG_GROUP_STOP(dgLB)
LOG_GROUP_START(dgRB)
LOG_ADD(LOG_FLOAT, x, &latest_depthgate.corners_xy[6])
LOG_ADD(LOG_FLOAT, y, &latest_depthgate.corners_xy[7])
LOG_ADD(LOG_FLOAT, logit, &latest_depthgate.visibility_logits[3])
LOG_GROUP_STOP(dgRB)
