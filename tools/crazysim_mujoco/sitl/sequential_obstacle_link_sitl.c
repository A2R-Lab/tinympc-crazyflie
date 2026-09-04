#include "sequential_obstacle_link.h"
#include "tinyracer_vision_packet.h"

#include "FreeRTOS.h"
#include "task.h"
#include "crc32.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define TINYRACER_SITL_VISION_PORT 19960
#define LEGACY_GATE_FX_NORMALIZED (89.15584f / 160.0f)
#define LEGACY_GATE_FY_NORMALIZED (89.46082f / 120.0f)

static int vision_socket = -1;
static TinyRacerPerceptionObservation latest_observation;
static uint32_t latest_rx_tick;
static uint32_t sample_count;
static uint16_t latest_sequence;
static uint32_t rejected_packet_count;
static SequentialObstacleThreatObservation latest_threat;
static SequentialObstacleGateObservation latest_gate;
static uint32_t latest_threat_rx_tick;
static uint32_t latest_gate_rx_tick;
static uint32_t threat_sample_count;
static uint32_t gate_sample_count;
static uint16_t latest_threat_sequence;
static uint16_t latest_gate_sequence;

static bool validPayload(
    const TinyRacerVisionV2Payload *payload,
    float gate_fx_normalized, float gate_fy_normalized,
    float gate_cx_normalized, float gate_cy_normalized,
    bool allow_residual_reference) {
  if (payload->sequence == 0) {
    return false;
  }
  const uint16_t known_flags = TINYRACER_VISION_HAS_METRIC_CLEARANCE |
      TINYRACER_VISION_HAS_SECTOR_DANGER | TINYRACER_VISION_GATE_VALID |
      TINYRACER_VISION_HAS_NAVIGATION_COMMAND |
      (allow_residual_reference ? TINYRACER_VISION_HAS_RESIDUAL_REFERENCE : 0u);
  if ((payload->flags & ~known_flags) != 0) {
    return false;
  }
  for (int sector = 0; sector < 4; ++sector) {
    if (!isfinite(payload->clearance_m[sector]) ||
        payload->clearance_m[sector] < 0.0f ||
        payload->clearance_m[sector] > 6.0f ||
        !isfinite(payload->confidence[sector]) ||
        !isfinite(payload->danger_probability[sector]) ||
        payload->danger_probability[sector] < 0.0f ||
        payload->danger_probability[sector] > 1.0f) {
      return false;
    }
  }
  for (int coordinate = 0; coordinate < 8; ++coordinate) {
    if (!isfinite(payload->gate_corners_xy[coordinate]) ||
        payload->gate_corners_xy[coordinate] < 0.0f ||
        payload->gate_corners_xy[coordinate] > 1.0f) {
      return false;
    }
  }
  return isfinite(payload->gate_confidence) &&
      payload->gate_confidence >= 0.0f && payload->gate_confidence <= 1.0f &&
      isfinite(payload->steering_command) &&
      payload->steering_command >= -1.0f && payload->steering_command <= 1.0f &&
      isfinite(payload->collision_probability) &&
      payload->collision_probability >= 0.0f &&
      payload->collision_probability <= 1.0f &&
      isfinite(gate_fx_normalized) && gate_fx_normalized >= 0.05f &&
      isfinite(gate_fy_normalized) && gate_fy_normalized >= 0.05f &&
      isfinite(gate_cx_normalized) && gate_cx_normalized >= 0.0f &&
      gate_cx_normalized <= 1.0f &&
      isfinite(gate_cy_normalized) && gate_cy_normalized >= 0.0f &&
      gate_cy_normalized <= 1.0f;
}

static void acceptPayload(
    const TinyRacerVisionV2Payload *payload,
    float gate_fx_normalized, float gate_fy_normalized,
    float gate_cx_normalized, float gate_cy_normalized) {
  if (payload->sequence == latest_sequence) {
    return;
  }
  memset(&latest_observation, 0, sizeof(latest_observation));
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
  latest_sequence = payload->sequence;
  latest_rx_tick = xTaskGetTickCount();
  ++sample_count;
  if (sample_count == 1) {
    printf("TinyRacer vision accepted sequence=%u flags=0x%04x "
           "danger=(%.2f,%.2f,%.2f)\n",
           payload->sequence, payload->flags,
           (double)payload->danger_probability[0],
           (double)fmaxf(payload->danger_probability[1],
                         payload->danger_probability[2]),
           (double)payload->danger_probability[3]);
  }
}

static void v5ToLegacy(const TinyRacerVisionV5Payload *payload,
                       TinyRacerVisionV4Payload *legacy) {
  memset(legacy, 0, sizeof(*legacy));
  TinyRacerVisionV2Payload *base = &legacy->base.base;
  base->source_timestamp_ms = payload->source_timestamp_ms;
  base->sequence = payload->sequence;
  base->flags = payload->flags;
  memcpy(base->clearance_m, payload->clearance_m, sizeof(base->clearance_m));
  memcpy(base->confidence, payload->confidence, sizeof(base->confidence));
  base->danger_probability[0] = payload->danger_probability[0];
  base->danger_probability[1] = payload->danger_probability[1];
  base->danger_probability[2] = payload->danger_probability[1];
  base->danger_probability[3] = payload->danger_probability[2];
  base->steering_command = payload->steering_command;
  base->collision_probability = payload->collision_probability;
  memcpy(base->gate_corners_xy, payload->gate_corners_xy,
         sizeof(base->gate_corners_xy));
  base->gate_confidence = payload->gate_confidence;
  legacy->base.gate_fx_normalized = payload->gate_fx_normalized;
  legacy->base.gate_fy_normalized = payload->gate_fy_normalized;
  legacy->base.gate_cx_normalized = payload->gate_cx_normalized;
  legacy->base.gate_cy_normalized = payload->gate_cy_normalized;
  legacy->lateral_reference_rate_mps = payload->lateral_reference_rate_mps;
  legacy->vertical_reference_rate_mps = payload->vertical_reference_rate_mps;
  legacy->progress_speed_scale = payload->progress_speed_scale;
}

static bool validV5Payload(const TinyRacerVisionV5Payload *payload) {
  TinyRacerVisionV4Payload legacy;
  v5ToLegacy(payload, &legacy);
  const TinyMpcVisionResidualAuthority authority =
      tinyMpcVisionResidualFullAuthorityV1();
  return validPayload(&legacy.base.base,
             legacy.base.gate_fx_normalized,
             legacy.base.gate_fy_normalized,
             legacy.base.gate_cx_normalized,
             legacy.base.gate_cy_normalized, true) &&
      tinyMpcVisionResidualPacketWithinAuthority(
          &authority, legacy.lateral_reference_rate_mps,
          legacy.vertical_reference_rate_mps, legacy.progress_speed_scale);
}

static void acceptV5Payload(const TinyRacerVisionV5Payload *payload) {
  TinyRacerVisionV4Payload legacy;
  v5ToLegacy(payload, &legacy);
  acceptPayload(&legacy.base.base, legacy.base.gate_fx_normalized,
      legacy.base.gate_fy_normalized, legacy.base.gate_cx_normalized,
      legacy.base.gate_cy_normalized, &legacy);
}

static bool validV7Payload(const TinyRacerVisionV7Payload *payload) {
  const uint16_t known_flags = TINYRACER_VISION_V7_HAS_COLLISION |
      TINYRACER_VISION_V7_HAS_SQUARE_OPENING;
  if (payload->sequence == 0u || payload->flags != known_flags ||
      !isfinite(payload->square_opening_visible_probability) ||
      payload->square_opening_visible_probability < 0.0f ||
      payload->square_opening_visible_probability > 1.0f) {
    return false;
  }
  for (int sector = 0; sector < TINYRACER_DANGER_SECTORS; ++sector) {
    if (!isfinite(payload->collision_probability[sector]) ||
        payload->collision_probability[sector] < 0.0f ||
        payload->collision_probability[sector] > 1.0f) {
      return false;
    }
  }
  return true;
}

static void acceptV7Payload(const TinyRacerVisionV7Payload *payload) {
  if (payload->sequence == latest_sequence) {
    return;
  }
  memset(&latest_observation, 0, sizeof(latest_observation));
  latest_observation.valid = true;
  latest_observation.source_timestamp = payload->source_timestamp_ms;
  latest_observation.sequence = payload->sequence;
  latest_observation.has_sector_danger = true;
  latest_observation.has_square_opening = true;
  latest_observation.progress_speed_scale = 1.0f;
  latest_observation.square_opening_visible_probability =
      payload->square_opening_visible_probability;
  memcpy(latest_observation.danger_probability,
         payload->collision_probability,
         sizeof(latest_observation.danger_probability));
  latest_observation.collision_probability = fmaxf(
      payload->collision_probability[0], fmaxf(
          payload->collision_probability[1],
          payload->collision_probability[2]));
  for (int sector = 0; sector < TINYRACER_CLEARANCE_SECTORS; ++sector) {
    latest_observation.clearance_m[sector] = 6.0f;
  }
  latest_observation.gate_fx_normalized = LEGACY_GATE_FX_NORMALIZED;
  latest_observation.gate_fy_normalized = LEGACY_GATE_FY_NORMALIZED;
  latest_observation.gate_cx_normalized = 0.5f;
  latest_observation.gate_cy_normalized = 0.5f;
  latest_sequence = payload->sequence;
  latest_rx_tick = xTaskGetTickCount();
  ++sample_count;
  if (sample_count == 1u) {
    printf("TinyRacer vision V7 accepted sequence=%u square_opening=%.3f\n",
           payload->sequence,
           (double)payload->square_opening_visible_probability);
  }
}

static bool validV8Payload(const TinyRacerVisionV8Payload *payload) {
  const uint16_t known_flags = TINYRACER_VISION_V8_HAS_NAVIGATION |
      TINYRACER_VISION_V8_HAS_SQUARE_OPENING;
  return payload->sequence != 0u && payload->flags == known_flags &&
      isfinite(payload->normalized_yaw_rate) &&
      payload->normalized_yaw_rate >= -1.0f &&
      payload->normalized_yaw_rate <= 1.0f &&
      isfinite(payload->collision_probability) &&
      payload->collision_probability >= 0.0f &&
      payload->collision_probability <= 1.0f &&
      isfinite(payload->square_opening_visible_probability) &&
      payload->square_opening_visible_probability >= 0.0f &&
      payload->square_opening_visible_probability <= 1.0f;
}

static void acceptV8Payload(const TinyRacerVisionV8Payload *payload) {
  if (payload->sequence == latest_sequence) {
    return;
  }
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
  latest_sequence = payload->sequence;
  latest_rx_tick = xTaskGetTickCount();
  ++sample_count;
  if (sample_count == 1u) {
    printf("TinyRacer vision V8 accepted sequence=%u collision=%.3f opening=%.3f\n",
           payload->sequence, (double)payload->collision_probability,
           (double)payload->square_opening_visible_probability);
  }
}

static bool validThreatPayload(const TinyRacerOlgmdThreatPayload *payload) {
  return payload->frame_sequence != 0u && payload->imminent_threat <= 1u &&
      payload->reserved == 0u;
}

static void acceptThreatPayload(const TinyRacerOlgmdThreatPayload *payload) {
  if (threat_sample_count != 0u &&
      payload->frame_sequence == latest_threat_sequence) {
    return;
  }
  latest_threat.valid = true;
  latest_threat.source_timestamp_ms = payload->source_timestamp_ms;
  latest_threat.sequence = payload->frame_sequence;
  latest_threat.imminent_threat = payload->imminent_threat != 0u;
  latest_threat_sequence = payload->frame_sequence;
  latest_threat_rx_tick = xTaskGetTickCount();
  latest_threat.sample = ++threat_sample_count;
}

static bool validGatePayload(const TinyRacerGateObservationPayload *payload) {
  if (payload->frame_sequence == 0u || payload->inference_valid > 1u ||
      payload->reserved != 0u) {
    return false;
  }
  for (int coordinate = 0; coordinate < 8; ++coordinate) {
    if (!isfinite(payload->corners_xy[coordinate]) ||
        payload->corners_xy[coordinate] < -1.0f ||
        payload->corners_xy[coordinate] > 2.0f) {
      return false;
    }
  }
  return true;
}

static void acceptGatePayload(const TinyRacerGateObservationPayload *payload) {
  if (gate_sample_count != 0u &&
      payload->frame_sequence == latest_gate_sequence) {
    return;
  }
  latest_gate.valid = payload->inference_valid != 0u;
  latest_gate.source_timestamp_ms = payload->source_timestamp_ms;
  latest_gate.sequence = payload->frame_sequence;
  memcpy(latest_gate.corners_xy, payload->corners_xy,
         sizeof(latest_gate.corners_xy));
  latest_gate_sequence = payload->frame_sequence;
  latest_gate_rx_tick = xTaskGetTickCount();
  latest_gate.sample = ++gate_sample_count;
}

void sequentialObstacleLinkInit(void) {
  memset(&latest_observation, 0, sizeof(latest_observation));
  memset(&latest_threat, 0, sizeof(latest_threat));
  memset(&latest_gate, 0, sizeof(latest_gate));
  const char *port_text = getenv("TINYMPC_VISION_PORT");
  const long requested_port = port_text != NULL ? strtol(port_text, NULL, 10)
                                                : TINYRACER_SITL_VISION_PORT;
  if (requested_port < 1 || requested_port > 65535) {
    fprintf(stderr, "Invalid TINYMPC_VISION_PORT=%s\n",
            port_text != NULL ? port_text : "");
    return;
  }
  vision_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (vision_socket < 0) {
    perror("TinyRacer vision socket");
    return;
  }
  const int flags = fcntl(vision_socket, F_GETFL, 0);
  fcntl(vision_socket, F_SETFL, flags | O_NONBLOCK);
  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons((uint16_t)requested_port);
  if (bind(vision_socket, (struct sockaddr *)&address, sizeof(address)) != 0) {
    perror("TinyRacer vision bind");
    close(vision_socket);
    vision_socket = -1;
    return;
  }
  printf("TinyRacer vision listening on udp://127.0.0.1:%ld\n", requested_port);
}

bool sequentialObstacleLinkGetLatest(
    TinyRacerPerceptionObservation *observation) {
  if (vision_socket >= 0) {
    union {
      TinyRacerVisionV2Packet v2;
      TinyRacerVisionV3Packet v3;
      TinyRacerVisionV4Packet v4;
      TinyRacerVisionV5Packet v5;
      TinyRacerVisionV7Packet v7;
      TinyRacerVisionV8Packet v8;
      TinyRacerOlgmdThreatPacket threat;
      TinyRacerGateObservationPacket gate;
    } packet;
    while (true) {
      const ssize_t received = recv(vision_socket, &packet, sizeof(packet), 0);
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (received < 0) {
        break;
      }
      const TinyMpcVisionResidualAuthority residual_authority =
          tinyMpcVisionResidualFullAuthorityV1();
      const bool is_threat =
          received == (ssize_t)sizeof(packet.threat) &&
          memcmp(packet.threat.header, TINYRACER_OLGMD_THREAT_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.threat,
              sizeof(packet.threat) - sizeof(packet.threat.checksum)) ==
                  packet.threat.checksum &&
          validThreatPayload(&packet.threat.payload);
      const bool is_gate =
          received == (ssize_t)sizeof(packet.gate) &&
          memcmp(packet.gate.header, TINYRACER_GATE_OBSERVATION_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.gate,
              sizeof(packet.gate) - sizeof(packet.gate.checksum)) ==
                  packet.gate.checksum &&
          validGatePayload(&packet.gate.payload);
      const bool is_v8 = received == (ssize_t)sizeof(packet.v8) &&
          memcmp(packet.v8.header, TINYRACER_VISION_V8_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v8,
              sizeof(packet.v8) - sizeof(packet.v8.checksum)) ==
                  packet.v8.checksum && validV8Payload(&packet.v8.payload);
      const bool is_v7 = received == (ssize_t)sizeof(packet.v7) &&
          memcmp(packet.v7.header, TINYRACER_VISION_V7_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v7,
              sizeof(packet.v7) - sizeof(packet.v7.checksum)) ==
                  packet.v7.checksum && validV7Payload(&packet.v7.payload);
      const bool is_v5 = received == (ssize_t)sizeof(packet.v5) &&
          memcmp(packet.v5.header, TINYRACER_VISION_V5_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v5,
              sizeof(packet.v5) - sizeof(packet.v5.checksum)) ==
                  packet.v5.checksum && validV5Payload(&packet.v5.payload);
      const bool is_v4 = received == (ssize_t)sizeof(packet.v4) &&
          memcmp(packet.v4.header, TINYRACER_VISION_V4_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v4,
              sizeof(packet.v4) - sizeof(packet.v4.checksum)) ==
                  packet.v4.checksum &&
          validPayload(
              &packet.v4.payload.base.base,
              packet.v4.payload.base.gate_fx_normalized,
              packet.v4.payload.base.gate_fy_normalized,
              packet.v4.payload.base.gate_cx_normalized,
              packet.v4.payload.base.gate_cy_normalized, true) &&
          tinyMpcVisionResidualPacketWithinAuthority(
              &residual_authority,
              packet.v4.payload.lateral_reference_rate_mps,
              packet.v4.payload.vertical_reference_rate_mps,
              packet.v4.payload.progress_speed_scale);
      const bool is_v3 = received == (ssize_t)sizeof(packet.v3) &&
          memcmp(packet.v3.header, TINYRACER_VISION_V3_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v3,
              sizeof(packet.v3) - sizeof(packet.v3.checksum)) ==
                  packet.v3.checksum &&
          validPayload(
              &packet.v3.payload.base,
              packet.v3.payload.gate_fx_normalized,
              packet.v3.payload.gate_fy_normalized,
              packet.v3.payload.gate_cx_normalized,
              packet.v3.payload.gate_cy_normalized);
      const bool is_v2 = received == (ssize_t)sizeof(packet.v2) &&
          memcmp(packet.v2.header, TINYRACER_VISION_V2_HEADER,
                 TINYRACER_VISION_HEADER_LEN) == 0 &&
          crc32CalculateBuffer(&packet.v2,
              sizeof(packet.v2) - sizeof(packet.v2.checksum)) ==
                  packet.v2.checksum &&
          validPayload(
              &packet.v2.payload, LEGACY_GATE_FX_NORMALIZED,
              LEGACY_GATE_FY_NORMALIZED, 0.5f, 0.5f);
      if (is_v3) {
        acceptPayload(
            &packet.v3.payload.base,
            packet.v3.payload.gate_fx_normalized,
            packet.v3.payload.gate_fy_normalized,
            packet.v3.payload.gate_cx_normalized,
            packet.v3.payload.gate_cy_normalized);
      } else if (is_v2) {
        acceptPayload(
            &packet.v2.payload, LEGACY_GATE_FX_NORMALIZED,
            LEGACY_GATE_FY_NORMALIZED, 0.5f, 0.5f);
      } else if (rejected_packet_count++ == 0) {
        fprintf(stderr,
                "TinyRacer vision rejected first datagram: bytes=%ld expected=%lu/%lu\n",
                (long)received, (unsigned long)sizeof(packet.v2),
                (unsigned long)sizeof(packet.v3));
      }
    }
  }
  if (sample_count == 0) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  *observation = latest_observation;
  observation->received_age_ms =
      (xTaskGetTickCount() - latest_rx_tick) * portTICK_PERIOD_MS;
  observation->sample = sample_count;
  return true;
}

bool sequentialObstacleLinkGetLatestThreat(
    SequentialObstacleThreatObservation *observation) {
  if (threat_sample_count == 0u) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  *observation = latest_threat;
  observation->received_age_ms =
      (xTaskGetTickCount() - latest_threat_rx_tick) * portTICK_PERIOD_MS;
  return true;
}

bool sequentialObstacleLinkGetLatestGate(
    SequentialObstacleGateObservation *observation) {
  if (gate_sample_count == 0u) {
    memset(observation, 0, sizeof(*observation));
    return false;
  }
  *observation = latest_gate;
  observation->received_age_ms =
      (xTaskGetTickCount() - latest_gate_rx_tick) * portTICK_PERIOD_MS;
  return true;
}
