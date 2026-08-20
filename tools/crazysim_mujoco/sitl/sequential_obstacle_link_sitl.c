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

static int vision_socket = -1;
static TinyRacerPerceptionObservation latest_observation;
static uint32_t latest_rx_tick;
static uint32_t sample_count;
static uint16_t latest_sequence;

static bool validPacket(const TinyRacerVisionV2Packet *packet) {
  if (memcmp(packet->header, TINYRACER_VISION_V2_HEADER,
             TINYRACER_VISION_HEADER_LEN) != 0) {
    return false;
  }
  const uint32_t checksum = crc32CalculateBuffer(
      packet, sizeof(*packet) - sizeof(packet->checksum));
  if (checksum != packet->checksum || packet->payload.sequence == 0) {
    return false;
  }
  const uint16_t known_flags = TINYRACER_VISION_HAS_METRIC_CLEARANCE |
      TINYRACER_VISION_HAS_SECTOR_DANGER | TINYRACER_VISION_GATE_VALID |
      TINYRACER_VISION_HAS_NAVIGATION_COMMAND;
  if ((packet->payload.flags & ~known_flags) != 0) {
    return false;
  }
  for (int sector = 0; sector < 4; ++sector) {
    if (!isfinite(packet->payload.clearance_m[sector]) ||
        packet->payload.clearance_m[sector] < 0.0f ||
        packet->payload.clearance_m[sector] > 6.0f ||
        !isfinite(packet->payload.confidence[sector]) ||
        !isfinite(packet->payload.danger_probability[sector]) ||
        packet->payload.danger_probability[sector] < 0.0f ||
        packet->payload.danger_probability[sector] > 1.0f) {
      return false;
    }
  }
  for (int coordinate = 0; coordinate < 8; ++coordinate) {
    if (!isfinite(packet->payload.gate_corners_xy[coordinate]) ||
        packet->payload.gate_corners_xy[coordinate] < 0.0f ||
        packet->payload.gate_corners_xy[coordinate] > 1.0f) {
      return false;
    }
  }
  return isfinite(packet->payload.gate_confidence) &&
      packet->payload.gate_confidence >= 0.0f &&
      packet->payload.gate_confidence <= 1.0f &&
      isfinite(packet->payload.steering_command) &&
      packet->payload.steering_command >= -1.0f &&
      packet->payload.steering_command <= 1.0f &&
      isfinite(packet->payload.collision_probability) &&
      packet->payload.collision_probability >= 0.0f &&
      packet->payload.collision_probability <= 1.0f;
}

static void acceptPacket(const TinyRacerVisionV2Packet *packet) {
  const TinyRacerVisionV2Payload *payload = &packet->payload;
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
  latest_sequence = payload->sequence;
  latest_rx_tick = xTaskGetTickCount();
  ++sample_count;
}

void sequentialObstacleLinkInit(void) {
  memset(&latest_observation, 0, sizeof(latest_observation));
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
    TinyRacerVisionV2Packet packet;
    while (true) {
      const ssize_t received = recv(vision_socket, &packet, sizeof(packet), 0);
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (received < 0) {
        break;
      }
      if (received == (ssize_t)sizeof(packet) && validPacket(&packet)) {
        acceptPacket(&packet);
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
