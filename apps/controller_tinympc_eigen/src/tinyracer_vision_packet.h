/* Versioned AI-deck -> STM32 and SITL vision packet ABI. */
#ifndef __TINYRACER_VISION_PACKET_H__
#define __TINYRACER_VISION_PACKET_H__

#include <stdint.h>

#define TINYRACER_VISION_V2_HEADER "\x90\x19\x08\x39"
#define TINYRACER_VISION_HEADER_LEN 4
#define TINYRACER_VISION_V2_VERSION 2

#define TINYRACER_VISION_HAS_METRIC_CLEARANCE (1u << 0)
#define TINYRACER_VISION_HAS_SECTOR_DANGER (1u << 1)
#define TINYRACER_VISION_GATE_VALID (1u << 2)
#define TINYRACER_VISION_HAS_NAVIGATION_COMMAND (1u << 3)

typedef struct __attribute__((packed)) {
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint16_t flags;
  float clearance_m[4];
  float confidence[4];
  float danger_probability[4];
  float steering_command;
  float collision_probability;
  float gate_corners_xy[8];
  float gate_confidence;
} TinyRacerVisionV2Payload;

typedef struct __attribute__((packed)) {
  uint8_t header[TINYRACER_VISION_HEADER_LEN];
  TinyRacerVisionV2Payload payload;
  uint32_t checksum;
} TinyRacerVisionV2Packet;

_Static_assert(sizeof(TinyRacerVisionV2Packet) == 108,
               "TinyRacer vision v2 packet ABI changed");

#endif
