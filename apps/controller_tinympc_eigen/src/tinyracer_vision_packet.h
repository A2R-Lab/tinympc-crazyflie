/* Versioned AI-deck -> STM32 and SITL vision packet ABI. */
#ifndef __TINYRACER_VISION_PACKET_H__
#define __TINYRACER_VISION_PACKET_H__

#include <stdint.h>

#define TINYRACER_VISION_V2_HEADER "\x90\x19\x08\x39"
#define TINYRACER_VISION_V3_HEADER "\x90\x19\x08\x3a"
#define TINYRACER_VISION_V4_HEADER "\x90\x19\x08\x3b"
#define TINYRACER_VISION_V5_HEADER "\x90\x19\x08\x3c"
#define TINYRACER_VISION_V6_HEADER "\x90\x19\x08\x3d"
#define TINYRACER_VISION_HEADER_LEN 4
#define TINYRACER_VISION_V2_VERSION 2

#define TINYRACER_VISION_HAS_METRIC_CLEARANCE (1u << 0)
#define TINYRACER_VISION_HAS_SECTOR_DANGER (1u << 1)
#define TINYRACER_VISION_GATE_VALID (1u << 2)
#define TINYRACER_VISION_HAS_NAVIGATION_COMMAND (1u << 3)
#define TINYRACER_VISION_HAS_RESIDUAL_REFERENCE (1u << 4)

#define TINYRACER_VISION_V6_HAS_COLLISION (1u << 0)
#define TINYRACER_VISION_V6_HAS_RECOVERY (1u << 1)

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

typedef struct __attribute__((packed)) {
  TinyRacerVisionV2Payload base;
  /* Pinhole intrinsics normalized by image width/height. */
  float gate_fx_normalized;
  float gate_fy_normalized;
  float gate_cx_normalized;
  float gate_cy_normalized;
} TinyRacerVisionV3Payload;

typedef struct __attribute__((packed)) {
  uint8_t header[TINYRACER_VISION_HEADER_LEN];
  TinyRacerVisionV3Payload payload;
  uint32_t checksum;
} TinyRacerVisionV3Packet;

typedef struct __attribute__((packed)) {
  TinyRacerVisionV3Payload base;
  /* Bounded physical residual-reference contract. */
  float lateral_reference_rate_mps;
  float vertical_reference_rate_mps;
  float progress_speed_scale;
} TinyRacerVisionV4Payload;

typedef struct __attribute__((packed)) {
  uint8_t header[TINYRACER_VISION_HEADER_LEN];
  TinyRacerVisionV4Payload payload;
  uint32_t checksum;
} TinyRacerVisionV4Packet;

/* V5 carries the network's native LEFT/CENTER/RIGHT risks exactly once.
 * The four clearance/confidence values remain independent metric rays. */
typedef struct __attribute__((packed)) {
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint16_t flags;
  float clearance_m[4];
  float confidence[4];
  float danger_probability[3];
  float steering_command;
  float collision_probability;
  float gate_corners_xy[8];
  float gate_confidence;
  float gate_fx_normalized;
  float gate_fy_normalized;
  float gate_cx_normalized;
  float gate_cy_normalized;
  float lateral_reference_rate_mps;
  float vertical_reference_rate_mps;
  float progress_speed_scale;
} TinyRacerVisionV5Payload;

typedef struct __attribute__((packed)) {
  uint8_t header[TINYRACER_VISION_HEADER_LEN];
  TinyRacerVisionV5Payload payload;
  uint32_t checksum;
} TinyRacerVisionV5Packet;

/* Native collision-v8 / close-rail-v9 packet. Collision probabilities stay
 * first-class: firmware selects their highest-risk sector before consulting
 * that sector's rail-presence and pass-right probabilities. */
typedef struct __attribute__((packed)) {
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint16_t flags;
  float collision_probability[3];
  float rail_present_probability[3];
  float pass_right_probability[3];
} TinyRacerVisionV6Payload;

typedef struct __attribute__((packed)) {
  uint8_t header[TINYRACER_VISION_HEADER_LEN];
  TinyRacerVisionV6Payload payload;
  uint32_t checksum;
} TinyRacerVisionV6Packet;

_Static_assert(sizeof(TinyRacerVisionV2Packet) == 108,
               "TinyRacer vision v2 packet ABI changed");
_Static_assert(sizeof(TinyRacerVisionV3Packet) == 124,
               "TinyRacer vision v3 packet ABI changed");
_Static_assert(sizeof(TinyRacerVisionV4Packet) == 136,
               "TinyRacer vision v4 packet ABI changed");
_Static_assert(sizeof(TinyRacerVisionV5Packet) == 132,
               "TinyRacer vision v5 packet ABI changed");
_Static_assert(sizeof(TinyRacerVisionV6Packet) == 52,
               "TinyRacer vision v6 packet ABI changed");

#endif
