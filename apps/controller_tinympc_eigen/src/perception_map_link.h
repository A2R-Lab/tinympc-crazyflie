/*
 * Compact NanoCockpit CNN control-map receiver.
 */
#ifndef __PERCEPTION_MAP_LINK_H__
#define __PERCEPTION_MAP_LINK_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PERCEPTION_MAP_MSG_HEADER "\x90\x19\x8\x37"
#define PERCEPTION_MAP_HEADER_LEN 4
#define PERCEPTION_MAP_W 10
#define PERCEPTION_MAP_H 10
#define PERCEPTION_MAP_CELLS 100
#define PERCEPTION_MAP_PACKED_BYTES 200
#define PERCEPTION_MAP_WIRE_VERSION 2
#define PERCEPTION_MAP_FLAG_DANGER_RAW_U8 (1u << 0)

typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  uint16_t sequence;
  uint8_t version;
  uint8_t width;
  uint8_t height;
  uint8_t flags;
  uint8_t packed_u4[PERCEPTION_MAP_PACKED_BYTES];
} perception_map_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[PERCEPTION_MAP_HEADER_LEN];
  perception_map_payload_t p;
  uint32_t checksum;
} perception_map_msg_t;

#ifdef __cplusplus
static_assert(sizeof(perception_map_payload_t) == 214,
              "perception map payload ABI changed");
static_assert(sizeof(perception_map_msg_t) == 222,
              "perception map packet ABI changed");
#else
_Static_assert(sizeof(perception_map_payload_t) == 214,
               "perception map payload ABI changed");
_Static_assert(sizeof(perception_map_msg_t) == 222,
               "perception map packet ABI changed");
#endif

void perceptionMapLinkInit(void);
bool perceptionMapLinkPublishFromRx(const perception_map_msg_t *msg);
void perceptionMapLinkNoteBadRx(void);
void perceptionMapLinkNoteCrcErr(void);

/* Maps are expanded to uint8 (uint4 * 17) after reception. */
bool perceptionMapLinkGetLatest(uint8_t obstacle_presence[PERCEPTION_MAP_CELLS],
                                uint8_t inverse_range[PERCEPTION_MAP_CELLS],
                                uint8_t uncertainty[PERCEPTION_MAP_CELLS],
                                /* nullable; unused by neural avoidance */
                                uint8_t gate_opening[PERCEPTION_MAP_CELLS],
                                uint32_t *out_age_ms,
                                uint32_t *out_stm32_capture_tick,
                                uint32_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif
