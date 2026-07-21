/*
 * flowdeck_obstacle_link.h
 * GAP8 AI-deck -> STM32 obstacle-flow sector link over the shared deck UART.
 */
#ifndef __FLOWDECK_OBSTACLE_LINK_H__
#define __FLOWDECK_OBSTACLE_LINK_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLOW_OBS_SECT_MAX 9
#define FLOW_OBS_MSG_HEADER "\x90\x19\x8\x34"
#define FLOW_OBS_HEADER_LEN 4

typedef struct __attribute__((packed)) {
  float azimuth_rad;
  float inv_depth;
  float ttc_s;
  float confidence;
} flow_obstacle_sector_t;

typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  float dt_s;
  uint8_t n_sectors;
  uint8_t flags;
  uint16_t reserved;
  flow_obstacle_sector_t sector[FLOW_OBS_SECT_MAX];
} flow_obstacle_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[FLOW_OBS_HEADER_LEN];
  flow_obstacle_payload_t p;
  uint32_t checksum;
} flow_obstacle_msg_t;

void flowObstacleLinkInit(void);
void flowObstacleLinkPublishFromRx(const flow_obstacle_msg_t *msg);
void flowObstacleLinkNoteBadRx(void);
void flowObstacleLinkNoteCrcErr(void);

bool flowObstacleLinkGetLatest(flow_obstacle_payload_t *out,
                               uint32_t *out_age_ms,
                               uint32_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif /* __FLOWDECK_OBSTACLE_LINK_H__ */
