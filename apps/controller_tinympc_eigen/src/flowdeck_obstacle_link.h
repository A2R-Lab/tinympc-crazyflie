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
#define FLOW_OBS_MSG_HEADER "\x90\x19\x8\x35"
#define FLOW_OBS_HEADER_LEN 4
#define FLOW_TRACK_MAX 32
#define FLOW_TRACK_WIRE_VERSION 1
#define FLOW_TRACK_MSG_HEADER "\x90\x19\x8\x36"

typedef struct __attribute__((packed)) {
  float azimuth_rad;
  float flow_x_rad_s;
  float flow_y_rad_s;
  float flow_sigma_rad_s;
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

typedef struct __attribute__((packed)) {
  uint16_t u_q4;
  uint16_t v_q4;
  int16_t du_q8;
  int16_t dv_q8;
  uint16_t lk_err_q8;
  uint16_t fb_err_q8;
} flow_track_wire_t;

typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  uint16_t sequence;
  uint16_t dt_us;
  uint8_t version;
  uint8_t count;
  uint8_t flags;
  uint8_t reserved;
  flow_track_wire_t track[FLOW_TRACK_MAX];
} flow_track_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[FLOW_OBS_HEADER_LEN];
  flow_track_payload_t p;
  uint32_t checksum;
} flow_track_msg_t;

#ifdef __cplusplus
static_assert(sizeof(float) == 4, "vision wire protocol requires float32");
static_assert(sizeof(flow_obstacle_payload_t) == 196,
              "flow payload ABI changed");
static_assert(sizeof(flow_obstacle_msg_t) == 204, "flow packet ABI changed");
static_assert(sizeof(flow_track_wire_t) == 12, "flow track ABI changed");
static_assert(sizeof(flow_track_payload_t) == 400,
              "flow track payload ABI changed");
static_assert(sizeof(flow_track_msg_t) == 408,
              "flow track packet ABI changed");
#else
_Static_assert(sizeof(float) == 4, "vision wire protocol requires float32");
_Static_assert(sizeof(flow_obstacle_payload_t) == 196,
               "flow payload ABI changed");
_Static_assert(sizeof(flow_obstacle_msg_t) == 204, "flow packet ABI changed");
_Static_assert(sizeof(flow_track_wire_t) == 12, "flow track ABI changed");
_Static_assert(sizeof(flow_track_payload_t) == 400,
               "flow track payload ABI changed");
_Static_assert(sizeof(flow_track_msg_t) == 408,
               "flow track packet ABI changed");
#endif

void flowObstacleLinkInit(void);
bool flowObstacleLinkPublishFromRx(const flow_obstacle_msg_t *msg);
bool flowObstacleLinkPublishTracksFromRx(const flow_track_msg_t *msg);
void flowObstacleLinkNoteBadRx(void);
void flowObstacleLinkNoteCrcErr(void);
void flowObstacleLinkRecordState(uint32_t timestamp_ms,
                                 float body_vx_m_s,
                                 float body_vy_m_s,
                                 float yaw_rate_rad_s,
                                 float world_x_m,
                                 float world_y_m,
                                 float yaw_rad);
void flowObstacleLinkUpdateDepth(float body_vx_m_s,
                                 float body_vy_m_s,
                                 float yaw_rate_rad_s,
                                 float world_x_m,
                                 float world_y_m,
                                 float yaw_rad);
bool flowObstacleLinkGetCylinder(float *out_world_x_m,
                                 float *out_world_y_m,
                                 float *out_radius_m,
                                 float *out_confidence);
bool flowObstacleLinkEmergencyBrake(void);

bool flowObstacleLinkGetLatest(flow_obstacle_payload_t *out,
                               uint32_t *out_age_ms,
                               uint32_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif /* __FLOWDECK_OBSTACLE_LINK_H__ */
