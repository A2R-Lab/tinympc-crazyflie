#ifndef ESPNET_COLLISION_LINK_H
#define ESPNET_COLLISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool valid;
  float probability[3]; /* left, center, right */
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint32_t received_age_ms;
  uint32_t sample;
} EspnetCollisionObservation;

typedef struct {
  bool valid;
  uint8_t corner_edge_mask; /* image edge support LT RT LB RB bits */
  float rail_probability[2]; /* left, right */
  float affordance_probability[3]; /* native network order */
  float corner_x[4], corner_y[4]; /* normalized [0,1], LT RT LB RB */
  float corner_confidence[4]; /* spatial softmax peak mass, not visibility */
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint32_t received_age_ms;
  uint32_t sample;
} EspnetGateObservation;

/* Aligned task-context copy, independent of the packed UART representation. */
typedef struct {
  bool valid;
  float inverse_depth[3]; /* left, center, right; 1/meters */
  float corners_xy[8]; /* LT RT LB RB, interleaved x/y pixels (160x128) */
  float visibility_logits[4];
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint16_t status;
  uint32_t inference_us;
  uint32_t received_age_ms; /* local time since accepted UART packet */
  uint32_t sample; /* accepted packet counter; sequence may restart/wrap */
} DepthGateObservation;

/* NanoCockpit perception-map v2. Four unsigned 4-bit channels are packed for
 * each 10x10 cell in this order: collision, inverse range, uncertainty, gate
 * opening. Values are the model's quantized outputs; physical calibration is
 * deliberately left to the consumer rather than guessed in the UART driver. */
#define PERCEPTION_MAP_WIDTH 10u
#define PERCEPTION_MAP_HEIGHT 10u
#define PERCEPTION_MAP_CELLS (PERCEPTION_MAP_WIDTH * PERCEPTION_MAP_HEIGHT)
#define PERCEPTION_MAP_PACKED_BYTES 200u
#define PERCEPTION_MAP_WIRE_VERSION 2u
#define PERCEPTION_MAP_FLAG_DANGER_RAW_U8 (1u << 0)

typedef enum {
  PERCEPTION_MAP_COLLISION = 0,
  PERCEPTION_MAP_INVERSE_RANGE = 1,
  PERCEPTION_MAP_UNCERTAINTY = 2,
  PERCEPTION_MAP_GATE_OPENING = 3,
} PerceptionMapChannel;

typedef struct {
  uint8_t packed_u4[PERCEPTION_MAP_PACKED_BYTES];
  uint32_t gap8_timestamp_us;
  uint32_t stm32_timestamp_echo;
  uint16_t sequence;
  uint8_t version;
  uint8_t width;
  uint8_t height;
  uint8_t flags;
  uint32_t received_age_ms;
  uint32_t sample;
} PerceptionMapObservation;

void espnetCollisionLinkInit(void);
/* Task-context coherent snapshot. True includes invalid inference packets;
 * callers must check valid and local received_age_ms before using the result.
 * No packet: false, zero fields and received_age_ms == UINT32_MAX. */
bool espnetCollisionLinkGetLatest(EspnetCollisionObservation *observation);
bool espnetGateLinkGetLatest(EspnetGateObservation *observation);
/* Only accepted valid packets are published. No packet: false, zero fields,
 * received_age_ms == UINT32_MAX. Age excludes inference and UART transit;
 * GAP-local source_timestamp_ms must not be subtracted from STM32 time. */
bool depthGateLinkGetLatest(DepthGateObservation *observation);
/* Receives NanoCockpit's dense perception-map v2 packet (header 90 19 08 37).
 * Use perceptionMapValue() to read one raw 4-bit cell/channel. */
bool perceptionMapLinkGetLatest(PerceptionMapObservation *observation);
uint8_t perceptionMapValue(const PerceptionMapObservation *observation,
                           unsigned x, unsigned y,
                           PerceptionMapChannel channel);

#ifdef __cplusplus
}
#endif
#endif
