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

#ifdef __cplusplus
}
#endif
#endif
