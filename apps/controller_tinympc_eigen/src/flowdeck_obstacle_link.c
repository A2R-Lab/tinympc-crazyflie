/*
 * flowdeck_obstacle_link.c
 * Published state for AI-deck obstacle-flow sector messages.
 *
 * The UART bytes are consumed by gate8_link.c because gate corners and flow sectors
 * share USART3. This module only owns validation counters, seqlock publication, and
 * a copy API for the controller.
 */
#include "flowdeck_obstacle_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"

#include <math.h>
#include <string.h>

#define FLOW_MIN_CONFIDENCE 0.02f
#define FLOW_MIN_TRANSLATION_M_S 0.03f
#define FLOW_MAX_INV_DEPTH_M 8.0f
#define FLOW_MAX_RANGE_M 10.0f

static volatile uint32_t g_seq = 0;
static flow_obstacle_payload_t g_payload;
static volatile uint32_t g_rxTick = 0;
static volatile uint32_t g_rxOk = 0;
static volatile uint32_t g_crcErr = 0;
static volatile uint32_t g_badRx = 0;
static float g_bodyVx = 0.0f;
static float g_bodyVy = 0.0f;
static float g_yawRate = 0.0f;
static float g_resFlow[FLOW_OBS_SECT_MAX] = {0};
static float g_velEff[FLOW_OBS_SECT_MAX] = {0};
static float g_invDepth[FLOW_OBS_SECT_MAX] = {0};
static float g_range[FLOW_OBS_SECT_MAX] = {0};
static float g_valid[FLOW_OBS_SECT_MAX] = {0};

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

void flowObstacleLinkInit(void) {
  memset(&g_payload, 0, sizeof(g_payload));
}

void flowObstacleLinkPublishFromRx(const flow_obstacle_msg_t *msg) {
  uint32_t s = g_seq + 1;
  g_seq = s;
  COMPILER_BARRIER();
  memcpy(&g_payload, &msg->p, sizeof(g_payload));
  if (g_payload.n_sectors > FLOW_OBS_SECT_MAX) {
    g_payload.n_sectors = FLOW_OBS_SECT_MAX;
  }
  g_rxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_seq = s + 1;
  g_rxOk++;
}

void flowObstacleLinkNoteBadRx(void) {
  g_badRx++;
}

void flowObstacleLinkNoteCrcErr(void) {
  g_crcErr++;
}

void flowObstacleLinkUpdateDepth(float body_vx_m_s,
                                 float body_vy_m_s,
                                 float yaw_rate_rad_s) {
  flow_obstacle_payload_t payload;
  uint32_t age_ms = 0;
  uint32_t sample = 0;
  if (!flowObstacleLinkGetLatest(&payload, &age_ms, &sample) || age_ms > 500) {
    for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      g_resFlow[i] = 0.0f;
      g_velEff[i] = 0.0f;
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
    }
    return;
  }

  g_bodyVx = body_vx_m_s;
  g_bodyVy = body_vy_m_s;
  g_yawRate = yaw_rate_rad_s;

  for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
    if (i >= payload.n_sectors || payload.sector[i].confidence < FLOW_MIN_CONFIDENCE) {
      g_resFlow[i] = 0.0f;
      g_velEff[i] = 0.0f;
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
      continue;
    }

    const float az = payload.sector[i].azimuth_rad;
    const float measured_flow = payload.sector[i].flow_x_rad_s;
    const float residual_flow = measured_flow - yaw_rate_rad_s;
    const float vel_eff = body_vx_m_s * sinf(az) - body_vy_m_s * cosf(az);

    g_resFlow[i] = residual_flow;
    g_velEff[i] = vel_eff;

    if (fabsf(vel_eff) < FLOW_MIN_TRANSLATION_M_S ||
        fabsf(residual_flow) < 1.0e-3f) {
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
      continue;
    }

    float inv_depth = residual_flow / vel_eff;
    if (inv_depth < 0.0f) {
      inv_depth = -inv_depth;
    }
    if (inv_depth > FLOW_MAX_INV_DEPTH_M) {
      inv_depth = FLOW_MAX_INV_DEPTH_M;
    }

    g_invDepth[i] = inv_depth;
    g_range[i] = inv_depth > 1.0e-3f ? (1.0f / inv_depth) : 0.0f;
    if (g_range[i] > FLOW_MAX_RANGE_M) {
      g_range[i] = FLOW_MAX_RANGE_M;
    }
    g_valid[i] = 1.0f;
  }
}

bool flowObstacleLinkGetLatest(flow_obstacle_payload_t *out,
                               uint32_t *out_age_ms,
                               uint32_t *out_sample) {
  uint32_t s1, s2, rxTick;
  do {
    s1 = g_seq;
    COMPILER_BARRIER();
    if (out) {
      memcpy(out, &g_payload, sizeof(*out));
    }
    rxTick = g_rxTick;
    COMPILER_BARRIER();
    s2 = g_seq;
  } while ((s1 & 1u) || s1 != s2);

  if (g_rxOk == 0) {
    return false;
  }
  if (out_age_ms) {
    *out_age_ms = (xTaskGetTickCount() - rxTick) * portTICK_PERIOD_MS;
  }
  if (out_sample) {
    *out_sample = s1 >> 1;
  }
  return true;
}

LOG_GROUP_START(flowObsRx)
LOG_ADD(LOG_UINT32, rxOk,   &g_rxOk)
LOG_ADD(LOG_UINT32, crcErr, &g_crcErr)
LOG_ADD(LOG_UINT32, badRx,  &g_badRx)
LOG_ADD(LOG_UINT8,  n,      &g_payload.n_sectors)
LOG_ADD(LOG_FLOAT,  dt,     &g_payload.dt_s)
LOG_ADD(LOG_FLOAT,  flowX0, &g_payload.sector[0].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX1, &g_payload.sector[1].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX2, &g_payload.sector[2].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX3, &g_payload.sector[3].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX4, &g_payload.sector[4].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX5, &g_payload.sector[5].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX6, &g_payload.sector[6].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX7, &g_payload.sector[7].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowX8, &g_payload.sector[8].flow_x_rad_s)
LOG_ADD(LOG_FLOAT,  flowY0, &g_payload.sector[0].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY1, &g_payload.sector[1].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY2, &g_payload.sector[2].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY3, &g_payload.sector[3].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY4, &g_payload.sector[4].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY5, &g_payload.sector[5].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY6, &g_payload.sector[6].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY7, &g_payload.sector[7].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  flowY8, &g_payload.sector[8].flow_y_rad_s)
LOG_ADD(LOG_FLOAT,  conf0,  &g_payload.sector[0].confidence)
LOG_ADD(LOG_FLOAT,  conf1,  &g_payload.sector[1].confidence)
LOG_ADD(LOG_FLOAT,  conf2,  &g_payload.sector[2].confidence)
LOG_ADD(LOG_FLOAT,  conf3,  &g_payload.sector[3].confidence)
LOG_ADD(LOG_FLOAT,  conf4,  &g_payload.sector[4].confidence)
LOG_ADD(LOG_FLOAT,  conf5,  &g_payload.sector[5].confidence)
LOG_ADD(LOG_FLOAT,  conf6,  &g_payload.sector[6].confidence)
LOG_ADD(LOG_FLOAT,  conf7,  &g_payload.sector[7].confidence)
LOG_ADD(LOG_FLOAT,  conf8,  &g_payload.sector[8].confidence)
LOG_ADD(LOG_FLOAT,  bodyVx, &g_bodyVx)
LOG_ADD(LOG_FLOAT,  bodyVy, &g_bodyVy)
LOG_ADD(LOG_FLOAT,  yawRate, &g_yawRate)
LOG_ADD(LOG_FLOAT,  resX0,  &g_resFlow[0])
LOG_ADD(LOG_FLOAT,  resX1,  &g_resFlow[1])
LOG_ADD(LOG_FLOAT,  resX2,  &g_resFlow[2])
LOG_ADD(LOG_FLOAT,  resX3,  &g_resFlow[3])
LOG_ADD(LOG_FLOAT,  resX4,  &g_resFlow[4])
LOG_ADD(LOG_FLOAT,  resX5,  &g_resFlow[5])
LOG_ADD(LOG_FLOAT,  resX6,  &g_resFlow[6])
LOG_ADD(LOG_FLOAT,  resX7,  &g_resFlow[7])
LOG_ADD(LOG_FLOAT,  resX8,  &g_resFlow[8])
LOG_ADD(LOG_FLOAT,  vEff0,  &g_velEff[0])
LOG_ADD(LOG_FLOAT,  vEff1,  &g_velEff[1])
LOG_ADD(LOG_FLOAT,  vEff2,  &g_velEff[2])
LOG_ADD(LOG_FLOAT,  vEff3,  &g_velEff[3])
LOG_ADD(LOG_FLOAT,  vEff4,  &g_velEff[4])
LOG_ADD(LOG_FLOAT,  vEff5,  &g_velEff[5])
LOG_ADD(LOG_FLOAT,  vEff6,  &g_velEff[6])
LOG_ADD(LOG_FLOAT,  vEff7,  &g_velEff[7])
LOG_ADD(LOG_FLOAT,  vEff8,  &g_velEff[8])
LOG_ADD(LOG_FLOAT,  inv0,   &g_invDepth[0])
LOG_ADD(LOG_FLOAT,  inv1,   &g_invDepth[1])
LOG_ADD(LOG_FLOAT,  inv2,   &g_invDepth[2])
LOG_ADD(LOG_FLOAT,  inv3,   &g_invDepth[3])
LOG_ADD(LOG_FLOAT,  inv4,   &g_invDepth[4])
LOG_ADD(LOG_FLOAT,  inv5,   &g_invDepth[5])
LOG_ADD(LOG_FLOAT,  inv6,   &g_invDepth[6])
LOG_ADD(LOG_FLOAT,  inv7,   &g_invDepth[7])
LOG_ADD(LOG_FLOAT,  inv8,   &g_invDepth[8])
LOG_ADD(LOG_FLOAT,  range0, &g_range[0])
LOG_ADD(LOG_FLOAT,  range1, &g_range[1])
LOG_ADD(LOG_FLOAT,  range2, &g_range[2])
LOG_ADD(LOG_FLOAT,  range3, &g_range[3])
LOG_ADD(LOG_FLOAT,  range4, &g_range[4])
LOG_ADD(LOG_FLOAT,  range5, &g_range[5])
LOG_ADD(LOG_FLOAT,  range6, &g_range[6])
LOG_ADD(LOG_FLOAT,  range7, &g_range[7])
LOG_ADD(LOG_FLOAT,  range8, &g_range[8])
LOG_ADD(LOG_FLOAT,  valid0, &g_valid[0])
LOG_ADD(LOG_FLOAT,  valid1, &g_valid[1])
LOG_ADD(LOG_FLOAT,  valid2, &g_valid[2])
LOG_ADD(LOG_FLOAT,  valid3, &g_valid[3])
LOG_ADD(LOG_FLOAT,  valid4, &g_valid[4])
LOG_ADD(LOG_FLOAT,  valid5, &g_valid[5])
LOG_ADD(LOG_FLOAT,  valid6, &g_valid[6])
LOG_ADD(LOG_FLOAT,  valid7, &g_valid[7])
LOG_ADD(LOG_FLOAT,  valid8, &g_valid[8])
LOG_GROUP_STOP(flowObsRx)
