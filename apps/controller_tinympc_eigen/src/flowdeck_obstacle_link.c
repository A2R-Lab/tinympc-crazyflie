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
#define FLOW_OBS_CANDIDATE_MAX_RANGE_M 3.0f
#define FLOW_OBS_CANDIDATE_MAX_YAW_RATE_RAD_S 0.6f
#define FLOW_OBS_CANDIDATE_MIN_SECTORS 2
#define FLOW_OBS_CANDIDATE_ALPHA 0.35f
#define FLOW_OBS_CANDIDATE_RADIUS_M 0.25f
#define FLOW_OBS_CYL_ALPHA 0.20f
#define FLOW_OBS_CYL_CONF_UP 0.20f
#define FLOW_OBS_CYL_CONF_DOWN 0.04f
#define FLOW_OBS_CYL_VALID_CONF 0.50f
#define FLOW_OBS_CYL_GATE_CONF 0.35f
#define FLOW_OBS_CYL_GATE_BASE_M 0.45f
#define FLOW_OBS_CYL_GATE_RANGE_FRAC 0.50f
#define FLOW_OBS_CYL_COV_ALPHA 0.15f
#define FLOW_OBS_CYL_COV_INIT_M2 0.04f
#define FLOW_OBS_CYL_COV_INFLATE_M2 0.002f
#define FLOW_OBS_CYL_COV_MAX_M2 4.0f

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
static float g_bodyX[FLOW_OBS_SECT_MAX] = {0};
static float g_bodyY[FLOW_OBS_SECT_MAX] = {0};
static float g_worldX[FLOW_OBS_SECT_MAX] = {0};
static float g_worldY[FLOW_OBS_SECT_MAX] = {0};
static float g_nearValid = 0.0f;
static uint8_t g_nearIdx = 0;
static float g_nearRange = 0.0f;
static float g_nearBodyX = 0.0f;
static float g_nearBodyY = 0.0f;
static float g_nearWorldX = 0.0f;
static float g_nearWorldY = 0.0f;
static float g_obsValid = 0.0f;
static uint8_t g_obsHits = 0;
static uint8_t g_obsClusterStart = 0;
static uint8_t g_obsClusterCount = 0;
static float g_obsRange = 0.0f;
static float g_obsBearing = 0.0f;
static float g_obsBodyX = 0.0f;
static float g_obsBodyY = 0.0f;
static float g_obsWorldX = 0.0f;
static float g_obsWorldY = 0.0f;
static float g_obsRadius = FLOW_OBS_CANDIDATE_RADIUS_M;
static float g_cylValid = 0.0f;
static float g_cylConf = 0.0f;
static float g_cylAge = 0.0f;
static float g_cylBodyX = 0.0f;
static float g_cylBodyY = 0.0f;
static float g_cylWorldX = 0.0f;
static float g_cylWorldY = 0.0f;
static float g_cylRadius = FLOW_OBS_CANDIDATE_RADIUS_M;
static float g_cylReject = 0.0f;
static float g_cylInnov = 0.0f;
static float g_cylVarX = FLOW_OBS_CYL_COV_INIT_M2;
static float g_cylVarY = FLOW_OBS_CYL_COV_INIT_M2;
static float g_cylCovXY = 0.0f;

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static void flowObstacleDecayCylinder(void) {
  g_cylConf -= FLOW_OBS_CYL_CONF_DOWN;
  if (g_cylConf < 0.0f) {
    g_cylConf = 0.0f;
  }
  g_cylAge += 1.0f;
  g_cylInnov = 0.0f;
  g_cylVarX += FLOW_OBS_CYL_COV_INFLATE_M2;
  g_cylVarY += FLOW_OBS_CYL_COV_INFLATE_M2;
  if (g_cylVarX > FLOW_OBS_CYL_COV_MAX_M2) {
    g_cylVarX = FLOW_OBS_CYL_COV_MAX_M2;
  }
  if (g_cylVarY > FLOW_OBS_CYL_COV_MAX_M2) {
    g_cylVarY = FLOW_OBS_CYL_COV_MAX_M2;
  }
  g_cylValid = g_cylConf >= FLOW_OBS_CYL_VALID_CONF ? 1.0f : 0.0f;
}

static void flowObstacleClearFrameDerived(void) {
  for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
    g_resFlow[i] = 0.0f;
    g_velEff[i] = 0.0f;
    g_invDepth[i] = 0.0f;
    g_range[i] = 0.0f;
    g_valid[i] = 0.0f;
    g_bodyX[i] = 0.0f;
    g_bodyY[i] = 0.0f;
    g_worldX[i] = 0.0f;
    g_worldY[i] = 0.0f;
  }
  g_nearValid = 0.0f;
  g_nearIdx = 0;
  g_nearRange = 0.0f;
  g_nearBodyX = 0.0f;
  g_nearBodyY = 0.0f;
  g_nearWorldX = 0.0f;
  g_nearWorldY = 0.0f;
  g_obsValid = 0.0f;
  g_obsHits = 0;
  g_obsClusterStart = 0;
  g_obsClusterCount = 0;
  g_obsRange = 0.0f;
  g_obsBearing = 0.0f;
  g_obsBodyX = 0.0f;
  g_obsBodyY = 0.0f;
  g_obsWorldX = 0.0f;
  g_obsWorldY = 0.0f;
}

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
                                 float yaw_rate_rad_s,
                                 float world_x_m,
                                 float world_y_m,
                                 float yaw_rad) {
  flow_obstacle_payload_t payload;
  uint32_t age_ms = 0;
  uint32_t sample = 0;
  if (!flowObstacleLinkGetLatest(&payload, &age_ms, &sample) || age_ms > 500) {
    flowObstacleClearFrameDerived();
    flowObstacleDecayCylinder();
    return;
  }

  g_bodyVx = body_vx_m_s;
  g_bodyVy = body_vy_m_s;
  g_yawRate = yaw_rate_rad_s;
  g_nearValid = 0.0f;
  g_nearIdx = 0;
  g_nearRange = 0.0f;
  g_nearBodyX = 0.0f;
  g_nearBodyY = 0.0f;
  g_nearWorldX = 0.0f;
  g_nearWorldY = 0.0f;

  const float yaw_c = cosf(yaw_rad);
  const float yaw_s = sinf(yaw_rad);
  uint8_t candidate_ok[FLOW_OBS_SECT_MAX] = {0};

  for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
    if (i >= payload.n_sectors || payload.sector[i].confidence < FLOW_MIN_CONFIDENCE) {
      g_resFlow[i] = 0.0f;
      g_velEff[i] = 0.0f;
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
      g_bodyX[i] = 0.0f;
      g_bodyY[i] = 0.0f;
      g_worldX[i] = 0.0f;
      g_worldY[i] = 0.0f;
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
      g_bodyX[i] = 0.0f;
      g_bodyY[i] = 0.0f;
      g_worldX[i] = 0.0f;
      g_worldY[i] = 0.0f;
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

    g_bodyX[i] = g_range[i] * cosf(az);
    g_bodyY[i] = g_range[i] * sinf(az);
    g_worldX[i] = world_x_m + yaw_c * g_bodyX[i] - yaw_s * g_bodyY[i];
    g_worldY[i] = world_y_m + yaw_s * g_bodyX[i] + yaw_c * g_bodyY[i];

    if (g_nearValid == 0.0f || g_range[i] < g_nearRange) {
      g_nearValid = 1.0f;
      g_nearIdx = i;
      g_nearRange = g_range[i];
      g_nearBodyX = g_bodyX[i];
      g_nearBodyY = g_bodyY[i];
      g_nearWorldX = g_worldX[i];
      g_nearWorldY = g_worldY[i];
    }

    if (fabsf(yaw_rate_rad_s) < FLOW_OBS_CANDIDATE_MAX_YAW_RATE_RAD_S &&
        g_range[i] > 0.0f &&
        g_range[i] < FLOW_OBS_CANDIDATE_MAX_RANGE_M) {
      candidate_ok[i] = 1;
    }
  }

  uint8_t best_start = 0;
  uint8_t best_count = 0;
  float best_range_sum = 0.0f;
  uint8_t start = 0;
  while (start < FLOW_OBS_SECT_MAX) {
    while (start < FLOW_OBS_SECT_MAX && !candidate_ok[start]) {
      start++;
    }
    if (start >= FLOW_OBS_SECT_MAX) {
      break;
    }

    uint8_t count = 0;
    float range_sum = 0.0f;
    while ((uint8_t)(start + count) < FLOW_OBS_SECT_MAX && candidate_ok[start + count]) {
      range_sum += g_range[start + count];
      count++;
    }

    if (count > best_count ||
        (count == best_count && count > 0 && range_sum < best_range_sum)) {
      best_start = start;
      best_count = count;
      best_range_sum = range_sum;
    }
    start += count;
  }

  g_obsClusterStart = best_start;
  g_obsClusterCount = best_count;
  bool obs_fresh = false;

  if (best_count >= FLOW_OBS_CANDIDATE_MIN_SECTORS) {
    obs_fresh = true;
    float weight_sum = 0.0f;
    float body_x_sum = 0.0f;
    float body_y_sum = 0.0f;
    float range_sum = 0.0f;
    for (uint8_t j = 0; j < best_count; j++) {
      const uint8_t i = best_start + j;
      const float weight = payload.sector[i].confidence > 1.0e-3f ?
                           payload.sector[i].confidence : 1.0e-3f;
      weight_sum += weight;
      body_x_sum += weight * g_bodyX[i];
      body_y_sum += weight * g_bodyY[i];
      range_sum += g_range[i];
    }

    const float raw_body_x = body_x_sum / weight_sum;
    const float raw_body_y = body_y_sum / weight_sum;

    if (g_obsHits == 0) {
      g_obsBodyX = raw_body_x;
      g_obsBodyY = raw_body_y;
    } else {
      g_obsBodyX += FLOW_OBS_CANDIDATE_ALPHA * (raw_body_x - g_obsBodyX);
      g_obsBodyY += FLOW_OBS_CANDIDATE_ALPHA * (raw_body_y - g_obsBodyY);
    }

    if (g_obsHits < 255) {
      g_obsHits++;
    }
    g_obsRange = range_sum / (float)best_count;
    g_obsBearing = atan2f(g_obsBodyY, g_obsBodyX);
    g_obsWorldX = world_x_m + yaw_c * g_obsBodyX - yaw_s * g_obsBodyY;
    g_obsWorldY = world_y_m + yaw_s * g_obsBodyX + yaw_c * g_obsBodyY;
    g_obsValid = g_obsHits >= 2 ? 1.0f : 0.0f;
  } else {
    if (g_obsHits > 0) {
      g_obsHits--;
    }
    g_obsValid = g_obsHits >= 2 ? 1.0f : 0.0f;
  }

  if (obs_fresh && g_obsValid > 0.5f) {
    const float dx = g_obsBodyX - g_cylBodyX;
    const float dy = g_obsBodyY - g_cylBodyY;
    g_cylInnov = sqrtf(dx * dx + dy * dy);
    const float gate = FLOW_OBS_CYL_GATE_BASE_M +
                       FLOW_OBS_CYL_GATE_RANGE_FRAC * g_cylBodyX;
    const bool accept_obs = g_cylConf < FLOW_OBS_CYL_GATE_CONF ||
                            g_cylInnov <= gate;

    if (accept_obs && g_cylConf <= 0.0f) {
      g_cylBodyX = g_obsBodyX;
      g_cylBodyY = g_obsBodyY;
      g_cylWorldX = g_obsWorldX;
      g_cylWorldY = g_obsWorldY;
      g_cylReject = 0.0f;
      g_cylVarX = FLOW_OBS_CYL_COV_INIT_M2;
      g_cylVarY = FLOW_OBS_CYL_COV_INIT_M2;
      g_cylCovXY = 0.0f;
    } else if (accept_obs) {
      g_cylBodyX += FLOW_OBS_CYL_ALPHA * (g_obsBodyX - g_cylBodyX);
      g_cylBodyY += FLOW_OBS_CYL_ALPHA * (g_obsBodyY - g_cylBodyY);
      g_cylWorldX += FLOW_OBS_CYL_ALPHA * (g_obsWorldX - g_cylWorldX);
      g_cylWorldY += FLOW_OBS_CYL_ALPHA * (g_obsWorldY - g_cylWorldY);
      g_cylReject = 0.0f;

      const float rx = g_obsBodyX - g_cylBodyX;
      const float ry = g_obsBodyY - g_cylBodyY;
      g_cylVarX += FLOW_OBS_CYL_COV_ALPHA * (rx * rx - g_cylVarX);
      g_cylVarY += FLOW_OBS_CYL_COV_ALPHA * (ry * ry - g_cylVarY);
      g_cylCovXY += FLOW_OBS_CYL_COV_ALPHA * (rx * ry - g_cylCovXY);
    } else {
      g_cylReject += 1.0f;
      g_cylVarX += FLOW_OBS_CYL_COV_INFLATE_M2;
      g_cylVarY += FLOW_OBS_CYL_COV_INFLATE_M2;
    }
    if (accept_obs) {
      g_cylConf += FLOW_OBS_CYL_CONF_UP * (1.0f - g_cylConf);
      g_cylAge = 0.0f;
    } else {
      g_cylConf -= FLOW_OBS_CYL_CONF_DOWN;
      if (g_cylConf < 0.0f) {
        g_cylConf = 0.0f;
      }
      g_cylAge += 1.0f;
    }
  } else {
    flowObstacleDecayCylinder();
  }
  g_cylValid = g_cylConf >= FLOW_OBS_CYL_VALID_CONF ? 1.0f : 0.0f;
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

bool flowObstacleLinkGetCylinder(float *out_world_x_m,
                                 float *out_world_y_m,
                                 float *out_radius_m,
                                 float *out_confidence) {
  if (g_cylValid < 0.5f) {
    return false;
  }
  if (out_world_x_m) {
    *out_world_x_m = g_cylWorldX;
  }
  if (out_world_y_m) {
    *out_world_y_m = g_cylWorldY;
  }
  if (out_radius_m) {
    *out_radius_m = g_cylRadius;
  }
  if (out_confidence) {
    *out_confidence = g_cylConf;
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
LOG_ADD(LOG_FLOAT,  bx0,    &g_bodyX[0])
LOG_ADD(LOG_FLOAT,  bx1,    &g_bodyX[1])
LOG_ADD(LOG_FLOAT,  bx2,    &g_bodyX[2])
LOG_ADD(LOG_FLOAT,  bx3,    &g_bodyX[3])
LOG_ADD(LOG_FLOAT,  bx4,    &g_bodyX[4])
LOG_ADD(LOG_FLOAT,  bx5,    &g_bodyX[5])
LOG_ADD(LOG_FLOAT,  bx6,    &g_bodyX[6])
LOG_ADD(LOG_FLOAT,  bx7,    &g_bodyX[7])
LOG_ADD(LOG_FLOAT,  bx8,    &g_bodyX[8])
LOG_ADD(LOG_FLOAT,  by0,    &g_bodyY[0])
LOG_ADD(LOG_FLOAT,  by1,    &g_bodyY[1])
LOG_ADD(LOG_FLOAT,  by2,    &g_bodyY[2])
LOG_ADD(LOG_FLOAT,  by3,    &g_bodyY[3])
LOG_ADD(LOG_FLOAT,  by4,    &g_bodyY[4])
LOG_ADD(LOG_FLOAT,  by5,    &g_bodyY[5])
LOG_ADD(LOG_FLOAT,  by6,    &g_bodyY[6])
LOG_ADD(LOG_FLOAT,  by7,    &g_bodyY[7])
LOG_ADD(LOG_FLOAT,  by8,    &g_bodyY[8])
LOG_ADD(LOG_FLOAT,  nearValid, &g_nearValid)
LOG_ADD(LOG_UINT8,  nearIdx,   &g_nearIdx)
LOG_ADD(LOG_FLOAT,  nearRange, &g_nearRange)
LOG_ADD(LOG_FLOAT,  nearBx,    &g_nearBodyX)
LOG_ADD(LOG_FLOAT,  nearBy,    &g_nearBodyY)
LOG_ADD(LOG_FLOAT,  nearWx,    &g_nearWorldX)
LOG_ADD(LOG_FLOAT,  nearWy,    &g_nearWorldY)
LOG_ADD(LOG_FLOAT,  obsValid,  &g_obsValid)
LOG_ADD(LOG_UINT8,  obsHits,   &g_obsHits)
LOG_ADD(LOG_UINT8,  obsStart,  &g_obsClusterStart)
LOG_ADD(LOG_UINT8,  obsCount,  &g_obsClusterCount)
LOG_ADD(LOG_FLOAT,  obsRange,  &g_obsRange)
LOG_ADD(LOG_FLOAT,  obsBear,   &g_obsBearing)
LOG_ADD(LOG_FLOAT,  obsBx,     &g_obsBodyX)
LOG_ADD(LOG_FLOAT,  obsBy,     &g_obsBodyY)
LOG_ADD(LOG_FLOAT,  obsWx,     &g_obsWorldX)
LOG_ADD(LOG_FLOAT,  obsWy,     &g_obsWorldY)
LOG_ADD(LOG_FLOAT,  obsRadius, &g_obsRadius)
LOG_ADD(LOG_FLOAT,  cylValid,  &g_cylValid)
LOG_ADD(LOG_FLOAT,  cylConf,   &g_cylConf)
LOG_ADD(LOG_FLOAT,  cylAge,    &g_cylAge)
LOG_ADD(LOG_FLOAT,  cylBx,     &g_cylBodyX)
LOG_ADD(LOG_FLOAT,  cylBy,     &g_cylBodyY)
LOG_ADD(LOG_FLOAT,  cylWx,     &g_cylWorldX)
LOG_ADD(LOG_FLOAT,  cylWy,     &g_cylWorldY)
LOG_ADD(LOG_FLOAT,  cylRadius, &g_cylRadius)
LOG_ADD(LOG_FLOAT,  cylReject, &g_cylReject)
LOG_ADD(LOG_FLOAT,  cylInnov,  &g_cylInnov)
LOG_ADD(LOG_FLOAT,  cylVarX,   &g_cylVarX)
LOG_ADD(LOG_FLOAT,  cylVarY,   &g_cylVarY)
LOG_ADD(LOG_FLOAT,  cylCovXY,  &g_cylCovXY)
LOG_GROUP_STOP(flowObsRx)
