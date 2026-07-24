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
#include "param.h"

#include <math.h>
#include <string.h>

#define FLOW_MIN_CONFIDENCE 0.02f
#define FLOW_MIN_TRANSLATION_M_S 0.08f
#define FLOW_MIN_FORWARD_LOOMING_M_S 0.15f
#define FLOW_MAX_LOOMING_YAW_RATE_RAD_S 0.20f
#define FLOW_MAX_INV_DEPTH_M 8.0f
#define FLOW_MAX_RANGE_M 10.0f
#define FLOW_OBS_CANDIDATE_MAX_RANGE_M 3.0f
#define FLOW_OBS_CANDIDATE_MAX_YAW_RATE_RAD_S 0.6f
#define FLOW_OBS_CANDIDATE_MIN_SECTORS 1
#define FLOW_OBS_GROUP_RADIUS_M 0.60f
#define FLOW_OBS_MAX_RANGE_DISPERSION_M 0.55f
#define FLOW_OBS_PERSIST_SPATIAL_GATE_M 0.55f
#define FLOW_OBS_PERSIST_WINDOW 3
#define FLOW_OBS_PERSIST_REQUIRED 2
#define FLOW_OBS_MIN_AGG_DISPLACEMENT_RAD 0.004f
#define FLOW_OBS_MAX_YAW_EXPLAINED_RATIO 0.80f
#define FLOW_OBS_MAX_INV_DEPTH_DISAGREEMENT_M 0.75f
#define FLOW_OBS_CANDIDATE_RADIUS_M 0.25f
#define FLOW_OBS_CYL_ALPHA 0.20f
#define FLOW_OBS_CYL_CONF_UP 0.20f
#define FLOW_OBS_CYL_CONF_DOWN 0.01f
#define FLOW_OBS_CYL_VALID_CONF 0.10f
#define FLOW_OBS_CYL_VALID_ACCEPTS 1
#define FLOW_OBS_CYL_FAR_RANGE_M 0.75f
#define FLOW_OBS_CYL_FAR_VALID_ACCEPTS 4
#define FLOW_OBS_CYL_GATE_CONF 0.35f
#define FLOW_OBS_CYL_GATE_BASE_M 0.25f
#define FLOW_OBS_CYL_GATE_RANGE_FRAC 0.20f
#define FLOW_OBS_CYL_GATE_MAX_M 0.50f
#define FLOW_OBS_CYL_SWITCH_MARGIN_M 0.50f
#define FLOW_OBS_CYL_COV_ALPHA 0.15f
#define FLOW_OBS_CYL_COV_INIT_M2 0.04f
#define FLOW_OBS_CYL_COV_INFLATE_M2 0.002f
#define FLOW_OBS_CYL_COV_MAX_M2 4.0f
#define FLOW_OBS_MAP_CELLS 16
#define FLOW_OBS_MAP_DECAY 0.999f
#define FLOW_OBS_MAP_STALE_DECAY 0.995f
#define FLOW_OBS_MAP_MIN_EVIDENCE 0.03f
#define FLOW_OBS_MAP_VOTE 0.18f
#define FLOW_OBS_MAP_MERGE_RADIUS_M 0.30f
#define FLOW_OBS_MAP_EXTRACT_RADIUS_M 0.30f
#define FLOW_OBS_MAP_VALID_EVIDENCE 0.10f
#define FLOW_OBS_MAP_CELL_SIGMA_M2 0.01f
#define FLOW_OBS_STATE_HISTORY 64
#define FLOW_OBS_MAX_SYNC_ERROR_MS 100
#define FLOW_OBS_VELOCITY_SIGMA_M_S 0.03f
#define FLOW_OBS_GYRO_SIGMA_RAD_S 0.02f
#define FLOW_OBS_MAX_RANGE_SIGMA_M 0.35f
#define FLOW_OBS_MAX_REL_RANGE_SIGMA 0.60f

static volatile uint32_t g_seq = 0;
static flow_obstacle_payload_t g_payload;
static volatile uint32_t g_trackSeqLock = 0;
static flow_track_payload_t g_trackPayload;
static volatile uint32_t g_trackRxTick = 0;
static volatile uint32_t g_trackRxOk = 0;
static volatile uint32_t g_trackInvalidRx = 0;
static volatile uint32_t g_trackDupRx = 0;
static uint16_t g_lastTrackWireSeq = 0;
static bool g_haveTrackWireSeq = false;
static volatile uint32_t g_trackWireSeqGaps = 0;
static volatile uint32_t g_rxTick = 0;
static volatile uint32_t g_rxOk = 0;
static volatile uint32_t g_crcErr = 0;
static volatile uint32_t g_badRx = 0;
static volatile uint32_t g_dupRx = 0;
static volatile uint32_t g_invalidRx = 0;
static uint16_t g_lastWireSeq = 0;
static bool g_haveWireSeq = false;
static volatile uint32_t g_wireSeqGaps = 0;
static volatile uint32_t g_wireSeqResets = 0;
static uint32_t g_sampleAgeMs = 0;
static uint32_t g_newSamplesProcessed = 0;
static uint32_t g_mapEvidenceVotes = 0;
static uint8_t g_lastSampleWasNew = 0;
static float g_bodyVx = 0.0f;
static float g_bodyVy = 0.0f;
static float g_yawRate = 0.0f;
static float g_resFlow[FLOW_OBS_SECT_MAX] = {0};
static float g_velEff[FLOW_OBS_SECT_MAX] = {0};
static float g_invDepth[FLOW_OBS_SECT_MAX] = {0};
static float g_range[FLOW_OBS_SECT_MAX] = {0};
static float g_rangeSigma[FLOW_OBS_SECT_MAX] = {0};
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
static float g_obsHistoryX[FLOW_OBS_PERSIST_WINDOW] = {0};
static float g_obsHistoryY[FLOW_OBS_PERSIST_WINDOW] = {0};
static uint8_t g_obsHistoryValid[FLOW_OBS_PERSIST_WINDOW] = {0};
static uint8_t g_obsHistoryHead = 0;
static uint8_t g_rejectReason = 0;
static uint32_t g_rejectLowMotion = 0;
static uint32_t g_rejectYaw = 0;
static uint32_t g_rejectDepthDisagree = 0;
static uint32_t g_rejectDispersion = 0;
static uint32_t g_rejectNoGroup = 0;
static float g_aggregateDisplacement = 0.0f;
static float g_yawExplainedRatio = 0.0f;
static float g_depthDisagreement = 0.0f;
static float g_groupDispersion = 0.0f;
static float g_groupScore = 0.0f;
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
static uint8_t g_cylAccepts = 0;
static float g_cylVarX = FLOW_OBS_CYL_COV_INIT_M2;
static float g_cylVarY = FLOW_OBS_CYL_COV_INIT_M2;
static float g_cylCovXY = 0.0f;
static uint32_t g_lastDepthSample = 0;
static float g_mapX[FLOW_OBS_MAP_CELLS] = {0};
static float g_mapY[FLOW_OBS_MAP_CELLS] = {0};
static float g_mapEvidence[FLOW_OBS_MAP_CELLS] = {0};
static uint8_t g_mapHits[FLOW_OBS_MAP_CELLS] = {0};
typedef struct {
  uint32_t timestamp_ms;
  float body_vx;
  float body_vy;
  float yaw_rate;
  float world_x;
  float world_y;
  float yaw;
} flow_state_sample_t;
static flow_state_sample_t g_stateHistory[FLOW_OBS_STATE_HISTORY];
static uint8_t g_stateHistoryHead = 0;
static uint8_t g_stateHistoryCount = 0;
static uint8_t g_syncValid = 0;
static uint32_t g_syncErrorMs = 0;
static uint32_t g_syncMiss = 0;
static float g_cameraYawRad = 0.0f;
static float g_cameraForwardM = 0.0f;
static float g_cameraLeftM = 0.0f;
static float g_mapPeak = 0.0f;
static uint8_t g_mapActive = 0;
static uint8_t g_mapBestIdx = 0;
static uint8_t g_resetRequested = 0;
static uint32_t g_resetCount = 0;

#define COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")

static uint8_t flowObstacleGroupRoot(uint8_t parent[FLOW_OBS_SECT_MAX],
                                     uint8_t index) {
  while (parent[index] != index) {
    parent[index] = parent[parent[index]];
    index = parent[index];
  }
  return index;
}

static bool flowObstacleCylinderMeetsValidity(void) {
  const float range = sqrtf(g_cylBodyX * g_cylBodyX +
                            g_cylBodyY * g_cylBodyY);
  const uint8_t required_accepts =
      range > FLOW_OBS_CYL_FAR_RANGE_M ?
      FLOW_OBS_CYL_FAR_VALID_ACCEPTS : FLOW_OBS_CYL_VALID_ACCEPTS;
  return g_cylConf >= FLOW_OBS_CYL_VALID_CONF &&
         g_cylAccepts >= required_accepts;
}

static void flowObstacleDecayMap(float decay) {
  g_mapActive = 0;
  g_mapPeak = 0.0f;
  g_mapBestIdx = 0;
  for (uint8_t i = 0; i < FLOW_OBS_MAP_CELLS; i++) {
    g_mapEvidence[i] *= decay;
    if (g_mapEvidence[i] < FLOW_OBS_MAP_MIN_EVIDENCE) {
      g_mapEvidence[i] = 0.0f;
      g_mapHits[i] = 0;
      continue;
    }
    g_mapActive++;
    if (g_mapEvidence[i] > g_mapPeak) {
      g_mapPeak = g_mapEvidence[i];
      g_mapBestIdx = i;
    }
  }
}

static void flowObstacleVoteMap(float wx, float wy, float weight) {
  uint8_t weakest_idx = 0;
  float weakest_ev = g_mapEvidence[0];
  bool merged = false;
  const float merge_r2 = FLOW_OBS_MAP_MERGE_RADIUS_M * FLOW_OBS_MAP_MERGE_RADIUS_M;

  for (uint8_t i = 0; i < FLOW_OBS_MAP_CELLS; i++) {
    if (g_mapEvidence[i] < weakest_ev) {
      weakest_ev = g_mapEvidence[i];
      weakest_idx = i;
    }
    if (g_mapEvidence[i] <= 0.0f) {
      continue;
    }
    const float dx = wx - g_mapX[i];
    const float dy = wy - g_mapY[i];
    const float d2 = dx * dx + dy * dy;
    if (d2 <= merge_r2) {
      const float proximity = 1.0f - d2 / merge_r2;
      const float vote = FLOW_OBS_MAP_VOTE * weight * proximity;
      const float alpha = 0.12f * weight * proximity;
      g_mapX[i] += alpha * (wx - g_mapX[i]);
      g_mapY[i] += alpha * (wy - g_mapY[i]);
      g_mapEvidence[i] += vote;
      if (g_mapEvidence[i] > 1.0f) {
        g_mapEvidence[i] = 1.0f;
      }
      if (g_mapHits[i] < 255) {
        g_mapHits[i]++;
      }
      merged = true;
    }
  }

  if (!merged) {
    g_mapX[weakest_idx] = wx;
    g_mapY[weakest_idx] = wy;
    g_mapEvidence[weakest_idx] = FLOW_OBS_MAP_VOTE * weight;
    g_mapHits[weakest_idx] = 1;
  }
}

static void flowObstacleExtractCylinder(float world_x_m,
                                        float world_y_m,
                                        float yaw_rad) {
  flowObstacleDecayMap(1.0f);

  if (g_mapPeak < FLOW_OBS_MAP_VALID_EVIDENCE) {
    g_cylConf = g_mapPeak;
    g_cylValid = 0.0f;
    g_cylAge += 1.0f;
    return;
  }

  /* A map vote already represents spatial agreement across at least two
   * distinct camera samples.  Among such temporally validated components,
   * publish the nearest hazard rather than allowing a well-textured distant
   * wall to displace a closer foreground obstacle. */
  uint8_t nearest_idx = g_mapBestIdx;
  float nearest_d2 = INFINITY;
  float nearest_evidence = 0.0f;
  uint8_t tracked_idx = g_mapBestIdx;
  float tracked_innovation_d2 = INFINITY;
  for (uint8_t i = 0; i < FLOW_OBS_MAP_CELLS; i++) {
    if (g_mapEvidence[i] < FLOW_OBS_MAP_VALID_EVIDENCE) {
      continue;
    }
    const float dx = g_mapX[i] - world_x_m;
    const float dy = g_mapY[i] - world_y_m;
    const float d2 = dx * dx + dy * dy;
    if (d2 < nearest_d2 ||
        (d2 == nearest_d2 && g_mapEvidence[i] > nearest_evidence)) {
      nearest_idx = i;
      nearest_d2 = d2;
      nearest_evidence = g_mapEvidence[i];
    }
    const float track_dx = g_mapX[i] - g_cylWorldX;
    const float track_dy = g_mapY[i] - g_cylWorldY;
    const float track_d2 = track_dx * track_dx + track_dy * track_dy;
    if (track_d2 < tracked_innovation_d2) {
      tracked_idx = i;
      tracked_innovation_d2 = track_d2;
    }
  }

  /* Preserve association with the published component through modest state
   * and map jitter. A new component may take over only when it is materially
   * closer to the vehicle; this still lets a near foreground obstacle replace
   * a previously confirmed far wall. */
  if (g_cylConf >= FLOW_OBS_CYL_VALID_CONF) {
    const float old_dx = g_cylWorldX - world_x_m;
    const float old_dy = g_cylWorldY - world_y_m;
    const float old_range = sqrtf(old_dx * old_dx + old_dy * old_dy);
    float track_gate = FLOW_OBS_CYL_GATE_BASE_M +
                       FLOW_OBS_CYL_GATE_RANGE_FRAC * old_range;
    if (track_gate > FLOW_OBS_CYL_GATE_MAX_M) {
      track_gate = FLOW_OBS_CYL_GATE_MAX_M;
    }
    if (tracked_innovation_d2 <= track_gate * track_gate) {
      const float tracked_dx = g_mapX[tracked_idx] - world_x_m;
      const float tracked_dy = g_mapY[tracked_idx] - world_y_m;
      const float tracked_range =
          sqrtf(tracked_dx * tracked_dx + tracked_dy * tracked_dy);
      const float nearest_range = sqrtf(nearest_d2);
      if (nearest_range + FLOW_OBS_CYL_SWITCH_MARGIN_M >= tracked_range) {
        nearest_idx = tracked_idx;
      }
    }
  }
  g_mapBestIdx = nearest_idx;
  const float best_x = g_mapX[nearest_idx];
  const float best_y = g_mapY[nearest_idx];
  const float extract_r2 = FLOW_OBS_MAP_EXTRACT_RADIUS_M * FLOW_OBS_MAP_EXTRACT_RADIUS_M;
  float w_sum = 0.0f;
  float x_sum = 0.0f;
  float y_sum = 0.0f;
  uint8_t support = 0;

  for (uint8_t i = 0; i < FLOW_OBS_MAP_CELLS; i++) {
    if (g_mapEvidence[i] <= 0.0f) {
      continue;
    }
    const float dx = g_mapX[i] - best_x;
    const float dy = g_mapY[i] - best_y;
    if (dx * dx + dy * dy > extract_r2) {
      continue;
    }
    const float w = g_mapEvidence[i];
    w_sum += w;
    x_sum += w * g_mapX[i];
    y_sum += w * g_mapY[i];
    support++;
  }

  if (w_sum <= 1.0e-3f || support == 0) {
    g_cylValid = 0.0f;
    return;
  }

  const float new_wx = x_sum / w_sum;
  const float new_wy = y_sum / w_sum;
  const float old_wx = g_cylWorldX;
  const float old_wy = g_cylWorldY;
  g_cylInnov = sqrtf((new_wx - old_wx) * (new_wx - old_wx) +
                     (new_wy - old_wy) * (new_wy - old_wy));
  g_cylWorldX = new_wx;
  g_cylWorldY = new_wy;

  float var_x = FLOW_OBS_MAP_CELL_SIGMA_M2;
  float var_y = FLOW_OBS_MAP_CELL_SIGMA_M2;
  float cov_xy = 0.0f;
  for (uint8_t i = 0; i < FLOW_OBS_MAP_CELLS; i++) {
    if (g_mapEvidence[i] <= 0.0f) {
      continue;
    }
    const float dx_best = g_mapX[i] - best_x;
    const float dy_best = g_mapY[i] - best_y;
    if (dx_best * dx_best + dy_best * dy_best > extract_r2) {
      continue;
    }
    const float dx = g_mapX[i] - g_cylWorldX;
    const float dy = g_mapY[i] - g_cylWorldY;
    const float w = g_mapEvidence[i] / w_sum;
    var_x += w * dx * dx;
    var_y += w * dy * dy;
    cov_xy += w * dx * dy;
  }
  g_cylVarX = var_x;
  g_cylVarY = var_y;
  g_cylCovXY = cov_xy;

  const float yaw_c = cosf(yaw_rad);
  const float yaw_s = sinf(yaw_rad);
  const float dx_body = g_cylWorldX - world_x_m;
  const float dy_body = g_cylWorldY - world_y_m;
  g_cylBodyX = yaw_c * dx_body + yaw_s * dy_body;
  g_cylBodyY = -yaw_s * dx_body + yaw_c * dy_body;
  g_cylConf = g_mapEvidence[nearest_idx];
  g_cylAccepts = g_mapHits[nearest_idx];
  g_cylAge = 0.0f;
  g_cylReject = 0.0f;
  g_cylValid = flowObstacleCylinderMeetsValidity() ? 1.0f : 0.0f;
}

static void flowObstacleDecayCylinder(float world_x_m,
                                      float world_y_m,
                                      float yaw_rad) {
  flowObstacleDecayMap(FLOW_OBS_MAP_STALE_DECAY);
  flowObstacleExtractCylinder(world_x_m, world_y_m, yaw_rad);
  g_cylAge += 1.0f;
  g_cylVarX += FLOW_OBS_CYL_COV_INFLATE_M2;
  g_cylVarY += FLOW_OBS_CYL_COV_INFLATE_M2;
  if (g_cylVarX > FLOW_OBS_CYL_COV_MAX_M2) {
    g_cylVarX = FLOW_OBS_CYL_COV_MAX_M2;
  }
  if (g_cylVarY > FLOW_OBS_CYL_COV_MAX_M2) {
    g_cylVarY = FLOW_OBS_CYL_COV_MAX_M2;
  }
  g_cylValid = flowObstacleCylinderMeetsValidity() ? 1.0f : 0.0f;
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

static void flowObstacleResetEstimator(void) {
  flowObstacleClearFrameDerived();
  memset(g_obsHistoryX, 0, sizeof(g_obsHistoryX));
  memset(g_obsHistoryY, 0, sizeof(g_obsHistoryY));
  memset(g_obsHistoryValid, 0, sizeof(g_obsHistoryValid));
  g_obsHistoryHead = 0;
  g_rejectReason = 0;
  g_rejectLowMotion = 0;
  g_rejectYaw = 0;
  g_rejectDepthDisagree = 0;
  g_rejectDispersion = 0;
  g_rejectNoGroup = 0;
  g_aggregateDisplacement = 0.0f;
  g_yawExplainedRatio = 0.0f;
  g_depthDisagreement = 0.0f;
  g_groupDispersion = 0.0f;
  g_groupScore = 0.0f;
  g_cylValid = 0.0f;
  g_cylConf = 0.0f;
  g_cylAge = 0.0f;
  g_cylBodyX = 0.0f;
  g_cylBodyY = 0.0f;
  g_cylWorldX = 0.0f;
  g_cylWorldY = 0.0f;
  g_cylReject = 0.0f;
  g_cylInnov = 0.0f;
  g_cylAccepts = 0;
  g_cylVarX = FLOW_OBS_CYL_COV_INIT_M2;
  g_cylVarY = FLOW_OBS_CYL_COV_INIT_M2;
  g_cylCovXY = 0.0f;
  g_lastDepthSample = 0;
  memset(g_mapX, 0, sizeof(g_mapX));
  memset(g_mapY, 0, sizeof(g_mapY));
  memset(g_mapEvidence, 0, sizeof(g_mapEvidence));
  memset(g_mapHits, 0, sizeof(g_mapHits));
  g_mapPeak = 0.0f;
  g_mapActive = 0;
  g_mapBestIdx = 0;
  g_mapEvidenceVotes = 0;
  g_resetCount++;
}

void flowObstacleLinkInit(void) {
  memset(&g_payload, 0, sizeof(g_payload));
  memset(&g_trackPayload, 0, sizeof(g_trackPayload));
  flowObstacleResetEstimator();
}

bool flowObstacleLinkPublishFromRx(const flow_obstacle_msg_t *msg) {
  const flow_obstacle_payload_t *p = &msg->p;
  if (p->n_sectors == 0 || p->n_sectors > FLOW_OBS_SECT_MAX ||
      !isfinite(p->dt_s) || p->dt_s < 0.004f || p->dt_s > 0.5f) {
    g_invalidRx++;
    return false;
  }
  for (uint8_t i = 0; i < p->n_sectors; i++) {
    const flow_obstacle_sector_t *sector = &p->sector[i];
    if (!isfinite(sector->azimuth_rad) ||
        !isfinite(sector->flow_x_rad_s) ||
        !isfinite(sector->flow_y_rad_s) ||
        !isfinite(sector->flow_sigma_rad_s) ||
        !isfinite(sector->confidence) ||
        fabsf(sector->azimuth_rad) > 2.0f ||
        fabsf(sector->flow_x_rad_s) > 25.0f ||
        fabsf(sector->flow_y_rad_s) > 25.0f ||
        sector->flow_sigma_rad_s < 0.0f ||
        sector->flow_sigma_rad_s > 25.0f ||
        sector->confidence < 0.0f || sector->confidence > 1.0f) {
      g_invalidRx++;
      return false;
    }
  }
  /* reserved is a nonzero producer sequence in the combined firmware. Keep
   * accepting zero for compatibility with the older flow-only producer. */
  if (p->reserved != 0 && g_haveWireSeq) {
    const uint16_t delta = (uint16_t)(p->reserved - g_lastWireSeq);
    if (delta == 0) {
      g_dupRx++;
      return false;
    }
    if (delta < 0x8000u) {
      g_wireSeqGaps += (uint32_t)(delta - 1u);
    } else {
      /* A producer restart normally jumps backwards. Treat a large modular
       * delta as a reset instead of reporting tens of thousands of losses. */
      g_wireSeqResets++;
    }
  }

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
  if (p->reserved != 0) {
    g_lastWireSeq = p->reserved;
    g_haveWireSeq = true;
  }
  return true;
}

bool flowObstacleLinkPublishTracksFromRx(const flow_track_msg_t *msg) {
  const flow_track_payload_t *p = &msg->p;
  if (p->version != FLOW_TRACK_WIRE_VERSION ||
      p->count > FLOW_TRACK_MAX ||
      p->dt_us < 4000u) {
    g_trackInvalidRx++;
    return false;
  }
  for (uint8_t i = 0; i < p->count; i++) {
    const flow_track_wire_t *track = &p->track[i];
    if (track->u_q4 >= 160u * 16u ||
        track->v_q4 >= 160u * 16u ||
        track->du_q8 < -32 * 256 || track->du_q8 > 32 * 256 ||
        track->dv_q8 < -32 * 256 || track->dv_q8 > 32 * 256 ||
        track->lk_err_q8 > 64u * 256u ||
        track->fb_err_q8 > 8u * 256u) {
      g_trackInvalidRx++;
      return false;
    }
  }
  if (g_haveTrackWireSeq) {
    const uint16_t delta =
        (uint16_t)(p->sequence - g_lastTrackWireSeq);
    if (delta == 0u) {
      g_trackDupRx++;
      return false;
    }
    if (delta < 0x8000u) {
      g_trackWireSeqGaps += (uint32_t)(delta - 1u);
    }
  }

  uint32_t s = g_trackSeqLock + 1u;
  g_trackSeqLock = s;
  COMPILER_BARRIER();
  memcpy(&g_trackPayload, p, sizeof(g_trackPayload));
  g_trackRxTick = xTaskGetTickCount();
  COMPILER_BARRIER();
  g_trackSeqLock = s + 1u;
  g_trackRxOk++;
  g_lastTrackWireSeq = p->sequence;
  g_haveTrackWireSeq = true;
  return true;
}

void flowObstacleLinkNoteBadRx(void) {
  g_badRx++;
}

void flowObstacleLinkNoteCrcErr(void) {
  g_crcErr++;
}

void flowObstacleLinkRecordState(uint32_t timestamp_ms,
                                 float body_vx_m_s,
                                 float body_vy_m_s,
                                 float yaw_rate_rad_s,
                                 float world_x_m,
                                 float world_y_m,
                                 float yaw_rad) {
  flow_state_sample_t *sample = &g_stateHistory[g_stateHistoryHead];
  sample->timestamp_ms = timestamp_ms;
  sample->body_vx = body_vx_m_s;
  sample->body_vy = body_vy_m_s;
  sample->yaw_rate = yaw_rate_rad_s;
  sample->world_x = world_x_m;
  sample->world_y = world_y_m;
  sample->yaw = yaw_rad;
  g_stateHistoryHead =
      (uint8_t)((g_stateHistoryHead + 1u) % FLOW_OBS_STATE_HISTORY);
  if (g_stateHistoryCount < FLOW_OBS_STATE_HISTORY) {
    g_stateHistoryCount++;
  }
}

static float flowLerp(float a, float b, float t) {
  return a + t * (b - a);
}

static bool flowObstacleStateAt(uint32_t timestamp_ms,
                                flow_state_sample_t *out) {
  const flow_state_sample_t *before = NULL;
  const flow_state_sample_t *after = NULL;
  int32_t before_dt = INT32_MIN;
  int32_t after_dt = INT32_MAX;
  for (uint8_t i = 0; i < g_stateHistoryCount; i++) {
    const flow_state_sample_t *sample = &g_stateHistory[i];
    const int32_t dt = (int32_t)(sample->timestamp_ms - timestamp_ms);
    if (dt <= 0 && dt > before_dt) {
      before = sample;
      before_dt = dt;
    }
    if (dt >= 0 && dt < after_dt) {
      after = sample;
      after_dt = dt;
    }
  }
  uint32_t nearest_error = UINT32_MAX;
  if (before) nearest_error = (uint32_t)(-before_dt);
  if (after && (uint32_t)after_dt < nearest_error) {
    nearest_error = (uint32_t)after_dt;
  }
  g_syncErrorMs = nearest_error;
  if (nearest_error > FLOW_OBS_MAX_SYNC_ERROR_MS) return false;
  if (!before) {
    *out = *after;
    return true;
  }
  if (!after) {
    *out = *before;
    return true;
  }
  const int32_t span = after_dt - before_dt;
  const float t = span > 0 ? (float)(-before_dt) / (float)span : 0.0f;
  out->timestamp_ms = timestamp_ms;
  out->body_vx = flowLerp(before->body_vx, after->body_vx, t);
  out->body_vy = flowLerp(before->body_vy, after->body_vy, t);
  out->yaw_rate = flowLerp(before->yaw_rate, after->yaw_rate, t);
  out->world_x = flowLerp(before->world_x, after->world_x, t);
  out->world_y = flowLerp(before->world_y, after->world_y, t);
  const float yaw_delta = atan2f(sinf(after->yaw - before->yaw),
                                 cosf(after->yaw - before->yaw));
  out->yaw = before->yaw + t * yaw_delta;
  return true;
}

void flowObstacleLinkUpdateDepth(float body_vx_m_s,
                                 float body_vy_m_s,
                                 float yaw_rate_rad_s,
                                 float world_x_m,
                                 float world_y_m,
                                 float yaw_rad) {
  if (g_resetRequested) {
    flowObstacleResetEstimator();
    g_resetRequested = 0;
  }
  flow_obstacle_payload_t payload;
  uint32_t age_ms = 0;
  uint32_t sample = 0;
  if (!flowObstacleLinkGetLatest(&payload, &age_ms, &sample) || age_ms > 500) {
    g_sampleAgeMs = age_ms;
    g_lastSampleWasNew = 0;
    flowObstacleClearFrameDerived();
    flowObstacleDecayCylinder(world_x_m, world_y_m, yaw_rad);
    return;
  }
  flow_state_sample_t synchronized;
  g_syncValid = 0;
  if (payload.stm32_ts_echo != 0u &&
      flowObstacleStateAt(payload.stm32_ts_echo, &synchronized)) {
    body_vx_m_s = synchronized.body_vx;
    body_vy_m_s = synchronized.body_vy;
    yaw_rate_rad_s = synchronized.yaw_rate;
    world_x_m = synchronized.world_x;
    world_y_m = synchronized.world_y;
    yaw_rad = synchronized.yaw;
    g_syncValid = 1;
  } else if (payload.stm32_ts_echo != 0u) {
    g_syncMiss++;
  }
  const bool new_sample = sample != g_lastDepthSample;
  g_sampleAgeMs = age_ms;
  g_lastSampleWasNew = new_sample ? 1u : 0u;
  if (!new_sample) {
    flowObstacleExtractCylinder(world_x_m, world_y_m, yaw_rad);
    return;
  }
  g_newSamplesProcessed++;

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
  memset(g_rangeSigma, 0, sizeof(g_rangeSigma));

  const float yaw_c = cosf(yaw_rad);
  const float yaw_s = sinf(yaw_rad);
  const float camera_yaw_c = cosf(g_cameraYawRad);
  const float camera_yaw_s = sinf(g_cameraYawRad);
  const float camera_body_vx =
      body_vx_m_s - yaw_rate_rad_s * g_cameraLeftM;
  const float camera_body_vy =
      body_vy_m_s + yaw_rate_rad_s * g_cameraForwardM;
  const float camera_vx =
      camera_yaw_c * camera_body_vx + camera_yaw_s * camera_body_vy;
  const float camera_vy =
      -camera_yaw_s * camera_body_vx + camera_yaw_c * camera_body_vy;
  uint8_t candidate_ok[FLOW_OBS_SECT_MAX] = {0};
  g_aggregateDisplacement = 0.0f;
  float yaw_displacement = 0.0f;
  g_depthDisagreement = 0.0f;
  g_groupDispersion = 0.0f;
  g_groupScore = 0.0f;
  g_rejectReason = 0;

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

    /* GAP8 sends normalized pinhole image coordinate q=(u-cx)/fx and its
     * derivative qdot=du/(fx*dt), not literal bearing/radial rate. Convert
     * through bearing=atan(q), whose derivative is qdot/(1+q^2). */
    const float image_q = payload.sector[i].azimuth_rad;
    const float camera_az = atanf(image_q);
    const float az = camera_az + g_cameraYawRad;
    const float measured_flow = payload.sector[i].flow_x_rad_s /
                                (1.0f + image_q * image_q);
    float residual_flow = measured_flow - yaw_rate_rad_s;
    float vel_eff = camera_vx * sinf(camera_az) -
                    camera_vy * cosf(camera_az);
    const float confidence = payload.sector[i].confidence;
    g_aggregateDisplacement += confidence * fabsf(measured_flow) * payload.dt_s;
    yaw_displacement += confidence * fabsf(yaw_rate_rad_s) * payload.dt_s;
    const bool parallax_observable = fabsf(vel_eff) >= FLOW_MIN_TRANSLATION_M_S;
    const bool looming_observable =
        fabsf(camera_vx) >= FLOW_MIN_FORWARD_LOOMING_M_S &&
        fabsf(payload.sector[i].flow_y_rad_s) >= 1.0e-3f;
    if (parallax_observable && looming_observable) {
      const float disagreement = fabsf(residual_flow / vel_eff -
                                         payload.sector[i].flow_y_rad_s / camera_vx);
      if (disagreement > g_depthDisagreement) {
        g_depthDisagreement = disagreement;
      }
      if (disagreement > FLOW_OBS_MAX_INV_DEPTH_DISAGREEMENT_M) {
        g_rejectDepthDisagree++;
        g_rejectReason = 3;
        g_resFlow[i] = residual_flow;
        g_velEff[i] = vel_eff;
        g_invDepth[i] = 0.0f;
        g_range[i] = 0.0f;
        g_valid[i] = 0.0f;
        g_bodyX[i] = 0.0f;
        g_bodyY[i] = 0.0f;
        g_worldX[i] = 0.0f;
        g_worldY[i] = 0.0f;
        continue;
      }
    }

    /* flow_y carries GAP8's sector radial-expansion rate. It makes forward
     * translation observable where horizontal parallax is near zero. */
    const bool use_looming =
        fabsf(vel_eff) < FLOW_MIN_TRANSLATION_M_S &&
        fabsf(camera_vx) >= FLOW_MIN_FORWARD_LOOMING_M_S &&
        fabsf(yaw_rate_rad_s) < FLOW_MAX_LOOMING_YAW_RATE_RAD_S &&
        fabsf(payload.sector[i].flow_y_rad_s) >= 1.0e-3f;
    if (use_looming) {
      residual_flow = payload.sector[i].flow_y_rad_s;
      vel_eff = camera_vx;
    }

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
    if (inv_depth <= 0.0f) {
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
      g_bodyX[i] = 0.0f;
      g_bodyY[i] = 0.0f;
      g_worldX[i] = 0.0f;
      g_worldY[i] = 0.0f;
      continue;
    }
    if (inv_depth > FLOW_MAX_INV_DEPTH_M) {
      inv_depth = FLOW_MAX_INV_DEPTH_M;
    }

    g_invDepth[i] = inv_depth;
    g_range[i] = inv_depth > 1.0e-3f ? (1.0f / inv_depth) : 0.0f;
    if (g_range[i] > FLOW_MAX_RANGE_M) {
      g_range[i] = FLOW_MAX_RANGE_M;
    }
    const float flow_sigma = sqrtf(
        payload.sector[i].flow_sigma_rad_s *
            payload.sector[i].flow_sigma_rad_s +
        FLOW_OBS_GYRO_SIGMA_RAD_S * FLOW_OBS_GYRO_SIGMA_RAD_S);
    const float inv_sigma_flow = flow_sigma / fabsf(vel_eff);
    const float inv_sigma_velocity =
        fabsf(residual_flow) * FLOW_OBS_VELOCITY_SIGMA_M_S /
        (vel_eff * vel_eff);
    const float inv_sigma = sqrtf(inv_sigma_flow * inv_sigma_flow +
                                  inv_sigma_velocity * inv_sigma_velocity);
    g_rangeSigma[i] = inv_sigma / (inv_depth * inv_depth);
    if (g_rangeSigma[i] > FLOW_OBS_MAX_RANGE_SIGMA_M ||
        g_rangeSigma[i] > FLOW_OBS_MAX_REL_RANGE_SIGMA * g_range[i]) {
      g_invDepth[i] = 0.0f;
      g_range[i] = 0.0f;
      g_valid[i] = 0.0f;
      continue;
    }
    g_valid[i] = 1.0f;

    g_bodyX[i] = g_cameraForwardM + g_range[i] * cosf(az);
    g_bodyY[i] = g_cameraLeftM + g_range[i] * sinf(az);
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

  g_yawExplainedRatio = yaw_displacement /
                        (g_aggregateDisplacement > 1.0e-9f ?
                         g_aggregateDisplacement : 1.0e-9f);
  if (g_aggregateDisplacement < FLOW_OBS_MIN_AGG_DISPLACEMENT_RAD) {
    memset(candidate_ok, 0, sizeof(candidate_ok));
    g_rejectLowMotion++;
    g_rejectReason = 1;
  } else if (g_yawExplainedRatio > FLOW_OBS_MAX_YAW_EXPLAINED_RATIO) {
    memset(candidate_ok, 0, sizeof(candidate_ok));
    g_rejectYaw++;
    g_rejectReason = 2;
  }
  /* The slot at head falls out before evaluating the current sample, leaving
   * exactly the previous M-1 samples for a current-inclusive N-of-M test. */
  g_obsHistoryValid[g_obsHistoryHead] = 0;

  uint8_t best_start = 0;
  uint8_t best_count = 0;
  float best_score = -1.0f;
  uint8_t parent[FLOW_OBS_SECT_MAX];
  for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
    parent[i] = i;
  }
  /* Bounded union-find produces disjoint foreground/background components.
   * Both body-point distance and range must agree; sector adjacency is not
   * required and overlapping seed-ball ties cannot introduce side bias.
   * A one-sector component is deliberately retained here: an obstacle at the
   * edge of the FOV can occupy only one sector in a frame.  It still cannot
   * vote by itself because the independent N-of-M temporal/spatial gate below
   * requires agreement in another frame. */
  for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
    if (!candidate_ok[i]) continue;
    for (uint8_t j = (uint8_t)(i + 1u); j < FLOW_OBS_SECT_MAX; j++) {
      if (!candidate_ok[j]) continue;
      const float dx = g_bodyX[i] - g_bodyX[j];
      const float dy = g_bodyY[i] - g_bodyY[j];
      if (dx * dx + dy * dy <=
            FLOW_OBS_GROUP_RADIUS_M * FLOW_OBS_GROUP_RADIUS_M &&
          fabsf(g_range[i] - g_range[j]) <=
            FLOW_OBS_MAX_RANGE_DISPERSION_M) {
        const uint8_t root_i = flowObstacleGroupRoot(parent, i);
        const uint8_t root_j = flowObstacleGroupRoot(parent, j);
        if (root_i != root_j) parent[root_j] = root_i;
      }
    }
  }
  for (uint8_t component = 0; component < FLOW_OBS_SECT_MAX; component++) {
    if (!candidate_ok[component] ||
        flowObstacleGroupRoot(parent, component) != component) {
      continue;
    }
    uint8_t count = 0;
    float confidence_sum = 0.0f;
    float weight_sum = 0.0f;
    float range_sum = 0.0f;
    float range_min = FLOW_MAX_RANGE_M;
    float range_max = 0.0f;
    float bx_sum = 0.0f;
    float by_sum = 0.0f;
    for (uint8_t i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      if (!candidate_ok[i] ||
          flowObstacleGroupRoot(parent, i) != component) {
        continue;
      }
      const float relative_sigma =
          g_rangeSigma[i] / (g_range[i] > 0.05f ? g_range[i] : 0.05f);
      const float uncertainty_quality =
          1.0f / (1.0f + relative_sigma * relative_sigma);
      const float quality = payload.sector[i].confidence *
                            uncertainty_quality;
      const float variance = g_rangeSigma[i] * g_rangeSigma[i] + 0.0025f;
      const float weight = (quality > 1.0e-3f ? quality : 1.0e-3f) /
                           variance;
      confidence_sum += quality;
      weight_sum += weight;
      bx_sum += weight * g_bodyX[i];
      by_sum += weight * g_bodyY[i];
      range_sum += g_range[i];
      if (g_range[i] < range_min) range_min = g_range[i];
      if (g_range[i] > range_max) range_max = g_range[i];
      count++;
    }
    const float dispersion = range_max - range_min;
    if (count < FLOW_OBS_CANDIDATE_MIN_SECTORS ||
        dispersion > FLOW_OBS_MAX_RANGE_DISPERSION_M) {
      if (dispersion > FLOW_OBS_MAX_RANGE_DISPERSION_M) {
        g_rejectDispersion++;
        g_groupDispersion = dispersion;
      }
      continue;
    }
    const float bx = bx_sum / weight_sum;
    const float by = by_sum / weight_sum;
    float temporal = 1.0f;
    for (uint8_t h = 0; h < FLOW_OBS_PERSIST_WINDOW; h++) {
      if (!g_obsHistoryValid[h]) continue;
      const float dx = bx - g_obsHistoryX[h];
      const float dy = by - g_obsHistoryY[h];
      const float dist = sqrtf(dx * dx + dy * dy);
      const float agreement = 1.0f - dist / FLOW_OBS_PERSIST_SPATIAL_GATE_M;
      if (agreement > temporal - 1.0f) temporal = 1.0f + agreement;
    }
    const float mean_range = range_sum / (float)count;
    const float score = 0.25f * (float)count + 2.0f * confidence_sum +
                        2.0f / (mean_range > 0.2f ? mean_range : 0.2f) +
                        temporal;
    if (score > best_score) {
      best_score = score;
      best_start = component;
      best_count = count;
      g_obsBodyX = bx;
      g_obsBodyY = by;
      g_obsRange = mean_range;
      g_groupDispersion = dispersion;
    }
  }

  g_obsClusterStart = best_start;
  g_obsClusterCount = best_count;
  g_groupScore = best_score > 0.0f ? best_score : 0.0f;
  const bool obs_fresh = best_count >= FLOW_OBS_CANDIDATE_MIN_SECTORS;

  g_obsHits = obs_fresh ? 1 : 0;
  if (obs_fresh) {
    for (uint8_t h = 0; h < FLOW_OBS_PERSIST_WINDOW; h++) {
      if (g_obsHistoryValid[h]) {
        const float dx = g_obsBodyX - g_obsHistoryX[h];
        const float dy = g_obsBodyY - g_obsHistoryY[h];
        if (dx * dx + dy * dy <=
            FLOW_OBS_PERSIST_SPATIAL_GATE_M * FLOW_OBS_PERSIST_SPATIAL_GATE_M) {
          g_obsHits++;
        }
      }
    }
    g_obsBearing = atan2f(g_obsBodyY, g_obsBodyX);
    g_obsWorldX = world_x_m + yaw_c * g_obsBodyX - yaw_s * g_obsBodyY;
    g_obsWorldY = world_y_m + yaw_s * g_obsBodyX + yaw_c * g_obsBodyY;
    g_obsValid = g_obsHits >= FLOW_OBS_PERSIST_REQUIRED ? 1.0f : 0.0f;
  } else {
    g_obsValid = 0.0f;
    if (g_rejectReason == 0) {
      g_rejectNoGroup++;
      g_rejectReason = 5;
    }
  }

  g_obsHistoryValid[g_obsHistoryHead] = obs_fresh ? 1 : 0;
  if (obs_fresh) {
    g_obsHistoryX[g_obsHistoryHead] = g_obsBodyX;
    g_obsHistoryY[g_obsHistoryHead] = g_obsBodyY;
  }
  g_obsHistoryHead = (uint8_t)((g_obsHistoryHead + 1u) %
                               FLOW_OBS_PERSIST_WINDOW);
  g_lastDepthSample = sample;
  flowObstacleDecayMap(FLOW_OBS_MAP_DECAY);
  if (obs_fresh && g_obsValid > 0.5f) {
    float vote_weight = 0.5f + 0.15f * (float)g_obsClusterCount;
    if (vote_weight > 1.0f) {
      vote_weight = 1.0f;
    }
    flowObstacleVoteMap(g_obsWorldX, g_obsWorldY, vote_weight);
    g_mapEvidenceVotes++;
  }
  flowObstacleExtractCylinder(world_x_m, world_y_m, yaw_rad);
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
LOG_ADD(LOG_UINT32, dupRx,  &g_dupRx)
LOG_ADD(LOG_UINT32, invalid, &g_invalidRx)
LOG_ADD(LOG_UINT16, wireSeq, &g_lastWireSeq)
LOG_ADD(LOG_UINT32, seqGap, &g_wireSeqGaps)
LOG_ADD(LOG_UINT32, seqReset, &g_wireSeqResets)
LOG_ADD(LOG_UINT32, trackRx, &g_trackRxOk)
LOG_ADD(LOG_UINT32, trackBad, &g_trackInvalidRx)
LOG_ADD(LOG_UINT32, trackDup, &g_trackDupRx)
LOG_ADD(LOG_UINT32, trackGap, &g_trackWireSeqGaps)
LOG_ADD(LOG_UINT16, trackSeq, &g_lastTrackWireSeq)
LOG_ADD(LOG_UINT8, trackN, &g_trackPayload.count)
LOG_ADD(LOG_UINT8, trackVer, &g_trackPayload.version)
LOG_ADD(LOG_UINT32, trackTick, &g_trackRxTick)
LOG_ADD(LOG_UINT32, gap8Ts, &g_payload.gap8_ts_us)
LOG_ADD(LOG_UINT32, ageMs, &g_sampleAgeMs)
LOG_ADD(LOG_UINT32, newCount, &g_newSamplesProcessed)
LOG_ADD(LOG_UINT32, mapVotes, &g_mapEvidenceVotes)
LOG_ADD(LOG_UINT8, wasNew, &g_lastSampleWasNew)
LOG_ADD(LOG_UINT8,  n,      &g_payload.n_sectors)
LOG_ADD(LOG_UINT8,  flags,  &g_payload.flags)
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
LOG_ADD(LOG_FLOAT,  sigma0, &g_payload.sector[0].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma1, &g_payload.sector[1].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma2, &g_payload.sector[2].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma3, &g_payload.sector[3].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma4, &g_payload.sector[4].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma5, &g_payload.sector[5].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma6, &g_payload.sector[6].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma7, &g_payload.sector[7].flow_sigma_rad_s)
LOG_ADD(LOG_FLOAT,  sigma8, &g_payload.sector[8].flow_sigma_rad_s)
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
LOG_ADD(LOG_UINT8,  syncOk,  &g_syncValid)
LOG_ADD(LOG_UINT32, syncErr, &g_syncErrorMs)
LOG_ADD(LOG_UINT32, syncMiss, &g_syncMiss)
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
LOG_ADD(LOG_FLOAT,  rangeSig0, &g_rangeSigma[0])
LOG_ADD(LOG_FLOAT,  rangeSig1, &g_rangeSigma[1])
LOG_ADD(LOG_FLOAT,  rangeSig2, &g_rangeSigma[2])
LOG_ADD(LOG_FLOAT,  rangeSig3, &g_rangeSigma[3])
LOG_ADD(LOG_FLOAT,  rangeSig4, &g_rangeSigma[4])
LOG_ADD(LOG_FLOAT,  rangeSig5, &g_rangeSigma[5])
LOG_ADD(LOG_FLOAT,  rangeSig6, &g_rangeSigma[6])
LOG_ADD(LOG_FLOAT,  rangeSig7, &g_rangeSigma[7])
LOG_ADD(LOG_FLOAT,  rangeSig8, &g_rangeSigma[8])
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
LOG_ADD(LOG_UINT8,  reject,    &g_rejectReason)
LOG_ADD(LOG_UINT32, rejMotion, &g_rejectLowMotion)
LOG_ADD(LOG_UINT32, rejYaw,    &g_rejectYaw)
LOG_ADD(LOG_UINT32, rejDepth,  &g_rejectDepthDisagree)
LOG_ADD(LOG_UINT32, rejDisp,   &g_rejectDispersion)
LOG_ADD(LOG_UINT32, rejGroup,  &g_rejectNoGroup)
LOG_ADD(LOG_FLOAT,  aggDisp,   &g_aggregateDisplacement)
LOG_ADD(LOG_FLOAT,  yawRatio,  &g_yawExplainedRatio)
LOG_ADD(LOG_FLOAT,  depDisagr, &g_depthDisagreement)
LOG_ADD(LOG_FLOAT,  grpDisp,   &g_groupDispersion)
LOG_ADD(LOG_FLOAT,  grpScore,  &g_groupScore)
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
LOG_ADD(LOG_FLOAT,  mapPeak,   &g_mapPeak)
LOG_ADD(LOG_UINT8,  mapActive, &g_mapActive)
LOG_ADD(LOG_UINT8,  mapBest,   &g_mapBestIdx)
LOG_ADD(LOG_UINT32, resetCnt,  &g_resetCount)
LOG_GROUP_STOP(flowObsRx)

PARAM_GROUP_START(flowObsCtl)
PARAM_ADD(PARAM_UINT8, reset, &g_resetRequested)
PARAM_GROUP_STOP(flowObsCtl)

PARAM_GROUP_START(flowCal)
PARAM_ADD(PARAM_FLOAT, camYaw, &g_cameraYawRad)
PARAM_ADD(PARAM_FLOAT, camFwd, &g_cameraForwardM)
PARAM_ADD(PARAM_FLOAT, camLeft, &g_cameraLeftM)
PARAM_GROUP_STOP(flowCal)
