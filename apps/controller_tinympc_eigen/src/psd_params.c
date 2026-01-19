// PSD parameters and log group definitions (C file for C linkage)

#include <stdint.h>
#include "param.h"
#include "log.h"

uint8_t en_psd_avoidance = 0;
uint8_t psd_max_peers = 3;
float psd_peer_radius = 0.3f;
float psd_peer_margin = 0.15f;
int32_t psd_peer_max_age_ms = 500;
float psd_rho = 5.0f;

uint32_t psd_replan_stride = 5;
uint32_t psd_horizon_guard = 5;
float psd_base_on = 2.5f;
float psd_base_off = 2.5f;
float psd_goal_on_bias = 0.3f;
float psd_goal_off_bias = 0.8f;
float psd_off_hysteresis = 0.3f;

uint8_t psd_log_plan_mode = 0;
uint8_t psd_log_plan_status = 0;
int32_t psd_log_plan_age = 0;
int32_t psd_log_disk_count = 0;
uint8_t psd_log_certified = 0;
float psd_log_trace_gap = 0.0f;
float psd_log_eta_min = 0.0f;

PARAM_GROUP_START(psdAvoid)
  /**
   * @brief Enable PSD obstacle avoidance
   */
  PARAM_ADD(PARAM_UINT8, enable, &en_psd_avoidance)
  /**
   * @brief Max peers to consider (<= PEER_LOCALIZATION_MAX_NEIGHBORS)
   */
  PARAM_ADD(PARAM_UINT8, maxPeers, &psd_max_peers)
  /**
   * @brief Safety radius around each peer [m]
   */
  PARAM_ADD(PARAM_FLOAT, radius, &psd_peer_radius)
  /**
   * @brief Additional safety margin [m]
   */
  PARAM_ADD(PARAM_FLOAT, margin, &psd_peer_margin)
  /**
   * @brief Max peer localization age [ms], <0 disables filtering
   */
  PARAM_ADD(PARAM_INT32, maxAgeMs, &psd_peer_max_age_ms)
  /**
   * @brief PSD penalty rho
   */
  PARAM_ADD(PARAM_FLOAT, rho, &psd_rho)
  /**
   * @brief Replan stride (steps)
   */
  PARAM_ADD(PARAM_UINT32, replanStride, &psd_replan_stride)
  /**
   * @brief Horizon guard (steps)
   */
  PARAM_ADD(PARAM_UINT32, horizonGuard, &psd_horizon_guard)
  /**
   * @brief Base threshold to turn PSD on
   */
  PARAM_ADD(PARAM_FLOAT, baseOn, &psd_base_on)
  /**
   * @brief Base threshold to turn PSD off
   */
  PARAM_ADD(PARAM_FLOAT, baseOff, &psd_base_off)
  /**
   * @brief Goal proximity bias for on threshold
   */
  PARAM_ADD(PARAM_FLOAT, goalOnBias, &psd_goal_on_bias)
  /**
   * @brief Goal proximity bias for off threshold
   */
  PARAM_ADD(PARAM_FLOAT, goalOffBias, &psd_goal_off_bias)
  /**
   * @brief Hysteresis offset for off threshold
   */
  PARAM_ADD(PARAM_FLOAT, offHyst, &psd_off_hysteresis)
PARAM_GROUP_STOP(psdAvoid)

LOG_GROUP_START(psdMPC)
  LOG_ADD(LOG_UINT8, planMode, &psd_log_plan_mode)
  LOG_ADD(LOG_UINT8, planStatus, &psd_log_plan_status)
  LOG_ADD(LOG_INT32, planAge, &psd_log_plan_age)
  LOG_ADD(LOG_INT32, diskCount, &psd_log_disk_count)
  LOG_ADD(LOG_UINT8, certified, &psd_log_certified)
  LOG_ADD(LOG_FLOAT, traceGap, &psd_log_trace_gap)
  LOG_ADD(LOG_FLOAT, etaMin, &psd_log_eta_min)
LOG_GROUP_STOP(psdMPC)
