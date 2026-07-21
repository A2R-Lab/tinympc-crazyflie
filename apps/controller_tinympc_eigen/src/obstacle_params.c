/*
 * obstacle_params.c
 * PARAM/LOG plumbing for AI-deck flow-sector obstacle constraints.
 */
#include "param.h"
#include "log.h"

#include <stdint.h>

extern uint8_t obsEnable;
extern uint8_t obsLogOnly;
extern float obsMinConf;
extern float obsMinDepth;
extern float obsMaxDepth;
extern uint32_t obsMaxAgeMs;
extern uint32_t obsHoldMs;
extern float obsSwitchRatio;
extern float obsMarginMin;
extern float obsMarginSlack;
extern uint8_t obsKStart;

extern uint8_t g_obs_active;
extern uint8_t g_obs_applied;
extern uint8_t g_obs_sector;
extern uint32_t g_obs_age_ms;
extern uint32_t g_obs_sample;
extern float g_obs_depth_m;
extern float g_obs_conf;
extern float g_obs_azimuth;
extern float g_obs_a0;
extern float g_obs_a1;
extern float g_obs_a2;
extern float g_obs_b;
extern float g_obs_margin;
extern float g_obs_violation;

PARAM_GROUP_START(obs)
PARAM_ADD(PARAM_UINT8,  enable,  &obsEnable)
PARAM_ADD(PARAM_UINT8,  logOnly, &obsLogOnly)
PARAM_ADD(PARAM_FLOAT,  minConf, &obsMinConf)
PARAM_ADD(PARAM_FLOAT,  minD,    &obsMinDepth)
PARAM_ADD(PARAM_FLOAT,  maxD,    &obsMaxDepth)
PARAM_ADD(PARAM_UINT32, maxAge,  &obsMaxAgeMs)
PARAM_ADD(PARAM_UINT32, holdMs,  &obsHoldMs)
PARAM_ADD(PARAM_FLOAT,  swRatio, &obsSwitchRatio)
PARAM_ADD(PARAM_FLOAT,  margin,  &obsMarginMin)
PARAM_ADD(PARAM_FLOAT,  mSlack,  &obsMarginSlack)
PARAM_ADD(PARAM_UINT8,  kStart,  &obsKStart)
PARAM_GROUP_STOP(obs)

LOG_GROUP_START(obs)
LOG_ADD(LOG_UINT8,  active, &g_obs_active)
LOG_ADD(LOG_UINT8,  apply,  &g_obs_applied)
LOG_ADD(LOG_UINT8,  sector, &g_obs_sector)
LOG_ADD(LOG_UINT32, age,    &g_obs_age_ms)
LOG_ADD(LOG_UINT32, sample, &g_obs_sample)
LOG_ADD(LOG_FLOAT,  depth,  &g_obs_depth_m)
LOG_ADD(LOG_FLOAT,  conf,   &g_obs_conf)
LOG_ADD(LOG_FLOAT,  az,     &g_obs_azimuth)
LOG_ADD(LOG_FLOAT,  a0,     &g_obs_a0)
LOG_ADD(LOG_FLOAT,  a1,     &g_obs_a1)
LOG_ADD(LOG_FLOAT,  a2,     &g_obs_a2)
LOG_ADD(LOG_FLOAT,  b,      &g_obs_b)
LOG_ADD(LOG_FLOAT,  margin, &g_obs_margin)
LOG_ADD(LOG_FLOAT,  viol,   &g_obs_violation)
LOG_GROUP_STOP(obs)
