/*
 * obstacle_params.c
 * PARAM/LOG plumbing for modeled-cylinder obstacle constraints.
 */
#include "param.h"
#include "log.h"

#include <stdint.h>

extern uint8_t obsEnable;
extern uint8_t obsLogOnly;
extern uint8_t obsPidPassthrough;
extern uint8_t obsUseFlow;
extern uint8_t obsFreezeFlow;
extern uint8_t obsFreezeClear;
extern uint32_t obsFreezeAfterMs;
extern float obsCx;
extern float obsCy;
extern float obsCz;
extern float obsRadius;
extern float obsHeight;
extern float obsSafety;
extern float obsActMargin;
extern float obsSide;
extern uint32_t obsDelayMs;
extern uint8_t obsKStart;

extern uint8_t g_obs_active;
extern uint8_t g_obs_applied;
extern uint8_t g_obs_count;
extern uint8_t g_obs_first_k;
extern float g_obs_a0;
extern float g_obs_a1;
extern float g_obs_a2;
extern float g_obs_b;
extern float g_obs_margin;
extern float g_obs_violation;
extern float g_obs_clearance;
extern uint8_t g_obs_source;
extern float g_obs_eff_cx;
extern float g_obs_eff_cy;
extern float g_obs_eff_radius;
extern float g_obs_flow_conf;
extern uint8_t g_obs_freeze_valid;
extern float g_obs_freeze_cx;
extern float g_obs_freeze_cy;
extern float g_obs_freeze_conf;
extern uint32_t g_mpc_solve_us;
extern uint8_t g_mpc_iter;

PARAM_GROUP_START(obs)
PARAM_ADD(PARAM_UINT8,  enable,  &obsEnable)
PARAM_ADD(PARAM_UINT8,  logOnly, &obsLogOnly)
PARAM_ADD(PARAM_UINT8,  pidPass, &obsPidPassthrough)
PARAM_ADD(PARAM_UINT8,  useFlow, &obsUseFlow)
PARAM_ADD(PARAM_UINT8,  freeze,  &obsFreezeFlow)
PARAM_ADD(PARAM_UINT8,  frzClear,&obsFreezeClear)
PARAM_ADD(PARAM_UINT32, frzAfter,&obsFreezeAfterMs)
PARAM_ADD(PARAM_FLOAT,  cx,      &obsCx)
PARAM_ADD(PARAM_FLOAT,  cy,      &obsCy)
PARAM_ADD(PARAM_FLOAT,  cz,      &obsCz)
PARAM_ADD(PARAM_FLOAT,  radius,  &obsRadius)
PARAM_ADD(PARAM_FLOAT,  height,  &obsHeight)
PARAM_ADD(PARAM_FLOAT,  safety,  &obsSafety)
PARAM_ADD(PARAM_FLOAT,  actMarg, &obsActMargin)
PARAM_ADD(PARAM_FLOAT,  side,    &obsSide)
PARAM_ADD(PARAM_UINT32, delayMs, &obsDelayMs)
PARAM_ADD(PARAM_UINT8,  kStart,  &obsKStart)
PARAM_GROUP_STOP(obs)

LOG_GROUP_START(obs)
LOG_ADD(LOG_UINT8,  active, &g_obs_active)
LOG_ADD(LOG_UINT8,  apply,  &g_obs_applied)
LOG_ADD(LOG_UINT8,  count,  &g_obs_count)
LOG_ADD(LOG_UINT8,  firstK, &g_obs_first_k)
LOG_ADD(LOG_FLOAT,  a0,     &g_obs_a0)
LOG_ADD(LOG_FLOAT,  a1,     &g_obs_a1)
LOG_ADD(LOG_FLOAT,  a2,     &g_obs_a2)
LOG_ADD(LOG_FLOAT,  b,      &g_obs_b)
LOG_ADD(LOG_FLOAT,  margin, &g_obs_margin)
LOG_ADD(LOG_FLOAT,  viol,   &g_obs_violation)
LOG_ADD(LOG_FLOAT,  clear,  &g_obs_clearance)
LOG_ADD(LOG_UINT8,  source, &g_obs_source)
LOG_ADD(LOG_FLOAT,  effCx,  &g_obs_eff_cx)
LOG_ADD(LOG_FLOAT,  effCy,  &g_obs_eff_cy)
LOG_ADD(LOG_FLOAT,  effRad, &g_obs_eff_radius)
LOG_ADD(LOG_FLOAT,  flowCf, &g_obs_flow_conf)
LOG_ADD(LOG_UINT8,  frzValid, &g_obs_freeze_valid)
LOG_ADD(LOG_FLOAT,  frzCx,  &g_obs_freeze_cx)
LOG_ADD(LOG_FLOAT,  frzCy,  &g_obs_freeze_cy)
LOG_ADD(LOG_FLOAT,  frzCf,  &g_obs_freeze_conf)
LOG_ADD(LOG_UINT32, mpcUs,  &g_mpc_solve_us)
LOG_ADD(LOG_UINT8,  iter,   &g_mpc_iter)
LOG_GROUP_STOP(obs)
