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
extern uint32_t g_mpc_solve_us;
extern uint8_t g_mpc_iter;
extern int8_t g_mpc_status;
extern float g_mpc_primal_residual;
extern float g_mpc_dual_residual;
extern uint8_t g_mpc_health_hold;
extern uint32_t g_mpc_health_faults;
extern uint32_t g_mpc_solve_started;
extern uint32_t g_mpc_solve_completed;
extern uint32_t g_mpc_heartbeat_age_ms;
extern uint8_t g_mpc_stall_hold;
extern uint32_t g_mpc_warm_resets;
extern uint8_t g_mpc_last_reset_reason;
extern uint8_t g_mpc_solve_route_phase;
extern uint32_t g_mpc_stack_free_words;
extern uint32_t g_mpc_scan_bypass_cycles;
extern uint32_t g_mpc_pid_bypass_cycles;

PARAM_GROUP_START(obs)
PARAM_ADD(PARAM_UINT8,  enable,  &obsEnable)
PARAM_ADD(PARAM_UINT8,  logOnly, &obsLogOnly)
PARAM_ADD(PARAM_UINT8,  pidPass, &obsPidPassthrough)
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
LOG_ADD(LOG_UINT32, mpcUs,  &g_mpc_solve_us)
LOG_ADD(LOG_UINT8,  iter,   &g_mpc_iter)
LOG_ADD(LOG_INT8,   status, &g_mpc_status)
LOG_ADD(LOG_FLOAT,  priRes, &g_mpc_primal_residual)
LOG_ADD(LOG_FLOAT,  duaRes, &g_mpc_dual_residual)
LOG_ADD(LOG_UINT8,  healthHold, &g_mpc_health_hold)
LOG_ADD(LOG_UINT32, healthFaults, &g_mpc_health_faults)
LOG_ADD(LOG_UINT32, scanBypass, &g_mpc_scan_bypass_cycles)
LOG_ADD(LOG_UINT32, pidBypass, &g_mpc_pid_bypass_cycles)
LOG_ADD(LOG_UINT32, solveStart, &g_mpc_solve_started)
LOG_ADD(LOG_UINT32, solveDone, &g_mpc_solve_completed)
LOG_ADD(LOG_UINT32, hbAge, &g_mpc_heartbeat_age_ms)
LOG_ADD(LOG_UINT8, stallHold, &g_mpc_stall_hold)
LOG_ADD(LOG_UINT32, warmResets, &g_mpc_warm_resets)
LOG_ADD(LOG_UINT8, resetWhy, &g_mpc_last_reset_reason)
LOG_ADD(LOG_UINT8, solvePhase, &g_mpc_solve_route_phase)
LOG_ADD(LOG_UINT32, stackFree, &g_mpc_stack_free_words)
LOG_GROUP_STOP(obs)
