#include "log.h"
#include "param.h"
#include <stdint.h>

extern uint8_t obsPidPassthrough;
extern uint32_t g_mpc_solve_us, g_mpc_health_faults, g_mpc_solve_started, g_mpc_solve_completed;
extern uint32_t g_mpc_heartbeat_age_ms, g_mpc_warm_resets, g_mpc_stack_free_words;
extern uint32_t g_mpc_scan_bypass_cycles, g_mpc_pid_bypass_cycles, g_mpc_commander_age_ms;
extern uint8_t g_mpc_iter, g_mpc_health_hold, g_mpc_stall_hold, g_mpc_last_reset_reason;
extern uint8_t g_mpc_solve_route_phase, g_mpc_commander_disabled;
extern int8_t g_mpc_status;
extern float g_mpc_primal_residual, g_mpc_dual_residual;

PARAM_GROUP_START(obs)
PARAM_ADD(PARAM_UINT8, pidPass, &obsPidPassthrough)
PARAM_GROUP_STOP(obs)

LOG_GROUP_START(obs)
LOG_ADD(LOG_UINT32, mpcUs, &g_mpc_solve_us)
LOG_ADD(LOG_UINT8, iter, &g_mpc_iter)
LOG_ADD(LOG_INT8, status, &g_mpc_status)
LOG_ADD(LOG_FLOAT, priRes, &g_mpc_primal_residual)
LOG_ADD(LOG_FLOAT, duaRes, &g_mpc_dual_residual)
LOG_ADD(LOG_UINT8, healthHold, &g_mpc_health_hold)
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
LOG_ADD(LOG_UINT32, cmdAge, &g_mpc_commander_age_ms)
LOG_ADD(LOG_UINT8, cmdDisabled, &g_mpc_commander_disabled)
LOG_GROUP_STOP(obs)
