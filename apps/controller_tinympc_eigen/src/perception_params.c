/* Runtime controls and diagnostics for NanoCockpit CNN danger maps. */
#include "log.h"
#include "param.h"

#include <stdint.h>

extern uint8_t perceptEnable;
extern uint8_t perceptConstraintEnable;
extern uint8_t perceptLogOnly;
extern float perceptHorizonS;
extern float perceptLatencyS;
extern float perceptMaxRangeM;
extern float perceptNominalSpeedMps;
extern float perceptDangerThreshold;
extern float perceptBaseMarginPx;
extern float perceptDroneRadiusM;
extern float perceptSafetyMarginM;
extern float perceptLookaheadS;
extern float perceptSlackPenalty;
extern uint8_t perceptKStart;
extern uint8_t perceptGateOpeningEnable;
extern float perceptGateOpeningThreshold;
extern float perceptGateOpeningInsetPx;
extern float perceptGateOpeningSafeCap;
extern float perceptGateOpeningRangeGuardM;
extern float perceptGateOpeningMaxUncertainty;
extern uint8_t g_percept_valid;
extern uint32_t g_percept_age_ms;
extern uint32_t g_percept_sample;
extern float g_percept_max_danger;
extern float g_percept_center_danger;
extern float g_percept_min_ttc;
extern uint8_t g_percept_corridor_valid;
extern uint8_t g_percept_constraints;
extern float g_percept_pixel_margin;
extern float g_percept_max_slack;
extern float g_percept_total_slack;
extern float g_percept_slack_cost;
extern uint32_t g_percept_failed_solves;
extern uint32_t g_percept_near_infeasible_solves;
extern float g_percept_left_n[3];
extern float g_percept_right_n[3];
extern uint8_t g_percept_gate_open_cells;

PARAM_GROUP_START(percept)
PARAM_ADD(PARAM_UINT8, enable, &perceptEnable)
PARAM_ADD(PARAM_UINT8, constrain, &perceptConstraintEnable)
PARAM_ADD(PARAM_UINT8, logOnly, &perceptLogOnly)
PARAM_ADD(PARAM_FLOAT, horizon, &perceptHorizonS)
PARAM_ADD(PARAM_FLOAT, latency, &perceptLatencyS)
PARAM_ADD(PARAM_FLOAT, maxRange, &perceptMaxRangeM)
PARAM_ADD(PARAM_FLOAT, nomSpeed, &perceptNominalSpeedMps)
PARAM_ADD(PARAM_FLOAT, dangerThr, &perceptDangerThreshold)
PARAM_ADD(PARAM_FLOAT, basePx, &perceptBaseMarginPx)
PARAM_ADD(PARAM_FLOAT, droneRad, &perceptDroneRadiusM)
PARAM_ADD(PARAM_FLOAT, safety, &perceptSafetyMarginM)
PARAM_ADD(PARAM_FLOAT, lookahead, &perceptLookaheadS)
PARAM_ADD(PARAM_FLOAT, slackPen, &perceptSlackPenalty)
PARAM_ADD(PARAM_UINT8, kStart, &perceptKStart)
PARAM_ADD(PARAM_UINT8, gateOpen, &perceptGateOpeningEnable)
PARAM_ADD(PARAM_FLOAT, gateThr, &perceptGateOpeningThreshold)
PARAM_ADD(PARAM_FLOAT, gateInset, &perceptGateOpeningInsetPx)
PARAM_ADD(PARAM_FLOAT, gateCap, &perceptGateOpeningSafeCap)
PARAM_ADD(PARAM_FLOAT, gateRange, &perceptGateOpeningRangeGuardM)
PARAM_ADD(PARAM_FLOAT, gateUnc, &perceptGateOpeningMaxUncertainty)
PARAM_GROUP_STOP(percept)

LOG_GROUP_START(percept)
LOG_ADD(LOG_UINT8, valid, &g_percept_valid)
LOG_ADD(LOG_UINT32, ageMs, &g_percept_age_ms)
LOG_ADD(LOG_UINT32, sample, &g_percept_sample)
LOG_ADD(LOG_FLOAT, maxDanger, &g_percept_max_danger)
LOG_ADD(LOG_FLOAT, ctrDanger, &g_percept_center_danger)
LOG_ADD(LOG_FLOAT, minTtc, &g_percept_min_ttc)
LOG_ADD(LOG_UINT8, corridor, &g_percept_corridor_valid)
LOG_ADD(LOG_UINT8, nCstr, &g_percept_constraints)
LOG_ADD(LOG_FLOAT, marginPx, &g_percept_pixel_margin)
LOG_ADD(LOG_FLOAT, maxSlack, &g_percept_max_slack)
LOG_ADD(LOG_FLOAT, sumSlack, &g_percept_total_slack)
LOG_ADD(LOG_FLOAT, slackCost, &g_percept_slack_cost)
LOG_ADD(LOG_UINT32, failSolve, &g_percept_failed_solves)
LOG_ADD(LOG_UINT32, nearInf, &g_percept_near_infeasible_solves)
LOG_ADD(LOG_FLOAT, leftNx, &g_percept_left_n[0])
LOG_ADD(LOG_FLOAT, leftNy, &g_percept_left_n[1])
LOG_ADD(LOG_FLOAT, rightNx, &g_percept_right_n[0])
LOG_ADD(LOG_FLOAT, rightNy, &g_percept_right_n[1])
LOG_ADD(LOG_UINT8, gateCells, &g_percept_gate_open_cells)
LOG_GROUP_STOP(percept)
