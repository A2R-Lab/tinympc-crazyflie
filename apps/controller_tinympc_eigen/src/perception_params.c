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
extern uint8_t perceptDangerQ;
extern uint8_t perceptKStart;
extern uint8_t perceptMassMinCells;
extern float perceptLineOffsetPx;
extern uint8_t perceptHoldSteps;
extern uint8_t perceptPersistMaps;
extern uint8_t perceptSafeBandCells;
extern uint8_t perceptConstraintSteps;
extern uint8_t perceptClearMaps;
extern uint8_t perceptRejoinSteps;
extern float perceptMaxLateralSpeedMps;
extern uint8_t g_percept_valid;
extern uint32_t g_percept_age_ms;
extern uint32_t g_percept_sample;
extern float g_percept_max_danger;
extern float g_percept_center_danger;
extern float g_percept_min_ttc;
extern uint8_t g_percept_corridor_valid;
extern uint8_t g_percept_constraints;
extern uint8_t g_percept_applied;
extern uint8_t g_percept_hold_active;
extern uint32_t g_percept_hold_age_ms;
extern uint8_t g_percept_mass_cells;
extern uint8_t g_percept_mass_detected;
extern uint8_t g_percept_center_q;
extern uint8_t g_percept_max_q;
extern uint8_t g_percept_obstacle_cells;
extern uint8_t g_percept_path_hit;
extern uint8_t g_percept_path_knot;
extern uint8_t g_percept_path_q;
extern uint8_t g_percept_safe_left_cells;
extern uint8_t g_percept_safe_right_cells;
extern float g_percept_path_u;
extern float g_percept_path_v;
extern float g_percept_plan_violation;
extern float g_percept_state_violation;
extern uint8_t g_percept_mode;
extern uint8_t g_percept_persist_count;
extern uint8_t g_percept_clear_count;
extern float g_percept_avoid_position[3];
extern float g_percept_reference_offset;
extern int8_t g_percept_avoid_side;
extern uint16_t g_percept_unsafe_rows[10];
extern float g_percept_plane_a[3];
extern float g_percept_plane_b;
extern float g_percept_safe_u;
extern float g_percept_line_u;
extern float g_percept_left_n[3];
extern float g_percept_right_n[3];
extern float g_mpc_output_position[3];
extern float g_mpc_output_yaw_deg;
extern uint8_t g_mpc_output_safe;
extern float g_mpc_output_violation;

PARAM_GROUP_START(percept)
PARAM_ADD(PARAM_UINT8, enable, &perceptEnable)
PARAM_ADD(PARAM_UINT8, constrain, &perceptConstraintEnable)
PARAM_ADD(PARAM_UINT8, logOnly, &perceptLogOnly)
PARAM_ADD(PARAM_FLOAT, horizon, &perceptHorizonS)
PARAM_ADD(PARAM_FLOAT, latency, &perceptLatencyS)
PARAM_ADD(PARAM_FLOAT, maxRange, &perceptMaxRangeM)
PARAM_ADD(PARAM_FLOAT, nomSpeed, &perceptNominalSpeedMps)
PARAM_ADD(PARAM_UINT8, dangerQ, &perceptDangerQ)
PARAM_ADD(PARAM_UINT8, kStart, &perceptKStart)
PARAM_ADD(PARAM_UINT8, massCells, &perceptMassMinCells)
PARAM_ADD(PARAM_FLOAT, lineOffset, &perceptLineOffsetPx)
PARAM_ADD(PARAM_UINT8, holdSteps, &perceptHoldSteps)
PARAM_ADD(PARAM_UINT8, persist, &perceptPersistMaps)
PARAM_ADD(PARAM_UINT8, safeBand, &perceptSafeBandCells)
PARAM_ADD(PARAM_UINT8, cstrSteps, &perceptConstraintSteps)
PARAM_ADD(PARAM_UINT8, clearMaps, &perceptClearMaps)
PARAM_ADD(PARAM_UINT8, rejoin, &perceptRejoinSteps)
PARAM_ADD(PARAM_FLOAT, maxLatVel, &perceptMaxLateralSpeedMps)
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
LOG_ADD(LOG_UINT8, applied, &g_percept_applied)
LOG_ADD(LOG_INT8, side, &g_percept_avoid_side)
LOG_ADD(LOG_UINT8, hold, &g_percept_hold_active)
LOG_ADD(LOG_UINT32, holdAge, &g_percept_hold_age_ms)
LOG_ADD(LOG_UINT8, mode, &g_percept_mode)
LOG_ADD(LOG_UINT8, persist, &g_percept_persist_count)
LOG_ADD(LOG_UINT8, clearCnt, &g_percept_clear_count)
LOG_ADD(LOG_FLOAT, leftNx, &g_percept_left_n[0])
LOG_ADD(LOG_FLOAT, leftNy, &g_percept_left_n[1])
LOG_ADD(LOG_FLOAT, rightNx, &g_percept_right_n[0])
LOG_ADD(LOG_FLOAT, rightNy, &g_percept_right_n[1])
LOG_GROUP_STOP(percept)

LOG_GROUP_START(perceptPlane)
LOG_ADD(LOG_FLOAT, ax, &g_percept_plane_a[0])
LOG_ADD(LOG_FLOAT, ay, &g_percept_plane_a[1])
LOG_ADD(LOG_FLOAT, az, &g_percept_plane_a[2])
LOG_ADD(LOG_FLOAT, b, &g_percept_plane_b)
LOG_ADD(LOG_FLOAT, safeU, &g_percept_safe_u)
LOG_ADD(LOG_FLOAT, lineU, &g_percept_line_u)
LOG_ADD(LOG_FLOAT, avoidX, &g_percept_avoid_position[0])
LOG_ADD(LOG_FLOAT, avoidY, &g_percept_avoid_position[1])
LOG_ADD(LOG_FLOAT, avoidZ, &g_percept_avoid_position[2])
LOG_ADD(LOG_FLOAT, refOff, &g_percept_reference_offset)
LOG_GROUP_STOP(perceptPlane)

LOG_GROUP_START(perceptGridA)
LOG_ADD(LOG_UINT16, r0, &g_percept_unsafe_rows[0])
LOG_ADD(LOG_UINT16, r1, &g_percept_unsafe_rows[1])
LOG_ADD(LOG_UINT16, r2, &g_percept_unsafe_rows[2])
LOG_ADD(LOG_UINT16, r3, &g_percept_unsafe_rows[3])
LOG_ADD(LOG_UINT16, r4, &g_percept_unsafe_rows[4])
LOG_GROUP_STOP(perceptGridA)

LOG_GROUP_START(perceptGridB)
LOG_ADD(LOG_UINT16, r5, &g_percept_unsafe_rows[5])
LOG_ADD(LOG_UINT16, r6, &g_percept_unsafe_rows[6])
LOG_ADD(LOG_UINT16, r7, &g_percept_unsafe_rows[7])
LOG_ADD(LOG_UINT16, r8, &g_percept_unsafe_rows[8])
LOG_ADD(LOG_UINT16, r9, &g_percept_unsafe_rows[9])
LOG_GROUP_STOP(perceptGridB)

LOG_GROUP_START(mpcDiag)
LOG_ADD(LOG_FLOAT, outX, &g_mpc_output_position[0])
LOG_ADD(LOG_FLOAT, outY, &g_mpc_output_position[1])
LOG_ADD(LOG_FLOAT, outZ, &g_mpc_output_position[2])
LOG_ADD(LOG_FLOAT, yawCmd, &g_mpc_output_yaw_deg)
LOG_ADD(LOG_UINT8, safe, &g_mpc_output_safe)
LOG_ADD(LOG_FLOAT, outVio, &g_mpc_output_violation)
LOG_GROUP_STOP(mpcDiag)

LOG_GROUP_START(perceptMass)
LOG_ADD(LOG_UINT8, cells, &g_percept_obstacle_cells)
LOG_ADD(LOG_UINT8, cluster, &g_percept_mass_cells)
LOG_ADD(LOG_UINT8, detected, &g_percept_mass_detected)
LOG_ADD(LOG_UINT8, centerQ, &g_percept_center_q)
LOG_ADD(LOG_UINT8, maxQ, &g_percept_max_q)
LOG_GROUP_STOP(perceptMass)

LOG_GROUP_START(perceptPath)
LOG_ADD(LOG_UINT8, hit, &g_percept_path_hit)
LOG_ADD(LOG_UINT8, knot, &g_percept_path_knot)
LOG_ADD(LOG_UINT8, q, &g_percept_path_q)
LOG_ADD(LOG_UINT8, safeL, &g_percept_safe_left_cells)
LOG_ADD(LOG_UINT8, safeR, &g_percept_safe_right_cells)
LOG_ADD(LOG_FLOAT, u, &g_percept_path_u)
LOG_ADD(LOG_FLOAT, v, &g_percept_path_v)
LOG_GROUP_STOP(perceptPath)

LOG_GROUP_START(perceptVio)
LOG_ADD(LOG_FLOAT, plan, &g_percept_plan_violation)
LOG_ADD(LOG_FLOAT, state, &g_percept_state_violation)
LOG_GROUP_STOP(perceptVio)
