/* Runtime controls and diagnostics for the four-direction visual corridor. */
#include "log.h"
#include "param.h"

#include <stdint.h>

extern uint8_t seqAvoidEnable;
extern uint8_t seqAvoidConstraintEnable;
extern uint8_t seqAvoidLogOnly;
extern float seqAvoidConfidenceMin;
extern uint32_t seqAvoidMaxAgeMs;
extern float seqAvoidDroneRadiusM;
extern float seqAvoidTrackingMarginM;
extern float seqAvoidLatencyS;
extern float seqAvoidPerceptionMarginM;
extern float seqAvoidConfidenceGainM;
extern float seqAvoidDefaultOffsetM;
extern float seqAvoidMaxRangeM;
extern float seqAvoidTriggerM;
extern float seqAvoidActivationEpsM;
extern float seqAvoidReferenceShiftM;
extern float seqAvoidSlackPenalty;
extern float seqAvoidDistanceWeight;
extern float seqAvoidGoalWeight;
extern float seqAvoidHysteresisWeight;
extern float seqAvoidDynamicWeight;
extern uint8_t seqAvoidKStart;

extern uint8_t g_seq_valid;
extern uint8_t g_seq_stop;
extern uint8_t g_seq_reliable_mask;
extern int8_t g_seq_chosen;
extern uint8_t g_seq_constraints;
extern uint32_t g_seq_age_ms;
extern uint32_t g_seq_sample;
extern float g_seq_pressure;
extern float g_seq_score;
extern float g_seq_max_slack;
extern float g_seq_effective[4];
extern float g_seq_margin[4];

PARAM_GROUP_START(seqAvoid)
PARAM_ADD(PARAM_UINT8, enable, &seqAvoidEnable)
PARAM_ADD(PARAM_UINT8, constrain, &seqAvoidConstraintEnable)
PARAM_ADD(PARAM_UINT8, logOnly, &seqAvoidLogOnly)
PARAM_ADD(PARAM_FLOAT, confMin, &seqAvoidConfidenceMin)
PARAM_ADD(PARAM_UINT32, maxAge, &seqAvoidMaxAgeMs)
PARAM_ADD(PARAM_FLOAT, droneRad, &seqAvoidDroneRadiusM)
PARAM_ADD(PARAM_FLOAT, trackMar, &seqAvoidTrackingMarginM)
PARAM_ADD(PARAM_FLOAT, latency, &seqAvoidLatencyS)
PARAM_ADD(PARAM_FLOAT, percMar, &seqAvoidPerceptionMarginM)
PARAM_ADD(PARAM_FLOAT, confGain, &seqAvoidConfidenceGainM)
PARAM_ADD(PARAM_FLOAT, defOff, &seqAvoidDefaultOffsetM)
PARAM_ADD(PARAM_FLOAT, maxRange, &seqAvoidMaxRangeM)
PARAM_ADD(PARAM_FLOAT, trigger, &seqAvoidTriggerM)
PARAM_ADD(PARAM_FLOAT, actEps, &seqAvoidActivationEpsM)
PARAM_ADD(PARAM_FLOAT, refShift, &seqAvoidReferenceShiftM)
PARAM_ADD(PARAM_FLOAT, slackPen, &seqAvoidSlackPenalty)
PARAM_ADD(PARAM_FLOAT, wDist, &seqAvoidDistanceWeight)
PARAM_ADD(PARAM_FLOAT, wGoal, &seqAvoidGoalWeight)
PARAM_ADD(PARAM_FLOAT, wHist, &seqAvoidHysteresisWeight)
PARAM_ADD(PARAM_FLOAT, wDyn, &seqAvoidDynamicWeight)
PARAM_ADD(PARAM_UINT8, kStart, &seqAvoidKStart)
PARAM_GROUP_STOP(seqAvoid)

LOG_GROUP_START(seqAvoid)
LOG_ADD(LOG_UINT8, valid, &g_seq_valid)
LOG_ADD(LOG_UINT8, stop, &g_seq_stop)
LOG_ADD(LOG_UINT8, mask, &g_seq_reliable_mask)
LOG_ADD(LOG_INT8, chosen, &g_seq_chosen)
LOG_ADD(LOG_UINT8, nCstr, &g_seq_constraints)
LOG_ADD(LOG_UINT32, ageMs, &g_seq_age_ms)
LOG_ADD(LOG_UINT32, sample, &g_seq_sample)
LOG_ADD(LOG_FLOAT, pressure, &g_seq_pressure)
LOG_ADD(LOG_FLOAT, score, &g_seq_score)
LOG_ADD(LOG_FLOAT, maxSlack, &g_seq_max_slack)
LOG_ADD(LOG_FLOAT, eff0, &g_seq_effective[0])
LOG_ADD(LOG_FLOAT, eff1, &g_seq_effective[1])
LOG_ADD(LOG_FLOAT, eff2, &g_seq_effective[2])
LOG_ADD(LOG_FLOAT, eff3, &g_seq_effective[3])
LOG_ADD(LOG_FLOAT, mar0, &g_seq_margin[0])
LOG_ADD(LOG_FLOAT, mar1, &g_seq_margin[1])
LOG_ADD(LOG_FLOAT, mar2, &g_seq_margin[2])
LOG_ADD(LOG_FLOAT, mar3, &g_seq_margin[3])
LOG_GROUP_STOP(seqAvoid)
