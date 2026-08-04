#include "log.h"
#include "param.h"
#include <stdint.h>

extern uint8_t circEn;
extern float gAx, gAy, gAz, gBx, gBy, gBz, loopWidth, circSpeed;
extern int8_t circDir;
extern float g_circ_phase;
extern uint32_t g_circ_laps;

PARAM_GROUP_START(circuit)
PARAM_ADD(PARAM_UINT8, en, &circEn)
PARAM_ADD(PARAM_FLOAT, gAx, &gAx)
PARAM_ADD(PARAM_FLOAT, gAy, &gAy)
PARAM_ADD(PARAM_FLOAT, gAz, &gAz)
PARAM_ADD(PARAM_FLOAT, gBx, &gBx)
PARAM_ADD(PARAM_FLOAT, gBy, &gBy)
PARAM_ADD(PARAM_FLOAT, gBz, &gBz)
PARAM_ADD(PARAM_FLOAT, loopW, &loopWidth)
PARAM_ADD(PARAM_FLOAT, speed, &circSpeed)
PARAM_ADD(PARAM_INT8, dir, &circDir)
PARAM_GROUP_STOP(circuit)

LOG_GROUP_START(visGate)
LOG_ADD(LOG_FLOAT, phase, &g_circ_phase)
LOG_ADD(LOG_UINT32, laps, &g_circ_laps)
LOG_GROUP_STOP(visGate)
