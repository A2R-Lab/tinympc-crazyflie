/*
 * Runtime interface for the vision-independent fixed half-space test.
 *
 * The controller normalizes (ax, ay, az, b) before installing
 *     ax*x + ay*y + az*z <= b
 * at each configured horizon knot.
 */
#include "log.h"
#include "param.h"

#include <stdint.h>

extern uint8_t testPlaneEnable;
extern uint8_t testPlaneLogOnly;
extern uint8_t testPlaneKStart;
extern uint8_t testPlaneMaxIter;
extern float testPlaneA[3];
extern float testPlaneB;

extern uint8_t g_test_plane_valid;
extern uint8_t g_test_plane_applied;
extern uint8_t g_test_plane_constraints;
extern uint8_t g_test_plane_worst_k;
extern float g_test_plane_a[3];
extern float g_test_plane_b;
extern float g_test_plane_plan_violation;
extern float g_test_plane_output_violation;
extern float g_test_plane_state_violation;

PARAM_GROUP_START(testPlane)
PARAM_ADD(PARAM_UINT8, enable, &testPlaneEnable)
PARAM_ADD(PARAM_UINT8, logOnly, &testPlaneLogOnly)
PARAM_ADD(PARAM_UINT8, kStart, &testPlaneKStart)
PARAM_ADD(PARAM_UINT8, maxIter, &testPlaneMaxIter)
PARAM_ADD(PARAM_FLOAT, ax, &testPlaneA[0])
PARAM_ADD(PARAM_FLOAT, ay, &testPlaneA[1])
PARAM_ADD(PARAM_FLOAT, az, &testPlaneA[2])
PARAM_ADD(PARAM_FLOAT, b, &testPlaneB)
PARAM_GROUP_STOP(testPlane)

LOG_GROUP_START(testPlane)
LOG_ADD(LOG_UINT8, valid, &g_test_plane_valid)
LOG_ADD(LOG_UINT8, applied, &g_test_plane_applied)
LOG_ADD(LOG_UINT8, nCstr, &g_test_plane_constraints)
LOG_ADD(LOG_UINT8, worstK, &g_test_plane_worst_k)
LOG_ADD(LOG_FLOAT, ax, &g_test_plane_a[0])
LOG_ADD(LOG_FLOAT, ay, &g_test_plane_a[1])
LOG_ADD(LOG_FLOAT, az, &g_test_plane_a[2])
LOG_ADD(LOG_FLOAT, b, &g_test_plane_b)
LOG_GROUP_STOP(testPlane)

LOG_GROUP_START(testPlaneVio)
LOG_ADD(LOG_FLOAT, plan, &g_test_plane_plan_violation)
LOG_ADD(LOG_FLOAT, output, &g_test_plane_output_violation)
LOG_ADD(LOG_FLOAT, state, &g_test_plane_state_violation)
LOG_GROUP_STOP(testPlaneVio)
