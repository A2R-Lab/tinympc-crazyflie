/*
 * yaw_cmd_params.c
 * PARAM plumbing to command yaw live from cfclient in the cascade controller.
 * (PARAM macros don't compile in the C++ controller TU.) Storage lives in
 * controller_tinympc.cpp (extern "C" block -> C linkage); this file exposes it.
 *
 * visYaw group:
 *   useRef  1 = the stock-PID yaw setpoint comes from yawRef below (default 0 = the
 *              normal source: face-forward trajectory heading, or the commander yaw)
 *   yawRef  commanded absolute heading [deg]
 */
#include "param.h"
#include <stdint.h>

extern uint8_t yawUseRef;
extern float   yawRefDeg;

PARAM_GROUP_START(visYaw)
PARAM_ADD(PARAM_UINT8, useRef, &yawUseRef)
PARAM_ADD(PARAM_FLOAT, yawRef, &yawRefDeg)
PARAM_GROUP_STOP(visYaw)
