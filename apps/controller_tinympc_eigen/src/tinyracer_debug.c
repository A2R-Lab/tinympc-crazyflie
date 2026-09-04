#include "tinyracer_debug.h"
#include "log.h"

TinyRacerDebugTelemetry tinyRacerDebug;

LOG_GROUP_START(trRef)
LOG_ADD(LOG_FLOAT, x, &tinyRacerDebug.ref_x)
LOG_ADD(LOG_FLOAT, y, &tinyRacerDebug.ref_y)
LOG_ADD(LOG_FLOAT, z, &tinyRacerDebug.ref_z)
LOG_ADD(LOG_FLOAT, hx, &tinyRacerDebug.horizon_x)
LOG_ADD(LOG_FLOAT, hy, &tinyRacerDebug.horizon_y)
LOG_ADD(LOG_FLOAT, hz, &tinyRacerDebug.horizon_z)
LOG_GROUP_STOP(trRef)

LOG_GROUP_START(trCyl)
LOG_ADD(LOG_FLOAT, x, &tinyRacerDebug.cylinder_x)
LOG_ADD(LOG_FLOAT, y, &tinyRacerDebug.cylinder_y)
LOG_ADD(LOG_FLOAT, r, &tinyRacerDebug.cylinder_radius)
LOG_ADD(LOG_FLOAT, nx, &tinyRacerDebug.plane_nx)
LOG_ADD(LOG_FLOAT, ny, &tinyRacerDebug.plane_ny)
LOG_ADD(LOG_FLOAT, b, &tinyRacerDebug.plane_boundary)
LOG_GROUP_STOP(trCyl)

LOG_GROUP_START(trCtl)
LOG_ADD(LOG_FLOAT, lateral, &tinyRacerDebug.lateral_offset)
LOG_ADD(LOG_FLOAT, violation, &tinyRacerDebug.plane_violation)
LOG_ADD(LOG_FLOAT, consensus, &tinyRacerDebug.consensus_error)
LOG_ADD(LOG_FLOAT, slack, &tinyRacerDebug.slack)
LOG_ADD(LOG_UINT32, solveUs, &tinyRacerDebug.solve_us)
LOG_ADD(LOG_UINT8, mode, &tinyRacerDebug.mode)
LOG_ADD(LOG_UINT8, cyl, &tinyRacerDebug.cylinder_active)
LOG_ADD(LOG_INT8, side, &tinyRacerDebug.pass_side)
LOG_GROUP_STOP(trCtl)

LOG_GROUP_START(trOpen)
LOG_ADD(LOG_FLOAT, probability, &tinyRacerDebug.square_opening_probability)
LOG_ADD(LOG_UINT8, eligible, &tinyRacerDebug.square_opening_eligible)
LOG_ADD(LOG_UINT8, seen, &tinyRacerDebug.square_opening_seen)
LOG_GROUP_STOP(trOpen)
