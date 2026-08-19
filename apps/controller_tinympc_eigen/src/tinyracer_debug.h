#ifndef __TINYRACER_DEBUG_H__
#define __TINYRACER_DEBUG_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float ref_x, ref_y, ref_z;
  float horizon_x, horizon_y, horizon_z;
  float cylinder_x, cylinder_y, cylinder_radius;
  float plane_nx, plane_ny, plane_boundary;
  float lateral_offset, plane_violation, consensus_error, slack;
  uint32_t solve_us;
  uint8_t mode, cylinder_active;
  int8_t pass_side;
} TinyRacerDebugTelemetry;

extern TinyRacerDebugTelemetry tinyRacerDebug;

#endif
