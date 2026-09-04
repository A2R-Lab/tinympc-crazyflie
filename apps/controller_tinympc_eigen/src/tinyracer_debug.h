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
  float square_opening_probability;
  uint32_t solve_us;
  uint8_t mode, cylinder_active;
  uint8_t square_opening_eligible, square_opening_seen;
  int8_t pass_side;
} TinyRacerDebugTelemetry;

extern TinyRacerDebugTelemetry tinyRacerDebug;

#endif
