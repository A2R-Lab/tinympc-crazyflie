/* Shared, controller-independent TinyRacer observations and intents. */
#ifndef __TINYRACER_INTERFACE_H__
#define __TINYRACER_INTERFACE_H__

#include <stdbool.h>
#include <stdint.h>

#define TINYRACER_CLEARANCE_SECTORS 4
#define TINYRACER_GATE_CORNERS 4

typedef struct {
  bool valid;
  uint32_t source_timestamp;
  uint32_t received_age_ms;
  uint32_t sample;
  uint16_t sequence;
  bool has_metric_clearance;
  bool has_sector_danger;
  bool has_navigation_command;
  bool gate_valid;
  float clearance_m[TINYRACER_CLEARANCE_SECTORS];
  float confidence[TINYRACER_CLEARANCE_SECTORS];
  float danger_probability[TINYRACER_CLEARANCE_SECTORS];
  /* DroNet-style high-level outputs: left-positive steering in [-1, 1]. */
  float steering_command;
  float collision_probability;
  /* Normalized image coordinates, ordered TL, TR, BR, BL. */
  float gate_corners_xy[TINYRACER_GATE_CORNERS * 2];
  float gate_confidence;
  float gate_fx_normalized;
  float gate_fy_normalized;
  float gate_cx_normalized;
  float gate_cy_normalized;
} TinyRacerPerceptionObservation;

typedef enum {
  TINYRACER_RACE_TRACK = 0,
  TINYRACER_RACE_BLOCKED = 1,
  TINYRACER_RACE_RECOVER = 2,
} TinyRacerRaceMode;

typedef struct {
  bool active;
  bool changed;
  float body_bearing_rad;
  float boundary_distance_m;
} TinyRacerSectorConstraint;

typedef struct {
  TinyRacerRaceMode mode;
  bool perception_fresh;
  bool gate_valid;
  bool constraint_active;
  bool constraint_changed;
  int8_t pass_side;
  float lateral_offset_m;
  float stop_boundary_distance_m;
  TinyRacerSectorConstraint sector[TINYRACER_CLEARANCE_SECTORS];
} TinyRacerRaceIntent;

#endif
