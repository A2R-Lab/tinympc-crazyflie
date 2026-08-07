#ifndef __TINYRACER_RACING_H__
#define __TINYRACER_RACING_H__

#include "tinyracer_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool obstacle_constraint_active;
  bool has_sample;
  uint32_t last_sample;
  int8_t pass_side;
  float lateral_offset_m;
  uint8_t clear_samples;
  bool sector_active[TINYRACER_CLEARANCE_SECTORS];
} TinyRacerRaceState;

typedef struct {
  float clearance_threshold_m;
  float confidence_threshold;
  float safety_margin_m;
  float minimum_boundary_distance_m;
  float bypass_offset_m;
  float bypass_rate_mps;
  float recovery_rate_mps;
  uint32_t maximum_age_ms;
  float release_clearance_m;
  float maximum_bypass_offset_m;
  uint8_t clear_samples_required;
} TinyRacerRaceConfig;

void tinyRacerRaceReset(TinyRacerRaceState *state);
void tinyRacerRaceUpdate(
    TinyRacerRaceState *state,
    const TinyRacerPerceptionObservation *observation,
    const TinyRacerRaceConfig *config,
    float dt_s,
    bool release_ready,
    bool hold_offset,
    TinyRacerRaceIntent *intent);

#ifdef __cplusplus
}
#endif
#endif
