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
  uint8_t blocked_samples;
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
  float hard_clearance_threshold_m;
  uint8_t blocked_samples_required;
  float danger_probability_threshold;
  uint8_t danger_sectors_required;
} TinyRacerRaceConfig;

/* PULP-DroNet's published flight law is body-forward speed plus yaw rate.
 * Keep that model-independent policy here so the TinyMPC layer only has to
 * turn the resulting curved flight intent into a horizon reference. */
typedef struct {
  uint32_t maximum_age_ms;
  float maximum_forward_speed_mps;
  float yaw_rate_scale_rad_s;
  float previous_output_weight;
} TinyRacerNavigationConfig;

typedef struct {
  bool initialized;
  uint32_t last_sample;
  float forward_speed_mps;
  float yaw_rate_rad_s;
  float heading_world_rad;
} TinyRacerNavigationState;

typedef struct {
  bool active;
  bool new_sample;
  float forward_speed_mps;
  float yaw_rate_rad_s;
  float heading_world_rad;
  float collision_probability;
} TinyRacerNavigationIntent;

typedef enum {
  TINYRACER_DODGE_TRACK = 0,
  TINYRACER_DODGE_SIDESTEP = 1,
  TINYRACER_DODGE_PASS = 2,
  TINYRACER_DODGE_REJOIN = 3,
} TinyRacerDodgePhase;

typedef struct {
  float trigger_probability;
  float release_probability;
  float lateral_offset_m;
  float sidestep_rate_mps;
  float rejoin_rate_mps;
  float sidestep_forward_speed_mps;
  float pass_forward_speed_mps;
  float rejoin_forward_speed_mps;
  float minimum_pass_distance_m;
  float minimum_rearm_distance_m;
  bool allow_rejoin_redirect;
  /* -1/right, +1/left, or 0 to use the navigation head's steering sign. */
  int8_t preferred_pass_side;
  uint8_t trigger_samples_required;
  uint8_t clear_samples_required;
} TinyRacerDodgeConfig;

typedef struct {
  TinyRacerDodgePhase phase;
  int8_t pass_side;
  float lateral_offset_m;
  float forward_distance_m;
  float rearm_distance_m;
  uint8_t trigger_samples;
  uint8_t clear_samples;
  bool armed;
} TinyRacerDodgeState;

typedef struct {
  TinyRacerDodgePhase phase;
  int8_t pass_side;
  float lateral_offset_m;
  float lateral_rate_mps;
  float forward_speed_mps;
} TinyRacerDodgeIntent;

typedef struct {
  float center_x_m;
  float center_y_m;
  float center_z_m;
  float normal_x;
  float normal_y;
  float maximum_lateral_error_m;
  float maximum_vertical_error_m;
} TinyRacerGateDefinition;

typedef struct {
  uint8_t next_gate;
  bool plane_initialized;
  float previous_signed_distance_m;
} TinyRacerGateProgress;

void tinyRacerRaceReset(TinyRacerRaceState *state);
void tinyRacerRaceUpdate(
    TinyRacerRaceState *state,
    const TinyRacerPerceptionObservation *observation,
    const TinyRacerRaceConfig *config,
    float dt_s,
    bool release_ready,
    bool hold_offset,
    bool allow_activation,
    uint8_t relevant_sector_mask,
    TinyRacerRaceIntent *intent);
void tinyRacerNavigationReset(TinyRacerNavigationState *state);
void tinyRacerNavigationUpdate(
    TinyRacerNavigationState *state,
    const TinyRacerPerceptionObservation *observation,
    const TinyRacerNavigationConfig *config,
    float dt_s,
    float measured_heading_world_rad,
    TinyRacerNavigationIntent *intent);
void tinyRacerDodgeReset(TinyRacerDodgeState *state);
void tinyRacerDodgeUpdate(
    TinyRacerDodgeState *state,
    const TinyRacerNavigationIntent *navigation,
    const TinyRacerDodgeConfig *config,
    float dt_s,
    float measured_forward_speed_mps,
    TinyRacerDodgeIntent *intent);
bool tinyRacerGateServoAllowed(
    TinyRacerRaceMode race_mode,
    TinyRacerDodgePhase dodge_phase,
    bool halfspace_active,
    bool recovery_active);
void tinyRacerGateProgressReset(TinyRacerGateProgress *progress);
bool tinyRacerGateProgressUpdate(
    TinyRacerGateProgress *progress,
    const TinyRacerGateDefinition *gates,
    uint8_t gate_count,
    float position_x_m,
    float position_y_m,
    float position_z_m);

#ifdef __cplusplus
}
#endif
#endif
