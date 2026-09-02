#ifndef __TINYRACER_RACING_H__
#define __TINYRACER_RACING_H__

#include <stddef.h>

#include "tinyracer_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES 5u

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
  float left_collision_probability;
  float center_collision_probability;
  float right_collision_probability;
} TinyRacerNavigationIntent;

/* Fuse LEFT/CENTER/RIGHT danger probabilities into the navigation fields.
 * The maximum risk governs speed/triggering. Above the supplied threshold,
 * steering points toward the lower-risk outer sector. If both sides
 * are within the ambiguity margin, a nonzero preferred side breaks the tie. */
void tinyRacerNavigationFuseSectorRisk(
    TinyRacerPerceptionObservation *observation, float risk_threshold,
    float ambiguity_margin, int8_t ambiguous_pass_side);

typedef enum {
  TINYRACER_DODGE_TRACK = 0,
  TINYRACER_DODGE_AVOID_LEFT = 1,
  TINYRACER_DODGE_AVOID_RIGHT = 2,
  TINYRACER_DODGE_EMERGENCY_BRAKE = 3,
  TINYRACER_DODGE_BACKTRACK = 4,
  TINYRACER_DODGE_HOLD_LEFT = 5,
  TINYRACER_DODGE_HOLD_RIGHT = 6,
  TINYRACER_DODGE_REJOIN_LEFT = 7,
  TINYRACER_DODGE_REJOIN_RIGHT = 8,
  TINYRACER_DODGE_REARM_LEFT = 9,
  TINYRACER_DODGE_REARM_RIGHT = 10,
  TINYRACER_DODGE_REDIRECT_PREP_LEFT = 11,
  TINYRACER_DODGE_REDIRECT_PREP_RIGHT = 12,
  TINYRACER_DODGE_BACKTRACK_SETTLE = 13,
  TINYRACER_DODGE_REJOIN_ALIGN_LEFT = 14,
  TINYRACER_DODGE_REJOIN_ALIGN_RIGHT = 15,
  TINYRACER_DODGE_LOOP_SCAN_LEFT = 16,
  TINYRACER_DODGE_LOOP_SCAN_RIGHT = 17,
  TINYRACER_DODGE_LOOP_ESCAPE_FORWARD = 18,
} TinyRacerDodgePhase;

typedef struct {
  float trigger_probability;
  float minimum_lateral_rate_mps;
  float maximum_lateral_rate_mps;
  float rejoin_lateral_rate_mps;
  float rejoin_spline_length_m;
  float lateral_acceleration_mps2;
  float maximum_lateral_offset_m;
  float minimum_lateral_offset_m;
  float emergency_brake_probability;
  float emergency_brake_deceleration_mps2;
  float backtrack_release_probability;
  float backtrack_speed_mps;
  float backtrack_settle_maximum_tilt_rad;
  float backtrack_settle_maximum_forward_speed_mps;
  float backtrack_settle_maximum_lateral_speed_mps;
  float backtrack_settle_maximum_vertical_speed_mps;
  float backtrack_settle_maximum_body_rate_rad_s;
  float recovery_forward_acceleration_mps2;
  float avoid_forward_speed_mps;
  float hold_forward_speed_mps;
  float rejoin_forward_speed_mps;
  float rearm_forward_speed_mps;
  float redirect_forward_speed_mps;
  float hold_release_probability;
  float minimum_hold_forward_progress_m;
  float minimum_rearm_forward_progress_m;
  float rejoin_lateral_tolerance_m;
  float rejoin_lateral_speed_tolerance_mps;
  float redirect_maximum_tilt_rad;
  float redirect_maximum_forward_speed_mps;
  float redirect_maximum_lateral_speed_mps;
  float redirect_minimum_vertical_speed_mps;
  uint8_t trigger_samples_required;
  uint8_t redirect_trigger_samples_required;
  uint8_t redirect_settle_samples_required;
  uint8_t backtrack_settle_samples_required;
  uint8_t hold_clear_samples_required;
  uint8_t rejoin_settle_samples_required;
  uint8_t rearm_clear_samples_required;
  float rejoin_alignment_heading_tolerance_rad;
  uint8_t rejoin_alignment_clear_samples_required;
  float loop_scan_yaw_rad;
  uint8_t loop_scan_samples_required;
  float loop_escape_lateral_step_m;
  float loop_escape_maximum_offset_m;
  float loop_escape_spline_length_m;
} TinyRacerDodgeConfig;

typedef struct {
  bool valid;
  float forward_progress_m;
  float lateral_offset_m;
  float lateral_speed_mps;
  float forward_speed_mps;
  float vertical_speed_mps;
  float tilt_rad;
  float body_rate_rad_s;
  float rejoin_heading_error_rad;
  float loop_scan_heading_error_rad;
} TinyRacerDodgeFeedback;

typedef struct {
  TinyRacerDodgePhase phase;
  float lateral_offset_m;
  float left_collision_probability;
  float center_collision_probability;
  float right_collision_probability;
  float emergency_forward_speed_mps;
  float recovery_forward_speed_mps;
  float backtrack_risk_window[TINYRACER_BACKTRACK_RISK_WINDOW_SAMPLES];
  float backtrack_risk_sum;
  float backtrack_risk_average;
  float loop_scan_risk_sum;
  float loop_scan_left_risk;
  float loop_scan_right_risk;
  float encounter_start_forward_progress_m;
  float rearm_start_forward_progress_m;
  float rejoin_spline_start_forward_progress_m;
  float rejoin_spline_start_offset_m;
  float rejoin_spline_start_slope;
  float rejoin_spline_progress_m;
  float rejoin_spline_slope;
  float lateral_rate_mps;
  float avoidance_target_offset_m;
  bool encounter_progress_valid;
  bool rearm_progress_valid;
  bool rejoin_spline_active;
  bool recovery_forward_speed_active;
  bool loop_scan_pending;
  bool loop_escape_active;
  uint8_t trigger_samples;
  uint8_t redirect_trigger_samples;
  uint8_t redirect_settle_samples;
  uint8_t backtrack_risk_window_count;
  uint8_t backtrack_risk_window_index;
  uint8_t backtrack_cycle_count;
  uint8_t loop_scan_samples;
  int8_t loop_escape_yaw_direction;
  uint8_t backtrack_settle_samples;
  uint8_t hold_clear_samples;
  uint8_t rejoin_settle_samples;
  uint8_t rearm_clear_samples;
  uint8_t rejoin_alignment_clear_samples;
} TinyRacerDodgeState;

typedef struct {
  TinyRacerDodgePhase phase;
  float lateral_offset_m;
  float lateral_rate_mps;
  float forward_speed_mps;
  float avoidance_probability;
} TinyRacerDodgeIntent;

/* Cubic Hermite lateral offset d(s) in a route-relative Frenet chart. The
 * returned slope is dd/ds, so a caller can construct a self-consistent
 * velocity and camera heading from route_speed * (tangent + slope * normal).
 * Clamping station at the endpoints makes both the bypass and route handoffs
 * deterministic. */
static inline void tinyRacerFrenetSplineSample(
    float station_m, float length_m,
    float start_offset_m, float start_slope,
    float end_offset_m, float end_slope,
    float *offset_m, float *slope) {
  const float safe_length_m = length_m > 1.0e-6f ? length_m : 1.0e-6f;
  float t = station_m / safe_length_m;
  if (t < 0.0f) {
    t = 0.0f;
  } else if (t > 1.0f) {
    t = 1.0f;
  }
  const float t2 = t * t;
  const float t3 = t2 * t;
  if (offset_m != NULL) {
    *offset_m =
        (2.0f * t3 - 3.0f * t2 + 1.0f) * start_offset_m +
        (t3 - 2.0f * t2 + t) * safe_length_m * start_slope +
        (-2.0f * t3 + 3.0f * t2) * end_offset_m +
        (t3 - t2) * safe_length_m * end_slope;
  }
  if (slope != NULL) {
    *slope =
        (6.0f * t2 - 6.0f * t) * start_offset_m / safe_length_m +
        (3.0f * t2 - 4.0f * t + 1.0f) * start_slope +
        (-6.0f * t2 + 6.0f * t) * end_offset_m / safe_length_m +
        (3.0f * t2 - 2.0f * t) * end_slope;
  }
}

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
    const TinyRacerDodgeFeedback *feedback,
    float dt_s,
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
