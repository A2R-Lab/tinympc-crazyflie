#ifndef TINYMPC_PROGRESS_PATH_H
#define TINYMPC_PROGRESS_PATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float z;
} TinyMpcPathPoint;

typedef struct {
  TinyMpcPathPoint position;
  TinyMpcPathPoint tangent;
  /* d(tangent)/ds = curvature_magnitude_per_m * principal_normal. */
  TinyMpcPathPoint curvature_vector_per_m;
  float curvature_magnitude_per_m;
  /* Signed horizontal heading curvature d(atan2(t_y,t_x))/ds. */
  float curvature_per_m;
  float speed_mps;
} TinyMpcPathSample;

typedef struct {
  const float *point_data;
  uint16_t point_stride;
  const float *derivative_data;
  uint16_t derivative_stride;
  uint16_t count;
  uint16_t lap_count;
  uint32_t virtual_count;
  /* Virtual reference progress and physically bounded measured progress are
   * intentionally separate. Both are monotonic path coordinates. */
  float progress;
  float measured_progress;
  uint16_t search_back_segments;
  uint16_t search_forward_segments;
  float minimum_speed_mps;
  float maximum_speed_mps;
  float curvature_speed_gain_m;
  float progress_tolerance_m;
  float maximum_command_advance_m;
  bool target_lead_bound_enabled;
  float maximum_target_lead_m;
  float completion_radius_m;
  TinyMpcPathPoint previous_vehicle;
  float cumulative_vehicle_displacement_m;
  float cumulative_forward_displacement_m;
  float cumulative_measured_advance_m;
  float cumulative_commanded_advance_m;
  float last_vehicle_displacement_m;
  float last_forward_displacement_m;
  float last_projection_candidate_advance_m;
  float last_measured_advance_bound_m;
  float last_measured_advance_m;
  float last_commanded_request_m;
  float last_reference_rebase_m;
  float last_commanded_advance_m;
  float cumulative_reference_rebase_m;
  /* A clean state-machine rejoin may legitimately place the vehicle farther
   * along a curved route than forward-displacement integration can prove.
   * Catch that measured phase up gradually; never jump the route clock. */
  bool geometric_catchup_active;
  bool geometric_catchup_update_enabled;
  float geometric_catchup_target_progress;
  float maximum_geometric_catchup_per_update_m;
  float last_geometric_projection_error_m;
  float last_geometric_catchup_m;
  float cumulative_geometric_catchup_m;
  float last_phase_lead_m;
  float last_phase_lead_bound_m;
  float last_phase_lag_m;
  bool vehicle_initialized;
  bool projection_limited;
  bool projection_bound_violation;
  bool command_step_violation;
  bool lead_bound_violation;
  bool lag_bound_violation;
  uint32_t projection_limited_count;
  uint32_t projection_bound_violation_count;
  uint32_t command_step_violation_count;
  uint32_t lead_bound_violation_count;
  uint32_t lag_bound_violation_count;
  bool complete;
} TinyMpcProgressPath;

/* Normalize any regular path derivative dr/du. The result is dr/ds, so this
 * works for arbitrary parameterizations and is not specific to circles. */
static inline TinyMpcPathPoint tinyMpcUnitTangent(
    TinyMpcPathPoint path_derivative) {
  const float norm = hypotf(
      hypotf(path_derivative.x, path_derivative.y), path_derivative.z);
  if (!isfinite(norm) || norm <= 1.0e-6f) {
    return (TinyMpcPathPoint){0.0f, 0.0f, 0.0f};
  }
  return (TinyMpcPathPoint){
      path_derivative.x / norm,
      path_derivative.y / norm,
      path_derivative.z / norm,
  };
}

static inline float tinyMpcPathClamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

/* Scale a proposed tangential speed so v^2 * |kappa| never exceeds the
 * configured centripetal-acceleration budget.  Passing the nominal speed plus
 * any linear-cost reward bias makes the cap apply to the optimizer's complete
 * proposed speed, rather than only to the stored velocity reference. */
static inline float tinyMpcProgressCentripetalSpeedScale(
    float proposed_speed_mps, float curvature_magnitude_per_m,
    float maximum_centripetal_acceleration_mps2) {
  if (!isfinite(proposed_speed_mps)
      || !isfinite(curvature_magnitude_per_m)
      || !isfinite(maximum_centripetal_acceleration_mps2)
      || proposed_speed_mps <= 0.0f
      || curvature_magnitude_per_m < 0.0f
      || maximum_centripetal_acceleration_mps2 <= 0.0f) {
    return 0.0f;
  }
  if (curvature_magnitude_per_m <= 1.0e-6f) {
    return 1.0f;
  }
  const float speed_limit_mps = sqrtf(
      maximum_centripetal_acceleration_mps2
      / curvature_magnitude_per_m);
  if (!isfinite(speed_limit_mps)) {
    return 0.0f;
  }
  return tinyMpcPathClamp(speed_limit_mps / proposed_speed_mps, 0.0f, 1.0f);
}

static inline float tinyMpcProgressRampSpeed(
    float current_speed_mps, float target_speed_mps,
    float maximum_acceleration_mps2, float dt_s) {
  if (!isfinite(current_speed_mps) || !isfinite(target_speed_mps)
      || !isfinite(maximum_acceleration_mps2) || !isfinite(dt_s)
      || target_speed_mps <= 0.0f || maximum_acceleration_mps2 <= 0.0f
      || dt_s <= 0.0f) {
    return 0.0f;
  }
  const float bounded_current = tinyMpcPathClamp(
      current_speed_mps, 0.0f, target_speed_mps);
  return fminf(
      target_speed_mps,
      bounded_current + maximum_acceleration_mps2 * dt_s);
}

static inline float tinyMpcProgressSlewSpeed(
    float current_speed_mps, float target_speed_mps,
    float maximum_acceleration_mps2, float maximum_deceleration_mps2,
    float dt_s) {
  if (!isfinite(current_speed_mps) || !isfinite(target_speed_mps)
      || !isfinite(maximum_acceleration_mps2)
      || !isfinite(maximum_deceleration_mps2) || !isfinite(dt_s)
      || target_speed_mps < 0.0f || maximum_acceleration_mps2 <= 0.0f
      || maximum_deceleration_mps2 <= 0.0f || dt_s <= 0.0f) {
    return 0.0f;
  }
  const float current = fmaxf(current_speed_mps, 0.0f);
  const float delta = target_speed_mps - current;
  return current + tinyMpcPathClamp(
      delta, -maximum_deceleration_mps2 * dt_s,
      maximum_acceleration_mps2 * dt_s);
}

/* Convert measured along-track speed deficit into a conservative
 * attitude/thrust reference acceleration. This does not alter commanded path
 * speed or progress. A terminal zero-speed sample disables the acceleration. */
static inline float tinyMpcProgressTangentialAcceleration(
    float commanded_speed_mps, float measured_tangent_speed_mps) {
  if (!isfinite(commanded_speed_mps)
      || !isfinite(measured_tangent_speed_mps)
      || commanded_speed_mps <= 0.0f) {
    return 0.0f;
  }
  const float raw_acceleration_mps2 =
      (commanded_speed_mps - measured_tangent_speed_mps) / 0.35f;
  if (!isfinite(raw_acceleration_mps2)) {
    return commanded_speed_mps > measured_tangent_speed_mps ? 0.50f : 0.0f;
  }
  return tinyMpcPathClamp(raw_acceleration_mps2, 0.0f, 0.50f);
}

static inline float tinyMpcPathSegmentLength(
    const TinyMpcProgressPath *path, uint32_t segment);
static inline float tinyMpcProgressPathAdvance(
    const TinyMpcProgressPath *path, float progress, float distance_m);
static inline float tinyMpcProgressPathDistance(
    const TinyMpcProgressPath *path, float from_progress, float to_progress);
static inline TinyMpcPathSample tinyMpcProgressPathSample(
    const TinyMpcProgressPath *path, float progress);

static inline TinyMpcPathPoint tinyMpcProgressPathPoint(
    const TinyMpcProgressPath *path, uint32_t index) {
  uint32_t source_index = index;
  if (path->lap_count > 1u && path->count > 1u) {
    const uint32_t segments_per_lap = (uint32_t)path->count - 1u;
    source_index = index >= path->virtual_count - 1u
        ? (uint32_t)path->count - 1u
        : index % segments_per_lap;
  }
  const float *point = path->point_data + source_index * path->point_stride;
  const TinyMpcPathPoint result = {point[0], point[1], point[2]};
  return result;
}

static inline float tinyMpcPathSegmentLength(
    const TinyMpcProgressPath *path, uint32_t segment) {
  const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
  const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float dz = b.z - a.z;
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

static inline void tinyMpcProgressPathInitLaps(
    TinyMpcProgressPath *path, const float *point_data, uint16_t point_stride,
    uint16_t count,
    uint16_t search_back_segments, uint16_t search_forward_segments,
    float minimum_speed_mps, float maximum_speed_mps,
    float curvature_speed_gain_m, float progress_tolerance_m,
    float maximum_command_advance_m, float completion_radius_m,
    uint16_t lap_count) {
  path->point_data = point_data;
  path->point_stride = point_stride;
  path->derivative_data = 0;
  path->derivative_stride = 0u;
  path->count = count;
  path->lap_count = lap_count > 0u ? lap_count : 1u;
  path->virtual_count = count < 2u
      ? count
      : (uint32_t)(count - 1u) * path->lap_count + 1u;
  path->progress = 0.0f;
  path->search_back_segments = search_back_segments;
  path->search_forward_segments = search_forward_segments;
  path->minimum_speed_mps = minimum_speed_mps;
  path->maximum_speed_mps = maximum_speed_mps;
  path->curvature_speed_gain_m = curvature_speed_gain_m;
  path->progress_tolerance_m = progress_tolerance_m;
  path->maximum_command_advance_m = maximum_command_advance_m;
  path->target_lead_bound_enabled = true;
  path->maximum_target_lead_m =
      maximum_command_advance_m + progress_tolerance_m;
  path->completion_radius_m = completion_radius_m;
  path->measured_progress = 0.0f;
  path->previous_vehicle = (TinyMpcPathPoint){0.0f, 0.0f, 0.0f};
  path->cumulative_vehicle_displacement_m = 0.0f;
  path->cumulative_forward_displacement_m = 0.0f;
  path->cumulative_measured_advance_m = 0.0f;
  path->cumulative_commanded_advance_m = 0.0f;
  path->last_vehicle_displacement_m = 0.0f;
  path->last_forward_displacement_m = 0.0f;
  path->last_projection_candidate_advance_m = 0.0f;
  path->last_measured_advance_bound_m = 0.0f;
  path->last_measured_advance_m = 0.0f;
  path->last_commanded_request_m = 0.0f;
  path->last_reference_rebase_m = 0.0f;
  path->last_commanded_advance_m = 0.0f;
  path->cumulative_reference_rebase_m = 0.0f;
  path->geometric_catchup_active = false;
  path->geometric_catchup_update_enabled = false;
  path->geometric_catchup_target_progress = 0.0f;
  path->maximum_geometric_catchup_per_update_m = 0.0f;
  path->last_geometric_projection_error_m = 0.0f;
  path->last_geometric_catchup_m = 0.0f;
  path->cumulative_geometric_catchup_m = 0.0f;
  path->last_phase_lead_m = 0.0f;
  path->last_phase_lead_bound_m = 0.0f;
  path->last_phase_lag_m = 0.0f;
  path->vehicle_initialized = false;
  path->projection_limited = false;
  path->projection_bound_violation = false;
  path->command_step_violation = false;
  path->lead_bound_violation = false;
  path->lag_bound_violation = false;
  path->projection_limited_count = 0u;
  path->projection_bound_violation_count = 0u;
  path->command_step_violation_count = 0u;
  path->lead_bound_violation_count = 0u;
  path->lag_bound_violation_count = 0u;
  path->complete = count < 2u;
}

/* Supply analytical dr/du samples corresponding one-for-one with path points.
 * Normalization in tinyMpcUnitTangent() converts them to dr/ds. */
static inline void tinyMpcProgressPathSetAnalyticalDerivatives(
    TinyMpcProgressPath *path, const float *derivative_data,
    uint16_t derivative_stride) {
  path->derivative_data = derivative_data;
  path->derivative_stride = derivative_stride;
}

/* Opt out of tying virtual target phase to measured forward displacement.
 * Measured phase remains physically bounded and still owns completion. */
static inline void tinyMpcProgressPathSetTargetLeadBound(
    TinyMpcProgressPath *path, bool enabled) {
  path->target_lead_bound_enabled = enabled;
}

/* Set an explicit maximum arc-length separation between virtual target phase
 * and physically measured phase. This does not alter the per-update command
 * step or measured-progress/completion invariants. */
static inline void tinyMpcProgressPathSetMaximumTargetLead(
    TinyMpcProgressPath *path, float maximum_target_lead_m) {
  if (!isfinite(maximum_target_lead_m) || maximum_target_lead_m < 0.0f) {
    return;
  }
  path->maximum_target_lead_m = maximum_target_lead_m;
  path->target_lead_bound_enabled = true;
}

static inline void tinyMpcProgressPathInit(
    TinyMpcProgressPath *path, const float *point_data, uint16_t point_stride,
    uint16_t count,
    uint16_t search_back_segments, uint16_t search_forward_segments,
    float minimum_speed_mps, float maximum_speed_mps,
    float curvature_speed_gain_m, float progress_tolerance_m,
    float maximum_command_advance_m, float completion_radius_m) {
  tinyMpcProgressPathInitLaps(
      path, point_data, point_stride, count,
      search_back_segments, search_forward_segments,
      minimum_speed_mps, maximum_speed_mps, curvature_speed_gain_m,
      progress_tolerance_m, maximum_command_advance_m, completion_radius_m,
      1u);
}

static inline float tinyMpcProgressPathTerminalProgress(
    const TinyMpcProgressPath *path) {
  return path->virtual_count > 0u ? (float)(path->virtual_count - 1u) : 0.0f;
}

static inline uint16_t tinyMpcProgressPathCompletedLaps(
    const TinyMpcProgressPath *path, float progress) {
  if (path->count < 2u || path->lap_count == 0u) {
    return 0u;
  }
  const float segments_per_lap = (float)(path->count - 1u);
  const float bounded = tinyMpcPathClamp(
      progress, 0.0f, tinyMpcProgressPathTerminalProgress(path));
  const uint32_t completed = (uint32_t)floorf(bounded / segments_per_lap);
  return (uint16_t)(completed < path->lap_count
      ? completed : path->lap_count);
}

static inline float tinyMpcProgressPathLapProgress(
    const TinyMpcProgressPath *path, float progress) {
  if (path->count < 2u) {
    return 0.0f;
  }
  const float segments_per_lap = (float)(path->count - 1u);
  const float bounded = tinyMpcPathClamp(
      progress, 0.0f, tinyMpcProgressPathTerminalProgress(path));
  const uint16_t completed = tinyMpcProgressPathCompletedLaps(path, bounded);
  return completed >= path->lap_count
      ? segments_per_lap
      : bounded - (float)completed * segments_per_lap;
}

static inline float tinyMpcProgressPathProjectionCandidateWindow(
    const TinyMpcProgressPath *path, TinyMpcPathPoint vehicle,
    uint16_t search_back_segments, uint16_t search_forward_segments,
    float *projection_error_m) {
  if (path->complete || path->point_data == 0 || path->count < 2u) {
    if (projection_error_m != 0) {
      *projection_error_m = 0.0f;
    }
    return path->measured_progress;
  }
  const uint32_t terminal_segment = path->virtual_count - 2u;
  const uint32_t current = (uint32_t)tinyMpcPathClamp(
      floorf(path->measured_progress), 0.0f, (float)terminal_segment);
  const uint32_t first = current > search_back_segments
      ? current - search_back_segments : 0u;
  uint32_t requested_last = (uint32_t)current + search_forward_segments;
  const uint32_t last = requested_last <= terminal_segment
      ? requested_last : terminal_segment;
  const uint32_t current_segment = (uint32_t)tinyMpcPathClamp(
      floorf(path->measured_progress), 0.0f, (float)terminal_segment);
  const float current_alpha = path->measured_progress - (float)current_segment;
  const TinyMpcPathPoint current_a = tinyMpcProgressPathPoint(path, current_segment);
  const TinyMpcPathPoint current_b = tinyMpcProgressPathPoint(path, current_segment + 1u);
  const float current_x = current_a.x + current_alpha * (current_b.x - current_a.x);
  const float current_y = current_a.y + current_alpha * (current_b.y - current_a.y);
  const float current_z = current_a.z + current_alpha * (current_b.z - current_a.z);
  float best_distance_sq =
      (vehicle.x - current_x) * (vehicle.x - current_x)
      + (vehicle.y - current_y) * (vehicle.y - current_y)
      + (vehicle.z - current_z) * (vehicle.z - current_z);
  float best_progress = path->measured_progress;
  for (uint32_t segment = first; segment <= last; ++segment) {
    const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
    const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float dz = b.z - a.z;
    const float length_sq = dx * dx + dy * dy + dz * dz;
    if (length_sq <= 1.0e-10f) {
      continue;
    }
    const float alpha = tinyMpcPathClamp(
        ((vehicle.x - a.x) * dx + (vehicle.y - a.y) * dy
         + (vehicle.z - a.z) * dz) / length_sq,
        0.0f, 1.0f);
    const float error_x = vehicle.x - (a.x + alpha * dx);
    const float error_y = vehicle.y - (a.y + alpha * dy);
    const float error_z = vehicle.z - (a.z + alpha * dz);
    const float distance_sq =
        error_x * error_x + error_y * error_y + error_z * error_z;
    const float candidate_progress = (float)segment + alpha;
    if (distance_sq < best_distance_sq &&
        candidate_progress + 1.0e-4f >= path->measured_progress) {
      best_distance_sq = distance_sq;
      best_progress = candidate_progress;
    }
  }
  if (projection_error_m != 0) {
    *projection_error_m = sqrtf(fmaxf(best_distance_sq, 0.0f));
  }
  return best_progress;
}

static inline float tinyMpcProgressPathProjectionCandidate(
    const TinyMpcProgressPath *path, TinyMpcPathPoint vehicle) {
  return tinyMpcProgressPathProjectionCandidateWindow(
      path, vehicle, path->search_back_segments,
      path->search_forward_segments, 0);
}

/* Schedule a bounded phase correction after a clean avoidance rejoin.  The
 * wide projection is evaluated only at that transition, so ordinary 50 Hz
 * path updates retain their small local search.  Cross-track gating prevents
 * a visually plausible but geometrically distant branch from advancing the
 * measured route phase. */
static inline bool tinyMpcProgressPathScheduleGeometricCatchup(
    TinyMpcProgressPath *path, TinyMpcPathPoint vehicle,
    uint16_t search_forward_segments, float maximum_cross_track_error_m,
    float maximum_catchup_per_update_m) {
  if (path == 0 || path->complete || !isfinite(maximum_cross_track_error_m)
      || !isfinite(maximum_catchup_per_update_m)
      || maximum_cross_track_error_m < 0.0f
      || maximum_catchup_per_update_m <= 0.0f) {
    return false;
  }
  float projection_error_m = 0.0f;
  const float candidate = tinyMpcProgressPathProjectionCandidateWindow(
      path, vehicle, 0u, search_forward_segments, &projection_error_m);
  path->last_geometric_projection_error_m = projection_error_m;
  const float candidate_advance_m = tinyMpcProgressPathDistance(
      path, path->measured_progress, candidate);
  /* A point on a repeated closed route has an identical copy every lap.  A
   * forward-only nearest-point search can therefore prefer the next-lap copy
   * by floating-point noise even when the vehicle is already at its current
   * phase.  Advances of half a lap or more are geometrically ambiguous and
   * must be earned by ordinary measured motion, never by rejoin catch-up. */
  if (path->lap_count > 1u && path->count > 1u) {
    const float lap_distance_m = tinyMpcProgressPathDistance(
        path, 0.0f, (float)(path->count - 1u));
    if (isfinite(lap_distance_m) && lap_distance_m > 1.0e-5f
        && candidate_advance_m >= 0.5f * lap_distance_m) {
      return false;
    }
  }
  if (!isfinite(candidate) || !isfinite(projection_error_m)
      || projection_error_m > maximum_cross_track_error_m
      || candidate_advance_m <= 1.0e-5f) {
    return false;
  }
  path->geometric_catchup_target_progress = candidate;
  path->maximum_geometric_catchup_per_update_m =
      maximum_catchup_per_update_m;
  path->geometric_catchup_active = true;
  path->geometric_catchup_update_enabled = true;
  return true;
}

/* Pause catch-up while an avoidance offset is active. The last clean target
 * is retained and is replaced by a new wide projection on the next rejoin. */
static inline void tinyMpcProgressPathSetGeometricCatchupUpdateEnabled(
    TinyMpcProgressPath *path, bool enabled) {
  if (path != 0) {
    path->geometric_catchup_update_enabled = enabled;
  }
}

static inline float tinyMpcProgressPathUpdate(
    TinyMpcProgressPath *path, TinyMpcPathPoint vehicle,
    float commanded_advance_m) {
  if (path->complete || path->point_data == 0 || path->count < 2u) {
    return path->progress;
  }

  path->last_vehicle_displacement_m = 0.0f;
  path->last_forward_displacement_m = 0.0f;
  path->last_geometric_catchup_m = 0.0f;
  if (path->vehicle_initialized) {
    const float dx = vehicle.x - path->previous_vehicle.x;
    const float dy = vehicle.y - path->previous_vehicle.y;
    const float dz = vehicle.z - path->previous_vehicle.z;
    path->last_vehicle_displacement_m = sqrtf(dx * dx + dy * dy + dz * dz);
    if (isfinite(path->last_vehicle_displacement_m)) {
      path->cumulative_vehicle_displacement_m +=
          path->last_vehicle_displacement_m;
      const TinyMpcPathSample measured_sample = tinyMpcProgressPathSample(
          path, path->measured_progress);
      const float forward_displacement_m =
          dx * measured_sample.tangent.x
          + dy * measured_sample.tangent.y
          + dz * measured_sample.tangent.z;
      path->last_forward_displacement_m = fminf(
          fmaxf(forward_displacement_m, 0.0f),
          path->last_vehicle_displacement_m);
      path->cumulative_forward_displacement_m +=
          path->last_forward_displacement_m;
    } else {
      path->last_vehicle_displacement_m = 0.0f;
    }
  } else {
    path->vehicle_initialized = true;
  }
  path->previous_vehicle = vehicle;

  const float projection_candidate =
      tinyMpcProgressPathProjectionCandidate(path, vehicle);
  const uint32_t projection_terminal_segment = path->virtual_count - 2u;
  const uint32_t projection_current_segment =
      (uint32_t)tinyMpcPathClamp(
          floorf(path->measured_progress), 0.0f,
          (float)projection_terminal_segment);
  const uint32_t projection_last_segment =
      projection_current_segment + path->search_forward_segments
              <= projection_terminal_segment
          ? projection_current_segment + path->search_forward_segments
          : projection_terminal_segment;
  const float projection_window_end = fminf(
      (float)(projection_last_segment + 1u),
      tinyMpcProgressPathTerminalProgress(path));
  const bool projection_at_forward_window_edge =
      projection_window_end > path->measured_progress + 1.0e-5f
      && projection_candidate >= projection_window_end - 1.0e-3f;
  path->last_projection_candidate_advance_m = tinyMpcProgressPathDistance(
      path, path->measured_progress, projection_candidate);
  const float cumulative_measured_bound_m =
      path->cumulative_forward_displacement_m + path->progress_tolerance_m;
  /* Forward travel that was not geometrically projected is not banked for a
   * later shortcut. Only this solve's forward motion plus the unconsumed part
   * of the single global tolerance can advance measured phase. */
  const float cumulative_remaining_m = fmaxf(
      cumulative_measured_bound_m - path->cumulative_measured_advance_m,
      0.0f);
  path->last_measured_advance_bound_m = fminf(
      cumulative_remaining_m,
      path->last_forward_displacement_m + path->progress_tolerance_m);
  const float accepted_measured_advance_m = fminf(
      path->last_projection_candidate_advance_m,
      path->last_measured_advance_bound_m);
  const float previous_measured_progress = path->measured_progress;
  path->measured_progress = tinyMpcProgressPathAdvance(
      path, path->measured_progress, accepted_measured_advance_m);
  path->last_measured_advance_m = tinyMpcProgressPathDistance(
      path, previous_measured_progress, path->measured_progress);
  path->cumulative_measured_advance_m += path->last_measured_advance_m;
  if (path->geometric_catchup_active
      && path->geometric_catchup_update_enabled) {
    /* The rejoin projection is a fixed correction target. Ratcheting it with
     * each later local projection turns a short phase correction into a
     * permanent synthetic-speed source, so the target must not move after it
     * has been accepted. */
    const float remaining_catchup_m = tinyMpcProgressPathDistance(
        path, path->measured_progress,
        path->geometric_catchup_target_progress);
    const float requested_catchup_m = fminf(
        remaining_catchup_m,
        path->maximum_geometric_catchup_per_update_m);
    const float before_catchup = path->measured_progress;
    path->measured_progress = tinyMpcProgressPathAdvance(
        path, path->measured_progress, requested_catchup_m);
    path->last_geometric_catchup_m = tinyMpcProgressPathDistance(
        path, before_catchup, path->measured_progress);
    path->cumulative_geometric_catchup_m +=
        path->last_geometric_catchup_m;
    path->last_measured_advance_m += path->last_geometric_catchup_m;
    path->cumulative_measured_advance_m += path->last_geometric_catchup_m;
    path->last_measured_advance_bound_m += path->last_geometric_catchup_m;
    if (remaining_catchup_m <=
            path->last_geometric_catchup_m + 1.0e-5f
        && !projection_at_forward_window_edge) {
      path->geometric_catchup_active = false;
    }
  }
  path->projection_limited =
      path->last_projection_candidate_advance_m >
      path->last_measured_advance_m + 1.0e-5f;
  path->projection_bound_violation =
      path->cumulative_measured_advance_m >
      cumulative_measured_bound_m
          + path->cumulative_geometric_catchup_m + 1.0e-5f;

  path->last_commanded_request_m =
      isfinite(commanded_advance_m) && commanded_advance_m > 0.0f
      ? fminf(commanded_advance_m, path->maximum_command_advance_m) : 0.0f;
  /* A physically bounded projection is the measurement of path phase, not a
   * speed command.  If the vehicle has moved ahead of the virtual reference,
   * rebase the reference to measured phase before applying the next commanded
   * advance.  Leaving the reference behind would ask the controller to fly
   * backward and would center path constraints on an obsolete cross-section. */
  const float rebased_progress = fmaxf(
      path->progress, path->measured_progress);
  path->last_reference_rebase_m = tinyMpcProgressPathDistance(
      path, path->progress, rebased_progress);
  path->cumulative_reference_rebase_m += path->last_reference_rebase_m;
  const float requested_progress = tinyMpcProgressPathAdvance(
      path, rebased_progress, path->last_commanded_request_m);
  path->last_phase_lead_bound_m = path->target_lead_bound_enabled
      ? path->maximum_target_lead_m
      : tinyMpcProgressPathDistance(
          path, path->measured_progress, requested_progress);
  const float bounded_progress = path->target_lead_bound_enabled
      ? fminf(
          requested_progress,
          tinyMpcProgressPathAdvance(
              path, path->measured_progress, path->last_phase_lead_bound_m))
      : requested_progress;
  path->progress = fmaxf(rebased_progress, bounded_progress);
  path->last_commanded_advance_m = tinyMpcProgressPathDistance(
      path, rebased_progress, path->progress);
  path->cumulative_commanded_advance_m += path->last_commanded_advance_m;
  path->last_phase_lead_m = tinyMpcProgressPathDistance(
      path, path->measured_progress, path->progress);
  path->last_phase_lag_m = tinyMpcProgressPathDistance(
      path, path->progress, path->measured_progress);
  path->command_step_violation = path->last_commanded_advance_m >
      path->last_commanded_request_m + 1.0e-5f;
  path->lead_bound_violation = path->last_phase_lead_m >
      path->last_phase_lead_bound_m + 1.0e-5f;
  path->lag_bound_violation = path->last_phase_lag_m > 1.0e-5f;
  path->projection_limited_count += path->projection_limited ? 1u : 0u;
  path->projection_bound_violation_count +=
      path->projection_bound_violation ? 1u : 0u;
  path->command_step_violation_count += path->command_step_violation ? 1u : 0u;
  path->lead_bound_violation_count += path->lead_bound_violation ? 1u : 0u;
  path->lag_bound_violation_count += path->lag_bound_violation ? 1u : 0u;

  const TinyMpcPathPoint terminal = tinyMpcProgressPathPoint(
      path, path->virtual_count - 1u);
  const float terminal_distance_sq =
      (vehicle.x - terminal.x) * (vehicle.x - terminal.x)
      + (vehicle.y - terminal.y) * (vehicle.y - terminal.y)
      + (vehicle.z - terminal.z) * (vehicle.z - terminal.z);
  const float measured_terminal_distance_m = tinyMpcProgressPathDistance(
      path, path->measured_progress, tinyMpcProgressPathTerminalProgress(path));
  /* A speed ramp can pack several final knots into less than the physical
   * progress tolerance. Treat that sub-tolerance tail as terminal only after
   * the commanded target and vehicle have both reached the endpoint. */
  const bool measured_at_terminal = measured_terminal_distance_m <=
      path->progress_tolerance_m;
  const bool target_at_terminal = path->progress >=
      tinyMpcProgressPathTerminalProgress(path) - 1.0e-3f;
  if (measured_at_terminal && target_at_terminal &&
      terminal_distance_sq <=
          path->completion_radius_m * path->completion_radius_m) {
    path->measured_progress = tinyMpcProgressPathTerminalProgress(path);
    path->progress = tinyMpcProgressPathTerminalProgress(path);
    path->complete = true;
  }
  return path->progress;
}

static inline float tinyMpcProgressPathAdvance(
    const TinyMpcProgressPath *path, float progress, float distance_m) {
  if (path->point_data == 0 || path->count < 2u || distance_m <= 0.0f) {
    return tinyMpcPathClamp(
        progress, 0.0f, tinyMpcProgressPathTerminalProgress(path));
  }
  const float terminal_progress = tinyMpcProgressPathTerminalProgress(path);
  float advanced = tinyMpcPathClamp(
      progress, 0.0f, terminal_progress);
  float remaining = distance_m;
  while (remaining > 0.0f && advanced < terminal_progress) {
    const uint32_t segment = (uint32_t)tinyMpcPathClamp(
        floorf(advanced), 0.0f, terminal_progress - 1.0f);
    const float length = tinyMpcPathSegmentLength(path, segment);
    if (length <= 1.0e-6f) {
      advanced = (float)(segment + 1u);
      continue;
    }
    const float alpha = advanced - (float)segment;
    const float available = (1.0f - alpha) * length;
    if (remaining < available) {
      advanced += remaining / length;
      remaining = 0.0f;
    } else {
      remaining -= available;
      advanced = (float)(segment + 1u);
    }
  }
  return tinyMpcPathClamp(advanced, 0.0f, terminal_progress);
}

static inline float tinyMpcProgressPathDistance(
    const TinyMpcProgressPath *path, float from_progress, float to_progress) {
  if (path->point_data == 0 || path->count < 2u ||
      to_progress <= from_progress) {
    return 0.0f;
  }
  float cursor = tinyMpcPathClamp(
      from_progress, 0.0f, tinyMpcProgressPathTerminalProgress(path));
  const float destination = tinyMpcPathClamp(
      to_progress, cursor, tinyMpcProgressPathTerminalProgress(path));
  float distance_m = 0.0f;
  while (cursor < destination) {
    const uint32_t segment = (uint32_t)tinyMpcPathClamp(
        floorf(cursor), 0.0f,
        tinyMpcProgressPathTerminalProgress(path) - 1.0f);
    const float segment_end = fminf((float)(segment + 1u), destination);
    const float length = tinyMpcPathSegmentLength(path, segment);
    distance_m += (segment_end - cursor) * length;
    cursor = segment_end;
  }
  return distance_m;
}

/* Continuous stopping-distance envelope. For closed racing paths the caller
 * supplies an extra virtual lap, so the requested finish can be crossed at
 * race speed before this envelope decelerates through the cooldown rollout. */
static inline float tinyMpcProgressTerminalSpeedLimit(
    const TinyMpcProgressPath *path, float progress,
    float maximum_deceleration_mps2) {
  if (path == 0 || !isfinite(progress)
      || !isfinite(maximum_deceleration_mps2)
      || maximum_deceleration_mps2 <= 0.0f) {
    return 0.0f;
  }
  const float remaining_distance_m = tinyMpcProgressPathDistance(
      path, progress, tinyMpcProgressPathTerminalProgress(path));
  return sqrtf(fmaxf(
      2.0f * maximum_deceleration_mps2 * remaining_distance_m, 0.0f));
}

static inline float tinyMpcProgressTerminalRewardScale(
    float nominal_rewarded_speed_mps, float terminal_speed_limit_mps) {
  if (!isfinite(nominal_rewarded_speed_mps)
      || !isfinite(terminal_speed_limit_mps)
      || nominal_rewarded_speed_mps <= 0.0f
      || terminal_speed_limit_mps <= 0.0f) {
    return 0.0f;
  }
  const float ratio = tinyMpcPathClamp(
      terminal_speed_limit_mps / nominal_rewarded_speed_mps, 0.0f, 1.0f);
  /* Smoothstep avoids a derivative discontinuity when braking begins. */
  return ratio * ratio * (3.0f - 2.0f * ratio);
}

static inline TinyMpcPathSample tinyMpcProgressPathSample(
    const TinyMpcProgressPath *path, float progress) {
  TinyMpcPathSample sample = {
      {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f};
  if (path->point_data == 0 || path->count == 0u) {
    return sample;
  }
  if (path->count == 1u) {
    sample.position = tinyMpcProgressPathPoint(path, 0u);
    return sample;
  }
  const float terminal_progress = tinyMpcProgressPathTerminalProgress(path);
  progress = tinyMpcPathClamp(progress, 0.0f, terminal_progress);
  const uint32_t segment = (uint32_t)tinyMpcPathClamp(
      floorf(progress), 0.0f, terminal_progress - 1.0f);
  const float alpha = progress - (float)segment;
  const TinyMpcPathPoint a = tinyMpcProgressPathPoint(path, segment);
  const TinyMpcPathPoint b = tinyMpcProgressPathPoint(path, segment + 1u);
  sample.position.x = a.x + alpha * (b.x - a.x);
  sample.position.y = a.y + alpha * (b.y - a.y);
  sample.position.z = a.z + alpha * (b.z - a.z);
  float segment_length = tinyMpcPathSegmentLength(path, segment);
  if (path->derivative_data != 0 && path->derivative_stride >= 3u) {
    const uint32_t segments_per_lap = (uint32_t)path->count - 1u;
    const uint32_t source_segment = path->lap_count > 1u
        ? segment % segments_per_lap : segment;
    const uint32_t source_next = source_segment + 1u;
    const float *derivative_a = path->derivative_data
        + source_segment * path->derivative_stride;
    const float *derivative_b = path->derivative_data
        + source_next * path->derivative_stride;
    TinyMpcPathPoint derivative = {
        (1.0f - alpha) * derivative_a[0] + alpha * derivative_b[0],
        (1.0f - alpha) * derivative_a[1] + alpha * derivative_b[1],
        (1.0f - alpha) * derivative_a[2] + alpha * derivative_b[2],
    };
    sample.tangent = tinyMpcUnitTangent(derivative);
    if (sample.tangent.x == 0.0f && sample.tangent.y == 0.0f
        && sample.tangent.z == 0.0f) {
      /* Time-scaled trajectories can have zero derivative at an endpoint.
       * Search only the supplied analytical derivatives for the closest
       * regular sample; never infer direction from position differences. */
      for (uint32_t offset = 1u; offset < path->count; ++offset) {
        if (source_segment + offset < path->count) {
          const float *candidate = path->derivative_data
              + (source_segment + offset) * path->derivative_stride;
          sample.tangent = tinyMpcUnitTangent(
              (TinyMpcPathPoint){candidate[0], candidate[1], candidate[2]});
        }
        if (sample.tangent.x != 0.0f || sample.tangent.y != 0.0f
            || sample.tangent.z != 0.0f) {
          break;
        }
        if (offset <= source_segment) {
          const float *candidate = path->derivative_data
              + (source_segment - offset) * path->derivative_stride;
          sample.tangent = tinyMpcUnitTangent(
              (TinyMpcPathPoint){candidate[0], candidate[1], candidate[2]});
          if (sample.tangent.x != 0.0f || sample.tangent.y != 0.0f
              || sample.tangent.z != 0.0f) {
            break;
          }
        }
      }
    }
  } else if (segment_length > 1.0e-6f) {
    const TinyMpcPathPoint current_tangent = {
        (b.x - a.x) / segment_length,
        (b.y - a.y) / segment_length,
        (b.z - a.z) / segment_length,
    };
    TinyMpcPathPoint tangent_at_a = current_tangent;
    TinyMpcPathPoint tangent_at_b = current_tangent;
    if (segment > 0u) {
      const TinyMpcPathPoint previous = tinyMpcProgressPathPoint(
          path, segment - 1u);
      const float previous_length = hypotf(
          hypotf(a.x - previous.x, a.y - previous.y),
          a.z - previous.z);
      if (previous_length > 1.0e-6f) {
        tangent_at_a.x += (a.x - previous.x) / previous_length;
        tangent_at_a.y += (a.y - previous.y) / previous_length;
        tangent_at_a.z += (a.z - previous.z) / previous_length;
      }
    }
    if (segment + 2u < path->virtual_count) {
      const TinyMpcPathPoint next = tinyMpcProgressPathPoint(
          path, segment + 2u);
      const float next_length = hypotf(
          hypotf(next.x - b.x, next.y - b.y),
          next.z - b.z);
      if (next_length > 1.0e-6f) {
        tangent_at_b.x += (next.x - b.x) / next_length;
        tangent_at_b.y += (next.y - b.y) / next_length;
        tangent_at_b.z += (next.z - b.z) / next_length;
      }
    }
    const float tangent_a_norm = hypotf(
        hypotf(tangent_at_a.x, tangent_at_a.y), tangent_at_a.z);
    const float tangent_b_norm = hypotf(
        hypotf(tangent_at_b.x, tangent_at_b.y), tangent_at_b.z);
    if (tangent_a_norm > 1.0e-6f) {
      tangent_at_a.x /= tangent_a_norm;
      tangent_at_a.y /= tangent_a_norm;
      tangent_at_a.z /= tangent_a_norm;
    }
    if (tangent_b_norm > 1.0e-6f) {
      tangent_at_b.x /= tangent_b_norm;
      tangent_at_b.y /= tangent_b_norm;
      tangent_at_b.z /= tangent_b_norm;
    }
    sample.tangent.x =
        (1.0f - alpha) * tangent_at_a.x + alpha * tangent_at_b.x;
    sample.tangent.y =
        (1.0f - alpha) * tangent_at_a.y + alpha * tangent_at_b.y;
    sample.tangent.z =
        (1.0f - alpha) * tangent_at_a.z + alpha * tangent_at_b.z;
    const float tangent_norm = hypotf(
        hypotf(sample.tangent.x, sample.tangent.y), sample.tangent.z);
    if (tangent_norm > 1.0e-6f) {
      sample.tangent.x /= tangent_norm;
      sample.tangent.y /= tangent_norm;
      sample.tangent.z /= tangent_norm;
    }
  }
  if (path->virtual_count >= 3u) {
    uint32_t first_segment = segment + 1u < path->virtual_count - 1u
        ? segment : segment - 1u;
    TinyMpcPathPoint first = {0.0f, 0.0f, 0.0f};
    TinyMpcPathPoint second = {0.0f, 0.0f, 0.0f};
    float first_length = 0.0f;
    float second_length = 0.0f;
    /* Generated closed paths may end in sub-micrometre settling segments.
     * Those points preserve the exact endpoint but are too short to define
     * curvature. Walk backward to the nearest two valid adjacent segments so
     * the terminal horizon retains the physical turn instead of snapping its
     * bank reference to level in one knot. */
    for (;;) {
      const TinyMpcPathPoint first_a =
          tinyMpcProgressPathPoint(path, first_segment);
      const TinyMpcPathPoint first_b =
          tinyMpcProgressPathPoint(path, first_segment + 1u);
      const TinyMpcPathPoint second_b =
          tinyMpcProgressPathPoint(path, first_segment + 2u);
      first = (TinyMpcPathPoint){
          first_b.x - first_a.x,
          first_b.y - first_a.y,
          first_b.z - first_a.z};
      second = (TinyMpcPathPoint){
          second_b.x - first_b.x,
          second_b.y - first_b.y,
          second_b.z - first_b.z};
      first_length = hypotf(hypotf(first.x, first.y), first.z);
      second_length = hypotf(hypotf(second.x, second.y), second.z);
      if ((first_length > 1.0e-4f && second_length > 1.0e-4f)
          || first_segment == 0u) {
        break;
      }
      --first_segment;
    }
    if (first_length > 1.0e-4f && second_length > 1.0e-4f) {
      const TinyMpcPathPoint first_tangent = {
          first.x / first_length, first.y / first_length,
          first.z / first_length};
      const TinyMpcPathPoint second_tangent = {
          second.x / second_length, second.y / second_length,
          second.z / second_length};
      const float average_arc_length = 0.5f * (
          first_length + second_length);
      const float tangent_dot = tinyMpcPathClamp(
          first_tangent.x * second_tangent.x
              + first_tangent.y * second_tangent.y
              + first_tangent.z * second_tangent.z,
          -1.0f, 1.0f);
      const float turn_magnitude = acosf(tangent_dot);
      TinyMpcPathPoint normal_direction = {
          second_tangent.x - first_tangent.x,
          second_tangent.y - first_tangent.y,
          second_tangent.z - first_tangent.z};
      const float normal_norm = hypotf(
          hypotf(normal_direction.x, normal_direction.y),
          normal_direction.z);
      sample.curvature_magnitude_per_m =
          turn_magnitude / average_arc_length;
      if (normal_norm > 1.0e-6f) {
        const float scale =
            sample.curvature_magnitude_per_m / normal_norm;
        sample.curvature_vector_per_m.x = normal_direction.x * scale;
        sample.curvature_vector_per_m.y = normal_direction.y * scale;
        sample.curvature_vector_per_m.z = normal_direction.z * scale;
      }
      const float first_horizontal = hypotf(first.x, first.y);
      const float second_horizontal = hypotf(second.x, second.y);
      if (first_horizontal > 1.0e-6f && second_horizontal > 1.0e-6f) {
        const float heading_cross =
            first.x * second.y - first.y * second.x;
        const float heading_dot =
            first.x * second.x + first.y * second.y;
        sample.curvature_per_m =
            atan2f(heading_cross, heading_dot) / average_arc_length;
      }
    }
  }
  const float curvature_scale = 1.0f + path->curvature_speed_gain_m
      * sample.curvature_magnitude_per_m;
  sample.speed_mps = tinyMpcPathClamp(
      path->maximum_speed_mps / curvature_scale,
      path->minimum_speed_mps, path->maximum_speed_mps);
  if (path->complete) {
    sample.speed_mps = 0.0f;
    sample.curvature_vector_per_m =
        (TinyMpcPathPoint){0.0f, 0.0f, 0.0f};
    sample.curvature_magnitude_per_m = 0.0f;
    sample.curvature_per_m = 0.0f;
  }
  return sample;
}

/* Back-propagate future curvature limits through a constant-deceleration
 * envelope: v_now^2 <= v_limit(s)^2 + 2*a_brake*distance.  This prevents a
 * locally valid cap from arriving too late when the vehicle approaches a
 * tight bend at high speed. */
static inline float tinyMpcProgressCentripetalBrakingSpeedScale(
    const TinyMpcProgressPath *path, float progress,
    float proposed_speed_mps, float maximum_centripetal_acceleration_mps2,
    float maximum_deceleration_mps2, float lookbehind_distance_m,
    float lookahead_distance_m, float sample_distance_m) {
  if (path == NULL || !isfinite(progress) || !isfinite(proposed_speed_mps)
      || !isfinite(maximum_centripetal_acceleration_mps2)
      || !isfinite(maximum_deceleration_mps2)
      || !isfinite(lookbehind_distance_m)
      || !isfinite(lookahead_distance_m) || !isfinite(sample_distance_m)
      || proposed_speed_mps <= 0.0f
      || maximum_centripetal_acceleration_mps2 <= 0.0f
      || maximum_deceleration_mps2 <= 0.0f
      || lookbehind_distance_m < 0.0f || lookahead_distance_m < 0.0f
      || sample_distance_m <= 0.0f) {
    return 0.0f;
  }
  float allowed_speed_mps = proposed_speed_mps;
  float behind_progress = tinyMpcPathClamp(
      progress, 0.0f, (float)(path->virtual_count - 1u));
  float behind_distance_m = 0.0f;
  while (behind_progress > 0.0f
      && behind_distance_m <= lookbehind_distance_m) {
    const TinyMpcPathSample behind = tinyMpcProgressPathSample(
        path, behind_progress);
    if (behind.curvature_magnitude_per_m > 1.0e-6f) {
      allowed_speed_mps = fminf(allowed_speed_mps, sqrtf(
          maximum_centripetal_acceleration_mps2
          / behind.curvature_magnitude_per_m));
    }
    const uint32_t previous_segment = (uint32_t)ceilf(behind_progress) - 1u;
    behind_distance_m += tinyMpcPathSegmentLength(path, previous_segment);
    behind_progress = (float)previous_segment;
  }
  for (float distance_m = 0.0f;
       distance_m <= lookahead_distance_m + 0.5f * sample_distance_m;
       distance_m += sample_distance_m) {
    const float ahead_progress = tinyMpcProgressPathAdvance(
        path, progress, distance_m);
    const TinyMpcPathSample ahead = tinyMpcProgressPathSample(
        path, ahead_progress);
    if (ahead.curvature_magnitude_per_m <= 1.0e-6f) {
      continue;
    }
    const float curve_speed_squared =
        maximum_centripetal_acceleration_mps2
        / ahead.curvature_magnitude_per_m;
    const float braking_speed_mps = sqrtf(fmaxf(
        curve_speed_squared
            + 2.0f * maximum_deceleration_mps2 * distance_m,
        0.0f));
    allowed_speed_mps = fminf(allowed_speed_mps, braking_speed_mps);
  }
  return tinyMpcPathClamp(
      allowed_speed_mps / proposed_speed_mps, 0.0f, 1.0f);
}

#ifdef __cplusplus
}
#endif

#endif
