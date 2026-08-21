#ifndef TINYMPC_WAYPOINT_NAV_H
#define TINYMPC_WAYPOINT_NAV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float z;
  float yaw_rad;
} TinyMpcWaypoint;

typedef struct {
  const TinyMpcWaypoint *points;
  uint16_t count;
  uint16_t current;
  uint16_t reached_count;
  float reached_radius_m;
  bool complete;
} TinyMpcWaypointNavigator;

static inline float tinyMpcWrapAngle(float angle_rad) {
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.28318530717958647692f;
  while (angle_rad > pi) {
    angle_rad -= two_pi;
  }
  while (angle_rad < -pi) {
    angle_rad += two_pi;
  }
  return angle_rad;
}

static inline float tinyMpcMoveAngleToward(
    float current_rad, float target_rad, float maximum_step_rad) {
  const float error = tinyMpcWrapAngle(target_rad - current_rad);
  if (error > maximum_step_rad) {
    return current_rad + maximum_step_rad;
  }
  if (error < -maximum_step_rad) {
    return current_rad - maximum_step_rad;
  }
  return current_rad + error;
}

/* Return a directed yaw deviation that avoids crossing the +/-pi chart seam.
 * The longer turn is intentional when current and target occupy opposite
 * sides of the seam; waypoint position remains held while this turn occurs. */
static inline float tinyMpcYawDeviationAvoidingPi(
    float current_rad, float target_rad) {
  const float half_pi = 1.57079632679489661923f;
  const float two_pi = 6.28318530717958647692f;
  float deviation = tinyMpcWrapAngle(target_rad - current_rad);
  if (current_rad < 0.0f && target_rad > half_pi && deviation < 0.0f) {
    deviation += two_pi;
  } else if (current_rad > 0.0f && target_rad < -half_pi &&
             deviation > 0.0f) {
    deviation -= two_pi;
  }
  return deviation;
}

static inline bool tinyMpcWaypointYawReached(
    const TinyMpcWaypoint *waypoint, float vehicle_yaw_rad,
    float tolerance_rad) {
  return waypoint != 0 &&
      (tinyMpcWrapAngle(waypoint->yaw_rad - vehicle_yaw_rad) <= tolerance_rad) &&
      (tinyMpcWrapAngle(waypoint->yaw_rad - vehicle_yaw_rad) >= -tolerance_rad);
}

static inline void tinyMpcWaypointNavigatorInit(
    TinyMpcWaypointNavigator *navigator,
    const TinyMpcWaypoint *points,
    uint16_t count,
    float reached_radius_m) {
  navigator->points = points;
  navigator->count = count;
  navigator->current = 0;
  navigator->reached_count = 0;
  navigator->reached_radius_m = reached_radius_m;
  navigator->complete = count == 0;
}

static inline bool tinyMpcWaypointPositionReached(
    const TinyMpcWaypointNavigator *navigator,
    TinyMpcWaypoint vehicle_position) {
  if (navigator->complete || navigator->points == 0 ||
      navigator->current >= navigator->count) {
    return false;
  }
  const TinyMpcWaypoint *target = &navigator->points[navigator->current];
  const float dx = target->x - vehicle_position.x;
  const float dy = target->y - vehicle_position.y;
  const float dz = target->z - vehicle_position.z;
  const float radius = navigator->reached_radius_m;
  return dx * dx + dy * dy + dz * dz <= radius * radius;
}

static inline bool tinyMpcWaypointNavigatorAdvance(
    TinyMpcWaypointNavigator *navigator) {
  if (navigator->complete || navigator->points == 0 ||
      navigator->current >= navigator->count) {
    return false;
  }
  ++navigator->reached_count;
  if (navigator->current + 1u < navigator->count) {
    ++navigator->current;
  } else {
    navigator->complete = true;
  }
  return true;
}

/* Advances by at most one waypoint. The active waypoint remains unchanged
 * until the vehicle enters its three-dimensional acceptance sphere. */
static inline bool tinyMpcWaypointNavigatorUpdate(
    TinyMpcWaypointNavigator *navigator,
    TinyMpcWaypoint vehicle_position) {
  if (navigator->complete || navigator->points == 0 ||
      navigator->current >= navigator->count) {
    return false;
  }
  if (!tinyMpcWaypointPositionReached(navigator, vehicle_position)) {
    return false;
  }
  return tinyMpcWaypointNavigatorAdvance(navigator);
}

static inline const TinyMpcWaypoint *tinyMpcWaypointNavigatorTarget(
    const TinyMpcWaypointNavigator *navigator) {
  if (navigator->points == 0 || navigator->count == 0 ||
      navigator->current >= navigator->count) {
    return 0;
  }
  return &navigator->points[navigator->current];
}

#ifdef __cplusplus
}
#endif

#endif
