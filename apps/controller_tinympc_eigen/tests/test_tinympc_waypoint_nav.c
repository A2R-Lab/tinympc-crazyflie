#include "tinympc_waypoint_nav.h"

#include <assert.h>
#include <math.h>

int main(void) {
  const TinyMpcWaypoint route[] = {
    {0.0f, 0.0f, 1.0f, 0.0f},
    {1.0f, 0.0f, 1.0f, 1.5707963f},
    {1.0f, 1.0f, 1.0f, 3.1415926f},
  };
  TinyMpcWaypointNavigator navigator;
  tinyMpcWaypointNavigatorInit(&navigator, route, 3, 0.25f);
  assert(tinyMpcWaypointPositionReached(
      &navigator, (TinyMpcWaypoint){0.10f, 0.0f, 1.0f, 0.0f}));
  assert(!tinyMpcWaypointPositionReached(
      &navigator, (TinyMpcWaypoint){0.30f, 0.0f, 1.0f, 0.0f}));
  assert(navigator.current == 0 && !navigator.complete);
  assert(!tinyMpcWaypointNavigatorUpdate(
      &navigator, (TinyMpcWaypoint){0.26f, 0.0f, 1.0f, 0.0f}));
  assert(navigator.current == 0);
  assert(tinyMpcWaypointNavigatorUpdate(
      &navigator, (TinyMpcWaypoint){0.20f, 0.0f, 1.0f, 0.0f}));
  assert(navigator.current == 1 && navigator.reached_count == 1);
  assert(!tinyMpcWaypointNavigatorUpdate(
      &navigator, (TinyMpcWaypoint){0.0f, 0.0f, 1.0f, 0.0f}));
  assert(tinyMpcWaypointNavigatorUpdate(
      &navigator, (TinyMpcWaypoint){1.0f, 0.0f, 1.0f, 0.0f}));
  assert(tinyMpcWaypointNavigatorUpdate(
      &navigator, (TinyMpcWaypoint){1.0f, 1.0f, 1.20f, 0.0f}));
  assert(navigator.complete && navigator.reached_count == 3);
  assert(tinyMpcWaypointNavigatorTarget(&navigator) == &route[2]);

  tinyMpcWaypointNavigatorInit(&navigator, route, 0, 0.25f);
  assert(navigator.complete);
  assert(tinyMpcWaypointNavigatorTarget(&navigator) == 0);

  assert(tinyMpcWrapAngle(3.5f) < -2.78f);
  assert(tinyMpcWrapAngle(-3.5f) > 2.78f);
  const float across_positive_wrap = tinyMpcMoveAngleToward(3.10f, -3.10f, 0.02f);
  assert(across_positive_wrap > 3.11f && across_positive_wrap < 3.13f);
  const float across_negative_wrap = tinyMpcMoveAngleToward(-3.10f, 3.10f, 0.02f);
  assert(across_negative_wrap < -3.11f && across_negative_wrap > -3.13f);
  assert(tinyMpcMoveAngleToward(0.0f, 0.01f, 0.02f) == 0.01f);
  assert(tinyMpcYawDeviationAvoidingPi(-1.8f, 2.5f) > 3.14159265f);
  assert(tinyMpcYawDeviationAvoidingPi(1.8f, -2.5f) < -3.14159265f);
  assert(fabsf(tinyMpcYawDeviationAvoidingPi(0.5f, 1.0f) - 0.5f) < 1e-5f);
  assert(tinyMpcWaypointYawReached(&route[1], 1.40f, 0.20f));
  assert(!tinyMpcWaypointYawReached(&route[1], 1.30f, 0.20f));
  assert(tinyMpcWaypointYawReached(&route[2], -3.10f, 0.10f));
  assert(!tinyMpcWaypointYawReached(0, 0.0f, 0.20f));
  return 0;
}
