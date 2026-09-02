#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "tinympc_progress_path.h"

static const float line[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f},
    {3.0f, 0.0f, 0.0f},
};

static const float corner[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {1.0f, 2.0f, 0.0f},
};

static const float braking_corner[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f},
    {3.0f, 0.0f, 0.0f},
    {3.0f, 1.0f, 0.0f},
    {3.0f, 2.0f, 0.0f},
};

/* The final branch passes through the route start. It reproduces the closed
 * path shortcut that an unconstrained nearest-segment projection can take. */
static const float shortcut[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f},
};

static const float closed_square[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
};

static bool near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static TinyMpcProgressPath makePath(
    const float *data, uint16_t count, uint16_t forward) {
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, data, 3u, count, 1u, forward, 0.10f, 0.40f, 1.0f,
      0.001f, 0.040f, 0.10f);
  return path;
}

static void initializeVehicle(
    TinyMpcProgressPath *path, TinyMpcPathPoint vehicle) {
  tinyMpcProgressPathUpdate(path, vehicle, 0.0f);
}

static void testStationaryAndCrossTrackMotionCannotAdvanceMeasuredPhase(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  for (int i = 0; i < 100; ++i) {
    const float y = (i & 1) == 0 ? 0.25f : -0.25f;
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){0.0f, y, 0.0f}, 0.0f);
  }
  assert(path.cumulative_forward_displacement_m <= 1.0e-6f);
  assert(path.cumulative_measured_advance_m <= path.progress_tolerance_m);
  assert(path.measured_progress <= 0.00101f);
}

static void testMeasuredPhaseCannotAdvanceFasterThanForwardTravel(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){0.20f, 0.0f, 0.0f}, 0.0f);
  assert(near(path.measured_progress, 0.20f, 1.0e-5f));
  assert(path.cumulative_measured_advance_m <=
      path.cumulative_forward_displacement_m + path.progress_tolerance_m
      + 1.0e-5f);
  const float before = path.measured_progress;
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){0.10f, 0.0f, 0.0f}, 0.0f);
  assert(near(path.measured_progress, before, 1.0e-6f));
  assert(!path.projection_bound_violation);
}

static void testShortcutProjectionIsPhysicallyBounded(void) {
  TinyMpcProgressPath path = makePath(&shortcut[0][0], 6u, 5u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){-0.01f, 0.0f, 0.0f}, 0.0f);
  assert(path.last_projection_candidate_advance_m > 3.5f);
  assert(path.last_forward_displacement_m <= 1.0e-6f);
  assert(path.cumulative_measured_advance_m <= 0.00101f);
  assert(path.projection_limited);
  assert(!path.projection_bound_violation);
}

static void testCommandedTargetUsesArcLengthStepAndIsMonotonic(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f}, 0.0f);
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f}, 0.020f);
  assert(near(path.last_commanded_advance_m, 0.020f, 1.0e-5f));
  assert(!path.command_step_violation);
  const float before = path.progress;
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){0.5f, 0.0f, 0.0f}, 0.020f);
  assert(path.progress >= before);
  assert(path.last_commanded_advance_m <= 0.02001f);
}

static void testMeasuredOverspeedRebasesReferenceBeforeCommandAdvance(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});

  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){0.50f, 0.0f, 0.0f}, 0.020f);

  assert(near(path.measured_progress, 0.50f, 1.0e-5f));
  assert(near(path.last_reference_rebase_m, 0.50f, 1.0e-5f));
  assert(near(path.last_commanded_advance_m, 0.020f, 1.0e-5f));
  assert(near(path.progress, 0.52f, 1.0e-5f));
  assert(path.progress >= path.measured_progress);
  assert(path.last_phase_lag_m <= 1.0e-6f);
  assert(!path.command_step_violation);
  assert(!path.lead_bound_violation);
  assert(!path.lag_bound_violation);
}

static void testTargetNeverExceedsOneStepPlusToleranceLead(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  for (int i = 0; i < 20; ++i) {
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f}, 0.040f);
    assert(path.last_phase_lead_m <= path.last_phase_lead_bound_m + 1.0e-5f);
    assert(path.last_phase_lead_bound_m <= 0.04101f);
    assert(!path.lead_bound_violation);
  }
}

static void testDisabledTargetLeadBoundAdvancesWhileStationary(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathSetTargetLeadBound(&path, false);
  for (int i = 0; i < 10; ++i) {
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f}, 0.040f);
  }
  assert(near(path.progress, 0.40f, 1.0e-5f));
  assert(path.measured_progress <= path.progress_tolerance_m + 1.0e-5f);
  assert(path.last_phase_lead_m >
      path.maximum_command_advance_m + path.progress_tolerance_m);
  assert(!path.lead_bound_violation);
  assert(!path.complete);
}

static void testExplicitHalfMeterTargetLeadBound(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathSetMaximumTargetLead(&path, 0.5f);
  for (int i = 0; i < 50; ++i) {
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f}, 0.040f);
    assert(path.last_phase_lead_m <= 0.50001f);
    assert(near(path.last_phase_lead_bound_m, 0.5f, 1.0e-6f));
    assert(!path.lead_bound_violation);
  }
  assert(path.last_phase_lead_m >= 0.499f);
  assert(!path.complete);
}

static void testCleanRejoinGeometricCatchupIsBounded(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 1u);
  tinyMpcProgressPathSetMaximumTargetLead(&path, 0.5f);
  initializeVehicle(&path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f});
  assert(tinyMpcProgressPathScheduleGeometricCatchup(
      &path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f}, 3u,
      0.25f, 0.02f));
  assert(near(path.geometric_catchup_target_progress, 2.0f, 1.0e-6f));
  for (int update = 0; update < 10; ++update) {
    const float before = path.measured_progress;
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f}, 0.04f);
    assert(path.measured_progress - before <= 0.02001f);
    assert(path.last_geometric_catchup_m <= 0.02001f);
    assert(path.last_phase_lead_m <= 0.50001f);
    assert(!path.projection_bound_violation);
    assert(!path.lead_bound_violation);
  }
  assert(near(path.measured_progress, 0.201f, 1.0e-5f));
  assert(near(path.cumulative_geometric_catchup_m, 0.20f, 1.0e-5f));
}

static void testDistantRejoinCannotScheduleGeometricCatchup(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 1u);
  assert(!tinyMpcProgressPathScheduleGeometricCatchup(
      &path, (TinyMpcPathPoint){2.0f, 0.40f, 0.0f}, 3u,
      0.25f, 0.02f));
  assert(!path.geometric_catchup_active);
  assert(near(path.measured_progress, 0.0f, 1.0e-7f));
}

static void testGeometricCatchupTargetDoesNotRatchetWithMotion(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  assert(tinyMpcProgressPathScheduleGeometricCatchup(
      &path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f}, 3u,
      0.25f, 0.02f));
  assert(near(path.geometric_catchup_target_progress, 2.0f, 1.0e-6f));
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){2.5f, 0.0f, 0.0f}, 0.0f);
  assert(near(path.geometric_catchup_target_progress, 2.0f, 1.0e-6f));
  assert(!path.geometric_catchup_active);
}

static void testClosedRouteCatchupCannotSelectNextLapCopy(void) {
  TinyMpcProgressPath path;
  tinyMpcProgressPathInitLaps(
      &path, &closed_square[0][0], 3u, 5u, 1u, 4u,
      0.10f, 0.40f, 1.0f, 0.001f, 0.040f, 0.10f, 3u);
  path.measured_progress = 0.10f;
  path.progress = 0.10f;
  assert(!tinyMpcProgressPathScheduleGeometricCatchup(
      &path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f}, 4u,
      0.25f, 0.02f));
  assert(!path.geometric_catchup_active);
  assert(near(path.measured_progress, 0.10f, 1.0e-7f));
}

static void testOversizedCommandIsClampedToConfiguredStep(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f}, 0.0f);
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f}, 1.0f);
  assert(near(path.last_commanded_request_m, 0.040f, 1.0e-6f));
  assert(path.last_commanded_advance_m <= 0.04001f);
  assert(!path.command_step_violation);
}

static void testClosedPathShortcutCannotComplete(void) {
  TinyMpcProgressPath path = makePath(&shortcut[0][0], 6u, 5u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  for (int i = 0; i < 20; ++i) {
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){-0.01f, 0.0f, 0.0f}, 0.040f);
  }
  assert(!path.complete);
  assert(path.measured_progress < 1.0f);
  assert(path.progress < 1.0f);
}

static void testGenuineBoundedTraversalCompletesAtTerminal(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  for (int i = 1; i <= 100 && !path.complete; ++i) {
    const float x = fminf((float)i * 0.10f, 3.0f);
    tinyMpcProgressPathUpdate(
        &path, (TinyMpcPathPoint){x, 0.0f, 0.0f}, 0.040f);
  }
  assert(path.complete);
  assert(near(path.measured_progress, 3.0f, 1.0e-5f));
  assert(near(path.progress, 3.0f, 1.0e-5f));
  assert(!path.projection_bound_violation);
  assert(!path.command_step_violation);
  assert(!path.lead_bound_violation);
  const TinyMpcPathSample terminal = tinyMpcProgressPathSample(
      &path, path.progress);
  assert(near(terminal.speed_mps, 0.0f, 1.0e-6f));
  assert(near(terminal.curvature_per_m, 0.0f, 1.0e-6f));
  assert(near(terminal.curvature_magnitude_per_m, 0.0f, 1.0e-6f));
  assert(near(terminal.curvature_vector_per_m.x, 0.0f, 1.0e-6f));
  assert(near(terminal.curvature_vector_per_m.y, 0.0f, 1.0e-6f));
  assert(near(terminal.curvature_vector_per_m.z, 0.0f, 1.0e-6f));
}

static void testSubToleranceTerminalKnotTailCompletes(void) {
  static const float dense_tail[][3] = {
      {0.0f, 0.0f, 0.0f},
      {0.9995f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},
  };
  TinyMpcProgressPath path = makePath(&dense_tail[0][0], 3u, 2u);
  initializeVehicle(&path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f});
  path.measured_progress = 1.0f;
  path.progress = 2.0f;
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){1.0f, 0.0f, 0.0f}, 0.0f);
  assert(path.complete);
  assert(near(path.measured_progress, 2.0f, 1.0e-6f));
  assert(near(path.progress, 2.0f, 1.0e-6f));
}

static void testLateralOffsetRetainsOnPathRecoveryReference(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  initializeVehicle(&path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  tinyMpcProgressPathUpdate(
      &path, (TinyMpcPathPoint){0.0f, 0.75f, 0.0f}, 0.020f);
  const TinyMpcPathSample recovery = tinyMpcProgressPathSample(
      &path, path.progress);
  assert(near(recovery.position.y, 0.0f, 1.0e-6f));
  assert(near(recovery.position.z, 0.0f, 1.0e-6f));
  assert(near(recovery.tangent.x, 1.0f, 1.0e-6f));
  assert(path.progress <= 0.02101f);
}

static void testAdvanceAndDistanceUseArcLength(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 2u);
  assert(near(tinyMpcProgressPathAdvance(&path, 0.5f, 1.25f),
              1.75f, 1.0e-5f));
  assert(near(tinyMpcProgressPathDistance(&path, 0.5f, 1.75f),
              1.25f, 1.0e-5f));
  assert(near(tinyMpcProgressPathAdvance(&path, 2.8f, 1.0f),
              3.0f, 1.0e-5f));
}

static void testCurvatureScheduling(void) {
  TinyMpcProgressPath path = makePath(&corner[0][0], 4u, 2u);
  const TinyMpcPathSample straight = tinyMpcProgressPathSample(&path, 2.2f);
  const TinyMpcPathSample turn = tinyMpcProgressPathSample(&path, 0.8f);
  assert(fabsf(turn.curvature_per_m) > 1.0f);
  assert(turn.speed_mps < straight.speed_mps);
  assert(turn.speed_mps >= path.minimum_speed_mps);
}

static void testThreeDimensionalCurvatureVector(void) {
  static const float spatial_turn[][3] = {
      {0.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},
      {1.0f, 1.0f, 1.0f},
      {1.0f, 2.0f, 2.0f},
  };
  TinyMpcProgressPath path = makePath(&spatial_turn[0][0], 4u, 2u);
  const TinyMpcPathSample turn = tinyMpcProgressPathSample(&path, 0.8f);
  const float vector_norm = hypotf(
      hypotf(turn.curvature_vector_per_m.x,
             turn.curvature_vector_per_m.y),
      turn.curvature_vector_per_m.z);
  assert(turn.curvature_magnitude_per_m > 0.5f);
  assert(near(vector_norm, turn.curvature_magnitude_per_m, 1.0e-5f));
  assert(turn.curvature_vector_per_m.z > 0.0f);
}

static void testEqualSpeedBoundsDisableCurvatureScheduling(void) {
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, &corner[0][0], 3u, 4u, 1u, 2u,
      0.12f, 0.12f, 1.0f, 0.001f, 0.0024f, 0.10f);
  const TinyMpcPathSample straight = tinyMpcProgressPathSample(&path, 2.2f);
  const TinyMpcPathSample turn = tinyMpcProgressPathSample(&path, 0.8f);
  assert(near(straight.speed_mps, 0.12f, 1.0e-6f));
  assert(near(turn.speed_mps, 0.12f, 1.0e-6f));
}

static void testTerminalTinySegmentRetainsNearestValidCurvature(void) {
  static const float terminal_tail[][3] = {
      {0.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},
      {1.0f, 1.0f, 0.0f},
      {1.0f, 1.0000001f, 0.0f},
  };
  TinyMpcProgressPath path = makePath(&terminal_tail[0][0], 4u, 2u);
  const TinyMpcPathSample terminal_approach =
      tinyMpcProgressPathSample(&path, 2.9f);
  assert(near(
      terminal_approach.curvature_per_m,
      1.57079632679f, 1.0e-5f));
}

static void testInterpolatedCircleTangentsProduceContinuousYawRate(void) {
  enum { circle_segments = 128 };
  float circle[circle_segments + 1][3];
  const float radius_m = 0.75f;
  const float two_pi = 6.28318530717958647692f;
  for (int index = 0; index <= circle_segments; ++index) {
    const float angle = two_pi * (float)index / (float)circle_segments;
    circle[index][0] = radius_m * sinf(angle);
    circle[index][1] = radius_m * (1.0f - cosf(angle));
    circle[index][2] = 0.0f;
  }
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, &circle[0][0], 3u, circle_segments + 1u, 2u, 8u,
      1.0f, 1.0f, radius_m, 0.001f, 0.02f, 0.10f);
  float progress = 16.25f;
  for (int knot = 0; knot < 20; ++knot) {
    const TinyMpcPathSample current = tinyMpcProgressPathSample(
        &path, progress);
    const float next_progress = tinyMpcProgressPathAdvance(
        &path, progress, 0.02f);
    const TinyMpcPathSample next = tinyMpcProgressPathSample(
        &path, next_progress);
    float yaw_step = atan2f(next.tangent.y, next.tangent.x)
        - atan2f(current.tangent.y, current.tangent.x);
    while (yaw_step > 3.14159265358979323846f) {
      yaw_step -= two_pi;
    }
    while (yaw_step < -3.14159265358979323846f) {
      yaw_step += two_pi;
    }
    const float yaw_rate_rad_s = yaw_step / 0.02f;
    assert(near(yaw_rate_rad_s, 1.0f / radius_m, 0.002f));
    progress = next_progress;
  }
}

static void testConservativeTangentialAccelerationReference(void) {
  assert(near(tinyMpcProgressTangentialAcceleration(1.5f, 1.5f),
              0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(1.5f, 1.8f),
              0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(0.0f, -1.0f),
              0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(1.5f, 1.4f),
              0.2857143f, 1.0e-6f));
  assert(near(tinyMpcProgressTangentialAcceleration(1.5f, 1.0f),
              0.5f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(2.0f, -10.0f),
              0.5f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(NAN, 0.0f),
              0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(1.0f, NAN),
              0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTangentialAcceleration(INFINITY, 0.0f),
              0.0f, 1.0e-7f));
  assert(isfinite(tinyMpcProgressTangentialAcceleration(FLT_MAX, -FLT_MAX)));
  assert(near(tinyMpcProgressTangentialAcceleration(FLT_MAX, -FLT_MAX),
              0.5f, 1.0e-7f));
}

static void testEntrySpeedRampRespectsConfiguredAcceleration(void) {
  float speed_mps = 0.0f;
  const float dt_s = 0.02f;
  const float maximum_acceleration_mps2 = 0.25f;
  for (int step = 0; step < 400; ++step) {
    const float before = speed_mps;
    speed_mps = tinyMpcProgressRampSpeed(
        speed_mps, 1.0f, maximum_acceleration_mps2, dt_s);
    assert(speed_mps >= before);
    assert(speed_mps - before
        <= maximum_acceleration_mps2 * dt_s + 1.0e-6f);
  }
  assert(near(speed_mps, 1.0f, 2.0e-5f));
  assert(near(tinyMpcProgressRampSpeed(1.0f, 1.0f, 0.25f, 0.02f),
              1.0f, 1.0e-7f));
  assert(near(tinyMpcProgressRampSpeed(NAN, 1.0f, 0.25f, 0.02f),
              0.0f, 1.0e-7f));
}

static void testCentripetalSpeedScaleCapsRewardedSpeed(void) {
  const float scale = tinyMpcProgressCentripetalSpeedScale(
      2.6f, 3.193526f, 12.0f);
  const float capped_speed = 2.6f * scale;
  assert(scale > 0.0f && scale < 1.0f);
  assert(near(capped_speed, sqrtf(12.0f / 3.193526f), 1.0e-6f));
  assert(capped_speed * capped_speed * 3.193526f <= 12.00001f);
  assert(near(tinyMpcProgressCentripetalSpeedScale(
      2.6f, 0.0f, 12.0f), 1.0f, 1.0e-7f));
  assert(near(tinyMpcProgressCentripetalSpeedScale(
      1.0f, 0.25f, 12.0f), 1.0f, 1.0e-7f));
  assert(near(tinyMpcProgressCentripetalSpeedScale(
      NAN, 1.0f, 12.0f), 0.0f, 1.0e-7f));
  assert(near(tinyMpcProgressCentripetalSpeedScale(
      1.0f, -1.0f, 12.0f), 0.0f, 1.0e-7f));
}

static void testCentripetalBrakingEnvelopeAnticipatesTurn(void) {
  TinyMpcProgressPath path = makePath(&braking_corner[0][0], 6u, 5u);
  const float local_scale = tinyMpcProgressCentripetalSpeedScale(
      2.6f, tinyMpcProgressPathSample(&path, 1.5f)
          .curvature_magnitude_per_m, 6.0f);
  const float braking_scale = tinyMpcProgressCentripetalBrakingSpeedScale(
      &path, 1.5f, 2.6f, 6.0f, 1.5f, 0.0f, 2.0f, 0.05f);
  assert(near(local_scale, 1.0f, 1.0e-6f));
  assert(braking_scale > 0.0f && braking_scale < local_scale);
  const float recovery_scale = tinyMpcProgressCentripetalBrakingSpeedScale(
      &path, 3.5f, 2.6f, 6.0f, 1.5f, 2.0f, 0.0f, 0.05f);
  assert(recovery_scale > 0.0f && recovery_scale < 1.0f);
  assert(near(tinyMpcProgressCentripetalBrakingSpeedScale(
      NULL, 0.0f, 2.6f, 6.0f, 1.5f, 0.0f, 2.0f, 0.05f),
      0.0f, 1.0e-7f));
}

static void testTerminalSpeedEnvelopeAndBidirectionalSlew(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  const float far_limit = tinyMpcProgressTerminalSpeedLimit(
      &path, 0.0f, 1.5f);
  const float near_limit = tinyMpcProgressTerminalSpeedLimit(
      &path, 2.5f, 1.5f);
  const float terminal_limit = tinyMpcProgressTerminalSpeedLimit(
      &path, 3.0f, 1.5f);
  assert(far_limit > near_limit);
  assert(near_limit > terminal_limit);
  assert(near(terminal_limit, 0.0f, 1.0e-7f));

  float speed = 1.0f;
  speed = tinyMpcProgressSlewSpeed(speed, 0.0f, 0.5f, 1.5f, 0.02f);
  assert(near(speed, 0.97f, 1.0e-6f));
  speed = tinyMpcProgressSlewSpeed(speed, 2.0f, 0.5f, 1.5f, 0.02f);
  assert(near(speed, 0.98f, 1.0e-6f));
  assert(near(tinyMpcProgressSlewSpeed(
      NAN, 1.0f, 0.5f, 1.5f, 0.02f), 0.0f, 1.0e-7f));

  assert(near(tinyMpcProgressTerminalRewardScale(2.0f, 2.0f),
              1.0f, 1.0e-7f));
  assert(near(tinyMpcProgressTerminalRewardScale(2.0f, 0.0f),
              0.0f, 1.0e-7f));
  const float half_scale = tinyMpcProgressTerminalRewardScale(2.0f, 1.0f);
  assert(near(half_scale, 0.5f, 1.0e-7f));
}

static void testClosedRaceFinishPrecedesCooldownTerminal(void) {
  enum { segments = 64 };
  float circle[segments + 1][3];
  const float radius_m = 0.75f;
  const float two_pi = 6.28318530717958647692f;
  for (int index = 0; index <= segments; ++index) {
    const float angle = two_pi * (float)index / (float)segments;
    circle[index][0] = radius_m * sinf(angle);
    circle[index][1] = radius_m * (1.0f - cosf(angle));
    circle[index][2] = 0.0f;
  }
  TinyMpcProgressPath path;
  tinyMpcProgressPathInitLaps(
      &path, &circle[0][0], 3u, segments + 1u, 2u, 8u,
      2.9f, 2.9f, radius_m, 0.001f, 0.058f, 0.15f, 2u);
  const float race_finish_limit = tinyMpcProgressTerminalSpeedLimit(
      &path, (float)segments, 1.5f);
  const float cooldown_limit = tinyMpcProgressTerminalSpeedLimit(
      &path, 1.875f * (float)segments, 1.5f);
  assert(race_finish_limit > 2.9f);
  assert(cooldown_limit < 2.9f);
  assert(near(tinyMpcProgressTerminalSpeedLimit(
      &path, 2.0f * (float)segments, 1.5f), 0.0f, 1.0e-6f));
}

static void testAnalyticalDerivativeDefinesUnitTangent(void) {
  const float points[][3] = {
      {0.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},
      {2.0f, 0.0f, 0.0f},
  };
  const float derivatives[][3] = {
      {0.0f, 0.0f, 0.0f},
      {0.0f, 2.0f, 0.0f},
      {0.0f, 4.0f, 0.0f},
  };
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, &points[0][0], 3u, 3u, 1u, 1u,
      0.1f, 0.2f, 0.75f, 0.001f, 0.01f, 0.15f);
  tinyMpcProgressPathSetAnalyticalDerivatives(
      &path, &derivatives[0][0], 3u);

  const TinyMpcPathSample endpoint = tinyMpcProgressPathSample(&path, 0.0f);
  const TinyMpcPathSample interior = tinyMpcProgressPathSample(&path, 1.5f);
  assert(near(endpoint.tangent.x, 0.0f, 1.0e-7f));
  assert(near(endpoint.tangent.y, 1.0f, 1.0e-7f));
  assert(near(interior.tangent.x, 0.0f, 1.0e-7f));
  assert(near(interior.tangent.y, 1.0f, 1.0e-7f));
}

static void testRepeatedClosedPathWrapAndMeasuredBounds(void) {
  enum { circle_segments = 750, lap_count = 3 };
  float circle[circle_segments + 1][3];
  const float radius_m = 0.75f;
  const float two_pi = 6.28318530717958647692f;
  for (int index = 0; index <= circle_segments; ++index) {
    const float angle = two_pi * (float)index / (float)circle_segments;
    circle[index][0] = radius_m * sinf(angle);
    circle[index][1] = radius_m * (1.0f - cosf(angle));
    circle[index][2] = 0.0f;
  }
  TinyMpcProgressPath path;
  tinyMpcProgressPathInitLaps(
      &path, &circle[0][0], 3u, circle_segments + 1u, 4u, 60u,
      1.0f, 1.0f, radius_m, 0.001f, 0.04f, 0.10f, lap_count);
  assert(path.count == circle_segments + 1u);
  assert(path.virtual_count == circle_segments * lap_count + 1u);
  assert(near(tinyMpcProgressPathTerminalProgress(&path),
              (float)(circle_segments * lap_count), 1.0e-6f));

  const TinyMpcPathSample before_wrap = tinyMpcProgressPathSample(
      &path, (float)circle_segments - 0.25f);
  const TinyMpcPathSample after_wrap = tinyMpcProgressPathSample(
      &path, (float)circle_segments + 0.25f);
  assert(hypotf(before_wrap.position.x - after_wrap.position.x,
                before_wrap.position.y - after_wrap.position.y) < 0.005f);
  assert(before_wrap.tangent.x * after_wrap.tangent.x
          + before_wrap.tangent.y * after_wrap.tangent.y > 0.999f);

  TinyMpcPathPoint vehicle = {circle[0][0], circle[0][1], circle[0][2]};
  initializeVehicle(&path, vehicle);
  float previous_measured = path.measured_progress;
  float previous_target = path.progress;
  uint16_t previous_laps = 0u;
  for (uint32_t virtual_index = 1u;
       virtual_index <= circle_segments * lap_count;
       ++virtual_index) {
    const uint32_t source_index = virtual_index == circle_segments * lap_count
        ? circle_segments : virtual_index % circle_segments;
    vehicle = (TinyMpcPathPoint){circle[source_index][0],
                                circle[source_index][1],
                                circle[source_index][2]};
    tinyMpcProgressPathUpdate(&path, vehicle, 0.04f);
    assert(path.measured_progress + 1.0e-6f >= previous_measured);
    assert(path.progress + 1.0e-6f >= previous_target);
    assert(path.cumulative_measured_advance_m
        <= path.cumulative_forward_displacement_m
            + path.progress_tolerance_m + 1.0e-5f);
    assert(path.last_phase_lead_m
        <= path.last_phase_lead_bound_m + 1.0e-5f);
    const uint16_t completed_laps = tinyMpcProgressPathCompletedLaps(
        &path, path.measured_progress);
    assert(completed_laps >= previous_laps);
    assert(tinyMpcProgressPathLapProgress(
        &path, path.measured_progress) >= 0.0f);
    assert(tinyMpcProgressPathLapProgress(
        &path, path.measured_progress) <= circle_segments + 1.0e-5f);
    previous_measured = path.measured_progress;
    previous_target = path.progress;
    previous_laps = completed_laps;
  }
  assert(path.complete);
  assert(tinyMpcProgressPathCompletedLaps(
      &path, path.measured_progress) == lap_count);
  assert(!path.projection_bound_violation);
  assert(!path.command_step_violation);
  assert(!path.lead_bound_violation);
}

int main(void) {
  testStationaryAndCrossTrackMotionCannotAdvanceMeasuredPhase();
  testMeasuredPhaseCannotAdvanceFasterThanForwardTravel();
  testShortcutProjectionIsPhysicallyBounded();
  testCommandedTargetUsesArcLengthStepAndIsMonotonic();
  testMeasuredOverspeedRebasesReferenceBeforeCommandAdvance();
  testTargetNeverExceedsOneStepPlusToleranceLead();
  testDisabledTargetLeadBoundAdvancesWhileStationary();
  testExplicitHalfMeterTargetLeadBound();
  testCleanRejoinGeometricCatchupIsBounded();
  testDistantRejoinCannotScheduleGeometricCatchup();
  testGeometricCatchupTargetDoesNotRatchetWithMotion();
  testClosedRouteCatchupCannotSelectNextLapCopy();
  testOversizedCommandIsClampedToConfiguredStep();
  testClosedPathShortcutCannotComplete();
  testGenuineBoundedTraversalCompletesAtTerminal();
  testSubToleranceTerminalKnotTailCompletes();
  testLateralOffsetRetainsOnPathRecoveryReference();
  testAdvanceAndDistanceUseArcLength();
  testCurvatureScheduling();
  testThreeDimensionalCurvatureVector();
  testEqualSpeedBoundsDisableCurvatureScheduling();
  testTerminalTinySegmentRetainsNearestValidCurvature();
  testInterpolatedCircleTangentsProduceContinuousYawRate();
  testConservativeTangentialAccelerationReference();
  testTerminalSpeedEnvelopeAndBidirectionalSlew();
  testClosedRaceFinishPrecedesCooldownTerminal();
  testEntrySpeedRampRespectsConfiguredAcceleration();
  testCentripetalSpeedScaleCapsRewardedSpeed();
  testCentripetalBrakingEnvelopeAnticipatesTurn();
  testAnalyticalDerivativeDefinesUnitTangent();
  testRepeatedClosedPathWrapAndMeasuredBounds();
  puts("tinympc_progress_path tests passed");
  return 0;
}
