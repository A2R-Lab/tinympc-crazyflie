#include <assert.h>
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

static const float crossing[][3] = {
    {0.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 0.0f},
    {2.0f, 0.0f, 0.0f},
    {1.0f, -1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {-1.0f, 1.0f, 0.0f},
    {-2.0f, 0.0f, 0.0f},
};

static bool near(float actual, float expected, float tolerance) {
  return fabsf(actual - expected) <= tolerance;
}

static TinyMpcProgressPath makePath(
    const float *data, uint16_t count, uint16_t forward) {
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, data, 3u, count, 1u, forward, 0.10f, 0.40f, 1.0f, 2.0f, 0.10f);
  return path;
}

static void testProjectionIsFractionalAndMonotonic(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 2u);
  assert(near(tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){0.45f, 0.2f, 0.0f}), 0.45f, 1.0e-5f));
  assert(near(tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){0.20f, 0.0f, 0.0f}), 0.45f, 1.0e-5f));
  assert(near(tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){1.60f, 0.0f, 0.0f}), 1.60f, 1.0e-5f));
}

static void testLocalWindowDoesNotJumpAtCrossing(void) {
  TinyMpcProgressPath path = makePath(&crossing[0][0], 7u, 2u);
  path.progress = 0.20f;
  tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){0.01f, 0.01f, 0.0f});
  assert(path.progress < 1.0f);
}

static void testAdvanceUsesArcLength(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 2u);
  assert(near(tinyMpcProgressPathAdvance(&path, 0.5f, 1.25f), 1.75f, 1.0e-5f));
  assert(near(tinyMpcProgressPathAdvance(&path, 2.8f, 1.0f), 3.0f, 1.0e-5f));
}

static void testProjectionAdvanceIsArcLengthLimited(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  path.maximum_projection_advance_m = 0.25f;
  tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f});
  assert(near(path.progress, 0.25f, 1.0e-5f));
}

static void testProjectionAdvanceCanUseFiniteEffectivelyUnboundedLimit(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  path.maximum_projection_advance_m = 1.0e6f;
  tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){2.0f, 0.0f, 0.0f});
  assert(near(path.progress, 2.0f, 1.0e-5f));
}

static void testCurvatureSlowsCorner(void) {
  TinyMpcProgressPath path = makePath(&corner[0][0], 4u, 2u);
  const TinyMpcPathSample straight = tinyMpcProgressPathSample(&path, 2.2f);
  const TinyMpcPathSample turn = tinyMpcProgressPathSample(&path, 0.8f);
  assert(fabsf(turn.curvature_per_m) > 1.0f);
  assert(turn.speed_mps < straight.speed_mps);
  assert(turn.speed_mps >= path.minimum_speed_mps);
}

static void testEqualSpeedBoundsDisableCurvatureScheduling(void) {
  TinyMpcProgressPath path;
  tinyMpcProgressPathInit(
      &path, &corner[0][0], 3u, 4u, 1u, 2u,
      0.12f, 0.12f, 1.0f, 2.0f, 0.10f);
  const TinyMpcPathSample straight = tinyMpcProgressPathSample(&path, 2.2f);
  const TinyMpcPathSample turn = tinyMpcProgressPathSample(&path, 0.8f);
  assert(near(straight.speed_mps, 0.12f, 1.0e-6f));
  assert(near(turn.speed_mps, 0.12f, 1.0e-6f));
}

static void testTerminalProjectionCompletes(void) {
  TinyMpcProgressPath path = makePath(&line[0][0], 4u, 3u);
  path.progress = 2.5f;
  tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){3.0f, 0.0f, 0.0f});
  assert(path.complete);
  assert(near(path.progress, 3.0f, 1.0e-5f));
}

static void testClosedPathStartDoesNotCompleteAtDuplicateTerminal(void) {
  TinyMpcProgressPath path = makePath(&crossing[0][0], 5u, 2u);
  tinyMpcProgressPathProject(
      &path, (TinyMpcPathPoint){0.0f, 0.0f, 0.0f});
  assert(!path.complete);
}

int main(void) {
  testProjectionIsFractionalAndMonotonic();
  testLocalWindowDoesNotJumpAtCrossing();
  testAdvanceUsesArcLength();
  testProjectionAdvanceIsArcLengthLimited();
  testProjectionAdvanceCanUseFiniteEffectivelyUnboundedLimit();
  testCurvatureSlowsCorner();
  testEqualSpeedBoundsDisableCurvatureScheduling();
  testTerminalProjectionCompletes();
  testClosedPathStartDoesNotCompleteAtDuplicateTerminal();
  puts("tinympc_progress_path tests passed");
  return 0;
}
