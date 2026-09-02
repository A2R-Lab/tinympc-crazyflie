#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "tinympc_rate_command_guard.h"

static bool near(float actual, float expected) {
  return fabsf(actual - expected) <= 1.0e-6f;
}

int main(void) {
  TinyMpcRateCommandBounds bounds = tinyMpcRateReferenceTrimBounds(
      -4.0f, 4.0f, 0.60f, 0.65f);
  assert(near(bounds.lower, -0.05f));
  assert(near(bounds.upper, 1.25f));

  bounds = tinyMpcRateIntersectSlewBounds(bounds, 0.0f, 0.30f);
  assert(near(bounds.lower, -0.05f));
  assert(near(bounds.upper, 0.30f));

  bounds = tinyMpcRateReferenceTrimBounds(
      -0.3055578f, 0.2244422f, 0.0f, 0.08f);
  bounds = tinyMpcRateIntersectSlewBounds(bounds, 0.0f, 0.025f);
  assert(near(bounds.lower, -0.025f));
  assert(near(bounds.upper, 0.025f));

  bounds = tinyMpcRateReferenceTrimBounds(
      -4.0f, 4.0f, 1.5f, 0.2f);
  bounds = tinyMpcRateIntersectSlewBounds(bounds, 0.0f, 0.3f);
  assert(near(bounds.lower, 0.3f));
  assert(near(bounds.upper, 0.3f));

  puts("tinympc rate command guard tests passed");
  return 0;
}
