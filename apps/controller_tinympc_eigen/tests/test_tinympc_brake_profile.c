#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/tinympc_brake_profile.h"

static void near(float actual, float expected, float tolerance)
{
  if (fabsf(actual - expected) > tolerance) { fprintf(stderr, "actual=%.9g expected=%.9g tol=%.9g\n", actual, expected, tolerance); }
  assert(fabsf(actual - expected) <= tolerance);
}

static float stopping_distance(float v)
{
  return v > 1.0f ? (v * v - 1.0f) / 12.0f + 1.0f / 9.0f
                  : v * sqrtf(v) / 9.0f;
}

int main(void)
{
  /* Crossing the taper boundary within a step integrates both pieces. */
  tinympcBrakeProfile p = {2.0f, 4.0f};
  tinympcBrakeProfileStep(&p, 0.5f);
  near(p.v, 1.0f, 1e-6f);
  near(p.s, 3.25f, 1e-6f);
  tinympcBrakeProfileStep(&p, 1.0f / 6.0f);
  near(p.v, 0.25f, 1e-6f);
  near(p.s, 3.25f + 7.0f / 72.0f, 1e-6f);
  tinympcBrakeProfileStep(&p, 1.0f);
  near(p.v, 0.0f, 0.0f);
  near(p.s, 2.0f + stopping_distance(4.0f), 1e-6f);
  const float endpoint = p.s;
  tinympcBrakeProfileStep(&p, 10.0f);
  assert(p.v == 0.0f && p.s == endpoint);

  const float speeds[] = {0.0f, 0.001f, 0.25f, 1.0f, 1.01f, 3.0f, 12.0f};
  for (unsigned k = 0; k < sizeof(speeds) / sizeof(speeds[0]); ++k) {
    p = (tinympcBrakeProfile){0.0f, speeds[k]};
    for (unsigned i = 0; i < 4000; ++i) {
      const tinympcBrakeProfile before = p;
      tinympcBrakeProfileStep(&p, 0.001f);
      assert(p.v >= 0.0f && p.v <= before.v);
      assert(p.s >= before.s);
      assert(before.v - p.v <= 0.006002f);
    }
    assert(p.v == 0.0f);
    near(p.s, stopping_distance(speeds[k]), 6e-4f);
  }

  /* Preview and live stepping agree despite different timestep partitions. */
  const float times[] = {0.02f, 0.49f, 0.51f, 0.7f, 0.84f, 2.0f};
  for (unsigned i = 0; i < sizeof(times) / sizeof(times[0]); ++i) {
    tinympcBrakeProfile whole = {0.7f, 4.0f};
    tinympcBrakeProfile split = whole;
    tinympcBrakeProfileStep(&whole, times[i]);
    for (unsigned j = 0; j < 100; ++j) {
      tinympcBrakeProfileStep(&split, times[i] / 100.0f);
    }
    near(whole.s, split.s, 2e-5f);
    near(whole.v, split.v, 2e-5f);
  }

  p = (tinympcBrakeProfile){1.0f, 3.0f};
  const float invalid_dt[] = {0.0f, -1.0f, NAN, INFINITY};
  for (unsigned i = 0; i < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++i) {
    tinympcBrakeProfileStep(&p, invalid_dt[i]);
    assert(p.s == 1.0f && p.v == 3.0f);
  }
  tinympcBrakeProfileStep(NULL, 0.01f);
  p.v = -1.0f;
  tinympcBrakeProfileStep(&p, 0.01f);
  assert(p.s == 1.0f && p.v == -1.0f);
  p.v = NAN;
  tinympcBrakeProfileStep(&p, 0.01f);
  assert(p.s == 1.0f && isnan(p.v));
  p = (tinympcBrakeProfile){NAN, 3.0f};
  tinympcBrakeProfileStep(&p, 0.01f);
  assert(isnan(p.s) && p.v == 3.0f);

  puts("brake profile tests passed");
  return 0;
}
