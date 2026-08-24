#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../src/tinympc_frenet_error.h"

static void assert_near(float actual, float expected, float tolerance) {
  assert(fabsf(actual - expected) <= tolerance);
}

int main(void) {
  const float reference[12] = {
      0.31f, -0.22f, 0.08f,
      -0.071f, 0.018f, 0.243f,
      0.82f, 0.57f, -0.03f,
      0.02f, -0.18f, 1.31f,
  };
  const float error[12] = {
      0.07f, -0.04f, 0.03f,
      0.02f, -0.015f, 0.025f,
      0.11f, -0.08f, 0.04f,
      0.07f, -0.05f, 0.09f,
  };
  float actual[12];
  float recovered[12];
  tinyMpcFrenetErrorDecode(error, reference, actual);
  tinyMpcFrenetErrorEncode(actual, reference, recovered);
  for (int index = 0; index < 12; ++index) {
    assert_near(recovered[index], error[index], 2.0e-6f);
  }

  tinyMpcFrenetErrorEncode(reference, reference, recovered);
  for (int index = 0; index < 12; ++index) {
    assert_near(recovered[index], 0.0f, 2.0e-6f);
  }

  const float anchor[3] = {0.3f, -0.4f, 1.2f};
  float canonical[12];
  const float speed = 1.0f;
  const float yaw_rate = 1.0f / 0.75f;
  tinyMpcFrenetCanonicalCircleState(
      anchor, 0.0f, -0.1349181f, 0.0504593891f,
      speed, yaw_rate, 0.3f, canonical);
  const float theta = yaw_rate * 0.3f;
  assert_near(canonical[0], anchor[0] + 0.75f * sinf(theta), 2.0e-6f);
  assert_near(
      canonical[1], anchor[1] + 0.75f * (1.0f - cosf(theta)), 2.0e-6f);
  assert_near(canonical[2], anchor[2], 2.0e-6f);
  assert_near(canonical[6], speed * cosf(theta), 2.0e-6f);
  assert_near(canonical[7], speed * sinf(theta), 2.0e-6f);
  assert_near(canonical[8], 0.0f, 2.0e-6f);
  float canonical_error[12];
  tinyMpcFrenetErrorEncode(canonical, canonical, canonical_error);
  for (int index = 0; index < 12; ++index) {
    assert_near(canonical_error[index], 0.0f, 2.0e-6f);
  }
  puts("tinympc_frenet_error tests passed");
  return 0;
}
