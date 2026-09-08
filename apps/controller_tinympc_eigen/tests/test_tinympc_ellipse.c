#include <assert.h>
#include <stdio.h>
#include "../src/tinympc_ellipse.h"

static void near(float actual, float expected, float tol) {
  assert(fabsf(actual - expected) < tol);
}

int main(void) {
  TinyMpcEllipse e = {1.0f, 2.0f, 0.5f, 0.0f, 2.0f, 1.0f};
  TinyMpcEllipseSample p;
  assert(tinyMpcEllipseSampleAt(&e, 0, 0.5f, &p));
  near(p.x, 1, 1e-6f); near(p.y, 2, 1e-6f);
  near(p.vx, 0.5f, 1e-6f); near(p.vy, 0, 1e-6f);
  near(p.curvature, 0.25f, 1e-6f);
  assert(tinyMpcEllipseSampleAt(&e, TINYMPC_ELLIPSE_PI / 2, 0.5f, &p));
  near(p.x, 3, 1e-6f); near(p.y, 3, 1e-6f);
  near(p.yaw, TINYMPC_ELLIPSE_PI / 2, 1e-6f);
  near(p.curvature, 2, 1e-6f);
  assert(tinyMpcEllipseSampleAt(&e, TINYMPC_ELLIPSE_PI, 0.5f, &p));
  near(p.x, 1, 1e-6f); near(p.y, 4, 1e-6f);
  assert(p.vx < 0);
  assert(tinyMpcEllipseSampleAt(&e, 2 * TINYMPC_ELLIPSE_PI, 0.5f, &p));
  near(p.x, 1, 1e-5f); near(p.y, 2, 1e-5f);

  /* Rotation and world translation commute with the anchored construction. */
  e.heading = TINYMPC_ELLIPSE_PI / 2;
  assert(tinyMpcEllipseSampleAt(&e, TINYMPC_ELLIPSE_PI / 2, 0.5f, &p));
  near(p.x, 0, 1e-6f); near(p.y, 4, 1e-6f);
  near(p.vx, -0.5f, 1e-6f); near(p.vy, 0, 1e-6f);

  /* Projection follows the supplied unwrapped window, including lap seams. */
  for (int i = 0; i < 100; ++i) {
    const float t = 5.0f + 0.03f * (float)i;
    assert(tinyMpcEllipseSampleAt(&e, t, 0.5f, &p));
    near(tinyMpcEllipseProjectForward(&e, p.x, p.y, t - 0.2f, t + 0.8f), t, 1e-4f);
    near(hypotf(p.vx, p.vy), 0.5f, 1e-6f);
  }
  assert(tinyMpcEllipseSampleAt(&e, 0.1f, 0.5f, &p));
  near(tinyMpcEllipseProjectForward(&e, p.x, p.y, 0.2f, 0.8f), 0.2f, 1e-6f);
  assert(tinyMpcEllipseSampleAt(&e, 1.0f, 0.5f, &p));
  near(tinyMpcEllipseProjectForward(&e, p.x, p.y, 0.2f, 0.8f), 0.8f, 1e-6f);

  /* Constant-speed arclength and CCW yaw-rate identity. */
  e.heading = 0;
  float t = 0, length = 0;
  TinyMpcEllipseSample old;
  assert(tinyMpcEllipseSampleAt(&e, t, 0.5f, &old));
  for (int i = 0; i < 2000; ++i) {
    const float next = tinyMpcEllipseAdvance(&e, t, 0.5f, 0.01f);
    assert(next > t);
    assert(tinyMpcEllipseSampleAt(&e, next, 0.5f, &p));
    length += hypotf(p.x - old.x, p.y - old.y);
    const float dyaw = atan2f(sinf(p.yaw - old.yaw), cosf(p.yaw - old.yaw));
    near(dyaw / 0.01f, 0.5f * old.curvature, 0.015f);
    old = p; t = next;
  }
  near(length, 10.0f, 0.002f);
  near(tinyMpcEllipseAdvance(&e, t, 0, 0.01f), t, 1e-6f);
  assert(!tinyMpcEllipseSampleAt(&e, NAN, 1, &p));
  assert(!tinyMpcEllipseSampleAt(&e, 0, -1, &p));
  e.b = 0;
  assert(!tinyMpcEllipseValid(&e));
  assert(!tinyMpcEllipseSampleAt(&e, 0, 1, &p));
  puts("ellipse geometry tests passed");
  return 0;
}
