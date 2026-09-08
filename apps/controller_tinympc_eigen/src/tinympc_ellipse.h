#ifndef TINYMPC_ELLIPSE_H
#define TINYMPC_ELLIPSE_H

#include <stdbool.h>
#include <math.h>

#define TINYMPC_ELLIPSE_PI 3.14159265358979323846f

/* World NWU geometry. theta=0 is the near minor-axis endpoint; its tangent
 * is heading. Increasing theta travels counter-clockwise. a/b are semiaxes,
 * not full diameters. Progress theta is unwrapped; output yaw is [-pi,pi]. */
typedef struct {
  float start_x, start_y, height, heading, a, b;
} TinyMpcEllipse;

typedef struct {
  float x, y, z, vx, vy, yaw, curvature, metric;
} TinyMpcEllipseSample;

static inline bool tinyMpcEllipseValid(const TinyMpcEllipse *e) {
  return e && isfinite(e->start_x) && isfinite(e->start_y) &&
      isfinite(e->height) && isfinite(e->heading) && isfinite(e->a) &&
      isfinite(e->b) && e->b > 0.001f && e->a >= e->b;
}

static inline bool tinyMpcEllipseSampleAt(const TinyMpcEllipse *e,
    float theta, float speed, TinyMpcEllipseSample *out) {
  if (!out || !tinyMpcEllipseValid(e) || !isfinite(theta) ||
      !isfinite(speed) || speed < 0.0f) return false;
  const float c = cosf(e->heading), s = sinf(e->heading);
  const float st = sinf(theta), ct = cosf(theta);
  const float u = e->a * st, v = e->b * (1.0f - ct);
  const float du = e->a * ct, dv = e->b * st;
  const float metric = hypotf(du, dv);
  const float tx = c * du - s * dv, ty = s * du + c * dv;
  out->x = e->start_x + c * u - s * v;
  out->y = e->start_y + s * u + c * v;
  out->z = e->height;
  out->vx = speed * tx / metric;
  out->vy = speed * ty / metric;
  out->yaw = atan2f(ty, tx);
  out->curvature = (e->a / metric) * (e->b / metric) / metric;
  out->metric = metric; /* ds/dtheta; yaw rate = speed * curvature. */
  return true;
}

/* Integrate dtheta/dt = speed / |dp/dtheta| using four fixed midpoint steps.
 * Intended for controller/horizon time steps, not multi-second mission jumps. */
static inline float tinyMpcEllipseAdvance(const TinyMpcEllipse *e,
    float theta, float speed, float dt) {
  if (!tinyMpcEllipseValid(e) || !isfinite(theta) || !isfinite(speed) ||
      !isfinite(dt) || speed < 0.0f || dt <= 0.0f) return theta;
  const float ds = speed * dt * 0.25f;
  for (int i = 0; i < 4; ++i) {
    const float m = hypotf(e->a * cosf(theta), e->b * sinf(theta));
    const float mid = theta + 0.5f * ds / m;
    theta += ds / hypotf(e->a * cosf(mid), e->b * sinf(mid));
  }
  return theta;
}

static inline float tinyMpcEllipseDistanceSquared(const TinyMpcEllipse *e,
    float u, float v, float theta) {
  const float dx = e->a * sinf(theta) - u;
  const float dy = e->b * (1.0f - cosf(theta)) - v;
  return dx * dx + dy * dy;
}

/* Find the nearest point in an explicitly forward progress window. The search
 * is bounded: 33 coarse samples then 20 golden-section refinements. At most one
 * lap is considered, preventing hidden lap jumps. Caller should use a smaller
 * window for rejoin. Endpoints are retained and ties select earlier progress. */
static inline float tinyMpcEllipseProjectForward(const TinyMpcEllipse *e,
    float x, float y, float theta_min, float theta_max) {
  if (!tinyMpcEllipseValid(e) || !isfinite(x) || !isfinite(y) ||
      !isfinite(theta_min) || !isfinite(theta_max) || theta_max <= theta_min)
    return theta_min;
  theta_max = fminf(theta_max, theta_min + 2.0f * TINYMPC_ELLIPSE_PI);
  const float c = cosf(e->heading), s = sinf(e->heading);
  const float dx = x - e->start_x, dy = y - e->start_y;
  const float u = c * dx + s * dy, v = -s * dx + c * dy;
  const float step = (theta_max - theta_min) / 32.0f;
  float best = theta_min;
  float best_d = tinyMpcEllipseDistanceSquared(e, u, v, best);
  int best_i = 0;
  for (int i = 1; i <= 32; ++i) {
    const float t = theta_min + step * (float)i;
    const float d = tinyMpcEllipseDistanceSquared(e, u, v, t);
    if (d < best_d) { best = t; best_d = d; best_i = i; }
  }
  float lo = theta_min + step * (float)(best_i > 0 ? best_i - 1 : 0);
  float hi = theta_min + step * (float)(best_i < 32 ? best_i + 1 : 32);
  const float ratio = 0.61803398875f;
  float p = hi - ratio * (hi - lo), q = lo + ratio * (hi - lo);
  float dp = tinyMpcEllipseDistanceSquared(e, u, v, p);
  float dq = tinyMpcEllipseDistanceSquared(e, u, v, q);
  for (int i = 0; i < 20; ++i) {
    if (dp <= dq) {
      hi = q; q = p; dq = dp; p = hi - ratio * (hi - lo);
      dp = tinyMpcEllipseDistanceSquared(e, u, v, p);
    } else {
      lo = p; p = q; dp = dq; q = lo + ratio * (hi - lo);
      dq = tinyMpcEllipseDistanceSquared(e, u, v, q);
    }
  }
  if (dp < best_d) { best = p; best_d = dp; }
  if (dq < best_d) best = q;
  return fmaxf(theta_min, fminf(theta_max, best));
}

#endif
