#include <assert.h>
#include <stdio.h>
#include "../src/tinympc_vision_stop.h"

static tinympcEspnetStraightState armed(int *turn)
{
  tinympcEspnetStraightState s;
  tinympcEspnetStraightInit(&s);
  *turn = 0;
  tinympcVisionStraightStep(&s, 0, 0, true, 0, 0, 0, false, .01f, turn);
  return s;
}
static tinympcEspnetStraightState running(int *turn)
{
  tinympcEspnetStraightState s = armed(turn);
  tinympcVisionStraightStep(&s, 0, 0, true, 0, 0, 0, true, .01f, turn);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  return s;
}
int main(void)
{
  for (unsigned mask = 0; mask < 8; ++mask) {
    float l = (mask & 1) ? .91f : .9f;
    float c = (mask & 2) ? .91f : .9f;
    float r = (mask & 4) ? .91f : .9f;
    unsigned n = !!(mask & 1) + !!(mask & 2) + !!(mask & 4);
    int expected = mask == 6 ? 1 : mask == 3 ? -1 : 0;
    tinympcVisionDecision d = tinympcVisionClassify(true, l, c, r);
    assert(d.valid && d.dangerous_sectors == n && d.turn_direction == expected);
    int turn;
    tinympcEspnetStraightState s = running(&turn);
    tinympcVisionStraightStep(&s, .1f, 3.0f, true, l, c, r, true, .01f, &turn);
    assert(s.phase == (n >= 2 ? TINYMPC_ESPNET_STRAIGHT_STOP : TINYMPC_ESPNET_STRAIGHT_RUN));
    assert(turn == expected);
    if (n >= 2) {
      assert(s.v == 0 && s.reason == TINYMPC_ESPNET_STRAIGHT_DANGER);
      tinympcVisionStraightStep(&s, .2f, 0, true, 0, 0, 0, false, .01f, &turn);
      assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && turn == expected);
      s = armed(&turn);
      tinympcVisionStraightStep(&s, 0, 0, true, l, c, r, true, .01f, &turn);
      assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE && turn == 0);
    } else assert(s.v == 4.0f && fabsf(s.s - .08f) < 1e-6f);
  }
  const float bad[] = {NAN, INFINITY, -.01f, 1.01f};
  for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
    for (unsigned sector = 0; sector < 3; ++sector) {
      float p[3] = {0, 0, 0}; p[sector] = bad[i];
      assert(!tinympcVisionClassify(true, p[0], p[1], p[2]).valid);
      int turn; tinympcEspnetStraightState s = running(&turn);
      tinympcVisionStraightStep(&s, 0, 0, true, p[0], p[1], p[2], true, .01f, &turn);
      assert(s.reason == TINYMPC_ESPNET_STRAIGHT_STALE && turn == 0 && s.v == 0);
    }
  int turn; tinympcEspnetStraightState s = running(&turn);
  tinympcVisionStraightStep(&s, 0, 0, false, 0, 0, 0, true, .01f, &turn);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_STALE && turn == 0);
  s = running(&turn);
  tinympcVisionStraightStep(&s, 0, 0, true, 0, 1, 1, false, .01f, &turn);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_CANCEL && turn == 0);
  s = running(&turn);
  tinympcVisionStraightStep(&s, -5, 0, true, 0, 0, 0, true, .01f, &turn);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_DISTANCE && turn == 0);
  s = running(&turn); s.elapsed = 14.99f;
  tinympcVisionStraightStep(&s, 0, 0, true, 0, 0, 0, true, .02f, &turn);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_TIMEOUT && turn == 0);
  s = armed(&turn);
  tinympcVisionStraightStep(&s, 0, .16f, true, 0, 0, 0, true, .01f, &turn);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_MOVING && turn == 0);
  s = running(&turn);
  tinympcVisionStraightStep(&s, 0, 0, true, 0, 0, 0, true, .5f, &turn);
  assert(fabsf(s.s - .44f) < 1e-6f);
  const float pi = 3.14159265358979323846f;
  const float headings[] = {0, .7f, -1.8f, pi - .01f, -pi + .01f, 9.0f};
  for (unsigned i = 0; i < sizeof(headings) / sizeof(headings[0]); ++i) {
    for (int direction = -1; direction <= 1; ++direction) {
      const float a = headings[i];
      const float angle = direction * pi / 4.0f;
      const float rotated = tinympcVisionTurnHeading(a, direction);
      assert(rotated >= -pi && rotated <= pi);
      /* The new leg's unit vector is the old leg rotated with the yaw. */
      assert(fabsf(cosf(rotated) -
          (cosf(a) * cosf(angle) - sinf(a) * sinf(angle))) < 1e-6f);
      assert(fabsf(sinf(rotated) -
          (sinf(a) * cosf(angle) + cosf(a) * sinf(angle))) < 1e-6f);
    }
  }
  assert(isnan(tinympcVisionTurnHeading(NAN, 1)));
  assert(isinf(tinympcVisionTurnHeading(INFINITY, -1)));
  assert(tinympcVisionTurnHeading(.2f, 2) == .2f);
  assert(tinympcVisionTurnHeading(.2f, -2) == .2f);
  float clear_time = 0;
  for (int i = 0; i < 29; ++i)
    assert(!tinympcVisionResumeClear(true, .9f, .9f, .9f, .15f, .01f, &clear_time));
  assert(tinympcVisionResumeClear(true, .9f, .9f, .9f, .15f, .01f, &clear_time));
  assert(tinympcVisionResumeClear(true, 0, 0, 0, 0, 0, &clear_time));
  for (unsigned sector = 0; sector < 3; ++sector) {
    float p[3] = {0, 0, 0}; p[sector] = .91f;
    assert(!tinympcVisionResumeClear(true, p[0], p[1], p[2], 0, .01f, &clear_time));
    assert(clear_time == 0);
    clear_time = .3f;
    p[sector] = NAN;
    assert(!tinympcVisionResumeClear(true, p[0], p[1], p[2], 0, .01f, &clear_time));
    assert(clear_time == 0);
  }
  clear_time = .3f;
  assert(!tinympcVisionResumeClear(false, 0, 0, 0, 0, .01f, &clear_time));
  assert(clear_time == 0);
  const float invalid_speed[] = {NAN, INFINITY, -.01f, .151f};
  for (unsigned i = 0; i < sizeof(invalid_speed) / sizeof(invalid_speed[0]); ++i) {
    clear_time = .3f;
    assert(!tinympcVisionResumeClear(true, 0, 0, 0, invalid_speed[i], .01f, &clear_time));
    assert(clear_time == 0);
  }
  const float invalid_time[] = {NAN, INFINITY, -.01f};
  for (unsigned i = 0; i < sizeof(invalid_time) / sizeof(invalid_time[0]); ++i) {
    clear_time = .3f;
    assert(!tinympcVisionResumeClear(true, 0, 0, 0, 0, invalid_time[i], &clear_time));
    assert(clear_time == 0);
    clear_time = invalid_time[i];
    assert(!tinympcVisionResumeClear(true, 0, 0, 0, 0, .01f, &clear_time));
    assert(clear_time == 0);
  }
  assert(!tinympcVisionResumeClear(true, 0, 0, 0, 0, 10, &clear_time));
  assert(fabsf(clear_time - .1f) < 1e-6f);
  assert(!tinympcVisionResumeClear(true, 0, 0, 0, 0, 10, &clear_time));
  assert(tinympcVisionResumeClear(true, 0, 0, 0, 0, 10, &clear_time));
  assert(!tinympcVisionResumeClear(true, 0, 0, 0, 0, .01f, NULL));
  assert(fabsf(tinympcVisionYawAdvance(0, 1, .1f) - pi / 60.0f) < 1e-6f);
  assert(fabsf(tinympcVisionYawAdvance(0, -1, .1f) + pi / 60.0f) < 1e-6f);
  assert(fabsf(tinympcVisionYawAdvance(0, 1, 10) - pi / 60.0f) < 1e-6f);
  assert(fabsf(tinympcVisionYawAdvance(0, .01f, .1f) - .01f) < 1e-6f);
  assert(tinympcVisionYawAdvance(.2f, 1, 0) == .2f);
  assert(tinympcVisionYawAdvance(.2f, 1, -1) == .2f);
  assert(tinympcVisionYawAdvance(.2f, 1, NAN) == .2f);
  assert(tinympcVisionYawAdvance(.2f, NAN, .1f) == .2f);
  assert(isnan(tinympcVisionYawAdvance(NAN, 1, .1f)));
  float wrap = tinympcVisionYawAdvance(pi - .02f, -pi + .02f, .1f);
  assert(fabsf(wrap - (-pi + .02f)) < 1e-6f);
  wrap = tinympcVisionYawAdvance(-pi + .02f, pi - .02f, .1f);
  assert(fabsf(wrap - (pi - .02f)) < 1e-6f);
  puts("Vision pair stop tests passed");
  return 0;
}
