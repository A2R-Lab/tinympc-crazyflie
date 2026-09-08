#include <assert.h>
#include <stdio.h>
#include "../src/tinympc_espnet_straight.h"

static void arm(tinympcEspnetStraightState *s) {
  tinympcEspnetStraightInit(s);
  tinympcEspnetStraightStep(s, 0, 0, true, 0.9f, 0.9f, 0.9f, false, .01f);
  tinympcEspnetStraightStep(s, 0, 0, true, 0.9f, 0.9f, 0.9f, true, .01f);
  assert(s->phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  assert(s->v == 3.0f); /* immediate velocity request */
}
int main(void) {
  assert(tinympcEspnetCollisionFresh(true, 267));
  assert(tinympcEspnetCollisionFresh(true, 400));
  assert(!tinympcEspnetCollisionFresh(true, 401));
  assert(!tinympcEspnetCollisionFresh(false, 0));
  assert(!tinympcEspnetCollisionFresh(true, UINT32_MAX));
  tinympcEspnetStraightState s;
  tinympcEspnetStraightInit(&s);
  tinympcEspnetStraightStep(&s, 0, 0, true, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
  tinympcEspnetStraightStep(&s, 0, 0, false, 0, 0, 0, false, .01f);
  tinympcEspnetStraightStep(&s, 0, 0, false, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
  tinympcEspnetStraightStep(&s, 0, 0, true, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE); /* no automatic retry */
  tinympcEspnetStraightStep(&s, 0, .16f, true, 0, 0, 0, false, .01f);
  tinympcEspnetStraightStep(&s, 0, .16f, true, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_MOVING);
  arm(&s); /* exactly .9 is allowed */
  tinympcEspnetStraightStep(&s, .7f, .5f, true, .90001f, .90001f, .90001f, true, .01f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_DANGER && s.v == 0 && s.s == .7f);
  tinympcEspnetStraightStep(&s, 1, 0, true, 0, 0, 0, false, .01f);
  tinympcEspnetStraightStep(&s, 1, 0, true, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && s.s == .7f);
  arm(&s);
  tinympcEspnetStraightStep(&s, .3f, 0, false, 0, 0, 0, true, .01f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_STALE && s.s == .3f);
  arm(&s);
  tinympcEspnetStraightStep(&s, 0, 0, true, NAN, NAN, NAN, true, .01f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_STALE);
  arm(&s);
  tinympcEspnetStraightStep(&s, .2f, 0, true, 0, 0, 0, false, .01f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_CANCEL);
  arm(&s);
  tinympcEspnetStraightStep(&s, 0, 0, true, 0, 0, 0, true, 15);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_TIMEOUT);
  arm(&s);
  tinympcEspnetStraightStep(&s, NAN, 0, true, 0, 0, 0, true, .01f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_INVALID && s.v == 0);
  arm(&s);
  tinympcEspnetStraightStep(&s, 0, 0, true, 0, 0, 0, true, NAN);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_INVALID);
  arm(&s);
  float horizon_s = s.s, horizon_v = s.v;
  tinympcEspnetStraightAdvanceReference(&horizon_s, &horizon_v, s.s, 1.0f);
  assert(horizon_v == 3.0f);
  assert(horizon_s <= s.s + .30001f); /* bounded integration */
  assert(horizon_s > s.s && s.elapsed == .01f); /* copies only */
  float peak = 0;
  for (int i=0; i<1400 && s.phase == TINYMPC_ESPNET_STRAIGHT_RUN; ++i) {
    float last_s = s.s;
    tinympcEspnetStraightStep(&s, s.s, s.v, true, 0, 0, 0, true, .01f);
    assert(isfinite(s.s) && isfinite(s.v) && s.s <= 5 && s.v <= 3 && s.v >= 0);
    assert(s.s >= last_s - .001f);

    peak = fmaxf(peak, s.v);
  }
  assert(peak > 2.99f && s.reason == TINYMPC_ESPNET_STRAIGHT_DISTANCE && s.v == 0);
  for (int sector = 0; sector < 3; ++sector) {
    float scores[3] = {.95f, .95f, .95f};
    scores[sector] = .9f;
    arm(&s);
    tinympcEspnetStraightStep(&s, 0, 0, true, scores[0], scores[1], scores[2], true, .01f);
    assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
    scores[sector] = .90001f;
    tinympcEspnetStraightStep(&s, 0, 0, true, scores[0], scores[1], scores[2], true, .01f);
    assert(s.reason == TINYMPC_ESPNET_STRAIGHT_DANGER);
    arm(&s);
    scores[sector] = NAN;
    tinympcEspnetStraightStep(&s, 0, 0, true, scores[0], scores[1], scores[2], true, .01f);
    assert(s.reason == TINYMPC_ESPNET_STRAIGHT_STALE);
  }
  puts("ESPNet straight-line state tests passed");
}
