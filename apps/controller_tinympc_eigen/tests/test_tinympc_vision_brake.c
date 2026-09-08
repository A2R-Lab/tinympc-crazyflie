#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/tinympc_vision_brake.h"

static uint16_t sequence;
static void next_frame(tinympcEspnetStraightState *s, float distance, float speed,
    bool fresh, float l, float c, float r, bool run, float dt) {
  sequence = sequence == 65535u ? 1u : (uint16_t)(sequence+1u);
  tinympcVisionBrakeStep(s,distance,speed,fresh,l,c,r,run,dt,sequence);
}
#define tinympcVisionBrakeStep next_frame

static void clear_step(tinympcEspnetStraightState *s, bool run, float dt)
{
  tinympcVisionBrakeStep(s, 0.0f, 0.0f, true, 0.0f, 0.0f, 0.0f, run, dt);
}

static tinympcEspnetStraightState idle(void)
{
  tinympcEspnetStraightState s;
  tinympcEspnetStraightInit(&s);
  clear_step(&s, true, 0.01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
  clear_step(&s, false, 0.01f);
  return s;
}

static tinympcEspnetStraightState start(void)
{
  tinympcEspnetStraightState s = idle();
  clear_step(&s, true, 0.01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && s.v == 2.0f);
  return s;
}

static void stopped(tinympcEspnetStraightState *s, tinympcEspnetStraightReason reason)
{
  assert(s->phase == TINYMPC_ESPNET_STRAIGHT_STOP && s->reason == reason && s->v == 0);
  clear_step(s, false, 0.01f);
  clear_step(s, true, 0.01f);
  assert(s->phase == TINYMPC_ESPNET_STRAIGHT_STOP && s->reason == reason);
}

int main(void)
{
  tinympcEspnetStraightState s = start();
  /* Exact .95 is clear; only center can trigger danger braking. */
  tinympcVisionBrakeStep(&s, 1, 5, true, 1, .95f, 1, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  for (unsigned mask = 0; mask < 8; ++mask) {
    s = start();
    tinympcVisionBrakeStep(&s, 50, 12, true, mask & 1 ? 1 : 0,
                          mask & 2 ? 1 : 0, mask & 4 ? 1 : 0, true, .01f);
    if (mask & 2) {
      assert(s.brake_frames == 1);
      stopped(&s, TINYMPC_ESPNET_STRAIGHT_DANGER);
    }
    else assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && s.v == 2);
  }
  s = start();
  tinympcVisionBrakeStep(&s, 2, 5, true, 0, .95f, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  tinympcVisionBrakeStep(&s, 2, 5, true, 0, .95001f, 0, true, .01f);
  assert(s.brake_frames == 1);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_DANGER);
  s = idle();
  tinympcVisionBrakeStep(&s, 0, 0, true, 0, .95001f, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE && s.reason == TINYMPC_ESPNET_STRAIGHT_DANGER);

  /* Rejected starts consume the edge without latching STOP. */
  for (unsigned kind = 0; kind < 3; ++kind) {
    s = idle();
    tinympcVisionBrakeStep(&s, 0, kind == 2 ? .151f : 0, kind != 0,
                          0, kind == 1 ? .95001f : 0, 0, true, .01f);
    assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
    assert(s.reason == (kind == 0 ? TINYMPC_ESPNET_STRAIGHT_STALE :
           kind == 1 ? TINYMPC_ESPNET_STRAIGHT_DANGER : TINYMPC_ESPNET_STRAIGHT_MOVING));
    clear_step(&s, true, .01f);
    assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
    clear_step(&s, false, .01f);
    tinympcVisionBrakeStep(&s, 0, .15f, true, 1, 0, 0, true, .01f);
    assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  }

  s = start();
  tinympcVisionBrakeStep(&s, 1, 3, false, 0, 0, 0, true, .01f);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_STALE);
  const float bad_probabilities[] = {NAN, INFINITY, -.01f, 1.01f};
  for (unsigned i = 0; i < 4; ++i) {
    s = start();
    tinympcVisionBrakeStep(&s, 1, 3, true, 0, bad_probabilities[i], 0, true, .01f);
    stopped(&s, TINYMPC_ESPNET_STRAIGHT_STALE);
  }
  s = start();
  clear_step(&s, false, .01f);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_CANCEL);
  s = start();
  s.elapsed = 14.5f;
  clear_step(&s, true, .25f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && fabsf(s.s - .22f) < 1e-6f);
  clear_step(&s, true, .25f);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_TIMEOUT);
  s = start();
  clear_step(&s, true, -1);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_INVALID);
  s = start();
  tinympcVisionBrakeStep(&s, NAN, 0, true, 0, 0, 0, true, .01f);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_INVALID);
  s = start();
  tinympcVisionBrakeStep(&s, 0, -1, true, 0, 0, 0, true, .01f);
  stopped(&s, TINYMPC_ESPNET_STRAIGHT_INVALID);
  // The previously missed 0.9692 warning now triggers immediately.
#undef tinympcVisionBrakeStep
  s = start();
  tinympcVisionBrakeStep(&s,0,1,true,.99115f,.96921f,.76886f,true,.01f,77);
  stopped(&s,TINYMPC_ESPNET_STRAIGHT_DANGER);
  // A later clear frame cannot release the latched brake.
  tinympcVisionBrakeStep(&s,0,1,true,0,0,0,true,.01f,1031);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP);
  s = start();
  tinympcVisionBrakeStep(&s,0,1,false,0,1,0,true,.01f,1000);
  stopped(&s,TINYMPC_ESPNET_STRAIGHT_STALE);
  puts("vision brake tests passed");
  return 0;
}
