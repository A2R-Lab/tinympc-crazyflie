#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../src/tinympc_espnet_straight.h"

static tinympcEspnetStraightState start(void)
{
  tinympcEspnetStraightState s;
  tinympcEspnetStraightInit(&s);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, false, .01f);
  tinympcSpeedBrakeStep(&s, 0, .15f, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && s.v == 5);
  return s;
}

static void expect_stop(float distance, float forward, bool run, float dt,
                        tinympcEspnetStraightReason reason)
{
  tinympcEspnetStraightState s = start();
  tinympcSpeedBrakeStep(&s, distance, fabsf(forward), forward, run, dt);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && s.reason == reason);
  assert(s.v == 0);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, false, .01f);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && s.reason == reason);
}

int main(void)
{
  assert(TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED == 8);
  tinympcEspnetStraightState s = start();
  tinympcSpeedBrakeStep(&s, .5f, 4.999f, 4.999f, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && fabsf(s.s - .10f) < 1e-6f && s.v == 5);
  expect_stop(.5f, 5, true, .01f, TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED);
  expect_stop(.5f, 5.1f, true, .01f, TINYMPC_ESPNET_STRAIGHT_SPEED_REACHED);
  s = start();
  tinympcSpeedBrakeStep(&s, 0, 3, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN); /* Lateral speed. */
  tinympcSpeedBrakeStep(&s, -.5f, 3, -3, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN); /* Backward speed. */
  tinympcSpeedBrakeStep(&s, .999f, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  tinympcSpeedBrakeStep(&s, -.999f, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  expect_stop(1, 0, true, .01f, TINYMPC_ESPNET_STRAIGHT_DISTANCE);
  expect_stop(-1, -3, true, .01f, TINYMPC_ESPNET_STRAIGHT_DISTANCE);
  expect_stop(1.1f, 0, true, .01f, TINYMPC_ESPNET_STRAIGHT_DISTANCE);
  expect_stop(-1.1f, 0, true, .01f, TINYMPC_ESPNET_STRAIGHT_DISTANCE);
  expect_stop(1, 1, false, .01f, TINYMPC_ESPNET_STRAIGHT_CANCEL);
  s = start();
  s.elapsed = 14.98f;
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .02f);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_TIMEOUT && s.v == 0);
  expect_stop(0, 0, true, 15, TINYMPC_ESPNET_STRAIGHT_TIMEOUT);

  tinympcEspnetStraightInit(&s);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE); /* Handoff high not start. */
  tinympcSpeedBrakeStep(&s, 0, 0, 0, false, .01f);
  tinympcSpeedBrakeStep(&s, 0, .151f, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE &&
         s.reason == TINYMPC_ESPNET_STRAIGHT_MOVING);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE); /* Rejected edge consumed. */
  tinympcSpeedBrakeStep(&s, 0, 0, 0, false, .01f);
  tinympcSpeedBrakeStep(&s, 0, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);

  const float invalid[] = {NAN, INFINITY, -INFINITY};
  for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) {
    for (int field = 0; field < 7; ++field) {
      s = start();
      float distance = 0, total = 0, forward = 0, dt = .01f;
      switch (field) {
        case 0: distance = invalid[i]; break;
        case 1: total = invalid[i]; break;
        case 2: forward = invalid[i]; break;
        case 3: dt = invalid[i]; break;
        case 4: s.s = invalid[i]; break;
        case 5: s.v = invalid[i]; break;
        case 6: s.elapsed = invalid[i]; break;
      }
      tinympcSpeedBrakeStep(&s, distance, total, forward, true, dt);
      assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && s.v == 0 &&
             s.reason == TINYMPC_ESPNET_STRAIGHT_INVALID);
    }
  }
  expect_stop(0, 0, true, -.01f, TINYMPC_ESPNET_STRAIGHT_INVALID);
  s = start();
  tinympcSpeedBrakeStep(&s, .2f, 0, 0, true, 0);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && fabsf(s.s - .05f) < 1e-6f);
  /* Retain forward position error; reference progression alone does not stop. */
  for (int i = 0; i < 300; ++i)
    tinympcSpeedBrakeStep(&s, .2f, 0, 0, true, .01f);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && fabsf(s.s - 15.05f) < 1e-3f);
  float predicted_s = 4.99f, predicted_v = 0;
  tinympcSpeedBrakeAdvanceReference(&predicted_s, &predicted_v, .02f);
  assert(fabsf(predicted_s - 5.09f) < 1e-5f && predicted_v == 5);
  tinympcSpeedBrakeAdvanceReference(&predicted_s, &predicted_v, 1);
  assert(fabsf(predicted_s - 5.59f) < 1e-5f);
  for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
    tinympcSpeedBrakeAdvanceReference(&predicted_s, &predicted_v, invalid[i]);
  tinympcSpeedBrakeAdvanceReference(&predicted_s, &predicted_v, 0);
  tinympcSpeedBrakeAdvanceReference(&predicted_s, &predicted_v, -1);
  assert(fabsf(predicted_s - 5.59f) < 1e-5f && predicted_v == 5);
  puts("Speed braking state tests passed");
  return 0;
}
