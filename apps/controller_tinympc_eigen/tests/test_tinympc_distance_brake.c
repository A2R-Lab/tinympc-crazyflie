#include <assert.h>
#include "../src/tinympc_distance_brake.h"
int main(void) {
  tinympcEspnetStraightState s;
  tinympcEspnetStraightInit(&s);
  tinympcDistanceBrakeStep(&s,0,0,false,.01f,3,2);
  tinympcDistanceBrakeStep(&s,0,0,true,.01f,3,2);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN && s.v == 3);
  s.s = 10; // An advancing reference must not trigger the measured-distance stop.
  tinympcDistanceBrakeStep(&s,1.9f,3,true,.01f,3,2);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_RUN);
  tinympcDistanceBrakeStep(&s,2,2.5f,true,.01f,3,2);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP && s.v == 0);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_DISTANCE);
  tinympcDistanceBrakeStep(&s,1,0,true,.01f,3,2);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_STOP);
  tinympcEspnetStraightInit(&s);
  tinympcDistanceBrakeStep(&s,0,0,true,.01f,NAN,2);
  assert(s.reason == TINYMPC_ESPNET_STRAIGHT_INVALID);
  tinympcEspnetStraightInit(&s);
  tinympcDistanceBrakeStep(&s,0,0,false,.01f,3,2);
  tinympcDistanceBrakeStep(&s,0,.3f,true,.01f,3,2);
  assert(s.phase == TINYMPC_ESPNET_STRAIGHT_IDLE);
  return 0;
}
