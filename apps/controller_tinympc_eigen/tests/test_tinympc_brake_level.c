#include <assert.h>
#include "../src/tinympc_brake_level.h"
int main(void) {
  TinyBrakeLevel s = {0};
  assert(!tinyBrakeLevelStep(&s,2.76f,0,0,.01f));
  // Representative measured braking tilt/rate from the failed run: begin
  // recovery while still moving forward, rather than waiting for reversal.
  assert(tinyBrakeLevelStep(&s,2.63f,.46f,5.1f,.01f));
  assert(s.phase == 1);
  assert(!tinyBrakeLevelStep(&s,2,.4f,1,.01f));
  tinyBrakeLevelStep(&s,.3f,.05f,.1f,.01f);
  assert(s.phase == 2);
  tinyBrakeLevelStep(&s,-1,.5f,1,.01f);
  assert(s.phase == 2);
  s = (TinyBrakeLevel){0};
  tinyBrakeLevelStep(&s,-.1f,.8f,1,.01f);
  for(int i=0;i<90;i++) tinyBrakeLevelStep(&s,1,.8f,1,.01f);
  assert(s.phase == 2); // bounded recovery interval
  return 0;
}
