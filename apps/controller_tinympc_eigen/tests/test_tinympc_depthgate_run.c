#include <assert.h>
#include <stdio.h>
#include "tinympc_depthgate_run.h"

static void near(float a,float b) { assert(fabsf(a-b)<1.e-5f); }
int main(void) {
  TinyDepthGateRun s={0};
  assert(tinyDepthGateRunConfig(0,1));
  assert(tinyDepthGateRunConfig(20,120));
  assert(!tinyDepthGateRunConfig(-1,10));
  assert(!tinyDepthGateRunConfig(20.01f,10));
  assert(!tinyDepthGateRunConfig(NAN,10));
  assert(!tinyDepthGateRunConfig(1,INFINITY));
  assert(!tinyDepthGateRunConfig(1,.99f));
  assert(!tinyDepthGateRunConfig(1,120.01f));
  /* Start at a nonzero world position, heading along +Y. Lateral motion
   * and backwards movement must not count towards a forward distance. */
  const float yaw=1.57079632679f;
  assert(!tinyDepthGateRunUpdate(&s,true,7,3,yaw,.01f,2,10));
  assert(!tinyDepthGateRunUpdate(&s,true,10,2,yaw,1,2,10));
  near(s.travel,-1);
  assert(!tinyDepthGateRunUpdate(&s,true,10,4,yaw,1,2,10));
  near(s.travel,1);
  assert(tinyDepthGateRunUpdate(&s,true,7,5,yaw,1,2,10)==9);
  /* Returning behind the limit or changing options cannot resume RUN. */
  assert(tinyDepthGateRunUpdate(&s,true,7,4,yaw,1,10,120)==9);
  near(s.travel,1);
  assert(tinyDepthGateRunUpdate(&s,true,7,4,yaw,1,NAN,120)==9);
  assert(!tinyDepthGateRunUpdate(&s,false,7,4,yaw,1,2,10));
  assert(!s.running); near(s.travel,0); near(s.elapsed,0);
  assert(!tinyDepthGateRunUpdate(&s,true,7,4,yaw,.01f,2,10));
  near(s.travel,0);
  /* Zero distance preserves time-limited behavior. */
  tinyDepthGateRunReset(&s);
  assert(!tinyDepthGateRunUpdate(&s,true,0,0,0,.01f,0,1));
  assert(!tinyDepthGateRunUpdate(&s,true,20,0,0,.5f,0,1));
  assert(tinyDepthGateRunUpdate(&s,true,20,0,0,.5f,0,1)==4);
  assert(tinyDepthGateRunUpdate(&s,true,20,0,0,.1f,0,120)==4);
  /* A parameter error during RUN is also latched until release/reset. */
  tinyDepthGateRunReset(&s);
  assert(!tinyDepthGateRunUpdate(&s,true,0,0,0,0,1,10));
  assert(tinyDepthGateRunUpdate(&s,true,0,0,0,.1f,21,10)==1);
  assert(tinyDepthGateRunUpdate(&s,true,0,0,0,.1f,1,10)==1);
  tinyDepthGateRunReset(&s);
  assert(tinyDepthGateRunUpdate(&s,true,NAN,0,0,.1f,1,10)==8);
  tinyDepthGateRunReset(&s);
  assert(!tinyDepthGateRunUpdate(&s,true,5,6,0,.01f,1,10));
  near(s.travel,0);
  puts("depthgate run limits: PASS");
  return 0;
}
