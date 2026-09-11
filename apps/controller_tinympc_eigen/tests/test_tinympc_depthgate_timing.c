#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "tinympc_depthgate_timing.h"

static void near(float a, float b) { assert(fabsf(a-b)<1.e-5f); }
static void trajectory(TinyDepthGateHistory *h, uint32_t start) {
  for(unsigned i=0;i<=120;++i) {
    float seconds=i*.05f;
    tinyDepthGateRecord(h,(TinyDepthGatePose){start+i*50u,
        .2f*seconds,-.1f*seconds,.1f*seconds});
  }
}
static void moving(uint32_t start) {
  TinyDepthGateHistory h={0}; TinyDepthGatePose out;
  trajectory(&h,start);
  assert(h.count==DG_POSE_CAPACITY);
  /* Six seconds now minus 100ms receive age, 1453ms inference, 7ms wire. */
  assert(tinyDepthGateCapturePose(&h,start+6000u,100,1452800,&out));
  assert(out.ms==start+4440u);
  near(out.x,.888f);near(out.y,-.444f);near(out.yaw,.444f);
  assert(tinyDepthGateCapturePose(&h,start+6000u,1800,2000000,&out));
  near(out.x,.4386f);
  assert(!tinyDepthGateCapturePose(&h,start+6000u,1801,1452800,&out));
  assert(!tinyDepthGateCapturePose(&h,start+6000u,0,2000001,&out));
}
int main(void) {
  assert(tinyDepthGateFresh(true,1800,2000000));
  assert(!tinyDepthGateFresh(false,0,1452800));
  assert(!tinyDepthGateFresh(true,0,0));
  assert(!tinyDepthGateFresh(true,UINT32_MAX,1452800));
  moving(10000u);moving(UINT32_MAX-3000u); /* clock and ring wrap */
  TinyDepthGateHistory h={0};TinyDepthGatePose out;
  assert(!tinyDepthGateCapturePose(&h,2000,0,1452800,&out));
  tinyDepthGateRecord(&h,(TinyDepthGatePose){500,0,0,3.10f});
  assert(!tinyDepthGateCapturePose(&h,2000,0,1452800,&out));
  tinyDepthGateRecord(&h,(TinyDepthGatePose){520,9,9,0});
  assert(h.count==1); /* throttled */
  tinyDepthGateRecord(&h,(TinyDepthGatePose){550,1,2,-3.10f});
  assert(tinyDepthGateCapturePose(&h,1985,0,1452800,&out));
  near(out.x,.5f);near(out.y,1.f);near(out.yaw,3.14159265f);
  assert(!tinyDepthGateCapturePose(&h,1900,0,1452800,&out));
  assert(!tinyDepthGateCapturePose(&h,2100,0,1452800,&out));
  tinyDepthGateRecord(&h,(TinyDepthGatePose){600,NAN,0,0});
  assert(h.count==2);
  tinyDepthGateRecord(&h,(TinyDepthGatePose){700,2,3,0});
  assert(!tinyDepthGateCapturePose(&h,2085,0,1452800,&out));
  puts("depthgate timing: PASS");
  return 0;
}
