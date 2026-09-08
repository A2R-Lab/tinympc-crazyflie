#include <assert.h>
#include <float.h>
#include <stddef.h>
#include "tinympc_depthgate_planes.h"

static TinyDepthGatePlanes depths(float l, float c, float r, unsigned prev) {
  const float inverse[3] = {1.f/l,1.f/c,1.f/r};
  return tinyDepthGatePlanes(inverse,.6f,.2f,2.f,8.f,prev);
}
static void check_plane(TinyDepthGatePlanes p, unsigned j, float x, float y) {
  assert(fabsf(hypotf(p.nx[j],p.ny[j])-1.f)<1.e-5f);
  /* Raw surface sample lies clearance meters outside inflated halfspace. */
  assert(fabsf(p.nx[j]*x+p.ny[j]*y-p.b[j]-.2f)<1.e-5f);
}
int main(void) {
  TinyDepthGatePlanes p=depths(3.f,.8f,1.f,0);
  assert(p.valid && p.mode==1 && p.count==1);
  assert(p.ny[0]<0); /* moving left relaxes center obstacle constraint */
  check_plane(p,0,.8f,0); check_plane(p,0,3.f,1.8f);
  p=depths(1.f,.8f,3.f,0);
  assert(p.valid && p.mode==2 && p.count==1 && p.ny[0]>0);
  check_plane(p,0,.8f,0); check_plane(p,0,3.f,-1.8f);
  p=depths(.8f,3.f,1.f,0);
  assert(p.valid && p.mode==3 && p.count==2);
  assert(p.ny[0]>0 && p.ny[1]<0); /* corridor narrows toward center apex */
  check_plane(p,0,.8f,.48f); check_plane(p,0,3.f,0);
  check_plane(p,1,1.f,-.6f); check_plane(p,1,3.f,0);
  assert(p.b[0]>0 && p.b[1]>0); /* body origin on permitted side */
  p=depths(1.f,1.f,1.f,0);
  assert(p.count==1 && fabsf(p.nx[0]-1.f)<1.e-6f);
  assert(fabsf(p.ny[0])<1.e-6f && fabsf(p.b[0]-.8f)<1.e-6f);
  p=depths(4.f,4.f,4.f,0); assert(p.valid && !p.count && !p.mode);
  p=depths(100.f,100.f,100.f,0); assert(p.valid && p.depth[0]==8.f);
  p=depths(1.f,.8f,1.01f,1); assert(p.mode==1); /* tie hysteresis */
  p=depths(1.f,1.1f,1.f,3); assert(p.mode==3);
  p=depths(1.f,1.1f,1.f,0); assert(p.mode!=3);
  float inv[3]={1,1,1};
  assert(!tinyDepthGatePlanes(NULL,.6f,.2f,2,8,0).valid);
  inv[0]=NAN; assert(!tinyDepthGatePlanes(inv,.6f,.2f,2,8,0).valid);
  inv[0]=INFINITY; assert(!tinyDepthGatePlanes(inv,.6f,.2f,2,8,0).valid);
  inv[0]=0; assert(!tinyDepthGatePlanes(inv,.6f,.2f,2,8,0).valid);
  inv[0]=-1; assert(!tinyDepthGatePlanes(inv,.6f,.2f,2,8,0).valid);
  inv[0]=1;
  assert(!tinyDepthGatePlanes(inv,0,.2f,2,8,0).valid);
  assert(!tinyDepthGatePlanes(inv,.6f,-1,2,8,0).valid);
  assert(!tinyDepthGatePlanes(inv,.6f,.2f,.1f,8,0).valid);
  assert(!tinyDepthGatePlanes(inv,.6f,.2f,2,1,0).valid);
  /* Very close surfaces may exclude origin: caller must handle infeasibility. */
  p=depths(.1f,.1f,.1f,0); assert(p.valid && p.b[0]<0);
  return 0;
}
