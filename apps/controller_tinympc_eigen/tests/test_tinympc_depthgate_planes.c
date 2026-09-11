#include <assert.h>
#include <stddef.h>
#include "tinympc_depthgate_planes.h"
static TinyDepthGatePlanes run(float l,float c,float r,
    const TinyDepthGatePoint *path,unsigned n) {
  const float inv[3]={1.f/l,1.f/c,1.f/r};
  return tinyDepthGatePlanes(inv,.598203f,.2f,6.f,path,n);
}
int main(void) {
  const TinyDepthGatePoint forward[]={{0,0},{2,0}};
  TinyDepthGatePlanes p=run(3,1,2,forward,2);
  assert(p.valid && p.count==1 && p.mode==2 && p.ny[0]<0);
  assert(fabsf(hypotf(p.nx[0],p.ny[0])-1)<1.e-6f);
  assert(fabsf(p.b[0]-(p.nx[0]*1.f-.2f))<1.e-6f);
  p=run(2,1,3,forward,2);assert(p.count==1 && p.ny[0]>0);
  p=run(1,1,1,forward,2);assert(p.count==1 && p.ny[0]==0);
  // A near obstacle outside the projected trajectory is irrelevant.
  p=run(.4f,4,4,forward,2);assert(p.valid && p.count==0);
  const TinyDepthGatePoint left[]={{0,0},{2,1.2f}};
  p=run(1,4,4,left,2);assert(p.count==1 && p.mode==1 && p.ny[0]>0);
  const TinyDepthGatePoint right[]={{0,0},{2,-1.2f}};
  p=run(4,4,1,right,2);assert(p.count==1 && p.mode==4 && p.ny[0]<0);
  // Image FOV, behind-camera points, and a path ending before clearance.
  const TinyDepthGatePoint outside[]={{0,0},{1,2}};
  p=run(.4f,.4f,.4f,outside,2);assert(p.count==0);
  const TinyDepthGatePoint behind[]={{-2,0},{-1,0}};
  p=run(.4f,.4f,.4f,behind,2);assert(p.count==0);
  const TinyDepthGatePoint shortpath[]={{0,0},{.7f,0}};
  p=run(3,1,3,shortpath,2);assert(p.count==0);
  // Segment clipping catches center collisions between knots in other strips.
  const TinyDepthGatePoint crossing[]={{1,.7f},{1,-.7f}};
  p=run(4,.9f,4,crossing,2);assert(p.count==1 && p.mode==2);
  const TinyDepthGatePoint multiple[]={{0,0},{1,.6f},{1,-.6f}};
  p=run(.8f,.8f,.8f,multiple,3);assert(p.count==2);
  p=run(100,100,100,forward,2);assert(p.valid && p.count==0);
  // Geometry depends on image/path and fixed clearance, not a speed margin.
  const TinyDepthGatePoint dense[]={{0,0},{.4f,0},{.8f,0},{1.2f,0},{2,0}};
  p=run(3,1,2,forward,2);
  TinyDepthGatePlanes q=run(3,1,2,dense,5);
  assert(p.count==q.count && fabsf(p.b[0]-q.b[0])<1.e-6f);
  float inv[3]={1,1,1};
  assert(!tinyDepthGatePlanes(NULL,.6f,.2f,6,forward,2).valid);
  inv[0]=NAN;assert(!tinyDepthGatePlanes(inv,.6f,.2f,6,forward,2).valid);
  inv[0]=0;assert(!tinyDepthGatePlanes(inv,.6f,.2f,6,forward,2).valid);
  inv[0]=1;assert(!tinyDepthGatePlanes(inv,.6f,.2f,6,NULL,2).valid);
  assert(!tinyDepthGatePlanes(inv,.6f,.2f,6,forward,1).valid);
  return 0;
}
