#ifndef TINYMPC_DEPTHGATE_PLANES_H
#define TINYMPC_DEPTHGATE_PLANES_H
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Camera coordinates: x optical depth, y left. The packet contains three
 * full-height image strips, not pixelwise depth. Planes are vertical. */
typedef struct { float x, y; } TinyDepthGatePoint;
typedef struct {
  bool valid;
  unsigned count, mode; /* mode is a bitmask of intersected image sectors */
  float depth[3], nx[2], ny[2], b[2];
} TinyDepthGatePlanes;

/* Clip a segment against a*x+b*y >= c. This avoids missing a collision
 * between trajectory knots or when a segment crosses an image-sector edge. */
static inline bool tinyDepthGateClip(TinyDepthGatePoint p, TinyDepthGatePoint q,
    float a, float b, float c, float *lo, float *hi) {
  const float v=a*p.x+b*p.y-c, dv=a*(q.x-p.x)+b*(q.y-p.y);
  if (fabsf(dv)<1.e-7f) return v>=0.f;
  const float t=-v/dv;
  if (dv>0.f) *lo=fmaxf(*lo,t); else *hi=fminf(*hi,t);
  return *lo<=*hi;
}

static inline TinyDepthGatePlanes tinyDepthGatePlanes(
    const float inverse[3], float slope, float clearance, float max_depth,
    const TinyDepthGatePoint *path, unsigned points) {
  TinyDepthGatePlanes out={0};
  if (!inverse || !path || points<2 || !isfinite(slope) || slope<.05f || slope>2.f ||
      !isfinite(clearance) || clearance<0.f || clearance>2.f ||
      !isfinite(max_depth) || max_depth<=clearance) return out;
  for (unsigned k=0;k<points;++k)
    if (!isfinite(path[k].x) || !isfinite(path[k].y)) return out;
  for (unsigned i=0;i<3;++i) {
    if (!isfinite(inverse[i]) || inverse[i]<=0.f) return out;
    out.depth[i]=fminf(1.f/inverse[i],max_depth);
  }
  out.valid=true;
  /* u=80-fx*y/x. Match the actual [0,53), [53,106), [106,160) packet strips.
   * raySlope remains the calibration parameter: (160/3)/fx. */
  const float fx=(160.f/3.f)/slope;
  const float edges[4]={0.f,53.f,106.f,160.f};
  float first[3]={INFINITY,INFINITY,INFINITY};
  TinyDepthGatePoint hit[3]={{0}};
  for (unsigned sector=0;sector<3;++sector) {
    /* A clipped far reading supplies no observed surface within our range. */
    if (1.f/inverse[sector]>=max_depth) continue;
    const float qlo=(80.f-edges[sector+1])/fx;
    const float qhi=(80.f-edges[sector])/fx;
    for (unsigned k=1;k<points;++k) {
      float lo=0.f,hi=1.f;
      const TinyDepthGatePoint p=path[k-1],q=path[k];
      if (!tinyDepthGateClip(p,q,1,0,.001f,&lo,&hi) ||
          !tinyDepthGateClip(p,q,-qlo,1,0,&lo,&hi) ||
          !tinyDepthGateClip(p,q,qhi,-1,0,&lo,&hi) ||
          !tinyDepthGateClip(p,q,1,0,out.depth[sector]-clearance,&lo,&hi)) continue;
      first[sector]=(float)(k-1)+lo;
      const float x=p.x+lo*(q.x-p.x),y=p.y+lo*(q.y-p.y);
      hit[sector].x=out.depth[sector];
      hit[sector].y=y*out.depth[sector]/fmaxf(x,.001f);
      break;
    }
  }
  /* The solver has two plane slots. Keep the first two encountered sectors. */
  for (unsigned j=0;j<2;++j) {
    unsigned sector=0;
    for(unsigned i=1;i<3;++i) if(first[i]<first[sector]) sector=i;
    if (!isfinite(first[sector])) break;
    first[sector]=INFINITY;
    float left=out.depth[sector],right=out.depth[sector];
    for(unsigned i=0;i<sector;++i) left=fmaxf(left,out.depth[i]);
    for(unsigned i=sector+1;i<3;++i) right=fmaxf(right,out.depth[i]);
    /* Only steer toward a side with measurably more clearance. Equal depths
     * produce a front-facing stop plane, not an arbitrary turn. */
    float ny=0.f;
    if(left>right+.15f) ny=-.5f; else if(right>left+.15f) ny=.5f;
    const float nx=sqrtf(1.f-ny*ny);
    out.nx[j]=nx;out.ny[j]=ny;
    out.b[j]=nx*hit[sector].x+ny*hit[sector].y-clearance;
    out.mode|=1u<<sector; ++out.count;
  }
  return out;
}
#endif
