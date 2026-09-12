#ifndef TINYMPC_DEPTHGATE_HEADING_H
#define TINYMPC_DEPTHGATE_HEADING_H
#include <math.h>
/* Keep heading continuous across +/-pi; don't aim at velocity noise in hold. */
static inline float tinyDepthGateAngle(float a) { return atan2f(sinf(a),cosf(a)); }
static inline float tinyDepthGateHeading(float previous,float vx,float vy,float dt) {
  if(!isfinite(previous) || !isfinite(vx) || !isfinite(vy) || !isfinite(dt) || dt<=0.f)
    return previous;
  if(hypotf(vx,vy)<.03f) return previous;
  const float error=tinyDepthGateAngle(atan2f(vy,vx)-previous);
  const float step=1.0471975512f*dt; /* 60 degrees/s */
  return tinyDepthGateAngle(previous+fmaxf(-step,fminf(step,error)));
}
#endif
