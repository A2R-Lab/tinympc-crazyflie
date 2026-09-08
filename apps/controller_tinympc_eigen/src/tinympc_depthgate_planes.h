#ifndef TINYMPC_DEPTHGATE_PLANES_H
#define TINYMPC_DEPTHGATE_PLANES_H
#include <math.h>
#include <stdbool.h>

/* Optical-axis depth, body +X forward / +Y left. Vertical planes have nz=0.
 * A sector statistic has no pixel bearing: its center ray is an approximation.
 * The allowed side is nx*x+ny*y<=b, including the clearance offset. */
typedef struct {
  bool valid;
  unsigned count, mode; /* 0 clear, 1 left-center, 2 center-right, 3 both */
  float depth[3], nx[2], ny[2], b[2];
} TinyDepthGatePlanes;

static inline TinyDepthGatePlanes tinyDepthGatePlanes(
    const float inverse[3], float slope, float clearance,
    float activation, float max_depth, unsigned previous_mode) {
  TinyDepthGatePlanes out = {0};
  if (!inverse || !isfinite(slope) || slope < .05f || slope > 2.f ||
      !isfinite(clearance) || clearance < 0.f || clearance > 2.f ||
      !isfinite(activation) || activation <= clearance ||
      !isfinite(max_depth) || max_depth < activation) return out;
  for (int i=0;i<3;++i) {
    if (!isfinite(inverse[i]) || inverse[i] <= 0.f) return out;
    out.depth[i] = fminf(1.f/inverse[i], max_depth);
  }
  out.valid = true;
  const float l=out.depth[0], c=out.depth[1], r=out.depth[2];
  if (fminf(l,fminf(c,r)) > activation) return out;
  const float split = previous_mode == 3 ? .05f : .15f;
  if (c > l+split && c > r+split && l <= activation && r <= activation)
    out.mode=3;
  else if (l < c-.05f && l <= r) out.mode=1;
  else if (r < c-.05f && r < l) out.mode=2;
  else if (fabsf(l-r)<.15f && (previous_mode==1 || previous_mode==2))
    out.mode=previous_mode;
  else out.mode = l>=r ? 1 : 2;
  for (unsigned pair=1;pair<=2;++pair) {
    if (!(out.mode & pair)) continue;
    const float side=pair==1?l:r;
    float nx=slope*side;
    float ny=pair==1 ? c-side : side-c;
    const float norm=hypotf(nx,ny);
    nx/=norm; ny/=norm;
    unsigned j=out.count++;
    out.nx[j]=nx;out.ny[j]=ny;
    out.b[j]=nx*c-clearance;
  }
  return out;
}
#endif
