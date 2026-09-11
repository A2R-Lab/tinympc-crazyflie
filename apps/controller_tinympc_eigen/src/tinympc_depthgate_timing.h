#ifndef TINYMPC_DEPTHGATE_TIMING_H
#define TINYMPC_DEPTHGATE_TIMING_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* The recovered DDND runs synchronously in about 1.45 s. Keep ~4.75 s of
 * measured poses, rather than extrapolating that far from current velocity.
 * All timestamps here use the STM32 clock; GAP timestamps are telemetry only. */
#define DG_POSE_CAPACITY 96u
#define DG_MAX_INFERENCE_US 2000000u
#define DG_MAX_RECEIVE_AGE_MS 1800u
#define DG_UART_MS 7u /* 80 bytes, 8N1 at 115200 baud, rounded up */
#define DG_MAX_SPEED .2f
typedef struct { uint32_t ms; float x, y, yaw; } TinyDepthGatePose;
typedef struct {
  TinyDepthGatePose poses[DG_POSE_CAPACITY];
  unsigned next, count;
} TinyDepthGateHistory;

static inline bool tinyDepthGateFresh(bool valid, uint32_t age, uint32_t inference) {
  return valid && age <= DG_MAX_RECEIVE_AGE_MS &&
      inference > 0 && inference <= DG_MAX_INFERENCE_US;
}
static inline void tinyDepthGateRecord(TinyDepthGateHistory *h,
                                      TinyDepthGatePose pose) {
  if (!isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.yaw)) return;
  if (h->count && pose.ms-h->poses[(h->next+DG_POSE_CAPACITY-1)%DG_POSE_CAPACITY].ms < 50u) return;
  h->poses[h->next]=pose;
  h->next=(h->next+1)%DG_POSE_CAPACITY;
  if (h->count<DG_POSE_CAPACITY) ++h->count;
}
static inline bool tinyDepthGateCapturePose(const TinyDepthGateHistory *h,
    uint32_t now, uint32_t age, uint32_t inference, TinyDepthGatePose *out) {
  if (!tinyDepthGateFresh(true,age,inference)) return false;
  const uint32_t delay=age+(inference+999u)/1000u+DG_UART_MS;
  for (unsigned i=1;i<h->count;++i) {
    const TinyDepthGatePose b=h->poses[(h->next+DG_POSE_CAPACITY-i)%DG_POSE_CAPACITY];
    const TinyDepthGatePose a=h->poses[(h->next+DG_POSE_CAPACITY-i-1)%DG_POSE_CAPACITY];
    const uint32_t older=now-a.ms, newer=now-b.ms;
    if (older<delay || newer>delay) continue;
    if (b.ms-a.ms>100u) return false; /* No interpolation across control gaps. */
    const float t=(float)(older-delay)/(float)(b.ms-a.ms);
    out->ms=now-delay;
    out->x=a.x+t*(b.x-a.x); out->y=a.y+t*(b.y-a.y);
    out->yaw=a.yaw+t*atan2f(sinf(b.yaw-a.yaw),cosf(b.yaw-a.yaw));
    return true;
  }
  return false;
}
#endif
