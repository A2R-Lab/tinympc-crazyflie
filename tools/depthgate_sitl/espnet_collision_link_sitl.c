/* Synthetic observation source for controller integration testing, never hardware. */
#include "espnet_collision_link.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>

void espnetCollisionLinkInit(void) {}
bool espnetCollisionLinkGetLatest(EspnetCollisionObservation *o) {
  if (!o) return false;
  memset(o,0,sizeof(*o)); o->received_age_ms=UINT32_MAX; return false;
}
bool espnetGateLinkGetLatest(EspnetGateObservation *o) {
  if (!o) return false;
  memset(o,0,sizeof(*o)); o->received_age_ms=UINT32_MAX; return false;
}
bool depthGateLinkGetLatest(DepthGateObservation *o) {
  if (!o) return false;
  memset(o,0,sizeof(*o)); o->received_age_ms=UINT32_MAX;
  /* Explicit opt-in; no synthetic perception silently enabled by default. */
  const char *text=getenv("DEPTHGATE_SITL_DEPTHS");
  if (!text) return false;
  for (unsigned i=0;i<3;i++) {
    char *end; const float d=strtof(text,&end);
    if (end==text || !(d>0.f)) return false;
    o->inverse_depth[i]=1.f/d;
    text=end;
    if (i<2) { if (*text!=',') return false; ++text; }
  }
  const uint32_t now=xTaskGetTickCount()*portTICK_PERIOD_MS;
  o->valid=true; o->status=3; o->sample=now/100+1;
  o->sequence=(uint16_t)o->sample;
  o->source_timestamp_ms=now-now%100;
  o->received_age_ms=now%100; o->inference_us=83000;
  return true;
}
