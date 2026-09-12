#include <assert.h>
#include "tinympc_depthgate_heading.h"
int main(void) {
 const float pi=3.14159265359f;
 float yaw=0;
 for(int i=0;i<200;++i) {
  float next=tinyDepthGateHeading(yaw,0,1,.01f);
  assert(fabsf(tinyDepthGateAngle(next-yaw))<=1.047198f*.01f+1.e-6f);yaw=next;
 }
 assert(fabsf(yaw-pi/2)<1.e-5f);
 assert(tinyDepthGateHeading(yaw,0,0,.01f)==yaw);
 assert(tinyDepthGateHeading(yaw,.001f,-.001f,.01f)==yaw);
 float start=179*pi/180,target=-179*pi/180;
 float next=tinyDepthGateHeading(start,cosf(target),sinf(target),.01f);
 assert(tinyDepthGateAngle(next-start)>0); // short arc through +pi
 assert(tinyDepthGateHeading(yaw,NAN,1,.01f)==yaw);
 assert(tinyDepthGateHeading(yaw,1,1,0)==yaw);
 // Tangent following must not overwrite the separate mission direction.
 yaw=0;for(int i=0;i<100;++i)yaw=tinyDepthGateHeading(yaw,1,-1,.01f);
 assert(fabsf(yaw+pi/4)<1.e-5f);
 return 0;
}
