#include <assert.h>
#include <stdio.h>
#include "../src/tinympc_gate_servo.h"
static TinyGateInput input(void) {
  TinyGateInput g = {0};
  g.fresh = true; g.pose_aligned = true; g.edge_mask = 15;
  g.rail[0] = g.rail[1] = .9f;
  g.x[0] = g.x[2] = .2f; g.x[1] = g.x[3] = .8f;
  g.y[0] = g.y[1] = .2f; g.y[2] = g.y[3] = .8f;
  for (unsigned i=0;i<4;++i) g.confidence[i] = .1f;
  return g;
}
static void step(TinyGateServo *s,TinyGateInput *g,float v,float distance) {
  ++g->sample; tinyGateStep(s,g,true,v,distance,.05f);
}
int main(void) {
  TinyGateServo s = {0}; TinyGateInput g = input();
  // Reusing one image cannot satisfy temporal confirmation.
  g.sample=1;
  for(int i=0;i<50;++i) tinyGateStep(&s,&g,true,0,0,.01f);
  assert(s.phase==GATE_SEARCH && s.acquire_frames==1);
  step(&s,&g,1,0); step(&s,&g,1,0);
  assert(s.phase==GATE_SLOW && s.forward==0);
  step(&s,&g,.1f,0); assert(s.phase==GATE_ALIGN);
  g.pose_aligned=false;
  for(int i=0;i<4;++i) step(&s,&g,0,0);
  assert(s.phase==GATE_ALIGN);
  g.pose_aligned=true;
  for(int i=0;i<3;++i) step(&s,&g,0,0);
  assert(s.phase==GATE_PASS && s.forward==.5f);
  g.fresh=false; step(&s,&g,.5f,.9f);
  assert(s.phase==GATE_PASS); // Image naturally disappears in passage.
  step(&s,&g,.5f,1); assert(s.phase==GATE_COOLDOWN);
  g=input(); g.sample=30;
  for(int i=0;i<30;++i) step(&s,&g,.5f,1);
  assert(s.phase==GATE_COOLDOWN); // Same gate must not retrigger.
  g.rail[0]=g.rail[1]=0;
  for(int i=0;i<21;++i) step(&s,&g,.5f,1);
  assert(s.phase==GATE_SEARCH);
  // Single matching rail may seek; it cannot claim a gate center.
  s=(TinyGateServo){0}; g=input(); g.rail[1]=0;
  for(int i=0;i<4;++i) step(&s,&g,0,0);
  step(&s,&g,0,0);
  assert(s.phase==GATE_ALIGN && s.lateral<0 && !s.centered);
  g.rail[0]=0;g.rail[1]=.9f;step(&s,&g,0,0);assert(s.lateral>0);
  g.fresh=false;step(&s,&g,0,0);assert(s.phase==GATE_ABORT && s.reject==3);
  // Matched side must agree with the corner label and independent edge.
  s=(TinyGateServo){0};g=input();g.rail[1]=0;g.edge_mask=10;
  for(int i=0;i<5;++i)step(&s,&g,0,0);
  assert(s.phase==GATE_SEARCH && s.matched==0);
  g=input();g.confidence[0]=g.confidence[2]=.0025f;
  tinyGateGeometry(&s,&g);assert(s.matched==2 && !s.centered);
  g=input();g.x[0]=.9f;tinyGateGeometry(&s,&g);assert(!s.centered);
  // Image right/down => physical right/down, with bounded speed.
  s=(TinyGateServo){.phase=GATE_ALIGN};g=input();
  for(int i=0;i<4;++i){g.x[i]+=.1f;g.y[i]+=.1f;}
  step(&s,&g,0,0);assert(s.lateral<0 && s.vertical<0 && hypotf(s.lateral,s.vertical)<=.5f);
  s.elapsed=10;step(&s,&g,0,0);assert(s.phase==GATE_ABORT && s.reject==2);
  s=(TinyGateServo){0};step(&s,&g,NAN,0);assert(s.phase==GATE_ABORT);
  tinyGateStep(&s,&g,false,0,0,.05f);assert(s.phase==GATE_SEARCH);
  puts("gate geometry/state tests passed");
}
