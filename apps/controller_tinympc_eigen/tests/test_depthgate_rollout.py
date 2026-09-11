#!/usr/bin/env python3
"""Quantify production-cache DepthGate projected references; not a flight simulation."""
from pathlib import Path
import subprocess
import tempfile
import os
workspace=tempfile.TemporaryDirectory(prefix='mpc-depthgate-rollout-')
out=Path(workspace.name)
app=Path(__file__).resolve().parents[1];s=(app/'src/controller_tinympc.cpp').read_text()
def fn(m):
 a=s.index(m);i=s.index('{',a)+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[a:i]
t='#include <cstdlib>\n#include <cstdio>\n#include <cmath>\n#include "tinympc_generated_params.h"\n#include "tinympc_attitude_cache.h"\n#include "tinympc/tinympc.h"\n#include "tinympc/position_projection.h"\nusing namespace Eigen;\n#undef NHORIZON\n#define NHORIZON 25\n#define DT .02f\n'
t+=s[s.index('static MatrixNf A;'):s.index('// Helper variables')]
for m in ['static void loadGeneratedSolverData(void)', 'static void resetMpcInputWarmStart()', 'static void initializeMpcSolver() {']: t+=fn(m)+'\n'
t+=r'''int main(int argc,char**argv){
 const char* names[]={"clear","wall","left_center","two_planes"};
 for(int mode=0;mode<4;mode++) {
 initializeMpcSolver();if(argc>1)stgs.max_iter=atoi(argv[1]);x0.setZero();x0(6)=std::getenv("DG_INITIAL_SPEED")?atof(std::getenv("DG_INITIAL_SPEED")):.5f;
 for(int k=0;k<NHORIZON;k++) {
   data.count_xy_hs[k]=mode==0?0:(mode==3?2:1);
   data.a_xy_hs[k][0]=mode==2?Vector2f(1,-1).normalized():Vector2f(1,0);
   data.b_xy_hs[k][0]=argc>2?atof(argv[2]):.1f; // Obstacle .6m away less .5m clearance.
   if(mode==3){data.a_xy_hs[k][0]=Vector2f(1,1).normalized();
     data.a_xy_hs[k][1]=Vector2f(1,-1).normalized();data.b_xy_hs[k][1]=data.b_xy_hs[k][0];}
 }
 Vector2f refs[NHORIZON];
 for(int k=0;k<NHORIZON;k++) {
   const float targetSpeed=std::getenv("DG_TARGET_SPEED")?atof(std::getenv("DG_TARGET_SPEED")):.5f;
   Vector2f target(targetSpeed*(k*DT),0);
   if(!tiny_ProjectXY(target,data.a_xy_hs[k],data.b_xy_hs[k],data.count_xy_hs[k],
       Vector2f(-100,-100),Vector2f(100,100),Vector2f(1,1),refs[k]))std::abort();
 }
 for(int k=0;k<NHORIZON;k++) {
   Xref[k].setZero();Xref[k].head<2>()=refs[k];
   if(k<NHORIZON-1)Xref[k].segment<2>(6)=(refs[k+1]-refs[k])/DT;
 }
 PRODUCTION_SOLVE_BLOCK
 VectorNf pred=x0;float violation=0,slackViolation=0,maxTilt=0,maxRate=0;
 for(int k=1;k<NHORIZON;k++){
   pred=(A*pred+B*ZU_new[k-1]+f).eval();
   if(!pred.allFinite())std::abort();
   maxTilt=fmaxf(maxTilt,2*atanf(pred.segment<2>(3).norm()));
   maxRate=fmaxf(maxRate,pred.segment<3>(9).cwiseAbs().maxCoeff());
   for(int j=0;j<data.count_xy_hs[k];j++){
     violation=fmaxf(violation,data.a_xy_hs[k][j].dot(pred.head<2>())-data.b_xy_hs[k][j]);
     slackViolation=fmaxf(slackViolation,data.a_xy_hs[k][j].dot(ZX_new[k].head<2>())-data.b_xy_hs[k][j]);
   }
 }
 printf("%s iter=%d terminal=(%.4f,%.4f) velocity=(%.4f,%.4f) rolloutViolation=%.6f slackViolation=%.6f pri=%.6f\n",names[mode],stgs.max_iter,pred(0),pred(1),pred(6),pred(7),violation,slackViolation,info.pri_res);
 printf("tilt=%.3fdeg rate=%.3fdeg/s\n",maxTilt*57.2957795f,maxRate*57.2957795f);
 if(slackViolation>2e-5f||data.xy_hs_projection_failed)std::abort();
 if(std::getenv("DG_ACCEPT") &&
     (violation>.01f || maxTilt>15.f/57.2957795f || maxRate>120.f/57.2957795f))std::abort();
 if(std::getenv("DG_HOLD_CHECK")){
  for(int k=0;k<NHORIZON;k++){Xref[k].setZero();if(k<NHORIZON-1)Uref[k]=ug;}
  tiny_UpdateLinearCost(&work);tiny_SolveAdmm(&work);
  pred=x0;violation=maxTilt=maxRate=0;
  for(int k=1;k<NHORIZON;k++){
   pred=(A*pred+B*ZU_new[k-1]+f).eval();
   maxTilt=fmaxf(maxTilt,2*atanf(pred.segment<2>(3).norm()));
   maxRate=fmaxf(maxRate,pred.segment<3>(9).cwiseAbs().maxCoeff());
   for(int j=0;j<data.count_xy_hs[k];j++)violation=fmaxf(violation,data.a_xy_hs[k][j].dot(pred.head<2>())-data.b_xy_hs[k][j]);
  }
  printf("holdFallback violation=%.6f tilt=%.3f rate=%.3f\n",violation,maxTilt*57.2957795f,maxRate*57.2957795f);
 }

 }
}
'''
start=s.index("    // Cold-start both input and state ADMM variables together.")
end=s.index("    mpc_constraints[0]",start)
t=t.replace(" PRODUCTION_SOLVE_BLOCK",s[start:end])
if os.environ.get('DG_CACHE'):
 t=t.replace('tinympc_attitude_cache.h',os.environ['DG_CACHE']).replace('attitude_cache_', 'depthgate_cache_')
if os.environ.get('DG_RAMP_LOOP'):
 import re,numpy as np
 from scipy.linalg import sqrtm
 params=(app/'src/tinympc_generated_params.h').read_text()
 def arr(name,shape):
  body=re.search(r'tinympc_generated_'+name+r'\[\d+\] = \{(.*?)\};',params,re.S)[1]
  return np.array([float(v.strip().rstrip('f')) for v in body.split(',') if v.strip()]).reshape(shape)
 augmented=np.eye(17);augmented[:12,:12]=arr('A',(12,12));augmented[:12,12:16]=arr('B',(12,4));augmented[:12,16]=arr('f',(12,))
 half=sqrtm(augmented);assert np.max(np.abs(half.imag))<1e-6;half=half.real
 def cpp(values):return ','.join(format(float(v),'.9g')+('f' if '.' in format(float(v),'.9g') or 'e' in format(float(v),'.9g') else '.0f') for v in values.flatten())
 loop=r''' initializeMpcSolver();stgs.max_iter=5;x0.setZero();
 MatrixNf ah;MatrixNMf bh;VectorNf fh;
 const float av[]={AV};const float bv[]={BV};const float fv[]={FV};
 for(int i=0;i<12;i++){fh(i)=fv[i];for(int j=0;j<12;j++)ah(i,j)=av[i*12+j];for(int j=0;j<4;j++)bh(i,j)=bv[i*4+j];}
 float distance=0,maxTilt=0,maxRate=0;
 for(int step=0;step<300;step++){
  const float speed=fminf(.5f,.5f*.01f*(step+1));
  x0.head<3>().setZero();
  for(int k=0;k<NHORIZON;k++){data.count_xy_hs[k]=0;data.en_hs[k]=0;Xref[k].setZero();Xref[k](0)=speed*(k*DT);if(k<NHORIZON-1){Xref[k](6)=speed;Uref[k]=ug;}}
  RAMP_SOLVE
  x0=(ah*x0+bh*ZU_new[0]+fh).eval();distance+=x0(0);
  maxTilt=fmaxf(maxTilt,2*atanf(x0.segment<2>(3).norm()));maxRate=fmaxf(maxRate,x0.segment<3>(9).cwiseAbs().maxCoeff());
  if(!x0.allFinite())std::abort();
  if(step%50==49)printf("RAMP t=%.2f distance=%.4f speed=%.4f tilt=%.3f rate=%.3f\n",(step+1)*.01f,distance,x0(6),maxTilt*57.2957795f,maxRate*57.2957795f);
 }
'''
 loop=loop.replace('AV',cpp(half[:12,:12])).replace('BV',cpp(half[:12,12:16])).replace('FV',cpp(half[:12,16])).replace('RAMP_SOLVE',s[start:end])
 at=t.rfind('}');t=t[:at]+loop+t[at:]
(out/'test.cpp').write_text(t)
solver=app/'TinyMPC-ADMM/src/tinympc'
c=[os.environ.get('CXX','clang++'),'-std=c++17','-O2','-Wno-deprecated-declarations','-DEIGEN_INITIALIZE_MATRICES_BY_ZERO','-DEIGEN_NO_MALLOC','-I'+str(app/'src'),'-I'+str(app/'TinyMPC-ADMM/ext/Eigen'),'-I'+str(app/'TinyMPC-ADMM/src'),str(out/'test.cpp')]+[str(solver/(n+'.cpp')) for n in ['utils','model','auxil','cost_lqr','lqr','constraint_linear','admm','rho_benchmark']]+['-o',str(out/'test')]
subprocess.run(c,check=True)
for iterations in map(int,os.environ.get('DG_ITERS','5').split(',')):
 for bound in (.1,.3):
  print(f'iterations={iterations} free_boundary={bound}',flush=True)
  subprocess.run([str(out/'test'),str(iterations),str(bound)],check=True)
workspace.cleanup()
