#!/usr/bin/env python3
"""Host-only production-cache rollout regression; not a flight simulation."""
from pathlib import Path
import subprocess
import tempfile
import os
workspace=tempfile.TemporaryDirectory(prefix='mpc-attitude-rollout-')
out=Path(workspace.name)
app=Path(__file__).resolve().parents[1];s=(app/'src/controller_tinympc.cpp').read_text()
def fn(m):
 a=s.index(m);i=s.index('{',a)+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[a:i]
t='#include <cstdlib>\n#include <cstdio>\n#include <cmath>\n#include "tinympc_generated_params.h"\n#include "tinympc_attitude_cache.h"\n#include "tinympc/tinympc.h"\nusing namespace Eigen;\n#undef NHORIZON\n#define NHORIZON 25\n#define DT .02f\n'
t+=s[s.index('static MatrixNf A;'):s.index('// Helper variables')]
for m in ['static void loadGeneratedSolverData(void)', 'static void resetMpcInputWarmStart()', 'static void initializeMpcSolver() {']: t+=fn(m)+'\n'
t+=r'''int main(int argc,char**argv){
 const char* names[]={"hover","acceleration","brake","outside_brake","large_error","diagonal_acceleration","diagonal_brake","yaw_rate","all_rates","handoff","all_rates80","lagged_run"};
 bool failed=false;
 for(int mode=0;mode<12;mode++) {
 initializeMpcSolver();if(argc>1)stgs.max_iter=atoi(argv[1]);x0.setZero();
 if(mode==2)x0(6)=3;
 if(mode==3||mode==4){x0(6)=1.3f;x0(4)=.26f;x0(10)=1.6f;}
 if(mode==6)x0(6)=x0(7)=3/sqrtf(2);
 if(mode==7)x0(11)=2.094395102f;
 if(mode==8)x0.segment<3>(9).setConstant(2.094395102f);
 if(mode==10)x0.segment<3>(9).setConstant(1.396263402f);
 if(mode==9){x0(6)=.12f;x0(4)=tanf(5*.01745329252f/2);}
 if(mode==11){x0(6)=1.3f;x0(4)=.05f;}
 const VectorNf measurement=x0;
 for(int k=0;k<NHORIZON;k++){
 Xref[k].setZero();
 if(mode==1||mode==4||mode==11){Xref[k](0)=.02f*k*3+((mode==4||mode==11)?4.4f:0);Xref[k](6)=3;}
 if(mode==5){Xref[k](0)=Xref[k](1)= .02f*k*3/sqrtf(2);Xref[k](6)=Xref[k](7)=3/sqrtf(2);}
 }
 for(int rep=0;rep<40;rep++) {
 PRODUCTION_SOLVE_BLOCK
 VectorNf pred=x0;float tilt=0,rate=0,rollComponent=0,pitchComponent=0;
 for(int k=1;k<NHORIZON;k++){
 pred=(A*pred+B*ZU_new[k-1]+f).eval();
 if(!pred.allFinite()||!ZU_new[k-1].allFinite())std::abort();
 tilt=fmaxf(tilt,2*atanf(sqrtf(pred(3)*pred(3)+pred(4)*pred(4))/sqrtf(1+pred(5)*pred(5))));
 rollComponent=fmaxf(rollComponent,fabsf(pred(3)));
 pitchComponent=fmaxf(pitchComponent,fabsf(pred(4)));
 for(int i=9;i<=11;i++)rate=fmaxf(rate,fabsf(pred(i)));
 }
 if((x0-measurement).norm()!=0)std::abort();
 if(rep==0||rep==39)printf("%s solve%d tiltDeg=%.4f rateDeg=%.4f pri=%.5f dua=%.5f\n",names[mode],rep,tilt*57.2957795f,rate*57.2957795f,info.pri_res,info.dua_res);
 if(rep==0)printf("Rodrigues components: roll=%.5f pitch=%.5f (pitch bound %.5f)\n",rollComponent,pitchComponent,pitchRodriguesBound);
 // Outside-envelope measured initial states cannot always be made feasible.
 if(mode!=3&&mode!=4&&(rollComponent>tanf(10*.01745329252f)+1e-4f||pitchComponent>pitchRodriguesBound+1e-4f||rate>2.094395102f+1e-3f))failed=true;
 }
 }
 if(failed){fprintf(stderr,"Physical tilt/rate envelope violated in nominal regression\n");return 1;}
}
'''
start=s.index("    // Cold-start both input and state ADMM variables together.")
end=s.index("    mpc_constraints[0]",start)
t=t.replace(" PRODUCTION_SOLVE_BLOCK",s[start:end])
(out/'test.cpp').write_text(t)
solver=app/'TinyMPC-ADMM/src/tinympc'
c=['clang++','-std=c++17','-O2','-Wno-deprecated-declarations','-DEIGEN_INITIALIZE_MATRICES_BY_ZERO','-DEIGEN_NO_MALLOC','-I'+str(app/'src'),'-I'+str(app/'TinyMPC-ADMM/ext/Eigen'),'-I'+str(app/'TinyMPC-ADMM/src'),str(out/'test.cpp')]+[str(solver/(n+'.cpp')) for n in ['utils','model','auxil','cost_lqr','lqr','constraint_linear','admm','rho_benchmark']]+['-o',str(out/'test')]
subprocess.run(c,check=True)
subprocess.run([str(out/'test')],check=True)
workspace.cleanup()
