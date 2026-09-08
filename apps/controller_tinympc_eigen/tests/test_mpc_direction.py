#!/usr/bin/env python3
"""Compile the actual MPC and reference transforms; verify body +X commands.

No flight or actuator validation is implied by this host regression.
"""
from pathlib import Path
import subprocess
import os
import re
import tempfile
app=Path(__file__).resolve().parents[1]
workspace=tempfile.TemporaryDirectory(prefix='tinympc-direction-')
out=Path(workspace.name)
src=(app/'src/controller_tinympc.cpp').read_text()
def function(marker):
 start=src.index(marker);op=src.index('{',start);end=op+1;depth=1
 while depth:
  depth+=(src[end]=='{')-(src[end]=='}');end+=1
 return src[start:end]
text='#include <cstdio>\n#include <cstdlib>\n#include "tinympc_generated_params.h"\n#include "tinympc/tinympc.h"\nusing namespace Eigen;\n#undef NHORIZON\n#define NHORIZON TINYMPC_GENERATED_HORIZON_KNOTS\n#define DT TINYMPC_GENERATED_MODEL_DT_S\n'
text+=src[src.index('static MatrixNf A;'):src.index('// Helper variables')]
for marker in ['static void loadGeneratedSolverData(void)', 'static void resetMpcInputWarmStart()', 'static void initializeMpcSolver() {']:text+=function(marker)+'\n'
text+=function('struct MpcLocalFrame {')+';\n'
text+=function('static Eigen::Vector3f worldVectorToLocal(')+'\n'
# Compile both actual forward expressions (normal update and accepted start).
forward_expressions=re.findall(r'(?:Eigen::Vector3f forward\(|forward = Eigen::Vector3f\()([^;]+)\);',src)
if len(forward_expressions)!=2:
 raise RuntimeError('Expected two production forward-vector expressions')
text+='static void checkForwardGeometry() {\n'
text+='for(float esp_test_yaw : {-3.0f, -1.57f, 0.0f, 0.7f, 1.57f, 3.0f}) {\n'
text+='MpcLocalFrame frame = {0,0,0,esp_test_yaw,cosf(esp_test_yaw),sinf(esp_test_yaw)};\n'
for expression in forward_expressions:
 text+='{const Eigen::Vector3f world('+expression+'); const auto local=worldVectorToLocal(frame,world); if(!local.allFinite() || (local-Eigen::Vector3f(1,0,0)).norm()>1e-5f) std::abort();}\n'
text+='}}\n'
text+='''int main(){
 checkForwardGeometry();
 VectorMf positive_input;
 for(float goal : {0.0f, 0.1f, -0.1f}) {
  initializeMpcSolver();x0.setZero();
  for(int k=0;k<NHORIZON;k++){Xref[k].setZero();Xref[k](0)=goal;}
  for(int k=0;k<NHORIZON-1;k++)r_tilde[k].setZero();
  tiny_UpdateLinearCost(&work);tiny_SolveAdmm(&work);
  VectorMf thrust;
  for(int i=0;i<4;i++)thrust(i)=ZU_new[0](i)+tinympc_generated_physical_hover_thrust[i];
  VectorNf delta=B*ZU_new[0];
  if(!delta.allFinite() || !thrust.allFinite())std::abort();
  if(goal>0)positive_input=ZU_new[0];
  if(goal<0 && (positive_input+ZU_new[0]).cwiseAbs().maxCoeff()>1e-7f)std::abort();
  const float pitch=-thrust(0)+thrust(1)+thrust(2)-thrust(3);
  if(goal!=0 && pitch*goal<=0)std::abort();
  if(goal==0 && (ZU_new[0].cwiseAbs().maxCoeff()>1e-8f || (thrust.array()-thrust(0)).abs().maxCoeff()>1e-8f))std::abort();
  if(goal!=0 && (delta(10)*goal<=0 || delta(6)*goal<=0))std::abort();
  printf("goalX=%+.3f deltaU=[%+.9f,%+.9f,%+.9f,%+.9f] physical=[%.9f,%.9f,%.9f,%.9f] modelDroll=%+.9f Dpitch=%+.9f Dvx=%+.9f\\n",goal,ZU_new[0](0),ZU_new[0](1),ZU_new[0](2),ZU_new[0](3),thrust(0),thrust(1),thrust(2),thrust(3),delta(9),delta(10),delta(6));
  printf("physical roll-pattern=%+.9f pitch-pattern=%+.9f yaw-pattern=%+.9f\\n", -thrust(0)-thrust(1)+thrust(2)+thrust(3),-thrust(0)+thrust(1)+thrust(2)-thrust(3),-thrust(0)+thrust(1)-thrust(2)+thrust(3));
 }
}
'''
(out/'solver_response.cpp').write_text(text)
solver=app/'TinyMPC-ADMM/src/tinympc'
cmd=[os.environ.get('CXX','clang++'),'-std=c++17','-O2','-DNDEBUG','-Wno-deprecated-declarations','-DEIGEN_INITIALIZE_MATRICES_BY_ZERO','-DEIGEN_NO_MALLOC','-I'+str(app/'src'),'-I'+str(app/'TinyMPC-ADMM/ext/Eigen'),'-I'+str(app/'TinyMPC-ADMM/src'),str(out/'solver_response.cpp')]+[str(solver/(n+'.cpp')) for n in ['utils','model','auxil','cost_lqr','lqr','constraint_linear','admm','rho_benchmark']]+['-o',str(out/'solver_response')]
subprocess.run(cmd,check=True);result=subprocess.check_output([str(out/'solver_response')],text=True);print(result);print('MPC direction and production forward geometry passed');workspace.cleanup()
