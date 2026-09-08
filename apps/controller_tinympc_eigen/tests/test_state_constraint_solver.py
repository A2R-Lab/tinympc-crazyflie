#!/usr/bin/env python3
"""Exercise production ADMM penalty rebuilding and fixed-initial-state projection."""
from pathlib import Path
import os
import subprocess
import tempfile
app = Path(__file__).resolve().parents[1]
source = r'''
#include <cassert>
#include <cmath>
#include "tinympc/tinympc.h"
using namespace Eigen;
int main() {
  constexpr int N=3;
  tiny_Model model={}; model.nhorizon=N;
  tiny_AdmmData data={}; data.model=&model;
  tiny_AdmmSettings stgs={}; stgs.en_cstr_states=1;
  tiny_AdmmSolution soln={}; tiny_AdmmInfo info={}; tiny_AdmmWorkspace w={};
  w.data=&data;w.stgs=&stgs;w.soln=&soln;w.info=&info;w.rho=2;
  MatrixNf Q=MatrixNf::Identity()*3,P=MatrixNf::Identity()*7;
  MatrixMf R=MatrixMf::Identity();
  VectorNf q[N],p[N],ref[N],zx[N],oldzx[N],yx[N],x[N];
  VectorMf r[N],uref[N];
  VectorNf lo=VectorNf::Constant(-.2f),hi=VectorNf::Constant(.2f);
  data.Q=&Q;data.R=&R;data.q=q;data.r=r;data.Xref=ref;data.Uref=uref;
  data.lcx=&lo;data.ucx=&hi;
  soln.p=p;soln.Pinf=&P;soln.X=x;soln.YX=yx;w.ZX_new=zx;w.ZX=oldzx;
  for(int k=0;k<N;k++) {ref[k].setConstant(1);uref[k].setZero();zx[k].setConstant(.5f);yx[k].setConstant(.1f);x[k].setConstant(.8f);oldzx[k]=zx[k];}
  tiny_UpdateConstrainedLinearCost(&w);
  assert(std::abs(q[1](0)+3.8f)<1e-6f);
  assert(std::abs(p[N-1](0)+5.8f)<1e-6f);
  tiny_UpdateConstrainedLinearCost(&w);
  assert(std::abs(q[1](0)+3.8f)<1e-6f);
  assert(std::abs(p[N-1](0)+5.8f)<1e-6f);
  VectorNf weights=VectorNf::Constant(3);data.state_constraint_weights=&weights;
  tiny_UpdateConstrainedLinearCost(&w);
  assert(std::abs(q[1](0)+5.4f)<1e-6f);
  assert(std::abs(p[N-1](0)+3.4f)<1e-6f);
  tiny_UpdateConstrainedLinearCost(&w);
  assert(std::abs(q[1](0)+5.4f)<1e-6f);
  data.state_constraint_weights=nullptr;
  UpdateSlackDual(&w);
  assert((zx[0]-x[0]).norm()==0);assert(yx[0].norm()==0);
  assert(std::abs(zx[1](0)-.2f)<1e-6f);
  assert(std::abs(yx[1](0)-.7f)<1e-6f);
  // Exercise the actual production slack update with two intersecting planes.
  lo.head<2>().setConstant(-10);hi.head<2>().setConstant(10);
  for(int k=1;k<N;k++) {
    x[k].setConstant(.8f);x[k](0)=2;x[k](1)=0;yx[k].setZero();
    data.count_xy_hs[k]=2;
    data.a_xy_hs[k][0]=Vector2f(1,1);data.b_xy_hs[k][0]=1;
    data.a_xy_hs[k][1]=Vector2f(1,-1);data.b_xy_hs[k][1]=1;
  }
  UpdateSlackDual(&w);
  assert(!data.xy_hs_projection_failed);
  assert((zx[1].head<2>()-Vector2f(1,0)).norm()<1e-6f);
  assert(std::abs(zx[1](3)-.2f)<1e-6f); // Tilt box still applies.
  assert(std::abs(zx[1](10)-.2f)<1e-6f); // Rate box still applies.
  assert((zx[0]-x[0]).norm()==0);
  // One plane with a weighted metric: project (2,0) to x+y=1.
  weights.setOnes();weights(0)=4;data.state_constraint_weights=&weights;
  yx[1].setZero();data.count_xy_hs[1]=1;
  UpdateSlackDual(&w);
  assert((zx[1].head<2>()-Vector2f(1.8f,-.8f)).norm()<1e-5f);
  // Oblique edge intersects a finite XY box; both sets must hold together.
  hi(1)=.25f;yx[1].setZero();x[1](0)=0;x[1](1)=2;
  data.a_xy_hs[1][0]=Vector2f(1,1);data.b_xy_hs[1][0]=0;
  UpdateSlackDual(&w);
  assert((zx[1].head<2>()-Vector2f(-.25f,.25f)).norm()<1e-5f);
  hi(1)=10;
  // Redundant parallel planes reduce to the tighter boundary.
  data.count_xy_hs[1]=2;yx[1].setZero();x[1](0)=2;x[1](1)=0;
  data.a_xy_hs[1][0]=Vector2f(1,0);data.b_xy_hs[1][0]=1;
  data.a_xy_hs[1][1]=Vector2f(2,0);data.b_xy_hs[1][1]=1;
  UpdateSlackDual(&w);assert(!data.xy_hs_projection_failed);
  assert(std::abs(zx[1](0)-.5f)<1e-6f);
  // Parallel contradictory planes report infeasibility instead of NaN.
  data.count_xy_hs[1]=2;yx[1].setZero();
  data.a_xy_hs[1][0]=Vector2f(1,0);data.b_xy_hs[1][0]=0;
  data.a_xy_hs[1][1]=Vector2f(-1,0);data.b_xy_hs[1][1]=-1;
  UpdateSlackDual(&w);assert(data.xy_hs_projection_failed);assert(zx[1].allFinite());
  // Restore the original no-plane regression scenario.
  data.state_constraint_weights=nullptr;weights.setConstant(3);
  for(int k=1;k<N;k++)data.count_xy_hs[k]=0;
  lo.head<2>().setConstant(-.2f);hi.head<2>().setConstant(.2f);
  for(int k=1;k<N;k++){x[k]=zx[k];oldzx[k]=zx[k];}
  oldzx[0].setConstant(1000);x[0].setConstant(1000);
  ComputePrimalResidual(&w);ComputeDualResidual(&w);
  assert(info.pri_res==0);assert(info.dua_res==0);

  // Full fixed-iteration solves must be bit-identical with optional baselines.
  // Repeat with changed references/rho to expose a stale per-solve cache.
  MatrixNf A=MatrixNf::Identity()*.95f, Am=A.transpose();
  MatrixNMf B=MatrixNMf::Constant(.01f), coeff=MatrixNMf::Zero();
  MatrixMNf K=MatrixMNf::Constant(.02f);
  MatrixMf inverse=MatrixMf::Identity()*.2f;
  VectorNf affine=VectorNf::Zero(), qbase[N], terminal;
  VectorMf input[N],yu[N],zu[N],oldzu[N],feed[N],rt[N],qu;
  VectorMf ulo=VectorMf::Constant(-.1f),uhi=VectorMf::Constant(.1f);
  VectorNf initial=VectorNf::Constant(.05f);
  model.nstates=NSTATES;model.ninputs=NINPUTS;model.A=&A;model.B=&B;model.f=&affine;
  data.x0=&initial;data.ucu=&uhi;data.lcu=&ulo;data.r_tilde=rt;
  soln.U=input;soln.YU=yu;soln.d=feed;soln.Kinf=&K;
  w.ZU_new=zu;w.ZU=oldzu;w.Qu=&qu;w.Quu_inv=&inverse;w.AmBKt=&Am;w.coeff_d2p=&coeff;
  stgs.en_cstr_inputs=1;stgs.max_iter=8;stgs.check_termination=1;
  stgs.tol_abs_prim=0;stgs.tol_abs_dual=0;
  for(int scenario=0;scenario<4;scenario++) {
    stgs.adaptive_horizon=scenario==3?1:0;
    data.model_s=&model;soln.Kinf_s=&K;soln.Pinf_s=&P;
    w.Quu_inv_s=&inverse;w.AmBKt_s=&Am;w.coeff_d2p_s=&coeff;
    VectorNf expected_x[N], expected_yx[N], expected_q[N], expected_p[N];
    VectorMf expected_u[N], expected_zu[N], expected_yu[N];
    float expected_pri=0,expected_dua=0;
    for(int cached=0;cached<3;cached++) {
      data.coeff_d2p_zero=cached==2;data.coeff_d2p_s_zero=cached==2;
      data.q_base=cached?qbase:nullptr;data.terminal_base=cached?&terminal:nullptr;
      w.rho=2.f+scenario;data.state_constraint_weights=&weights;
      for(int k=0;k<N;k++) {
        ref[k].setConstant(.2f*(scenario+1)*(k+1));uref[k].setConstant(.01f*scenario);
        x[k].setZero();yx[k].setZero();zx[k].setZero();oldzx[k].setZero();
        input[k].setZero();yu[k].setZero();zu[k].setZero();oldzu[k].setZero();
      }
      tiny_SolveAdmm(&w); // Intentionally no caller-side linear-cost refresh.
      for(int k=0;k<N;k++) {
        if(!cached) {
          expected_x[k]=x[k];expected_yx[k]=yx[k];expected_p[k]=p[k];
          if(k<N-1) {expected_q[k]=q[k];expected_u[k]=input[k];expected_zu[k]=zu[k];expected_yu[k]=yu[k];}
        } else {
          assert((expected_x[k]-x[k]).norm()==0);assert((expected_yx[k]-yx[k]).norm()==0);
          assert((expected_p[k]-p[k]).norm()==0);
          if(k<N-1) {assert((expected_q[k]-q[k]).norm()==0);assert((expected_u[k]-input[k]).norm()==0);assert((expected_zu[k]-zu[k]).norm()==0);assert((expected_yu[k]-yu[k]).norm()==0);}
        }
      }
      if(!cached){expected_pri=info.pri_res;expected_dua=info.dua_res;}
      else {assert(expected_pri==info.pri_res);assert(expected_dua==info.dua_res);}
    }
  }
}
'''
with tempfile.TemporaryDirectory(prefix='mpc-state-admm-') as temp:
    root=Path(temp); cpp=root/'test.cpp'; cpp.write_text(source)
    solver=app/'TinyMPC-ADMM/src/tinympc'
    command=[os.environ.get('CXX','clang++'),'-std=c++17','-O1','-Wno-deprecated-declarations','-DEIGEN_INITIALIZE_MATRICES_BY_ZERO','-DEIGEN_NO_MALLOC','-I'+str(app/'TinyMPC-ADMM/ext/Eigen'),'-I'+str(app/'TinyMPC-ADMM/src'),str(cpp)]
    command += [str(solver/(name+'.cpp')) for name in ['utils','model','auxil','cost_lqr','lqr','constraint_linear','admm','rho_benchmark']]
    command += ['-o',str(root/'test')]
    subprocess.run(command,check=True)
    subprocess.run([str(root/'test')],check=True)
print('State ADMM penalty and initial-state projection regression passed')
