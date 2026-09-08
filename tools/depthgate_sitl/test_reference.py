"""Host-test extracted production reference logic and actual plane projector.

Firmware I/O and final reference serialization are replaced by plain structures;
this does not execute a solver or validate flight dynamics.
"""
from pathlib import Path
import subprocess
import tempfile
app=Path(__file__).resolve().parents[2]/'apps/controller_tinympc_eigen'
s=(app/'src/controller_tinympc.cpp').read_text()
frame=s[s.index('struct MpcLocalFrame {'):s.index('static struct vec worldQuaternionToLocalRodrigues')]
globals_=s[s.index('static uint8_t dg_enable'):s.index('static void selectDepthGateCache')]
logic=s[s.index('static void setDepthGateHoldReference'):s.index('static void resetEspnetStraightTest')]
prefix=r'''
#include <cassert>
#include <cstdint>
#include <cmath>
#include <Eigen/Dense>
#include "espnet_collision_link.h"
#include "tinympc_depthgate_planes.h"
#include "tinympc/position_projection.h"
#define NHORIZON 20
#define DT .02f
#define portTICK_PERIOD_MS 1
struct XYZ {float x=0,y=0,z=0;};
struct state_t {XYZ position,velocity; struct {float roll=0,pitch=0;} attitude;};
struct Ref {Eigen::Vector3f p,v;};
static Ref Xref[NHORIZON];
static float Uref[NHORIZON],ug;
static Eigen::Matrix<float,12,1> x0=Eigen::Matrix<float,12,1>::Zero();
static struct {unsigned count_xy_hs[NHORIZON];Eigen::Vector2f a_xy_hs[NHORIZON][2];float b_xy_hs[NHORIZON][2];} data;
static bool esp_test_run;
static Eigen::Vector3f esp_test_origin=Eigen::Vector3f::Zero();
static float esp_test_yaw,esp_test_speed;
static uint32_t esp_test_age_ms;
static uint8_t esp_test_fresh,esp_test_phase;
static DepthGateObservation input;
static bool depthGateLinkGetLatest(DepthGateObservation *o) {*o=input;return true;}
static int mkvec(int,int,float){return 0;}
static int rpy2quat(int){return 0;}
static void setLocalReferenceState(Ref& r,const Eigen::Vector3f& p,int,
 const Eigen::Vector3f& v,const Eigen::Vector3f&){r.p=p;r.v=v;}
'''
# Header declares getter externally: define with matching linkage instead.
prefix=prefix.replace('static bool depthGateLinkGetLatest','bool depthGateLinkGetLatest')
main=r'''
int main() {
 state_t state; state.position={10,20,1}; esp_test_origin={10,20,1};
 input.valid=true;input.inference_us=83000;input.sample=1;
 input.inverse_depth[0]=1.f/1.5f; input.inverse_depth[1]=1.f/3.f;input.inverse_depth[2]=1.f/1.5f;
 updateDepthGateReference(state,100);
 assert(dg_fresh && dg_count==2 && !dg_fault);
 assert(data.count_xy_hs[0]==0 && data.count_xy_hs[1]==2);
 const Eigen::Vector2f n=dg_world_n[0]; const float b=dg_world_b[0];
 const float local_b=data.b_xy_hs[1][0];
 // Same observation must stay anchored in the world as vehicle moves.
 state.position.x+=.1f; input.received_age_ms=20;
 updateDepthGateReference(state,120);
 assert(dg_world_b[0]==b && dg_world_n[0]==n);
 assert(fabsf(data.b_xy_hs[1][0]-(local_b-.1f*n.x()))<1e-5f);
 esp_test_run=true; updateDepthGateReference(state,140);
 assert(!dg_fault && dg_command_speed>0);
 for(const auto& r:Xref) assert(r.p.allFinite()&&r.v.allFinite());
 input.received_age_ms=201;updateDepthGateReference(state,160);
 assert(dg_fault==1 && dg_command_speed==0);
 input.received_age_ms=0;input.sample++;
 updateDepthGateReference(state,180);assert(dg_fault==1); // no blind autoresume
 esp_test_run=false;updateDepthGateReference(state,200);assert(!dg_fault);
 esp_test_run=true;updateDepthGateReference(state,220);assert(!dg_fault);
 // Mode re-entry with RUN held requires release; reset path uses dg_tick=0.
 dg_tick=0;updateDepthGateReference(state,240);assert(dg_fault==7);
 esp_test_run=false;updateDepthGateReference(state,260);assert(!dg_fault);
 esp_test_run=true;updateDepthGateReference(state,280);assert(!dg_fault);
 input.sample++;input.inverse_depth[0]=input.inverse_depth[1]=input.inverse_depth[2]=.1f;
 updateDepthGateReference(state,300);assert(dg_count==0);
 for(const auto& r:Xref) assert(r.p.allFinite()&&r.v.allFinite());
 return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
 p=Path(tmp)/'test.cpp';p.write_text(prefix+frame+'\nstatic MpcLocalFrame active_local_frame={0,0,0,0,1,0};\n'+globals_+logic+main)
 exe=Path(tmp)/'test'
 subprocess.run(['c++','-std=c++17','-O1','-Wall','-Wextra','-Werror','-Wno-unused-variable','-Wno-missing-field-initializers',
   '-isystem',str(app/'TinyMPC-ADMM/ext/Eigen'),'-I'+str(app/'TinyMPC-ADMM/src'),
   '-I'+str(app/'src'),str(p),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('production reference cases passed')
