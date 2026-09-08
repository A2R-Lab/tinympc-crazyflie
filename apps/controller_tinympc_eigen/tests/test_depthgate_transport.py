"""Use production DepthGate acceptance and CRC to test wire corruption."""
from pathlib import Path
import subprocess
import tempfile
import unittest
class TransportTest(unittest.TestCase):
 def test_wire(self):
  app=Path(__file__).resolve().parents[1]; root=app.parents[1]
  text=(app/'src/espnet_collision_link.c').read_text()
  state=text[text.index('static DepthGatePayload latest_depthgate;'):text.index('static logByFunction_t dg_age_log')]
  accept=text[text.index('static void acceptDepthGateBytes'):text.index('static void collisionRxTask')]
  code='''
#include <assert.h>
#include <string.h>
#include "depthgate_packet.h"
#include "espnet_collision_link.h"
#include "crc32.h"
static uint32_t now;
static uint32_t xTaskGetTickCount(void) { return now; }
#define portTICK_PERIOD_MS 1
#define taskENTER_CRITICAL() do {} while(0)
#define taskEXIT_CRITICAL() do {} while(0)
'''+state+accept+'''
int main(void) {
 DepthGateObservation observation;
 memset(&observation, 0xff, sizeof(observation));
 assert(!depthGateLinkGetLatest(NULL));
 assert(!depthGateLinkGetLatest(&observation));
 assert(!observation.valid && observation.sample==0 && observation.inverse_depth[1]==0);
 assert(observation.received_age_ms==UINT32_MAX);
 DepthGatePacket p = {0}; memcpy(p.header, DEPTHGATE_HEADER, 4);
 p.payload.status=3; p.payload.sequence=1; p.payload.inverse_depth[1]=2.0f;
 p.payload.inference_us=83000; p.payload.source_timestamp_ms=12345;
 p.checksum=crc32CalculateBuffer(&p,76);
 assert(crc32CalculateBuffer("123456789",9)==0xcbf43926);
 acceptDepthGateBytes((const uint8_t *)&p); assert(dg_rx_ok==1);
 assert(latest_depthgate.inverse_depth[1]==2);
 now=17;
 assert(depthGateLinkGetLatest(&observation));
 assert(observation.valid && observation.sample==1 && observation.sequence==1);
 assert(observation.received_age_ms==17 && observation.inference_us==83000);
 assert(observation.inverse_depth[1]==2 && observation.source_timestamp_ms==12345);
 p.payload.sequence=2; acceptDepthGateBytes((const uint8_t *)&p);
 assert(dg_crc_errors==1 && dg_rx_ok==1);
 p.checksum=crc32CalculateBuffer(&p,76); acceptDepthGateBytes((const uint8_t *)&p);
 assert(dg_rx_ok==2); acceptDepthGateBytes((const uint8_t *)&p); assert(dg_stale==1);
 now=29;
 assert(depthGateLinkGetLatest(&observation));
 assert(observation.sample==2 && observation.received_age_ms==12);
 p.payload.sequence=3;p.payload.corners_xy[0]=NAN;p.checksum=crc32CalculateBuffer(&p,76);
 acceptDepthGateBytes((const uint8_t *)&p);assert(dg_invalid==1&&dg_rx_ok==2);
 assert(logDepthGateAge(0,0)==12 && dg_short==0);
 /* Local age survives tick wrap; accepted sequence zero remains a packet. */
 now=UINT32_MAX-3; p.payload.corners_xy[0]=0; p.payload.sequence=65535;
 p.checksum=crc32CalculateBuffer(&p,76); acceptDepthGateBytes((const uint8_t *)&p);
 now=4; assert(depthGateLinkGetLatest(&observation));
 assert(observation.received_age_ms==8);
 p.payload.sequence=0; p.checksum=crc32CalculateBuffer(&p,76);
 acceptDepthGateBytes((const uint8_t *)&p);
 assert(depthGateLinkGetLatest(&observation) && observation.sequence==0);
 return 0;
}
'''
  with tempfile.TemporaryDirectory() as d:
   source=Path(d)/'test.c';source.write_text(code);exe=Path(d)/'test'
   (Path(d)/'static_mem.h').write_text('#define NO_DMA_CCM_SAFE_ZERO_INIT\n')
   vendor=Path(d)/'crc.o'
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Wno-sign-compare','-I'+d,'-I'+str(root/'crazyflie-firmware/src/utils/interface'),'-c',str(root/'crazyflie-firmware/src/utils/src/crc32.c'),'-o',str(vendor)],check=True)
   subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+d,'-I'+str(app/'src'),'-I'+str(root/'crazyflie-firmware/src/utils/interface'),str(source),str(vendor),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
