"""Compile actual SITL packing functions with host stubs; check Python wire format."""
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).parent / '.deps/CrazySim/crazyflie-firmware/src/modules/src/power_distribution_sitl.c'


def function(source, name):
    match = re.search(r'(?:static )?(?:void|bool) ' + name + r'\([^\n]*\)\s*\{', source)
    assert match, name
    start = match.start()
    depth = 1
    end = match.end()
    while depth:
        if source[end] == '{': depth += 1
        elif source[end] == '}': depth -= 1
        end += 1
    return source[start:end]


class SenderTest(unittest.TestCase):
    def test_actual_c_sender_wire_format(self):
        source = SOURCE.read_text()
        declarations = source[source.index('static bool directThrustMode'):source.index('static void packU32LE')]
        c = r"""
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#define CRTP_PORT_SETPOINT_SIM 9
#define CRTP_HEADER(port, channel) (((port)<<4)|(channel))
#define M2T(x) (x)
typedef struct { union { struct {uint16_t m1,m2,m3,m4;} motors; uint16_t list[4]; }; } motors_thrust_pwm_t;
static struct {uint16_t m1,m2,m3,m4; uint32_t firmwareTick,stabilizerSequence;} __attribute__((packed)) motorPower;
static uint32_t lastSentTime;
static struct {uint8_t header,size,data[30];} p;
static uint32_t xTaskGetTickCount(void) {return 0x12345678;}
static void crtpSendPacket(void *ignored) {(void)ignored; fwrite(&p.header,1,1,stdout); fwrite(p.data,1,p.size,stdout);}
""" + declarations + function(source, 'packU32LE') + function(source, 'motorsSetRatio') + r"""
int main(void) {
  motors_thrust_pwm_t pwm = {.list={1,2,3,4}};
  directThrustMode = true;
  directThrustN[0]=0.0f; directThrustN[1]=0.103005f;
  directThrustN[2]=0.2f; directThrustN[3]=0.28835022f;
  motorPower.stabilizerSequence=0x34567890;
  motorsSetRatio(&pwm);
  assert(p.header==0x93 && p.size==24);
  assert(motorPower.m1==1 && motorPower.m4==4);
  directThrustMode=false; lastSentTime=0;
  motorsSetRatio(&pwm);
  assert(p.header==0x90 && p.size==16);
  return 0;
}
"""
        with tempfile.TemporaryDirectory(prefix='tinympc-thrust-sender-') as temp:
            executable = str(Path(temp) / 'sender')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-x', 'c', '-', '-o', executable],
                           input=c, text=True, check=True)
            output = subprocess.check_output([executable])
        self.assertEqual(len(output), 25 + 17)
        self.assertEqual(output[0], 0x93)
        unpacked = struct.unpack('<ffffII', output[1:25])
        for actual, expected in zip(unpacked[:4], [0., .103005, .2, .28835022]):
            self.assertAlmostEqual(actual, expected, places=7)
        self.assertEqual(unpacked[4:], (0x12345678, 0x34567891))
        self.assertEqual(output[25], 0x90)
        self.assertEqual(struct.unpack('<HHHHII', output[26:]),
                         (1, 2, 3, 4, 0x12345678, 0x34567892))


if __name__ == '__main__':
    unittest.main()
