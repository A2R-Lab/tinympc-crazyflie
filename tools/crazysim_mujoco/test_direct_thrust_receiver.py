"""Dependency-light tests of actual receiver methods (no MuJoCo installation)."""
import ast
import math
from pathlib import Path
import struct
import threading
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).parent / '.deps/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py'


def make_receiver():
    tree = ast.parse(SOURCE.read_text())
    names = {'_thrust_to_rpm', '_pwm_to_rpm', '_accept_thrust_packet'}
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {'math': math, 'struct': struct, 'CRTP_HDR_THRUST': 0x93}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    receiver = type('Receiver', (), {name: namespace[name] for name in names})()
    receiver._params = SimpleNamespace(
        rpm2thrust=(0.0, -3.133427287299859e-7, 4.407354891648379e-10),
        max_rpm=17037.03, pwm_thrust_full=0.20)
    receiver._motor_lock = threading.Lock()
    receiver._thrust_requested = [0.0] * 4
    receiver._rpm_ref = [0.0] * 4
    receiver._firmware_tick = receiver._stabilizer_sequence = 0
    receiver._motor_command_mode = receiver._motor_command_packet_count = 0
    return receiver


class DirectThrustTests(unittest.TestCase):
    def setUp(self):
        self.r = make_receiver()

    def packet(self, forces=(0.01, 0.03, 0.08, 0.103005)):
        return bytes([0x93]) + struct.pack('<ffffII', *forces, 12345, 6789)

    def test_thrust_roundtrip(self):
        a, b, c = self.r._params.rpm2thrust
        for requested in (0.001, 0.05, 0.103005, 0.12):
            rpm = self.r._thrust_to_rpm(requested)
            self.assertAlmostEqual(a + b*rpm + c*rpm*rpm, requested, places=12)

    def test_zero_and_limit(self):
        self.assertEqual(self.r._thrust_to_rpm(0), 0)
        self.assertEqual(self.r._thrust_to_rpm(1.0), self.r._params.max_rpm)

    def test_explicit_units_metadata_order(self):
        self.assertTrue(self.r._accept_thrust_packet(self.packet()))
        self.assertEqual(self.r._firmware_tick, 12345)
        self.assertEqual(self.r._stabilizer_sequence, 6789)
        self.assertEqual(self.r._motor_command_mode, 1)
        self.assertEqual(self.r._motor_command_packet_count, 1)
        self.assertEqual(self.r._rpm_ref, sorted(self.r._rpm_ref))
        self.assertAlmostEqual(self.r._thrust_requested[3], 0.103005, places=7)

    def test_invalid_packets_leave_state_unchanged(self):
        self.r._accept_thrust_packet(self.packet())
        initial = (self.r._rpm_ref[:], self.r._thrust_requested[:],
                   self.r._motor_command_packet_count, self.r._firmware_tick)
        invalid = [b'', self.packet()[:-1], self.packet()+b'X',
                   bytes([0x90])+self.packet()[1:]]
        invalid += [self.packet((value, .03, .08, .1))
                    for value in (-.01, math.nan, math.inf, -math.inf)]
        for packet in invalid:
            self.assertFalse(self.r._accept_thrust_packet(packet))
            self.assertEqual(initial, (self.r._rpm_ref[:], self.r._thrust_requested[:],
                                      self.r._motor_command_packet_count, self.r._firmware_tick))

    def test_request_logged_before_limit(self):
        self.assertTrue(self.r._accept_thrust_packet(self.packet((1., 0., .1, .1))))
        self.assertEqual(self.r._thrust_requested[0], 1.0)
        self.assertEqual(self.r._rpm_ref[0], self.r._params.max_rpm)
        self.assertEqual(self.r._rpm_ref[1], 0)

    def test_legacy_pwm_preserved(self):
        self.assertEqual(self.r._pwm_to_rpm(6999), 0)
        self.assertEqual(self.r._pwm_to_rpm(40000),
                         self.r._thrust_to_rpm((40000 / 65535)**2 * .2))


if __name__ == '__main__':
    unittest.main()
