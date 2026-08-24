import csv
import ast
import json
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "tools/crazysim_mujoco/vision_bridge.py"
RUNNER = ROOT / "tools/crazysim_mujoco/run.sh"


def unused_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class SensorModeTests(unittest.TestCase):
    def test_runner_records_and_forwards_flowdeck_as_first_class_mode(self):
        source = RUNNER.read_text()
        self.assertIn('FLOWDECK_ENABLED=0', source)
        self.assertIn('"flowdeck_enabled": bool(int(flowdeck_enabled))', source)
        self.assertIn('EXTRA_SIM_ARGS+=(--flowdeck)', source)
        self.assertIn('--flowdeck)', source)
        self.assertIn('--no-flowdeck)', source)

    def test_runner_separates_camera_only_from_inference(self):
        source = RUNNER.read_text()
        self.assertIn('"camera_only_enabled": bool(int(camera_only))', source)
        self.assertIn('"camera_inference_enabled": bool(int(vision_enabled))', source)
        self.assertIn('--camera-only --camera-port 5200', source)
        self.assertNotIn('--camera-only --camera-port 5200 --firmware-port', source)
        result = subprocess.run(
            [str(RUNNER), "--camera-only", "--vision-model", "unused.onnx"],
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("mutually exclusive", result.stderr)

    def test_camera_only_records_frame_without_firmware_datagram(self):
        camera_port = unused_udp_port()
        firmware = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        firmware.bind(("127.0.0.1", 0))
        firmware.settimeout(0.25)
        sender = None
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            process = subprocess.Popen(
                [
                    sys.executable, str(BRIDGE), "--camera-only",
                    "--camera-port", str(camera_port), "--camera-fps", "20",
                    "--firmware-port", str(firmware.getsockname()[1]),
                    "--log", str(output / "camera.csv"),
                    "--frames-dir", str(output / "frames"),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                ready = process.stdout.readline()
                self.assertIn("passive; no firmware output", ready)
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sender.sendto(
                    struct.pack("<HHHH", 0, 1, 2, 2) + bytes((1, 2, 3, 4)),
                    ("127.0.0.1", camera_port),
                )
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if (output / "frames/metadata.json").is_file():
                        break
                    time.sleep(0.02)
                metadata = json.loads((output / "frames/metadata.json").read_text())
                self.assertEqual(metadata["frames_received"], 1)
                self.assertFalse(metadata["inference_enabled"])
                self.assertFalse(metadata["firmware_output_enabled"])
                with (output / "camera.csv").open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(rows[0]["width"], "2")
                self.assertEqual(rows[0]["height"], "2")
                with self.assertRaises(socket.timeout):
                    firmware.recvfrom(4096)
            finally:
                process.terminate()
                process.wait(timeout=3)
                process.stdout.close()
                if sender is not None:
                    sender.close()
                firmware.close()

    def test_passive_function_has_no_inference_or_send_call(self):
        tree = ast.parse(BRIDGE.read_text())
        function = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "record_camera_only"
        )
        names = {
            node.func.id for node in ast.walk(function)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        attributes = {
            node.func.attr for node in ast.walk(function)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertFalse({"make_adapter", "packet_bytes"} & names)
        self.assertNotIn("predict", attributes)
        self.assertNotIn("sendto", attributes)


if __name__ == "__main__":
    unittest.main()
