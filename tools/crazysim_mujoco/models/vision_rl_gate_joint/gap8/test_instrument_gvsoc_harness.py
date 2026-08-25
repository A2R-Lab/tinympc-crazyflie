"""Host tests for the bounded, ordered GVSOC corpus harness."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.crazysim_mujoco.models.vision_rl_gate_joint.gap8 import instrument_gvsoc_harness as harness


class StreamingHarnessTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.app_dir = Path(self.tempdir.name) / "app"
        (self.app_dir / "src").mkdir(parents=True)
        (self.app_dir / "src/gap8_main.c").write_text(
            "void application(void *arg) { void *l2 = pi_l2_malloc(417000); }\n")
        (self.app_dir / "src/mem.c").write_text(
            "size_t load_file_to_ram(const void *dest, const char *filename) {\n"
            "  pi_fs_file_t *fd = pi_fs_open(&fs, filename, 0);\n"
            "  if (fd == NULL) return 0;\n"
            "  size_t offset = 0;\n"
            "  return offset;\n"
            "}\n")
        (self.app_dir / "src/gap8_network.c").write_text(
            "#ifdef VERBOSE\n"
            "    printf(\"Layer %s %d ended: \\n\", Layers_name[i], i);\n"
            "#endif\n")
        # Include one stale payload after the snapshot to exercise normalization.
        (self.app_dir / "gap8_vars.mk").write_text(
            "FLASH_FILES += hex/weights.hex\n"
            "FLASH_FILES += hex/gap8_inputs.hex\n"
            "READFS_FILES := $(FLASH_FILES)\n"
            "FLASH_FILES += hex/joint_gvsoc_clip_000.hex\n")
        self.frames = np.zeros((harness.JOINT_CLIPS, 2, 160, 160), dtype=np.uint8)
        for index in range(harness.JOINT_CLIPS):
            self.frames[index, 0, 0, 0] = index
            self.frames[index, 1, 159, 159] = 255 - index

    def tearDown(self):
        self.tempdir.cleanup()

    def test_streaming_manifest_order_idempotency_and_memory_contract(self):
        expected_names = harness.clip_filenames()
        first_l2 = harness.install_streaming_harness(self.app_dir, self.frames)
        first_vars = (self.app_dir / "gap8_vars.mk").read_text()
        second_l2 = harness.install_streaming_harness(self.app_dir, self.frames)
        second_vars = (self.app_dir / "gap8_vars.mk").read_text()

        self.assertEqual(first_l2, 417000)
        self.assertEqual(second_l2, 417000)  # DORY L2 allocation is untouched.
        self.assertEqual(first_vars, second_vars)  # safe to instrument twice.

        before_readfs = second_vars.split("READFS_FILES := $(FLASH_FILES)", 1)[0]
        flash_names = tuple(
            line[len("FLASH_FILES += hex/"):]
            for line in before_readfs.splitlines()
            if line.startswith("FLASH_FILES += hex/"))
        self.assertEqual(flash_names, ("weights.hex", "gap8_inputs.hex") + expected_names)
        self.assertEqual(tuple(path.name for path in sorted((self.app_dir / "hex").iterdir())),
                         expected_names)

        expected_bytes = self.frames.transpose(0, 2, 3, 1)
        for index, filename in enumerate(expected_names):
            self.assertEqual((self.app_dir / "hex" / filename).read_bytes(),
                             expected_bytes[index].tobytes())

        source = (self.app_dir / "src/gap8_main.c").read_text()
        self.assertIn("#define JOINT_L2_BYTES 417000", source)
        self.assertIn("void *staging = ram_malloc(JOINT_INPUT_BYTES);", source)
        self.assertNotIn("ram_malloc(JOINT_CLIPS * JOINT_INPUT_BYTES)", source)
        self.assertEqual(source.count("ram_malloc(JOINT_INPUT_BYTES)"), 1)
        self.assertEqual(source.count("load_file_to_ram(staging, input_file)"), 1)
        self.assertEqual(source.count("ram_free(staging, JOINT_INPUT_BYTES)"), 1)
        self.assertIn("char input_file[] = \"joint_gvsoc_clip_000.hex\"", source)
        self.assertIn("input_file[17] = '0' + (i / 100);", source)
        self.assertIn("input_file[18] = '0' + ((i / 10) % 10);", source)
        self.assertIn("input_file[19] = '0' + (i % 10);", source)
        self.assertIn("for (int i = 0; !status && i < JOINT_CLIPS; ++i)", source)
        self.assertIn("ram_read(l2, staging, JOINT_INPUT_BYTES);", source)
        self.assertLessEqual(harness.JOINT_INPUT_BYTES, 51200)

        loader = (self.app_dir / "src/mem.c").read_text()
        self.assertEqual(loader.count("pi_fs_open(&fs, filename, 0)"), 1)
        self.assertEqual(loader.count("pi_fs_close(fd);"), 1)
        self.assertLess(loader.index("pi_fs_close(fd);"), loader.index("return offset;"))
        network = (self.app_dir / "src/gap8_network.c").read_text()
        self.assertEqual(network.count("JOINT_LAYER_CHECK"), 1)
        self.assertIn("if (exec == 0)", network)

    def test_reader_patch_fails_closed_without_generated_loader(self):
        (self.app_dir / "src/mem.c").write_text("void unrelated(void) {}\n")
        with self.assertRaisesRegex(RuntimeError, "generated READFS loader"):
            harness.close_generated_readfs_descriptor(self.app_dir)


if __name__ == "__main__":
    unittest.main()
