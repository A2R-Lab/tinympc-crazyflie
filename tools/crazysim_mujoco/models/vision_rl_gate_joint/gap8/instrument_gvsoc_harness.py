#!/usr/bin/env python3
"""Install a generated-C, 200-clip GVSOC timing/output harness after DORY codegen."""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np


JOINT_CLIPS = 200
JOINT_INPUT_BYTES = 2 * 160 * 160
JOINT_OUTPUT_BYTES = 12
_READFS_ASSIGNMENT = "READFS_FILES := $(FLASH_FILES)"


def clip_filenames() -> tuple:
    """Return the sealed corpus filenames in execution order."""
    return tuple("joint_gvsoc_clip_%03d.hex" % index
                 for index in range(JOINT_CLIPS))


def add_streaming_payloads_to_readfs(variables: Path,
                                     filenames: tuple) -> None:
    """Make every ordered clip visible to READFS before its make snapshot."""
    existing = variables.read_text()
    if _READFS_ASSIGNMENT not in existing:
        raise RuntimeError("unable to locate DORY READFS_FILES assignment")
    markers = tuple("FLASH_FILES += hex/%s\n" % name for name in filenames)
    # Remove a stale or partial previous install before placing one canonical
    # manifest before READFS_FILES' immediately-expanded assignment.
    normalized = existing
    for marker in markers:
        normalized = normalized.replace(marker, "")
    manifest = "".join(markers)
    normalized = normalized.replace(
        _READFS_ASSIGNMENT, manifest + _READFS_ASSIGNMENT, 1)
    if normalized != existing:
        variables.write_text(normalized)


def render_streaming_harness(l2_bytes: int) -> str:
    """Render C that reuses exactly one external-RAM input staging buffer."""
    return """#include \"mem.h\"\n#include \"gap8_network.h\"\n#include \"pmsis.h\"\n#include <stdio.h>\n\nextern int gap8_cycle_network_execution;\n#define JOINT_CLIPS %d\n#define JOINT_INPUT_BYTES %d\n#define JOINT_OUTPUT_BYTES %d\n#define JOINT_L2_BYTES %d\n\nvoid application(void *arg) {\n  int status = 0;\n  char input_file[] = \"joint_gvsoc_clip_000.hex\";\n  mem_init(); gap8_network_initialize();\n  void *l2 = pi_l2_malloc(JOINT_L2_BYTES);\n  void *out = pi_l2_malloc(JOINT_OUTPUT_BYTES);\n  // The complete corpus lives in READFS/flash. HyperRAM holds one clip only.\n  void *staging = ram_malloc(JOINT_INPUT_BYTES);\n  if (!l2 || !out || !staging) {\n    printf(\"JOINT_GVSOC_ALLOC_FAIL\\n\");\n    status = -1;\n  }\n  for (int i = 0; !status && i < JOINT_CLIPS; ++i) {\n    input_file[17] = '0' + (i / 100);\n    input_file[18] = '0' + ((i / 10) %% 10);\n    input_file[19] = '0' + (i %% 10);\n    size_t bytes = load_file_to_ram(staging, input_file);\n    if (bytes != JOINT_INPUT_BYTES) {\n      printf(\"JOINT_GVSOC_INPUT_FAIL %%d %%u\\n\", i, (unsigned int)bytes);\n      status = -1;\n      break;\n    }\n    ram_read(l2, staging, JOINT_INPUT_BYTES);\n    gap8_network_run(l2, JOINT_L2_BYTES, out, i, 1);\n    printf(\"JOINT_GVSOC %%d %%d\", i, gap8_cycle_network_execution);\n    for (int j = 0; j < JOINT_OUTPUT_BYTES; ++j) printf(\" %%u\", ((uint8_t *)out)[j]);\n    printf(\"\\n\");\n  }\n  if (staging) ram_free(staging, JOINT_INPUT_BYTES);\n  if (out) pi_l2_free(out, JOINT_OUTPUT_BYTES);\n  if (l2) pi_l2_free(l2, JOINT_L2_BYTES);\n  gap8_network_terminate(); pmsis_exit(status);\n}\nint main(void) { pmsis_kickoff((void *)application); return 0; }\n""" % (JOINT_CLIPS, JOINT_INPUT_BYTES, JOINT_OUTPUT_BYTES, l2_bytes)


def close_generated_readfs_descriptor(app_dir: Path) -> None:
    """Close every generated READFS descriptor after its complete transfer."""
    mem_c = app_dir / "src/mem.c"
    source = mem_c.read_text()
    match = re.search(
        r"(size_t load_file_to_ram\(const void \*dest, const char \*filename\) \{.*?"
        r"\n  return offset;\n)(\})",
        source, flags=re.DOTALL)
    if not match:
        raise RuntimeError("unable to locate generated READFS loader")
    loader = match.group(1)
    if "pi_fs_open(&fs, filename, 0)" not in loader:
        raise RuntimeError("generated READFS loader does not open descriptors")
    if "pi_fs_close(fd);" not in loader:
        loader = loader.replace(
            "\n  return offset;\n", "\n  pi_fs_close(fd);\n  return offset;\n")
        source = source[:match.start(1)] + loader + source[match.start(2):]
        mem_c.write_text(source)


def instrument_first_clip_layer_checks(app_dir: Path) -> None:
    """Emit every layer checksum for clip zero outside timed regions."""
    network_c = app_dir / "src/gap8_network.c"
    source = network_c.read_text()
    marker = "JOINT_LAYER_CHECK"
    if marker in source:
        return
    anchor = """#ifdef VERBOSE
    printf("Layer %s %d ended: \\n", Layers_name[i], i);
"""
    if anchor not in source:
        raise RuntimeError("unable to locate generated layer-check insertion point")
    check = """if (exec == 0) {
      uint32_t joint_sum = 0;
      for (int joint_index = 0; joint_index < activations_out_size[i]; ++joint_index)
        joint_sum += ((uint8_t *)L2_output)[joint_index];
      printf("JOINT_LAYER_CHECK %d %u %u\\n", i,
             (unsigned int)activations_out_checksum[i][0],
             (unsigned int)joint_sum);
    }

"""
    network_c.write_text(source.replace(anchor, check + anchor, 1))


def install_streaming_harness(app_dir: Path, frames: np.ndarray) -> int:
    """Install the bounded harness and return DORY's unchanged L2 allocation."""
    if frames.dtype != np.uint8 or frames.shape != (JOINT_CLIPS, 2, 160, 160):
        raise RuntimeError("generated-C harness requires sealed uint8[200,2,160,160]")
    main_c = app_dir / "src/gap8_main.c"
    original = main_c.read_text()
    match = re.search(r"#define JOINT_L2_BYTES (\d+)", original)
    if not match:
        match = re.search(r"pi_l2_malloc\((\d+)\)", original)
    if not match:
        raise RuntimeError("unable to determine DORY L2 allocator size")
    l2_bytes = int(match.group(1))
    filenames = clip_filenames()
    inputs_dir = app_dir / "hex"
    inputs_dir.mkdir(parents=True, exist_ok=True)
    # READFS keeps all immutable clips; C copies each one into the single
    # staging allocation immediately before inference.
    ordered_frames = frames.transpose(0, 2, 3, 1)
    for index, filename in enumerate(filenames):
        (inputs_dir / filename).write_bytes(ordered_frames[index].tobytes())
    add_streaming_payloads_to_readfs(app_dir / "gap8_vars.mk", filenames)
    close_generated_readfs_descriptor(app_dir)
    instrument_first_clip_layer_checks(app_dir)
    main_c.write_text(render_streaming_harness(l2_bytes))
    return l2_bytes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--app-dir", type=Path, required=True)
    parser.add_argument("--corpus", type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.corpus, allow_pickle=False) as source:
        frames = np.asarray(source["frames"])
    l2_bytes = install_streaming_harness(args.app_dir, frames)
    print("instrumented %s with %d streaming clips, input_bytes=%d l2_bytes=%d" %
          (args.app_dir, JOINT_CLIPS, frames.nbytes, l2_bytes))


if __name__ == "__main__":
    main()
