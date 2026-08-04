#!/usr/bin/env python3
"""Cross-check and exercise the GAP8/STM32 fixed feature-track wire ABI."""

from __future__ import annotations

import argparse
import binascii
import struct
import subprocess
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
APP = HERE.parents[1]
STM32_HEADER = APP / "src" / "flowdeck_obstacle_link.h"
DEFAULT_GAP8_HEADER = (
    APP.parents[2]
    / "tinympc-nanocockpit"
    / "src"
    / "gap"
    / "examples"
    / "pulp-frontnet"
    / "flow_obstacle_uart.h"
)

HEADER = bytes((0x90, 0x19, 0x08, 0x36))
TRACK = struct.Struct("<HHhhHH")
PREFIX = struct.Struct("<IIHHBBBB")
TRACK_COUNT = 32
PAYLOAD_SIZE = PREFIX.size + TRACK_COUNT * TRACK.size
MESSAGE_SIZE = len(HEADER) + PAYLOAD_SIZE + 4


def compile_probe(header: Path, stub_dir: Path, output: Path) -> bytes:
    source = output.with_suffix(".c")
    source.write_text(
        f"""
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "{header}"
int main(void) {{
  flow_track_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  memcpy(msg.header, FLOW_TRACK_MSG_HEADER, 4);
  msg.p.gap8_ts_us = 0x11223344u;
  msg.p.stm32_ts_echo = 0x55667788u;
  msg.p.sequence = 0x99aau;
  msg.p.dt_us = 65000u;
  msg.p.version = FLOW_TRACK_WIRE_VERSION;
  msg.p.count = FLOW_TRACK_MAX;
  msg.p.flags = 0x5au;
  msg.p.track[0].u_q4 = 0x1234u;
  msg.p.track[0].v_q4 = 0x5678u;
  msg.p.track[0].du_q8 = -321;
  msg.p.track[0].dv_q8 = 654;
  msg.p.track[0].lk_err_q8 = 0x1357u;
  msg.p.track[0].fb_err_q8 = 0x2468u;
  printf("%zu %zu %zu\\n", sizeof(flow_track_wire_t),
         sizeof(flow_track_payload_t), sizeof(flow_track_msg_t));
  const uint8_t *bytes = (const uint8_t *)&msg;
  for (size_t i = 0; i < sizeof(msg) - sizeof(msg.checksum); i++) {{
    printf("%02x", bytes[i]);
  }}
  printf("\\n");
  return 0;
}}
"""
    )
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(stub_dir),
            str(source),
            "-o",
            str(output),
        ],
        check=True,
    )
    return subprocess.check_output([str(output)])


def parse_probe(raw: bytes) -> tuple[tuple[int, int, int], bytes]:
    sizes_line, hex_line = raw.decode().splitlines()
    return tuple(map(int, sizes_line.split())), bytes.fromhex(hex_line)


def expected_prefix() -> bytes:
    first = TRACK.pack(0x1234, 0x5678, -321, 654, 0x1357, 0x2468)
    payload = PREFIX.pack(
        0x11223344,
        0x55667788,
        0x99AA,
        65000,
        1,
        TRACK_COUNT,
        0x5A,
        0,
    )
    return HEADER + payload + first + bytes((TRACK_COUNT - 1) * TRACK.size)


def frame(body: bytes) -> bytes:
    return body + struct.pack("<I", binascii.crc32(body) & 0xFFFFFFFF)


def scan(stream: bytes) -> list[bytes]:
    packets: list[bytes] = []
    cursor = 0
    while True:
        start = stream.find(HEADER, cursor)
        if start < 0 or start + MESSAGE_SIZE > len(stream):
            return packets
        candidate = stream[start : start + MESSAGE_SIZE]
        expected_crc = struct.unpack_from("<I", candidate, MESSAGE_SIZE - 4)[0]
        actual_crc = binascii.crc32(candidate[:-4]) & 0xFFFFFFFF
        if expected_crc == actual_crc:
            packets.append(candidate)
            cursor = start + MESSAGE_SIZE
        else:
            cursor = start + 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gap8-header", type=Path, default=DEFAULT_GAP8_HEADER)
    args = parser.parse_args()

    if not args.gap8_header.exists():
        raise SystemExit(f"GAP8 header not found: {args.gap8_header}")

    with tempfile.TemporaryDirectory(prefix="flow-track-wire-") as tmp:
        tmp_path = Path(tmp)
        (tmp_path / "uart.h").write_text(
            "typedef struct uart_stub uart_t;\n"
        )
        (tmp_path / "pmsis.h").write_text(
            "typedef struct pi_task_stub pi_task_t;\n"
        )
        stm32 = parse_probe(
            compile_probe(STM32_HEADER, tmp_path, tmp_path / "stm32_probe")
        )
        gap8 = parse_probe(
            compile_probe(args.gap8_header, tmp_path, tmp_path / "gap8_probe")
        )

    required_sizes = (TRACK.size, PAYLOAD_SIZE, MESSAGE_SIZE)
    assert stm32[0] == required_sizes, (stm32[0], required_sizes)
    assert gap8[0] == required_sizes, (gap8[0], required_sizes)
    assert stm32[1] == gap8[1] == expected_prefix()

    valid = frame(expected_prefix())
    corrupt = bytearray(valid)
    corrupt[37] ^= 0x80
    recovered = scan(b"noise" + bytes(corrupt) + b"\x90\x19" + valid)
    assert recovered == [valid]

    print(
        f"PASS flow-track ABI: track={TRACK.size} payload={PAYLOAD_SIZE} "
        f"message={MESSAGE_SIZE}, CRC/resync verified"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
