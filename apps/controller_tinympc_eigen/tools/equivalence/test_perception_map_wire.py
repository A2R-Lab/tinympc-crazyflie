#!/usr/bin/env python3
"""Host checks for the NanoCockpit GAP8 -> STM32 perception-map ABI."""

import binascii
import re
import struct
from pathlib import Path


APP = Path(__file__).resolve().parents[2]
GAP_APP = Path("/home/cchen/tinympc-nanocockpit/src/gap/examples/pulp-frontnet")
STM_HEADER = APP / "src" / "perception_map_link.h"
GAP_HEADER = GAP_APP / "perception_map_uart.h"

HEADER = b"\x90\x19\x08\x37"
WIDTH = 10
HEIGHT = 10
CELLS = WIDTH * HEIGHT
CHANNELS = 4
PACKED_BYTES = 200
PAYLOAD = struct.Struct("<IIHBBBB200s")
MESSAGE_BYTES = 4 + PAYLOAD.size + 4


def _define(path: Path, name: str) -> str:
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+(.+?)\s*$",
        path.read_text(),
        re.MULTILINE,
    )
    assert match, f"{name} missing from {path}"
    return match.group(1)


def _set_nibble(packed: bytearray, index: int, value: int) -> None:
    byte_index = index >> 1
    if index & 1:
        packed[byte_index] = (packed[byte_index] & 0x0F) | ((value & 0x0F) << 4)
    else:
        packed[byte_index] = (packed[byte_index] & 0xF0) | (value & 0x0F)


def _get_nibble(packed: bytes, index: int) -> int:
    value = packed[index >> 1]
    return value >> 4 if index & 1 else value & 0x0F


def _pool_u4(source: bytes, use_minimum: bool = False) -> list[int]:
    result = []
    for out_y in range(HEIGHT):
        for out_x in range(WIDTH):
            values = [
                source[(2 * out_y + dy) * 20 + 2 * out_x + dx]
                for dy in range(2)
                for dx in range(2)
            ]
            result.append((min(values) if use_minimum else max(values)) >> 4)
    return result


def test_headers_define_identical_wire_abi() -> None:
    for name in (
        "PERCEPTION_MAP_W",
        "PERCEPTION_MAP_H",
        "PERCEPTION_MAP_PACKED_BYTES",
        "PERCEPTION_MAP_WIRE_VERSION",
    ):
        assert _define(GAP_HEADER, name) == _define(STM_HEADER, name)
    assert PAYLOAD.size == 214
    assert MESSAGE_BYTES == 222


def test_pool_pack_expand_and_crc_roundtrip() -> None:
    maps = [
        bytes((13 * i + 7 * channel) & 0xFF for i in range(400))
        for channel in range(CHANNELS)
    ]
    pooled = [
        _pool_u4(source, use_minimum=(channel == 3))
        for channel, source in enumerate(maps)
    ]
    packed = bytearray(PACKED_BYTES)
    for cell in range(CELLS):
        for channel in range(CHANNELS):
            _set_nibble(packed, CHANNELS * cell + channel,
                        pooled[channel][cell])

    payload = PAYLOAD.pack(
        123456, 789, 42, 2, WIDTH, HEIGHT, 0, bytes(packed)
    )
    body = HEADER + payload
    packet = body + struct.pack("<I", binascii.crc32(body) & 0xFFFFFFFF)
    assert len(packet) == MESSAGE_BYTES
    assert struct.unpack_from("<I", packet, MESSAGE_BYTES - 4)[0] == (
        binascii.crc32(packet[:-4]) & 0xFFFFFFFF
    )

    received_packed = PAYLOAD.unpack_from(packet, len(HEADER))[-1]
    expanded = [
        [_get_nibble(received_packed, CHANNELS * cell + channel) * 17
         for cell in range(CELLS)]
        for channel in range(CHANNELS)
    ]
    assert expanded == [
        [value * 17 for value in channel_values] for channel_values in pooled
    ]


def test_max_pool_is_conservative_at_wire_resolution() -> None:
    source = bytearray(400)
    source[7 * 20 + 11] = 255
    pooled = _pool_u4(bytes(source))
    assert sum(value != 0 for value in pooled) == 1
    assert pooled[(7 // 2) * WIDTH + (11 // 2)] == 15


def test_gate_permission_uses_conservative_min_pool() -> None:
    source = bytearray([255] * 400)
    source[7 * 20 + 11] = 0
    pooled = _pool_u4(bytes(source), use_minimum=True)
    assert pooled[(7 // 2) * WIDTH + (11 // 2)] == 0
