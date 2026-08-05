#!/usr/bin/env python3
"""Byte-exact GAP8→STM32 mixed vision-link model and fault injector."""

from __future__ import annotations

import math
import random
import struct
import zlib
from dataclasses import dataclass

GATE_HEADER = b"\x90\x19\x08\x33"
FLOW_HEADER = b"\x90\x19\x08\x35"
GATE_PAYLOAD = struct.Struct("<I8f")
SECTOR = struct.Struct("<4f")
FLOW_PREFIX = struct.Struct("<IIfBBH")
GATE_BYTES = 44
FLOW_BYTES = 168


def crc_packet(header: bytes, payload: bytes) -> bytes:
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def gate_packet(sequence: int, corners: tuple[float, ...] | None = None) -> bytes:
    if corners is None:
        corners = (40.0, 20.0, 40.0, 75.0, 120.0, 75.0, 120.0, 20.0)
    return crc_packet(GATE_HEADER, GATE_PAYLOAD.pack(sequence, *corners))


def flow_packet(sequence: int, timestamp: int | None = None) -> bytes:
    timestamp = sequence * 66667 if timestamp is None else timestamp
    payload = FLOW_PREFIX.pack(timestamp, 0, 1 / 15, 9, 0, sequence & 0xFFFF)
    payload += b"".join(SECTOR.pack(-0.8 + 0.2*i, 0.2, 1.0, 0.8)
                        for i in range(9))
    return crc_packet(FLOW_HEADER, payload)


@dataclass
class LinkStats:
    gate_ok: int = 0
    flow_ok: int = 0
    crc_error: int = 0
    invalid: int = 0
    duplicate: int = 0
    discarded: int = 0


class VisionParser:
    """Mirrors gate8RxTask framing, validation and consecutive de-duplication."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = LinkStats()
        self.gate_sequences: list[int] = []
        self.flow_sequences: list[int] = []
        self.last_gate = 0
        self.last_flow = 0

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)
        while True:
            gate_at = self.buffer.find(GATE_HEADER)
            flow_at = self.buffer.find(FLOW_HEADER)
            starts = [(i, "gate") for i in (gate_at,) if i >= 0]
            starts += [(i, "flow") for i in (flow_at,) if i >= 0]
            if not starts:
                # Firmware retains a three-byte header prefix in its sync window.
                discard = max(0, len(self.buffer) - 3)
                self.stats.discarded += discard
                del self.buffer[:discard]
                return
            start, kind = min(starts)
            self.stats.discarded += start
            del self.buffer[:start]
            size = GATE_BYTES if kind == "gate" else FLOW_BYTES
            if len(self.buffer) < size:
                return
            packet = bytes(self.buffer[:size])
            del self.buffer[:size]
            expected = struct.unpack_from("<I", packet, size - 4)[0]
            if zlib.crc32(packet[:-4]) & 0xFFFFFFFF != expected:
                self.stats.crc_error += 1
                continue
            if kind == "gate":
                self._gate(packet[4:-4])
            else:
                self._flow(packet[4:-4])

    def _gate(self, payload: bytes) -> None:
        sequence, *corners = GATE_PAYLOAD.unpack(payload)
        valid = all(math.isfinite(v) for v in corners)
        valid &= all((-80 <= v <= 240) if i % 2 == 0 else (-48 <= v <= 144)
                     for i, v in enumerate(corners))
        if not valid:
            self.stats.invalid += 1
        elif sequence and sequence == self.last_gate:
            self.stats.duplicate += 1
        else:
            self.stats.gate_ok += 1
            self.gate_sequences.append(sequence)
            if sequence:
                self.last_gate = sequence

    def _flow(self, payload: bytes) -> None:
        _, _, dt, count, _, sequence = FLOW_PREFIX.unpack_from(payload)
        values = [SECTOR.unpack_from(payload, FLOW_PREFIX.size + i*SECTOR.size)
                  for i in range(min(count, 9))]
        valid = (0 < count <= 9 and math.isfinite(dt) and 0.004 <= dt <= 0.5)
        valid &= all(all(math.isfinite(v) for v in row) and
                     abs(row[0]) <= 2 and abs(row[1]) <= 25 and
                     abs(row[2]) <= 25 and 0 <= row[3] <= 1 for row in values)
        if not valid:
            self.stats.invalid += 1
        elif sequence and sequence == self.last_flow:
            self.stats.duplicate += 1
        else:
            self.stats.flow_ok += 1
            self.flow_sequences.append(sequence)
            if sequence:
                self.last_flow = sequence


def corrupted_stream(frames: int, seed: int) -> tuple[bytes, bytes]:
    """Return a faulted mixed stream followed by a clean recovery sentinel."""
    rng = random.Random(seed)
    out = bytearray()
    for sequence in range(1, frames + 1):
        for packet in (gate_packet(sequence), flow_packet(sequence)):
            mode = rng.randrange(7)
            if mode == 0:                         # clean
                out.extend(packet)
            elif mode == 1:                       # duplicate
                out.extend(packet + packet)
            elif mode == 2:                       # bit corruption
                bad = bytearray(packet)
                bad[rng.randrange(4, len(bad) - 4)] ^= 1 << rng.randrange(8)
                out.extend(bad)
            elif mode == 3:                       # truncation
                out.extend(packet[:rng.randrange(4, len(packet))])
            elif mode == 4:                       # random line noise
                out.extend(rng.randbytes(rng.randrange(1, 20)) + packet)
            elif mode == 5:                       # dropped packet
                pass
            else:                                 # valid packet
                out.extend(packet)
    # A truncated fixed-length frame can legitimately consume the immediately
    # following header before CRC failure. Three clean pairs prove bounded
    # resynchronization rather than assuming zero collateral packet loss.
    sentinel = b"".join(gate_packet(i) + flow_packet(i)
                        for i in range(frames + 1, frames + 4))
    return bytes(out), sentinel


def queue_holdoff_margin_ms(queue_bytes: int = 512, baud: int = 115200) -> float:
    return queue_bytes * 10 * 1000 / baud
