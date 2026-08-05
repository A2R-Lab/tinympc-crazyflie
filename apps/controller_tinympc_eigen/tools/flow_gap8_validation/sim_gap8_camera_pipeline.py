#!/usr/bin/env python3
"""Small event model for GAP8 camera callback serialization and recovery."""

from __future__ import annotations

from dataclasses import dataclass


def max_concurrent_inferences(cnn_ms: float, frames: int,
                              serialized_callbacks: bool) -> int:
    """Model alternating flow/CNN callbacks arriving every 33.3 ms.

    Flow callbacks wait for the prior inference. Without consumer serialization,
    a later CNN callback can overtake that suspended flow callback.
    """
    frame_ms = 1000 / 30
    callback_free_ms = 0.0
    inference_ends: list[float] = []
    peak = 0
    for sequence in range(frames):
        arrival = sequence * frame_ms
        start = max(arrival, callback_free_ms) if serialized_callbacks else arrival
        inference_ends = [end for end in inference_ends if end > start]
        if sequence % 2 == 0:  # flow waits for all earlier cluster work
            finish = max([start, *inference_ends])
            if serialized_callbacks:
                callback_free_ms = finish
        else:
            inference_ends.append(start + cnn_ms)
            peak = max(peak, len(inference_ends))
            if serialized_callbacks:
                callback_free_ms = start
    return peak


@dataclass
class CameraWatchdog:
    capture_timeout_us: int = 250_000
    cooldown_us: int = 500_000
    last_capture_us: int = 0
    last_recovery_us: int = 0
    recoveries: int = 0

    def poll(self, now_us: int, stage: str,
             capture_event_done: bool = False) -> bool:
        if (stage != "wait_capture" or
                capture_event_done or
                now_us - self.last_capture_us <= self.capture_timeout_us or
                now_us - self.last_recovery_us <= self.cooldown_us):
            return False
        self.last_capture_us = now_us
        self.last_recovery_us = now_us
        self.recoveries += 1
        return True
