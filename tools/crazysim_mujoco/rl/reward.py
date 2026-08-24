"""Reward used by the vision-RL/TinyMPC proof of concept."""

from __future__ import annotations

import numpy as np


def transition_reward(
    progress_delta_m,
    cross_track_m,
    tangential_speed_mps,
    minimum_clearance_m,
    action,
    previous_action,
    contact,
    complete,
    safe_clearance_m: float = 0.20,
):
    """Return a vectorized reward without exposing privileged data to policy."""
    progress_delta_m = np.asarray(progress_delta_m, dtype=np.float32)
    cross_track_m = np.asarray(cross_track_m, dtype=np.float32)
    tangential_speed_mps = np.asarray(tangential_speed_mps, dtype=np.float32)
    minimum_clearance_m = np.asarray(minimum_clearance_m, dtype=np.float32)
    switch = np.asarray(action) != np.asarray(previous_action)
    reverse = tangential_speed_mps < 0.0
    proximity = np.maximum(safe_clearance_m - minimum_clearance_m, 0.0)
    return (
        4.0 * progress_delta_m
        - 2.0 * np.square(cross_track_m)
        - 0.5 * reverse.astype(np.float32)
        - 5.0 * np.square(proximity)
        - 0.02 * switch.astype(np.float32)
        - 100.0 * np.asarray(contact, dtype=np.float32)
        + 20.0 * np.asarray(complete, dtype=np.float32)
    ).astype(np.float32)
