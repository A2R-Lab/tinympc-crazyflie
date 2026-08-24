"""Hardware-calibrated HM01B0 camera transform for the gate POC only.

CrazySim renders a centered, ideal pinhole image.  The two-frame gate model
was trained with the calibrated HM01B0 projection and Isaac's lightweight
mono sensor response.  This module converts a small overscan render into the
same 160x160 uint8 domain without changing any normal vision path.
"""

from __future__ import annotations

import hashlib
import math

import numpy as np


TARGET_WIDTH = 160
TARGET_HEIGHT = 160
SOURCE_WIDTH = 192
SOURCE_HEIGHT = 192
FX_PX = 89.1558392549
FY_PX = 89.4608171623
CX_PX = 81.1038105230
CY_PX = 73.3473030288
DISTORTION = (-0.0176448766, 0.0994132451, 0.0054432154,
              -0.0060400120, -0.055)
# The source is a square-pixel pinhole render whose vertical focal length is
# FY_PX.  Its horizontal focal therefore differs from calibrated FX_PX by the
# same small anisotropy that the remap below corrects.
SOURCE_FOCAL_PX = FY_PX
SOURCE_FOVY_DEG = math.degrees(2.0 * math.atan(
    (SOURCE_HEIGHT * 0.5) / SOURCE_FOCAL_PX))
SENSOR_RESPONSE = {
    "gain_mean": 1.0,
    "gain_stddev": 0.06,
    "black_level": 0.05,
    "gamma": 1.15,
    "shot_noise_variance_offset": 14.0,
    "shot_noise_variance_slope": 55.0,
}


def _undistort_normalized(xd: np.ndarray, yd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Invert OpenCV plumb-bob distortion with a deterministic fixed iteration."""
    k1, k2, p1, p2, k3 = DISTORTION
    x, y = xd.copy(), yd.copy()
    for _ in range(32):
        radius2 = x * x + y * y
        radial = 1.0 + k1 * radius2 + k2 * radius2 ** 2 + k3 * radius2 ** 3
        model_x = x * radial + 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x)
        model_y = y * radial + p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x += xd - model_x
        y += yd - model_y
    return x, y


def calibrated_remap() -> tuple[np.ndarray, np.ndarray]:
    """Return source pixel coordinates for every target HM01B0 pixel.

    This is equivalent to Isaac's output-pixel inverse-distortion remap, but
    uses a 192x192 centered pinhole overscan source rather than its 512x512
    renderer.  The calibrated target K includes the non-central principal
    point, so the output image also carries the correct center-ray geometry.
    """
    target_y, target_x = np.mgrid[0:TARGET_HEIGHT, 0:TARGET_WIDTH]
    xd = (target_x.astype(np.float64) - CX_PX) / FX_PX
    yd = (target_y.astype(np.float64) - CY_PX) / FY_PX
    ideal_x, ideal_y = _undistort_normalized(xd, yd)
    source_x = ideal_x * SOURCE_FOCAL_PX + SOURCE_WIDTH * 0.5
    source_y = ideal_y * SOURCE_FOCAL_PX + SOURCE_HEIGHT * 0.5
    if (not np.isfinite(source_x).all() or not np.isfinite(source_y).all() or
            source_x.min() < 0.0 or source_x.max() > SOURCE_WIDTH - 1.0 or
            source_y.min() < 0.0 or source_y.max() > SOURCE_HEIGHT - 1.0):
        raise RuntimeError("HM01B0 POC remap exceeds the 192x192 overscan image")
    return source_x.astype(np.float32), source_y.astype(np.float32)


class Hm01b0PocCamera:
    """Apply Isaac's calibrated projection, gray response, and seeded noise."""

    def __init__(self, seed: int):
        self.map_x, self.map_y = calibrated_remap()
        self.rng = np.random.default_rng(seed)

    @staticmethod
    def _bilinear(frame: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x0 = np.floor(x).astype(np.intp)
        y0 = np.floor(y).astype(np.intp)
        x1 = np.minimum(x0 + 1, frame.shape[1] - 1)
        y1 = np.minimum(y0 + 1, frame.shape[0] - 1)
        wx, wy = x - x0, y - y0
        return ((1.0 - wx) * (1.0 - wy) * frame[y0, x0] +
                wx * (1.0 - wy) * frame[y0, x1] +
                (1.0 - wx) * wy * frame[y1, x0] +
                wx * wy * frame[y1, x1])

    def transform(self, overscan_gray: np.ndarray) -> np.ndarray:
        if overscan_gray.shape != (SOURCE_HEIGHT, SOURCE_WIDTH):
            raise ValueError(
                "HM01B0 POC camera requires a 192x192 gray overscan frame; "
                f"received {overscan_gray.shape}")
        ideal = self._bilinear(overscan_gray.astype(np.float32), self.map_x, self.map_y)
        normalized = np.clip(ideal / 255.0, 0.0, 1.0)
        gain = float(self.rng.normal(SENSOR_RESPONSE["gain_mean"],
                                     SENSOR_RESPONSE["gain_stddev"]))
        response = np.clip(
            SENSOR_RESPONSE["black_level"] +
            (1.0 - SENSOR_RESPONSE["black_level"]) *
            np.power(np.clip(normalized * gain, 0.0, 1.0),
                     SENSOR_RESPONSE["gamma"]),
            0.0, 1.0)
        sigma = np.sqrt(SENSOR_RESPONSE["shot_noise_variance_offset"] +
                        SENSOR_RESPONSE["shot_noise_variance_slope"] * response)
        noisy = response * 255.0 + self.rng.normal(0.0, sigma, response.shape)
        return np.clip(np.rint(noisy), 0.0, 255.0).astype(np.uint8)

    def remap_sha256(self) -> str:
        return hashlib.sha256(
            self.map_x.tobytes() + self.map_y.tobytes()).hexdigest()
