"""Reproducible IMAV 2022 nanocopter arena geometry and asset contract.

The organizers published the final object counts and dimensions, but not the
exact final-round object coordinates.  This module therefore keeps the two
claims separate:

* ``competition_reference_layout`` is a deterministic reconstruction using
  the published final-round inventory.
* ``sample_randomized_layout`` samples the same inventory inside the published
  8 m x 8 m scoring area for training and robustness evaluation.

Both modes use textures vendored from the official IMAV 2022 simulator branch.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import math
from pathlib import Path
import random
from typing import Any, Iterable


IMAV22_ARENA_ABI = "imav22_nanocopter_visual_fidelity_v3"
ARENA_SIZE_M = (10.0, 10.0)
SCORING_AREA_SIZE_M = (8.0, 8.0)
GATE_ONLY = "gate_only"
STATIC_FULL = "static_full"
RELOCATED_FULL = "relocated_full"
COMPLEXITY_MODES = (GATE_ONLY, STATIC_FULL, RELOCATED_FULL)
COMPLEXITY_SCORE_MULTIPLIER = {
    GATE_ONLY: 1,
    STATIC_FULL: 5,
    RELOCATED_FULL: 10,
}
RELOCATION_PERIOD_S = 30.0
CAMERA_HORIZONTAL_FOV_DEG = 87.0

# Published physical obstacle dimensions.
GATE_CLEAR_OPENING_M = (0.400, 0.400)
GATE_BORDER_M = 0.094
GATE_OUTER_SIZE_M = (
    GATE_CLEAR_OPENING_M[0] + 2.0 * GATE_BORDER_M,
    GATE_CLEAR_OPENING_M[1] + 2.0 * GATE_BORDER_M,
)
GATE_CENTER_HEIGHT_M = 1.0
GATE_DEPTH_M = 0.080
POLE_RADIUS_M = 0.150
POLE_HEIGHT_M = 2.200

OFFICIAL_SIMULATOR_URL = "https://github.com/tudelft/crazyflie-simulation"
OFFICIAL_SIMULATOR_BRANCH = "imav2022"
OFFICIAL_SIMULATOR_COMMIT = "da3636651e43ba7663eb3ed4f73c59f641058cef"
ASSET_ROOT = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "imav22_crazyflie_simulation"
    / "webots"
    / "worlds"
    / "textures"
)
GATE_DAE_PATH = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "imav22_crazyflie_simulation"
    / "webots"
    / "worlds"
    / "blender"
    / "imav2022-gate.dae"
)
GATE_DAE_SHA256 = "afa65a0b5130a268a1a73f5a81cd039ddad78b05eb1afd236320cc7328f36be1"
DERIVED_ASSET_ROOT = Path(__file__).resolve().parents[1] / "assets" / "imav22" / "derived"
DERIVED_GATE_TEXTURE = "gate_orange_fabric_from_official_atlas.png"
DERIVED_GATE_TEXTURE_SHA256 = (
    "604f3fbf3316b2508b2ef66b04a65b898a78ccf11a2311de37c88d21900a3aa4"
)
DERIVED_FLAG_TEXTURE = "flag_white_blue_proxy_v1.png"
DERIVED_FLAG_TEXTURE_SHA256 = (
    "8fdf17147fb5055e2514c906590a7d2a69be3f4408415e05c618fc177d88150e"
)

TEXTURE_SHA256 = {
    "cyberzoo_floor/grass_green.png": "14afb8bc3961dcdee01a4f833fd68024177e6d30d4842cdc1c7fc4ff779e5a78",
    "cyberzoo_floor/grass_black.png": "3c5059fea187f6b3ce05a53d9cbc67fe18f4247fb36560cd7719366370befc7d",
    "cyberzoo_floor/grass_blue.png": "6782d555b7ff0333dd71173644f599bfd435643b95594a2ee723f27e0eefe52e",
    "orange_pole/orange_pole.png": "27574859a4969b59c24f856d19bda908771d3387f004ca1216ce4c3b3fc21eba",
    "imav2022-gate/GateTex.png": "5643f7bea93f18bef334407efba8cc5cecdb0a26bd355e495cc062fdcefc4b64",
    "metal_panel/metal_panel1.png": "aba3c4fa41ed11e7ef7e3c75f53264b164e87865b3542905423ee259431358ab",
    "metal_panel/metal_panel2.png": "834a8dedb54f39a3f892090607db75eba8c518a8e5a45b842a76313b7b83e7be",
    "metal_panel/metal_panel3.png": "2801e1ff6dca49624bff5c7e6c55005dc2635e0b89a17d4c71576fb36de5a0e2",
    "metal_panel/metal_panel4.png": "f37b7ea65750ea8473774fd1f0bd4771f7019c56bf6efc716169bcb56597ae92",
    "shower_curtain/curtain_striped.png": "2c267c86737e3b33294037a362785edf2c51df416babc839ec7a777904581ff7",
    "traffic_mat/mat_traffic_1.png": "48a20e97f43d1049b7497b7b9ecd54b6b7ff04cc7b0241113a0c2c94e09d5ba2",
    "cyberzoo_net/net.png": "c61f75bc61b38cf1bf3203c19961b1c1387c56965e389ee0bacf12377f0f469d",
    "cyberzoo_walls/wall_curtains.png": "5e7795e46fd378f01689f151f9ae7e1f186e99c79da3a020607e7817ac79ddda",
    "cyberzoo_poles/pole.png": "704a636049ffc4635349241c9b3747baaa056e4e96dacb77ab5d4351d64b55df",
    "cyberzoo_surroundings/cabinets.png": "894db9b6b97aa4d7cce0c10138a0a1952a5c85653ae16e8dad539ae3eb378134",
}

FINAL_OBJECT_COUNTS = {"pole": 4, "panel": 2, "flag": 2, "gate": 2}
MODE_OBJECT_COUNTS = {
    GATE_ONLY: {"pole": 0, "panel": 0, "flag": 0, "gate": 2},
    STATIC_FULL: FINAL_OBJECT_COUNTS,
    RELOCATED_FULL: FINAL_OBJECT_COUNTS,
}


@dataclass(frozen=True)
class ArenaObjectSpec:
    """One static vertical obstacle expressed in arena coordinates."""

    object_id: str
    kind: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    yaw_rad: float
    texture_relative_path: str
    color_rgb: tuple[float, float, float]
    footprint_radius_m: float
    source_fidelity: str


@dataclass(frozen=True)
class ArenaLayout:
    """A complete, deterministic IMAV 2022 arena instance."""

    layout_id: str
    seed: int | None
    randomized: bool
    objects: tuple[ArenaObjectSpec, ...]
    complexity_mode: str = STATIC_FULL
    relocation_event_index: int = 0
    last_relocated_object_id: str | None = None

    def to_manifest(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "arena_abi": IMAV22_ARENA_ABI,
            "layout_id": self.layout_id,
            "seed": self.seed,
            "randomized": self.randomized,
            "complexity_mode": self.complexity_mode,
            "complexity_score_multiplier": COMPLEXITY_SCORE_MULTIPLIER[
                self.complexity_mode
            ],
            "relocation_period_s": (
                RELOCATION_PERIOD_S if self.complexity_mode == RELOCATED_FULL else None
            ),
            "relocation_event_index": self.relocation_event_index,
            "last_relocated_object_id": self.last_relocated_object_id,
            "arena_size_m": list(ARENA_SIZE_M),
            "scoring_area_size_m": list(SCORING_AREA_SIZE_M),
            "official_simulator": {
                "url": OFFICIAL_SIMULATOR_URL,
                "branch": OFFICIAL_SIMULATOR_BRANCH,
                "commit": OFFICIAL_SIMULATOR_COMMIT,
            },
            "final_object_counts": dict(FINAL_OBJECT_COUNTS),
            "active_object_counts": dict(MODE_OBJECT_COUNTS[self.complexity_mode]),
            "layout_claim": (
                "seeded randomized reconstruction of the published final-round inventory"
                if self.randomized
                else "deterministic reference reconstruction; final coordinates were not published"
            ),
            "objects": [asdict(obj) for obj in self.objects],
        }


def texture_path(relative: str) -> Path:
    """Return a verified-contract texture path without performing I/O."""

    if relative not in TEXTURE_SHA256:
        raise ValueError(f"texture is outside the pinned IMAV22 asset contract: {relative}")
    return ASSET_ROOT / relative


def verify_imav22_assets() -> dict[str, str]:
    """Verify byte-pinned textures from the official simulator checkout."""

    resolved: dict[str, str] = {}
    for relative, expected in TEXTURE_SHA256.items():
        path = ASSET_ROOT / relative
        if not path.is_file():
            raise FileNotFoundError(f"missing official IMAV22 simulator texture: {path}")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(
                f"official IMAV22 texture changed: {path}; expected {expected}, got {actual}"
            )
        resolved[relative] = str(path)
    derived = DERIVED_ASSET_ROOT / DERIVED_GATE_TEXTURE
    if not derived.is_file():
        raise FileNotFoundError(f"missing derived official-gate texture crop: {derived}")
    actual = hashlib.sha256(derived.read_bytes()).hexdigest()
    if actual != DERIVED_GATE_TEXTURE_SHA256:
        raise RuntimeError(
            f"derived official-gate texture crop changed: expected {DERIVED_GATE_TEXTURE_SHA256}, got {actual}"
        )
    resolved[f"derived/{DERIVED_GATE_TEXTURE}"] = str(derived)
    derived_flag = DERIVED_ASSET_ROOT / DERIVED_FLAG_TEXTURE
    if not derived_flag.is_file():
        raise FileNotFoundError(f"missing derived feather-banner texture: {derived_flag}")
    actual = hashlib.sha256(derived_flag.read_bytes()).hexdigest()
    if actual != DERIVED_FLAG_TEXTURE_SHA256:
        raise RuntimeError(
            "derived feather-banner texture changed: "
            f"expected {DERIVED_FLAG_TEXTURE_SHA256}, got {actual}"
        )
    resolved[f"derived/{DERIVED_FLAG_TEXTURE}"] = str(derived_flag)
    if not GATE_DAE_PATH.is_file():
        raise FileNotFoundError(f"missing official IMAV22 gate DAE: {GATE_DAE_PATH}")
    gate_dae_actual = hashlib.sha256(GATE_DAE_PATH.read_bytes()).hexdigest()
    if gate_dae_actual != GATE_DAE_SHA256:
        raise RuntimeError(
            f"official IMAV22 gate DAE changed: expected {GATE_DAE_SHA256}, got {gate_dae_actual}"
        )
    resolved["blender/imav2022-gate.dae"] = str(GATE_DAE_PATH)
    return resolved


def _pole(object_id: str, x: float, y: float) -> ArenaObjectSpec:
    return ArenaObjectSpec(
        object_id=object_id,
        kind="pole",
        center_m=(x, y, POLE_HEIGHT_M / 2.0),
        size_m=(2.0 * POLE_RADIUS_M, 2.0 * POLE_RADIUS_M, POLE_HEIGHT_M),
        yaw_rad=0.0,
        texture_relative_path="orange_pole/orange_pole.png",
        color_rgb=(0.95, 0.25, 0.02),
        footprint_radius_m=POLE_RADIUS_M,
        source_fidelity="official_simulator_geometry_and_texture",
    )


def _gate(object_id: str, x: float, y: float, yaw: float) -> ArenaObjectSpec:
    return ArenaObjectSpec(
        object_id=object_id,
        kind="gate",
        center_m=(x, y, GATE_CENTER_HEIGHT_M),
        size_m=(GATE_DEPTH_M, GATE_OUTER_SIZE_M[0], GATE_OUTER_SIZE_M[1]),
        yaw_rad=yaw,
        texture_relative_path="imav2022-gate/GateTex.png",
        color_rgb=(0.96, 0.27, 0.02),
        footprint_radius_m=GATE_OUTER_SIZE_M[0] / 2.0,
        source_fidelity="published_dimensions_with_official_simulator_texture",
    )


def _panel(
    object_id: str,
    x: float,
    y: float,
    yaw: float,
    width: float,
    texture_index: int,
) -> ArenaObjectSpec:
    return ArenaObjectSpec(
        object_id=object_id,
        kind="panel",
        center_m=(x, y, 1.1),
        size_m=(0.03, width, 1.8),
        yaw_rad=yaw,
        texture_relative_path=f"metal_panel/metal_panel{texture_index}.png",
        color_rgb=(0.12, 0.13, 0.14),
        footprint_radius_m=width / 2.0,
        source_fidelity="official_simulator_object_family_and_texture",
    )


def _flag(object_id: str, x: float, y: float, yaw: float) -> ArenaObjectSpec:
    return ArenaObjectSpec(
        object_id=object_id,
        kind="flag",
        center_m=(x, y, 0.95),
        size_m=(0.045, 0.80, 1.55),
        yaw_rad=yaw,
        texture_relative_path="shower_curtain/curtain_striped.png",
        color_rgb=(0.96, 0.97, 0.95),
        footprint_radius_m=0.40,
        source_fidelity="smooth_feather_banner_proxy_from_final_arena_photographs",
    )


def competition_reference_layout() -> ArenaLayout:
    """Return a stable published-inventory reconstruction for regression tests."""

    objects = (
        _gate("gate_0", -1.65, -1.85, 0.0),
        _gate("gate_1", 2.05, 1.75, math.pi / 2.0),
        _panel("panel_0", -0.50, 1.15, 0.40, 1.00, 1),
        _panel("panel_1", 2.75, -1.25, -0.55, 1.00, 3),
        _flag("flag_0", -2.70, 1.90, -0.25),
        _flag("flag_1", 0.85, -2.55, 0.65),
        _pole("pole_0", -2.55, -0.55),
        _pole("pole_1", 0.15, 2.65),
        _pole("pole_2", 2.90, 0.25),
        _pole("pole_3", 0.75, 0.10),
    )
    validate_layout(objects, complexity_mode=STATIC_FULL)
    return ArenaLayout(
        layout_id="imav22_competition_reference_v1",
        seed=None,
        randomized=False,
        objects=objects,
        complexity_mode=STATIC_FULL,
    )


def _is_clear(
    x: float,
    y: float,
    radius: float,
    placed: Iterable[ArenaObjectSpec],
) -> bool:
    for keepout_x, keepout_y, keepout_radius in ((-3.35, 0.0, 0.72), (3.35, 0.0, 0.72)):
        if math.hypot(x - keepout_x, y - keepout_y) < radius + keepout_radius:
            return False
    return all(
        math.hypot(x - other.center_m[0], y - other.center_m[1])
        >= radius + other.footprint_radius_m + 0.28
        for other in placed
    )


def sample_randomized_layout(
    seed: int,
    *,
    complexity_mode: str = STATIC_FULL,
) -> ArenaLayout:
    """Sample one official complexity mode with reproducible keepouts."""

    if complexity_mode not in COMPLEXITY_MODES:
        raise ValueError(f"unknown IMAV22 complexity mode: {complexity_mode}")

    rng = random.Random(int(seed))
    placed: list[ArenaObjectSpec] = []
    if complexity_mode == GATE_ONLY:
        recipes: list[tuple[str, float]] = [
            ("gate", GATE_OUTER_SIZE_M[0] / 2.0),
            ("gate", GATE_OUTER_SIZE_M[0] / 2.0),
        ]
    else:
        recipes = [
            ("panel", 0.50),
            ("panel", 0.50),
            ("flag", 0.40),
            ("flag", 0.40),
            ("gate", GATE_OUTER_SIZE_M[0] / 2.0),
            ("gate", GATE_OUTER_SIZE_M[0] / 2.0),
            *(("pole", POLE_RADIUS_M) for _ in range(4)),
        ]
    kind_indices = {kind: 0 for kind in FINAL_OBJECT_COUNTS}
    for kind, radius in recipes:
        for _attempt in range(500):
            margin = radius + 0.12
            limit = SCORING_AREA_SIZE_M[0] / 2.0 - margin
            x = rng.uniform(-limit, limit)
            y = rng.uniform(-limit, limit)
            if _is_clear(x, y, radius, placed):
                break
        else:
            raise RuntimeError(f"could not place randomized IMAV22 {kind} for seed {seed}")
        index = kind_indices[kind]
        kind_indices[kind] += 1
        yaw = rng.uniform(-math.pi, math.pi)
        if kind == "panel":
            placed.append(_panel(f"panel_{index}", x, y, yaw, 1.00, 1 + 2 * index))
        elif kind == "flag":
            placed.append(_flag(f"flag_{index}", x, y, yaw))
        elif kind == "gate":
            placed.append(_gate(f"gate_{index}", x, y, yaw))
        else:
            placed.append(_pole(f"pole_{index}", x, y))
    validate_layout(placed, complexity_mode=complexity_mode)
    return ArenaLayout(
        layout_id=f"imav22_{complexity_mode}_seed_{int(seed)}",
        seed=int(seed),
        randomized=True,
        objects=tuple(placed),
        complexity_mode=complexity_mode,
    )


def _wrapped_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def relocation_event_due(elapsed_s: float) -> int:
    """Return the completed 30-second relocation-event count."""

    if elapsed_s < 0.0 or not math.isfinite(elapsed_s):
        raise ValueError("elapsed relocation time must be finite and non-negative")
    return int(math.floor((elapsed_s + 1.0e-9) / RELOCATION_PERIOD_S))


def relocate_one_obstacle(
    layout: ArenaLayout,
    *,
    event_index: int,
    drone_position_m: tuple[float, float],
    drone_yaw_rad: float,
    minimum_drone_clearance_m: float = 1.50,
    forward_exclusion_half_angle_deg: float = CAMERA_HORIZONTAL_FOV_DEG / 2.0 + 5.0,
) -> ArenaLayout:
    """Relocate one non-gate object outside the drone's forward safety sector.

    Gates remain static.  The chosen obstacle and its new pose are deterministic
    for ``(layout.seed, event_index)`` so an episode can be replayed exactly.
    """

    if layout.complexity_mode != RELOCATED_FULL:
        raise ValueError("obstacle relocation is only valid in relocated_full mode")
    if event_index <= layout.relocation_event_index:
        raise ValueError("relocation event index must advance monotonically")
    if event_index != layout.relocation_event_index + 1:
        raise ValueError("relocation events must be applied consecutively")
    if minimum_drone_clearance_m <= 0.0:
        raise ValueError("minimum drone clearance must be positive")
    if not 0.0 < forward_exclusion_half_angle_deg < 180.0:
        raise ValueError("forward exclusion half-angle must be in (0, 180) degrees")

    movable = tuple(obj for obj in layout.objects if obj.kind != "gate")
    if not movable:
        raise RuntimeError("relocated_full layout has no movable obstacles")
    selected = movable[(event_index - 1) % len(movable)]
    fixed = tuple(obj for obj in layout.objects if obj.object_id != selected.object_id)
    seed = int(layout.seed or 0)
    rng = random.Random(seed * 1_000_003 + event_index * 97_409 + 22_022)
    half_angle = math.radians(forward_exclusion_half_angle_deg)
    for _attempt in range(1_000):
        margin = selected.footprint_radius_m + 0.12
        limit = SCORING_AREA_SIZE_M[0] / 2.0 - margin
        x = rng.uniform(-limit, limit)
        y = rng.uniform(-limit, limit)
        dx = x - float(drone_position_m[0])
        dy = y - float(drone_position_m[1])
        distance = math.hypot(dx, dy)
        bearing_error = abs(_wrapped_angle(math.atan2(dy, dx) - float(drone_yaw_rad)))
        moved_distance = math.hypot(x - selected.center_m[0], y - selected.center_m[1])
        if (
            distance >= minimum_drone_clearance_m + selected.footprint_radius_m
            and bearing_error > half_angle
            and moved_distance >= 0.75
            and _is_clear(x, y, selected.footprint_radius_m, fixed)
        ):
            break
    else:
        raise RuntimeError(
            f"could not safely relocate {selected.object_id} at event {event_index}"
        )
    yaw = 0.0 if selected.kind == "pole" else rng.uniform(-math.pi, math.pi)
    relocated = replace(selected, center_m=(x, y, selected.center_m[2]), yaw_rad=yaw)
    objects = tuple(relocated if obj.object_id == selected.object_id else obj for obj in layout.objects)
    validate_layout(objects, complexity_mode=RELOCATED_FULL)
    return replace(
        layout,
        layout_id=f"imav22_relocated_full_seed_{seed}_event_{event_index}",
        objects=objects,
        relocation_event_index=event_index,
        last_relocated_object_id=selected.object_id,
    )


def validate_layout(
    objects: Iterable[ArenaObjectSpec],
    *,
    complexity_mode: str = STATIC_FULL,
) -> None:
    """Reject count, boundary, or conservative-footprint contract violations."""

    objects = tuple(objects)
    if complexity_mode not in COMPLEXITY_MODES:
        raise ValueError(f"unknown IMAV22 complexity mode: {complexity_mode}")
    counts = {kind: sum(obj.kind == kind for obj in objects) for kind in FINAL_OBJECT_COUNTS}
    expected = MODE_OBJECT_COUNTS[complexity_mode]
    if counts != expected:
        raise ValueError(f"IMAV22 {complexity_mode} layout has wrong inventory: {counts}")
    ids = [obj.object_id for obj in objects]
    if len(set(ids)) != len(ids):
        raise ValueError("IMAV22 layout object ids must be unique")
    half = SCORING_AREA_SIZE_M[0] / 2.0
    for obj in objects:
        x, y, z = obj.center_m
        if abs(x) + obj.footprint_radius_m > half or abs(y) + obj.footprint_radius_m > half:
            raise ValueError(f"IMAV22 object leaves the scoring area: {obj.object_id}")
        if z <= 0.0 or obj.size_m[2] <= 0.0:
            raise ValueError(f"IMAV22 object has invalid vertical geometry: {obj.object_id}")
    for index, first in enumerate(objects):
        for second in objects[index + 1 :]:
            distance = math.hypot(
                first.center_m[0] - second.center_m[0],
                first.center_m[1] - second.center_m[1],
            )
            if distance < first.footprint_radius_m + second.footprint_radius_m + 0.05:
                raise ValueError(
                    f"IMAV22 object footprints overlap: {first.object_id}, {second.object_id}"
                )
