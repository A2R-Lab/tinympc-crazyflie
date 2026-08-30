from __future__ import annotations

from collections import Counter

import pytest
import trimesh

from envs.imav22_arena import (
    COMPLEXITY_SCORE_MULTIPLIER,
    FINAL_OBJECT_COUNTS,
    GATE_ONLY,
    GATE_BORDER_M,
    GATE_CENTER_HEIGHT_M,
    GATE_CLEAR_OPENING_M,
    GATE_DAE_PATH,
    GATE_OUTER_SIZE_M,
    IMAV22_ARENA_ABI,
    MODE_OBJECT_COUNTS,
    RELOCATED_FULL,
    RELOCATION_PERIOD_S,
    SCORING_AREA_SIZE_M,
    STATIC_FULL,
    competition_reference_layout,
    relocate_one_obstacle,
    relocation_event_due,
    sample_randomized_layout,
    verify_imav22_assets,
)


def test_official_imav22_assets_are_byte_pinned() -> None:
    resolved = verify_imav22_assets()
    assert len(resolved) == 18
    assert IMAV22_ARENA_ABI == "imav22_nanocopter_visual_fidelity_v3"


def test_published_gate_contract_is_preserved() -> None:
    assert GATE_CLEAR_OPENING_M == (0.400, 0.400)
    assert GATE_BORDER_M == 0.094
    assert GATE_OUTER_SIZE_M == pytest.approx((0.588, 0.588))
    assert GATE_CENTER_HEIGHT_M == 1.0


def test_official_gate_mesh_has_complete_floor_stand_and_uv_map() -> None:
    scene = trimesh.load(GATE_DAE_PATH, force="scene", process=False)
    assert len(scene.geometry) == 1
    mesh = next(iter(scene.geometry.values()))
    assert mesh.extents == pytest.approx((0.94, 0.40, 1.294), abs=1.0e-5)
    assert len(mesh.vertices) == 424
    assert len(mesh.faces) == 232
    assert mesh.visual.uv is not None
    assert len(mesh.visual.uv) == len(mesh.vertices)


@pytest.mark.parametrize("seed", [0, 22, 2022, 17_041])
def test_randomized_layout_is_deterministic_valid_and_complete(seed: int) -> None:
    first = sample_randomized_layout(seed)
    second = sample_randomized_layout(seed)
    assert first == second
    assert Counter(obj.kind for obj in first.objects) == FINAL_OBJECT_COUNTS
    assert first.complexity_mode == STATIC_FULL
    half = SCORING_AREA_SIZE_M[0] / 2.0
    for obj in first.objects:
        assert abs(obj.center_m[0]) + obj.footprint_radius_m <= half
        assert abs(obj.center_m[1]) + obj.footprint_radius_m <= half


def test_different_random_seeds_change_the_course() -> None:
    assert sample_randomized_layout(22).objects != sample_randomized_layout(2022).objects


def test_reference_layout_is_an_explicit_reconstruction_not_claimed_final_coordinates() -> None:
    layout = competition_reference_layout()
    manifest = layout.to_manifest()
    assert Counter(obj.kind for obj in layout.objects) == FINAL_OBJECT_COUNTS
    assert not layout.randomized
    assert "coordinates were not published" in manifest["layout_claim"]
    panels = [obj for obj in layout.objects if obj.kind == "panel"]
    assert [obj.size_m[1] for obj in panels] == [1.0, 1.0]
    assert all(obj.size_m == (0.03, 1.0, 1.8) for obj in panels)
    assert all(obj.center_m[2] == 1.1 for obj in panels)
    flags = [obj for obj in layout.objects if obj.kind == "flag"]
    assert all("smooth_feather_banner" in obj.source_fidelity for obj in flags)


@pytest.mark.parametrize("mode", [GATE_ONLY, STATIC_FULL, RELOCATED_FULL])
def test_all_official_complexity_modes_have_the_correct_inventory_and_multiplier(mode: str) -> None:
    layout = sample_randomized_layout(2022, complexity_mode=mode)
    manifest = layout.to_manifest()
    counts = Counter(obj.kind for obj in layout.objects)
    assert {kind: counts[kind] for kind in FINAL_OBJECT_COUNTS} == MODE_OBJECT_COUNTS[mode]
    assert manifest["complexity_mode"] == mode
    assert manifest["complexity_score_multiplier"] == COMPLEXITY_SCORE_MULTIPLIER[mode]
    assert manifest["relocation_period_s"] == (
        RELOCATION_PERIOD_S if mode == RELOCATED_FULL else None
    )


def test_relocation_schedule_is_exactly_thirty_seconds() -> None:
    assert relocation_event_due(0.0) == 0
    assert relocation_event_due(29.999) == 0
    assert relocation_event_due(30.0) == 1
    assert relocation_event_due(59.999) == 1
    assert relocation_event_due(60.0) == 2
    with pytest.raises(ValueError, match="non-negative"):
        relocation_event_due(-0.01)


def test_relocated_full_moves_one_non_gate_behind_or_outside_camera_sector() -> None:
    initial = sample_randomized_layout(22, complexity_mode=RELOCATED_FULL)
    kwargs = {
        "event_index": 1,
        "drone_position_m": (0.0, 0.0),
        "drone_yaw_rad": 0.0,
    }
    relocated = relocate_one_obstacle(initial, **kwargs)
    replay = relocate_one_obstacle(initial, **kwargs)
    assert relocated == replay
    before = {obj.object_id: obj for obj in initial.objects}
    after = {obj.object_id: obj for obj in relocated.objects}
    changed = [object_id for object_id in before if before[object_id] != after[object_id]]
    assert changed == [relocated.last_relocated_object_id]
    moved = after[changed[0]]
    assert moved.kind != "gate"
    assert all(before[key] == after[key] for key in before if before[key].kind == "gate")
    distance = (moved.center_m[0] ** 2 + moved.center_m[1] ** 2) ** 0.5
    assert distance >= 1.50 + moved.footprint_radius_m
    assert relocated.relocation_event_index == 1


def test_relocations_are_consecutive_and_cycle_over_non_gate_obstacles() -> None:
    layout = sample_randomized_layout(17_041, complexity_mode=RELOCATED_FULL)
    moved_ids = []
    for event_index in range(1, 10):
        layout = relocate_one_obstacle(
            layout,
            event_index=event_index,
            drone_position_m=(0.0, 0.0),
            drone_yaw_rad=0.0,
        )
        moved_ids.append(layout.last_relocated_object_id)
    assert len(set(moved_ids[:8])) == 8
    assert moved_ids[8] == moved_ids[0]
    with pytest.raises(ValueError, match="advance monotonically"):
        relocate_one_obstacle(
            layout,
            event_index=9,
            drone_position_m=(0.0, 0.0),
            drone_yaw_rad=0.0,
        )


def test_static_modes_reject_runtime_relocation() -> None:
    for mode in (GATE_ONLY, STATIC_FULL):
        layout = sample_randomized_layout(22, complexity_mode=mode)
        with pytest.raises(ValueError, match="only valid"):
            relocate_one_obstacle(
                layout,
                event_index=1,
                drone_position_m=(0.0, 0.0),
                drone_yaw_rad=0.0,
            )
