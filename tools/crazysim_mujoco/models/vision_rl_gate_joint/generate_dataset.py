#!/usr/bin/env python3
"""Generate a deterministic, scene-isolated synthetic gate+obstacle corpus.

This is deliberately an offline curriculum generator, not a simulator.  Gate
and obstacle geometry are privileged labels/reward inputs only; the policy sees
only the two rendered grayscale frames.  MuJoCo collection can later write the
same NPZ schema without changing the trainer.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np

TRACK, LEFT, RIGHT = 0, 1, 2

def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()

def split_for_scene(scene_id: int, seed: int, fraction: float, test_fraction: float = .0) -> str:
    # Scene, not sample/augmentation, is the indivisible provenance group.
    value = int.from_bytes(hashlib.sha256(f"{seed}:scene:{scene_id}".encode()).digest()[:8], "little")
    unit = value / 2**64
    if unit < test_fraction: return "test"
    return "validation" if unit < test_fraction + fraction else "train"

def disk(image, x, y, radius, value):
    yy, xx = np.ogrid[:image.shape[0], :image.shape[1]]
    image[(xx - x) ** 2 + (yy - y) ** 2 <= radius ** 2] = value

def render(size, gate_visible, gate_x, gate_y, gate_scale, obstacle_x, obstacle_y, obstacle_scale, brightness, noise, rng):
    image = np.full((size, size), 23.0 * brightness, dtype=np.float32)
    if gate_visible:
        cx, cy = int((gate_x + 1) * .5 * (size - 1)), int((gate_y + 1) * .5 * (size - 1))
        half_w, half_h = max(5, int(gate_scale * size)), max(5, int(gate_scale * .72 * size))
        thick = max(2, size // 64)
        image[max(0, cy-half_h):min(size, cy-half_h+thick), max(0, cx-half_w):min(size, cx+half_w)] = 205 * brightness
        image[max(0, cy+half_h-thick):min(size, cy+half_h), max(0, cx-half_w):min(size, cx+half_w)] = 205 * brightness
        image[max(0, cy-half_h):min(size, cy+half_h), max(0, cx-half_w):min(size, cx-half_w+thick)] = 205 * brightness
        image[max(0, cy-half_h):min(size, cy+half_h), max(0, cx+half_w-thick):min(size, cx+half_w)] = 205 * brightness
    disk(image, int((obstacle_x + 1) * .5 * (size - 1)), int((obstacle_y + 1) * .5 * (size - 1)), max(2, int(obstacle_scale * size)), 145 * brightness)
    return np.clip(image + rng.normal(0, noise, image.shape), 0, 255).astype(np.uint8)

def reward(progress, gate_center_error, clearance, obstacle_contact, frame_contact, ordered_gate_passed, switched, cfg):
    """Privileged training reward: progress + centering + passage - safety/switch."""
    proximity = max(float(cfg["safe_clearance_m"]) - clearance, 0.0)
    return (4 * progress - 2 * gate_center_error ** 2 - 5 * proximity ** 2
            - .02 * switched - cfg["contact_penalty"] * (obstacle_contact + frame_contact)
            + cfg["gate_reward"] * ordered_gate_passed)

def generate(cfg: dict, output: Path) -> dict:
    rng = np.random.default_rng(int(cfg["seed"])); size = int(cfg["image_size"])
    buckets = {"train": {}, "validation": {}, "test": {}}
    names = ("frames", "action", "gate_corners", "gate_visible", "obstacle_risk", "reward", "episode_id", "scene_id", "layout_seed")
    for name in buckets:
        buckets[name] = {key: [] for key in names}
    for scene in range(int(cfg["episodes"])):
        split = split_for_scene(scene, int(cfg["seed"]), float(cfg["validation_fraction"]), float(cfg.get("test_fraction", 0)))
        # One scene creates all temporal samples and therefore cannot leak.
        gate_mode = scene % 2 == 0
        gate_x, gate_y = rng.uniform(-.48, .48), rng.uniform(-.28, .28)
        obstacle_x, obstacle_y = rng.uniform(-.72, .72), rng.uniform(-.30, .42)
        gate_scale, obstacle_scale = rng.uniform(.12, .25), rng.uniform(.06, .18)
        brightness, noise = rng.uniform(.72, 1.22), rng.uniform(1., 8.)
        previous = None; previous_action = TRACK
        for sample in range(int(cfg["samples_per_episode"])):
            phase = sample / max(int(cfg["samples_per_episode"]) - 1, 1)
            # Approach motion makes the second frame useful while keeping labels analytic.
            gx = gate_x * (1 - .12 * phase) if gate_mode else 0.0
            gy = gate_y * (1 - .12 * phase) if gate_mode else 0.0
            ox = obstacle_x * (1 - .20 * phase); oy = obstacle_y + .06 * np.sin(phase * 6.28)
            clearance = max(.01, 1.0 - obstacle_scale * 5 - abs(ox) * .45 - max(oy, 0) * .20)
            obstacle_risk = float(np.clip((.34 - clearance) / .34, 0, 1))
            gate_error = float(np.hypot(gx, gy)) if gate_mode else 0.0
            # Gate center wins unless its requested side would move into the obstacle.
            desired = TRACK
            lateral = gx if gate_mode else 0.0
            if obstacle_risk > .32: lateral += -.75 * np.sign(ox if abs(ox) > .03 else 1)
            if lateral > .13: desired = LEFT
            elif lateral < -.13: desired = RIGHT
            frame = render(size, gate_mode, gx, gy, gate_scale * (1 + phase), ox, oy, obstacle_scale, brightness, noise, rng)
            if previous is None: previous = frame
            obstacle_contact = float(clearance < .035 and desired == (LEFT if ox < 0 else RIGHT))
            # A gate frame is unsafe if crossing occurs outside its privileged opening.
            frame_contact = float(gate_mode and phase > .96 and gate_error >= .18)
            passed = float(gate_mode and phase > .96 and gate_error < .18 and not obstacle_contact and not frame_contact)
            r = reward(.025, gate_error, clearance, obstacle_contact, frame_contact, passed, desired != previous_action, cfg)
            half_w, half_h = gate_scale * (1 + phase), gate_scale * .72 * (1 + phase)
            # Stored TL/TR/BR/BL targets are normalized image coordinates [0,1].
            corners = np.clip((np.array([[gx-half_w,gy-half_h],[gx+half_w,gy-half_h],[gx+half_w,gy+half_h],[gx-half_w,gy+half_h]],np.float32) + 1.0) * .5, 0., 1.) if gate_mode else np.zeros((4,2),np.float32)
            data = buckets[split]
            data["frames"].append(np.stack((previous, frame)))
            data["action"].append(desired)
            data["gate_corners"].append(corners); data["gate_visible"].append(float(gate_mode))
            data["obstacle_risk"].append(obstacle_risk)
            data["reward"].append(r); data["episode_id"].append(scene); data["scene_id"].append(scene); data["layout_seed"].append(int(cfg["seed"]) + scene)
            previous, previous_action = frame, desired
    output.mkdir(parents=True, exist_ok=True); summaries = {}
    for split, values in buckets.items():
        arrays = {key: np.asarray(value, dtype=np.uint8 if key == "frames" else np.float32 if key in ("gate_corners", "gate_visible", "obstacle_risk", "reward") else np.int64) for key, value in values.items()}
        path = output / f"{split}.npz"; np.savez_compressed(path, **arrays)
        summaries[split] = {"samples": int(len(arrays["action"])), "scenes": sorted(set(map(int, arrays["scene_id"]))), "sha256": digest(path)}
    groups=[set(summaries[x]["scenes"]) for x in ("train","validation","test")]
    if any(groups[i]&groups[j] for i in range(3) for j in range(i)) or any(not x for x in groups): raise RuntimeError("invalid scene split")
    manifest = {"schema_version": 2, "generator": "generate_dataset.py", "representativeness": "analytic offline proxy only; not a closed-loop RL or MuJoCo acceptance claim", "config": cfg, "reward_contract": "4*progress - 2*gate_center_error^2 - 5*max(safe_clearance-clearance,0)^2 - .02*action_switch - 100*(obstacle_contact+gate_frame_contact) + 20*one_time_ordered_gate_plane_passage", "privileged_only": ["gate_plane/opening/frame geometry", "gate_corners", "obstacle_risk", "reward"], "splits": summaries, "scene_leakage": False}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return manifest

def main():
    p=argparse.ArgumentParser(); p.add_argument("--config", type=Path, required=True); p.add_argument("--output", type=Path, required=True); args=p.parse_args()
    print(json.dumps(generate(json.loads(args.config.read_text()), args.output), indent=2))
if __name__ == "__main__": main()
