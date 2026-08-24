#!/usr/bin/env python3
"""Train the basic privileged-latent Dyna/MBPO camera policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import random
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional

from . import ACTION_COUNT
from .data import LATENT_NAMES, concatenate_runs
from .export_onnx import export
from .model import LatentDynamics, QNetwork, TemporalPolicy


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)


def file_sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def batches(count: int, batch_size: int, generator: torch.Generator):
    while True:
        yield torch.randint(count, (min(batch_size, count),), generator=generator)


def train(args) -> dict:
    config = json.loads(args.config.read_text())
    seed = int(config.get("optimizer_seed", args.seed))
    seed_everything(seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    expected_training_seeds = {int(value) for value in config["training_seeds"]}
    observed_training_seeds = set()
    for path in args.runs:
        run_config_path = path / "run_config.json"
        if not run_config_path.is_file():
            raise ValueError(f"missing run_config.json: {path}")
        observed_training_seeds.add(
            int(json.loads(run_config_path.read_text())["random_seed"]))
    if observed_training_seeds != expected_training_seeds:
        raise ValueError(
            "input run random seeds do not match configured training_seeds: "
            f"observed={sorted(observed_training_seeds)} "
            f"expected={sorted(expected_training_seeds)}")
    dataset = concatenate_runs(args.runs, args.course)
    mean = dataset.latent.mean(axis=0)
    scale = np.maximum(dataset.latent.std(axis=0), 1.0e-3)
    latent = torch.from_numpy((dataset.latent - mean) / scale).float()
    next_latent = torch.from_numpy((dataset.next_latent - mean) / scale).float()
    frames = torch.from_numpy(dataset.frames).float().div_(255.0)
    actions = torch.from_numpy(dataset.action).long()
    rewards = torch.from_numpy(dataset.reward).float()
    done = torch.from_numpy(dataset.done).float()
    latent_dim = latent.shape[1]
    ensemble_size = int(config.get("ensemble_size", 5))
    horizon = min(5, max(1, int(config.get("imagined_horizon", 3))))
    policy = TemporalPolicy(latent_dim).to(device)
    dynamics = torch.nn.ModuleList(
        [LatentDynamics(latent_dim) for _ in range(ensemble_size)]).to(device)
    critic = QNetwork(latent_dim).to(device)
    target = QNetwork(latent_dim).to(device)
    target.load_state_dict(critic.state_dict())
    encoder_optimizer = torch.optim.Adam(policy.encoder.parameters(), lr=3.0e-4)
    actor_optimizer = torch.optim.Adam(policy.actor.parameters(), lr=3.0e-4)
    dynamics_optimizer = torch.optim.Adam(dynamics.parameters(), lr=5.0e-4)
    critic_optimizer = torch.optim.Adam(critic.parameters(), lr=5.0e-4)
    generator = torch.Generator().manual_seed(seed)
    iterator = batches(len(actions), int(config.get("batch_size", 64)), generator)
    gamma = float(config.get("gamma", 0.98))
    metrics = []
    steps_per_epoch = int(config.get("steps_per_epoch", 100))
    for epoch in range(int(config.get("epochs", 20))):
        totals = {name: 0.0 for name in
                  ("latent", "dynamics", "critic", "actor", "behavior")}
        for _ in range(steps_per_epoch):
            index = next(iterator)
            image_batch = frames[index].to(device)
            latent_target = latent[index].to(device)
            next_target = next_latent[index].to(device)
            action = actions[index].to(device)
            reward = rewards[index].to(device)
            terminal = done[index].to(device)

            encoded = policy.encode(image_batch)
            latent_loss = functional.mse_loss(encoded, latent_target)
            encoder_optimizer.zero_grad()
            latent_loss.backward()
            encoder_optimizer.step()

            one_hot = functional.one_hot(action, ACTION_COUNT).float()
            dynamics_loss = torch.zeros((), device=device)
            delta_target = next_target - latent_target
            for model in dynamics:
                delta, predicted_reward, terminal_logit = model(latent_target, one_hot)
                dynamics_loss = dynamics_loss + functional.mse_loss(delta, delta_target)
                dynamics_loss = dynamics_loss + functional.mse_loss(predicted_reward, reward)
                dynamics_loss = dynamics_loss + functional.binary_cross_entropy_with_logits(
                    terminal_logit, terminal)
            dynamics_loss /= ensemble_size
            dynamics_optimizer.zero_grad()
            dynamics_loss.backward()
            dynamics_optimizer.step()

            with torch.no_grad():
                real_target = reward + gamma * (1.0 - terminal) * \
                    target(next_target).max(dim=1).values
            critic_loss = functional.mse_loss(
                critic(latent_target).gather(1, action[:, None]).squeeze(1), real_target)
            imagined = latent_target.detach()
            for rollout in range(horizon):
                with torch.no_grad():
                    probabilities = torch.softmax(policy.actor(imagined), dim=1)
                    imagined_action = torch.multinomial(
                        probabilities.cpu(), 1, generator=generator).squeeze(1).to(device)
                    imagined_one_hot = functional.one_hot(
                        imagined_action, ACTION_COUNT).float()
                    delta, imagined_reward, imagined_done_logit = dynamics[
                        rollout % ensemble_size](imagined, imagined_one_hot)
                    imagined_next = imagined + delta
                    imagined_done = torch.sigmoid(imagined_done_logit)
                    imagined_target = imagined_reward + gamma * (1.0 - imagined_done) * \
                        target(imagined_next).max(dim=1).values
                critic_loss = critic_loss + 0.25 * functional.mse_loss(
                    critic(imagined).gather(1, imagined_action[:, None]).squeeze(1),
                    imagined_target)
                imagined = imagined_next.detach()
            critic_optimizer.zero_grad()
            critic_loss.backward()
            critic_optimizer.step()

            logits = policy.actor(latent_target)
            probabilities = torch.softmax(logits, dim=1)
            entropy = -(probabilities * torch.log(probabilities + 1.0e-8)).sum(dim=1).mean()
            actor_loss = -(probabilities * critic(latent_target).detach()).sum(dim=1).mean()
            actor_loss -= 0.01 * entropy
            behavior_weight = max(0.0, 1.0 - epoch / max(1, int(config.get("epochs", 20)) // 2))
            behavior_loss = functional.cross_entropy(logits, action)
            actor_total = actor_loss + behavior_weight * behavior_loss
            actor_optimizer.zero_grad()
            actor_total.backward()
            actor_optimizer.step()
            with torch.no_grad():
                for target_parameter, parameter in zip(target.parameters(), critic.parameters()):
                    target_parameter.mul_(0.995).add_(parameter, alpha=0.005)
            for name, value in (("latent", latent_loss), ("dynamics", dynamics_loss),
                                ("critic", critic_loss), ("actor", actor_loss),
                                ("behavior", behavior_loss)):
                totals[name] += float(value.detach())
        metrics.append({"epoch": epoch + 1, **{
            name: value / steps_per_epoch for name, value in totals.items()}})
        print(json.dumps(metrics[-1]), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    source_runs = []
    for path in args.runs:
        run_config = path / "run_config.json"
        source_runs.append({
            "path": str(path.resolve()),
            "run_config_sha256": file_sha256(run_config),
            "state_csv_sha256": file_sha256(path / "state.csv"),
            "vision_csv_sha256": file_sha256(path / "vision.csv"),
            "fpv_camera_mp4_sha256": file_sha256(path / "fpv_camera.mp4"),
        })
    training_config = {
        **config, "optimizer_seed": seed, "device": str(device),
        "run_directories": [str(path.resolve()) for path in args.runs],
        "source_runs": source_runs,
        "training_seeds": config.get("training_seeds", [seed]),
        "course": str(args.course.resolve()), "samples": int(len(actions)),
        "latent_names": LATENT_NAMES, "latent_mean": mean.tolist(),
        "latent_scale": scale.tolist(), "imagined_horizon": horizon,
    }
    checkpoint_path = args.output / "checkpoint.pt"
    torch.save({"policy": policy.cpu().state_dict(), "latent_dim": latent_dim,
                "training_config": training_config}, checkpoint_path)
    (args.output / "training_metrics.json").write_text(
        json.dumps({"config": training_config, "epochs": metrics}, indent=2) + "\n")
    export(checkpoint_path, args.output / "policy.onnx",
           args.output / "bundle.json", training_config)
    return metrics[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260824)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    train(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
