#!/usr/bin/env python3
"""Train the basic privileged-latent Dyna/MBPO camera policy."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import random
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional

from . import ACTION_COUNT
from .data import (LATENT_NAMES, RunTransitions, concatenate_runs,
                   load_expert_npz, split_by_episode)
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


def balanced_expert_indices(dataset: RunTransitions, batch_size: int,
                            generator: torch.Generator) -> torch.Tensor:
    """Sample LEFT/RIGHT/TRACK evenly and retain hard TRACK examples."""
    action = dataset.expert_action
    pools = [np.flatnonzero(action == value) for value in range(ACTION_COUNT)]
    if any(pool.size == 0 for pool in pools):
        raise ValueError("balanced expert batches require TRACK, LEFT, and RIGHT")
    counts = [batch_size // 3] * 3
    for index in range(batch_size % 3):
        counts[index] += 1

    def sample(pool, count):
        selected = torch.randint(
            len(pool), (count,), generator=generator).numpy()
        return pool[selected]

    hard_track = np.flatnonzero(
        (action == 0) & np.asarray(dataset.hard_track_mask, dtype=bool))
    easy_track = np.flatnonzero(
        (action == 0) & ~np.asarray(dataset.hard_track_mask, dtype=bool))
    hard_count = counts[0] // 2 if hard_track.size else 0
    easy_count = counts[0] - hard_count
    track_parts = []
    if hard_count:
        track_parts.append(sample(hard_track, hard_count))
    track_parts.append(sample(easy_track if easy_track.size else pools[0], easy_count))
    indices = np.concatenate((np.concatenate(track_parts), sample(pools[1], counts[1]),
                              sample(pools[2], counts[2])))
    permutation = torch.randperm(len(indices), generator=generator).numpy()
    return torch.from_numpy(indices[permutation]).long()


def _classification_metrics(truth: np.ndarray, prediction: np.ndarray) -> dict:
    confusion = np.asarray([
        [np.sum((truth == actual) & (prediction == predicted))
         for predicted in range(ACTION_COUNT)] for actual in range(ACTION_COUNT)])
    recall = np.diag(confusion) / np.maximum(confusion.sum(axis=1), 1)
    precision = np.diag(confusion) / np.maximum(confusion.sum(axis=0), 1)
    f1 = 2.0 * precision * recall / np.maximum(precision + recall, 1.0e-9)
    return {
        "confusion": confusion.tolist(), "recall": recall.tolist(),
        "balanced_accuracy": float(np.mean(recall)), "macro_f1": float(np.mean(f1)),
    }


def evaluate_policy(policy: TemporalPolicy, dataset: RunTransitions,
                    latent_mean: np.ndarray, latent_scale: np.ndarray,
                    device: torch.device, batch_size: int = 128) -> dict:
    policy.eval()
    logits_parts = []
    encoded_parts = []
    with torch.no_grad():
        for start in range(0, len(dataset.expert_action), batch_size):
            frames = torch.from_numpy(dataset.frames[start:start + batch_size]).float()
            frames = frames.div_(255.0).to(device)
            encoded = policy.encode(frames)
            encoded_parts.append(encoded.cpu().numpy())
            logits_parts.append(policy.actor(encoded).cpu().numpy())
    logits = np.concatenate(logits_parts)
    encoded = np.concatenate(encoded_parts)
    prediction = np.argmax(logits, axis=1)
    truth = dataset.expert_action
    overall = _classification_metrics(truth, prediction)
    decision = np.asarray(dataset.decision_mask, dtype=bool)
    decision_metrics = _classification_metrics(
        truth[decision], prediction[decision]) if np.any(decision) else overall
    easy_track = (truth == 0) & ~np.asarray(dataset.hard_track_mask, dtype=bool)
    hard_track = (truth == 0) & np.asarray(dataset.hard_track_mask, dtype=bool)
    target_latent = (dataset.latent - latent_mean) / latent_scale
    expert_logit = logits[np.arange(len(truth)), truth]
    masked = logits.copy()
    masked[np.arange(len(truth)), truth] = -np.inf
    score_gap = dataset.expert_scores[np.arange(len(truth)), truth, None] - \
        dataset.expert_scores
    valid_ranking = score_gap > 1.0e-6
    valid_ranking[np.arange(len(truth)), truth] = False
    ranking_correct = expert_logit[:, None] > logits
    ranking_accuracy = (float(np.sum(ranking_correct & valid_ranking)) /
                        max(1, int(np.sum(valid_ranking))))
    return {
        **overall,
        "decision_macro_f1": decision_metrics["macro_f1"],
        "decision_balanced_accuracy": decision_metrics["balanced_accuracy"],
        "left_recall": overall["recall"][1], "right_recall": overall["recall"][2],
        "hard_track_recall": float(np.mean(prediction[hard_track] == 0))
            if np.any(hard_track) else None,
        "easy_track_false_dodge_rate": float(np.mean(prediction[easy_track] != 0))
            if np.any(easy_track) else None,
        "counterfactual_ranking_accuracy": float(ranking_accuracy),
        "latent_rmse": float(np.sqrt(np.mean(np.square(encoded - target_latent)))),
        "prediction_fraction": [float(np.mean(prediction == value))
                                for value in range(ACTION_COUNT)],
    }


def validation_gates(metrics: dict, config: dict) -> dict:
    thresholds = config.get("validation_gates", {})
    checks = {
        "left_recall": metrics["left_recall"] >= thresholds.get("left_recall", 0.80),
        "right_recall": metrics["right_recall"] >= thresholds.get("right_recall", 0.80),
        "hard_track_recall": metrics["hard_track_recall"] is None or
            metrics["hard_track_recall"] >= thresholds.get("hard_track_recall", 0.85),
        "easy_track_false_dodge_rate": metrics["easy_track_false_dodge_rate"] is None or
            metrics["easy_track_false_dodge_rate"] <=
            thresholds.get("easy_track_false_dodge_rate", 0.05),
    }
    return {**checks, "all_passed": all(checks.values())}


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
    expert_dataset = load_expert_npz(args.expert_dataset) \
        if args.expert_dataset is not None else dataset
    validation_dataset = None
    if args.validation_dataset is not None:
        validation_dataset = load_expert_npz(args.validation_dataset)
    elif args.expert_dataset is not None:
        expert_dataset, validation_dataset = split_by_episode(
            expert_dataset, float(config.get("validation_fraction", 0.20)), seed)
    # The deployed encoder is supervised primarily by the expert images.  Use
    # both real behavior transitions and the training-only expert split for
    # normalization so neither distribution is mapped with statistics from a
    # different dataset.  Validation episodes remain excluded.
    normalization_latent = np.concatenate(
        (dataset.latent, expert_dataset.latent), axis=0)
    mean = normalization_latent.mean(axis=0)
    scale = np.maximum(normalization_latent.std(axis=0), 1.0e-3)
    latent = torch.from_numpy((dataset.latent - mean) / scale).float()
    next_latent = torch.from_numpy((dataset.next_latent - mean) / scale).float()
    frames = torch.from_numpy(dataset.frames).float().div_(255.0)
    actions = torch.from_numpy(dataset.behavior_action).long()
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
    policy_optimizer = torch.optim.Adam(policy.parameters(), lr=3.0e-4)
    dynamics_optimizer = torch.optim.Adam(dynamics.parameters(), lr=5.0e-4)
    critic_optimizer = torch.optim.Adam(critic.parameters(), lr=5.0e-4)
    generator = torch.Generator().manual_seed(seed)
    iterator = batches(len(actions), int(config.get("batch_size", 64)), generator)
    gamma = float(config.get("gamma", 0.98))
    expert_batch_size = int(config.get("expert_batch_size", config.get("batch_size", 64)))
    latent_weight = float(config.get("latent_weight", 1.0))
    behavior_weight = float(config.get("behavior_cloning_weight", 1.0))
    ranking_weight = float(config.get("counterfactual_ranking_weight", 0.5))
    rl_actor_weight = float(config.get("rl_actor_weight", 0.05))
    entropy_weight = float(config.get("entropy_weight", 0.005))
    cql_weight = float(config.get("cql_weight", 0.1))
    ranking_margin = float(config.get("ranking_margin", 0.20))
    metrics = []
    best_policy = None
    best_validation_score = -float("inf")
    best_validation_gates_passed = False
    best_epoch = None
    steps_per_epoch = int(config.get("steps_per_epoch", 100))
    for epoch in range(int(config.get("epochs", 20))):
        totals = {name: 0.0 for name in
                  ("latent", "dynamics", "critic", "cql", "actor", "behavior",
                   "ranking")}
        for _ in range(steps_per_epoch):
            index = next(iterator)
            latent_target = latent[index].to(device)
            next_target = next_latent[index].to(device)
            action = actions[index].to(device)
            reward = rewards[index].to(device)
            terminal = done[index].to(device)

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
            behavior_q = critic(latent_target)
            critic_loss = functional.mse_loss(
                behavior_q.gather(1, action[:, None]).squeeze(1), real_target)
            cql_loss = (torch.logsumexp(behavior_q, dim=1) -
                        behavior_q.gather(1, action[:, None]).squeeze(1)).mean()
            critic_loss = critic_loss + cql_weight * cql_loss
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

            try:
                expert_index = balanced_expert_indices(
                    expert_dataset, expert_batch_size, generator)
            except ValueError:
                expert_index = torch.randint(
                    len(expert_dataset.expert_action),
                    (min(expert_batch_size, len(expert_dataset.expert_action)),),
                    generator=generator)
            expert_frames = torch.from_numpy(expert_dataset.frames[expert_index]).float()
            expert_frames = expert_frames.div_(255.0).to(device)
            expert_latent = torch.from_numpy(
                (expert_dataset.latent[expert_index] - mean) / scale).float().to(device)
            expert_action = torch.from_numpy(
                expert_dataset.expert_action[expert_index]).long().to(device)
            expert_scores = torch.from_numpy(
                expert_dataset.expert_scores[expert_index]).float().to(device)
            encoded = policy.encode(expert_frames)
            logits = policy.actor(encoded)
            probabilities = torch.softmax(logits, dim=1)
            entropy = -(probabilities * torch.log(probabilities + 1.0e-8)).sum(dim=1).mean()
            actor_loss = -(probabilities * critic(encoded).detach()).sum(dim=1).mean()
            behavior_loss = functional.cross_entropy(logits, expert_action)
            latent_loss = functional.mse_loss(encoded, expert_latent)
            row = torch.arange(len(expert_action), device=device)
            selected_logit = logits[row, expert_action]
            selected_score = expert_scores[row, expert_action]
            score_gap = selected_score[:, None] - expert_scores
            valid_rank = score_gap > 1.0e-6
            valid_rank[row, expert_action] = False
            margins = ranking_margin + 0.2 * torch.clamp(score_gap, 0.0, 1.0)
            rank_terms = functional.relu(margins + logits - selected_logit[:, None])
            ranking_loss = (rank_terms * valid_rank).sum() / valid_rank.sum().clamp(min=1)
            policy_loss = (latent_weight * latent_loss +
                           behavior_weight * behavior_loss +
                           ranking_weight * ranking_loss +
                           rl_actor_weight * actor_loss - entropy_weight * entropy)
            policy_optimizer.zero_grad()
            policy_loss.backward()
            policy_optimizer.step()
            with torch.no_grad():
                for target_parameter, parameter in zip(target.parameters(), critic.parameters()):
                    target_parameter.mul_(0.995).add_(parameter, alpha=0.005)
            for name, value in (("latent", latent_loss), ("dynamics", dynamics_loss),
                                ("critic", critic_loss), ("cql", cql_loss),
                                ("actor", actor_loss), ("behavior", behavior_loss),
                                ("ranking", ranking_loss)):
                totals[name] += float(value.detach())
        epoch_metrics = {"epoch": epoch + 1, **{
            name: value / steps_per_epoch for name, value in totals.items()}}
        if validation_dataset is not None:
            validation = evaluate_policy(
                policy, validation_dataset, mean, scale, device)
            validation["gates"] = validation_gates(validation, config)
            epoch_metrics["validation"] = validation
            score = validation["decision_macro_f1"]
            gates_passed = bool(validation["gates"]["all_passed"])
            if ((gates_passed and not best_validation_gates_passed) or
                    (gates_passed == best_validation_gates_passed and
                     score > best_validation_score)):
                best_validation_score = score
                best_validation_gates_passed = gates_passed
                best_policy = copy.deepcopy(policy.state_dict())
                best_epoch = epoch + 1
        metrics.append(epoch_metrics)
        print(json.dumps(metrics[-1]), flush=True)
    best_validation = None
    if best_policy is not None:
        policy.load_state_dict(best_policy)
        best_validation = evaluate_policy(
            policy, validation_dataset, mean, scale, device)
        best_validation["gates"] = validation_gates(best_validation, config)
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
        "expert_samples": int(len(expert_dataset.expert_action)),
        "validation_samples": int(len(validation_dataset.expert_action))
            if validation_dataset is not None else 0,
        "expert_dataset": str(args.expert_dataset.resolve())
            if args.expert_dataset is not None else None,
        "expert_dataset_sha256": file_sha256(args.expert_dataset)
            if args.expert_dataset is not None else None,
        "validation_dataset": str(args.validation_dataset.resolve())
            if args.validation_dataset is not None else None,
        "validation_dataset_sha256": file_sha256(args.validation_dataset)
            if args.validation_dataset is not None else None,
        "best_epoch": best_epoch,
        "best_validation_gates_passed": best_validation_gates_passed,
        "best_validation": best_validation,
        "latent_names": LATENT_NAMES, "latent_mean": mean.tolist(),
        "latent_scale": scale.tolist(), "imagined_horizon": horizon,
    }
    checkpoint_path = args.output / "checkpoint.pt"
    torch.save({"policy": policy.cpu().state_dict(), "latent_dim": latent_dim,
                "training_config": training_config}, checkpoint_path)
    (args.output / "training_metrics.json").write_text(json.dumps(
        {"config": training_config, "epochs": metrics,
         "best_epoch": best_epoch, "best_validation": best_validation},
        indent=2) + "\n")
    export(checkpoint_path, args.output / "policy.onnx",
           args.output / "bundle.json", training_config)
    return metrics[-1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expert-dataset", type=Path)
    parser.add_argument("--validation-dataset", type=Path)
    parser.add_argument("--seed", type=int, default=20260824)
    parser.add_argument("--cpu", action="store_true")
    args = parser.parse_args()
    train(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
