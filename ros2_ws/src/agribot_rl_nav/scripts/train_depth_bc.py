#!/usr/bin/env python3

import argparse
import json
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset


ACTION_SCALE = torch.tensor([0.5, 0.64], dtype=torch.float32)


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def flatten_aux_features(
    goal: np.ndarray,
    path_features: np.ndarray,
    path_points: np.ndarray,
    velocity: np.ndarray,
) -> np.ndarray:
    path_features_used = path_features[:, :4]
    path_points_flat = path_points.reshape(path_points.shape[0], -1)
    return np.concatenate([goal, path_features_used, path_points_flat, velocity], axis=1).astype(np.float32)


def list_shards(dataset_dir: str, max_shards: int) -> List[Path]:
    shards = sorted(Path(dataset_dir).glob("depth_rl_shard_*.npz"))
    if max_shards > 0:
        shards = shards[:max_shards]
    if not shards:
        raise FileNotFoundError(f"No shards found under {dataset_dir}")
    return shards


@dataclass
class ShardSequence:
    name: str
    depth: np.ndarray
    aux: np.ndarray
    action: np.ndarray


def load_dataset(shards: Sequence[Path]) -> List[ShardSequence]:
    sequences: List[ShardSequence] = []
    for shard in shards:
        data = np.load(shard)
        sequences.append(
            ShardSequence(
                name=shard.name,
                depth=data["depth"].astype(np.float32),
                aux=flatten_aux_features(
                    data["goal"].astype(np.float32),
                    data["path_features"].astype(np.float32),
                    data["path_points"].astype(np.float32),
                    data["velocity"].astype(np.float32),
                ),
                action=data["action"].astype(np.float32),
            )
        )
    return sequences


def build_window_index(
    sequences: Sequence[ShardSequence],
    chunk_size: int,
) -> List[Tuple[int, int]]:
    windows: List[Tuple[int, int]] = []
    for sequence_idx, sequence in enumerate(sequences):
        sample_count = int(sequence.action.shape[0])
        if sample_count < chunk_size:
            continue
        for end_idx in range(sample_count - chunk_size + 1):
            windows.append((sequence_idx, end_idx))
    if not windows:
        raise RuntimeError("Dataset is too small for the requested chunk size.")
    return windows


def split_indices(num_samples: int, val_split: float, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    indices = np.arange(num_samples, dtype=np.int64)
    rng = np.random.default_rng(seed)
    rng.shuffle(indices)
    val_count = max(1, int(round(num_samples * val_split)))
    val_indices = np.sort(indices[:val_count])
    train_indices = np.sort(indices[val_count:])
    return train_indices, val_indices


class ResidualBlock(nn.Module):
    def __init__(self, channels: int) -> None:
        super().__init__()
        self.block = nn.Sequential(
            nn.ReLU(inplace=True),
            nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(channels, channels, kernel_size=3, stride=1, padding=1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.block(x)


class ImpalaBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=1, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.res1 = ResidualBlock(out_channels)
        self.res2 = ResidualBlock(out_channels)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        x = self.pool(x)
        x = self.res1(x)
        x = self.res2(x)
        return x


class ImpalaDepthEncoder(nn.Module):
    def __init__(self, out_dim: int) -> None:
        super().__init__()
        self.features = nn.Sequential(
            ImpalaBlock(1, 16),
            ImpalaBlock(16, 32),
            ImpalaBlock(32, 48),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.proj = nn.Sequential(
            nn.Flatten(),
            nn.Linear(48, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.proj(self.features(x))


class ChunkedDepthBCNet(nn.Module):
    def __init__(
        self,
        aux_dim: int,
        chunk_size: int,
        hidden_dim: int = 256,
        visual_dim: int = 192,
        aux_hidden_dim: int = 128,
    ) -> None:
        super().__init__()
        self.chunk_size = chunk_size
        self.depth_encoder = ImpalaDepthEncoder(out_dim=visual_dim)
        self.aux_encoder = nn.Sequential(
            nn.Linear(aux_dim, aux_hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(aux_hidden_dim, aux_hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.token_proj = nn.Sequential(
            nn.Linear(visual_dim + aux_hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.gru = nn.GRU(input_size=hidden_dim, hidden_size=hidden_dim, num_layers=1, batch_first=True)
        self.head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, chunk_size * 2),
        )

    def forward(self, depth: torch.Tensor, aux: torch.Tensor) -> torch.Tensor:
        batch_size, seq_len = depth.shape[:2]
        depth = depth.reshape(batch_size * seq_len, 1, depth.shape[-2], depth.shape[-1])
        depth_features = self.depth_encoder(depth).reshape(batch_size, seq_len, -1)
        aux_features = self.aux_encoder(aux.reshape(batch_size * seq_len, -1)).reshape(batch_size, seq_len, -1)
        tokens = self.token_proj(torch.cat([depth_features, aux_features], dim=-1))
        outputs, _ = self.gru(tokens)
        last_hidden = outputs[:, -1, :]
        logits = self.head(last_hidden).reshape(batch_size, self.chunk_size, 2)
        return torch.tanh(logits)


class ScriptedWrapper(nn.Module):
    def __init__(
        self,
        model: ChunkedDepthBCNet,
        feature_mean: torch.Tensor,
        feature_std: torch.Tensor,
        action_scale: torch.Tensor,
    ) -> None:
        super().__init__()
        self.model = model
        self.register_buffer("feature_mean", feature_mean.float())
        self.register_buffer("feature_std", feature_std.float())
        self.register_buffer("action_scale", action_scale.float())

    def forward(
        self,
        depth: torch.Tensor,
        goal: torch.Tensor,
        path_points: torch.Tensor,
        path_features: torch.Tensor,
        velocity: torch.Tensor,
    ) -> torch.Tensor:
        path_points_flat = path_points.reshape(path_points.shape[0], path_points.shape[1], -1).float()
        aux = torch.cat(
            [
                goal.float(),
                path_features[:, :, :4].float(),
                path_points_flat,
                velocity.float(),
            ],
            dim=-1,
        )
        aux = (aux - self.feature_mean.view(1, 1, -1)) / self.feature_std.view(1, 1, -1)
        return self.model(depth.float(), aux) * self.action_scale.view(1, 1, 2)


class DepthSequenceDataset(Dataset):
    def __init__(
        self,
        sequences: Sequence[ShardSequence],
        windows: Sequence[Tuple[int, int]],
        seq_len: int,
        chunk_size: int,
        feature_mean: np.ndarray,
        feature_std: np.ndarray,
        action_scale: np.ndarray,
        training: bool,
    ) -> None:
        self.sequences = sequences
        self.windows = list(windows)
        self.seq_len = seq_len
        self.chunk_size = chunk_size
        self.feature_mean = feature_mean.astype(np.float32, copy=False)
        self.feature_std = feature_std.astype(np.float32, copy=False)
        self.action_scale = action_scale.astype(np.float32, copy=False)
        self.training = training

    def __len__(self) -> int:
        return len(self.windows)

    def _build_history(self, source: np.ndarray, end_idx: int) -> np.ndarray:
        indices = [max(0, end_idx - self.seq_len + 1 + step) for step in range(self.seq_len)]
        return source[np.asarray(indices, dtype=np.int64)]

    def __getitem__(self, item: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        sequence_idx, end_idx = self.windows[item]
        sequence = self.sequences[sequence_idx]

        depth = self._build_history(sequence.depth, end_idx)
        if self.training and random.random() < 0.35:
            noise = np.random.normal(0.0, 0.01, size=depth.shape).astype(np.float32)
            depth = np.clip(depth + noise, 0.0, 1.0)

        aux = self._build_history(sequence.aux, end_idx)
        aux = (aux - self.feature_mean[None, :]) / self.feature_std[None, :]

        target = sequence.action[end_idx : end_idx + self.chunk_size] / self.action_scale[None, :]

        return (
            torch.from_numpy(depth.astype(np.float32, copy=False)),
            torch.from_numpy(aux.astype(np.float32, copy=False)),
            torch.from_numpy(target.astype(np.float32, copy=False)),
        )


def weighted_chunk_loss(pred: torch.Tensor, target: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
    per_action = nn.functional.smooth_l1_loss(pred, target, beta=0.05, reduction="none").mean(dim=-1)
    return (per_action * weights.view(1, -1)).mean()


@torch.no_grad()
def evaluate(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
    action_scale: torch.Tensor,
    chunk_weights: torch.Tensor,
) -> Dict[str, float]:
    model.eval()
    loss_sum = 0.0
    mae_sum = 0.0
    first_mae_sum = 0.0
    linear_mae_sum = 0.0
    angular_mae_sum = 0.0
    count = 0
    for depth, aux, target in loader:
        depth = depth.to(device, non_blocking=True)
        aux = aux.to(device, non_blocking=True)
        target = target.to(device, non_blocking=True)
        pred = model(depth, aux)
        loss = weighted_chunk_loss(pred, target, chunk_weights)
        pred_scaled = pred * action_scale.view(1, 1, 2)
        target_scaled = target * action_scale.view(1, 1, 2)
        per_timestep_mae = (pred_scaled - target_scaled).abs()
        batch_size = target.shape[0]
        loss_sum += float(loss.item()) * batch_size
        mae_sum += float(per_timestep_mae.mean(dim=(1, 2)).sum().item())
        first_mae_sum += float(per_timestep_mae[:, 0, :].mean(dim=1).sum().item())
        linear_mae_sum += float(per_timestep_mae[:, :, 0].mean(dim=1).sum().item())
        angular_mae_sum += float(per_timestep_mae[:, :, 1].mean(dim=1).sum().item())
        count += batch_size
    return {
        "loss": loss_sum / max(count, 1),
        "mae": mae_sum / max(count, 1),
        "first_action_mae": first_mae_sum / max(count, 1),
        "linear_mae": linear_mae_sum / max(count, 1),
        "angular_mae": angular_mae_sum / max(count, 1),
    }


def save_run_artifacts(
    output_dir: Path,
    model: ChunkedDepthBCNet,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    action_scale: torch.Tensor,
    train_count: int,
    val_count: int,
    args: Dict[str, object],
    history: List[Dict[str, float]],
    shards: Sequence[Path],
) -> None:
    scripted = torch.jit.script(
        ScriptedWrapper(
            model=model.cpu(),
            feature_mean=torch.from_numpy(feature_mean),
            feature_std=torch.from_numpy(feature_std),
            action_scale=action_scale.cpu(),
        )
    )
    scripted.save(str(output_dir / "policy.ts"))

    checkpoint = {
        "model_state_dict": {k: v.detach().cpu() for k, v in model.state_dict().items()},
        "feature_mean": torch.from_numpy(feature_mean.copy()),
        "feature_std": torch.from_numpy(feature_std.copy()),
        "action_scale": action_scale.cpu(),
        "feature_layout": {
            "goal": 3,
            "path_features_used": 4,
            "path_points_flat": 10,
            "velocity": 2,
        },
        "model_kind": "impala_gru_chunked",
        "train_windows_count": int(train_count),
        "val_windows_count": int(val_count),
        "args": args,
        "history": history,
        "shards": [str(s) for s in shards],
    }
    torch.save(checkpoint, output_dir / "checkpoint.pt")

    best_entry = min(history, key=lambda x: x["val_loss"])
    metrics = {
        "best_val_loss": best_entry["val_loss"],
        "best_val_mae": best_entry["val_mae"],
        "best_first_action_mae": best_entry["val_first_action_mae"],
        "best_linear_mae": best_entry["val_linear_mae"],
        "best_angular_mae": best_entry["val_angular_mae"],
        "final_epoch": history[-1]["epoch"],
        "epochs": history,
    }
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train an IMPALA+GRU chunked depth BC policy.")
    default_data_root = Path.home() / ".local" / "share" / "agribot"
    parser.add_argument(
        "--dataset-dir", default=str(default_data_root / "data" / "depth_rl")
    )
    parser.add_argument(
        "--output-root", default=str(default_data_root / "models" / "depth_bc")
    )
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--val-split", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--max-shards", type=int, default=0)
    parser.add_argument("--seq-len", type=int, default=8)
    parser.add_argument("--chunk-size", type=int, default=4)
    parser.add_argument("--hidden-dim", type=int, default=256)
    parser.add_argument("--visual-dim", type=int, default=192)
    parser.add_argument("--aux-hidden-dim", type=int, default=128)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    args_dict = vars(args).copy()
    set_seed(args.seed)
    device_name = "cuda" if args.device == "auto" and torch.cuda.is_available() else ("cpu" if args.device == "auto" else args.device)
    device = torch.device(device_name)

    shards = list_shards(args.dataset_dir, args.max_shards)
    sequences = load_dataset(shards)
    windows = build_window_index(sequences, args.chunk_size)
    train_indices, val_indices = split_indices(len(windows), args.val_split, args.seed)
    train_windows = [windows[int(idx)] for idx in train_indices]
    val_windows = [windows[int(idx)] for idx in val_indices]

    train_aux = np.concatenate([sequences[s].aux[t] for s, t in train_windows], axis=0).reshape(len(train_windows), -1)
    feature_mean = train_aux.mean(axis=0).astype(np.float32)
    feature_std = train_aux.std(axis=0).astype(np.float32)
    feature_std[feature_std < 1e-6] = 1.0
    action_scale = ACTION_SCALE.clone()

    train_dataset = DepthSequenceDataset(
        sequences=sequences,
        windows=train_windows,
        seq_len=args.seq_len,
        chunk_size=args.chunk_size,
        feature_mean=feature_mean,
        feature_std=feature_std,
        action_scale=action_scale.numpy(),
        training=True,
    )
    val_dataset = DepthSequenceDataset(
        sequences=sequences,
        windows=val_windows,
        seq_len=args.seq_len,
        chunk_size=args.chunk_size,
        feature_mean=feature_mean,
        feature_std=feature_std,
        action_scale=action_scale.numpy(),
        training=False,
    )

    loader_kwargs = {
        "batch_size": args.batch_size,
        "num_workers": args.workers,
        "pin_memory": device.type == "cuda",
    }
    train_loader = DataLoader(train_dataset, shuffle=True, **loader_kwargs)
    val_loader = DataLoader(val_dataset, shuffle=False, **loader_kwargs)

    aux_dim = int(sequences[0].aux.shape[1])
    model = ChunkedDepthBCNet(
        aux_dim=aux_dim,
        chunk_size=args.chunk_size,
        hidden_dim=args.hidden_dim,
        visual_dim=args.visual_dim,
        aux_hidden_dim=args.aux_hidden_dim,
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scaler = torch.cuda.amp.GradScaler(enabled=device.type == "cuda")
    chunk_weights = torch.tensor([0.65 ** step for step in range(args.chunk_size)], dtype=torch.float32, device=device)
    chunk_weights = chunk_weights / chunk_weights.sum()

    history: List[Dict[str, float]] = []
    best_state: Dict[str, torch.Tensor] = {}
    best_val_loss = float("inf")

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss_sum = 0.0
        train_mae_sum = 0.0
        train_first_mae_sum = 0.0
        train_count = 0

        for depth_batch, aux_batch, target_batch in train_loader:
            depth_batch = depth_batch.to(device, non_blocking=True)
            aux_batch = aux_batch.to(device, non_blocking=True)
            target_batch = target_batch.to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)
            with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                pred = model(depth_batch, aux_batch)
                loss = weighted_chunk_loss(pred, target_batch, chunk_weights)

            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            scaler.step(optimizer)
            scaler.update()

            pred_scaled = pred.detach() * action_scale.to(device).view(1, 1, 2)
            target_scaled = target_batch.detach() * action_scale.to(device).view(1, 1, 2)
            mae = (pred_scaled - target_scaled).abs()
            batch_size = target_batch.shape[0]
            train_loss_sum += float(loss.item()) * batch_size
            train_mae_sum += float(mae.mean(dim=(1, 2)).sum().item())
            train_first_mae_sum += float(mae[:, 0, :].mean(dim=1).sum().item())
            train_count += batch_size

        train_loss = train_loss_sum / max(train_count, 1)
        train_mae = train_mae_sum / max(train_count, 1)
        train_first_mae = train_first_mae_sum / max(train_count, 1)
        val_metrics = evaluate(model, val_loader, device, action_scale.to(device), chunk_weights)

        history.append(
            {
                "epoch": epoch,
                "train_loss": train_loss,
                "train_mae": train_mae,
                "train_first_action_mae": train_first_mae,
                "val_loss": val_metrics["loss"],
                "val_mae": val_metrics["mae"],
                "val_first_action_mae": val_metrics["first_action_mae"],
                "val_linear_mae": val_metrics["linear_mae"],
                "val_angular_mae": val_metrics["angular_mae"],
            }
        )
        print(
            f"epoch {epoch:03d} "
            f"train_loss={train_loss:.5f} train_mae={train_mae:.5f} train_first_mae={train_first_mae:.5f} "
            f"val_loss={val_metrics['loss']:.5f} val_mae={val_metrics['mae']:.5f} "
            f"val_first_mae={val_metrics['first_action_mae']:.5f}"
        )

        if val_metrics["loss"] <= best_val_loss:
            best_val_loss = val_metrics["loss"]
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

    if not best_state:
        raise RuntimeError("Training did not produce a valid checkpoint.")

    model.load_state_dict(best_state)
    run_name = time.strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root) / run_name
    output_dir.mkdir(parents=True, exist_ok=True)
    save_run_artifacts(
        output_dir=output_dir,
        model=model,
        feature_mean=feature_mean,
        feature_std=feature_std,
        action_scale=action_scale,
        train_count=len(train_windows),
        val_count=len(val_windows),
        args=args_dict,
        history=history,
        shards=shards,
    )

    latest = Path(args.output_root) / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(output_dir.name)


if __name__ == "__main__":
    main()
