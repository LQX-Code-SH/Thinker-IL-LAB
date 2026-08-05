#!/usr/bin/env python

"""
Evaluate a LeRobot ACT/Pi0.5 policy on a dataset by computing action prediction MSE.

This is the offline evaluation counterpart to rollout.sh — instead of deploying
the policy on a real robot, it runs inference frame-by-frame against a recorded
LeRobot dataset and compares predicted actions to ground-truth actions.

Usage:
    python eval_policy.py \\
        --policy-path /ubt_IL/model/Walker_S2_pick_part_real_30hz_act/checkpoints/last/pretrained_model \\
        --dataset-path /ubt_IL/dataset/Walker_S2_pick_part_real_30hz \\
        --episodes 5 --max-steps 150 --device cuda
"""

import argparse
import json
import os
import random
import sys
from pathlib import Path

import numpy as np
import torch

# Use non-interactive backend for headless container environments
try:
    import matplotlib as _mpl

    _mpl.use("Agg")
except ImportError:
    pass

from lerobot.configs import PreTrainedConfig
from lerobot.datasets import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.policies import get_policy_class, make_pre_post_processors


# ── Reproducibility ────────────────────────────────────────────────────────────

def set_seed(seed: int):
    """Set random seeds for reproducibility."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)


# ── Policy / processor loading ────────────────────────────────────────────────

def load_policy_and_processors(policy_path: str, device: str) -> tuple:
    """Load a trained policy together with its preprocessor and postprocessor.

    Args:
        policy_path: Path to the ``pretrained_model`` directory containing
            ``config.json``, ``model.safetensors``, ``policy_preprocessor.json``,
            and ``policy_postprocessor.json``.
        device: Torch device string (``"cuda"`` or ``"cpu"``).

    Returns:
        ``(policy, preprocessor, postprocessor, policy_cfg)`` where *policy* is
        an ``ACTPolicy`` (or similar ``PreTrainedPolicy`` subclass) in eval mode.
    """
    # Load policy configuration from the checkpoint
    policy_cfg = PreTrainedConfig.from_pretrained(policy_path)
    policy_cfg.pretrained_path = str(policy_path)

    # Instantiate the correct policy class and load weights
    policy_cls = get_policy_class(policy_cfg.type)
    policy = policy_cls.from_pretrained(policy_path)
    policy.to(device)
    policy.eval()

    # Load preprocessor / postprocessor pipelines (normalization + key renaming)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=str(policy_path),
        preprocessor_overrides={
            "device_processor": {"device": device},
        },
    )

    return policy, preprocessor, postprocessor, policy_cfg


# ── Dataset loading ───────────────────────────────────────────────────────────

def load_dataset(dataset_path: str, episodes: list[int] | None = None) -> LeRobotDataset:
    """Load a LeRobot v3 dataset from a local directory.

    Args:
        dataset_path: Path to the dataset root (contains ``meta/info.json``).
        episodes: Optional list of episode indices to load.  ``None`` loads all.

    Returns:
        A :class:`LeRobotDataset` ready for frame-level indexing.
    """
    repo_id = Path(dataset_path).name
    # n_obs_steps=1 for ACT → no temporal window, delta_timestamps not needed
    dataset = LeRobotDataset(
        repo_id,
        root=dataset_path,
        episodes=episodes,
        delta_timestamps=None,
    )
    return dataset


# ── Episode helpers ───────────────────────────────────────────────────────────

def get_episode_frame_indices(dataset: LeRobotDataset, episode_idx: int) -> list[int]:
    """Return the global frame indices belonging to *episode_idx*."""
    hf_dataset = dataset.hf_dataset
    ep_col = np.atleast_1d(np.asarray(hf_dataset["episode_index"]))
    return np.where(ep_col == episode_idx)[0].tolist()


# ── Single-episode evaluation ─────────────────────────────────────────────────

@torch.inference_mode()
def eval_episode(
    policy,
    preprocessor,
    postprocessor,
    dataset: LeRobotDataset,
    episode_idx: int,
    max_steps: int | None,
    device: str,
    use_amp: bool = False,
    inference_freq: int = 0,
) -> dict:
    """Run inference on an episode and compute MSE vs ground truth.

    Two modes:
    - **per‑step** (``inference_freq == 1``): every frame triggers a fresh
      ``predict_action_chunk()`` call, ``chunk[0]`` is compared to ground truth.
    - **queued** (``inference_freq > 1``): the policy is queried every *N* steps.
      The predicted chunk is denormalized and pushed into a queue, from which
      one action is popped per frame — matching real‑robot deployment with
      ``lerobot‑rollout``.

    Args:
        policy: A loaded ``PreTrainedPolicy`` subclass.
        preprocessor: Normalization / renaming pipeline.
        postprocessor: Denormalization pipeline.
        dataset: LeRobot dataset.
        episode_idx: Which episode to evaluate.
        max_steps: Cap on number of frames (``None`` = whole episode).
        device: Torch device.
        use_amp: Enable automatic mixed precision (read from policy config).
        inference_freq: Steps between policy inference calls.
            ``0`` or ``1`` = every step; ``> 1`` = queued deployment mode.

    Returns:
        Dict with keys ``episode``, ``mse``, ``mse_per_joint``, ``n_steps``,
        ``per_step_mse``, ``predictions``, ``ground_truth``.
    """
    indices = get_episode_frame_indices(dataset, episode_idx)
    if max_steps is not None:
        indices = indices[:max_steps]

    if not indices:
        return {
            "episode": episode_idx,
            "mse": float("nan"),
            "mse_per_joint": [],
            "n_steps": 0,
            "per_step_mse": [],
            "predictions": np.empty((0, 0)),
            "ground_truth": np.empty((0, 0)),
        }

    preds = []
    gts = []
    per_step_mse = []

    device_type = str(device).split(":")[0]
    autocast_enabled = use_amp and device_type == "cuda"

    # ── queued‑mode state ──────────────────────────────────────────────────
    action_queue: list[torch.Tensor] = []  # pre‑denormalized, CPU tensors [D]
    queue_step = 0  # force first inference

    for step_i, idx in enumerate(indices):
        frame = dataset[idx]

        # Ground-truth action (already a float32 tensor)
        gt_action = frame["action"]  # [D]

        # ── decide whether to run inference ──────────────────────────────
        if inference_freq <= 1:
            run_inference = True
        elif len(action_queue) == 0:
            run_inference = True
            queue_step = step_i
        elif (step_i - queue_step) >= inference_freq:
            run_inference = True
            queue_step = step_i
        else:
            run_inference = False

        if run_inference:
            # Build observation dict and preprocess
            obs = {}
            for key in dataset.meta.camera_keys:
                obs[key] = frame[key]  # [C, H, W]
            obs["observation.state"] = frame["observation.state"]  # [D]
            obs_batch = {k: v.unsqueeze(0).to(device) for k, v in obs.items()}

            with torch.autocast(device_type=device_type, enabled=autocast_enabled):
                obs_processed = preprocessor(obs_batch)
                action_chunk = policy.predict_action_chunk(obs_processed)  # [1, chunk_size, D]

            # Denormalize and queue all chunk actions (up to inference_freq)
            n_queue = inference_freq if inference_freq > 1 else 1
            n_queue = min(n_queue, action_chunk.shape[1])
            action_queue = []
            for j in range(n_queue):
                act = postprocessor(action_chunk[:, j, :]).squeeze(0).cpu().float()
                action_queue.append(act)

        # ── consume action ───────────────────────────────────────────────
        if action_queue:
            action_pred = action_queue.pop(0)
        else:
            action_pred = action_queue[0]  # unreachable, but keep pylance happy

        preds.append(action_pred.numpy())
        gts.append(gt_action.numpy())

        step_mse = float(torch.nn.functional.mse_loss(action_pred, gt_action).item())
        per_step_mse.append(step_mse)

    preds_arr = np.array(preds)  # [N, D]
    gts_arr = np.array(gts)  # [N, D]

    sq_errors = (preds_arr - gts_arr) ** 2
    mse = float(np.mean(sq_errors))
    mse_per_joint = np.mean(sq_errors, axis=0).tolist()  # [D]

    return {
        "episode": episode_idx,
        "mse": mse,
        "mse_per_joint": mse_per_joint,
        "n_steps": len(indices),
        "per_step_mse": per_step_mse,
        "predictions": preds_arr,
        "ground_truth": gts_arr,
    }


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_episode_results(
    result: dict,
    joint_names: list[str],
    plot_dir: str,
):
    """Save a per-episode prediction-vs-ground-truth comparison plot.

    Args:
        result: Dict returned by :func:`eval_episode`.
        joint_names: Human-readable names for each action dimension.
        plot_dir: Directory to write the PNG into.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not available — skipping plot.", file=sys.stderr)
        return

    os.makedirs(plot_dir, exist_ok=True)

    preds = result["predictions"]  # [N, D]
    if preds.size == 0:
        print(f"  [WARN] Episode {result['episode']} has no frames — skipping plot.", file=sys.stderr)
        return

    gts = result["ground_truth"]  # [N, D]
    ep_idx = result["episode"]
    mse = result["mse"]
    n_joints = preds.shape[1]

    # Trim joint names to match actual data dims
    if len(joint_names) > n_joints:
        joint_names = joint_names[:n_joints]
    elif len(joint_names) < n_joints:
        joint_names = list(joint_names) + [f"dim_{i}" for i in range(len(joint_names), n_joints)]

    fig, axes = plt.subplots(n_joints, 1, figsize=(14, 2.2 * max(n_joints, 1)), sharex=True)
    if n_joints == 1:
        axes = [axes]

    for j in range(n_joints):
        ax = axes[j]
        ax.plot(gts[:, j], "b-", label="Ground Truth", alpha=0.7, linewidth=1.2)
        ax.plot(preds[:, j], "r--", label="Predicted", alpha=0.7, linewidth=1.2)
        ax.set_ylabel(joint_names[j], fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Step")
    fig.suptitle(f"Episode {ep_idx}  —  MSE = {mse:.6f}", fontsize=13)
    fig.tight_layout()

    save_path = os.path.join(plot_dir, f"episode_{ep_idx:04d}.png")
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Plot saved → {save_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate a LeRobot ACT/Pi0.5 policy on a dataset (offline MSE)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--policy-path", type=str, required=True,
        help="Path to the pretrained_model directory (config.json + model.safetensors).",
    )
    parser.add_argument(
        "--dataset-path", type=str, required=True,
        help="Path to the LeRobot dataset root (contains meta/info.json).",
    )
    parser.add_argument(
        "--episodes", type=int, default=None,
        help="Number of episodes to evaluate (default: all).",
    )
    parser.add_argument(
        "--start-episode", type=int, default=0,
        help="First episode index (default: 0).",
    )
    parser.add_argument(
        "--max-steps", type=int, default=None,
        help="Max frames per episode (default: full episode).",
    )
    parser.add_argument(
        "--inference-freq", type=int, default=None,
        help=(
            "Run policy inference every N steps, queue predicted chunk actions "
            "in between (mimics real deployment). Default: policy n_action_steps. "
            "Set to 1 for per-step inference."
        ),
    )
    parser.add_argument(
        "--device", type=str, default="cuda",
        help="Torch device: 'cuda' or 'cpu' (default: cuda).",
    )
    parser.add_argument(
        "--output", type=str, default="/ubt_IL/scripts/deploy/output/eval_plots/results.json",
        help="Path to save results JSON (default: /ubt_IL/scripts/deploy/output/eval_plots/results.json).",
    )
    parser.add_argument(
        "--plot", action="store_true", default=False,
        help="Generate per-episode prediction-vs-ground-truth plots.",
    )
    parser.add_argument(
        "--plot-dir", type=str, default="/ubt_IL/scripts/deploy/output/eval_plots",
        help="Directory to save plots (default: /ubt_IL/scripts/deploy/output/eval_plots).",
    )
    parser.add_argument(
        "--seed", type=int, default=0,
        help="Random seed for reproducibility (default: 0).",
    )
    parser.add_argument(
        "--save-predictions", action="store_true", default=False,
        help="Include raw prediction and ground-truth arrays in output JSON (large file).",
    )

    args = parser.parse_args()

    # ── Validate paths ────────────────────────────────────────────────────
    policy_path = Path(args.policy_path)
    if not (policy_path / "config.json").exists():
        print(f"[ERROR] config.json not found in {policy_path}", file=sys.stderr)
        sys.exit(1)
    if not (policy_path / "model.safetensors").exists():
        print(f"[ERROR] model.safetensors not found in {policy_path}", file=sys.stderr)
        sys.exit(1)

    dataset_path = Path(args.dataset_path)
    if not (dataset_path / "meta" / "info.json").exists():
        print(f"[ERROR] meta/info.json not found in {dataset_path}", file=sys.stderr)
        sys.exit(1)

    # ── Device fallback + reproducibility ──────────────────────────────────
    if args.device == "cuda" and not torch.cuda.is_available():
        print("[INFO] CUDA not available, falling back to CPU.")
        args.device = "cpu"

    set_seed(args.seed)

    # ── 1. Load policy + processors ───────────────────────────────────────
    print(f"Loading policy  : {policy_path}")
    policy, preprocessor, postprocessor, policy_cfg = load_policy_and_processors(
        str(policy_path), args.device
    )
    action_dim = policy_cfg.output_features["action"].shape[0]
    use_amp = getattr(policy_cfg, "use_amp", False)

    # Resolve inference frequency: CLI arg → policy n_action_steps → 1 (every step)
    if args.inference_freq is not None:
        inference_freq = args.inference_freq
    else:
        inference_freq = getattr(policy_cfg, "n_action_steps", None)
        if inference_freq is None or inference_freq <= 1:
            inference_freq = 1

    print(f"  Policy type    : {policy_cfg.type}")
    print(f"  Action dim     : {action_dim}")
    print(f"  Device         : {args.device}")
    print(f"  AMP            : {use_amp}")
    print(f"  Inference freq : {inference_freq}  (chunk_size={getattr(policy_cfg, 'chunk_size', '?')})")

    # ── 2. Load dataset ───────────────────────────────────────────────────
    print(f"Loading dataset : {dataset_path}")

    # Determine which episodes to load (use lightweight metadata first)
    ds_meta = LeRobotDatasetMetadata(Path(dataset_path).name, root=str(dataset_path))
    total_episodes = ds_meta.total_episodes

    start_ep = args.start_episode
    n_eps = args.episodes if args.episodes is not None else (total_episodes - start_ep)
    end_ep = min(start_ep + n_eps, total_episodes)
    selected_episodes = list(range(start_ep, end_ep))

    dataset = load_dataset(str(dataset_path), episodes=selected_episodes)
    print(f"  Total episodes : {total_episodes}")
    print(f"  Evaluating     : {start_ep} → {end_ep - 1}  ({len(selected_episodes)} episodes)")

    # ── 3. Get joint names for plots ──────────────────────────────────────
    action_feature = dataset.meta.features.get("action", {})
    joint_names = action_feature.get("names", None)
    if joint_names is None and hasattr(policy_cfg, "action_feature_names"):
        joint_names = policy_cfg.action_feature_names
    if joint_names is None:
        joint_names = [f"joint_{i}" for i in range(action_dim)]

    # ── 4. Run evaluation ─────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Evaluating ...")
    print(f"{'='*60}")

    all_mses = []
    all_results = []

    for ep_idx in selected_episodes:
        print(f"Episode {ep_idx:4d} ...", end=" ", flush=True)
        try:
            result = eval_episode(
                policy, preprocessor, postprocessor, dataset,
                ep_idx, args.max_steps, args.device, use_amp=use_amp,
                inference_freq=inference_freq,
            )
        except Exception as e:
            print(f"SKIPPED ({e})")
            continue

        if result["n_steps"] == 0:
            print(f"SKIPPED (0 frames in episode)")
            continue

        all_mses.append(result["mse"])
        all_results.append(result)
        print(f"MSE = {result['mse']:.6f}  ({result['n_steps']} steps)")

        if args.plot:
            plot_episode_results(result, joint_names, args.plot_dir)

    if not all_mses:
        print("[ERROR] No episodes were successfully evaluated.", file=sys.stderr)
        sys.exit(1)

    # ── 5. Summary ────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Summary")
    print(f"{'='*60}")
    print(f"  Episodes       : {len(all_mses)}")
    print(f"  Mean MSE       : {np.mean(all_mses):.6f}")
    print(f"  Std MSE        : {np.std(all_mses):.6f}")
    print(f"  Min MSE        : {np.min(all_mses):.6f}")
    print(f"  Max MSE        : {np.max(all_mses):.6f}")

    # Per-episode breakdown
    print(f"\n  {'Episode':>8s}  {'MSE':>12s}  {'Steps':>8s}")
    print(f"  {'-'*8}  {'-'*12}  {'-'*8}")
    for r in all_results:
        print(f"  {r['episode']:8d}  {r['mse']:12.6f}  {r['n_steps']:8d}")

    # ── Lag analysis ─────────────────────────────────────────────────────
    if args.save_predictions:
        print(f"\n{'='*60}")
        print("Cross-correlation lag analysis")
        print(f"{'='*60}")
        for r in all_results:
            preds = r["predictions"]  # [N, D]
            gts = r["ground_truth"]
            n_steps, n_dims = preds.shape
            best_lags = []
            max_search = min(150, n_steps // 4)  # search up to ±150 frames
            for d in range(n_dims):
                # Cross-correlation: slide pred over gt, find best match
                best_corr = -np.inf
                best_lag = 0
                for lag in range(-max_search, max_search + 1):
                    if lag < 0:
                        # pred shifted left (earlier) relative to gt
                        corr = np.corrcoef(preds[:lag, d], gts[-lag:, d])[0, 1]
                    elif lag > 0:
                        corr = np.corrcoef(preds[lag:, d], gts[:-lag, d])[0, 1]
                    else:
                        corr = np.corrcoef(preds[:, d], gts[:, d])[0, 1]
                    if corr > best_corr:
                        best_corr = corr
                        best_lag = lag
                best_lags.append(best_lag)
            # Remove near-constant joints (head + grip) from summary
            active_joints = [i for i in range(n_dims)
                           if np.std(gts[:, i]) > 1e-4]
            active_lags = [best_lags[i] for i in active_joints]
            if active_lags:
                mean_lag = np.mean(active_lags)
                pos = "pred 比 GT 提前" if mean_lag < 0 else "pred 比 GT 滞后"
                print(f"  Episode {r['episode']:4d}: mean lag = {mean_lag:+.1f} 帧  ({pos})")
                print(f"  Per-joint lags:  " + "  ".join(
                    f"{j:>6.0f}" for j in best_lags))
                print(f"  Joint names:    " + "  ".join(
                    f"{joint_names[i][:6]:>6s}" if i < len(joint_names) else f"{'':>6s}"
                    for i in range(n_dims)))

    # Compute aggregate per-joint MSE
    all_mse_per_joint = np.array([r["mse_per_joint"] for r in all_results])  # [E, D]
    mean_mse_per_joint = np.mean(all_mse_per_joint, axis=0).tolist()

    # Per-joint MSE breakdown
    print(f"\n  {'Joint':>24s}  {'Mean MSE':>12s}")
    print(f"  {'-'*24}  {'-'*12}")
    n_joints_show = min(len(joint_names), len(mean_mse_per_joint))
    for j in range(n_joints_show):
        print(f"  {joint_names[j]:>24s}  {mean_mse_per_joint[j]:12.6f}")

    # ── 6. Save output JSON ───────────────────────────────────────────────

    if args.output:
        output_data = {
            "policy_path": str(policy_path),
            "dataset_path": str(dataset_path),
            "policy_type": policy_cfg.type,
            "action_dim": action_dim,
            "device": args.device,
            "seed": args.seed,
            "use_amp": use_amp,
            "inference_freq": inference_freq,
            "chunk_size": getattr(policy_cfg, "chunk_size", None),
            "joint_names": joint_names[:action_dim],
            "summary": {
                "n_episodes": len(all_mses),
                "mean_mse": float(np.mean(all_mses)),
                "std_mse": float(np.std(all_mses)),
                "min_mse": float(np.min(all_mses)),
                "max_mse": float(np.max(all_mses)),
                "mean_mse_per_joint": mean_mse_per_joint,
            },
            "episodes": [
                {
                    "episode": r["episode"],
                    "mse": r["mse"],
                    "mse_per_joint": r["mse_per_joint"],
                    "n_steps": r["n_steps"],
                    **(
                        {
                            "predictions": r["predictions"].tolist(),
                            "ground_truth": r["ground_truth"].tolist(),
                        }
                        if args.save_predictions
                        else {}
                    ),
                }
                for r in all_results
            ],
        }
        with open(args.output, "w") as f:
            json.dump(output_data, f, indent=2)
        print(f"\nResults saved → {args.output}")

    print("\nDone.")


if __name__ == "__main__":
    main()
