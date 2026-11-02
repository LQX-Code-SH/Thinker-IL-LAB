#!/usr/bin/env python3
"""Load a Walker C1 checkpoint and infer one action from a dataset frame.

Sanity check for "converted dataset + trained policy" without touching hardware:
verifies the policy interface (13/26 DOF), the `observation.images.camera_head`
key and that the produced action is finite with the expected shape.

Usage:
  python infer_check.py \
    --checkpoint /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
    --dataset /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
    --frame 0
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import get_policy_class, make_pre_post_processors

IMAGE_KEY = "observation.images.camera_head"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--device", default=None, help="覆盖 checkpoint 里的 device（如 cpu）")
    args = parser.parse_args()

    config = PreTrainedConfig.from_pretrained(args.checkpoint, local_files_only=True)
    if args.device:
        config.device = args.device

    policy_class = get_policy_class(config.type)
    policy = policy_class.from_pretrained(args.checkpoint, config=config, local_files_only=True)
    policy = policy.to(config.device).eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=args.checkpoint,
        preprocessor_overrides={"device_processor": {"device": str(config.device)}},
    )

    dataset = LeRobotDataset(args.dataset.name, root=str(args.dataset), video_backend="torchcodec")
    sample = dataset[args.frame]
    if IMAGE_KEY not in sample:
        raise SystemExit(f"[infer_check] dataset has no {IMAGE_KEY}; keys={sorted(sample)}")

    observation = {
        IMAGE_KEY: sample[IMAGE_KEY].unsqueeze(0),
        "observation.state": sample["observation.state"].unsqueeze(0),
    }

    policy.reset()
    with torch.inference_mode():
        action = postprocessor(policy.select_action(preprocessor(observation)))

    action = action[0].cpu()
    print(f"policy={type(policy).__name__}")
    print(f"frame={args.frame}")
    print(f"state_dim={tuple(sample['observation.state'].shape)}")
    print(f"action_shape={tuple(action.shape)}")
    print(f"finite={bool(torch.isfinite(action).all())}")
    print("action=" + "[" + ", ".join(f"{v:.4f}" for v in action.tolist()) + "]")

    expected = config.output_features.get("action")
    expected_dim = int(expected.shape[0]) if expected is not None else None
    if expected_dim is not None and tuple(action.shape) != (expected_dim,):
        raise SystemExit(
            f"[infer_check] 期望 {expected_dim} 维动作（来自 checkpoint 配置），"
            f"实际 {tuple(action.shape)}"
        )


if __name__ == "__main__":
    main()
