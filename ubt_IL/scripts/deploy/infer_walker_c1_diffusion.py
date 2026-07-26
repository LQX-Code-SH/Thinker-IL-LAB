#!/usr/bin/env python3
"""Load a C1 Diffusion checkpoint and infer one action from a dataset frame."""

import argparse
from pathlib import Path

import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import get_policy_class, make_pre_post_processors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=0)
    args = parser.parse_args()

    config = PreTrainedConfig.from_pretrained(args.checkpoint, local_files_only=True)
    policy_class = get_policy_class(config.type)
    policy = policy_class.from_pretrained(
        args.checkpoint,
        config=config,
        local_files_only=True,
    ).to(config.device).eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=args.checkpoint,
        preprocessor_overrides={"device_processor": {"device": str(config.device)}},
    )
    dataset = LeRobotDataset(
        args.dataset.name,
        root=str(args.dataset),
        video_backend="torchcodec",
    )
    sample = dataset[args.frame]
    observation = {
        "observation.images.camera_head": sample["observation.images.camera_head"].unsqueeze(0),
        "observation.state": sample["observation.state"].unsqueeze(0),
    }

    policy.reset()
    with torch.inference_mode():
        action = postprocessor(policy.select_action(preprocessor(observation)))

    print(f"policy={type(policy).__name__}")
    print(f"frame={args.frame}")
    print(f"action_shape={tuple(action.shape)}")
    print(f"finite={bool(torch.isfinite(action).all())}")
    print(f"action={action[0].cpu().tolist()}")


if __name__ == "__main__":
    main()
