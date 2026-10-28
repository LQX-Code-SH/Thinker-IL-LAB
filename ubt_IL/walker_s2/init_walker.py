#!/usr/bin/env python3
"""Automated installation script for the Walker S2 robot plugin.

Installs the lerobot_robot_walker package in editable mode and verifies
that it integrates with the LeRobot framework.

Usage:
    python walker/init_walker.py          # Install
    python walker/init_walker.py --uninstall  # Uninstall
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

PLUGIN_DIR = Path(__file__).resolve().parent / "lerobot_robot_walker"


def install() -> None:
    print(f"Installing Walker plugin from: {PLUGIN_DIR}")
    if not PLUGIN_DIR.is_dir():
        print(f"ERROR: Plugin directory not found: {PLUGIN_DIR}")
        sys.exit(1)

    # Install in editable mode
    subprocess.check_call(
        [sys.executable, "-m", "pip", "install", "-e", str(PLUGIN_DIR)]
    )

    # Verify import
    print("Verifying plugin registration...")
    result = subprocess.run(
        [
            sys.executable, "-c",
            "from lerobot_robot_walker import ("
            "WalkerRobot, WalkerRobotConfig, "
            "WalkerCamera, WalkerCameraConfig); "
            "print('Import OK'); "
            "print(f'  Robot type: {WalkerRobotConfig.type}'); "
            "print(f'  Camera type: {WalkerCameraConfig.type}')"
        ],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        print(result.stdout.strip())
        print("\nWalker plugin installed successfully!")
        print("\nUsage:")
        print("  lerobot-rollout --robot.type=walker --policy.path=<model> ...")
    else:
        print(f"Verification failed:\n{result.stderr}")
        sys.exit(1)


def uninstall() -> None:
    print("Uninstalling Walker plugin...")
    subprocess.call(
        [sys.executable, "-m", "pip", "uninstall", "-y", "lerobot_robot_walker"]
    )
    print("Done.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Walker S2 robot plugin installer")
    parser.add_argument(
        "--uninstall", action="store_true", help="Uninstall the plugin"
    )
    args = parser.parse_args()

    if args.uninstall:
        uninstall()
    else:
        install()


if __name__ == "__main__":
    main()
