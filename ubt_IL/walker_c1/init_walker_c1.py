#!/usr/bin/env python3
"""Automated installation script for the Walker C1 robot plugin.

Installs the lerobot_robot_walker_c1 package in editable mode and verifies that
it integrates with the LeRobot framework (robot type + camera type registered).

Usage:
    python walker_c1/init_walker_c1.py             # Install
    python walker_c1/init_walker_c1.py --uninstall # Uninstall
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

PLUGIN_DIR = Path(__file__).resolve().parent / "lerobot_robot_walker_c1"


def install() -> None:
    print(f"Installing Walker C1 plugin from: {PLUGIN_DIR}")
    if not PLUGIN_DIR.is_dir():
        print(f"ERROR: Plugin directory not found: {PLUGIN_DIR}")
        sys.exit(1)

    subprocess.check_call([sys.executable, "-m", "pip", "install", "-e", str(PLUGIN_DIR)])

    print("Verifying plugin registration...")
    result = subprocess.run(
        [
            sys.executable, "-c",
            "from lerobot_robot_walker_c1 import ("
            "WalkerC1Robot, WalkerC1RobotConfig, "
            "WalkerC1Camera, WalkerC1CameraConfig); "
            "print('Import OK'); "
            "print(f'  Robot type: {WalkerC1RobotConfig().type}'); "
            "print(f'  Camera type: {WalkerC1CameraConfig().type}')"
        ],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        print(result.stdout.strip())
        print("\nWalker C1 plugin installed successfully!")
        print("\nUsage:")
        print("  lerobot-rollout --robot.type=walker_c1 --robot.joint_config=walker_c1_26d ...")
    else:
        print(f"Verification failed:\n{result.stderr}")
        sys.exit(1)


def uninstall() -> None:
    print("Uninstalling Walker C1 plugin...")
    subprocess.call(
        [sys.executable, "-m", "pip", "uninstall", "-y", "lerobot_robot_walker_c1"]
    )
    print("Done.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Walker C1 robot plugin installer")
    parser.add_argument("--uninstall", action="store_true", help="Uninstall the plugin")
    args = parser.parse_args()

    if args.uninstall:
        uninstall()
    else:
        install()


if __name__ == "__main__":
    main()
