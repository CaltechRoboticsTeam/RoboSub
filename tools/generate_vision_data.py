#!/usr/bin/env python3
"""Generate pinhole images with target pose labels."""

import argparse

import _vision  # noqa: F401
from robosub_vision.synthetic import generate_offline, load_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--seed", type=int, default=135)
    args = parser.parse_args()
    if args.frames < 10:
        parser.error("Use at least 10 frames for train/validation splits")
    try:
        generate_offline(
            load_scene(args.scene), args.output, args.frames, args.seed
        )
    except (OSError, ValueError) as error:
        parser.exit(2, f"Synthetic data: {error}\n")


if __name__ == "__main__":
    main()
