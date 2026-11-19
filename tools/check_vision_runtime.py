#!/usr/bin/env python3
"""Report installed vision software."""

import argparse
import json

import _vision  # noqa: F401
from robosub_vision.config import load_config
from robosub_vision.runtime import inspect_runtime, machine_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    args = parser.parse_args()
    print(json.dumps(machine_report(), indent=2))
    if args.config:
        try:
            _, _, description = inspect_runtime(load_config(args.config))
            print(description)
        except (ValueError, ImportError, OSError) as error:
            parser.exit(2, f"Runtime not ready: {error}\n")


if __name__ == "__main__":
    main()
