#!/usr/bin/env python3
"""Reproduce metric meshes; panel origin is its front surface center."""

import argparse
from pathlib import Path

import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", default=("robosub_ws/src/robosub_vision/models")
    )
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    meshes = {
        "example_box": trimesh.creation.box([0.3, 0.2, 0.15]),
        "example_panel": trimesh.creation.box([0.4, 0.3, 0.004]),
        "example_cylinder": trimesh.creation.cylinder(0.025, 0.5, sections=32),
    }
    meshes["example_panel"].apply_translation([0, 0, 0.002])
    for name, mesh in meshes.items():
        path = output / f"{name}.stl"
        if path.exists():
            parser.error(f"Refusing to overwrite {path}")
        stl = trimesh.exchange.stl.export_stl_ascii(mesh)
        path.write_text(
            "\n".join(line.rstrip() for line in stl.splitlines()) + "\n"
        )


if __name__ == "__main__":
    main()
