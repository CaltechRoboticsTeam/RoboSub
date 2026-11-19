#!/usr/bin/env python3
"""Import two native Calibmar camera YAML exports, without reoptimizing them."""

import argparse
import json
from pathlib import Path

import _vision  # noqa: F401
import numpy as np
from robosub_vision.refraction import PortCamera
import yaml


def convert(data, unit_scale):
    model = data["model"]
    p = np.asarray(data["parameters"], dtype=float)
    if model == "OPENCV" and len(p) == 8:
        fx, fy, cx, cy, k1, k2, p1, p2 = p
        d = [k1, k2, p1, p2, 0]
    elif model == "FULL_OPENCV" and len(p) == 12:
        fx, fy, cx, cy = p[:4]
        d = p[4:].tolist()
    elif model == "PINHOLE" and len(p) == 4:
        fx, fy, cx, cy = p
        d = [0] * 5
    else:
        raise ValueError(
            f"Unsupported Calibmar camera model: {model}; "
            "use OPENCV or FULL_OPENCV with fixed factory intrinsics"
        )
    housing = np.asarray(data["non_svp_parameters"], dtype=float)
    if housing.shape != (8,):
        raise ValueError("Calibmar export must contain eight port parameters")
    port = data["non_svp_model"]
    if port == "FLATPORT":
        housing[3:5] *= unit_scale
    elif port == "DOMEPORT":
        housing[:5] *= unit_scale
    else:
        raise ValueError("Calibmar non_svp_model must be FLATPORT or DOMEPORT")
    # Calibmar/COLMAP uses the pixel corner origin; OpenCV uses pixel centers.
    result = {
        "K": [
            [float(fx), 0, float(cx) - 0.5],
            [0, float(fy), float(cy) - 0.5],
            [0, 0, 1],
        ],
        "D": list(map(float, d)),
        "port_model": port,
        "port_parameters": housing.tolist(),
    }
    PortCamera(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True)
    parser.add_argument("--right", required=True)
    parser.add_argument("--factory", required=True)
    parser.add_argument("--length-unit", choices=("m", "mm"), required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        factory = json.loads(Path(args.factory).read_text())
        result = {
            "schema": 1,
            "units": "meters",
            "image_size": factory["image_size"],
            "source": "Calibmar native per-camera YAML; fixed factory rig",
        }
        for side in ("left", "right"):
            data = yaml.safe_load(Path(getattr(args, side)).read_text())
            if [data["width"], data["height"]] != factory["image_size"]:
                raise ValueError(
                    "Calibmar exports must use the factory capture size"
                )
            result[side] = convert(
                data, 1 if args.length_unit == "m" else 0.001
            )
            if not np.allclose(
                result[side]["K"], factory[side]["K"], atol=0.05, rtol=1e-5
            ):
                raise ValueError(
                    f"{side}: Calibmar intrinsics changed; "
                    "fix factory intrinsics during housing calibration "
                    "and add 0.5 to its principal point on input"
                )
            expected_d = np.asarray(factory[side]["D"]).ravel()
            actual_d = np.asarray(result[side]["D"])
            count = max(len(expected_d), len(actual_d))
            if not np.allclose(
                np.pad(expected_d, (0, count - len(expected_d))),
                np.pad(actual_d, (0, count - len(actual_d))),
                atol=1e-5,
            ):
                raise ValueError(f"{side}: physical lens distortion changed")
        r = np.asarray(factory["physical_R"])
        t = np.asarray(factory["physical_t"])
        transform = np.eye(4)
        transform[:3, :3] = r.T
        transform[:3, 3] = -r.T @ t
        result["T_left_right"] = transform.tolist()
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as file:
            yaml.safe_dump(result, file, sort_keys=False)
        print(f"Wrote physical housing model: {output}")
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, f"Calibmar import: {error}\n")


if __name__ == "__main__":
    main()
