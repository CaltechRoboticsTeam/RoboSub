"""Read one YAML file; paths belong to the YAML file's directory."""

from copy import deepcopy
from numbers import Real
from pathlib import Path
import re

import numpy as np
import yaml

DEFAULTS = {
    "camera": {
        "enabled": True,
        "name": "zed",
        "serial_number": 0,
        "resolution": "HD720",
        "fps": 30,
        "publish_fps": 10.0,
        "depth_mode": "NEURAL_LIGHT",
        "slam": True,
        "tracking_mode": "GEN_3",
        "compute_preference": "PREFER_CPU",
        "publish_tf": True,
        "publish_map_tf": True,
        "sdk_calibration": None,
    },
    "underwater": {
        "depth_source": "refractive",
        "housing": None,
        "rectification": None,
    },
    "perception": {
        "enabled": True,
        "detector_model": None,
        "image_size": 640,
        "device": "0",
        "max_fps": 10.0,
        "roi_stride": 3,
        "max_points": 2500,
        "min_points": 30,
        "min_depth": 0.3,
        "max_depth": 3.0,
        "max_confidence": 50.0,
        "max_ray_gap": 0.01,
        "min_ray_angle_deg": 0.5,
        "max_depth_spread": 0.15,
        "measurement_sigma": 0.02,
        "debug_clouds": False,
    },
    "frames": {
        "output": [],
        "tracking_timeout": 0.3,
        "camera_to_base": None,
    },
    "targets": [],
}
TARGET_KEYS = {
    "name",
    "enabled",
    "detector_class",
    "method",
    "min_confidence",
    "mesh",
    "units",
    "refine",
    "symmetry",
    "width",
    "height",
    "radius",
    "keypoint_indices",
    "landmarks",
    "max_rmse",
    "max_distance",
    "iterations",
    "min_fitness",
    "min_arc_deg",
}


def positive(value, name):
    if (
        not isinstance(value, Real)
        or isinstance(value, bool)
        or not np.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be finite and positive")


def load_config(filename):
    path = Path(filename).expanduser().resolve()
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError("Vision YAML must be a mapping")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"Unknown config sections: {sorted(unknown)}")
    config = deepcopy(DEFAULTS)
    for section, value in raw.items():
        if section == "targets":
            config[section] = value
            continue
        if not isinstance(value, dict):
            raise ValueError(f"{section} must be a mapping")
        unknown = set(value) - set(config[section])
        if unknown:
            raise ValueError(f"Unknown {section} options: {sorted(unknown)}")
        config[section].update(value)

    def asset(value, label):
        if not value:
            raise ValueError(f"Missing {label}")
        result = (path.parent / Path(str(value)).expanduser()).resolve()
        if not result.is_file():
            raise ValueError(f"Missing {label}: {result}")
        return str(result)

    camera, water = config["camera"], config["underwater"]
    for section in ("camera", "perception"):
        for key, default in DEFAULTS[section].items():
            if type(default) is bool and type(config[section][key]) is not bool:
                raise ValueError(f"{section}.{key} must be true or false")
    if type(camera["serial_number"]) is not int or camera["serial_number"] < 0:
        raise ValueError("camera.serial_number must be a nonnegative integer")
    if not isinstance(camera["name"], str):
        raise ValueError("camera.name must be a string")
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", camera["name"]):
        raise ValueError("camera.name must be a simple ROS name")
    if camera["depth_mode"] not in ("NEURAL_LIGHT", "NEURAL"):
        raise ValueError("camera.depth_mode must be NEURAL_LIGHT or NEURAL")
    if camera["tracking_mode"] != "GEN_3":
        raise ValueError("This launch supports GEN_3; disable SLAM if unused")
    if camera["compute_preference"] not in ("PREFER_CPU", "AUTO"):
        raise ValueError("compute_preference must be PREFER_CPU or AUTO")
    if camera["resolution"] not in ("HD720", "HD1080", "HD2K", "VGA"):
        raise ValueError("Unsupported ZED 2i resolution")
    rates = {
        "HD720": (15, 30, 60),
        "HD1080": (15, 30),
        "HD2K": (15,),
        "VGA": (15, 30, 60, 100),
    }
    if camera["fps"] not in rates[camera["resolution"]]:
        raise ValueError("Unsupported resolution/fps combination for ZED 2i")
    positive(camera["publish_fps"], "camera.publish_fps")
    if camera["publish_fps"] > camera["fps"]:
        raise ValueError("publish_fps cannot exceed capture fps")
    if camera["sdk_calibration"]:
        camera["sdk_calibration"] = asset(
            camera["sdk_calibration"], "effective SDK calibration"
        )
    if water["depth_source"] not in ("refractive", "zed"):
        raise ValueError("depth_source must be refractive or zed")
    if (
        config["perception"]["enabled"]
        and water["depth_source"] == "refractive"
    ):
        for key in ("housing", "rectification"):
            water[key] = asset(water[key], f"underwater.{key}")
        if not camera["sdk_calibration"]:
            raise ValueError("Refractive mode requires sdk_calibration")
    p = config["perception"]
    for key in (
        "max_fps",
        "roi_stride",
        "max_points",
        "min_points",
        "image_size",
        "min_depth",
        "max_depth",
        "max_ray_gap",
        "measurement_sigma",
        "max_depth_spread",
    ):
        positive(p[key], f"perception.{key}")
    for key in ("roi_stride", "max_points", "min_points", "image_size"):
        if type(p[key]) is not int:
            raise ValueError(f"perception.{key} must be an integer")
    if p["min_depth"] >= p["max_depth"]:
        raise ValueError("min_depth must be less than max_depth")
    if p["min_points"] > p["max_points"]:
        raise ValueError("min_points must not exceed max_points")
    positive(p["min_ray_angle_deg"], "min_ray_angle_deg")
    if not 0 < p["min_ray_angle_deg"] < 90:
        raise ValueError("min_ray_angle_deg must be between 0 and 90")
    if (
        not isinstance(p["max_confidence"], Real)
        or not np.isfinite(p["max_confidence"])
        or not 0 <= p["max_confidence"] <= 100
    ):
        raise ValueError("max_confidence must be between 0 and 100")
    if not isinstance(config["targets"], list):
        raise ValueError("targets must be a list")
    names = set()
    enabled = []
    for t in config["targets"]:
        if not isinstance(t, dict) or set(t) - TARGET_KEYS:
            raise ValueError(f"Invalid target definition: {t}")
        if not isinstance(t.get("name"), str) or not re.fullmatch(
            r"[A-Za-z][A-Za-z0-9_]*", t["name"]
        ):
            raise ValueError("Each target needs a simple, unique name")
        if t["name"] in names:
            raise ValueError(f"Duplicate target: {t['name']}")
        names.add(t["name"])
        if type(t.get("enabled", True)) is not bool:
            raise ValueError("target.enabled must be true or false")
        if not t.get("enabled", True):
            continue
        if t.get("method") not in ("cad", "planar", "cylinder", "position"):
            raise ValueError(f"Unsupported solver for {t['name']}")
        if not isinstance(t.get("detector_class"), str):
            raise ValueError(f"Missing detector_class for {t['name']}")
        t.setdefault("min_confidence", 0.65)
        positive(t["min_confidence"], "min_confidence")
        if not 0 < t["min_confidence"] <= 1:
            raise ValueError("min_confidence must be in (0, 1]")
        if t["method"] == "cad":
            t["mesh"] = asset(t.get("mesh"), f"mesh for {t['name']}")
            if t.get("units") not in ("meters", "millimeters"):
                raise ValueError("CAD units must be meters or millimeters")
        if t["method"] == "planar":
            for key in ("width", "height"):
                positive(t.get(key, 0), f"{t['name']}.{key}")
            indices = t.get("keypoint_indices", [0, 1, 2, 3])
            if len(indices) != 4 or len(set(indices)) != 4:
                raise ValueError("Planar solver needs four distinct keypoints")
            if any(type(i) is not int or i < 0 for i in indices):
                raise ValueError(
                    "keypoint_indices must be nonnegative integers"
                )
        if "radius" in t:
            positive(t["radius"], f"{t['name']}.radius")
        for key in (
            "max_rmse",
            "max_distance",
            "iterations",
            "min_fitness",
            "min_arc_deg",
        ):
            if key in t:
                positive(t[key], f"{t['name']}.{key}")
        if "iterations" in t and type(t["iterations"]) is not int:
            raise ValueError("CAD iterations must be an integer")
        if not 0 < t.get("min_fitness", 0.6) <= 1:
            raise ValueError("min_fitness must be in (0, 1]")
        if type(t.get("refine", True)) is not bool:
            raise ValueError("target.refine must be true or false")
        if t.get("symmetry"):
            from robosub_vision.geometry import canonical_rotation

            canonical_rotation(np.eye(3), t["symmetry"])
        for landmark in t.get("landmarks", []):
            if (
                not isinstance(landmark, dict)
                or type(landmark.get("index")) is not int
                or landmark["index"] < 0
                or np.shape(landmark.get("xyz")) != (3,)
                or not np.isfinite(landmark["xyz"]).all()
            ):
                raise ValueError(
                    "CAD landmarks need index >=0 and xyz in metres"
                )
        if (
            t["method"] == "cad"
            and not t.get("refine", True)
            and len(t.get("landmarks", [])) < 3
        ):
            raise ValueError(
                "CAD without refinement needs >=3 metric landmarks"
            )
        enabled.append(t)
    config["targets"] = enabled
    if enabled and p["enabled"]:
        p["detector_model"] = asset(p["detector_model"], "detector model")
    frames = config["frames"]
    if not isinstance(frames["output"], list):
        raise ValueError("frames.output must be a list")
    if set(frames["output"]) - {"base_link", "odom", "map"}:
        raise ValueError("Output frames are base_link, odom, map")
    positive(frames["tracking_timeout"], "frames.tracking_timeout")
    mount = frames["camera_to_base"]
    if mount is not None:
        if len(mount) != 7 or not np.isfinite(mount).all():
            raise ValueError("camera_to_base needs xyz and xyzw")
        if not np.isclose(np.linalg.norm(mount[3:]), 1, atol=1e-5):
            raise ValueError("camera_to_base quaternion must have unit norm")
        if not camera["publish_tf"]:
            raise ValueError(
                "Publish measured mount through robot URDF when "
                "another navigation system owns odometry"
            )
    return config
