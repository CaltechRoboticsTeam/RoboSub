"""Read one YAML file; paths belong to the YAML file's directory."""

from copy import deepcopy
from pathlib import Path

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


def load_config(filename):
    path = Path(filename).expanduser().resolve()
    config = deepcopy(DEFAULTS)
    for section, value in yaml.safe_load(path.read_text()).items():
        if section == "targets":
            config[section] = value
        else:
            config[section].update(value)

    def asset(value):
        return str((path.parent / Path(value).expanduser()).resolve())

    camera = config["camera"]
    if camera["sdk_calibration"]:
        camera["sdk_calibration"] = asset(camera["sdk_calibration"])
    water = config["underwater"]
    perception = config["perception"]
    if perception["enabled"] and water["depth_source"] == "refractive":
        for key in ("housing", "rectification"):
            water[key] = asset(water[key])
    targets = []
    for target in config["targets"]:
        if not target.get("enabled", True):
            continue
        target.setdefault("min_confidence", 0.65)
        if target["method"] == "cad":
            target["mesh"] = asset(target["mesh"])
        targets.append(target)
    config["targets"] = targets
    if targets and perception["enabled"]:
        perception["detector_model"] = asset(perception["detector_model"])
    return config
