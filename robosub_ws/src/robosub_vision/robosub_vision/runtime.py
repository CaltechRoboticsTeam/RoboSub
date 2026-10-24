"""Version checks and explicit translation into official wrapper parameters."""

import os
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import yaml


def version_tuple(value):
    match = re.match(r"(\d+)\.(\d+)\.(\d+)", str(value))
    if not match:
        raise ValueError(f"Cannot parse version: {value}")
    return tuple(map(int, match.groups()))


def flatten(mapping, prefix=""):
    result = {}
    for key, value in mapping.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            result.update(flatten(value, name))
        else:
            result[name] = value
    return result


def wrapper_parameters(config, sdk_version):
    c = config["camera"]
    params = {
        "general.grab_resolution": c["resolution"],
        "general.grab_frame_rate": c["fps"],
        "general.pub_resolution": "NATIVE",
        "general.pub_frame_rate": float(c["publish_fps"]),
        "general.self_calib": False,
        "general.optional_opencv_calibration_file": c["sdk_calibration"] or "",
        "general.publish_status": True,
        "video.publish_rgb": True,
        "video.publish_left_right": True,
        "video.publish_raw": True,
        "depth.depth_mode": c["depth_mode"],
        "depth.min_depth": float(config["perception"]["min_depth"]),
        "depth.max_depth": float(config["perception"]["max_depth"]),
        "depth.openni_depth_mode": False,
        "depth.publish_depth_map": True,
        "depth.publish_disparity": True,
        "depth.publish_depth_confidence": True,
        "depth.publish_point_cloud": False,
        "pos_tracking.pos_tracking_enabled": c["slam"],
        "pos_tracking.pos_tracking_mode": "GEN_3",
        "pos_tracking.imu_fusion": True,
        "pos_tracking.map_frame": "map",
        "pos_tracking.odometry_frame": "odom",
        "sensors.publish_imu": True,
        "object_detection.od_enabled": False,
        "body_tracking.bt_enabled": False,
        "mapping.mapping_enabled": False,
    }
    if version_tuple(sdk_version) >= (5, 5, 0):
        params["pos_tracking.compute_preference"] = c["compute_preference"]
    return params


def inspect_runtime(config):
    """Check versions and parameter keys without opening a camera."""
    from ament_index_python.packages import get_package_share_directory
    import pyzed.sl as sl

    if os.environ.get("ROS_DISTRO") != "humble":
        raise ValueError("Source ROS 2 Humble before launching vision")
    sdk = sl.Camera.get_sdk_version()
    if version_tuple(sdk) < (5, 2, 1):
        raise ValueError(
            "GEN_3 integration requires ZED SDK >=5.2.1; "
            "5.2.0 has known tracking issues"
        )
    share = Path(get_package_share_directory("zed_wrapper"))
    wrapper_version = ET.parse(share / "package.xml").findtext("version")
    if version_tuple(wrapper_version) < (5, 5, 0):
        raise ValueError(
            "Use the wrapper revision in robosub_ws/vision.repos; "
            "older wrappers have different topic/parameter names"
        )
    known = {}
    for filename in (
        "common_stereo.yaml",
        "zed2i.yaml",
        "object_detection.yaml",
    ):
        data = yaml.safe_load((share / "config" / filename).read_text())
        known.update(flatten(data["/**"]["ros__parameters"]))
    params = wrapper_parameters(config, sdk)
    missing = set(params) - set(known)
    if missing:
        raise ValueError(
            f"Installed wrapper lacks parameters: {sorted(missing)}"
        )
    description = f"ZED SDK {sdk}, wrapper {wrapper_version}"
    if version_tuple(sdk) < (5, 5, 0):
        description += "; compute_preference unavailable, using SDK default"
    return share, params, description


def machine_report():
    """Inspect local software only; no USB probing, SSH, or camera ownership."""
    report = {"ROS_DISTRO": os.environ.get("ROS_DISTRO", "unavailable")}
    for path in ("/etc/nv_tegra_release", "/etc/os-release"):
        if Path(path).is_file():
            report[path] = Path(path).read_text().strip()
    for name, command in (
        (
            "JetPack/L4T",
            ["dpkg-query", "-W", "nvidia-jetpack", "nvidia-l4t-core"],
        ),
        (
            "CUDA compiler",
            [
                "/usr/local/cuda/bin/nvcc"
                if Path("/usr/local/cuda/bin/nvcc").is_file()
                else "nvcc",
                "--version",
            ],
        ),
    ):
        try:
            result = subprocess.run(
                command, text=True, capture_output=True, timeout=10, check=False
            )
            report[name] = (result.stdout or result.stderr).strip()
        except (OSError, subprocess.TimeoutExpired):
            report[name] = "unavailable"
    try:
        import pyzed.sl as sl

        report["ZED SDK"] = sl.Camera.get_sdk_version()
    except (ImportError, OSError, ValueError):
        report["ZED SDK"] = "unavailable"
    return report
