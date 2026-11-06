"""Start the ZED wrapper and perception node."""

import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robosub_vision.config import load_config
from robosub_vision.runtime import inspect_runtime


def start(context):
    filename = str(
        Path(LaunchConfiguration("config").perform(context)).resolve()
    )
    config = load_config(filename)
    c = config["camera"]
    actions = []
    if c["enabled"]:
        share, params, description = inspect_runtime(config)
        with tempfile.NamedTemporaryFile(
            mode="w", prefix="robosub-vision-", suffix=".yaml", delete=False
        ) as file:
            yaml.safe_dump({"/**": {"ros__parameters": params}}, file)
            override = file.name

        def cleanup(_context):
            Path(override).unlink(missing_ok=True)
            return []

        actions.extend(
            [
                LogInfo(msg=description),
                RegisterEventHandler(
                    OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(share / "launch/zed_camera.launch.py")
                    ),
                    launch_arguments={
                        "camera_model": "zed2i",
                        "camera_name": c["name"],
                        "serial_number": str(c["serial_number"]),
                        "ros_params_override_path": override,
                        "publish_tf": str(
                            c["slam"] and c["publish_tf"]
                        ).lower(),
                        "publish_map_tf": str(
                            c["slam"] and c["publish_map_tf"]
                        ).lower(),
                        "publish_imu_tf": "false",
                        "publish_urdf": "true",
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }.items(),
                ),
            ]
        )
    mount = config["frames"]["camera_to_base"]
    if mount:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    *map(str, mount),
                    f"{c['name']}_camera_link",
                    "base_link",
                ],
                output="screen",
            )
        )
    if config["perception"]["enabled"]:
        actions.append(
            Node(
                package="robosub_vision",
                executable="perception",
                output="screen",
                parameters=[
                    {
                        "config": filename,
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("robosub_vision"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config/vision.yaml")
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=start),
        ]
    )
