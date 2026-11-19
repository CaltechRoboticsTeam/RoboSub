"""Record each robot run and expose its topics to Foxglove."""

import json
import os
import socket
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

SESSION_DEFAULTS = {
    "record": "true",
    "log_directory": "~/.ros/robosub_runs",
    "foxglove": "true",
    "foxglove_address": "0.0.0.0",
    "foxglove_port": "8766",
}


def session_arguments():
    """Declare recording and Foxglove options."""
    return [
        DeclareLaunchArgument(
            name,
            default_value=default,
            **(
                {"choices": ["true", "false"]}
                if name in ("record", "foxglove")
                else {}
            ),
        )
        for name, default in SESSION_DEFAULTS.items()
    ]


def session_actions(context, *, simulated, robot_actions):
    """Start recording and Foxglove with the robot."""

    def argument(name):
        return LaunchConfiguration(name).perform(context)

    record = argument("record") == "true"
    foxglove = argument("foxglove") == "true"
    if record:
        get_package_share_directory("ros2bag")
        get_package_share_directory("rosbag2_storage_mcap")
    if foxglove:
        get_package_share_directory("foxglove_bridge")
        address = argument("foxglove_address")
        port = int(argument("foxglove_port"))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            try:
                listener.bind((address, port))
            except OSError as error:
                raise RuntimeError(
                    f"Cannot bind Foxglove at {address}:{port}. Stop the "
                    "previous standalone bridge/robot launch, or choose "
                    "another foxglove_port."
                ) from error

    actions = []
    if foxglove:
        actions.extend(
            [
                LogInfo(
                    msg=(
                        f"Foxglove WebSocket on {address}:{port}; "
                        f"local URL ws://127.0.0.1:{port}, "
                        f"remote URL ws://<robot-ip>:{port}"
                    )
                ),
                Node(
                    package="foxglove_bridge",
                    executable="foxglove_bridge",
                    name="foxglove_bridge",
                    parameters=[
                        {
                            "address": address,
                            "port": port,
                            "use_sim_time": simulated,
                        }
                    ],
                    output="screen",
                ),
            ]
        )
    if not record:
        return [*actions, *robot_actions]

    root = Path(argument("log_directory")).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    mode = argument("mode")
    vehicle = argument("vehicle")
    run = Path(
        tempfile.mkdtemp(
            prefix=f"{mode}-{vehicle}-{started:%Y%m%dT%H%M%S.%fZ}-", dir=root
        )
    )
    bag = run / "bag"
    settings = {
        name: argument(name)
        for name in (
            "mode",
            "vehicle",
            "world",
            "gui",
            "mission",
            "mission_config",
            "vision",
            "vision_config",
            "teleop",
            "teleop_scale",
            "actuators",
            "driver_station",
            "driver_station_port",
            "control_rate",
            "output_rate",
            *SESSION_DEFAULTS,
        )
    }
    metadata = {
        "started_utc": started.isoformat(),
        "clock": "simulation" if simulated else "system",
        "bag_directory": str(bag),
        "launch_arguments": settings,
        "ros_environment": {
            name: os.environ.get(name, "")
            for name in (
                "ROS_DISTRO",
                "ROS_DOMAIN_ID",
                "ROS_LOCALHOST_ONLY",
                "RMW_IMPLEMENTATION",
            )
        },
        "config_files": {
            name: Path(settings[name]).read_text()
            for name in ("mission_config", "vision_config")
            if settings.get(name)
        },
    }
    if not simulated:
        hardware = (
            Path(get_package_share_directory("robosub_bringup"))
            / "config/hardware.yaml"
        )
        metadata["config_files"]["hardware"] = hardware.read_text()
    (run / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    command = [
        FindExecutable(name="ros2"),
        "bag",
        "record",
        "--all",
        "--include-hidden-topics",
        "--storage",
        "mcap",
        "--storage-preset-profile",
        "zstd_fast",
        "--max-bag-size",
        str(1024**3),
        "--output",
        str(bag),
    ]
    if simulated:
        command.append("--use-sim-time")
    recorder = ExecuteProcess(
        cmd=command,
        name="run_recorder",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="10",
    )

    def recording_stopped(event, launch_context):
        if not launch_context.is_shutdown:
            raise RuntimeError(
                f"Run recorder exited unexpectedly ({event.returncode}); "
                f"stopping the robot. Recording directory: {run}"
            )
        return []

    return [
        LogInfo(msg=f"Automatic MCAP recording: {bag}"),
        RegisterEventHandler(
            OnProcessExit(target_action=recorder, on_exit=recording_stopped)
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=recorder, on_start=[*actions, *robot_actions]
            )
        ),
        recorder,
    ]
