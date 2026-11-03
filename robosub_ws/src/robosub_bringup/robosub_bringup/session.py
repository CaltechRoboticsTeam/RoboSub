"""Record each robot run and expose its topics to Foxglove."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import tempfile

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
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
    """Share recording/connection options between robot entry points."""
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


def require_package(package):
    """Reject an incomplete environment before launching robot processes."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError as error:
        name = package.replace("_", "-")
        raise RuntimeError(
            f"Automatic run logging needs {package}. On the Mac, run "
            f"'pixi add ros-humble-{name}' in ~/Developer/robosub-ros, "
            "then setup_ros_shell. On Ubuntu Humble, install "
            f"'ros-humble-{name}' with apt."
        ) from error


def session_actions(context, *, simulated, robot_actions):
    """Start observation before the robot and stop it with the same launch."""

    def argument(name):
        return LaunchConfiguration(name).perform(context)

    record = argument("record") == "true"
    foxglove = argument("foxglove") == "true"
    if record:
        require_package("ros2bag")
        require_package("rosbag2_storage_mcap")
    if foxglove:
        require_package("foxglove_bridge")
        address = argument("foxglove_address")
        port = int(argument("foxglove_port"))
        if not 1 <= port <= 65535:
            raise ValueError("foxglove_port must be between 1 and 65535")
        menu = (
            argument("driver_station") == "true" or argument("teleop") != "off"
        )
        if menu and port == int(argument("driver_station_port")):
            raise ValueError(
                "Foxglove and the driver station need different ports"
            )
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
            if settings[name]
        },
    }
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
