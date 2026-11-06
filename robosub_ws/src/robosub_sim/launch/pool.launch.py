"""Start Gazebo and vehicle control."""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robosub_control.config import constants
from robosub_sim.simulation_world import configured_world, environment_floor_z


def start_world(context):
    package = Path(get_package_share_directory("robosub_sim"))
    environment = LaunchConfiguration("world").perform(context)
    text = configured_world(
        (package / "worlds/pool.sdf").read_text(),
        package / "meshes",
        LaunchConfiguration("vehicle").perform(context),
        environment=environment,
    )
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="robosub-world-", suffix=".sdf", delete=False
    ) as file:
        file.write(text)
        world = Path(file.name)

    def cleanup(_context):
        world.unlink(missing_ok=True)
        return []

    # Native macOS requires server and GUI in separate processes.
    processes = [
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", str(world)],
            name="gazebo_server",
            output="screen",
        )
    ]
    if LaunchConfiguration("gui").perform(context) == "true":
        processes.append(
            ExecuteProcess(
                cmd=["gz", "sim", "-g"],
                name="gazebo_gui",
                output="screen",
            )
        )
    sensors = []
    for executable in ("depth_sensor", "dvl_sensor"):
        parameters = {
            "use_sim_time": True,
            "vehicle_profile": LaunchConfiguration("vehicle"),
        }
        if executable == "dvl_sensor":
            parameters["floor_z"] = environment_floor_z(environment)
        sensors.append(
            Node(
                package="robosub_sim",
                executable=executable,
                output="screen",
                parameters=[parameters],
            )
        )
    return [
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])
        ),
        *[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=process,
                    on_exit=[Shutdown(reason="Gazebo stopped")],
                )
            )
            for process in processes
        ],
        *processes,
        *sensors,
    ]


def generate_launch_description():
    package = Path(get_package_share_directory("robosub_sim"))
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": str(package / "config/bridge.yaml")}],
        output="screen",
    )
    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("robosub_control"))
                / "launch/vehicle.launch.py"
            )
        ),
        launch_arguments={
            "vehicle": LaunchConfiguration("vehicle"),
            "manual_control": LaunchConfiguration("manual_control"),
            "use_sim_time": "true",
            "simulate_actuators": "true",
            "control_rate": LaunchConfiguration("control_rate"),
            "output_rate": LaunchConfiguration("output_rate"),
            "max_power": LaunchConfiguration("max_power"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=constants.DEFAULT_VEHICLE_PROFILE,
                choices=["current_m5", "future_t200"],
            ),
            DeclareLaunchArgument("manual_control", default_value="false"),
            DeclareLaunchArgument("max_power", default_value="1.0"),
            DeclareLaunchArgument(
                "world", default_value="pool", choices=["pool", "bb_woollett"]
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "control_rate",
                default_value=str(constants.CONTROLLER_DEFAULTS["rate"]),
            ),
            DeclareLaunchArgument(
                "output_rate",
                default_value=str(constants.CONTROLLER_DEFAULTS["output_rate"]),
            ),
            OpaqueFunction(function=start_world),
            bridge,
            vehicle,
        ]
    )
