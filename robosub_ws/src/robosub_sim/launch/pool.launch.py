"""Simulation environment plus the shared movement launch description."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from robosub_sim import constants
from robosub_sim.simulation_world import configured_world


def start_world(context):
    package = Path(get_package_share_directory("robosub_sim"))
    text = configured_world(
        (package / "worlds/pool.sdf").read_text(),
        package / "meshes",
        LaunchConfiguration("vehicle").perform(context),
    )
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="robosub-world-", suffix=".sdf", delete=False
    ) as file:
        file.write(text)
        world = Path(file.name)

    def cleanup(_context):
        world.unlink(missing_ok=True)
        return []

    gui = LaunchConfiguration("gui")
    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            PythonExpression(["'' if '", gui, "' == 'true' else '-s'"]),
            str(world),
        ],
        output="screen",
    )
    return [
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])
        ),
        gazebo,
    ]


def generate_launch_description():
    package = Path(get_package_share_directory("robosub_sim"))
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": str(package / "config/bridge.yaml")}],
        output="screen",
    )
    sensors = [
        Node(
            package="robosub_sim",
            executable=executable,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "vehicle_profile": LaunchConfiguration("vehicle"),
                }
            ],
        )
        for executable in ("depth_sensor", "dvl_sensor")
    ]
    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package / "launch/vehicle.launch.py")
        ),
        launch_arguments={
            "vehicle": LaunchConfiguration("vehicle"),
            "manual_control": LaunchConfiguration("manual_control"),
            "use_sim_time": "true",
            "simulate_actuators": "true",
            "tuning": LaunchConfiguration("tuning"),
            "tuning_file": LaunchConfiguration("tuning_file"),
            "control_rate": LaunchConfiguration("control_rate"),
            "output_rate": LaunchConfiguration("output_rate"),
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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("tuning", default_value="false"),
            DeclareLaunchArgument("tuning_file", default_value=""),
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
            *sensors,
            vehicle,
        ]
    )
