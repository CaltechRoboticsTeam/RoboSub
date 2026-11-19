"""Run the Woollett five-metre move and depth-hold simulation scenario."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robosub_bringup.session import SESSION_DEFAULTS, session_arguments
from robosub_control.config import constants


def score_finished(event, context):
    """Finish scoring normally; surface an observer process error."""
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError("Simulation observer crashed; see its output")
    return [Shutdown(reason="Simulation scoring finished")]


def start(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    config = argument("mission_config")
    share = Path(get_package_share_directory("robosub_bringup"))
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/robot.launch.py")),
        launch_arguments={
            "mode": "sim",
            "world": "bb_woollett",
            "vehicle": argument("vehicle"),
            "gui": argument("gui"),
            "mission": "true",
            "mission_config": config,
            "vision": "false",
            "teleop": "off",
            "actuators": "preview",
            "driver_station": argument("driver_station"),
            "driver_station_port": argument("driver_station_port"),
            "plots": argument("plots"),
            **{name: argument(name) for name in SESSION_DEFAULTS},
        }.items(),
    )
    scorer = Node(
        package="robosub_sim",
        executable="motion_test",
        output="screen",
        parameters=[
            config,
            {"use_sim_time": True},
        ],
    )
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=scorer,
                on_exit=score_finished,
            )
        ),
        scorer,
        robot,
    ]


def generate_launch_description():
    sim = Path(get_package_share_directory("robosub_sim"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=constants.DEFAULT_VEHICLE_PROFILE,
                choices=["current_m5", "future_t200"],
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "driver_station",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("driver_station_port", default_value="8765"),
            *session_arguments(),
            DeclareLaunchArgument(
                "plots", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "mission_config",
                default_value=str(sim / "config/move5_hold.yaml"),
            ),
            OpaqueFunction(function=start),
        ]
    )
