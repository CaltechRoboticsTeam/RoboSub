"""Launch shared thruster logic and optional autonomous control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robosub_control.config import constants


def start(context):
    profile = {"vehicle_profile": LaunchConfiguration("vehicle")}
    shared = {
        **profile,
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
        "tuning_enabled": ParameterValue(
            LaunchConfiguration("tuning"), value_type=bool
        ),
    }
    filename = LaunchConfiguration("tuning_file").perform(context)
    files = [filename] if filename else []
    manual = LaunchConfiguration("manual_control").perform(context) == "true"
    preview = LaunchConfiguration("preview_commands").perform(context) == "true"
    actions = [
        Node(
            package="robosub_control",
            executable="thruster_mixer",
            output="screen",
            parameters=[{**profile, "use_sim_time": shared["use_sim_time"]}],
        ),
    ]
    if preview:
        actions.append(
            Node(
                package="robosub_control",
                executable="actuator_commands",
                output="screen",
                parameters=[
                    {
                        **profile,
                        "use_sim_time": shared["use_sim_time"],
                        "simulation": ParameterValue(
                            LaunchConfiguration("simulate_actuators"),
                            value_type=bool,
                        ),
                        "rate": ParameterValue(
                            LaunchConfiguration("control_rate"),
                            value_type=float,
                        ),
                        "output_rate": ParameterValue(
                            LaunchConfiguration("output_rate"), value_type=float
                        ),
                    }
                ],
            )
        )
    if not manual:
        actions.extend(
            [
                Node(
                    package="robosub_control",
                    executable="state_estimator",
                    output="screen",
                    parameters=[*files, shared],
                ),
                Node(
                    package="robosub_control",
                    executable="depth_heading_controller",
                    output="screen",
                    parameters=[
                        *files,
                        shared,
                        {
                            "manual_control": False,
                            "rate": ParameterValue(
                                LaunchConfiguration("control_rate"),
                                value_type=float,
                            ),
                            "output_rate": ParameterValue(
                                LaunchConfiguration("output_rate"),
                                value_type=float,
                            ),
                        },
                    ],
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=constants.DEFAULT_VEHICLE_PROFILE,
                choices=["current_m5", "future_t200"],
            ),
            DeclareLaunchArgument("manual_control", default_value="false"),
            DeclareLaunchArgument("preview_commands", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("simulate_actuators", default_value="false"),
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
            OpaqueFunction(function=start),
        ]
    )
