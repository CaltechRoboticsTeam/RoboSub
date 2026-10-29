"""Shared movement programs and profile-specific command previews.

The same estimator, controller and mixer execute in simulation and real mode.
Sensor drivers must supply body-frame /imu/data_raw, /depth and /dvl/velocity.
actuator_commands previews M5 power or T200 PWM. The master launcher can add the M5 USB output
transport for real manual control. Sensor adapters remain separate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robosub_sim import constants


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
    return [
        Node(
            package="robosub_sim",
            executable="thruster_mixer",
            output="screen",
            parameters=[{**profile, "use_sim_time": shared["use_sim_time"]}],
        ),
        Node(
            package="robosub_sim",
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
                        LaunchConfiguration("control_rate"), value_type=float
                    ),
                    "output_rate": ParameterValue(
                        LaunchConfiguration("output_rate"), value_type=float
                    ),
                }
            ],
        ),
        Node(
            package="robosub_sim",
            executable="state_estimator",
            output="screen",
            parameters=[*files, shared],
        ),
        Node(
            package="robosub_sim",
            executable="depth_heading_controller",
            output="screen",
            parameters=[
                *files,
                shared,
                {
                    "manual_control": ParameterValue(
                        LaunchConfiguration("manual_control"), value_type=bool
                    ),
                    "rate": ParameterValue(
                        LaunchConfiguration("control_rate"), value_type=float
                    ),
                    "output_rate": ParameterValue(
                        LaunchConfiguration("output_rate"), value_type=float
                    ),
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=constants.DEFAULT_VEHICLE_PROFILE,
                choices=["current_m5", "future_t200"],
            ),
            DeclareLaunchArgument("manual_control", default_value="false"),
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
