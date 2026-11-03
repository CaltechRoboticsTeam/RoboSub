"""Launch shared thruster logic and optional autonomous control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robosub_control.config import constants
from robosub_control.config.vehicle_profiles import get_profile


def start(context):
    profile = {"vehicle_profile": LaunchConfiguration("vehicle")}
    shared = {
        **profile,
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
    }
    manual = LaunchConfiguration("manual_control").perform(context) == "true"
    preview = LaunchConfiguration("preview_commands").perform(context) == "true"
    simulated = (
        LaunchConfiguration("simulate_actuators").perform(context) == "true"
    )
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
                        "max_power": ParameterValue(
                            LaunchConfiguration("max_power"), value_type=float
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
    estimator_parameters = dict(shared)
    if not simulated:
        vehicle = get_profile(LaunchConfiguration("vehicle").perform(context))
        # The existing DVL reader already corrects velocity to the Pixhawk.
        estimator_parameters["dvl_offset"] = list(
            vehicle.CONTROL_BOARD_POSITION_BODY_M
        )
        actions.append(
            Node(
                package="robosub_control",
                executable="hardware_sensors",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        )
    actions.append(
        Node(
            package="robosub_control",
            executable="state_estimator",
            output="screen",
            parameters=[estimator_parameters],
        )
    )
    if not manual:
        actions.extend(
            [
                Node(
                    package="robosub_control",
                    executable="depth_heading_controller",
                    output="screen",
                    parameters=[
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
            DeclareLaunchArgument("max_power", default_value="1.0"),
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
