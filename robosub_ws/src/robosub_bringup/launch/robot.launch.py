"""Start here: choose real or simulated I/O around shared movement programs."""

import math
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robosub_sim import constants
from robosub_sim.vehicle_profiles import get_profile


def include(package, filename, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / filename)),
        launch_arguments=arguments.items(),
    )


def start(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    profile = get_profile(argument("vehicle"))
    mode = argument("mode")
    if mode not in ("sim", "real"):
        raise ValueError("mode must be sim or real")
    simulated = mode == "sim"
    tuning = argument("tuning") == "true"
    mission = argument("mission") == "true"
    teleop = argument("teleop")
    if teleop not in ("off", "browser", "local"):
        raise ValueError("teleop must be off, browser or local")
    if mission and teleop != "off":
        raise ValueError("mission and teleop are mutually exclusive")
    teleop_scale = float(argument("teleop_scale"))
    if not math.isfinite(teleop_scale) or not 0 < teleop_scale <= 1:
        raise ValueError("teleop_scale must be greater than 0 and at most 1")
    actuators = argument("actuators")
    if actuators not in ("preview", "teensy_usb"):
        raise ValueError("actuators must be preview or teensy_usb")
    physical = actuators == "teensy_usb"
    if physical and (
        simulated or profile.NAME != "current_m5" or teleop == "off"
    ):
        raise ValueError("teensy_usb requires real current_m5 teleop")
    vision = argument("vision")
    vision = not simulated if vision == "auto" else vision == "true"
    rate = float(argument("control_rate"))
    output_rate = float(argument("output_rate"))
    if not math.isfinite(rate) or not (
        constants.MIN_CONTROL_RATE_HZ <= rate <= constants.MAX_CONTROL_RATE_HZ
    ):
        raise ValueError("control_rate must be between 1 and 1000 Hz")
    if not math.isfinite(output_rate) or not (
        constants.MIN_CONTROL_RATE_HZ
        <= output_rate
        <= profile.MAX_OUTPUT_RATE_HZ
    ):
        raise ValueError(
            f"output_rate must be 1-{profile.MAX_OUTPUT_RATE_HZ:g} Hz "
            f"for {profile.NAME}"
        )
    if tuning and constants.LOCK_TUNING_DEFAULTS:
        raise ValueError("Live tuning is locked in constants.py")
    if simulated and vision:
        raise ValueError("Gazebo stereo perception is not implemented")
    if not simulated and mission:
        raise ValueError("Real mission startup needs hardware drivers first")
    tuning_file = argument("tuning_file")
    if tuning_file and not Path(tuning_file).is_file():
        raise ValueError(f"Tuning file does not exist: {tuning_file}")
    if tuning_file and not tuning:
        raise ValueError("tuning_file requires tuning:=true")

    shared = {
        "vehicle": profile.NAME,
        "manual_control": str(teleop != "off").lower(),
        "tuning": str(tuning).lower(),
        "tuning_file": tuning_file,
        "control_rate": str(rate),
        "output_rate": str(output_rate),
    }
    if simulated:
        actions = [
            include(
                "robosub_sim",
                "pool.launch.py",
                {
                    **shared,
                    "gui": argument("gui"),
                },
            )
        ]
    else:
        actions = [
            LogInfo(
                msg=(
                    f"Real mode: actuator transport={actuators}. "
                    "Automatic hold needs pressure/DVL and an IMU adapter; "
                    "manual control uses body efforts directly."
                )
            ),
            include(
                "robosub_sim",
                "vehicle.launch.py",
                {
                    **shared,
                    "use_sim_time": "false",
                    "simulate_actuators": "false",
                },
            ),
        ]
    if vision:
        try:
            share = Path(get_package_share_directory("robosub_vision"))
        except PackageNotFoundError as error:
            raise RuntimeError(
                "Build/source robosub_vision before using real vision"
            ) from error
        config = argument("vision_config") or str(
            share / "config/development.yaml"
        )
        actions.append(
            include(
                "robosub_vision",
                "vision.launch.py",
                {
                    "config": config,
                    "use_sim_time": "false",
                },
            )
        )
    if mission:
        actions.append(
            Node(
                package="robosub_sim",
                executable="mission",
                output="screen",
                parameters=[{"use_sim_time": simulated}],
            )
        )
    if teleop != "off":
        actions.append(
            Node(
                package="robosub_bringup",
                executable="xbox_teleop",
                output="screen",
                parameters=[
                    {
                        "source": teleop,
                        "max_effort": teleop_scale,
                        "use_sim_time": False,
                    }
                ],
            )
        )
    if teleop == "local":
        try:
            get_package_share_directory("joy")
        except PackageNotFoundError as error:
            raise RuntimeError(
                "Install ros-humble-joy for teleop:=local"
            ) from error
        device = int(argument("joy_device"))
        if device < 0:
            raise ValueError("joy_device must be a nonnegative device index")
        actions.append(
            Node(
                package="joy",
                executable="game_controller_node",
                output="screen",
                parameters=[
                    {
                        "device_id": device,
                        # Apply the common deadzone once in our mapping code.
                        "deadzone": 0.0,
                        "autorepeat_rate": constants.GAMEPAD_PUBLISH_RATE_HZ,
                        "sticky_buttons": False,
                        "use_sim_time": False,
                    }
                ],
                remappings=[("joy", "/operator/joy")],
            )
        )
    if tuning or teleop != "off":
        port = int(argument("tuning_port"))
        if not 1 <= port <= 65535:
            raise ValueError("tuning_port must be between 1 and 65535")
        actions.append(
            Node(
                package="robosub_bringup",
                executable="tuning_menu",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "teleop_source": teleop,
                        "max_effort": teleop_scale,
                        "vehicle_profile": profile.NAME,
                        "simulation": simulated,
                        "actuator_transport": actuators,
                        "use_sim_time": False,
                    }
                ],
            )
        )
    if physical:
        driver = Node(
            package="robosub_bringup",
            executable="teensy_actuators",
            output="screen",
            parameters=[
                {
                    "device": argument("teensy_device"),
                    "max_power": teleop_scale,
                    "use_sim_time": False,
                }
            ],
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=driver,
                        on_exit=[Shutdown(reason="Teensy output stopped")],
                    )
                ),
                driver,
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
            DeclareLaunchArgument(
                "mode", default_value="sim", choices=["sim", "real"]
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "vision",
                default_value="auto",
                choices=["auto", "true", "false"],
                description="auto enables the physical ZED only in real mode",
            ),
            DeclareLaunchArgument("vision_config", default_value=""),
            DeclareLaunchArgument(
                "mission", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "tuning", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="off",
                choices=["off", "browser", "local"],
                description="Xbox on the browser computer or Linux ROS host",
            ),
            DeclareLaunchArgument(
                "teleop_scale", default_value=str(constants.GAMEPAD_MAX_EFFORT)
            ),
            DeclareLaunchArgument("joy_device", default_value="0"),
            DeclareLaunchArgument(
                "actuators",
                default_value="preview",
                choices=["preview", "teensy_usb"],
                description="Physical M5 output requires real teleop mode",
            ),
            DeclareLaunchArgument(
                "teensy_device",
                default_value=(
                    "/dev/serial/by-id/usb-Teensyduino_USB_Serial_19620450-if00"
                ),
            ),
            DeclareLaunchArgument("tuning_file", default_value=""),
            DeclareLaunchArgument("tuning_port", default_value="8765"),
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
