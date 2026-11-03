"""Start here: choose real or simulated I/O around shared movement programs."""

import errno
import math
from pathlib import Path
import socket

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
from robosub_bringup.session import session_actions, session_arguments
from robosub_control.config import constants
from robosub_control.config.vehicle_profiles import get_profile


def include(package, filename, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / filename)),
        launch_arguments=arguments.items(),
    )


def check_driver_station_port(port):
    """Catch an existing driver station before starting another robot stack."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind(("127.0.0.1", port))
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
            raise RuntimeError(
                f"Driver station port {port} is already in use. "
                "Stop the previous RoboSub launch with Ctrl+C before "
                "starting this one. Two robot stacks on the same ROS "
                "domain can command the same simulated thrusters."
            ) from error


def start(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    profile = get_profile(argument("vehicle"))
    mode = argument("mode")
    if mode not in ("sim", "real"):
        raise ValueError("mode must be sim or real")
    simulated = mode == "sim"
    world = argument("world")
    if world not in ("pool", "bb_woollett"):
        raise ValueError("world must be pool or bb_woollett")
    driver_station = argument("driver_station") == "true"
    plots = argument("plots") == "true"
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
        simulated
        or profile.NAME != "current_m5"
        or (teleop == "off" and not mission)
    ):
        raise ValueError(
            "teensy_usb requires real current_m5 teleop or mission"
        )
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
    if simulated and vision:
        raise ValueError("Gazebo stereo perception is not implemented")
    mission_config = argument("mission_config")
    if mission_config and not mission:
        raise ValueError("mission_config requires mission:=true")
    if mission_config and not Path(mission_config).is_file():
        raise ValueError(f"Mission config does not exist: {mission_config}")
    if driver_station or teleop != "off":
        port = int(argument("driver_station_port"))
        if not 1 <= port <= 65535:
            raise ValueError("driver_station_port must be between 1 and 65535")
        check_driver_station_port(port)
    if plots:
        try:
            get_package_share_directory("rqt_plot")
        except PackageNotFoundError as error:
            raise RuntimeError(
                "Install/source rqt_plot before using plots:=true"
            ) from error
    shared = {
        "vehicle": profile.NAME,
        "manual_control": str(teleop != "off").lower(),
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
                    "world": world,
                },
            )
        ]
    else:
        actions = [
            LogInfo(
                msg=(
                    f"Real mode: actuator transport={actuators}. "
                    "Using existing MAVROS IMU, USB/MAVROS pressure and "
                    "DVL odometry through hardware_sensors. "
                    "Manual control uses body efforts directly."
                )
            ),
            include(
                "robosub_control",
                "vehicle.launch.py",
                {
                    **shared,
                    "use_sim_time": "false",
                    "simulate_actuators": "false",
                    "preview_commands": str(not physical).lower(),
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
        # if auto, do auto.
        actions.append(
            Node(
                package="robosub_control",
                executable="mission",
                output="screen",
                parameters=[
                    *([mission_config] if mission_config else []),
                    {"use_sim_time": simulated},
                ],
            )
        )
    if teleop != "off":
        actions.append(
            Node(
                package="robosub_control",
                executable="xbox_teleop",
                output="screen",
                parameters=[
                    {
                        "source": teleop,
                        "vehicle_profile": profile.NAME,
                        "max_effort": teleop_scale,
                        "manual_control": False,
                        "rate": rate,
                        "output_rate": output_rate,
                        "use_sim_time": simulated,
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
    if driver_station or teleop != "off":
        actions.append(
            Node(
                package="robosub_bringup",
                executable="driver_station",
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
    if plots:
        position_fields = [
            f"{topic}/pose/pose/position/{axis}"
            for axis in ("x", "y")
            for topic in ("/sub/reference", "/sub/state", "/sub/odom")
        ]
        for name, fields in (
            ("position_plot", position_fields),
            ("depth_plot", ["/sub/setpoint/depth/data", "/depth/data"]),
            (
                "attitude_plot",
                [f"/sub/state/rpy/vector/{axis}" for axis in ("x", "y", "z")],
            ),
        ):
            actions.append(
                Node(
                    package="rqt_plot",
                    executable="rqt_plot",
                    name=name,
                    arguments=fields,
                    parameters=[{"use_sim_time": simulated}],
                    output="screen",
                )
            )
    # Physical USB transport is selected separately from numeric previews.
    if physical:
        driver = Node(
            package="robosub_control",
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
    return session_actions(context, simulated=simulated, robot_actions=actions)


def generate_launch_description():
    """Declare vehicle, I/O and operator settings for the robot launch."""
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
                "world",
                default_value="pool",
                choices=["pool", "bb_woollett"],
                description="Gazebo environment used in simulation mode",
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
            DeclareLaunchArgument("mission_config", default_value=""),
            DeclareLaunchArgument(
                "driver_station",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "plots",
                default_value="false",
                choices=["true", "false"],
                description="Open position and depth diagnostic plots",
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
            DeclareLaunchArgument("driver_station_port", default_value="8765"),
            *session_arguments(),
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
