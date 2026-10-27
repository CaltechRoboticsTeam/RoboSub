"""Bring up the whole SIM: Gazebo + bridge + sensor stand-ins + the vehicle stack.

    ros2 launch robosub_sim pool.launch.py          # with GUI
    ros2 launch robosub_sim pool.launch.py gui:=false
    ros2 launch robosub_sim pool.launch.py gui:=false web:=true  # gzweb

The vehicle stack (mixer, estimator, controller) comes from vehicle.launch.py,
the same file the Jetson runs, so what you test here is what drives the sub.
This file only adds what the sim needs instead of hardware.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robosub_sim')
    world = os.path.join(pkg, 'worlds', 'pool.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    websocket_cfg = os.path.join(pkg, 'config', 'websocket.gzlaunch')

    gui = LaunchConfiguration('gui')
    web = LaunchConfiguration('web')

    # "-r" runs immediately, "-s" is server-only (headless).
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PythonExpression(
            ["'' if '", gui, "' == 'true' else '-s'"]), world],
        output='screen',
    )

    # Websocket server on port 9002 for gzweb (browser 3D view).
    websocket = ExecuteProcess(
        cmd=['gz', 'launch', '-v', '3', websocket_cfg],
        output='screen',
        condition=IfCondition(web),
    )

    # Gazebo topics <-> ROS topics (odom, imu, altimeter, camera, thrust).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg}],
        output='screen',
    )

    # Sensor stand-ins: the drivers the real vehicle has, faked from Gazebo.
    depth = Node(package='robosub_sim', executable='depth_sensor', output='screen')   # altimeter -> /depth
    dvl = Node(package='robosub_sim', executable='dvl_sensor', output='screen')       # ground truth -> /dvl/*

    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vehicle.launch.py')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('web', default_value='false'),
        gz, websocket, bridge, depth, dvl, vehicle,
    ])
