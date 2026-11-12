"""Bring up the whole SIM: Gazebo + bridge + sensor stand-ins + the vehicle stack.

    ros2 launch robosub_sim pool.launch.py          # with GUI
    ros2 launch robosub_sim pool.launch.py gui:=false

The vehicle stack (mixer, estimator, controller) comes from vehicle.launch.py,
the same file the Jetson runs, so what you test here is what drives the sub.
This file only adds what the sim needs instead of hardware.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robosub_sim')
    world = os.path.join(pkg, 'worlds', 'robosub_2026_pool.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')

    # Point Gazebo to the package and models share directory to resolve URDF/SDF models
    models_path = os.path.join(pkg, 'models')
    existing_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_sim_resource_path = (
        f"{pkg}:{models_path}:{existing_path}"
        if existing_path
        else f"{pkg}:{models_path}"
    )

    gui = LaunchConfiguration('gui')

    # "-r" runs immediately, "-s" is server-only (headless).
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PythonExpression(
            ["'' if '", gui, "' == 'true' else '-s'"]), world],
        output='screen',
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
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_sim_resource_path),
        DeclareLaunchArgument('gui', default_value='true'),
        gz, bridge, depth, dvl, vehicle,
    ])