"""Install shared robot logic and its ROS adapters."""

from glob import glob
from setuptools import find_packages, setup

setup(
    name="robosub_control",
    version="0.1.0",
    packages=find_packages(),
    package_data={"robosub_control.thrusters": ["data/*.json", "data/*.md"]},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robosub_control"],
        ),
        ("share/robosub_control", ["package.xml"]),
        ("share/robosub_control/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Caltech Robotics Team",
    maintainer_email="crt@caltech.edu",
    description="Shared control logic, vehicle settings and hardware drivers.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "thruster_mixer = robosub_control.thrusters.thruster_mixer:main",
            "actuator_commands = robosub_control.thrusters.actuator_commands:main",
            "state_estimator = robosub_control.autonomy.state_estimator:main",
            "depth_heading_controller = robosub_control.autonomy.depth_heading_controller:main",
            "mission = robosub_control.autonomy.mission:main",
            "xbox_teleop = robosub_control.teleop.xbox_teleop:main",
            "teensy_actuators = robosub_control.drivers.teensy_actuators:main",
        ]
    },
)
