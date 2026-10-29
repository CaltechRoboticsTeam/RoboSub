"""ROS package installation."""

from glob import glob

from setuptools import setup

setup(
    name="robosub_vision",
    version="0.1.0",
    packages=["robosub_vision"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robosub_vision"],
        ),
        ("share/robosub_vision", ["package.xml", "README.md"]),
        ("share/robosub_vision/launch", glob("launch/*.py")),
        ("share/robosub_vision/config", glob("config/*.yaml")),
        ("share/robosub_vision/models", glob("models/*.stl")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "PyYAML"],
    maintainer="Caltech Robotics Team",
    maintainer_email="crt@caltech.edu",
    description="Configurable stereo underwater object geometry.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception = robosub_vision.perception_node:main",
            "record_geometry = robosub_vision.record_geometry:main",
        ]
    },
)
