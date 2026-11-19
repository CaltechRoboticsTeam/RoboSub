from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "robosub_sim"

bb_data_files = []
asset_directories = sorted(
    {
        path.parent
        for path in Path("worlds/bb_worlds").rglob("*")
        if path.is_file()
    }
)
for directory in asset_directories:
    bb_data_files.append(
        (
            str(Path("share") / package_name / directory),
            [
                str(path)
                for path in sorted(directory.iterdir())
                if path.is_file()
            ],
        )
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/meshes", glob("meshes/*.stl")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        *bb_data_files,
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Caltech Robotics Team",
    maintainer_email="crt@caltech.edu",
    description="Minimal RoboSub AUV simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "depth_sensor = robosub_sim.depth_sensor:main",
            "dvl_sensor = robosub_sim.dvl_sensor:main",
            "dashboard = robosub_sim.dashboard:main",
            "motion_test = robosub_sim.motion_test:main",
        ],
    },
)
