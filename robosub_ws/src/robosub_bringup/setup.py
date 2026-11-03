"""Install the robot launch files and Xbox driver station."""

from glob import glob

from setuptools import setup

PACKAGE = "robosub_bringup"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    package_data={PACKAGE: ["mac_gamepad.swift"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.py")),
        (f"share/{PACKAGE}/web", glob("web/*.html") + glob("web/*.js")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Caltech Robotics Team",
    maintainer_email="crt@caltech.edu",
    description="Robot entry point and Xbox driver station.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver_station = robosub_bringup.driver_station:main",
            "mac_gamepad = robosub_bringup.mac_gamepad:main",
        ],
    },
)
