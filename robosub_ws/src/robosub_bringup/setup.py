"""Install the master launch file and its developer menu."""

from glob import glob

from setuptools import setup

PACKAGE = "robosub_bringup"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
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
    description="Robot entry point and developer tuning tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tuning_menu = robosub_bringup.tuning_menu:main",
            "xbox_teleop = robosub_bringup.xbox_teleop:main",
            "field_test = robosub_bringup.field_test:main",
            "teensy_test = robosub_bringup.teensy_test:main",
            "teensy_relay = robosub_bringup.teensy_relay:main",
            "teensy_actuators = robosub_bringup.teensy_actuators:main",
        ],
    },
)
