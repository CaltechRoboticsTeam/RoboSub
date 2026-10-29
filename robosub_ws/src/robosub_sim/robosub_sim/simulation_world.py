"""Apply the shared mount constants to a Gazebo world template at launch."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from robosub_control.thrusters.t200 import T200Model
from robosub_control.config.vehicle_profiles import get_profile


def configured_world(source, mesh_directory, vehicle="current_m5"):
    constants = get_profile(vehicle)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(source, parser=parser)
    model = root.find("world/model[@name='sub']")
    if model is None:
        raise ValueError("World has no sub model")
    base = model.find("link[@name='base_link']")
    if base is None:
        raise ValueError("Sub model has no base_link")
    coefficient = None
    if constants.ACTUATOR_KIND == "m5":
        diameter = constants.M5_PROPELLER_DIAMETER_M
        # The template RPM coefficient is cosmetic and uncalibrated for M5.
        # Force-command mode receives the explicit simulation proxy in N.
    else:
        profile = T200Model(
            constants.THRUSTER_SUPPLY_VOLTAGE_V,
            constants.ESC_CURRENT_MODEL_LIMIT_A,
        )
        diameter = constants.T200_PROPELLER_DIAMETER_M
        omega = profile.reference_forward_rpm * 2.0 * math.pi / 60.0
        coefficient = profile.reference_forward_force_n / (
            1000.0 * omega**2 * diameter**4
        )
    for name in constants.THRUSTER_NAMES:
        topic = constants.THRUSTER_TOPIC_IDS[name]
        index = topic[1:]
        link = model.find(f"link[@name='thr_{index}']")
        if link is None or link.find("pose") is None:
            raise ValueError(f"Missing simulated thruster: {name}")
        old = [float(value) for value in link.findtext("pose").split()]
        position = constants.THRUSTER_POSITIONS_BODY_M[name]
        yaw = -math.radians(constants.THRUSTER_YAW_CW_DEG[name])
        pitch_down = math.radians(constants.THRUSTER_PITCH_DOWN_DEG[name])
        rotation = (0.0, math.pi / 2.0 + pitch_down, yaw)
        link.find("pose").text = " ".join(
            f"{value:.12g}" for value in (*position, *rotation)
        )
        visual_pose = base.find(f"visual[@name='thr_{index}_body']/pose")
        if visual_pose is not None:
            values = [float(value) for value in visual_pose.text.split()]
            values[:3] = [
                value + new - previous
                for value, new, previous in zip(
                    values[:3], position, old[:3], strict=True
                )
            ]
            visual_pose.text = " ".join(f"{value:.12g}" for value in values)
    for plugin in model.findall("plugin"):
        if plugin.get("name") != "gz::sim::systems::Thruster":
            continue
        plugin.find("propeller_diameter").text = str(diameter)
        if coefficient is not None:
            plugin.find("thrust_coefficient").text = f"{coefficient:.12g}"
    camera = base.find("sensor[@name='front_camera']/pose")
    if camera is not None:
        camera.text = " ".join(
            f"{value:.12g}"
            for value in (
                *constants.CAMERA_POSITION_BODY_M,
                *constants.CAMERA_RPY_BODY_RAD,
            )
        )
    prefix = Path(mesh_directory).resolve().as_uri() + "/"
    for uri in root.iter("uri"):
        if uri.text and uri.text.startswith(
            "file:///ws/src/robosub_sim/meshes/"
        ):
            uri.text = prefix + uri.text.rsplit("/", 1)[-1]
    return ET.tostring(root, encoding="unicode")
