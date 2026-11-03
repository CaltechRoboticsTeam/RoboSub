"""Apply the shared mount constants to a Gazebo world template at launch."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from robosub_control.config.vehicle_profiles import get_profile
from robosub_control.thrusters.t200 import T200Model

from robosub_sim.vehicle_physics import configure_m5_physics


def environment_floor_z(environment):
    """Return the floor surface used by the synthetic DVL, in world metres."""
    floors = {"pool": -9.5, "bb_woollett": -2.1336}
    if environment not in floors:
        raise ValueError(f"Unknown simulation environment: {environment}")
    return floors[environment]


def _bumblebee_environment(world, asset_directory):
    for name in ("pool_floor", "gate"):
        existing = world.find(f"model[@name='{name}']")
        if existing is not None:
            world.remove(existing)
    for relative, pose in (
        ("models/woollett_pool/model.sdf", "0 0 0 0 0 0"),
        (
            "models/robosub26/gate/model.sdf",
            f"6 0 -2.1336 0 0 {math.pi / 2.0:.12g}",
        ),
    ):
        model = ET.parse(asset_directory / relative).getroot().find("model")
        if model is None:
            raise ValueError(f"Bumblebee asset has no model: {relative}")
        ET.SubElement(model, "pose").text = pose
        for element in model.iter():
            if element.tag in ("diffuse", "specular"):
                values = (element.text or "").split()
                if len(values) == 3:
                    element.text = " ".join([*values, "1"])
            if element.tag not in ("uri", "albedo_map", "emissive_map"):
                continue
            text = element.text or ""
            if text.startswith("model://"):
                path = asset_directory / "models" / text[len("model://") :]
            elif text.startswith("file://media/"):
                path = asset_directory / text[len("file://") :]
            else:
                raise ValueError(f"Unsupported Bumblebee asset path: {text}")
            if not path.is_file():
                raise ValueError(f"Missing Bumblebee asset: {path}")
            element.text = path.resolve().as_uri()
        if model.get("name") == "robosub26_gate":
            model.set("name", "gate")
            # OBJ and STL share a Y-up frame; both need the visual's roll.
            link = model.find("link[@name='gate']")
            link.find("collision/pose").text = link.findtext("visual/pose")
        world.append(model)
    world.find("model[@name='sub']/pose").text = "0 0 -1 0 0 0"


def configured_world(
    source, mesh_directory, vehicle="current_m5", environment="pool"
):
    environment_floor_z(environment)
    constants = get_profile(vehicle)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(source, parser=parser)
    model = root.find("world/model[@name='sub']")
    if model is None:
        raise ValueError("World has no sub model")
    base = model.find("link[@name='base_link']")
    if base is None:
        raise ValueError("Sub model has no base_link")
    if environment == "bb_woollett":
        _bumblebee_environment(
            root.find("world"),
            Path(mesh_directory).resolve().parent / "worlds/bb_worlds",
        )
    coefficient = None
    if constants.ACTUATOR_KIND == "m5":
        diameter = constants.M5_PROPELLER_DIAMETER_M
        # The template RPM coefficient is cosmetic and uncalibrated for M5.
        # Force-command mode receives the static thrust model's newtons.
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
    if constants.NAME == "current_m5":
        configure_m5_physics(root, model, mesh_directory)
    prefix = Path(mesh_directory).resolve().as_uri() + "/"
    for uri in root.iter("uri"):
        if uri.text and uri.text.startswith(
            "file:///ws/src/robosub_sim/meshes/"
        ):
            uri.text = prefix + uri.text.rsplit("/", 1)[-1]
    return ET.tostring(root, encoding="unicode")
