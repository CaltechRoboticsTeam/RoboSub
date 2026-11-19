"""Apply M5 vehicle physics to Gazebo."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def passive_pitch_audit(physics, speed_m_s=1.0):
    """Estimate pitch stability at a fixed speed."""
    buoyancy_n = (physics["mass_kg"] + physics["excess_buoyancy_kg"]) * 9.81
    restoring = buoyancy_n * physics["center_of_buoyancy_m"][2]
    difference = physics["added_mass"][2] - physics["added_mass"][0]
    return {
        "speed_m_s": speed_m_s,
        "restoring_Nm_per_rad": restoring,
        "added_mass_tipping_Nm_per_rad": difference * speed_m_s**2,
        "net_restoring_Nm_per_rad": restoring - difference * speed_m_s**2,
        "scope": (
            "Fixed horizontal velocity; excludes thrust offsets, drag, "
            "control and other couplings"
        ),
    }


def _set(parent, name, value):
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = value if isinstance(value, str) else f"{value:.12g}"
    return element


def _vector(values):
    return " ".join(f"{value:.12g}" for value in values)


def _parallel_axis(mass, position):
    return mass * (
        np.dot(position, position) * np.eye(3) - np.outer(position, position)
    )


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _inertia_tensor(inertial):
    values = {
        key: float(inertial.findtext(f"inertia/{key}"))
        for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    }
    return np.array(
        [
            [values["ixx"], values["ixy"], values["ixz"]],
            [values["ixy"], values["iyy"], values["iyz"]],
            [values["ixz"], values["iyz"], values["izz"]],
        ]
    )


def _write_inertia(inertial, mass, center, tensor):
    _set(inertial, "mass", mass)
    _set(inertial, "pose", _vector([*center, 0, 0, 0]))
    inertia = inertial.find("inertia")
    if inertia is None:
        inertia = ET.SubElement(inertial, "inertia")
    for name, i, j in (
        ("ixx", 0, 0),
        ("iyy", 1, 1),
        ("izz", 2, 2),
        ("ixy", 0, 1),
        ("ixz", 0, 2),
        ("iyz", 1, 2),
    ):
        _set(inertia, name, tensor[i, j])


def _load(directory):
    physics = json.loads((directory / "current_m5_physics.json").read_text())
    geometry = json.loads((directory / "current_m5_geometry.json").read_text())
    return physics, geometry


def configure_m5_physics(root, model, mesh_directory):
    """Give contact and displacement separate links and preserve total mass."""
    physics, geometry = _load(Path(mesh_directory).resolve().parent / "config")
    base = model.find("link[@name='base_link']")
    for visual in geometry["visuals"]:
        pose = base.findtext(f"visual[@name='{visual['name']}']/pose")
        actual = [float(value) for value in pose.split()]
        if not np.allclose(actual, visual["pose_xyz_rpy"], rtol=0, atol=1e-6):
            raise ValueError("CAD snapshot is stale; regenerate geometry data")

    size = np.asarray(geometry["collision"]["size_m"], dtype=float)
    size += 2.0 * physics["collision_padding_m"]
    center = np.asarray(geometry["collision"]["center_m"], dtype=float)
    target_inertia = (
        np.asarray(geometry["inertia"]["unit_mass_tensor_m2"], dtype=float)
        * physics["mass_kg"]
    )
    if (
        size.shape != (3,)
        or center.shape != (3,)
        or target_inertia.shape != (3, 3)
        or not np.all(np.isfinite(size))
        or np.any(size <= 0)
        or not np.all(np.isfinite(center))
        or not np.all(np.isfinite(target_inertia))
        or not np.allclose(target_inertia, target_inertia.T)
    ):
        raise ValueError("Invalid CAD geometry snapshot")

    old_contact = model.find("link[@name='contact_link']")
    if old_contact is not None:
        model.remove(old_contact)
        model.remove(model.find("joint[@name='contact_joint']"))
    contact = ET.SubElement(model, "link", name="contact_link")
    contact_mass = physics["contact_link_mass_kg"]
    contact_inertia = np.diag(
        contact_mass
        / 12.0
        * np.array(
            [
                size[1] ** 2 + size[2] ** 2,
                size[0] ** 2 + size[2] ** 2,
                size[0] ** 2 + size[1] ** 2,
            ]
        )
    )
    _write_inertia(
        ET.SubElement(contact, "inertial"),
        contact_mass,
        center,
        contact_inertia,
    )
    collision = ET.SubElement(contact, "collision", name="contact_envelope")
    _set(collision, "pose", _vector([*center, 0, 0, 0]))
    ET.SubElement(
        ET.SubElement(ET.SubElement(collision, "geometry"), "box"), "size"
    ).text = _vector(size)
    joint = ET.SubElement(model, "joint", name="contact_joint", type="fixed")
    _set(joint, "parent", "base_link")
    _set(joint, "child", "contact_link")

    other_mass = contact_mass
    other_moment = contact_mass * center
    other_inertia = contact_inertia + _parallel_axis(contact_mass, center)
    for link in model.findall("link"):
        if link is base or link is contact:
            continue
        inertial = link.find("inertial")
        mass = float(inertial.findtext("mass"))
        pose = [float(value) for value in link.findtext("pose").split()]
        rotation = _rotation(pose[3:])
        position = np.asarray(pose[:3])
        other_mass += mass
        other_moment += mass * position
        other_inertia += rotation @ _inertia_tensor(inertial) @ rotation.T
        other_inertia += _parallel_axis(mass, position)
    base_mass = physics["mass_kg"] - other_mass
    if base_mass <= 0:
        raise ValueError("Vehicle mass must exceed non-hull link masses")
    base_center = -other_moment / base_mass
    base_tensor = target_inertia - other_inertia
    base_tensor -= _parallel_axis(base_mass, base_center)
    moments = np.linalg.eigvalsh(base_tensor)
    if moments[0] <= 0 or moments[-1] > moments[0] + moments[1]:
        raise ValueError("CAD-derived hull inertia is not physically valid")
    inertial = base.find("inertial")
    _write_inertia(inertial, base_mass, base_center, base_tensor)
    added_mass = _set(inertial, "fluid_added_mass", "")
    for name, value in zip(
        ("xx", "yy", "zz", "pp", "qq", "rr"), physics["added_mass"], strict=True
    ):
        _set(added_mass, name, value)
    root.set("version", "1.11")

    for existing in base.findall("collision"):
        base.remove(existing)
    displacement = ET.SubElement(base, "collision", name="buoyancy_volume")
    _set(
        displacement,
        "pose",
        _vector([*physics["center_of_buoyancy_m"], 0, 0, 0]),
    )
    volume = physics["mass_kg"] + physics["excess_buoyancy_kg"]
    volume /= physics["water_density_kg_m3"]
    buoyancy_size = [size[0], size[1], volume / (size[0] * size[1])]
    ET.SubElement(
        ET.SubElement(ET.SubElement(displacement, "geometry"), "box"), "size"
    ).text = _vector(buoyancy_size)
    contact_surface = ET.SubElement(
        ET.SubElement(displacement, "surface"), "contact"
    )
    _set(contact_surface, "collide_bitmask", "0")
    buoyancy = root.find("world/plugin[@name='gz::sim::systems::Buoyancy']")
    for enabled in buoyancy.findall("enable"):
        buoyancy.remove(enabled)
    _set(buoyancy, "enable", "sub::base_link")
    _set(
        buoyancy.find("graded_buoyancy"),
        "default_density",
        physics["water_density_kg_m3"],
    )

    hydro = model.find("plugin[@name='gz::sim::systems::Hydrodynamics']")
    for element in list(hydro):
        if element.tag != "link_name":
            hydro.remove(element)
    for names, values in (
        (("xU", "yV", "zW", "kP", "mQ", "nR"), physics["linear_damping"]),
        (
            ("xUabsU", "yVabsV", "zWabsW", "kPabsP", "mQabsQ", "nRabsR"),
            physics["quadratic_damping"],
        ),
    ):
        for name, value in zip(names, values, strict=True):
            _set(hydro, name, -value)
    _set(hydro, "disable_added_mass", "true")
