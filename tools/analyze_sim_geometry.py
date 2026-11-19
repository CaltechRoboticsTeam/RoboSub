#!/usr/bin/env python3
"""Extract simulation geometry and inertia estimates from CAD meshes."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = Path("robosub_ws/src/robosub_sim")
SNAPSHOT = PACKAGE / "config/current_m5_geometry.json"
ANALYSIS_MESHES = {
    "hull_lowpoly.stl": "urdf_base_link_assembly.stl",
    "thruster_lowpoly.stl": "video_ray_thruster.stl",
}


def fingerprint(value):
    """Hash a portable canonical JSON representation."""
    data = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(data.encode()).hexdigest()


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def geometry_inputs(root=ROOT):
    """Reproduce current body-visual translations without loading the world."""
    control = root / "robosub_ws/src/robosub_control"
    if str(control) not in sys.path:
        sys.path.insert(0, str(control))
    from robosub_control.config import current_m5

    model = ET.parse(root / PACKAGE / "worlds/pool.sdf").find(
        "world/model[@name='sub']"
    )
    base = model.find("link[@name='base_link']")
    if base.findtext("pose", "0 0 0 0 0 0") != "0 0 0 0 0 0":
        raise ValueError("Expected base_link at the model origin")
    translations = {}
    mounts = {}
    for name in current_m5.THRUSTER_NAMES:
        index = current_m5.THRUSTER_TOPIC_IDS[name][1:]
        old = list(
            map(
                float, model.findtext(f"link[@name='thr_{index}']/pose").split()
            )
        )
        position = list(current_m5.THRUSTER_POSITIONS_BODY_M[name])
        translations[f"thr_{index}_body"] = [
            new - previous
            for new, previous in zip(position, old[:3], strict=True)
        ]
        mounts[name] = {"template_link_pose": old, "profile_position": position}
    visuals = []
    for element in base.findall("visual"):
        mesh = element.find("geometry/mesh")
        if mesh is None:
            continue
        pose_element = element.find("pose")
        if pose_element is not None and pose_element.get("relative_to"):
            raise ValueError(
                "Relative visual frames require an explicit resolver"
            )
        mesh_uri = mesh.findtext("uri")
        mesh_name = Path(mesh_uri).name
        if not mesh_uri.endswith(f"/robosub_sim/meshes/{mesh_name}"):
            raise ValueError("Expected a mesh from robosub_sim/meshes")
        if mesh_name not in ANALYSIS_MESHES:
            raise ValueError(f"No analysis counterpart for {mesh_name}")
        original = list(map(float, element.findtext("pose").split()))
        pose = original.copy()
        if element.get("name") in translations:
            delta = translations[element.get("name")]
            pose[:3] = [a + b for a, b in zip(pose[:3], delta, strict=True)]
            # Match simulation_world's serialization of translated poses.
            pose = [float(f"{value:.12g}") for value in pose]
        scale = list(map(float, mesh.findtext("scale", "1 1 1").split()))
        if len(pose) != 6 or len(scale) != 3:
            raise ValueError("Expected six pose and three scale coordinates")
        if (
            not all(math.isfinite(v) for v in (*pose, *scale))
            or min(scale) <= 0
        ):
            raise ValueError("Geometry transforms must be finite and positive")
        visuals.append(
            {
                "name": element.get("name"),
                "template_mesh_uri": mesh_uri,
                "rendered_mesh": mesh_name,
                "analysis_mesh": ANALYSIS_MESHES[mesh_name],
                "template_pose_xyz_rpy": original,
                "pose_xyz_rpy": pose,
                "scale": scale,
            }
        )
    if len(visuals) != 7:
        raise ValueError("Expected the hull and six motor-body CAD visuals")
    return {
        "vehicle_profile": "current_m5",
        "mounts": mounts,
        "visuals": visuals,
    }


def source_fingerprints(root=ROOT):
    inputs = geometry_inputs(root)
    mesh_names = sorted(set(ANALYSIS_MESHES) | set(ANALYSIS_MESHES.values()))
    meshes = {
        str(PACKAGE / "meshes" / name): file_hash(
            root / PACKAGE / "meshes" / name
        )
        for name in mesh_names
    }
    return {
        "meshes_sha256": meshes,
        "geometry_inputs_sha256": fingerprint(inputs),
        "visual_transform_sha256": fingerprint(inputs["visuals"]),
    }


def load_stl(path):
    """Read bounded binary STL coordinates; units are supplied by the SDF."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Truncated binary STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if count == 0 or len(data) != 84 + 50 * count:
        raise ValueError(f"Invalid binary STL triangle count: {path}")
    dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]
    )
    triangles = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].astype(
        float
    )
    if not np.all(np.isfinite(triangles)):
        raise ValueError(f"Non-finite mesh coordinates: {path}")
    return triangles


def transform(triangles, pose, scale):
    """Apply SDF fixed-axis roll/pitch/yaw and translation in body axes."""
    r, p, y = pose[3:]
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    return (triangles * scale) @ rotation.T + pose[:3]


def bounds(triangles):
    low = triangles.min(axis=(0, 1))
    high = triangles.max(axis=(0, 1))
    return {
        "bounds_min_m": low.tolist(),
        "bounds_max_m": high.tolist(),
        "center_m": ((low + high) / 2).tolist(),
        "size_m": (high - low).tolist(),
    }


def mesh_quality(triangles):
    """Report topology after a 1um weld, without repairing the input."""
    vertices, inverse = np.unique(
        np.round(triangles.reshape(-1, 3), 6), axis=0, return_inverse=True
    )
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate(
        [faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]
    )
    edges.sort(axis=1)
    unique_edges, counts = np.unique(edges, axis=0, return_counts=True)
    parents = np.arange(len(vertices))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for first, second in unique_edges:
        first, second = find(first), find(second)
        if first != second:
            parents[second] = first
    components = len({find(index) for index in range(len(vertices))})
    vectors = (
        np.cross(
            triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
        )
        / 2
    )
    areas = np.linalg.norm(vectors, axis=1)
    return {
        "triangles": len(triangles),
        "weld_tolerance_m": 1e-6,
        "welded_vertices": len(vertices),
        "components_after_weld": components,
        "boundary_edges": int(np.count_nonzero(counts == 1)),
        "nonmanifold_edges": int(np.count_nonzero(counts > 2)),
        "degenerate_triangles_area_below_1e_12_m2": int(
            np.count_nonzero(areas < 1e-12)
        ),
        "surface_area_m2": float(areas.sum()),
        "closure_area_vector_m2": vectors.sum(axis=0).tolist(),
    }


def unit_mass_inertia(triangles):
    """Integrate tetrahedra for unit-mass centroidal inertia."""
    area_vectors = (
        np.cross(
            triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
        )
        / 2
    )
    area = np.linalg.norm(area_vectors, axis=1).sum()
    if np.linalg.norm(area_vectors.sum(axis=0)) > max(1e-12, area * 1e-9):
        raise ValueError("Oriented surface is not closed")
    origin = np.array(bounds(triangles)["center_m"])
    a, b, c = (triangles - origin).transpose(1, 0, 2)
    volumes = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6
    volume = volumes.sum()
    if not math.isfinite(volume) or volume <= 1e-12:
        raise ValueError("Mesh needs a positive oriented material volume")
    sums = a + b + c
    center = np.sum(volumes[:, None] * sums / 4, axis=0) / volume
    second = np.einsum("i,ij,ik->jk", volumes, sums, sums)
    for vertex in (a, b, c):
        second += np.einsum("i,ij,ik->jk", volumes, vertex, vertex)
    covariance = second / (20 * volume) - np.outer(center, center)
    inertia = np.eye(3) * np.trace(covariance) - covariance
    inertia = (inertia + inertia.T) / 2
    if (
        not np.all(np.isfinite(inertia))
        or np.min(np.linalg.eigvalsh(inertia)) <= 0
    ):
        raise ValueError(
            "Integrated inertia must be finite and positive definite"
        )
    return {
        "unit_mass_tensor_m2": inertia.tolist(),
        "uniform_solid_centroid_body_m": (center + origin).tolist(),
        "signed_material_volume_m3_NOT_DISPLACEMENT": float(volume),
    }


def raster_area(triangles, coordinates, pitch):
    """Union projected triangle scanlines at pixel centers; preserve holes."""
    if not math.isfinite(pitch) or pitch < 0.0001:
        raise ValueError("Raster pitch must be finite and at least 0.1mm")
    projected = triangles[:, :, coordinates]
    low = projected.min(axis=(0, 1))
    high = projected.max(axis=(0, 1))
    width, height = np.maximum(1, np.ceil((high - low) / pitch).astype(int))
    if width * height > 100_000_000:
        raise ValueError("Projection exceeds 100 million pixels")
    mask = np.zeros((height, width), dtype=bool)
    for triangle in projected:
        first = max(0, math.ceil((triangle[:, 1].min() - low[1]) / pitch - 0.5))
        last = min(
            height - 1,
            math.floor((triangle[:, 1].max() - low[1]) / pitch - 0.5),
        )
        if first > last:
            continue
        rows = np.arange(first, last + 1)
        y = low[1] + (rows + 0.5) * pitch
        left = np.full(len(rows), np.inf)
        right = np.full(len(rows), -np.inf)
        for start, end in zip(
            triangle, np.roll(triangle, -1, axis=0), strict=True
        ):
            if start[1] == end[1]:
                continue
            fraction = (y - start[1]) / (end[1] - start[1])
            valid = (fraction >= 0) & (fraction <= 1)
            x = start[0] + fraction * (end[0] - start[0])
            left = np.minimum(left, np.where(valid, x, np.inf))
            right = np.maximum(right, np.where(valid, x, -np.inf))
        valid = np.isfinite(left) & np.isfinite(right)
        starts = np.maximum(
            0, np.ceil((left[valid] - low[0]) / pitch - 0.5)
        ).astype(int)
        ends = np.minimum(
            width - 1, np.floor((right[valid] - low[0]) / pitch - 0.5)
        ).astype(int)
        for row, start, end in zip(rows[valid], starts, ends, strict=True):
            if start <= end:
                mask[row, start : end + 1] = True
    return float(np.count_nonzero(mask) * pitch**2)


def generate(root=ROOT, pitch=0.001, mass=22.6796, raster=True):
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("Reference mass must be finite and positive")
    inputs = geometry_inputs(root)
    names = sorted(set(ANALYSIS_MESHES) | set(ANALYSIS_MESHES.values()))
    meshes = {
        name: load_stl(root / PACKAGE / "meshes" / name) for name in names
    }
    rendered, solids = [], []
    for visual in inputs["visuals"]:
        for collection, key in (
            (rendered, "rendered_mesh"),
            (solids, "analysis_mesh"),
        ):
            collection.append(
                transform(
                    meshes[visual[key]], visual["pose_xyz_rpy"], visual["scale"]
                )
            )
    rendered = np.concatenate(rendered)
    solids = np.concatenate(solids)
    inertia = unit_mass_inertia(solids)
    inertia.update(
        {
            "reference": (
                "uniform_CAD_solid_centroid_recentered_at_assumed_body_origin"
            ),
            "assumed_runtime_cg_body_m": [0.0, 0.0, 0.0],
            "reference_mass_kg": mass,
            "reference_mass_tensor_kg_m2": (
                mass * np.array(inertia["unit_mass_tensor_m2"])
            ).tolist(),
        }
    )
    areas = {"raster_pitch_m": pitch, "method": "pixel_center_triangle_union"}
    for axis, coordinates in (("x", (1, 2)), ("y", (0, 2)), ("z", (0, 1))):
        areas[f"{axis}_m2"] = (
            raster_area(rendered, coordinates, pitch) if raster else None
        )
    return {
        "schema_version": 1,
        "vehicle_profile": "current_m5",
        "scope": (
            "base_link hull and six motor-body CAD visuals; "
            "cosmetic rotating propeller primitives excluded"
        ),
        "body_axes": (
            "x forward, y port, z up; metres; Rz(yaw) Ry(pitch) Rx(roll)"
        ),
        "methods": {
            "bounds": "min/max of transformed rendered STL vertices",
            "projection": (
                "orthographic union of rendered triangles sampled at pixel "
                "centers; holes and gaps remain open"
            ),
            "inertia": (
                "oriented tetrahedral volume moments of high-resolution CAD "
                "parts, normalized to unit total mass, about their centroid; "
                "tensor retains body-axis orientation"
            ),
            "fingerprints": (
                "mesh byte hashes and canonical geometry inputs are checked; "
                "whole-file source hashes record generation provenance only"
            ),
        },
        "sources": source_fingerprints(root),
        "generation_provenance_sha256": {
            str(path): file_hash(root / path)
            for path in (
                PACKAGE / "worlds/pool.sdf",
                Path("robosub_ws/src/robosub_control/robosub_control/config")
                / "current_m5.py",
                Path("tools/analyze_sim_geometry.py"),
            )
        },
        "assumptions": {
            "COG_NOT_MEASURED": True,
            "INERTIA_NOT_MEASURED": True,
            "MASS_NOT_INFERRED_FROM_MESH": True,
            "DISPLACEMENT_NOT_INFERRED_FROM_MESH": True,
            "VISUAL_TRANSFORM_ALIGNMENT_UNVERIFIED": True,
            "mass_distribution": (
                "uniform density over high-resolution CAD solids; "
                "overlapping parts count separately"
            ),
            "centroid": (
                "computed uniform-solid centroid is diagnostic only; use "
                "centroidal tensor at assumed runtime CG without a "
                "parallel-axis shift"
            ),
            "alignment": (
                "current mount translation shifts horizontal motor visuals "
                "outward 58mm per end relative to original CAD; rendered "
                "union is preserved, not repaired"
            ),
            "collision": (
                "bounds are a contact envelope, not buoyancy volume "
                "or sealed hull volume"
            ),
            "quality": (
                "multipart/nonmanifold CAD is not a certified closed exterior "
                "shell; lowpoly hull contains boundary edges"
            ),
            "reference_mass": (
                "22.6796kg is the requested approximate total-mass reference, "
                "not a CAD measurement; account for separate propeller-link "
                "masses at runtime"
            ),
        },
        "visuals": inputs["visuals"],
        "collision": bounds(rendered),
        "projected_area": areas,
        "inertia": inertia,
        "quality": {
            name: mesh_quality(triangles) for name, triangles in meshes.items()
        },
    }


def check_snapshot(snapshot, root=ROOT):
    """Check current source identity without invoking ROS or mesh analysis."""
    if snapshot.get("schema_version") != 1:
        raise ValueError("Unsupported geometry snapshot schema")
    if snapshot.get("vehicle_profile") != "current_m5":
        raise ValueError("Geometry snapshot is not for current_m5")
    expected = source_fingerprints(root)
    if snapshot.get("sources") != expected:
        raise ValueError("Geometry snapshot is stale; regenerate offline")
    if (
        fingerprint(snapshot.get("visuals"))
        != expected["visual_transform_sha256"]
    ):
        raise ValueError(
            "Snapshot visual transforms do not match their fingerprint"
        )
    tensor = np.asarray(snapshot["inertia"]["unit_mass_tensor_m2"], dtype=float)
    if tensor.shape != (3, 3) or not np.all(np.isfinite(tensor)):
        raise ValueError("Invalid unit-mass inertia tensor")
    if (
        not np.allclose(tensor, tensor.T)
        or np.min(np.linalg.eigvalsh(tensor)) <= 0
    ):
        raise ValueError(
            "Unit-mass inertia must be symmetric positive definite"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / SNAPSHOT)
    parser.add_argument("--raster-pitch", type=float, default=0.001)
    parser.add_argument("--mass", type=float, default=22.6796)
    parser.add_argument("--skip-raster", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.check:
        check_snapshot(json.loads(args.output.read_text()))
        print(
            f"Geometry mesh and transform fingerprints verified: {args.output}"
        )
        return
    result = generate(
        pitch=args.raster_pitch, mass=args.mass, raster=not args.skip_raster
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Generated offline geometry snapshot: {args.output}")


if __name__ == "__main__":
    main()
