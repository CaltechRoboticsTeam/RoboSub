"""Meaningful numerical checks; all geometry here is synthetic, not hardware."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation

SOURCE = Path(__file__).resolve().parents[2] / "robosub_ws/src/robosub_vision"
sys.path.insert(0, str(SOURCE))

from robosub_vision.calibration import (  # noqa: E402
    TagGrid,
    read_opencv,
    solve_stereo,
    write_opencv,
)
from robosub_vision.config import load_config  # noqa: E402
from robosub_vision.geometry import (  # noqa: E402
    covariance,
    rotate_covariance,
    transform_points,
)
from robosub_vision.pipeline import Pipeline  # noqa: E402
from robosub_vision.rectification import fit_rectification  # noqa: E402
from robosub_vision.refraction import (  # noqa: E402
    PortCamera,
    RefractiveStereo,
    triangulate,
)
from robosub_vision.runtime import (  # noqa: E402
    flatten,
    wrapper_parameters,
)
from robosub_vision.solvers import (  # noqa: E402
    CadSolver,
    cylinder,
    panel_landmarks,
    planar,
)
from robosub_vision.synthetic import (  # noqa: E402
    generate_offline,
    load_scene,
)
from robosub_vision.types import Detection  # noqa: E402

K = np.array([[300.0, 0, 160], [0, 300.0, 120], [0, 0, 1]])


def flat_camera():
    return {
        "K": K.tolist(),
        "D": [0] * 5,
        "port_model": "FLATPORT",
        "port_parameters": [0, 0, 1, 0.025, 0.008, 1.0, 1.5, 1.333],
    }


def flat_project(point):
    """Independent scalar Snell calculation for a front-parallel test port."""
    point = np.asarray(point)
    radial = np.linalg.norm(point[:2])

    def lateral(theta):
        glass = np.arcsin(np.sin(theta) / 1.5)
        water = np.arcsin(np.sin(theta) / 1.333)
        return (
            0.025 * np.tan(theta)
            + 0.008 * np.tan(glass)
            + (point[2] - 0.033) * np.tan(water)
        )

    theta = brentq(lambda value: lateral(value) - radial, 0, 1.3)
    normalized = point[:2] / max(radial, 1e-12) * np.tan(theta)
    return normalized * [K[0, 0], K[1, 1]] + K[:2, 2]


def test_apriltag_grid_and_effective_calibration_export(tmp_path):
    grid = TagGrid(SOURCE / "config/board.yaml", require_measured=False)
    image = grid.board.generateImage((840, 580), marginSize=40)
    detected = grid.detect(image)
    assert set(detected) == set(range(24))
    obj = np.concatenate([grid.objects[i] for i in sorted(detected)])
    pixels = np.concatenate([detected[i] for i in sorted(detected)])
    homography, _ = cv2.findHomography(obj[:, :2], pixels)
    predicted = cv2.perspectiveTransform(obj[:, None, :2], homography)[:, 0]
    assert np.max(np.linalg.norm(predicted - pixels, axis=1)) < 1.0
    with pytest.raises(ValueError, match="Measure board"):
        TagGrid(SOURCE / "config/board.yaml")
    rng = np.random.default_rng(22)
    factory = {
        "image_size": [640, 480],
        "left": {
            "K": (K * np.array([[2, 1, 2], [1, 2, 2], [1, 1, 1]])).tolist(),
            "D": [0] * 5,
        },
        "right": {
            "K": [[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]],
            "D": [0] * 5,
        },
        "physical_R": Rotation.from_rotvec([0.002, -0.003, 0.001])
        .as_matrix()
        .tolist(),
        "physical_t": [-0.12, 0.0002, 0.0001],
    }
    true_k = np.array([[790.0, 0, 321], [0, 795.0, 239], [0, 0, 1]])
    d = np.array([-0.02, 0.004, 0.0005, -0.0003, 0.0])
    r, t = np.array(factory["physical_R"]), np.array(factory["physical_t"])
    observations = []
    for _ in range(14):
        rv = rng.uniform([-0.4, -0.4, -0.2], [0.4, 0.4, 0.2])
        tv = rng.uniform([-0.22, -0.16, 0.65], [-0.05, -0.02, 1.3])
        left = cv2.projectPoints(obj, rv, tv, true_k, d)[0][:, 0]
        right = cv2.projectPoints(
            obj,
            cv2.Rodrigues(r @ cv2.Rodrigues(rv)[0])[0],
            r @ tv + t,
            true_k,
            d,
        )[0][:, 0]
        observations.append(
            (
                obj,
                left + rng.normal(0, 0.02, left.shape),
                right + rng.normal(0, 0.02, right.shape),
            )
        )
    k1, d1, k2, d2, fitted_r, fitted_t, errors = solve_stereo(
        observations, factory
    )
    np.testing.assert_array_equal(fitted_r, r)
    np.testing.assert_array_equal(fitted_t, t)
    np.testing.assert_allclose([k1[0, 0], k2[1, 1]], [790, 795], rtol=0.01)
    assert max(errors) < 0.1
    path = tmp_path / "underwater.yml"
    write_opencv(path, [640, 480], k1, d1, k2, d2, r, t)
    exported = read_opencv(path)
    assert exported["size"] == (640, 480)
    np.testing.assert_allclose(exported["T"].ravel(), t * 1000)
    np.testing.assert_allclose(cv2.Rodrigues(exported["R"])[0], r)


def test_actual_rectification_with_nonidentity_rotation():
    xx, yy = np.meshgrid(np.linspace(-0.4, 0.4, 16), np.linspace(-0.3, 0.3, 12))
    points = np.c_[xx.ravel(), yy.ravel(), np.ones(xx.size)]
    r = Rotation.from_euler("xyz", [1, -2, 0.5], degrees=True).as_matrix()
    d = np.array([-0.04, 0.01, 0.001, -0.002, 0.0])
    raw = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, d)[0][:, 0]
    rotated = points @ r.T
    rect = rotated @ K.T
    rect = rect[:, :2] / rect[:, 2:]
    maps, fitted, rms = fit_rectification(raw, rect, K, d, K, (320, 240))
    np.testing.assert_allclose(fitted, r, atol=1e-6)
    assert rms < 1e-4
    mapped = RefractiveStereo.sample(maps, rect)
    assert np.max(np.linalg.norm(mapped - raw, axis=1)) < 0.05


def test_flat_refraction_metric_stereo_and_ray_rejection():
    camera = PortCamera(flat_camera())
    truth = np.array([[0.12, 0.07, 0.6], [-0.15, 0.04, 1.2], [0.2, -0.1, 2.0]])
    pixels_left = np.array([flat_project(p) for p in truth])
    pixels_right = np.array([flat_project(p - [0.12, 0, 0]) for p in truth])
    o1, d1 = camera.rays(pixels_left)
    o2, d2 = camera.rays(pixels_right)
    result, valid, gap = triangulate(o1, d1, o2 + [0.12, 0, 0], d2)
    assert valid.all()
    np.testing.assert_allclose(result, truth, atol=1e-7)
    assert max(gap) < 1e-8
    _, valid, _ = triangulate(o1, d1, o1, d1)
    assert not valid.any()
    _, valid, _ = triangulate(o1, d1, o2 + [0.12, 0.1, 0], d2, max_gap=0.001)
    assert not valid.any()
    _, valid, _ = triangulate(o1, -d1, o2 + [0.12, 0, 0], -d2)
    assert not valid.any()
    dome = flat_camera()
    dome["port_model"] = "DOMEPORT"
    dome["port_parameters"][:5] = [0, 0, 0, 0.08, 0.01]
    origins, directions = PortCamera(dome).rays(pixels_left)
    incident = np.c_[pixels_left, np.ones(3)] @ np.linalg.inv(K).T
    incident /= np.linalg.norm(incident, axis=1)[:, None]
    np.testing.assert_allclose(directions, incident, atol=1e-8)
    np.testing.assert_allclose(np.linalg.norm(origins, axis=1), 0.09, atol=1e-8)


def test_roi_ray_lookup_matches_independent_geometry(tmp_path):
    data = {
        "schema": 1,
        "units": "meters",
        "image_size": [320, 240],
        "left": flat_camera(),
        "right": flat_camera(),
        "T_left_right": np.eye(4).tolist(),
    }
    data["T_left_right"][0][3] = 0.12
    housing = tmp_path / "housing.yaml"
    housing.write_text(yaml.safe_dump(data))
    xx, yy = np.meshgrid(np.arange(320), np.arange(240))
    mapping = np.dstack([xx, yy]).astype(np.float32)
    np.savez(
        tmp_path / "rect.npz",
        validated=True,
        raw_size=[320, 240],
        left_map=mapping,
        right_map=mapping,
        R_left=np.eye(3),
        K_left_rect=K,
        left_info=[1],
        right_info=[1],
        sdk_sha256="test",
    )
    stereo = RefractiveStereo(housing, tmp_path / "rect.npz", "test")
    truth = np.array([[0.11, 0.06, 0.8], [-0.1, 0.02, 1.8]])
    left = np.array([flat_project(p) for p in truth])
    right = np.array([flat_project(p - [0.12, 0, 0]) for p in truth])
    # Under a refractive camera the corresponding rows need not be equal.
    # Select points with y=0 for a strictly horizontal synthetic matcher.
    truth[:, 1] = 0
    left = np.array([flat_project(p) for p in truth])
    right = np.array([flat_project(p - [0.12, 0, 0]) for p in truth])
    reconstructed, valid, _ = stereo.reconstruct(left, left[:, 0] - right[:, 0])
    assert valid.all()
    np.testing.assert_allclose(reconstructed, truth, atol=0.002)
    with pytest.raises(ValueError, match="different SDK"):
        RefractiveStereo(housing, tmp_path / "rect.npz", "changed")


def test_planar_and_refractive_plane_refinement():
    target = {"width": 0.4, "height": 0.3}
    obj = panel_landmarks(target)
    rv, t = np.array([0.2, -0.3, 0.04]), np.array([0.03, -0.02, 1.2])
    r = cv2.Rodrigues(rv)[0]
    xx, yy = np.meshgrid(
        np.linspace(-0.18, 0.18, 18), np.linspace(-0.13, 0.13, 13)
    )
    points = transform_points(
        np.c_[xx.ravel(), yy.ravel(), np.zeros(xx.size)], r, t
    )
    image = cv2.projectPoints(obj, rv, t, K, None)[0][:, 0]
    estimate = planar(target, image, points, K, 0.003)
    np.testing.assert_allclose(estimate.position, t, atol=0.003)
    assert Rotation.from_matrix(estimate.rotation.T @ r).magnitude() < 0.01
    raw = np.array([flat_project(p) for p in transform_points(obj, r, t)])

    class PhysicalRays:
        def left_rays_output(self, pixels):
            return PortCamera(flat_camera()).rays(pixels)

    estimate = planar(target, raw, points, K, 0.003, PhysicalRays())
    np.testing.assert_allclose(estimate.position, t, atol=0.005)
    assert Rotation.from_matrix(estimate.rotation.T @ r).magnitude() < 0.03
    with pytest.raises(ValueError, match="keypoints"):
        planar(target, None, points, K, 0.003)


def test_cylinder_observability_and_position_reference():
    theta, height = np.meshgrid(
        np.linspace(-1.0, 1.0, 30), np.linspace(-0.2, 0.2, 20)
    )
    radius = 0.025
    local = np.c_[
        radius * np.sin(theta.ravel()),
        height.ravel(),
        -radius * np.cos(theta.ravel()),
    ]
    r = Rotation.from_euler("xyz", [0.15, -0.1, 0.2]).as_matrix()
    center = np.array([0.1, 0.02, 1.3])
    points = transform_points(local, r, center)
    for target in ({"radius": radius}, {}):
        estimate = cylinder(target, points, 0.003)
        assert estimate.rotation is None
        assert abs(estimate.axis @ r[:, 1]) > 0.999
        assert abs(estimate.radius - radius) < 0.001
        np.testing.assert_allclose(estimate.position, center, atol=0.003)
    unknown = covariance(0.02)
    moved = rotate_covariance(unknown, r, False)
    assert not np.isnan(moved).any()
    assert np.isinf(moved[3:, 3:].diagonal()).all()


def test_visible_box_cad_registration_and_unobservable_plane():
    target = {
        "mesh": str(SOURCE / "models/example_box.stl"),
        "units": "meters",
        "iterations": 35,
        "max_rmse": 0.02,
        "symmetry": {
            "rotations_xyzw": [
                [0, 0, 0, 1],
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
            ]
        },
    }
    solver = CadSolver(target)
    r = Rotation.from_euler("xyz", [0.35, -0.5, 0.2]).as_matrix()
    t = np.array([0.04, -0.02, 1.2])
    points = transform_points(solver.points, r, t)
    visible = np.sum((solver.normals @ r.T) * points, axis=1) < 0
    points = points[visible]
    estimate = solver.solve(points, 0.003)
    np.testing.assert_allclose(estimate.position, t, atol=0.008)
    assert (
        estimate.rotation is not None and np.isfinite(estimate.covariance).all()
    )
    assert estimate.rmse < 0.015
    face = points[np.abs((points - t) @ r[:, 2] + 0.075) < 1e-5]
    with pytest.raises(ValueError, match="registration"):
        solver.solve(face, 0.003, prior=(r, t))


def test_config_selection_foreground_and_pipeline(tmp_path):
    config = load_config(SOURCE / "config/development.yaml")
    config["targets"] = [
        {
            "name": "sample",
            "detector_class": "sample",
            "method": "position",
            "min_confidence": 0.6,
        }
    ]
    depth = np.full((240, 320), 2.5, np.float32)
    depth[80:160, 110:210] = 1.2
    depth[90:100, 130:140] = np.nan
    detection = Detection("sample", 0.9, np.array([100, 70, 220, 170]))
    pipeline = Pipeline(config)
    results, errors = pipeline.process(
        [detection], depth, None, np.zeros_like(depth), K, 1.0
    )
    assert not errors and len(results) == 1
    estimate = results[0][1]
    assert (
        estimate.rotation is None
        and estimate.reference_point == "median_visible_surface"
    )
    assert abs(estimate.position[2] - 1.2) < 0.001
    results, errors = pipeline.process(
        [detection], depth, None, np.full_like(depth, 100), K, 2.0
    )
    assert not results and errors
    source = {
        "underwater": {"depth_source": "zed"},
        "targets": [
            {
                "name": "off",
                "enabled": False,
                "method": "cad",
                "mesh": "does-not-exist.stl",
            }
        ],
    }
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(source))
    assert load_config(path)["targets"] == []


def test_synthetic_export_labels_and_metric_ground_truth(tmp_path):
    scene = load_scene(SOURCE / "config/synthetic.yaml")
    scene["image_size"] = [160, 120]
    scene["focal_px"] = 140
    destination = tmp_path / "dataset"
    generate_offline(scene, destination, 15, 135)
    metadata = json.loads((destination / "metadata.json").read_text())
    assert metadata["optics"] == "pinhole_no_refraction"
    images = list((destination / "images").rglob("*.png"))
    assert len(images) == 15
    assert len(list((destination / "images/val").glob("*.png"))) == 3
    for label in (destination / "labels").rglob("*.txt"):
        values = np.fromstring(label.read_text(), sep=" ")
        if not len(values):
            continue
        assert len(values) == 5 + 4 * 3
        assert 0 <= values[0] <= 2
        assert np.all((values[1:5] >= 0) & (values[1:5] <= 1))
        pose_path = (
            destination / "poses" / label.parent.name / f"{label.stem}.json"
        )
        pose = np.array(json.loads(pose_path.read_text())["T_camera_object"])
        assert 0.5 <= pose[2, 3] <= 2
        np.testing.assert_allclose(
            pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-8
        )


def test_pinned_wrapper_contract_if_sources_available():
    # Skip the external contract check unless its source checkout exists.
    import os

    path = os.environ.get("ZED_WRAPPER_SOURCE")
    if not path:
        pytest.skip(
            "Set ZED_WRAPPER_SOURCE to check the pinned upstream source"
        )
    config = load_config(SOURCE / "config/development.yaml")
    known = {}
    for filename in (
        "common_stereo.yaml",
        "zed2i.yaml",
        "object_detection.yaml",
    ):
        file = Path(path) / "zed_wrapper/config" / filename
        data = yaml.safe_load(file.read_text())
        known.update(flatten(data.get("/**", {}).get("ros__parameters", {})))
    assert not set(wrapper_parameters(config, "5.5.0")) - set(known)
    assert "pos_tracking.compute_preference" not in wrapper_parameters(
        config, "5.4.0"
    )
    source = (
        Path(path)
        / "zed_components/src/zed_camera/src"
        / "zed_camera_component_video_depth.cpp"
    )
    text = source.read_text()
    for topic in (
        "disparity/map",
        "confidence/confidence_map",
        "depth/depth_registered",
    ):
        assert topic in text
