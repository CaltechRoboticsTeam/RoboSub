"""Estimate target position and pose."""

import json
from itertools import permutations, product

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from robosub_vision.geometry import (
    canonical_rotation,
    covariance,
    pose_covariance,
    rigid_fit,
    transform_points,
    unit,
)
from robosub_vision.types import Estimate


def position(points, sigma):
    center = np.median(points, axis=0)
    residual = points - center
    # Points on a visible surface are correlated; do not divide by sample count.
    cov = covariance(sigma)
    cov[:3, :3] += np.cov(points.T)
    return Estimate(
        center,
        cov,
        "median_visible_surface",
        float(np.sqrt(np.mean(residual**2))),
        len(points),
    )


def panel_landmarks(target):
    w, h = target["width"] / 2, target["height"] / 2
    return np.array([[-w, -h, 0], [w, -h, 0], [w, h, 0], [-w, h, 0]])


def planar(target, keypoints, points, k, sigma, stereo=None):
    if keypoints is None:
        raise ValueError("Planar pose requires ordered detector keypoints")
    indices = target.get("keypoint_indices", [0, 1, 2, 3])
    if max(indices) >= len(keypoints):
        raise ValueError("Detector did not supply the configured panel corners")
    image = np.asarray(keypoints, dtype=float)[indices]
    if not np.isfinite(image).all():
        raise ValueError("Panel corners are occluded or unreliable")
    obj = panel_landmarks(target)
    found, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, image, k, None, flags=cv2.SOLVEPNP_IPPE
    )
    if not found:
        raise ValueError("Planar IPPE initialization failed")
    if stereo:
        origins, directions = stereo.left_rays_output(image)
    else:
        origins = np.zeros((4, 3))
        directions = unit(np.c_[image, np.ones(4)] @ np.linalg.inv(k).T)
    if not np.isfinite(directions).all():
        raise ValueError("Panel corners lie outside calibrated ray maps")
    solutions = []

    def residual(parameters):
        t, rv = parameters[:3], parameters[3:]
        r = Rotation.from_rotvec(rv).as_matrix()
        corners = transform_points(obj, r, t)
        offset = corners - origins
        # Point-to-refracted-ray error, in metres, plus stereo plane error.
        ray_error = offset - np.sum(offset * directions, axis=1)[:, None] * (
            directions
        )
        local = (points - t) @ r
        edge_x = np.maximum(np.abs(local[:, 0]) - target["width"] / 2, 0)
        edge_y = np.maximum(np.abs(local[:, 1]) - target["height"] / 2, 0)
        return np.r_[ray_error.ravel(), local[:, 2], edge_x, edge_y]

    for rv, tv in zip(rvecs, tvecs, strict=True):
        fit = least_squares(
            residual,
            np.r_[tv.ravel(), rv.ravel()],
            loss="soft_l1",
            f_scale=sigma,
            max_nfev=100,
        )
        r = Rotation.from_rotvec(fit.x[3:]).as_matrix()
        corners = transform_points(obj, r, fit.x[:3])
        if np.any(corners[:, 2] <= 0) or not fit.success:
            continue
        error = float(np.sqrt(np.mean(residual(fit.x) ** 2)))
        solutions.append((error, fit, r))
    if not solutions:
        raise ValueError("No valid forward planar pose")
    solutions.sort(key=lambda item: item[0])
    error, fit, r = solutions[0]
    if error > target.get("max_rmse", 0.025):
        raise ValueError("Panel geometry is inconsistent with measured stereo")
    if len(solutions) > 1:
        other = solutions[1]
        angle = Rotation.from_matrix(other[2].T @ r).magnitude()
        if angle > 0.15 and other[0] < error + sigma * 0.05:
            raise ValueError("Planar two-pose ambiguity remains unresolved")
    extent = max(min(target["width"], target["height"]), 0.01)
    cov = pose_covariance(fit.jac, fit.fun, sigma, extent)
    return Estimate(fit.x[:3], cov, "panel_center", error, len(points), r)


def cylinder(target, points, sigma):
    mean = np.mean(points, axis=0)
    _, _, vt = np.linalg.svd(points - mean, full_matrices=False)
    known = target.get("radius")
    solutions = []
    # Trying each principal axis avoids assuming the object's direction.
    for initial in vt:
        basis = np.linalg.svd(initial.reshape(1, 3), full_matrices=True)[2][1:]

        def unpack(x, basis=basis, initial=initial):
            center = mean + x[:2] @ basis
            axis = unit(initial + x[2:4] @ basis)
            radius = known if known is not None else x[4]
            return center, axis, radius

        def residual(x, unpack=unpack):
            center, axis, radius = unpack(x)
            delta = points - center
            radial = delta - (delta @ axis)[:, None] * axis
            return np.linalg.norm(radial, axis=1) - radius

        initial_radius = max(
            np.median(np.linalg.norm((points - mean) @ basis.T, axis=1)), 0.005
        )
        x = [0, 0, 0, 0] + ([] if known is not None else [initial_radius])
        bound = [np.inf] * 4 + ([] if known is not None else [1.0])
        lower = [-np.inf] * 4 + ([] if known is not None else [0.001])
        fit = least_squares(
            residual,
            x,
            bounds=(lower, bound),
            loss="soft_l1",
            f_scale=sigma / 2,
            max_nfev=200,
        )
        center, axis, radius = unpack(fit.x)
        delta = points - center
        radial = delta - (delta @ axis)[:, None] * axis
        angles = unit(radial)
        # Circular coverage indicates whether the radius is observable.
        b = np.linalg.svd(axis.reshape(1, 3), full_matrices=True)[2][1:]
        theta = np.sort(np.arctan2(angles @ b[1], angles @ b[0]))
        arc = 2 * np.pi - np.max(np.diff(np.r_[theta, theta[0] + 2 * np.pi]))
        length = np.ptp(delta @ axis)
        rmse = float(np.sqrt(np.mean(fit.fun**2)))
        singular = np.linalg.svd(fit.jac, compute_uv=False)
        condition = singular[0] / max(singular[-1], 1e-12)
        if (
            fit.success
            and arc >= np.deg2rad(target.get("min_arc_deg", 45))
            and length > 2 * radius
            and condition < 1e5
        ):
            axial = delta @ axis
            center = center + (np.min(axial) + np.max(axial)) / 2 * axis
            solutions.append((rmse, center, axis, radius))
    if not solutions:
        raise ValueError("Cylinder axis/radius insufficiently observable")
    rmse, center, axis, radius = min(solutions, key=lambda item: item[0])
    if rmse > target.get("max_rmse", 0.02):
        raise ValueError("Cylinder fit residual too large")
    axis *= 1 if axis[np.argmax(np.abs(axis))] >= 0 else -1
    return Estimate(
        center,
        covariance(max(sigma, rmse)),
        "axis_midpoint_of_observed_extent",
        rmse,
        len(points),
        axis=axis,
        radius=float(radius),
        symmetry="continuous_about_axis; axial_extent_partial",
    )


class CadSolver:
    """Fit a CAD model to visible surface points using ICP."""

    def __init__(self, target, samples=2000):
        import trimesh  # Only enabled CAD targets load the mesh library.

        self.target = target
        mesh = trimesh.load(target["mesh"], force="mesh", process=True)
        if target["units"] == "millimeters":
            mesh.apply_scale(0.001)
        if mesh.is_empty or not np.isfinite(mesh.vertices).all():
            raise ValueError(f"Invalid mesh: {target['mesh']}")
        self.points, face = trimesh.sample.sample_surface(mesh, samples, seed=0)
        self.normals = np.asarray(mesh.face_normals)[face]
        self.extent = float(np.max(mesh.extents))

    def _refine(self, points, r, t, iterations, sigma):
        target = self.target
        distance_limit = target.get("max_distance", 0.08)
        for _ in range(iterations):
            model = transform_points(self.points, r, t)
            normals = self.normals @ r.T
            visible = np.sum(normals * model, axis=1) < 0
            if visible.sum() < 20:
                raise ValueError("CAD has insufficient visible surface")
            model, normals = model[visible], normals[visible]
            distances, idx = cKDTree(model).query(points)
            good = distances < min(distance_limit, np.quantile(distances, 0.85))
            if good.sum() < 20:
                raise ValueError("CAD has insufficient stereo overlap")
            observed, matched, n = (
                points[good],
                model[idx[good]],
                normals[idx[good]],
            )
            residual = np.sum(n * (matched - observed), axis=1)
            # Apply rotation increments about the object origin.
            j = np.c_[n, np.cross(matched - t, n)]
            weights = np.sqrt(
                np.minimum(1, sigma / np.maximum(np.abs(residual), 1e-9))
            )
            step, _, _, singular = np.linalg.lstsq(
                j * weights[:, None], -residual * weights, rcond=1e-7
            )
            if singular[-1] < singular[0] * 1e-5:
                raise ValueError("Visible CAD surfaces do not constrain 6-DoF")
            if np.linalg.norm(step[3:]) > 0.5:
                step *= 0.5 / np.linalg.norm(step[3:])
            t = t + step[:3]
            r = Rotation.from_rotvec(step[3:]).as_matrix() @ r
            if np.linalg.norm(step) < 1e-5:
                break
        model = transform_points(self.points, r, t)
        normals = self.normals @ r.T
        visible = np.sum(normals * model, axis=1) < 0
        distances, idx = cKDTree(model[visible]).query(points)
        good = distances < distance_limit
        fitness = good.mean()
        if fitness < target.get("min_fitness", 0.6):
            raise ValueError("CAD alignment has insufficient overlap")
        matched, n = model[visible][idx[good]], normals[visible][idx[good]]
        residual = np.sum(n * (matched - points[good]), axis=1)
        j = np.c_[n, np.cross(matched - t, n)]
        rmse = float(np.sqrt(np.mean(distances[good] ** 2)))
        return r, t, rmse, j, residual

    def solve(self, points, sigma, prior=None, landmarks=None):
        if landmarks is not None:
            object_points, observed = landmarks
            r, t = rigid_fit(object_points, observed)
            starts = [(r, t)]
        elif prior is not None:
            starts = [prior]
        else:
            _, _, vt = np.linalg.svd(points - np.mean(points, 0))
            starts = []
            for perm in permutations(range(3)):
                for signs in product((-1, 1), repeat=3):
                    r = vt.T[:, perm] @ np.diag(signs)
                    if np.linalg.det(r) < 0:
                        continue
                    model = self.points @ r.T
                    n = self.normals @ r.T
                    visible = np.sum(n * unit(np.mean(points, 0)), axis=1) < 0
                    t = np.median(points, 0) - np.mean(model[visible], 0)
                    starts.append((r, t))
        candidates = []
        refine = self.target.get("refine", True)
        if not refine:
            if landmarks is None:
                raise ValueError(
                    "CAD without refinement needs metric landmarks"
                )
            r, t = starts[0]
            residual = transform_points(landmarks[0], r, t) - landmarks[1]
            error = float(np.sqrt(np.mean(residual**2)))
            if error > self.target.get("max_rmse", 0.025):
                raise ValueError("CAD landmark fit residual too large")
            return Estimate(
                t,
                covariance(sigma, sigma / self.extent),
                "cad_origin",
                error,
                len(landmarks[0]),
                r,
            )
        for r, t in starts:
            try:
                candidates.append(
                    self._refine(
                        points, r, t, self.target.get("iterations", 25), sigma
                    )
                )
            except ValueError:
                continue
        if not candidates:
            raise ValueError("CAD registration failed or pose is unobservable")
        r, t, error, j, residual = min(candidates, key=lambda item: item[2])
        if error > self.target.get("max_rmse", 0.025):
            raise ValueError("CAD registration residual too large")
        r = canonical_rotation(
            r, self.target.get("symmetry"), prior[0] if prior else None
        )
        for candidate_r, _, candidate_error, _, _ in candidates:
            if candidate_error > error + sigma * 0.1:
                continue
            equivalent = canonical_rotation(
                candidate_r, self.target.get("symmetry"), r
            )
            if Rotation.from_matrix(r.T @ equivalent).magnitude() > 0.2:
                raise ValueError(
                    "CAD pose remains ambiguous; supply landmarks "
                    "or define the object's rotational symmetries"
                )
        cov = pose_covariance(j, residual, sigma, self.extent)
        return Estimate(
            t,
            cov,
            "cad_origin",
            error,
            len(points),
            r,
            symmetry=json.dumps(self.target.get("symmetry", {})),
        )
