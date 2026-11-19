"""Optical coordinates: x right, y down, z forward; lengths in metres."""

import numpy as np
from scipy.spatial.transform import Rotation


def unit(v):
    a = np.asarray(v, dtype=float)
    norm = np.linalg.norm(a, axis=-1, keepdims=True)
    return a / np.where(norm > 1e-12, norm, np.nan)


def rigid_fit(source, destination):
    """Least-squares T_destination_source, without reflection or scaling."""
    a, b = np.asarray(source), np.asarray(destination)
    if len(a) < 3 or np.linalg.matrix_rank(a - a.mean(0)) < 2:
        raise ValueError("Rigid fit needs three non-collinear correspondences")
    u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ correction @ u.T
    return r, b.mean(0) - r @ a.mean(0)


def transform_points(points, r, t):
    return np.asarray(points) @ np.asarray(r).T + np.asarray(t)


def covariance(sigma, rotation_sigma=None):
    result = np.zeros((6, 6))
    result[:3, :3] = np.eye(3) * sigma**2
    result[3:, 3:] = np.diag(
        [np.inf] * 3 if rotation_sigma is None else [rotation_sigma**2] * 3
    )
    return result


def rotate_covariance(cov, r, orientation_valid):
    result = np.zeros((6, 6))
    result[:3, :3] = r @ cov[:3, :3] @ r.T
    if orientation_valid:
        block = np.zeros((6, 6))
        block[:3, :3] = block[3:, 3:] = r
        return block @ cov @ block.T
    result[3:, 3:] = np.diag([np.inf] * 3)
    return result


def pose_covariance(jacobian, residual, sigma, min_extent=0.05):
    """Local fit uncertainty with a floor for correlated depth/model error."""
    j = np.asarray(jacobian)
    singular = np.linalg.svd(j, compute_uv=False)
    if singular[-1] < singular[0] * 1e-7:
        raise ValueError("Pose is geometrically unobservable")
    variance = max(float(np.mean(np.square(residual))), 1e-10)
    cov = np.linalg.inv(j.T @ j) * variance
    cov[:3, :3] += np.eye(3) * sigma**2
    cov[3:, 3:] += np.eye(3) * (sigma / min_extent) ** 2
    return cov


def canonical_rotation(rotation, symmetry, prior=None):
    if not symmetry:
        return rotation
    if "rotations_xyzw" in symmetry:
        values = np.asarray(symmetry["rotations_xyzw"], dtype=float)
        group = [np.eye(3), *Rotation.from_quat(values).as_matrix()]
    else:
        axis = unit(symmetry.get("axis", [0, 0, 0]))
        order = symmetry.get("order")
        group = [
            Rotation.from_rotvec(axis * (2 * np.pi * i / order)).as_matrix()
            for i in range(order)
        ]
    candidates = [rotation @ element for element in group]
    reference = np.eye(3) if prior is None else prior
    return min(
        candidates,
        key=lambda r: Rotation.from_matrix(reference.T @ r).magnitude(),
    )
