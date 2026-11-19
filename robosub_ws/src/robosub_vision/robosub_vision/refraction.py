"""Trace rays through flat or dome ports and triangulate stereo points."""

from pathlib import Path

import cv2
import numpy as np
import yaml

from robosub_vision.geometry import unit


def refract(direction, normal, index_in, index_out):
    n = np.broadcast_to(normal, direction.shape)
    cosine = np.sum(direction * n, axis=-1, keepdims=True)
    tangent = (index_in / index_out) * (direction - cosine * n)
    square = 1 - np.sum(tangent * tangent, axis=-1, keepdims=True)
    result = tangent + np.sqrt(np.maximum(square, 0)) * n
    result[(square[:, 0] < 0) | (cosine[:, 0] <= 0)] = np.nan
    return unit(result)


class PortCamera:
    """Raw image pixel -> ray origin and unit direction in that camera."""

    def __init__(self, data):
        self.k = np.asarray(data["K"], dtype=float).reshape(3, 3)
        self.d = np.asarray(data["D"], dtype=float)
        self.model = data["port_model"]
        self.p = np.asarray(data["port_parameters"], dtype=float)

    def rays(self, pixels):
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        xy = cv2.undistortPoints(pixels[:, None, :], self.k, self.d)
        direction = unit(np.c_[xy[:, 0, :], np.ones(len(xy))])
        origin = np.zeros_like(direction)
        shape = self.p[:3]
        distance, thickness, air, glass, water = self.p[3:]
        for surface, n_in, n_out in (
            (distance, air, glass),
            (distance + thickness, glass, water),
        ):
            if self.model == "FLATPORT":
                denominator = direction @ shape
                travel = (surface - origin @ shape) / np.where(
                    denominator > 1e-9, denominator, np.nan
                )
                normal = np.broadcast_to(shape, direction.shape)
            else:
                offset = origin - shape
                b = np.sum(direction * offset, axis=1)
                discriminant = (
                    b * b - np.sum(offset * offset, axis=1) + surface**2
                )
                travel = -b + np.sqrt(
                    np.where(discriminant >= 0, discriminant, np.nan)
                )
                normal = None
            origin = origin + travel[:, None] * direction
            origin[travel < -1e-9] = np.nan
            if normal is None:
                normal = unit(origin - shape)
            direction = refract(direction, normal, n_in, n_out)
        return origin, direction


def triangulate(o1, d1, o2, d2, max_gap=0.01, min_angle_deg=0.5):
    """Intersect forward water rays, rejecting parallel/skew/backward rays."""
    w = o1 - o2
    dot = np.sum(d1 * d2, axis=1)
    denominator = 1 - dot**2
    safe = np.where(denominator > 1e-12, denominator, np.nan)
    a = np.sum(d1 * w, axis=1)
    b = np.sum(d2 * w, axis=1)
    s = (dot * b - a) / safe
    t = (b - dot * a) / safe
    p1, p2 = o1 + s[:, None] * d1, o2 + t[:, None] * d2
    gap = np.linalg.norm(p1 - p2, axis=1)
    valid = (
        (s > 0)
        & (t > 0)
        & (gap <= max_gap)
        & (denominator >= np.sin(np.deg2rad(min_angle_deg)) ** 2)
    )
    points = (p1 + p2) / 2
    valid &= np.isfinite(points).all(axis=1)
    points[~valid] = np.nan
    return points, valid, gap


class RefractiveStereo:
    """Cache raw-pixel rays and sample SDK rectification maps per ROI."""

    def __init__(self, housing_path, rectification_path, sdk_hash=None):
        data = yaml.safe_load(Path(housing_path).read_text())
        self.left, self.right = (
            PortCamera(data["left"]),
            PortCamera(data["right"]),
        )
        self.size = tuple(data["image_size"])
        transform = np.asarray(data["T_left_right"], dtype=float)
        self.r, self.t = transform[:3, :3], transform[:3, 3]
        maps = np.load(rectification_path, allow_pickle=False)
        if not bool(maps["validated"]):
            raise ValueError(
                "Rectification has not passed raw/rect image checks"
            )
        if sdk_hash and str(maps["sdk_sha256"]) != sdk_hash:
            raise ValueError(
                "Rectification belongs to a different SDK calibration"
            )
        if tuple(maps["raw_size"]) != self.size:
            raise ValueError("Calibmar/raw rectification resolutions differ")
        self.maps = [maps["left_map"], maps["right_map"]]
        self.output_r = maps["R_left"]
        self.k = maps["K_left_rect"]
        self.fingerprints = [maps["left_info"], maps["right_info"]]
        self.rect_size = self.maps[0].shape[1::-1]
        self.raw_rays = []
        width, height = self.size
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))
        pixels = np.c_[xx.ravel(), yy.ravel()]
        for camera in (self.left, self.right):
            origin, direction = camera.rays(pixels)
            self.raw_rays.append(
                (
                    origin.reshape(height, width, 3).astype(np.float32),
                    direction.reshape(height, width, 3).astype(np.float32),
                )
            )

    @staticmethod
    def sample(array, pixels):
        """Bilinear interpolation with explicit NaN outside the image."""
        p = np.asarray(pixels, np.float32).reshape(-1, 1, 2)
        return cv2.remap(
            array,
            p[..., 0],
            p[..., 1],
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(float("nan"),) * 4,
        ).reshape(len(p), -1)

    def rays(self, pixels, side=0):
        raw = self.sample(self.maps[side], pixels)
        o, d = self.raw_rays[side]
        origin = self.sample(o, raw)
        direction = unit(self.sample(d, raw))
        if side == 1:
            origin = origin @ self.r.T + self.t
            direction = direction @ self.r.T
        return origin, direction

    def reconstruct(self, pixels, disparity, **kwargs):
        right_pixels = np.asarray(pixels).copy().astype(float)
        # Input d is normalized to u_left-u_right by the ROS adapter.
        right_pixels[:, 0] -= disparity
        o1, d1 = self.rays(pixels, 0)
        o2, d2 = self.rays(right_pixels, 1)
        points, valid, gap = triangulate(o1, d1, o2, d2, **kwargs)
        return points @ self.output_r.T, valid, gap

    def left_rays_output(self, pixels):
        origin, direction = self.rays(pixels)
        return origin @ self.output_r.T, direction @ self.output_r.T
