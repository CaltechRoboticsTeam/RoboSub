"""Bounded ROI reconstruction and foreground selection. THIS IS FROM A PAPER!"""

import numpy as np


def roi_pixels(box, shape, stride, max_points):
    height, width = shape
    box = np.asarray(box, dtype=float)
    if box.shape != (4,) or not np.isfinite(box).all():
        raise ValueError("Detection box must contain four finite coordinates")
    x0, y0 = np.maximum(np.floor(box[:2]).astype(int), [0, 0])
    x1, y1 = np.minimum(np.ceil(box[2:]).astype(int), [width, height])
    if x1 <= x0 or y1 <= y0:
        return np.empty((0, 2), dtype=int)
    step = max(
        stride, int(np.ceil(np.sqrt((x1 - x0) * (y1 - y0) / max_points)))
    )
    xx, yy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.c_[xx.ravel(), yy.ravel()][:max_points]


def zed_points(pixels, depth, k):
    u, v = pixels.T
    z = depth[v, u]
    return np.c_[(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z]


def foreground(points, pixels, box, spread, min_points):
    """Choose a supported near depth layer seeded in the box's central area.

    The result is a visible surface, not an inferred hidden object center.
    CAD/keypoints give a stronger geometric reference for their solvers.
    """
    valid = np.isfinite(points).all(axis=1)
    p, uv = points[valid], pixels[valid]
    if len(p) < min_points:
        raise ValueError("Too few reliable stereo points in target region")
    box = np.asarray(box)
    center = (box[:2] + box[2:]) / 2
    half = np.maximum((box[2:] - box[:2]) * 0.25, 1)
    central = np.all(np.abs(uv - center) <= half, axis=1)
    seeds = p[central, 2]
    if len(seeds) < min_points // 3:
        raise ValueError("No supported foreground in detection center")
    edges = np.arange(
        p[:, 2].min(), p[:, 2].max() + spread * 1.01, max(spread / 2, 1e-3)
    )
    if len(edges) < 2:
        edges = [p[:, 2].min(), p[:, 2].min() + spread]
    count, edges = np.histogram(seeds, edges)
    supported = np.flatnonzero(count >= max(3, count.max() * 0.3))
    if not len(supported):
        raise ValueError("No stable depth layer in target region")
    z = (edges[supported[0]] + edges[supported[0] + 1]) / 2
    layer = np.abs(p[:, 2] - z) <= spread
    p, uv = p[layer], uv[layer]
    if len(p) < min_points:
        raise ValueError("Foreground layer has insufficient stereo support")
    median = np.median(p, axis=0)
    deviation = np.linalg.norm(p - median, axis=1)
    limit = np.median(deviation) + 4 * max(
        np.median(np.abs(deviation - np.median(deviation))), 0.005
    )
    keep = deviation <= limit
    return p[keep], uv[keep]


class DepthRegion:
    def __init__(self, options, stereo=None):
        self.options = options
        self.stereo = stereo

    def points(self, box, depth, disparity, confidence, k):
        p = self.options
        pixels = roi_pixels(box, depth.shape, p["roi_stride"], p["max_points"])
        if not len(pixels):
            raise ValueError("Detection does not intersect the image")
        u, v = pixels.T
        good = np.isfinite(confidence[v, u]) & (
            confidence[v, u] <= p["max_confidence"]
        )
        if self.stereo:
            d = disparity[v, u]
            good &= np.isfinite(d) & (d > 0)
            points, ray_valid, _ = self.stereo.reconstruct(
                pixels,
                d,
                max_gap=p["max_ray_gap"],
                min_angle_deg=p["min_ray_angle_deg"],
            )
            good &= ray_valid
        else:
            points = zed_points(pixels, depth, k)
        good &= (
            np.isfinite(points).all(axis=1)
            & (points[:, 2] >= p["min_depth"])
            & (points[:, 2] <= p["max_depth"])
        )
        points[~good] = np.nan
        return foreground(
            points, pixels, box, p["max_depth_spread"], p["min_points"]
        )
