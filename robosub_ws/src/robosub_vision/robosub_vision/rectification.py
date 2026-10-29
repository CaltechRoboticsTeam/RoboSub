"""Recover and verify SDK image warps from matched raw/rectified tag corners.

Do not substitute cv2.stereoRectify for the SDK's own image transformation.
Fit the actual raw-to-rectified rotation in each camera and check held-out
corners. Lens intrinsics/distortion come from the running wrapper.
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from robosub_vision.geometry import unit


def camera_fingerprint(raw, rect):
    """Persist exactly the SDK K/D and rectified P that define the mapping."""
    return np.r_[
        raw.width, raw.height, rect.width, rect.height, raw.k, raw.d, rect.p
    ]


def fit_rectification(raw_pixels, rect_pixels, k_raw, d_raw, k_rect, size):
    raw_pixels, rect_pixels = np.asarray(raw_pixels), np.asarray(rect_pixels)
    if len(raw_pixels) < 80:
        raise ValueError("Need >=80 raw/rectified corner correspondences")
    hull = cv2.convexHull(rect_pixels.astype(np.float32))
    coverage = cv2.contourArea(hull) / np.prod(size)
    if coverage < 0.25:
        raise ValueError(
            "Move board across >=25% of the image before saving maps"
        )
    raw_dirs = unit(
        np.c_[
            cv2.undistortPoints(raw_pixels[:, None], k_raw, d_raw)[:, 0],
            np.ones(len(raw_pixels)),
        ]
    )
    rect_dirs = unit(
        np.c_[rect_pixels, np.ones(len(rect_pixels))] @ np.linalg.inv(k_rect).T
    )
    # Hold out every third corner; it is never used to fit the rotation.
    # we don't use it because the SDK's rectification is not perfect, so we use our own analysis to check it.
    train = np.arange(len(raw_pixels)) % 3 != 0
    r = Rotation.align_vectors(rect_dirs[train], raw_dirs[train])[0].as_matrix()
    projected = (raw_dirs @ r.T) @ k_rect.T
    projected = projected[:, :2] / projected[:, 2:]
    errors = np.linalg.norm(projected - rect_pixels, axis=1)
    rms = float(np.sqrt(np.mean(errors[~train] ** 2)))
    if rms > 0.5 or np.quantile(errors, 0.99) > 1.5:
        raise ValueError(f"SDK raw/rect mapping fails checks: RMS {rms:.3f}px")
    mx, my = cv2.initUndistortRectifyMap(
        k_raw, d_raw, r, k_rect, tuple(size), cv2.CV_32FC1
    )
    return np.dstack([mx, my]), r, rms
