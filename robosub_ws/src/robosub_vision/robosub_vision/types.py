"""Detection and geometry data types."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    class_name: str
    confidence: float
    box: np.ndarray  # x0, y0, x1, y1, rectified left image pixels.
    keypoints: np.ndarray | None = None  # N x 2; missing points are NaN.


@dataclass
class Estimate:
    position: np.ndarray
    covariance: np.ndarray
    reference_point: str
    rmse: float
    support: int
    rotation: np.ndarray | None = None
    axis: np.ndarray | None = None
    radius: float | None = None
    symmetry: str = "none"
