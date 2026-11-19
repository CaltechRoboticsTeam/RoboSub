"""Detection -> selected target -> ROI depth -> one of four solvers."""

import cv2
import numpy as np

from robosub_vision.depth import DepthRegion
from robosub_vision.solvers import CadSolver, cylinder, planar, position


class Pipeline:
    def __init__(self, config, stereo=None):
        self.targets = config["targets"]
        self.options = config["perception"]
        self.stereo = stereo
        self.depth = DepthRegion(self.options, stereo)
        self.cad = {
            t["name"]: CadSolver(t)
            for t in self.targets
            if t["method"] == "cad"
        }
        self.priors = {}

    def process(self, detections, depth, disparity, confidence, k, stamp):
        observations, rejected = [], []
        sigma = self.options["measurement_sigma"]
        for target in self.targets:
            candidates = [
                d
                for d in detections
                if d.class_name == target["detector_class"]
                and d.confidence >= target["min_confidence"]
            ]
            if not candidates:
                continue
            # Initially publish one observation per target class per image.
            detection = max(candidates, key=lambda d: d.confidence)
            try:
                points, pixels = self.depth.points(
                    detection.box, depth, disparity, confidence, k
                )
                if len(points) < self.options["min_points"]:
                    raise ValueError("Insufficient filtered target points")
                method = target["method"]
                if method == "position":
                    estimate = position(points, sigma)
                elif method == "planar":
                    estimate = planar(
                        target,
                        detection.keypoints,
                        points,
                        k,
                        sigma,
                        self.stereo,
                    )
                elif method == "cylinder":
                    estimate = cylinder(target, points, sigma)
                else:
                    prior = self.priors.get(target["name"])
                    if prior and not 0 < stamp - prior[0] < 0.5:
                        prior = None
                    landmarks = None
                    if target.get("landmarks"):
                        if detection.keypoints is None:
                            raise ValueError(
                                "CAD landmarks need a keypoint model"
                            )
                        observed, model = [], []
                        for landmark in target["landmarks"]:
                            index = landmark["index"]
                            if index >= len(detection.keypoints):
                                continue
                            pixel = detection.keypoints[index]
                            near = np.linalg.norm(pixels - pixel, axis=1) < 6
                            if near.sum() >= 3:
                                observed.append(np.median(points[near], axis=0))
                                model.append(landmark["xyz"])
                        landmarks = (np.asarray(model), np.asarray(observed))
                    solver = self.cad[target["name"]]
                    try:
                        estimate = solver.solve(
                            points,
                            sigma,
                            prior[1] if prior else None,
                            landmarks,
                        )
                    except ValueError:
                        if not prior:
                            raise
                        estimate = solver.solve(
                            points, sigma, landmarks=landmarks
                        )
                if not np.isfinite(estimate.position).all():
                    raise ValueError("Nonfinite target estimate")
                if estimate.rotation is not None:
                    self.priors[target["name"]] = (
                        stamp,
                        (estimate.rotation, estimate.position),
                    )
                observations.append((target, estimate, points))
            except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                rejected.append(f"{target['name']}: {error}")
        return observations, rejected
