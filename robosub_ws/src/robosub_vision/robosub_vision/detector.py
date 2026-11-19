"""Detect targets and landmarks with YOLO."""

import numpy as np

from robosub_vision.types import Detection


class Detector:
    def __init__(self, options, targets):
        from ultralytics import YOLO

        self.options = options
        self.model = YOLO(options["detector_model"])
        names = self.model.names
        requested = {t["detector_class"] for t in targets}
        self.classes = [i for i, name in names.items() if name in requested]
        self.confidence = min(t["min_confidence"] for t in targets)

    def detect(self, bgr):
        result = self.model.predict(
            bgr,
            imgsz=self.options["image_size"],
            conf=self.confidence,
            classes=self.classes,
            device=self.options["device"],
            verbose=False,
            max_det=32,
        )[0]
        detections = []
        for i, box in enumerate(result.boxes):
            keypoints = None
            if result.keypoints is not None:
                keypoints = result.keypoints.xy[i].cpu().numpy().astype(float)
                if result.keypoints.conf is not None:
                    score = result.keypoints.conf[i].cpu().numpy()
                    keypoints[score < 0.5] = np.nan
                keypoints[np.all(keypoints == 0, axis=1)] = np.nan
            detections.append(
                Detection(
                    result.names[int(box.cls.item())],
                    float(box.conf.item()),
                    box.xyxy[0].cpu().numpy(),
                    keypoints,
                )
            )
        return detections
