#!/usr/bin/env python3
"""Train/validate YOLOv11 or export the trained detector on its target Jetson."""

import argparse
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("train", "validate", "export"))
    parser.add_argument(
        "--model",
        required=True,
        help="Local nano detection/pose .pt, .yaml or .engine",
    )
    parser.add_argument("--data")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--device", default="0")
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--precision", choices=("fp32", "fp16"), default="fp16")
    args = parser.parse_args()
    from ultralytics import YOLO

    if not Path(args.model).is_file():
        parser.error("Supply a local model/architecture; no implicit downloads")
    model = YOLO(args.model)
    if args.command == "export":
        import tensorrt as trt

        # TensorRT 11 changes precision selection to Ultralytics 'quantize'.
        precision = (
            {"quantize": args.precision}
            if int(trt.__version__.split(".")[0]) >= 11
            else {"half": args.precision == "fp16"}
        )
        model.export(
            format="engine",
            device=args.device,
            batch=1,
            imgsz=args.image_size,
            workspace=1,
            dynamic=False,
            **precision,
        )
    else:
        if not args.data or not Path(args.data).is_file():
            parser.error("Supply --data dataset.yaml")
        data = yaml.safe_load(Path(args.data).read_text())
        if "kpt_shape" in data and model.task != "pose":
            parser.error(
                "Dataset has keypoints; use a nano pose model, or generate "
                "a scene without landmarks for a detection model"
            )
        if args.command == "train":
            model.train(
                data=args.data,
                epochs=args.epochs,
                device=args.device,
                batch=args.batch,
                imgsz=args.image_size,
                seed=135,
                fliplr=0,
                flipud=0,
                workers=2,
            )
        else:
            model.val(
                data=args.data,
                device=args.device,
                imgsz=args.image_size,
                batch=args.batch,
            )


if __name__ == "__main__":
    main()
