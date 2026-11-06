"""Render pinhole images and export YOLO labels and target poses."""

import json
from pathlib import Path

import cv2
import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation

from robosub_vision.geometry import transform_points


def load_scene(filename):
    path = Path(filename).resolve()
    data = yaml.safe_load(path.read_text())
    data["backgrounds"] = [
        str((path.parent / p).resolve()) for p in data.get("backgrounds", [])
    ]
    for target in data["targets"]:
        mesh_path = path.parent / target["mesh"]
        target["geometry"] = trimesh.load(mesh_path, force="mesh", process=True)
        scale = {"meters": 1, "millimeters": 0.001}[target["units"]]
        target["geometry"].apply_scale(scale)
        if target["geometry"].is_empty:
            raise ValueError(f"Empty mesh: {mesh_path}")
        target["landmarks"] = (
            np.asarray(target.get("landmarks", []), dtype=float).reshape(-1, 3)
            * scale
        )
    return data


def random_pose(rng, distance):
    # Full random rotation, metric camera distance and modest off-axis location.
    r = Rotation.random(random_state=rng).as_matrix()
    z = rng.uniform(*distance)
    return r, np.array(
        [rng.uniform(-0.2, 0.2) * z, rng.uniform(-0.15, 0.15) * z, z]
    )


def render_mesh(mesh, r, t, k, depth, labels, label):
    """Rasterize triangles using metric, perspective-correct depth."""
    vertices = transform_points(np.asarray(mesh.vertices), r, t)
    projected = vertices @ k.T
    projected = projected[:, :2] / projected[:, 2:]
    height, width = depth.shape
    shading = np.zeros(depth.shape, dtype=float)
    for face in np.asarray(mesh.faces):
        camera_triangle = vertices[face]
        if np.any(camera_triangle[:, 2] <= 0.01):
            continue
        triangle = projected[face]
        x0, y0 = np.maximum(np.floor(triangle.min(0)).astype(int), [0, 0])
        x1, y1 = np.minimum(
            np.ceil(triangle.max(0)).astype(int), [width - 1, height - 1]
        )
        if x0 > x1 or y0 > y1:
            continue
        a, b, c = triangle
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (
            a[1] - c[1]
        )
        if abs(denominator) < 1e-8:
            continue
        xx, yy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        w0 = (
            (b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])
        ) / denominator
        w1 = (
            (c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])
        ) / denominator
        w2 = 1 - w0 - w1
        inverse = (
            w0 / camera_triangle[0, 2]
            + w1 / camera_triangle[1, 2]
            + (w2 / camera_triangle[2, 2])
        )
        z = 1 / np.maximum(inverse, 1e-12)
        view = depth[y0 : y1 + 1, x0 : x1 + 1]
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0) & (z < view)
        view[inside] = z[inside]
        labels[y0 : y1 + 1, x0 : x1 + 1][inside] = label
        normal = np.cross(
            camera_triangle[1] - camera_triangle[0],
            camera_triangle[2] - camera_triangle[0],
        )
        normal /= max(np.linalg.norm(normal), 1e-12)
        shading[y0 : y1 + 1, x0 : x1 + 1][inside] = 0.25 + 0.75 * abs(
            normal @ np.array([0.3, -0.4, -0.866])
        )
    return shading


def underwater_appearance(bgr, depth, rng):
    z = np.where(np.isfinite(depth), depth, 4)[..., None]
    # BGR channel attenuation; this changes appearance without changing pixels.
    transmission = np.exp(-z * rng.uniform([0.03, 0.05, 0.2], [0.3, 0.4, 1.0]))
    water = rng.uniform([50, 35, 5], [140, 110, 45])
    image = bgr * transmission + water * (1 - transmission)
    image += rng.normal(0, rng.uniform(0, 5), image.shape)
    image = np.clip(image, 0, 255).astype(np.uint8)
    if rng.random() < 0.3:
        image = cv2.GaussianBlur(image, (3, 3), rng.uniform(0.2, 1.2))
    return image


class DatasetWriter:
    def __init__(self, output, scene, count, seed, renderer):
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.scene = scene
        self.count = count
        width, height = scene.get("image_size", [640, 480])
        focal = scene.get("focal_px", 550)
        self.k = np.array(
            [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]]
        )
        self.size = width, height
        self.keypoint_count = max(len(t["landmarks"]) for t in scene["targets"])
        for split in ("train", "val"):
            for kind in ("images", "labels", "poses"):
                (self.output / kind / split).mkdir(parents=True)
        dataset = {
            "path": str(self.output),
            "train": "images/train",
            "val": "images/val",
            "names": {i: t["name"] for i, t in enumerate(scene["targets"])},
        }
        if self.keypoint_count:
            dataset["kpt_shape"] = [self.keypoint_count, 3]
        (self.output / "dataset.yaml").write_text(yaml.safe_dump(dataset))
        metadata = {
            "renderer": renderer,
            "optics": "pinhole_no_refraction",
            "units": "meters",
            "K": self.k.tolist(),
            "seed": seed,
            "frames": count,
            "warning": "Appearance baseline; add real underwater training data",
        }
        (self.output / "metadata.json").write_text(
            json.dumps(metadata, indent=2)
        )

    def write(self, index, class_id, image, depth, box, r, t):
        # Keep every target in both splits, including when class count is five.
        split = (
            "val" if (index // len(self.scene["targets"])) % 5 == 0 else "train"
        )
        name = f"{index:06}"
        if not cv2.imwrite(
            str(self.output / "images" / split / f"{name}.png"), image
        ):
            raise ValueError("Could not write dataset image")
        labels, keypoints = [], []
        if box is not None:
            width, height = self.size
            x0, y0, x1, y1 = box
            labels = [
                class_id,
                (x0 + x1) / (2 * width),
                (y0 + y1) / (2 * height),
                (x1 - x0) / width,
                (y1 - y0) / height,
            ]
            landmarks = self.scene["targets"][class_id]["landmarks"]
            points = transform_points(landmarks, r, t)
            projected = points @ self.k.T
            for point, pixel in zip(points, projected, strict=True):
                uv = pixel[:2] / pixel[2]
                visible = False
                if point[2] > 0 and 0 <= uv[0] < width and 0 <= uv[1] < height:
                    x, y = np.clip(
                        np.rint(uv).astype(int), [0, 0], [width - 1, height - 1]
                    )
                    visible = abs(depth[y, x] - point[2]) < 0.012
                    value = [
                        float(uv[0] / width),
                        float(uv[1] / height),
                        2 if visible else 1,
                    ]
                else:
                    value = [0, 0, 0]
                keypoints.append(value)
            keypoints += [[0, 0, 0]] * (self.keypoint_count - len(keypoints))
            labels += np.asarray(keypoints).ravel().tolist()
        (self.output / "labels" / split / f"{name}.txt").write_text(
            " ".join(map(str, labels)) + ("\n" if labels else "")
        )
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        pose = {
            "class_id": class_id,
            "T_camera_object": transform.tolist(),
            "keypoints": keypoints,
            "bbox_xyxy": box,
            "frame": "camera_optical",
            "optics": "pinhole_no_refraction",
        }
        (self.output / "poses" / split / f"{name}.json").write_text(
            json.dumps(pose)
        )


def generate_offline(scene, output, frames, seed):
    rng = np.random.default_rng(seed)
    writer = DatasetWriter(output, scene, frames, seed, "software_mesh_zbuffer")
    width, height = writer.size
    backgrounds = [Path(p) for p in scene.get("backgrounds", [])]
    for index in range(frames):
        class_id = index % len(scene["targets"])
        target = scene["targets"][class_id]
        r, t = random_pose(rng, scene.get("distance", [0.5, 2.0]))
        depth = np.full((height, width), np.inf)
        labels = np.full((height, width), -1, dtype=int)
        shade = render_mesh(
            target["geometry"], r, t, writer.k, depth, labels, 1
        )
        if backgrounds:
            image = cv2.imread(str(backgrounds[index % len(backgrounds)]))
            if image is None:
                raise ValueError("Unreadable synthetic background")
            image = cv2.resize(image, writer.size).astype(float)
        else:
            image = rng.uniform(20, 170, (height, width, 3))
            image = cv2.GaussianBlur(image, (31, 31), 10)
        color = rng.uniform(60, 255, 3)
        image[labels == 1] = shade[labels == 1, None] * color
        if rng.random() < 0.35:
            occluder = trimesh.creation.box([0.08, 0.2, 0.05])
            front = t + rng.uniform([-0.1, -0.05, -0.2], [0.1, 0.05, -0.1])
            render_mesh(occluder, np.eye(3), front, writer.k, depth, labels, 2)
            image[labels == 2] = rng.uniform(20, 150, 3)
        image = underwater_appearance(image, depth, rng)
        y, x = np.where(labels == 1)
        box = (
            [int(x.min()), int(y.min()), int(x.max() + 1), int(y.max() + 1)]
            if len(x) >= 20
            else None
        )
        writer.write(index, class_id, image, depth, box, r, t)
