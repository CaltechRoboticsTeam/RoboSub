"""AprilTag stereo calibration, with fixed factory physical extrinsics."""

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class TagGrid:
    def __init__(self, filename, require_measured=True):
        self.data = yaml.safe_load(Path(filename).read_text())
        data = self.data
        if require_measured and data.get("measured") is not True:
            raise ValueError(
                "Measure board dimensions, then set measured: true"
            )
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, data["dictionary"])
        )
        count = data["columns"] * data["rows"]
        self.ids = np.arange(
            data["first_id"], data["first_id"] + count, dtype=np.int32
        )
        self.board = cv2.aruco.GridBoard(
            (data["columns"], data["rows"]),
            data["marker_size"],
            data["marker_spacing"],
            self.dictionary,
            self.ids,
        )
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.objects = dict(
            zip(self.ids, self.board.getObjPoints(), strict=True)
        )

    def detect(self, image):
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            return {}
        return {
            int(i): np.asarray(c).reshape(4, 2)
            for i, c in zip(ids.ravel(), corners, strict=True)
            if i in self.objects
        }

    def common(self, left, right):
        ids = sorted(set(left) & set(right))
        if len(ids) < 4:
            raise ValueError("Need at least four shared grid markers")
        return (
            np.concatenate([self.objects[i] for i in ids]),
            np.concatenate([left[i] for i in ids]),
            np.concatenate([right[i] for i in ids]),
        )


def write_opencv(path, size, k1, d1, k2, d2, r, t):
    """Write SDK calibration: Rodrigues R in radians, T in millimetres."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise ValueError(f"Cannot write {path}")
    fs.startWriteStruct("Size", cv2.FileNode_SEQ)
    for value in size:
        fs.write("", int(value))
    fs.endWriteStruct()
    for key, value in (
        ("K_LEFT", k1),
        ("K_RIGHT", k2),
        ("D_LEFT", d1.reshape(1, -1)),
        ("D_RIGHT", d2.reshape(1, -1)),
        ("R", cv2.Rodrigues(r)[0]),
        ("T", np.asarray(t).reshape(3, 1) * 1000),
    ):
        fs.write(key, np.asarray(value, dtype=float))
    fs.release()


def read_opencv(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"Cannot read SDK calibration: {path}")
    try:
        size_node = fs.getNode("Size")
        size = tuple(int(size_node.at(i).real()) for i in range(2))
        result = {
            key: fs.getNode(key).mat()
            for key in ("K_LEFT", "K_RIGHT", "D_LEFT", "D_RIGHT", "R", "T")
        }
    finally:
        fs.release()
    if any(v is None or not np.isfinite(v).all() for v in result.values()):
        raise ValueError("SDK calibration lacks finite matrices")
    for side in ("LEFT", "RIGHT"):
        k, d = result[f"K_{side}"], result[f"D_{side}"]
        if (
            k.shape != (3, 3)
            or k[0, 0] <= 0
            or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or d.size not in (4, 5, 8, 12, 14)
        ):
            raise ValueError("Invalid SDK intrinsic/distortion matrix")
    if result["R"].shape != (3, 1) or result["T"].shape != (3, 1):
        raise ValueError("SDK R and T must be 3x1 vectors")
    if result["T"][0, 0] >= 0:
        raise ValueError("SDK OpenCV T.x must be negative millimetres")
    result["size"] = size
    return result


def solve_stereo(observations, factory):
    """Fit intrinsics and board poses; keep camera R and baseline fixed."""
    objects, left, right = map(list, zip(*observations, strict=True))
    size = tuple(factory["image_size"])
    r = np.asarray(factory["physical_R"])
    t = np.asarray(factory["physical_t"])
    initial = []
    poses = None
    for side, pixels in (("left", left), ("right", right)):
        k = np.array(factory[side]["K"], dtype=float)
        d = np.array(factory[side]["D"], dtype=float).ravel()[:5]
        _, k, d, rv, tv = cv2.calibrateCamera(
            [x.astype(np.float32) for x in objects],
            [x.astype(np.float32) for x in pixels],
            size,
            k,
            d,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS,
        )
        initial.extend([k[0, 0], k[1, 1], k[0, 2], k[1, 2], *d.ravel()[:5]])
        if side == "left":
            poses = [
                np.r_[a.ravel(), b.ravel()] for a, b in zip(rv, tv, strict=True)
            ]
    initial = np.r_[initial, np.concatenate(poses)]

    def unpack(x, offset):
        fx, fy, cx, cy = x[offset : offset + 4]
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), (
            x[offset + 4 : offset + 9]
        )

    def residual(x):
        k1, d1 = unpack(x, 0)
        k2, d2 = unpack(x, 9)
        errors = []
        for i, (obj, lp, rp) in enumerate(observations):
            rv, tv = x[18 + 6 * i : 21 + 6 * i], x[21 + 6 * i : 24 + 6 * i]
            board_r = cv2.Rodrigues(rv)[0]
            predicted_l = cv2.projectPoints(obj, rv, tv, k1, d1)[0][:, 0]
            predicted_r = cv2.projectPoints(
                obj, cv2.Rodrigues(r @ board_r)[0], r @ tv + t, k2, d2
            )[0][:, 0]
            errors.extend(
                ((predicted_l - lp).ravel(), (predicted_r - rp).ravel())
            )
        return np.concatenate(errors)

    count = sum(4 * len(x[0]) for x in observations)
    sparse = lil_matrix((count, len(initial)), dtype=int)
    offset = 0
    for i, (obj, _, _) in enumerate(observations):
        rows = 4 * len(obj)
        sparse[offset : offset + rows, :18] = 1
        sparse[offset : offset + rows, 18 + 6 * i : 24 + 6 * i] = 1
        offset += rows
    lower = np.full(len(initial), -np.inf)
    upper = np.full(len(initial), np.inf)
    for offset in (0, 9):
        lower[offset : offset + 2] = 0.1 * min(size)
        upper[offset : offset + 2] = 10 * max(size)
        lower[offset + 2 : offset + 4] = 0
        upper[offset + 2 : offset + 4] = size
    fit = least_squares(
        residual,
        initial,
        jac_sparsity=sparse.tocsr(),
        bounds=(lower, upper),
        x_scale="jac",
        loss="soft_l1",
        f_scale=1,
        max_nfev=300,
    )
    if not fit.success or not np.isfinite(fit.fun).all():
        raise ValueError(f"Effective stereo optimization failed: {fit.message}")
    k1, d1 = unpack(fit.x, 0)
    k2, d2 = unpack(fit.x, 9)
    per_view = []
    offset = 0
    for obj, _, _ in observations:
        count = len(obj) * 4
        errors = fit.fun[offset : offset + count].reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))))
        offset += count
    return k1, d1, k2, d2, r, t, per_view


def capture(args):
    import pyzed.sl as sl

    grid = TagGrid(args.board)
    output = Path(args.session)
    if (output / "factory.json").exists():
        raise ValueError("Use a new session directory to preserve its metadata")
    (output / "left").mkdir(parents=True, exist_ok=True)
    (output / "right").mkdir(exist_ok=True)
    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.camera_disable_self_calib = True
    if args.serial:
        init.set_from_serial_number(args.serial)
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise ValueError(f"Cannot open ZED for raw capture: {status}")
    try:
        info = camera.get_camera_information()
        calibration = info.camera_configuration.calibration_parameters_raw

        def parameters(cam):
            return {
                "K": [[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]],
                "D": list(map(float, cam.disto[:8])),
            }

        # Match Stereolabs StereoCalib::setFrom, in IMAGE coordinates/metres.
        physical_t = np.asarray(
            calibration.stereo_transform.get_translation().get()
        ).copy()
        physical_t[0] *= -1
        physical_r = np.asarray(
            calibration.stereo_transform.get_rotation_matrix().r
        ).reshape(3, 3)
        metadata = {
            "serial": info.serial_number,
            "sdk_version": sl.Camera.get_sdk_version(),
            "image_size": [
                info.camera_configuration.resolution.width,
                info.camera_configuration.resolution.height,
            ],
            "resolution": args.resolution,
            "left": parameters(calibration.left_cam),
            "right": parameters(calibration.right_cam),
            "physical_R": physical_r.tolist(),
            "physical_t": physical_t.tolist(),
            "imu_transform": np.asarray(
                info.sensors_configuration.camera_imu_transform.m
            ).tolist(),
            "board": grid.data,
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }
        (output / "factory.json").write_text(json.dumps(metadata, indent=2))
        (output / "board.yaml").write_text(yaml.safe_dump(grid.data))
        left, right = sl.Mat(), sl.Mat()
        coverage = np.zeros((4, 6), dtype=bool)
        samples = 0
        with (output / "frames.jsonl").open("x") as log:
            while True:
                if camera.grab() != sl.ERROR_CODE.SUCCESS:
                    continue
                camera.retrieve_image(left, sl.VIEW.LEFT_UNRECTIFIED)
                camera.retrieve_image(right, sl.VIEW.RIGHT_UNRECTIFIED)
                images = [
                    left.get_data()[..., :3].copy(),
                    right.get_data()[..., :3].copy(),
                ]
                detections = [grid.detect(image) for image in images]
                preview = []
                for image, tags in zip(images, detections, strict=True):
                    shown = image.copy()
                    if tags:
                        cv2.aruco.drawDetectedMarkers(
                            shown,
                            [c[None].astype(np.float32) for c in tags.values()],
                            np.array(list(tags), np.int32)[:, None],
                        )
                    preview.append(shown)
                shown = np.hstack(preview)
                text = (
                    f"s: save | q: quit | {samples} pairs | "
                    f"coverage {coverage.mean():.0%} | "
                    f"tags {len(detections[0])}/{len(detections[1])}"
                )
                cv2.putText(
                    shown,
                    text,
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow(
                    "Underwater raw stereo",
                    cv2.resize(
                        shown,
                        (1280, round(1280 * shown.shape[0] / shown.shape[1])),
                    ),
                )
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key != ord("s"):
                    continue
                try:
                    _, lp, _ = grid.common(*detections)
                except ValueError as error:
                    print(error)
                    continue
                name = f"{samples:05}.png"
                for side, image in zip(("left", "right"), images, strict=True):
                    if not cv2.imwrite(str(output / side / name), image):
                        raise ValueError("Failed to save calibration image")
                cells = (lp / np.array(metadata["image_size"]) * [6, 4]).astype(
                    int
                )
                cells = np.clip(cells, [0, 0], [5, 3])
                coverage[cells[:, 1], cells[:, 0]] = True
                log.write(
                    json.dumps(
                        {
                            "file": name,
                            "timestamp_ns": camera.get_timestamp(
                                sl.TIME_REFERENCE.IMAGE
                            ).get_nanoseconds(),
                        }
                    )
                    + "\n"
                )
                log.flush()
                samples += 1
    finally:
        camera.close()
        cv2.destroyAllWindows()


def solve(args):
    session = Path(args.session)
    factory = json.loads((session / "factory.json").read_text())
    grid = TagGrid(session / "board.yaml")
    observations, files = [], []
    for left_file in sorted((session / "left").glob("*.png")):
        right_file = session / "right" / left_file.name
        images = [
            cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
            for p in (left_file, right_file)
        ]
        if any(image is None for image in images):
            raise ValueError(f"Missing/unreadable pair: {left_file.name}")
        if any(
            image.shape[::-1] != tuple(factory["image_size"])
            for image in images
        ):
            raise ValueError(
                "Image resolution differs from saved factory calibration"
            )
        try:
            observations.append(grid.common(*(grid.detect(x) for x in images)))
            files.append(left_file.name)
        except ValueError:
            print(f"Skipping {left_file.name}: insufficient shared tags")
    if len(observations) < 12:
        raise ValueError(
            "Collect at least 12 varied stereo pairs; 25-40 preferred"
        )
    k1, d1, k2, d2, r, t, errors = solve_stereo(observations, factory)
    output = Path(args.output or session / "underwater_opencv.yml")
    if output.exists():
        raise ValueError(f"Refusing to overwrite {output}; choose --output")
    if max(errors) > args.max_error:
        raise ValueError(
            f"View RMS {max(errors):.2f}px exceeds --max-error; "
            "inspect board, coverage and corner detections"
        )
    write_opencv(output, factory["image_size"], k1, d1, k2, d2, r, t)
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        k1, d1, k2, d2, tuple(factory["image_size"]), r, t
    )
    # These are OpenCV QA rectification matrices, NOT the SDK runtime maps.
    np.savez(
        output.with_suffix(".opencv_rectification.npz"),
        R1=r1,
        R2=r2,
        P1=p1,
        P2=p2,
        Q=q,
    )
    epipolar = []
    for _, lp, rp in observations:
        a = cv2.undistortPoints(lp[:, None], k1, d1, R=r1, P=p1)[:, 0]
        b = cv2.undistortPoints(rp[:, None], k2, d2, R=r2, P=p2)[:, 0]
        epipolar.extend(a[:, 1] - b[:, 1])
    report = {
        "schema": 1,
        "factory_sha256": sha256(session / "factory.json"),
        "baseline_m": float(np.linalg.norm(t)),
        "physical_extrinsics": "fixed to factory snapshot",
        "per_view_rms_px": dict(zip(files, errors, strict=True)),
        "epipolar_rms_px": float(np.sqrt(np.mean(np.square(epipolar)))),
        "sdk_validated": False,
        "warning": "Effective pinhole model; underwater accuracy unmeasured",
    }
    output.with_suffix(".report.json").write_text(json.dumps(report, indent=2))
    print(f"Wrote {output}; worst view RMS {max(errors):.3f}px")


def check(args):
    data = read_opencv(args.file)
    print(
        f"Schema OK; size {data['size']}; baseline "
        f"{np.linalg.norm(data['T']) / 1000:.6f} m"
    )
    if not args.sdk:
        print("SDK loading NOT checked. Use check --sdk on the Jetson.")
        return
    import pyzed.sl as sl

    camera = sl.Camera()
    init = sl.InitParameters()
    init.optional_opencv_calibration_file = str(Path(args.file).resolve())
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_disable_self_calib = True
    if args.serial:
        init.set_from_serial_number(args.serial)
    try:
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise ValueError(
                f"SDK rejected calibration or camera open: {status}"
            )
        info = camera.get_camera_information().camera_configuration
        actual = info.calibration_parameters_raw
        sx = info.resolution.width / data["size"][0]
        sy = info.resolution.height / data["size"][1]
        for side, cam in (
            ("LEFT", actual.left_cam),
            ("RIGHT", actual.right_cam),
        ):
            expected = data[f"K_{side}"]
            if not np.allclose(
                [cam.fx, cam.fy, cam.cx, cam.cy],
                [
                    expected[0, 0] * sx,
                    expected[1, 1] * sy,
                    expected[0, 2] * sx,
                    expected[1, 2] * sy,
                ],
                rtol=0.002,
                atol=0.2,
            ):
                raise ValueError(
                    "SDK opened but did not apply expected intrinsics"
                )
            distortion = data[f"D_{side}"].ravel()
            if not np.allclose(
                np.asarray(cam.disto)[: len(distortion)],
                distortion,
                atol=1e-5,
                rtol=0.002,
            ):
                raise ValueError("SDK distortion differs from exported file")
        if not np.isclose(
            actual.get_camera_baseline(),
            np.linalg.norm(data["T"]) / 1000,
            rtol=0.002,
        ):
            raise ValueError(
                "SDK baseline does not match the exported calibration"
            )
        rotation = np.asarray(
            actual.stereo_transform.get_rotation_matrix().r
        ).reshape(3, 3)
        if not np.allclose(rotation, cv2.Rodrigues(data["R"])[0], atol=1e-4):
            raise ValueError("SDK stereo rotation differs from exported file")
        print("SDK opened camera and applied effective intrinsics/baseline")
    finally:
        camera.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    cap = sub.add_parser("capture", help="Stop the wrapper before raw capture")
    cap.add_argument("--board", required=True)
    cap.add_argument("--session", required=True)
    cap.add_argument("--resolution", default="HD720")
    cap.add_argument("--serial", type=int, default=0)
    sol = sub.add_parser("solve")
    sol.add_argument("--session", required=True)
    sol.add_argument("--output")
    sol.add_argument("--max-error", type=float, default=2.0)
    chk = sub.add_parser("check")
    chk.add_argument("--file", required=True)
    chk.add_argument("--sdk", action="store_true")
    chk.add_argument("--resolution", default="HD720")
    chk.add_argument("--serial", type=int, default=0)
    args = parser.parse_args()
    try:
        {"capture": capture, "solve": solve, "check": check}[args.command](args)
    except (ValueError, OSError, ImportError, cv2.error) as error:
        parser.exit(2, f"Calibration: {error}\n")
