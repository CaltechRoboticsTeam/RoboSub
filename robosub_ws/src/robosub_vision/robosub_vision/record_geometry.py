"""Record ZED topics to measure SDK rectification."""

import argparse
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robosub_vision.calibration import TagGrid, sha256
from robosub_vision.rectification import (
    camera_fingerprint,
    fit_rectification,
)


class GeometryRecorder(Node):
    def __init__(self, args):
        super().__init__("record_zed_geometry")
        self.args = args
        self.grid = TagGrid(args.board)
        self.bridge = CvBridge()
        self.info = {}
        self.corners = [[[], []], [[], []]]
        self.samples = 0
        self.saved = False
        root = f"/{args.camera_name}/zed_node"
        self.info_subscribers = []
        self.images = []
        for side in ("left", "right"):
            for view in ("raw", "rect"):
                key = f"{side}_{view}"
                topic = f"{root}/{side}/color/{view}"
                self.info_subscribers.append(
                    self.create_subscription(
                        CameraInfo,
                        topic + "/camera_info",
                        lambda msg, key=key: self.info.update({key: msg}),
                        qos_profile_sensor_data,
                    )
                )
                self.images.append(
                    Subscriber(
                        self,
                        Image,
                        topic + "/image",
                        qos_profile=qos_profile_sensor_data,
                    )
                )
        self.sync = TimeSynchronizer(self.images, 5)
        self.sync.registerCallback(self.on_images)
        self.get_logger().info(
            "Move the measured grid across both images; "
            "maps save after enough coverage and checks"
        )

    def on_images(self, *messages):
        if len(self.info) != 4 or self.saved:
            return
        try:
            detections = [
                self.grid.detect(self.bridge.imgmsg_to_cv2(msg, "mono8"))
                for msg in messages
            ]
        except CvBridgeError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)
            return
        matches = []
        for side in range(2):
            raw, rect = detections[side * 2 : side * 2 + 2]
            common = sorted(set(raw) & set(rect))
            if len(common) < 4:
                return
            matches.append(
                (
                    np.concatenate([raw[i] for i in common]),
                    np.concatenate([rect[i] for i in common]),
                )
            )
        for side, (raw, rect) in enumerate(matches):
            self.corners[side][0].append(raw)
            self.corners[side][1].append(rect)
        self.samples += 1
        if self.samples < 12 or self.samples % 6:
            return
        result = {}
        try:
            for index, side in enumerate(("left", "right")):
                raw, rect = self.info[f"{side}_raw"], self.info[f"{side}_rect"]
                if raw.distortion_model not in (
                    "plumb_bob",
                    "rational_polynomial",
                ):
                    raise ValueError(
                        "Only ZED 2i radial/tangential lenses supported"
                    )
                k_raw = np.asarray(raw.k).reshape(3, 3)
                k_rect = np.asarray(rect.p).reshape(3, 4)[:, :3]
                mapping, rotation, rms = fit_rectification(
                    np.concatenate(self.corners[index][0]),
                    np.concatenate(self.corners[index][1]),
                    k_raw,
                    np.asarray(raw.d),
                    k_rect,
                    (rect.width, rect.height),
                )
                result[f"{side}_map"] = mapping
                result[f"R_{side}"] = rotation
                result[f"K_{side}_rect"] = k_rect
                result[f"{side}_info"] = camera_fingerprint(raw, rect)
                result[f"{side}_rms_px"] = rms
            result["raw_size"] = [raw.width, raw.height]
            result["validated"] = True
            result["sdk_sha256"] = sha256(self.args.sdk_calibration)
            output = Path(self.args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("xb") as file:
                np.savez_compressed(file, **result)
            self.saved = True
            self.get_logger().info(f"Saved validated SDK maps: {output}")
        except ValueError as error:
            self.get_logger().info(str(error))
        if self.samples >= 120:
            # Bound storage while preserving a spread of previously seen poses.
            self.corners = [
                [items[::2] for items in side] for side in self.corners
            ]
            self.samples = len(self.corners[0][0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board", required=True)
    parser.add_argument("--sdk-calibration", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--camera-name", default="zed")
    args, ros_args = parser.parse_known_args()
    if not Path(args.sdk_calibration).is_file():
        parser.error("SDK calibration file does not exist")
    if Path(args.output).exists():
        parser.error("Output already exists; choose a new file")
    rclpy.init(args=ros_args)
    node = GeometryRecorder(args)
    try:
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
